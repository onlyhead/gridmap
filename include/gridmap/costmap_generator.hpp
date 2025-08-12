#pragma once

#include "concord/concord.hpp"
#include "obstacle_inflation.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <cstdint>

namespace gridmap {

class CostmapGenerator {
public:
    // Convert O_MAP occupancy values to C_MAP cost values
    // O_MAP: 0=occupied/black, 255=free/white, 128=unknown/gray (PGM standard)
    // C_MAP: 0=good_travel/white, 255=impossible/black (Navigation standard)
    static uint8_t occupancyToCost(uint8_t occupancy_value) {
        if (occupancy_value == 255) {
            // PGM: 255 = free/white -> C_MAP: 0 = good_travel/white
            return 0;
        } else if (occupancy_value == 0) {
            // PGM: 0 = occupied/black -> C_MAP: 255 = impossible/black  
            return 255;
        } else {
            // PGM: 128 = unknown/gray -> C_MAP: medium cost (conservative)
            // Map unknown areas to medium-high cost for safety
            return 150;
        }
    }
    
    // Convert entire O_MAP grid to C_MAP with inflation
    static concord::Grid<uint8_t> generateCostmapFromOccupancy(
        const concord::Grid<uint8_t>& occupancy_grid,
        double robot_radius,
        double map_resolution) {
        
        // Create cost grid with same dimensions and properties
        auto cost_grid = occupancy_grid; // Copy structure
        
        // Convert occupancy values to cost values
        for (size_t r = 0; r < cost_grid.rows(); ++r) {
            for (size_t c = 0; c < cost_grid.cols(); ++c) {
                uint8_t occ_val = occupancy_grid(r, c);
                uint8_t cost_val = occupancyToCost(occ_val);
                cost_grid.set_value(r, c, cost_val);
            }
        }
        
        // Apply obstacle inflation for safety gradients
        ObstacleInflation::inflateObstacles(cost_grid, robot_radius, map_resolution);
        
        return cost_grid;
    }
    
    // Combine multiple C_MAP layers into single costmap using various strategies
    enum class CombinationStrategy {
        MAX_COST,           // Take maximum cost (most conservative)
        WEIGHTED_AVERAGE,   // Weighted average of all layers
        MIN_COST,           // Take minimum cost (most optimistic)
        MEDIAN_COST         // Take median cost (balanced approach)
    };
    
    static concord::Grid<uint8_t> combineLayers(
        const std::vector<concord::Grid<uint8_t>>& layers,
        const std::vector<double>& weights = {},
        CombinationStrategy strategy = CombinationStrategy::MAX_COST) {
        
        if (layers.empty()) {
            throw std::runtime_error("No layers to combine");
        }
        
        auto result = layers[0]; // Start with first layer structure
        
        // Single layer case
        if (layers.size() == 1) {
            return result;
        }
        
        // Validate all layers have same dimensions
        for (size_t i = 1; i < layers.size(); ++i) {
            if (layers[i].rows() != result.rows() || 
                layers[i].cols() != result.cols()) {
                throw std::runtime_error("All layers must have same dimensions");
            }
        }
        
        // Process each cell
        for (size_t r = 0; r < result.rows(); ++r) {
            for (size_t c = 0; c < result.cols(); ++c) {
                std::vector<uint8_t> values;
                values.reserve(layers.size());
                
                // Collect values from all layers
                for (size_t i = 0; i < layers.size(); ++i) {
                    uint8_t layer_val = layers[i](r, c);
                    
                    // Apply weight if provided
                    if (i < weights.size() && weights[i] != 1.0) {
                        layer_val = static_cast<uint8_t>(
                            std::clamp(layer_val * weights[i], 0.0, 255.0));
                    }
                    
                    values.push_back(layer_val);
                }
                
                // Combine values based on strategy
                uint8_t combined_value = combineValues(values, strategy);
                result.set_value(r, c, combined_value);
            }
        }
        
        return result;
    }
    
    // Add traversability constraints based on elevation and slope
    static void applyTraversabilityConstraints(
        concord::Grid<uint8_t>& costmap,
        const concord::Grid<float>& elevation,
        double max_slope_degrees) {
        
        if (costmap.rows() != elevation.rows() || 
            costmap.cols() != elevation.cols()) {
            throw std::runtime_error("Costmap and elevation grids must have same dimensions");
        }
        
        double max_slope_radians = max_slope_degrees * M_PI / 180.0;
        double resolution = costmap.inradius();
        
        for (size_t r = 1; r < costmap.rows() - 1; ++r) {
            for (size_t c = 1; c < costmap.cols() - 1; ++c) {
                // Calculate local slope using central differences
                float center = elevation(r, c);
                float dx = (elevation(r, c+1) - elevation(r, c-1)) / (2.0f * static_cast<float>(resolution));
                float dy = (elevation(r+1, c) - elevation(r-1, c)) / (2.0f * static_cast<float>(resolution));
                
                float slope_magnitude = std::sqrt(dx*dx + dy*dy);
                float slope_radians = std::atan(slope_magnitude);
                
                // If slope exceeds maximum, add cost penalty
                if (slope_radians > max_slope_radians) {
                    // Scale cost based on how much slope exceeds limit
                    float excess_ratio = (slope_radians - max_slope_radians) / max_slope_radians;
                    uint8_t slope_penalty = static_cast<uint8_t>(
                        std::min(100.0f, excess_ratio * 100.0f));
                    
                    uint8_t current = costmap(r, c);
                    uint8_t new_cost = static_cast<uint8_t>(
                        std::min(255, static_cast<int>(current) + slope_penalty));
                    costmap.set_value(r, c, new_cost);
                }
            }
        }
    }
    
    // Add terrain-specific costs (mud, rough terrain, etc.)
    static void applyTerrainCosts(
        concord::Grid<uint8_t>& costmap,
        const concord::Grid<uint8_t>& terrain_map,
        const std::vector<uint8_t>& terrain_costs) {
        
        if (costmap.rows() != terrain_map.rows() || 
            costmap.cols() != terrain_map.cols()) {
            throw std::runtime_error("Costmap and terrain map must have same dimensions");
        }
        
        for (size_t r = 0; r < costmap.rows(); ++r) {
            for (size_t c = 0; c < costmap.cols(); ++c) {
                uint8_t terrain_type = terrain_map(r, c);
                
                // Apply terrain cost if valid terrain type
                if (terrain_type < terrain_costs.size()) {
                    uint8_t terrain_cost = terrain_costs[terrain_type];
                    uint8_t current = costmap(r, c);
                    uint8_t new_cost = static_cast<uint8_t>(
                        std::min(255, static_cast<int>(current) + terrain_cost));
                    costmap.set_value(r, c, new_cost);
                }
            }
        }
    }
    
    // Generate costmap with dynamic obstacles (temporary high costs)
    static void addDynamicObstacles(
        concord::Grid<uint8_t>& costmap,
        const std::vector<concord::Point>& obstacle_positions,
        double obstacle_radius,
        uint8_t obstacle_cost = 200) {
        
        double resolution = costmap.inradius();
        int radius_cells = static_cast<int>(std::ceil(obstacle_radius / resolution));
        
        for (const auto& pos : obstacle_positions) {
            // Convert world coordinates to grid coordinates
            auto [center_r, center_c] = costmap.world_to_grid(pos);
            // world_to_grid always returns valid clamped indices
            
            // Apply cost in circular area around obstacle
            for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
                for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
                    int r = center_r + dr;
                    int c = center_c + dc;
                    
                    // Check bounds
                    if (r < 0 || r >= static_cast<int>(costmap.rows()) ||
                        c < 0 || c >= static_cast<int>(costmap.cols())) continue;
                    
                    // Check if within circular radius
                    double distance = std::sqrt(dr*dr + dc*dc) * resolution;
                    if (distance <= obstacle_radius) {
                        uint8_t current = costmap(static_cast<size_t>(r), static_cast<size_t>(c));
                        costmap.set_value(static_cast<size_t>(r), static_cast<size_t>(c), 
                                        std::max(current, obstacle_cost));
                    }
                }
            }
        }
    }
    
private:
    // Combine multiple cost values using specified strategy
    static uint8_t combineValues(const std::vector<uint8_t>& values, 
                                CombinationStrategy strategy) {
        if (values.empty()) return 0;
        if (values.size() == 1) return values[0];
        
        switch (strategy) {
            case CombinationStrategy::MAX_COST: {
                return *std::max_element(values.begin(), values.end());
            }
            
            case CombinationStrategy::MIN_COST: {
                return *std::min_element(values.begin(), values.end());
            }
            
            case CombinationStrategy::WEIGHTED_AVERAGE: {
                double sum = 0.0;
                for (uint8_t val : values) {
                    sum += val;
                }
                return static_cast<uint8_t>(sum / values.size());
            }
            
            case CombinationStrategy::MEDIAN_COST: {
                auto sorted_values = values;
                std::sort(sorted_values.begin(), sorted_values.end());
                size_t mid = sorted_values.size() / 2;
                if (sorted_values.size() % 2 == 0) {
                    return static_cast<uint8_t>((sorted_values[mid-1] + sorted_values[mid]) / 2);
                } else {
                    return sorted_values[mid];
                }
            }
            
            default:
                return values[0];
        }
    }
};

} // namespace gridmap
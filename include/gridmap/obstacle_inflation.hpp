#pragma once

#include "concord/concord.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <limits>
#include <cstdint>

namespace gridmap {

class ObstacleInflation {
public:
    // Inflate obstacles with distance-based gradient for safety margins
    static void inflateObstacles(concord::Grid<uint8_t>& grid, 
                                 double robot_radius,
                                 double map_resolution) {
        // Calculate inflation radius in cells with resolution-based safety factor
        // Coarser resolution = more inflation needed for safety
        double safety_factor = std::max(1.0, map_resolution * 10.0); // 10cm res = 1.0, 1m res = 10x
        double inflation_radius = robot_radius * safety_factor;
        int inflation_cells = static_cast<int>(std::ceil(inflation_radius / map_resolution));
        
        // Ensure minimum inflation radius
        inflation_cells = std::max(inflation_cells, 1);
        
        // Create a copy for reading original values during processing
        auto original = grid;
        
        // Process each cell in the grid
        for (size_t r = 0; r < grid.rows(); ++r) {
            for (size_t c = 0; c < grid.cols(); ++c) {
                // Skip if already marked as obstacle
                if (original(r, c) == 255) continue;
                
                // Find distance to nearest obstacle
                double min_distance = std::numeric_limits<double>::max();
                
                // Check neighborhood within inflation radius
                for (int dr = -inflation_cells; dr <= inflation_cells; ++dr) {
                    for (int dc = -inflation_cells; dc <= inflation_cells; ++dc) {
                        int nr = static_cast<int>(r) + dr;
                        int nc = static_cast<int>(c) + dc;
                        
                        // Check bounds
                        if (nr < 0 || nr >= static_cast<int>(grid.rows()) || 
                            nc < 0 || nc >= static_cast<int>(grid.cols())) continue;
                        
                        // If this neighbor is an obstacle (PGM standard: 255 = occupied)
                        if (original(static_cast<size_t>(nr), static_cast<size_t>(nc)) == 255) {
                            double distance = std::sqrt(dr*dr + dc*dc) * map_resolution;
                            min_distance = std::min(min_distance, distance);
                        }
                    }
                }
                
                // Apply gradient based on distance
                if (min_distance <= inflation_radius) {
                    uint8_t new_value = calculateInflationValue(
                        min_distance, robot_radius, inflation_radius);
                    
                    // Only increase occupancy, never decrease (conservative approach)
                    uint8_t current = grid(r, c);
                    grid.set_value(r, c, std::max(current, new_value));
                }
            }
        }
    }
    
    // Calculate inflation value based on distance from obstacle
    static uint8_t calculateInflationValue(double distance, 
                                          double robot_radius,
                                          double inflation_radius) {
        if (distance <= 0) {
            return 255; // Obstacle itself
        } else if (distance <= robot_radius * 0.5) {
            // Very close to obstacle - almost certain collision
            return 254; 
        } else if (distance <= robot_radius) {
            // Within robot radius - likely collision
            // Linear gradient from 254 to 200
            double ratio = (robot_radius - distance) / robot_radius;
            return static_cast<uint8_t>(200 + ratio * 54);
        } else if (distance <= inflation_radius) {
            // Within inflation zone - exponential falloff gradient
            double ratio = (inflation_radius - distance) / 
                          (inflation_radius - robot_radius);
            
            // Exponential falloff for smoother, more natural gradient
            double exp_ratio = std::exp(-3.0 * (1.0 - ratio));
            return static_cast<uint8_t>(51 + exp_ratio * 149);
        } else {
            // Outside inflation - completely safe
            return 0;
        }
    }
    
    // Advanced inflation considering map uncertainty and localization errors
    static void adaptiveInflation(concord::Grid<uint8_t>& grid,
                                  double robot_radius,
                                  double map_resolution,
                                  double localization_uncertainty) {
        // Adjust inflation based on multiple uncertainty sources
        double resolution_uncertainty = map_resolution * 0.5; // Half-cell uncertainty
        double total_uncertainty = std::sqrt(
            localization_uncertainty * localization_uncertainty +
            resolution_uncertainty * resolution_uncertainty);
        
        // More uncertainty = more inflation needed
        double adjusted_radius = robot_radius + total_uncertainty;
        
        inflateObstacles(grid, adjusted_radius, map_resolution);
    }
    
    // Create Gaussian-based inflation for smoother gradients
    static void gaussianInflation(concord::Grid<uint8_t>& grid,
                                  double robot_radius,
                                  double map_resolution) {
        double sigma = robot_radius / 2.0; // Standard deviation
        int kernel_size = static_cast<int>(std::ceil(3 * sigma / map_resolution));
        
        // Pre-compute Gaussian kernel
        std::vector<std::vector<double>> kernel(
            2 * kernel_size + 1, 
            std::vector<double>(2 * kernel_size + 1));
        
        for (int i = -kernel_size; i <= kernel_size; ++i) {
            for (int j = -kernel_size; j <= kernel_size; ++j) {
                double distance = std::sqrt(i*i + j*j) * map_resolution;
                kernel[static_cast<size_t>(i + kernel_size)][static_cast<size_t>(j + kernel_size)] = 
                    std::exp(-(distance * distance) / (2 * sigma * sigma));
            }
        }
        
        // Apply convolution with obstacles
        auto original = grid;
        for (size_t r = 0; r < grid.rows(); ++r) {
            for (size_t c = 0; c < grid.cols(); ++c) {
                if (original(r, c) == 255) continue;
                
                double influence = 0.0;
                for (int i = -kernel_size; i <= kernel_size; ++i) {
                    for (int j = -kernel_size; j <= kernel_size; ++j) {
                        int nr = static_cast<int>(r) + i;
                        int nc = static_cast<int>(c) + j;
                        
                        if (nr >= 0 && nr < static_cast<int>(grid.rows()) && 
                            nc >= 0 && nc < static_cast<int>(grid.cols())) {
                            if (original(static_cast<size_t>(nr), static_cast<size_t>(nc)) == 255) {
                                influence += kernel[static_cast<size_t>(i + kernel_size)][static_cast<size_t>(j + kernel_size)];
                            }
                        }
                    }
                }
                
                // Convert influence to occupancy value
                if (influence > 0) {
                    uint8_t inflated = static_cast<uint8_t>(std::min(254.0, influence * 200.0));
                    uint8_t current = grid(r, c);
                    grid.set_value(r, c, std::max(current, inflated));
                }
            }
        }
    }
    
    // Fast inflation using jump flooding algorithm for large grids
    static void fastInflation(concord::Grid<uint8_t>& grid,
                              double robot_radius,
                              double map_resolution) {
        int inflation_cells = static_cast<int>(std::ceil(robot_radius / map_resolution));
        
        // Create distance transform grid
        concord::Grid<float> distance_grid(grid.rows(), grid.cols(), map_resolution, 
                                          true, concord::Pose{}, false);
        
        // Initialize distance grid
        for (size_t r = 0; r < grid.rows(); ++r) {
            for (size_t c = 0; c < grid.cols(); ++c) {
                if (grid(r, c) == 255) {
                    distance_grid.set_value(r, c, 0.0f); // Obstacle distance is 0
                } else {
                    distance_grid.set_value(r, c, std::numeric_limits<float>::max());
                }
            }
        }
        
        // Jump flooding algorithm for fast distance transform
        jumpFloodingDistance(distance_grid, map_resolution);
        
        // Apply inflation based on distance
        for (size_t r = 0; r < grid.rows(); ++r) {
            for (size_t c = 0; c < grid.cols(); ++c) {
                if (grid(r, c) == 255) continue; // Skip obstacles
                
                float distance = distance_grid(r, c);
                if (distance <= robot_radius) {
                    uint8_t inflated_value = calculateInflationValue(
                        static_cast<double>(distance), robot_radius, robot_radius);
                    uint8_t current = grid(r, c);
                    grid.set_value(r, c, std::max(current, inflated_value));
                }
            }
        }
    }
    
private:
    // Jump flooding algorithm for fast distance transform computation
    static void jumpFloodingDistance(concord::Grid<float>& distance_grid, double resolution) {
        size_t max_step = std::max(distance_grid.rows(), distance_grid.cols());
        
        // Jump flooding with decreasing step sizes
        for (size_t step = max_step / 2; step >= 1; step /= 2) {
            auto temp_grid = distance_grid;
            
            for (size_t r = 0; r < distance_grid.rows(); ++r) {
                for (size_t c = 0; c < distance_grid.cols(); ++c) {
                    float min_dist = distance_grid(r, c);
                    
                    // Check 9 neighbors at current step size
                    for (int dr = -1; dr <= 1; ++dr) {
                        for (int dc = -1; dc <= 1; ++dc) {
                            int nr = static_cast<int>(r) + dr * static_cast<int>(step);
                            int nc = static_cast<int>(c) + dc * static_cast<int>(step);
                            
                            if (nr >= 0 && nr < static_cast<int>(distance_grid.rows()) &&
                                nc >= 0 && nc < static_cast<int>(distance_grid.cols())) {
                                
                                float neighbor_dist = distance_grid(static_cast<size_t>(nr), static_cast<size_t>(nc));
                                if (neighbor_dist < std::numeric_limits<float>::max()) {
                                    float total_dist = neighbor_dist + 
                                        static_cast<float>(std::sqrt(dr*dr + dc*dc) * step * resolution);
                                    min_dist = std::min(min_dist, total_dist);
                                }
                            }
                        }
                    }
                    
                    temp_grid.set_value(r, c, min_dist);
                }
            }
            
            distance_grid = temp_grid;
        }
    }
};

} // namespace gridmap
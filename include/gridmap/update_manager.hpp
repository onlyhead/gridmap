#pragma once

#include "concord/concord.hpp"
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdint>

namespace gridmap {

class UpdateManager {
public:
    // Raytrace update for laser scan integration using Bresenham's algorithm
    static void raytraceUpdate(concord::Grid<uint8_t>& grid,
                               const concord::Point& origin,
                               const concord::Point& endpoint,
                               bool hit) {
        
        // Convert world coordinates to grid indices
        auto [origin_r, origin_c] = grid.world_to_grid(origin);
        auto [endpoint_r, endpoint_c] = grid.world_to_grid(endpoint);
        // world_to_grid always returns valid clamped indices
        
        int x0 = static_cast<int>(origin_c);    // column
        int y0 = static_cast<int>(origin_r);     // row
        int x1 = static_cast<int>(endpoint_c);  // column
        int y1 = static_cast<int>(endpoint_r);   // row
        
        // Bresenham's line algorithm
        std::vector<std::pair<int, int>> line_points = bresenhamLine(x0, y0, x1, y1);
        
        // Update cells along the ray
        for (size_t i = 0; i < line_points.size(); ++i) {
            int x = line_points[i].first;
            int y = line_points[i].second;
            
            // Check bounds
            if (y < 0 || y >= static_cast<int>(grid.rows()) ||
                x < 0 || x >= static_cast<int>(grid.cols())) {
                continue;
            }
            
            size_t r = static_cast<size_t>(y);
            size_t c = static_cast<size_t>(x);
            uint8_t current_value = grid(r, c);
            
            if (i == line_points.size() - 1) {
                // Last point - the endpoint
                if (hit) {
                    // Hit - increase occupancy probability
                    uint8_t new_value = probabilisticUpdate(current_value, true, 0.7, 0.3);
                    grid.set_value(r, c, new_value);
                } else {
                    // Miss - decrease occupancy probability
                    uint8_t new_value = probabilisticUpdate(current_value, false, 0.7, 0.3);
                    grid.set_value(r, c, new_value);
                }
            } else {
                // Intermediate points - ray passed through, so mark as free
                uint8_t new_value = probabilisticUpdate(current_value, false, 0.7, 0.3);
                grid.set_value(r, c, new_value);
            }
        }
    }
    
    // Probabilistic update using log-odds representation
    static uint8_t probabilisticUpdate(uint8_t current_value,
                                       bool observation,
                                       double hit_odds,
                                       double miss_odds) {
        
        // Convert current value to probability [0, 1]
        double current_prob = static_cast<double>(current_value) / 255.0;
        
        // Clamp to avoid extreme values
        current_prob = std::clamp(current_prob, 0.01, 0.99);
        
        // Convert to log-odds
        double current_logodds = std::log(current_prob / (1.0 - current_prob));
        
        // Update based on observation
        double update_logodds;
        if (observation) {
            // Hit - increase occupancy
            update_logodds = std::log(hit_odds / (1.0 - hit_odds));
        } else {
            // Miss - decrease occupancy  
            update_logodds = std::log(miss_odds / (1.0 - miss_odds));
        }
        
        // Apply update
        double new_logodds = current_logodds + update_logodds;
        
        // Convert back to probability
        double new_prob = 1.0 / (1.0 + std::exp(-new_logodds));
        
        // Clamp and convert to uint8_t
        new_prob = std::clamp(new_prob, 0.01, 0.99);
        return static_cast<uint8_t>(new_prob * 255.0);
    }
    
    // Update from multiple laser scan points
    static void updateFromLaserScan(concord::Grid<uint8_t>& grid,
                                    const std::vector<concord::Point>& scan_points,
                                    const concord::Point& laser_origin,
                                    double max_range) {
        
        for (const auto& point : scan_points) {
            double range = laser_origin.distance_to(point);
            
            if (range <= max_range) {
                // Valid measurement - hit at endpoint
                raytraceUpdate(grid, laser_origin, point, true);
            } else {
                // Max range measurement - miss along entire ray
                concord::Point direction = point - laser_origin;
                double length = direction.magnitude();
                if (length > 0) {
                    direction = direction / length; // Normalize manually
                }
                concord::Point max_point = laser_origin + direction * max_range;
                raytraceUpdate(grid, laser_origin, max_point, false);
            }
        }
    }
    
    // Update from 3D point cloud (project to 2D layer)
    static void updateFromPointCloud(concord::Grid<uint8_t>& grid,
                                     const std::vector<concord::Point>& cloud_points,
                                     const concord::Point& sensor_origin,
                                     double layer_height_min,
                                     double layer_height_max) {
        
        // Filter points within layer height range
        std::vector<concord::Point> layer_points;
        for (const auto& point : cloud_points) {
            if (point.z >= layer_height_min && point.z <= layer_height_max) {
                // Project to 2D for this layer
                layer_points.emplace_back(point.x, point.y, 0);
            }
        }
        
        // Project sensor origin to 2D
        concord::Point sensor_2d(sensor_origin.x, sensor_origin.y, 0);
        
        // Update grid using projected points
        updateFromLaserScan(grid, layer_points, sensor_2d, 50.0); // 50m max range
    }
    
    // Temporal decay for dynamic obstacles
    static void temporalDecay(concord::Grid<uint8_t>& grid,
                              double decay_rate,
                              double time_delta) {
        
        double decay_factor = std::exp(-decay_rate * time_delta);
        
        for (size_t r = 0; r < grid.rows(); ++r) {
            for (size_t c = 0; c < grid.cols(); ++c) {
                uint8_t current = grid(r, c);
                
                // Convert to probability, apply decay, convert back
                double prob = static_cast<double>(current) / 255.0;
                
                // Decay towards neutral value (0.5 probability = 128 value)
                double neutral_prob = 0.5;
                double decayed_prob = neutral_prob + (prob - neutral_prob) * decay_factor;
                
                uint8_t new_value = static_cast<uint8_t>(decayed_prob * 255.0);
                grid.set_value(r, c, new_value);
            }
        }
    }
    
    // Apply confidence-based updates
    static void confidenceWeightedUpdate(concord::Grid<uint8_t>& grid,
                                        const concord::Grid<float>& confidence_grid,
                                        const concord::Grid<uint8_t>& observation_grid,
                                        double min_confidence = 0.1) {
        
        if (grid.rows() != confidence_grid.rows() || 
            grid.cols() != confidence_grid.cols() ||
            grid.rows() != observation_grid.rows() ||
            grid.cols() != observation_grid.cols()) {
            throw std::runtime_error("All grids must have same dimensions");
        }
        
        for (size_t r = 0; r < grid.rows(); ++r) {
            for (size_t c = 0; c < grid.cols(); ++c) {
                float confidence = confidence_grid(r, c);
                
                // Only update if confidence is above threshold
                if (confidence >= min_confidence) {
                    uint8_t current = grid(r, c);
                    uint8_t observation = observation_grid(r, c);
                    
                    // Weight the update by confidence
                    double weight = static_cast<double>(confidence);
                    double current_prob = static_cast<double>(current) / 255.0;
                    double obs_prob = static_cast<double>(observation) / 255.0;
                    
                    // Weighted average
                    double new_prob = (1.0 - weight) * current_prob + weight * obs_prob;
                    uint8_t new_value = static_cast<uint8_t>(new_prob * 255.0);
                    
                    grid.set_value(r, c, new_value);
                }
            }
        }
    }
    
    // Batch update from multiple sensors
    static void batchSensorUpdate(concord::Grid<uint8_t>& grid,
                                  const std::vector<std::vector<concord::Point>>& sensor_scans,
                                  const std::vector<concord::Point>& sensor_origins,
                                  const std::vector<double>& sensor_weights = {}) {
        
        if (sensor_scans.size() != sensor_origins.size()) {
            throw std::runtime_error("Number of scans must match number of origins");
        }
        
        // Create temporary grids for each sensor
        std::vector<concord::Grid<uint8_t>> sensor_grids;
        for (size_t i = 0; i < sensor_scans.size(); ++i) {
            auto temp_grid = grid; // Copy structure
            
            // Fill with neutral values (128 = unknown)
            for (size_t r = 0; r < temp_grid.rows(); ++r) {
                for (size_t c = 0; c < temp_grid.cols(); ++c) {
                    temp_grid.set_value(r, c, 128);
                }
            }
            
            // Update with this sensor's data
            updateFromLaserScan(temp_grid, sensor_scans[i], sensor_origins[i], 50.0);
            sensor_grids.push_back(temp_grid);
        }
        
        // Combine all sensor updates
        for (size_t r = 0; r < grid.rows(); ++r) {
            for (size_t c = 0; c < grid.cols(); ++c) {
                std::vector<double> probabilities;
                std::vector<double> weights;
                
                for (size_t i = 0; i < sensor_grids.size(); ++i) {
                    double prob = static_cast<double>(sensor_grids[i](r, c)) / 255.0;
                    probabilities.push_back(prob);
                    
                    double weight = (i < sensor_weights.size()) ? sensor_weights[i] : 1.0;
                    weights.push_back(weight);
                }
                
                // Weighted average of probabilities
                double weighted_sum = 0.0;
                double weight_sum = 0.0;
                for (size_t i = 0; i < probabilities.size(); ++i) {
                    weighted_sum += probabilities[i] * weights[i];
                    weight_sum += weights[i];
                }
                
                double final_prob = (weight_sum > 0) ? weighted_sum / weight_sum : 0.5;
                uint8_t final_value = static_cast<uint8_t>(final_prob * 255.0);
                
                grid.set_value(r, c, final_value);
            }
        }
    }
    
private:
    // Bresenham's line algorithm implementation
    static std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) {
        std::vector<std::pair<int, int>> points;
        
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int x = x0;
        int y = y0;
        
        int x_inc = (x1 > x0) ? 1 : -1;
        int y_inc = (y1 > y0) ? 1 : -1;
        
        int error = dx - dy;
        
        for (int i = 0; i < dx + dy; ++i) {
            points.emplace_back(x, y);
            
            if (error > 0) {
                x += x_inc;
                error -= dy;
            } else {
                y += y_inc;
                error += dx;
            }
        }
        
        // Add final point
        points.emplace_back(x1, y1);
        
        return points;
    }
};

} // namespace gridmap
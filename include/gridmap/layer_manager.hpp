#pragma once

#include <vector>
#include <cstddef>
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace gridmap {

class LayerManager {
public:
    // Find appropriate layer for given height
    static size_t getLayerIndex(double height, 
                                const std::vector<double>& layer_heights) {
        if (layer_heights.empty()) {
            throw std::runtime_error("No layers available");
        }
        
        // Find the layer that contains this height
        for (size_t i = 0; i < layer_heights.size(); ++i) {
            if (height <= layer_heights[i]) {
                return i;
            }
        }
        
        // If height is above all layers, return the highest layer
        return layer_heights.size() - 1;
    }
    
    // Find the two layers that bracket the given height for interpolation
    static std::pair<size_t, size_t> getBracketingLayers(
        double height,
        const std::vector<double>& layer_heights) {
        
        if (layer_heights.empty()) {
            throw std::runtime_error("No layers available");
        }
        
        if (layer_heights.size() == 1) {
            return {0, 0}; // Single layer, no interpolation possible
        }
        
        // If height is below first layer
        if (height <= layer_heights[0]) {
            return {0, 1};
        }
        
        // If height is above last layer
        if (height >= layer_heights.back()) {
            size_t last = layer_heights.size() - 1;
            return {last - 1, last};
        }
        
        // Find the two layers that bracket the height
        for (size_t i = 0; i < layer_heights.size() - 1; ++i) {
            if (height >= layer_heights[i] && height <= layer_heights[i + 1]) {
                return {i, i + 1};
            }
        }
        
        // Fallback (should not reach here)
        return {0, 1};
    }
    
    // Interpolate between two layer values based on height
    static double interpolateValue(double height,
                                   const std::vector<double>& layer_heights,
                                   size_t lower_layer,
                                   size_t upper_layer,
                                   double lower_value,
                                   double upper_value) {
        
        if (lower_layer == upper_layer || 
            layer_heights.size() <= lower_layer || 
            layer_heights.size() <= upper_layer) {
            return lower_value; // No interpolation possible
        }
        
        double lower_height = layer_heights[lower_layer];
        double upper_height = layer_heights[upper_layer];
        
        // Avoid division by zero
        if (std::abs(upper_height - lower_height) < 1e-9) {
            return lower_value;
        }
        
        // Linear interpolation
        double t = (height - lower_height) / (upper_height - lower_height);
        t = std::clamp(t, 0.0, 1.0); // Clamp to [0, 1]
        
        return lower_value + t * (upper_value - lower_value);
    }
    
    // Generate optimal layer heights for environment
    static std::vector<double> generateLayerHeights(double min_height,
                                                    double max_height,
                                                    double robot_height,
                                                    size_t num_layers = 0) {
        if (max_height <= min_height) {
            throw std::invalid_argument("max_height must be greater than min_height");
        }
        
        // If num_layers is 0, calculate based on robot height and environment
        if (num_layers == 0) {
            double height_range = max_height - min_height;
            double layer_spacing = robot_height * 0.5; // Half robot height per layer
            num_layers = std::max(2UL, static_cast<size_t>(std::ceil(height_range / layer_spacing)));
        }
        
        num_layers = std::max(2UL, num_layers); // Minimum 2 layers
        
        std::vector<double> heights;
        heights.reserve(num_layers);
        
        // Generate evenly spaced heights
        for (size_t i = 0; i < num_layers; ++i) {
            double t = static_cast<double>(i) / (num_layers - 1);
            double height = min_height + t * (max_height - min_height);
            heights.push_back(height);
        }
        
        return heights;
    }
    
    // Generate layer heights optimized for specific environments
    static std::vector<double> generateBarnLayers(double robot_height = 1.0,
                                                  double max_barn_height = 5.0) {
        std::vector<double> heights;
        
        // Ground level (0 - robot_height/2)
        heights.push_back(robot_height * 0.5);
        
        // Robot operating level (robot_height/2 - robot_height + 0.5m)
        heights.push_back(robot_height + 0.5);
        
        // Overhead obstacles level (equipment, feeders, etc.)
        heights.push_back(2.0);
        heights.push_back(3.0);
        
        // Ceiling level
        heights.push_back(max_barn_height);
        
        return heights;
    }
    
    static std::vector<double> generateWarehouseLayers(double robot_height = 1.0,
                                                       double shelf_height = 2.5,
                                                       double max_warehouse_height = 10.0) {
        std::vector<double> heights;
        
        // Ground level 
        heights.push_back(robot_height * 0.5);
        
        // Robot level
        heights.push_back(robot_height);
        
        // Shelf levels (multiple levels for tall shelving)
        for (double h = shelf_height; h < max_warehouse_height; h += shelf_height) {
            heights.push_back(h);
        }
        
        // Ceiling
        heights.push_back(max_warehouse_height);
        
        return heights;
    }
    
    // Validate layer height sequence
    static bool validateLayerHeights(const std::vector<double>& layer_heights) {
        if (layer_heights.size() < 2) {
            return false; // Need at least 2 layers for dual-layer system
        }
        
        // Check if heights are sorted in ascending order
        for (size_t i = 1; i < layer_heights.size(); ++i) {
            if (layer_heights[i] <= layer_heights[i - 1]) {
                return false; // Heights must be strictly increasing
            }
        }
        
        return true;
    }
    
    // Add a new layer height while maintaining sorted order
    static void addLayerHeight(std::vector<double>& layer_heights, double new_height) {
        // Find insertion point to maintain sorted order
        auto it = std::lower_bound(layer_heights.begin(), layer_heights.end(), new_height);
        
        // Only insert if not already present (avoid duplicates)
        if (it == layer_heights.end() || std::abs(*it - new_height) > 1e-9) {
            layer_heights.insert(it, new_height);
        }
    }
    
    // Remove a layer height
    static bool removeLayerHeight(std::vector<double>& layer_heights, double height) {
        auto it = std::find_if(layer_heights.begin(), layer_heights.end(),
                              [height](double h) { return std::abs(h - height) < 1e-9; });
        
        if (it != layer_heights.end()) {
            layer_heights.erase(it);
            return true;
        }
        
        return false;
    }
    
    // Calculate total height range covered by layers
    static double getTotalHeightRange(const std::vector<double>& layer_heights) {
        if (layer_heights.empty()) {
            return 0.0;
        }
        
        return layer_heights.back() - layer_heights.front();
    }
    
    // Get average layer spacing
    static double getAverageLayerSpacing(const std::vector<double>& layer_heights) {
        if (layer_heights.size() < 2) {
            return 0.0;
        }
        
        double total_range = getTotalHeightRange(layer_heights);
        return total_range / (layer_heights.size() - 1);
    }
};

} // namespace gridmap
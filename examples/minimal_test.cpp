#include <iostream>

// Test layer manager functionality without full gridmap
#include "gridmap/layer_manager.hpp"

int main() {
    std::cout << "=== GridMap Minimal Test ===" << std::endl;
    
    try {
        // Test LayerManager functionality
        std::vector<double> test_heights = {0.0, 0.5, 1.0, 1.5, 2.0};
        
        std::cout << "Testing LayerManager:" << std::endl;
        
        // Test layer index finding
        size_t idx = gridmap::LayerManager::getLayerIndex(0.75, test_heights);
        std::cout << "  Layer index for 0.75m: " << idx << std::endl;
        
        // Test bracketing
        auto bracket = gridmap::LayerManager::getBracketingLayers(0.75, test_heights);
        std::cout << "  Bracketing layers: [" << bracket.first << ", " << bracket.second << "]" << std::endl;
        
        // Test interpolation
        double interp = gridmap::LayerManager::interpolateValue(0.75, test_heights, 1, 2, 100.0, 150.0);
        std::cout << "  Interpolated value: " << interp << std::endl;
        
        // Test height generation
        auto generated = gridmap::LayerManager::generateLayerHeights(0.0, 3.0, 1.0, 4);
        std::cout << "  Generated heights: ";
        for (double h : generated) {
            std::cout << h << "m ";
        }
        std::cout << std::endl;
        
        // Test validation
        bool valid = gridmap::LayerManager::validateLayerHeights(test_heights);
        std::cout << "  Heights valid: " << (valid ? "Yes" : "No") << std::endl;
        
        std::cout << "\n=== Minimal Test Passed ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
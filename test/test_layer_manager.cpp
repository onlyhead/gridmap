#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "gridmap/layer_manager.hpp"

TEST_CASE("LayerManager Basic Operations") {
    SUBCASE("Layer index finding") {
        std::vector<double> heights = {0.5, 1.0, 1.5, 2.0, 2.5};
        
        CHECK(gridmap::LayerManager::getLayerIndex(0.3, heights) == 0);
        CHECK(gridmap::LayerManager::getLayerIndex(0.5, heights) == 0);
        CHECK(gridmap::LayerManager::getLayerIndex(0.8, heights) == 1);
        CHECK(gridmap::LayerManager::getLayerIndex(1.0, heights) == 1);
        CHECK(gridmap::LayerManager::getLayerIndex(2.8, heights) == 4); // Above highest
        
        // Empty heights should throw
        std::vector<double> empty_heights;
        CHECK_THROWS(gridmap::LayerManager::getLayerIndex(1.0, empty_heights));
    }
    
    SUBCASE("Bracketing layers") {
        std::vector<double> heights = {0.5, 1.0, 1.5, 2.0};
        
        auto bracket = gridmap::LayerManager::getBracketingLayers(0.75, heights);
        CHECK(bracket.first == 0);
        CHECK(bracket.second == 1);
        
        bracket = gridmap::LayerManager::getBracketingLayers(1.25, heights);
        CHECK(bracket.first == 1);
        CHECK(bracket.second == 2);
        
        // Edge cases
        bracket = gridmap::LayerManager::getBracketingLayers(0.3, heights); // Below first
        CHECK(bracket.first == 0);
        CHECK(bracket.second == 1);
        
        bracket = gridmap::LayerManager::getBracketingLayers(2.5, heights); // Above last
        CHECK(bracket.first == 2);
        CHECK(bracket.second == 3);
        
        // Single layer
        std::vector<double> single_height = {1.0};
        bracket = gridmap::LayerManager::getBracketingLayers(0.5, single_height);
        CHECK(bracket.first == 0);
        CHECK(bracket.second == 0);
    }
}

TEST_CASE("LayerManager Interpolation") {
    SUBCASE("Value interpolation") {
        std::vector<double> heights = {0.0, 1.0, 2.0};
        
        // Perfect interpolation case
        double interpolated = gridmap::LayerManager::interpolateValue(
            0.5,     // height
            heights, // layer heights
            0,       // lower layer
            1,       // upper layer  
            100.0,   // lower value
            200.0    // upper value
        );
        CHECK(interpolated == doctest::Approx(150.0)); // Midpoint
        
        // Quarter point
        interpolated = gridmap::LayerManager::interpolateValue(
            0.25, heights, 0, 1, 100.0, 200.0);
        CHECK(interpolated == doctest::Approx(125.0));
        
        // Three-quarter point
        interpolated = gridmap::LayerManager::interpolateValue(
            0.75, heights, 0, 1, 100.0, 200.0);
        CHECK(interpolated == doctest::Approx(175.0));
        
        // Outside bounds should clamp
        interpolated = gridmap::LayerManager::interpolateValue(
            -0.5, heights, 0, 1, 100.0, 200.0);
        CHECK(interpolated == doctest::Approx(100.0)); // Clamped to lower
        
        interpolated = gridmap::LayerManager::interpolateValue(
            1.5, heights, 0, 1, 100.0, 200.0);
        CHECK(interpolated == doctest::Approx(200.0)); // Clamped to upper
    }
    
    SUBCASE("Edge cases") {
        std::vector<double> heights = {1.0, 1.0}; // Same heights (should not divide by zero)
        
        double interpolated = gridmap::LayerManager::interpolateValue(
            1.0, heights, 0, 1, 100.0, 200.0);
        CHECK(interpolated == doctest::Approx(100.0)); // Should return lower value
        
        // Same layer indices
        interpolated = gridmap::LayerManager::interpolateValue(
            1.0, heights, 0, 0, 150.0, 250.0);
        CHECK(interpolated == doctest::Approx(150.0)); // Should return lower value
    }
}

TEST_CASE("LayerManager Height Generation") {
    SUBCASE("Basic height generation") {
        auto heights = gridmap::LayerManager::generateLayerHeights(0.0, 3.0, 1.0, 4);
        
        CHECK(heights.size() == 4);
        CHECK(heights[0] == doctest::Approx(0.0));
        CHECK(heights[3] == doctest::Approx(3.0));
        CHECK(heights[1] < heights[2]);
        CHECK(heights[2] < heights[3]);
    }
    
    SUBCASE("Auto layer count") {
        auto heights = gridmap::LayerManager::generateLayerHeights(0.0, 5.0, 1.0, 0);
        
        CHECK(heights.size() >= 2); // Minimum 2 layers
        CHECK(heights.front() == doctest::Approx(0.0));
        CHECK(heights.back() == doctest::Approx(5.0));
        
        // Layers should be evenly spaced around robot height / 2
        double expected_spacing = 1.0 * 0.5; // robot_height * 0.5
        double actual_spacing = gridmap::LayerManager::getAverageLayerSpacing(heights);
        CHECK(actual_spacing <= expected_spacing * 1.5); // Allow some tolerance
    }
    
    SUBCASE("Invalid parameters") {
        CHECK_THROWS(gridmap::LayerManager::generateLayerHeights(3.0, 1.0, 1.0, 4)); // max < min
    }
    
    SUBCASE("Environment-specific generators") {
        // Barn layers
        auto barn_heights = gridmap::LayerManager::generateBarnLayers(1.2, 4.0);
        CHECK(barn_heights.size() >= 3);
        CHECK(barn_heights.front() <= 1.0);
        CHECK(barn_heights.back() == doctest::Approx(4.0));
        
        // Warehouse layers
        auto warehouse_heights = gridmap::LayerManager::generateWarehouseLayers(1.0, 2.0, 8.0);
        CHECK(warehouse_heights.size() >= 4);
        CHECK(warehouse_heights.back() == doctest::Approx(8.0));
    }
}

TEST_CASE("LayerManager Validation and Modification") {
    SUBCASE("Height validation") {
        std::vector<double> valid_heights = {0.5, 1.0, 1.5, 2.0};
        CHECK(gridmap::LayerManager::validateLayerHeights(valid_heights) == true);
        
        std::vector<double> invalid_heights = {1.0, 0.5, 2.0}; // Not sorted
        CHECK(gridmap::LayerManager::validateLayerHeights(invalid_heights) == false);
        
        std::vector<double> duplicate_heights = {0.5, 1.0, 1.0, 2.0}; // Duplicates
        CHECK(gridmap::LayerManager::validateLayerHeights(duplicate_heights) == false);
        
        std::vector<double> single_height = {1.0}; // Too few layers
        CHECK(gridmap::LayerManager::validateLayerHeights(single_height) == false);
        
        std::vector<double> empty_heights; // Empty
        CHECK(gridmap::LayerManager::validateLayerHeights(empty_heights) == false);
    }
    
    SUBCASE("Height modification") {
        std::vector<double> heights = {0.5, 1.5, 2.5};
        
        // Add height
        gridmap::LayerManager::addLayerHeight(heights, 1.0);
        CHECK(heights.size() == 4);
        CHECK(heights[1] == doctest::Approx(1.0)); // Should be inserted at correct position
        CHECK(gridmap::LayerManager::validateLayerHeights(heights) == true);
        
        // Try to add duplicate (should not add)
        size_t old_size = heights.size();
        gridmap::LayerManager::addLayerHeight(heights, 1.0);
        CHECK(heights.size() == old_size); // Size unchanged
        
        // Remove height
        bool removed = gridmap::LayerManager::removeLayerHeight(heights, 1.0);
        CHECK(removed == true);
        CHECK(heights.size() == 3);
        
        // Try to remove non-existent height
        removed = gridmap::LayerManager::removeLayerHeight(heights, 3.0);
        CHECK(removed == false);
    }
}

TEST_CASE("LayerManager Statistics") {
    SUBCASE("Height range and spacing") {
        std::vector<double> heights = {0.0, 1.0, 2.0, 4.0};
        
        double range = gridmap::LayerManager::getTotalHeightRange(heights);
        CHECK(range == doctest::Approx(4.0));
        
        double avg_spacing = gridmap::LayerManager::getAverageLayerSpacing(heights);
        CHECK(avg_spacing == doctest::Approx(4.0 / 3.0)); // total_range / (num_layers - 1)
        
        // Edge cases
        std::vector<double> single_height = {2.0};
        CHECK(gridmap::LayerManager::getTotalHeightRange(single_height) == doctest::Approx(0.0));
        CHECK(gridmap::LayerManager::getAverageLayerSpacing(single_height) == doctest::Approx(0.0));
        
        std::vector<double> empty_heights;
        CHECK(gridmap::LayerManager::getTotalHeightRange(empty_heights) == doctest::Approx(0.0));
        CHECK(gridmap::LayerManager::getAverageLayerSpacing(empty_heights) == doctest::Approx(0.0));
    }
}
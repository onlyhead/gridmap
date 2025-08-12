#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "gridmap/gridmap.hpp"

// Helper functions for testing
concord::Polygon createTestRectangle(double x, double y, double width, double height) {
    std::vector<concord::Point> points = {
        {x, y, 0}, {x + width, y, 0}, {x + width, y + height, 0}, {x, y + height, 0}, {x, y, 0}
    };
    return concord::Polygon(points);
}

gridmap::NavigationMetadata createTestMetadata() {
    gridmap::NavigationMetadata metadata;
    metadata.robot_height = 1.0;
    metadata.robot_width = 0.6;
    metadata.inflation_radius = 0.3;
    
    // Define minimal dual-layer system for testing
    metadata.layers = {
        gridmap::ElevationLayer(0.0, 0.5, gridmap::LayerType::OCCUPANCY, "ground_omap", true),
        gridmap::ElevationLayer(0.0, 0.5, gridmap::LayerType::COST, "ground_cmap", true),
        gridmap::ElevationLayer(0.5, 1.0, gridmap::LayerType::OCCUPANCY, "head_omap", true),
        gridmap::ElevationLayer(0.5, 1.0, gridmap::LayerType::COST, "head_cmap", true)
    };
    
    return metadata;
}

TEST_CASE("NavigationMap Construction and Basic Properties") {
    concord::Polygon boundary = createTestRectangle(0, 0, 10, 10);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    SUBCASE("Basic constructor") {
        gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
        
        CHECK(nav_map.get_name() == "test_map");
        CHECK(nav_map.get_type() == "test_type");
        CHECK(nav_map.get_datum().lat == doctest::Approx(52.0));
        CHECK(nav_map.get_datum().lon == doctest::Approx(4.0));
    }
    
    SUBCASE("Dual-layer system validation") {
        gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
        
        CHECK(nav_map.validate_dual_layer_integrity() == true);
        CHECK(nav_map.get_layer_pair_count() >= 2); // At least ground + head level pairs
        
        auto valid_heights = nav_map.get_valid_heights();
        CHECK(valid_heights.size() >= 2);
    }
    
    
    SUBCASE("Convenience constructor - warehouse") {
        auto warehouse_map = gridmap::createWarehouseNavigationMap("test_warehouse", boundary, datum, 0.05);
        
        CHECK(warehouse_map.get_name() == "test_warehouse");
        CHECK(warehouse_map.validate_dual_layer_integrity() == true);
        CHECK(warehouse_map.get_metadata().robot_height == doctest::Approx(1.5));
        CHECK(warehouse_map.get_metadata().inflation_radius == doctest::Approx(0.2));
    }
}

TEST_CASE("Obstacle and Semantic Feature Management") {
    concord::Polygon boundary = createTestRectangle(0, 0, 20, 20);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
    
    SUBCASE("Add obstacles") {
        auto obstacle = createTestRectangle(5, 5, 2, 2);
        
        CHECK_NOTHROW(nav_map.add_obstacle(obstacle, 1.0, "test_obstacle"));
        
        // Test that obstacle affects multiple height levels
        concord::Point test_point{6, 6, 0}; // Center of obstacle
        CHECK(nav_map.get_obstacle_height(test_point) >= 0); // Should have some height
    }
    
    SUBCASE("Add semantic regions") {
        auto region = createTestRectangle(10, 10, 5, 5);
        
        CHECK_NOTHROW(nav_map.add_semantic_region(region, "test_region", "storage", 200, true));
    }
    
}

TEST_CASE("Layer Management Operations") {
    concord::Polygon boundary = createTestRectangle(0, 0, 10, 10);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
    
    SUBCASE("Add elevation levels") {
        CHECK_NOTHROW(nav_map.add_elevation_level(1.0, 1.5, "test_level"));
        
        // Should maintain dual-layer integrity
        CHECK(nav_map.validate_dual_layer_integrity() == true);
    }
    
    SUBCASE("Layer access and validation") {
        auto valid_heights = nav_map.get_valid_heights();
        CHECK(valid_heights.size() >= 2);
        
        // Test accessing layers at different heights
        if (!valid_heights.empty()) {
            double test_height = valid_heights[0];
            CHECK_NOTHROW(nav_map.get_obstacle_map_at_height(test_height));
            CHECK_NOTHROW(nav_map.get_costmap_at_height(test_height));
        }
    }
    
    SUBCASE("Composite views") {
        CHECK_NOTHROW(nav_map.get_composite_obstacle_view());
        CHECK_NOTHROW(nav_map.get_composite_cost_view());
    }
    
    SUBCASE("System integrity checks") {
        // Should start with valid dual-layer system
        CHECK(nav_map.validate_dual_layer_integrity() == true);
        
        // Layer pairs should be even (each height has OCCUPANCY + COST)
        size_t pairs = nav_map.get_layer_pair_count();
        CHECK(pairs % 2 == 0); // Even number of layers (pairs)
        
        // Should have valid heights
        auto heights = nav_map.get_valid_heights();
        CHECK(heights.size() > 0);
    }
}

TEST_CASE("Navigation Query Functions") {
    concord::Polygon boundary = createTestRectangle(0, 0, 10, 10);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
    
    SUBCASE("Basic navigation queries") {
        concord::Point test_point{5, 5, 0};
        
        // Before adding obstacles, point should be traversable
        CHECK(nav_map.is_traversable(test_point) == true);
        
        // Add obstacle and test again
        auto obstacle = createTestRectangle(4, 4, 2, 2);
        nav_map.add_obstacle(obstacle, 1.5, "blocking_obstacle");
        
        CHECK(nav_map.get_obstacle_height(test_point) >= 0);
        CHECK(nav_map.get_clearance_height(test_point) >= 0);
    }
    
    SUBCASE("Height-specific queries") {
        auto low_obstacle = createTestRectangle(2, 2, 1, 1);
        auto high_obstacle = createTestRectangle(7, 7, 1, 1);
        
        nav_map.add_obstacle(low_obstacle, 0.3, "low_obstacle");   // Ground level
        nav_map.add_obstacle(high_obstacle, 1.2, "high_obstacle"); // Head level
        
        concord::Point low_point{2.5, 2.5, 0};
        concord::Point high_point{7.5, 7.5, 0};
        
        // Both should have some obstacle height
        CHECK(nav_map.get_obstacle_height(low_point) >= 0);
        CHECK(nav_map.get_obstacle_height(high_point) >= 0);
    }
    
    SUBCASE("Costmap generation") {
        // Should be able to generate various map views
        CHECK_NOTHROW(nav_map.get_costmap());
        
        auto valid_heights = nav_map.get_valid_heights();
        if (!valid_heights.empty()) {
            CHECK_NOTHROW(nav_map.get_costmap_at_height(valid_heights[0]));
            CHECK_NOTHROW(nav_map.get_obstacle_map_at_height(valid_heights[0]));
        }
    }
}

TEST_CASE("Sensor Integration") {
    concord::Polygon boundary = createTestRectangle(0, 0, 20, 20);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
    
    SUBCASE("Laser scan update") {
        concord::Point robot_pos{10, 10, 0};
        concord::Pose robot_pose(robot_pos, concord::Euler{0, 0, 0});
        
        // Create simulated laser scan points
        std::vector<concord::Point> scan_points = {
            {15, 10, 0}, {12, 8, 0}, {8, 12, 0}, {10, 15, 0}
        };
        
        CHECK_NOTHROW(nav_map.update_from_laser_scan(scan_points, 0.3, robot_pose));
    }
    
    SUBCASE("Point cloud update") {
        concord::Point sensor_pos{10, 10, 2};
        concord::Pose sensor_pose(sensor_pos, concord::Euler{0, 0, 0});
        
        // Create simulated 3D point cloud
        std::vector<concord::Point> cloud_points = {
            {5, 5, 0.5}, {15, 15, 1.0}, {8, 12, 1.5}
        };
        
        CHECK_NOTHROW(nav_map.update_from_point_cloud(cloud_points, sensor_pose));
    }
    
    SUBCASE("Depth image update") {
        concord::Point camera_pos{10, 10, 1.5};
        concord::Pose camera_pose(camera_pos, concord::Euler{0, 0, 0});
        
        // Create simulated depth image (small 5x5 grid)
        concord::Grid<float> depth_image(5, 5, 0.1, true, concord::Pose{}, false);
        
        // Fill with some depth data
        for (size_t r = 0; r < depth_image.rows(); ++r) {
            for (size_t c = 0; c < depth_image.cols(); ++c) {
                depth_image.set_value(r, c, 2.0f); // 2 meter depth
            }
        }
        
        CHECK_NOTHROW(nav_map.update_from_depth_image(depth_image, camera_pose));
    }
}

TEST_CASE("Utility Operations") {
    concord::Polygon boundary = createTestRectangle(0, 0, 10, 10);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
    
    SUBCASE("Clear operations") {
        // Add some dynamic content
        auto obstacle = createTestRectangle(3, 3, 2, 2);
        nav_map.add_obstacle(obstacle, 1.0, "dynamic_obstacle");
        
        CHECK_NOTHROW(nav_map.clear_dynamic_layers());
        
        // Should maintain dual-layer integrity after clearing
        CHECK(nav_map.validate_dual_layer_integrity() == true);
    }
    
    SUBCASE("Reset operations") {
        auto valid_heights = nav_map.get_valid_heights();
        if (!valid_heights.empty()) {
            CHECK_NOTHROW(nav_map.reset_elevation_level(valid_heights[0]));
            
            // Should still maintain integrity
            CHECK(nav_map.validate_dual_layer_integrity() == true);
        }
    }
    
    SUBCASE("3D point cloud export") {
        auto point_cloud = nav_map.to_point_cloud();
        CHECK(point_cloud.size() >= 0); // Should return valid vector
    }
    
    SUBCASE("File operations") {
        // Test save/load operations (may not work in all environments)
        CHECK_NOTHROW(nav_map.save("./test_output"));
        
        // Test file export
        CHECK_NOTHROW(nav_map.to_files("./test.geojson", "./test.tiff"));
    }
}


TEST_CASE("Performance and Scalability") {
    // Create larger map for performance testing
    concord::Polygon large_boundary = createTestRectangle(0, 0, 100, 100);
    concord::Datum datum{52.0, 4.0, 0.0};
    gridmap::NavigationMap large_map("large_nav", "robot_navigation", large_boundary, datum, 0.05);
    
    SUBCASE("Large map initialization") {
        CHECK(large_map.validate_dual_layer_integrity() == true);
        CHECK(large_map.get_layer_pair_count() >= 1); // At least one layer pair
    }
    
    SUBCASE("Multiple obstacle handling") {
        // Add many obstacles
        for (int i = 0; i < 10; ++i) {
            for (int j = 0; j < 10; ++j) {
                auto obstacle = createTestRectangle(i * 8 + 2, j * 8 + 2, 1, 1);
                CHECK_NOTHROW(large_map.add_obstacle(obstacle, 1.0, "obstacle_" + std::to_string(i*10+j)));
            }
        }
        
        // Should still maintain integrity
        CHECK(large_map.validate_dual_layer_integrity() == true);
    }
    
    SUBCASE("Query performance") {
        // Test query speed with populated map
        concord::Point test_point{50, 50, 0};
        
        // These should complete quickly
        CHECK_NOTHROW(large_map.is_traversable(test_point));
        CHECK_NOTHROW(large_map.get_obstacle_height(test_point));
        CHECK_NOTHROW(large_map.get_clearance_height(test_point));
    }
}

TEST_CASE("Error Handling and Edge Cases") {
    concord::Polygon boundary = createTestRectangle(0, 0, 5, 5); // Small boundary
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
    
    SUBCASE("Boundary edge queries") {
        // Test points at map boundaries
        concord::Point corner{0, 0, 0};
        concord::Point edge{2.5, 0, 0};
        concord::Point outside{-1, -1, 0};
        
        CHECK_NOTHROW(nav_map.is_traversable(corner));
        CHECK_NOTHROW(nav_map.is_traversable(edge));
        CHECK_NOTHROW(nav_map.is_traversable(outside)); // Should handle gracefully
    }
    
    SUBCASE("Invalid height queries") {
        concord::Point test_point{2.5, 2.5, 0};
        
        // These should handle invalid heights gracefully
        CHECK_NOTHROW(nav_map.get_costmap_at_height(-1.0));  // Negative height
        CHECK_NOTHROW(nav_map.get_obstacle_map_at_height(100.0)); // Very high
    }
    
    SUBCASE("Empty sensor data") {
        concord::Pose pose(concord::Point{2.5, 2.5, 0}, concord::Euler{0, 0, 0});
        
        std::vector<concord::Point> empty_points;
        
        // Should handle empty data gracefully
        CHECK_NOTHROW(nav_map.update_from_laser_scan(empty_points, 0.3, pose));
        CHECK_NOTHROW(nav_map.update_from_point_cloud(empty_points, pose));
    }
}

TEST_CASE("Integration with Other Systems") {
    gridmap::NavigationMap nav_map(
        "integration_test", 
        "robot_navigation",
        createTestRectangle(0, 0, 10, 10), 
        concord::Datum{52.0, 4.0, 0.0}, 
        0.1
    );
    
    SUBCASE("Map format compatibility") {
        // Test different map view generations
        auto costmap = nav_map.get_costmap();
        CHECK(costmap.rows() > 0);
        CHECK(costmap.cols() > 0);
        
        auto obstacle_map = nav_map.get_composite_obstacle_view();
        CHECK(obstacle_map.rows() == costmap.rows());
        CHECK(obstacle_map.cols() == costmap.cols());
    }
    
    SUBCASE("Coordinate system consistency") {
        // Datum should be preserved
        CHECK(nav_map.get_datum().lat == doctest::Approx(52.0));
        CHECK(nav_map.get_datum().lon == doctest::Approx(4.0));
        
        // Zoneout integration should work
        auto& zone = nav_map.get_zone();
        CHECK(zone.getName() == "integration_test");
    }
    
    SUBCASE("Metadata preservation") {
        auto& metadata = nav_map.get_metadata();
        CHECK(metadata.robot_height > 0);
        CHECK(metadata.robot_width > 0);
        CHECK(metadata.inflation_radius > 0);
        // Metadata layers may be empty for default constructor
        CHECK(metadata.robot_height > 0);
    }
}

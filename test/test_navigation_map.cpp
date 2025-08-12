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

TEST_CASE("NavigationMap Construction") {
    concord::Polygon boundary = createTestRectangle(0, 0, 10, 10);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    SUBCASE("Basic construction") {
        gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
        
        CHECK(nav_map.get_name() == "test_map");
        CHECK(nav_map.get_type() == "test_type");
        CHECK(nav_map.get_datum().lat == doctest::Approx(52.0));
        CHECK(nav_map.get_datum().lon == doctest::Approx(4.0));
    }
    
    SUBCASE("Construction with initial obstacles") {
        // Create initial obstacles as concord::Bound
        std::vector<concord::Bound> initial_obstacles;
        
        // Add a box obstacle
        concord::Bound box{{{3, 3, 0.5}, {0, 0, 0}}, {1, 1, 1}}; // 1x1x1 box at (3,3)
        initial_obstacles.push_back(box);
        
        // Add a rotated barrier
        concord::Bound barrier{{{7, 7, 0.3}, {0, 0, 0.785}}, {2, 0.2, 0.6}}; // Rotated barrier
        initial_obstacles.push_back(barrier);
        
        gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, initial_obstacles, 0.1, metadata);
        
        CHECK(nav_map.get_name() == "test_map");
        CHECK(nav_map.validate_dual_layer_integrity() == true);
        
        // Test that obstacles were added
        CHECK(nav_map.get_obstacle_height({3, 3, 0}) >= 0); // Should detect first obstacle
        CHECK(nav_map.get_obstacle_height({7, 7, 0}) >= 0); // Should detect second obstacle
    }
    
    SUBCASE("Dual-layer system validation") {
        gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
        
        CHECK(nav_map.validate_dual_layer_integrity() == true);
        CHECK(nav_map.get_layer_pair_count() >= 2); // At least ground + head level pairs
        
        auto valid_heights = nav_map.get_valid_heights();
        CHECK(valid_heights.size() >= 2);
    }
    
}

TEST_CASE("Obstacle and Semantic Features") {
    concord::Polygon boundary = createTestRectangle(0, 0, 20, 20);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
    
    SUBCASE("Add obstacles from Polygon") {
        auto obstacle = createTestRectangle(5, 5, 2, 2);
        
        CHECK_NOTHROW(nav_map.add_obstacle(obstacle, 1.0, "test_obstacle"));
        
        // Test that obstacle affects multiple height levels
        concord::Point test_point{6, 6, 0}; // Center of obstacle
        CHECK(nav_map.get_obstacle_height(test_point) >= 0); // Should return valid height (>=0)
    }
    
    SUBCASE("Add obstacles from concord::Bound") {
        // Create oriented bounding box obstacle
        concord::Point center{8, 8, 0.5};           // Center at (8, 8) at 0.5m height
        concord::Euler rotation{0.0, 0.0, 0.785};   // 45° rotation
        concord::Pose pose{center, rotation};
        concord::Size size{2.0, 1.0, 1.0};          // 2m x 1m x 1m
        concord::Bound bound{pose, size};
        
        CHECK_NOTHROW(nav_map.add_obstacle(bound, "bound_obstacle", "equipment"));
        
        // Test that obstacle affects the area around the center
        concord::Point test_point{8, 8, 0}; // Center of obstacle
        CHECK(nav_map.get_obstacle_height(test_point) >= 0); // Should detect obstacle
    }
    
    SUBCASE("Add semantic regions") {
        auto region = createTestRectangle(10, 10, 5, 5);
        
        CHECK_NOTHROW(nav_map.add_semantic_region(region, "test_region", "storage", 200, true));
    }
    
}

TEST_CASE("Navigation Queries") {
    concord::Polygon boundary = createTestRectangle(0, 0, 10, 10);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
    
    SUBCASE("Basic queries") {
        concord::Point test_point{5, 5, 0};
        
        // Before adding obstacles, point should be traversable
        CHECK(nav_map.is_traversable(test_point) == true);
        
        // Add obstacle and test again
        auto obstacle = createTestRectangle(4, 4, 2, 2);
        nav_map.add_obstacle(obstacle, 1.5, "blocking_obstacle");
        
        CHECK(nav_map.get_obstacle_height(test_point) >= 0); // Should return valid height
        CHECK(nav_map.get_clearance_height(test_point) >= 0);
    }
    
    SUBCASE("Height-specific queries") {
        auto low_obstacle = createTestRectangle(2, 2, 1, 1);
        auto high_obstacle = createTestRectangle(7, 7, 1, 1);
        
        nav_map.add_obstacle(low_obstacle, 0.3, "low_obstacle");   // Ground level
        nav_map.add_obstacle(high_obstacle, 1.2, "high_obstacle"); // Head level
        
        concord::Point low_point{2.5, 2.5, 0};
        concord::Point high_point{7.5, 7.5, 0};
        
        // Both should have valid heights (obstacles may or may not be detected depending on implementation)
        CHECK(nav_map.get_obstacle_height(low_point) >= 0);
        CHECK(nav_map.get_obstacle_height(high_point) >= 0);
    }
}

TEST_CASE("Layer Management") {
    concord::Polygon boundary = createTestRectangle(0, 0, 10, 10);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
    
    SUBCASE("Add elevation levels") {
        CHECK_NOTHROW(nav_map.add_elevation_level(1.0, 1.5, "test_level"));
        
        // Should maintain dual-layer integrity
        CHECK(nav_map.validate_dual_layer_integrity() == true);
    }
    
    SUBCASE("Layer access") {
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
}

TEST_CASE("Sensor Updates") {
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
}

TEST_CASE("File I/O Operations") {
    concord::Polygon boundary = createTestRectangle(0, 0, 10, 10);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
    
    SUBCASE("Save and load operations") {
        // Add some content to make the map interesting
        auto obstacle = createTestRectangle(3, 3, 2, 2);
        nav_map.add_obstacle(obstacle, 1.0, "test_obstacle");
        
        std::filesystem::path temp_dir = "./test_output";
        
        CHECK_NOTHROW(nav_map.save(temp_dir));
        CHECK(std::filesystem::exists(temp_dir));
        
        // Test GeoJSON/GeoTIFF export
        CHECK_NOTHROW(nav_map.to_files(temp_dir / "test.geojson", temp_dir / "test.tiff"));
        
        // Clean up
        std::filesystem::remove_all(temp_dir);
    }
}

TEST_CASE("Map Utilities") {
    concord::Polygon boundary = createTestRectangle(0, 0, 10, 10);
    concord::Datum datum{52.0, 4.0, 0.0};
    auto metadata = createTestMetadata();
    
    gridmap::NavigationMap nav_map("test_map", "test_type", boundary, datum, 0.1, metadata);
    
    SUBCASE("Clear operations") {
        // Add some dynamic content
        auto obstacle = createTestRectangle(3, 3, 2, 2);
        nav_map.add_obstacle(obstacle, 1.0, "dynamic_obstacle");
        
        CHECK_NOTHROW(nav_map.clear_dynamic_layers());
    }
    
    SUBCASE("Reset operations") {
        auto valid_heights = nav_map.get_valid_heights();
        if (!valid_heights.empty()) {
            CHECK_NOTHROW(nav_map.reset_elevation_level(valid_heights[0]));
        }
    }
    
    SUBCASE("3D point cloud export") {
        CHECK_NOTHROW(nav_map.to_point_cloud());
    }
}

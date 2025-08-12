/**
 * Layer Testing Example - Clear verification of multi-height obstacles
 *
 * This example creates a simple grid with 10 layers (0-2m, every 20cm)
 * and places obstacles at specific heights to verify layer functionality.
 */

#include "gridmap/gridmap.hpp"
#include <iostream>
#include <vector>

int main() {
    std::cout << "🧪 GridMap Layer Testing - 10 layers, obstacles at different heights\n\n";

    // Create a simple 10m x 10m area
    std::vector<concord::Point> boundary_points = {
        {0.0, 0.0, 0.0},   // Bottom-left
        {10.0, 0.0, 0.0},  // Bottom-right
        {10.0, 10.0, 0.0}, // Top-right
        {0.0, 10.0, 0.0},  // Top-left
        {0.0, 0.0, 0.0}    // Close polygon
    };
    concord::Polygon boundary(boundary_points);
    concord::Datum datum{0.0, 0.0, 0.0}; // Simple origin

    // Create navigation map with 10cm resolution
    gridmap::NavigationMap nav_map("layer_test", "testing", boundary, datum, 0.1);

    std::cout << "✓ Created 10m x 10m test area with 10cm resolution\n";

    // Add 10 layers: 0-20cm, 20-40cm, 40-60cm, ..., 180-200cm
    std::cout << "📏 Adding 10 layers (every 20cm up to 2m height):\n";
    for (int i = 0; i < 10; i++) {
        double height_min = i * 0.2;       // 0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8
        double height_max = (i + 1) * 0.2; // 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0

        std::string layer_name = "layer_" + std::to_string(static_cast<int>(height_max * 100)) + "cm";
        nav_map.add_elevation_level(height_min, height_max, layer_name);

        std::cout << "  - Layer " << i << ": " << height_min << "m to " << height_max << "m (" << layer_name << ")\n";
    }

    std::cout << "✓ Total layers: " << nav_map.get_layer_pair_count() << "\n\n";

    // Add obstacles at different heights to test layer distribution
    std::cout << "🚧 Adding test obstacles at different heights:\n";

    // Obstacle 1: Short box (30cm high) at position (2, 2)
    {
        concord::Point box_center(2.0, 2.0, 0.15); // Center at 15cm (half of 30cm)
        concord::Euler box_rotation(0.0, 0.0, 0.0);
        concord::Pose box_pose{box_center, box_rotation};
        concord::Size box_size{0.5, 0.5, 0.3}; // 50cm x 50cm x 30cm
        concord::Bound short_box{box_pose, box_size};
        nav_map.add_obstacle(short_box, "short_box", "test");
        std::cout << "  1. Short box: 30cm high at (2, 2) - should appear in layers 0-1\n";
    }

    // Obstacle 2: Medium pole (80cm high) at position (4, 4)
    {
        concord::Point pole_center(4.0, 4.0, 0.4); // Center at 40cm (half of 80cm)
        concord::Euler pole_rotation(0.0, 0.0, 0.0);
        concord::Pose pole_pose{pole_center, pole_rotation};
        concord::Size pole_size{0.2, 0.2, 0.8}; // 20cm x 20cm x 80cm
        concord::Bound medium_pole{pole_pose, pole_size};
        nav_map.add_obstacle(medium_pole, "medium_pole", "test");
        std::cout << "  2. Medium pole: 80cm high at (4, 4) - should appear in layers 0-3\n";
    }

    // Obstacle 3: Tall wall (150cm high) at position (6, 6)
    {
        concord::Point wall_center(6.0, 6.0, 0.75); // Center at 75cm (half of 150cm)
        concord::Euler wall_rotation(0.0, 0.0, 0.0);
        concord::Pose wall_pose{wall_center, wall_rotation};
        concord::Size wall_size{1.0, 0.3, 1.5}; // 100cm x 30cm x 150cm
        concord::Bound tall_wall{wall_pose, wall_size};
        nav_map.add_obstacle(tall_wall, "tall_wall", "test");
        std::cout << "  3. Tall wall: 150cm high at (6, 6) - should appear in layers 0-7\n";
    }

    // Obstacle 4: Very tall antenna (200cm high) at position (8, 8)
    {
        concord::Point antenna_center(8.0, 8.0, 1.0); // Center at 100cm (half of 200cm)
        concord::Euler antenna_rotation(0.0, 0.0, 0.0);
        concord::Pose antenna_pose{antenna_center, antenna_rotation};
        concord::Size antenna_size{0.1, 0.1, 2.0}; // 10cm x 10cm x 200cm
        concord::Bound very_tall_antenna{antenna_pose, antenna_size};
        nav_map.add_obstacle(very_tall_antenna, "antenna", "test");
        std::cout << "  4. Antenna: 200cm high at (8, 8) - should appear in ALL layers 0-9\n";
    }

    std::cout << "\n🔍 Validation:\n";
    if (nav_map.validate_dual_layer_integrity()) {
        std::cout << "✓ Dual-layer system integrity: PASSED\n";
    } else {
        std::cout << "✗ Dual-layer system integrity: FAILED\n";
        return -1;
    }

    // Export all layers to output/ folder for inspection
    std::cout << "\n💾 Exporting all layers to output/ folder:\n";
    std::filesystem::create_directories("output");

    auto heights = nav_map.get_valid_heights();
    std::sort(heights.begin(), heights.end());

    for (size_t i = 0; i < heights.size(); ++i) {
        double height = heights[i];
        std::string filename = "output/test_layer_" + std::to_string(static_cast<int>(height * 100)) + "cm.pgm";

        try {
            auto obstacle_grid = nav_map.get_obstacle_map_at_height(height);
            gridmap::PgmExporter::exportObstacleMapToPGM(obstacle_grid, filename);

            // Count obstacles by checking for black pixels (0 = occupied)
            size_t obstacle_count = 0;
            for (size_t r = 0; r < obstacle_grid.rows(); ++r) {
                for (size_t c = 0; c < obstacle_grid.cols(); ++c) {
                    if (obstacle_grid(r, c) == 0) { // occupied
                        obstacle_count++;
                    }
                }
            }

            std::cout << "  Layer " << i << " (" << static_cast<int>(height * 100) << "cm): " << filename;
            std::cout << " - " << obstacle_count << " obstacle pixels";

            // Predict what should be in this layer
            std::cout << " (expect: ";
            bool first = true;
            if (height <= 0.3) { // Short box height
                if (!first)
                    std::cout << ", ";
                std::cout << "short_box";
                first = false;
            }
            if (height <= 0.8) { // Medium pole height
                if (!first)
                    std::cout << ", ";
                std::cout << "medium_pole";
                first = false;
            }
            if (height <= 1.5) { // Tall wall height
                if (!first)
                    std::cout << ", ";
                std::cout << "tall_wall";
                first = false;
            }
            // Antenna appears in all layers
            if (!first)
                std::cout << ", ";
            std::cout << "antenna";
            std::cout << ")\n";

        } catch (const std::exception &e) {
            std::cout << "  ⚠ Failed to export layer " << i << ": " << e.what() << "\n";
        }
    }

    std::cout << "\n✅ Layer test complete!\n";
    std::cout << "📋 Summary:\n";
    std::cout << "  - 10 layers created (0-200cm, every 20cm)\n";
    std::cout << "  - 4 obstacles at different heights\n";
    std::cout << "  - All layers exported to output/test_layer_*cm.pgm\n";
    std::cout << "  - Check obstacle pixel counts to verify correct distribution\n";

    std::cout << "\n🔍 Verification tips:\n";
    std::cout << "  - Layers 0-1 (0-40cm): should have ALL 4 obstacles\n";
    std::cout << "  - Layers 2-3 (40-80cm): should have 3 obstacles (no short_box)\n";
    std::cout << "  - Layers 4-7 (80-160cm): should have 2 obstacles (antenna + tall_wall)\n";
    std::cout << "  - Layers 8-9 (160-200cm): should have 1 obstacle (antenna only)\n";

    return 0;
}

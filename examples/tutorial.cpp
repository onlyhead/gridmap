#include "gridmap/gridmap.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

void printSectionHeader(const std::string &title) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << " " << title << "\n";
    std::cout << std::string(60, '=') << "\n";
}

void printStepHeader(int step, const std::string &description) {
    std::cout << "\n--- Step " << step << ": " << description << " ---\n";
}

int main() {
    std::cout << "🗺️  GridMap Library Tutorial - Robot Navigation\n";
    std::cout << "Featuring concord::Bound obstacle support for oriented bounding boxes\n";
    std::cout << "Built on Zoneout ecosystem with mandatory dual-layer architecture\n";

    printSectionHeader("1. SETUP - Creating a Navigation Map");

    printStepHeader(1, "Read existing map dimensions from test.yaml");
    // Read the test.yaml to get proper dimensions and origin
    std::ifstream yaml_file("maps/test.yaml");
    double resolution = 0.05; // Default fallback
    double origin_x = 0.0, origin_y = 0.0, origin_z = 0.0;

    if (yaml_file.is_open()) {
        std::string line;
        while (std::getline(yaml_file, line)) {
            if (line.find("resolution:") == 0) {
                resolution = std::stod(line.substr(line.find(":") + 1));
            } else if (line.find("origin:") == 0) {
                // Parse origin: [x, y, z] format
                size_t start = line.find("[");
                size_t end = line.find("]");
                if (start != std::string::npos && end != std::string::npos) {
                    std::string origin_str = line.substr(start + 1, end - start - 1);
                    std::istringstream ss(origin_str);
                    std::string token;
                    int i = 0;
                    while (std::getline(ss, token, ',') && i < 3) {
                        if (i == 0)
                            origin_x = std::stod(token);
                        else if (i == 1)
                            origin_y = std::stod(token);
                        else if (i == 2)
                            origin_z = std::stod(token);
                        i++;
                    }
                }
            }
        }
        yaml_file.close();
    }

    // Calculate boundary from test.pgm dimensions (480x480 pixels at resolution)
    double map_width = 480 * resolution;  // 480 pixels * 0.05m = 24m
    double map_height = 480 * resolution; // 480 pixels * 0.05m = 24m

    // Create boundary polygon using the same origin as test.pgm
    std::vector<concord::Point> area_points = {
        {origin_x, origin_y, 0.0},                          // Bottom-left (from yaml)
        {origin_x + map_width, origin_y, 0.0},              // Bottom-right
        {origin_x + map_width, origin_y + map_height, 0.0}, // Top-right
        {origin_x, origin_y + map_height, 0.0},             // Top-left
        {origin_x, origin_y, 0.0}                           // Close polygon
    };
    concord::Polygon area_boundary(area_points);

    std::cout << "✓ Read map configuration from maps/test.yaml:\n";
    std::cout << "  - Resolution: " << resolution << "m per pixel\n";
    std::cout << "  - Origin: (" << origin_x << ", " << origin_y << ", " << origin_z << ")\n";
    std::cout << "  - Map size: " << map_width << "m x " << map_height << "m\n";
    std::cout << "  - Grid dimensions: 480x480 pixels\n";

    printStepHeader(2, "Set WGS84 datum coordinates");
    // Use coordinates that match the test map area
    concord::Datum area_datum{52.1234, 4.5678, origin_z}; // Use Z from yaml

    std::cout << "✓ Navigation area location: Lat=" << area_datum.lat << ", Lon=" << area_datum.lon
              << ", Alt=" << area_datum.alt << "\n";
    std::cout << "✓ This follows zoneout ecosystem WGS84 → ENU workflow\n";

    printStepHeader(3, "Configure robot and navigation metadata");
    // Configure robot-specific navigation parameters
    gridmap::NavigationMetadata metadata;
    metadata.robot_height = 1.5;           // 1.5m robot height
    metadata.robot_width = 0.8;            // 0.8m robot width
    metadata.max_traversable_step = 0.15;  // 15cm max step
    metadata.max_traversable_slope = 20.0; // 20° max slope
    metadata.inflation_radius = 0.4;       // 40cm safety margin

    std::cout << "✓ Robot configuration:\n";
    std::cout << "  - Height: " << metadata.robot_height << "m\n";
    std::cout << "  - Width: " << metadata.robot_width << "m\n";
    std::cout << "  - Max step: " << metadata.max_traversable_step << "m\n";
    std::cout << "  - Max slope: " << metadata.max_traversable_slope << "°\n";
    std::cout << "  - Safety radius: " << metadata.inflation_radius << "m\n";

    printStepHeader(4, "Create NavigationMap with dual-layer system");

    // Option 1: Create with initial obstacles as concord::Bound objects
    std::vector<concord::Bound> initial_obstacles;

    // Pre-define some obstacles as oriented bounding boxes
    // Office desk (0.8m high, 1.2m x 0.6m)
    concord::Bound desk_bound{
        {{8.0, 12.0, 0.4}, {0.0, 0.0, 0.0}}, // Pose: center at (8, 12, 0.4)
        {1.2, 0.6, 0.8}                      // Size: 1.2m x 0.6m x 0.8m
    };
    initial_obstacles.push_back(desk_bound);

    // Filing cabinet (1.4m high, 0.4m x 0.6m) rotated 30°
    concord::Bound cabinet_bound{
        {{12.0, 8.0, 0.7}, {0.0, 0.0, 0.524}}, // Pose: center at (12, 8, 0.7), 30° rotation
        {0.4, 0.6, 1.4}                        // Size: 0.4m x 0.6m x 1.4m
    };
    initial_obstacles.push_back(cabinet_bound);

    // Create the navigation map with initial obstacles using dimensions from test.yaml
    gridmap::NavigationMap nav_map("navigation_area", "robot_navigation", area_boundary, area_datum, initial_obstacles,
                                   resolution, metadata);

    std::cout << "✓ NavigationMap created with:\n";
    std::cout << "  - Resolution: " << (resolution * 100) << "cm per pixel\n";
    std::cout << "  - Mandatory dual-layer system (OCCUPANCY + COST)\n";
    std::cout << "  - Initial obstacles: " << initial_obstacles.size() << " concord::Bound objects\n";
    std::cout << "  - Integrated with zoneout::Zone infrastructure\n";
    std::cout << "  - Layer pairs: " << nav_map.get_layer_pair_count() << "\n";

    // Verify dual-layer integrity
    if (nav_map.validate_dual_layer_integrity()) {
        std::cout << "✓ Dual-layer system validation: PASSED\n";
    } else {
        std::cout << "✗ Dual-layer system validation: FAILED\n";
        return -1;
    }

    printSectionHeader("2. MULTI-HEIGHT LAYERS - 2.5D Navigation");

    printStepHeader(5, "Add elevation layers for different heights");
    // Add layers for robot navigation at different elevations
    nav_map.add_elevation_level(0.0, 0.5, "ground_level");   // Ground level (0-50cm)
    nav_map.add_elevation_level(0.5, 1.0, "low_obstacles");  // Low obstacles (50cm-1m)
    nav_map.add_elevation_level(1.0, 2.0, "high_obstacles"); // High obstacles (1-2m)

    std::cout << "✓ Added 3 elevation layers:\n";
    auto valid_heights = nav_map.get_valid_heights();
    for (size_t i = 0; i < valid_heights.size(); ++i) {
        std::cout << "  - Layer " << i << ": " << valid_heights[i] << "m (OCCUPANCY + COST)\n";
    }
    std::cout << "✓ Total layer pairs: " << nav_map.get_layer_pair_count() << "\n";

    printStepHeader(6, "Add static obstacles using concord::Bound (oriented bounding boxes)");

    // Create obstacles as oriented bounding boxes with Pose + Size

    // Static wall obstacle (2m high, 2m wide, 0.5m thick) at position (6, 8.25)
    concord::Point wall_center{6.0, 8.25, 1.0};  // Center at 1m height (half of 2m)
    concord::Euler wall_rotation{0.0, 0.0, 0.0}; // No rotation
    concord::Pose wall_pose{wall_center, wall_rotation};
    concord::Size wall_size{2.0, 0.5, 2.0}; // 2m wide, 0.5m thick, 2m high
    concord::Bound wall_bound{wall_pose, wall_size};
    nav_map.add_obstacle(wall_bound, "wall_1", "wall");

    // Low barrier (0.6m high, 3m long, 0.3m thick) at position (13.5, 5.15) rotated 45°
    concord::Point barrier_center{13.5, 5.15, 0.3};   // Center at 0.3m height (half of 0.6m)
    concord::Euler barrier_rotation{0.0, 0.0, 0.785}; // 45° rotation (π/4 radians)
    concord::Pose barrier_pose{barrier_center, barrier_rotation};
    concord::Size barrier_size{3.0, 0.3, 0.6}; // 3m long, 0.3m thick, 0.6m high
    concord::Bound barrier_bound{barrier_pose, barrier_size};
    nav_map.add_obstacle(barrier_bound, "barrier_1", "barrier");

    // Equipment box (1.5m high, 1m x 1m footprint) at position (18, 3)
    concord::Point box_center{18.0, 3.0, 0.75}; // Center at 0.75m height (half of 1.5m)
    concord::Euler box_rotation{0.0, 0.0, 0.0}; // No rotation
    concord::Pose box_pose{box_center, box_rotation};
    concord::Size box_size{1.0, 1.0, 1.5}; // 1m x 1m x 1.5m
    concord::Bound box_bound{box_pose, box_size};
    nav_map.add_obstacle(box_bound, "equipment_box", "equipment");

    std::cout << "✓ Added oriented bounding box obstacles:\n";
    std::cout << "  - Wall: 2m×0.5m×2m at (6, 8.25) - no rotation\n";
    std::cout << "  - Barrier: 3m×0.3m×0.6m at (13.5, 5.15) - 45° rotation\n";
    std::cout << "  - Equipment: 1m×1m×1.5m at (18, 3) - no rotation\n";
    std::cout << "  - All obstacles automatically distributed to appropriate height layers\n";

    printSectionHeader("3. SEMANTIC MAPPING - Named Regions");

    printStepHeader(7, "Add semantic regions for path planning");

    // Define a work area
    std::vector<concord::Point> work_area_points = {{2.0, 2.0, 0.0}, {8.0, 2.0, 0.0}, {8.0, 4.0, 0.0}, {2.0, 4.0, 0.0}};
    concord::Polygon work_area(work_area_points);
    nav_map.add_semantic_region(work_area, "work_zone", "workspace", 0, true);

    // Define a storage area
    std::vector<concord::Point> storage_points = {
        {16.0, 10.0, 0.0}, {19.0, 10.0, 0.0}, {19.0, 13.0, 0.0}, {16.0, 13.0, 0.0}};
    concord::Polygon storage_area(storage_points);
    nav_map.add_semantic_region(storage_area, "storage_zone", "storage", 0, true);

    std::cout << "✓ Added semantic regions:\n";
    std::cout << "  - Work zone: 6m x 2m area with workspace properties\n";
    std::cout << "  - Storage zone: 3m x 3m area with storage properties\n";
    std::cout << "✓ Semantic regions affect COST costs for intelligent planning\n";

    printSectionHeader("4. SENSOR INTEGRATION - Dynamic Updates");

    printStepHeader(8, "Simulate 2D laser scan updates");
    // Simulate robot at position (3, 3) scanning obstacles
    concord::Point robot_pos{3.0, 3.0, 0.0};
    concord::Pose robot_pose{{3.0, 3.0, 0.0}, {0.0, 0.0, 0.0}}; // Facing north

    // Simulate laser scan detecting obstacles
    std::vector<concord::Point> laser_points = {
        {4.5, 3.2, 0.0}, // Detected obstacle point 1
        {4.6, 3.1, 0.0}, // Detected obstacle point 2
        {4.7, 3.0, 0.0}, // Detected obstacle point 3
    };

    nav_map.update_from_laser_scan(laser_points, 0.3, robot_pose); // Scan at 30cm height

    std::cout << "✓ Updated from 2D laser scan:\n";
    std::cout << "  - Robot position: (" << robot_pos.x << ", " << robot_pos.y << ")\n";
    std::cout << "  - Scan height: 0.3m\n";
    std::cout << "  - Detected: " << laser_points.size() << " obstacle points\n";

    printStepHeader(9, "Simulate 3D point cloud updates");
    // Simulate 3D sensor detecting obstacles at various heights
    std::vector<concord::Point> cloud_points = {
        {10.5, 7.0, 0.2}, // Ground level obstacle
        {10.6, 7.1, 0.8}, // Mid-level obstacle
        {10.7, 7.2, 1.5}, // High obstacle
        {11.0, 7.0, 0.3}, // Another ground obstacle
    };

    concord::Pose sensor_pose{{10.0, 6.0, 1.0}, {0.0, 0.0, 1.57}}; // Sensor at 1m height, facing east
    nav_map.update_from_point_cloud(cloud_points, sensor_pose);

    std::cout << "✓ Updated from 3D point cloud:\n";
    std::cout << "  - Sensor position: (10.0, 6.0, 1.0)\n";
    std::cout << "  - Point cloud: " << cloud_points.size() << " points at various heights\n";
    std::cout << "  - Automatically distributed to appropriate elevation layers\n";

    printSectionHeader("5. NAVIGATION QUERIES - Path Planning Support");

    printStepHeader(10, "Query obstacle and traversability information");

    // Test navigation queries at different positions
    std::vector<concord::Point> test_points = {
        {1.0, 1.0, 0.0},  // Should be free
        {5.5, 8.2, 0.0},  // Near wall obstacle
        {4.6, 3.1, 0.0},  // Laser-detected obstacle
        {10.6, 7.1, 0.0}, // Point cloud obstacle
    };

    for (const auto &point : test_points) {
        bool traversable = nav_map.is_traversable(point);
        double obstacle_height = nav_map.get_obstacle_height(point);
        double clearance = nav_map.get_clearance_height(point);
        uint8_t cost = nav_map.get_cost_value(point, 0.25); // Check cost at 25cm height

        std::cout << "Point (" << point.x << ", " << point.y << "):\n";
        std::cout << "  - Traversable: " << (traversable ? "YES" : "NO") << "\n";
        std::cout << "  - Obstacle height: " << obstacle_height << "m\n";
        std::cout << "  - Clearance height: " << clearance << "m\n";
        std::cout << "  - Navigation cost: " << static_cast<int>(cost) << "/255\n";
    }

    printStepHeader(11, "Generate costmaps for path planners");

    // Get different costmap representations
    const auto &combined_costmap = nav_map.get_costmap();    // All layers combined
    auto ground_costmap = nav_map.get_costmap_at_height(0.25); // Ground level only
    auto composite_view = nav_map.get_composite_cost_view();   // Max projection

    std::cout << "✓ Generated costmaps for path planning:\n";
    std::cout << "  - Combined costmap: " << combined_costmap.rows() << "x" << combined_costmap.cols() << " cells\n";
    std::cout << "  - Ground level costmap: " << ground_costmap.rows() << "x" << ground_costmap.cols() << " cells\n";
    std::cout << "  - Composite view: " << composite_view.rows() << "x" << composite_view.cols() << " cells\n";
    std::cout << "✓ Ready for ROS nav_msgs/OccupancyGrid or path planners\n";

    printSectionHeader("6. EXPORT - PGM and Zoneout Formats");

    printStepHeader(12, "Export obstacle maps to PGM format");

    // Export all height layers including obstacles at each level
    std::cout << "✓ Exporting all layers with obstacles at their respective heights\n";

    // Create output directory 
    std::filesystem::create_directories("output");
    
    // Export ALL height layers (including ground level with obstacles)
    auto export_heights = nav_map.get_valid_heights();
    std::sort(export_heights.begin(), export_heights.end());

    for (size_t i = 0; i < export_heights.size(); ++i) {
        double height = export_heights[i];

        std::string filename = "output/obstacle_map_" + std::to_string(static_cast<int>(height * 100)) + "cm.pgm";
        std::string yaml_filename = "output/obstacle_map_" + std::to_string(static_cast<int>(height * 100)) + "cm.yaml";

        try {
            auto obstacle_grid = nav_map.get_obstacle_map_at_height(height);
            gridmap::PgmExporter::exportObstacleMapToPGM(obstacle_grid, filename);

            // Create corresponding YAML metadata
            std::ofstream yaml_file(yaml_filename);
            yaml_file << "image: obstacle_map_" << static_cast<int>(height * 100) << "cm.pgm\n";
            yaml_file << "resolution: " << resolution << "\n";
            yaml_file << "origin: [" << origin_x << ", " << origin_y << ", " << height << "]\n";
            yaml_file << "negate: 0\n";
            yaml_file << "occupied_thresh: 0.65\n";
            yaml_file << "free_thresh: 0.196\n";
            yaml_file << "height_layer: " << height << "\n";
            yaml_file << "reference_map: maps/test.pgm\n";
            yaml_file.close();

            std::cout << "✓ Exported layer " << i << ": " << filename << " (Height: " << height << "m";
            if (height <= 0.1) std::cout << " - Ground with obstacles";
            else if (height <= 0.8) std::cout << " - Should show desk/cabinet obstacles";  
            else if (height <= 1.5) std::cout << " - Should show wall/equipment obstacles";
            else std::cout << " - High obstacles";
            std::cout << ")\n";
        } catch (const std::exception &e) {
            std::cout << "⚠ Failed to export " << filename << ": " << e.what() << "\n";
        }
    }

    
    // Also copy the reference base map for comparison
    try {
        std::filesystem::copy_file("maps/test.pgm", "output/reference_base_map.pgm", 
                                   std::filesystem::copy_options::overwrite_existing);
        std::filesystem::copy_file("maps/test.yaml", "output/reference_base_map.yaml", 
                                   std::filesystem::copy_options::overwrite_existing);
        std::cout << "✓ Created reference: output/reference_base_map.pgm (original test.pgm for comparison)\n";
    } catch (const std::exception &e) {
        std::cout << "⚠ Failed to copy reference map: " << e.what() << "\n";
    }
    
    std::cout << "✓ Multi-layer map created: " << export_heights.size() << " height layers in output/\n";
    std::cout << "✓ Each layer shows obstacles that exist at that height level\n";
    std::cout << "✓ Compare with reference_base_map.pgm to see added obstacles\n";

    printStepHeader(13, "Export to zoneout ecosystem format");

    // Save using zoneout's WGS84/ENU workflow
    std::filesystem::path output_dir = "navigation_map_export";
    nav_map.save(output_dir);

    std::cout << "✓ Exported to zoneout format:\n";
    std::cout << "  - Directory: " << output_dir << "\n";
    std::cout << "  - Vector data: GeoJSON with WGS84 coordinates\n";
    std::cout << "  - Raster data: GeoTIFF with proper georeference\n";
    std::cout << "  - Compatible with QGIS, ArcGIS, and geospatial tools\n";

    printSectionHeader("7. ADVANCED FEATURES");

    printStepHeader(14, "3D point cloud export");

    // Generate point cloud representation
    auto point_cloud = nav_map.to_point_cloud();
    std::cout << "✓ Generated 3D point cloud: " << point_cloud.size() << " points\n";
    std::cout << "  - Contains all occupied voxels from OCCUPANCY layers\n";
    std::cout << "  - Ready for 3D visualization or SLAM integration\n";

    printStepHeader(15, "Layer management and validation");

    // Demonstrate layer management
    std::cout << "Layer system status:\n";
    std::cout << "  - Valid heights: " << nav_map.get_valid_heights().size() << "\n";
    std::cout << "  - Layer pairs: " << nav_map.get_layer_pair_count() << "\n";
    std::cout << "  - Dual-layer integrity: " << (nav_map.validate_dual_layer_integrity() ? "VALID" : "INVALID") << "\n";

    // Demonstrate layer reset
    nav_map.reset_elevation_level(0.25); // Reset ground level
    std::cout << "✓ Reset ground level layer (demo of dynamic updates)\n";

    printStepHeader(16, "Performance and memory usage");

    // Demonstrate clearing dynamic obstacles
    auto start_time = std::chrono::high_resolution_clock::now();
    nav_map.clear_dynamic_layers();
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "✓ Cleared dynamic layers in " << duration.count() << " microseconds\n";
    std::cout << "✓ Ready for next sensor update cycle\n";

    printSectionHeader("TUTORIAL COMPLETE");

    std::cout << "\n🎉 Successfully demonstrated GridMap library capabilities:\n\n";

    std::cout << "✅ Mandatory dual-layer system (OCCUPANCY + COST)\n";
    std::cout << "✅ Multi-height 2.5D navigation mapping\n";
    std::cout << "✅ Oriented bounding box obstacles (concord::Bound)\n";
    std::cout << "✅ Zoneout ecosystem integration (WGS84/ENU)\n";
    std::cout << "✅ Semantic region mapping\n";
    std::cout << "✅ Multi-sensor integration (2D/3D)\n";
    std::cout << "✅ Real-time navigation queries\n";
    std::cout << "✅ PGM robotics standard export\n";
    std::cout << "✅ Geospatial format compatibility\n";
    std::cout << "✅ Performance optimized operations\n";

    std::cout << "\n📚 The library is ready for:\n";
    std::cout << "  • ROS navigation stack integration\n";
    std::cout << "  • Path planning algorithms (A*, RRT*, etc.)\n";
    std::cout << "  • SLAM system integration\n";
    std::cout << "  • Multi-robot coordination\n";
    std::cout << "  • Real-time obstacle avoidance\n";
    std::cout << "  • Geospatial analysis workflows\n";

    std::cout << "\nFor more information, see README.md and API documentation.\n";

    return 0;
}

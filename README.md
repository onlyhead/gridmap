# GridMap Navigation Library

A dual-layer navigation mapping library built on the **Zoneout ecosystem** for robot navigation. Features mandatory O_MAP + C_MAP pairs at each elevation level with PGM standard compliance.

## 🚀 Quick Start

```cpp
#include "gridmap/gridmap.hpp"

// Create a navigation map with initial obstacles
std::vector<concord::Bound> obstacles = {
    {{{3, 2, 0.75}, {0, 0, 0}}, {1, 1, 1.5}},  // 1x1x1.5m box at (3,2)
    {{{7, 4, 0.4}, {0, 0, 0.785}}, {2, 0.2, 0.8}} // Rotated barrier
};

auto boundary = concord::Polygon({{0,0,0}, {10,0,0}, {10,5,0}, {0,5,0}, {0,0,0}});
auto datum = concord::Datum{52.0, 4.0, 0.0}; // WGS84 coordinates

auto nav_map = gridmap::NavigationMap("nav_area", "robot_navigation", 
                                      boundary, datum, obstacles, 0.1);

// Or add obstacles later using concord::Bound
concord::Bound equipment{{{8, 3, 1.0}, {0, 0, 0}}, {0.5, 0.5, 2.0}};
nav_map.addObstacle(equipment, "equipment_1");

// Check navigation
concord::Point robot_pos{5, 2.5, 0};
bool can_navigate = nav_map.isTraversable(robot_pos);
double clearance = nav_map.getClearanceHeight(robot_pos);
```

## 📋 Table of Contents

- [Features](#-features)
- [Architecture](#-architecture)  
- [Installation](#-installation)
- [Core Concepts](#-core-concepts)
- [API Reference](#-api-reference)
- [Examples](#-examples)
- [Agricultural Features](#-agricultural-features)
- [Export Formats](#-export-formats)
- [Integration](#-integration)
- [Testing](#-testing)

## ✨ Features

### Core Navigation Features
- **Mandatory Dual-Layer System**: Every height level has both O_MAP (obstacle detection) and C_MAP (navigation cost) layers
- **Multi-Level Mapping**: 2.5D navigation with height-based layer management
- **PGM Standard Compliance**: Robotics-standard obstacle map export (255=free/white, 0=occupied/black)
- **Real-time Sensor Integration**: Laser scan, point cloud, and depth camera updates
- **Height-aware Navigation**: Clearance calculation and traversability analysis

### Navigation Features
- **Oriented Bounding Box Obstacles**: Support for `concord::Bound` objects with pose + size
- **Multi-Height Mapping**: Pre-configured height levels for robot navigation
- **Semantic Regions**: Named areas with navigation properties
- **Dynamic Obstacles**: Real-time obstacle detection and mapping
- **Safety Margins**: Conservative inflation for safe robot navigation

### Technical Excellence
- **Zoneout Integration**: Built on proven WGS84/ENU coordinate workflow
- **Header-Only Design**: Easy integration without compilation dependencies
- **Automatic Discovery**: CMake finds all examples and tests automatically
- **Comprehensive Testing**: Full test coverage with DocTest framework

## 🏗️ Architecture

### Dual-Layer System (Mandatory)

Every elevation level **MUST** have both layer types:

```
Height Level    O_MAP Layer           C_MAP Layer
-----------     -----------           -----------
0.0 - 0.5m     Ground Obstacles  →   Ground Costs
0.5 - 1.0m     Knee Height       →   Knee Costs  
1.0 - 1.5m     Robot Height      →   Robot Costs
1.5 - 2.5m     Overhead          →   Overhead Costs
```

- **O_MAP** (Obstacle Map): Binary obstacle detection using PGM standard
  - `0` = occupied/black (obstacle present)
  - `255` = free/white (clear space)
  - `128` = unknown/gray (unexplored)

- **C_MAP** (Cost Map): Navigation cost for path planning
  - `0` = good travel/white (preferred path)
  - `255` = impossible/black (avoid completely)
  - `1-254` = increasing cost/gray (use with caution)

### Zoneout Ecosystem Integration

```cpp
// Follows Plot->Zone->Poly+Grid pattern from Zoneout
zoneout::Zone nav_zone("navigation_area", "robot_navigation", boundary, datum);
gridmap::NavigationMap nav_map(barn_zone); // Extends Zone with navigation
```

## 🛠️ Installation

### Requirements
- C++17 or later
- CMake 3.20+
- Dependencies: `concord`, `zoneout` (automatically fetched)

### Build from Source

```bash
git clone <repository-url>
cd gridmap
make config      # Configure with examples and tests
make build       # Build library and examples  
make test        # Run comprehensive test suite
```

### CMake Integration

```cmake
find_package(gridmap REQUIRED)
target_link_libraries(your_target gridmap::gridmap)
```

### Header-Only Usage

```cpp
#include "gridmap/gridmap.hpp"  // Single header includes everything
```

## 💡 Core Concepts

### 1. Dual-Layer Integrity

**Every operation maintains dual-layer integrity:**

```cpp
// Adding elevation level creates BOTH layers
nav_map.addElevationLevel(1.0, 1.5, "robot_height");
// → Creates: robot_height_omap AND robot_height_cmap

// Adding obstacles updates BOTH layers  
nav_map.addObstacle(obstacle_poly, 1.2, "conveyor");
// → Updates O_MAP with obstacle, generates C_MAP costs

// Validation ensures integrity
bool is_valid = nav_map.validateDualLayerIntegrity();
size_t layer_pairs = nav_map.getLayerPairCount(); // Always even number
```

### 2. Height-Based Navigation

```cpp
// Query navigation at specific heights
double obstacle_height = nav_map.getObstacleHeight({x, y, 0});
double clearance = nav_map.getClearanceHeight({x, y, 0});
bool traversable = nav_map.isTraversable({x, y, 0});

// Layer-specific access
auto ground_obstacles = nav_map.getObstacleMapAtHeight(0.25);  // O_MAP
auto ground_costs = nav_map.getCostmapAtHeight(0.25);          // C_MAP
```

### 3. Sensor Integration Workflow

```cpp
// 1. Collect sensor data
std::vector<concord::Point> laser_points = getLaserScan();
concord::Pose robot_pose = getCurrentPose();

// 2. Update O_MAP layers based on sensor data
nav_map.updateFromLaserScan(laser_points, 0.3, robot_pose);

// 3. Generate C_MAP from updated O_MAP  
auto costmap = nav_map.getCostmap(); // Automatically updated
```

## 📚 API Reference

### Construction

```cpp
// Navigation environment
auto nav_map = gridmap::NavigationMap(
    "nav_area", "robot_navigation", boundary_polygon, wgs84_datum, resolution);

// Warehouse environment (precise navigation) 
auto warehouse_map = gridmap::createWarehouseNavigationMap(
    "warehouse_name", boundary_polygon, wgs84_datum, resolution);

// Custom configuration
gridmap::NavigationMetadata metadata;
metadata.robot_height = 1.2;
metadata.inflation_radius = 0.4;
// ... configure layers
gridmap::NavigationMap custom_map("name", "type", boundary, datum, resolution, metadata);
```

### Obstacle Management

```cpp
// Add static obstacles
nav_map.addObstacle(footprint_polygon, height, "obstacle_name");

// Add semantic regions
nav_map.addSemanticRegion(region_polygon, "region_name", "type", height_cm);

// Agricultural-specific features
nav_map.addObstacle(obstacle_polygon, obstacle_height, "obstacle_01");
nav_map.addSemanticRegion(work_area, "work_zone", "workspace", 0, true);
```

### Navigation Queries

```cpp
// Point-based queries
bool traversable = nav_map.isTraversable(point);
double max_obstacle_height = nav_map.getObstacleHeight(point);  
double clearance_height = nav_map.getClearanceHeight(point);

// Map generation
auto costmap_2d = nav_map.getCostmap();                    // Combined 2D costmap
auto obstacles_at_height = nav_map.getObstacleMapAtHeight(1.0);  // O_MAP slice
auto costs_at_height = nav_map.getCostmapAtHeight(1.0);          // C_MAP slice

// Composite views
auto all_obstacles = nav_map.getCompositeObstacleView();   // Max projection O_MAP
auto all_costs = nav_map.getCompositeCostView();           // Max projection C_MAP
```

### Sensor Updates

```cpp
// Laser scan (2D at known height)
std::vector<concord::Point> scan_points;
nav_map.updateFromLaserScan(scan_points, scan_height, robot_pose);

// Point cloud (3D)  
std::vector<concord::Point> cloud_points;
nav_map.updateFromPointCloud(cloud_points, sensor_pose);

// Depth camera
concord::Grid<float> depth_image;
nav_map.updateFromDepthImage(depth_image, camera_pose);
```

### File I/O

```cpp
// Save using Zoneout's WGS84/ENU workflow
nav_map.save("/path/to/maps/");  // → GeoJSON + GeoTIFF files

// Load from directory
auto loaded_map = gridmap::NavigationMap::load("/path/to/maps/");

// Export specific formats
nav_map.toFiles("vector.geojson", "raster.tiff");  // Zoneout format
```

## 🌾 Agricultural Features

### Navigation Setup

```cpp
// Create navigation map
auto nav_map = gridmap::NavigationMap("navigation_area", "robot_navigation", boundary, datum);

// Add static obstacles
nav_map.addObstacle(wall_polygon, 2.0, "wall_1");  // 2m wall height
nav_map.addObstacle(barrier_polygon, 0.8, "barrier_1");  // 0.8m barrier height

// Add semantic regions
nav_map.addSemanticRegion(work_area, "work_zone", "workspace", 0, true);
nav_map.addSemanticRegion(storage_area, "storage_zone", "storage", 0, true);
```

### Obstacle Integration

```cpp
// Map static obstacles
for (const auto& obstacle : static_obstacles) {
    nav_map.addObstacle(obstacle.footprint, obstacle.height, obstacle.name);
}

// Add equipment obstacles
nav_map.addObstacle(equipment_footprint, 2.5, "equipment_1", "static");

// Create navigation corridors (clear zones)
nav_map.addSemanticRegion(corridor, "navigation_corridor", "preferred_path", 0);
```

## 📤 Export Formats

### PGM Export (Robotics Standard)

```cpp
// Export O_MAP layers as PGM files for ROS/robotics tools
gridmap::PgmExporter::PgmMetadata metadata("nav_map.pgm", 0.1);
metadata.origin = {utm_x, utm_y, height};

// Single layer export
gridmap::PgmExporter::exportObstacleMapToPGM(
    nav_map.getObstacleMapAtHeight(0.5), "ground_obstacles.pgm", metadata);

// Multi-layer export
auto heights = nav_map.getValidHeights();
std::vector<std::string> names = {"ground", "knee", "robot", "overhead"};
gridmap::PgmExporter::exportLayersToPGM(
    obstacle_layers, heights, names, "./export/", true, metadata);
```

### Point Cloud Export

```cpp
// Export 3D representation
auto point_cloud = nav_map.toPointCloud();
// → std::vector<concord::Point> with obstacle points at correct heights
```

## 🔗 Integration

### ROS Integration

```cpp
// Convert to ROS occupancy grid
auto obstacle_map = nav_map.getObstacleMapAtHeight(robot_height);
nav_msgs::OccupancyGrid ros_map = convertToROSGrid(obstacle_map);

// Convert to ROS costmap
auto cost_map = nav_map.getCostmapAtHeight(robot_height);  
costmap_2d::Costmap2D ros_costmap = convertToROSCostmap(cost_map);
```

### Path Planning Integration

```cpp
// Get costmap for A* planning
auto costmap = nav_map.getCostmap();
auto path = astar_planner.plan(start, goal, costmap);

// Height-aware planning
double robot_height = 1.2;
auto height_costmap = nav_map.getCostmapAtHeight(robot_height);
auto safe_path = planner.planWithClearance(start, goal, height_costmap, min_clearance);
```

## 🧪 Testing

### Run Test Suite

```bash
make test        # Run all tests
make test ARGS="--verbose"  # Verbose output
./build/test_layer_manager   # Run specific test
```

### Test Coverage

- **LayerManager**: Height indexing, interpolation, validation (56 assertions)
- **PgmExporter**: PGM format compliance, multi-layer export (238 assertions)  
- **NavigationMap**: Dual-layer integrity, agricultural features, sensor updates
- **Integration**: Complete workflow from sensor data to navigation queries

### Performance Tests

```cpp
// Benchmark layer operations
auto start = std::chrono::high_resolution_clock::now();
nav_map.updateFromPointCloud(large_cloud, pose);
auto duration = std::chrono::high_resolution_clock::now() - start;
```

## 🏭 Use Cases

### Robotics Applications
- **Indoor Navigation**: Office buildings, warehouses, factories
- **Service Robotics**: Safe path planning around equipment and people
- **Autonomous Delivery**: Navigation through complex indoor environments
- **Mobile Platforms**: Multi-level obstacle-aware navigation

### Indoor Logistics
- **Warehouse Navigation**: Shelf-aware path planning with height clearance
- **Factory Automation**: Multi-level obstacle detection for AGVs
- **Hospital Logistics**: Safe navigation around equipment and people
- **Retail Robotics**: Customer-safe navigation in store environments

## 🤝 Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature/dual-layer-enhancement`
3. Follow dual-layer architecture requirements
4. Add comprehensive tests
5. Submit pull request

## 📜 License

MIT License - Built on the Zoneout ecosystem for agricultural robot navigation.

## 🔗 Related Projects

- **Zoneout**: WGS84/ENU coordinate system and zone management
- **Concord**: Core geometry and grid operations
- **GeoTIV**: Raster operations and GeoTIFF export
- **GeoSON**: Vector operations and GeoJSON export

---

**GridMap Navigation Library** - Enabling safe, efficient robot navigation in agricultural and indoor environments with mandatory dual-layer mapping architecture.
# GridMap - Navigation Mapping Library

## 🎯 Core Architecture: Dual Map System

GridMap implements a **mandatory dual-layer architecture** with clear separation between obstacle detection and navigation cost:

### 1. **O_Map (Obstacle Map)** - Deterministic Layer
- **Purpose**: Pure obstacle detection and occupancy representation
- **Values**: ONLY 3 discrete values:
  - `0` (Black) = Occupied/Obstacle  
  - `255` (White) = Free/Empty space
  - `128` (Gray) = Unknown/Not_scanned
- **Format**: **Direct PGM compatibility** - can import/export PGM files directly
- **Standard**: Follows robotics PGM convention (white=free, black=occupied)
- **Use Cases**: Sensor data, SLAM output, obstacle detection, ROS navigation compatibility

### 2. **C_Map (Cost Map)** - Continuous Navigation Layer  
- **Purpose**: Navigation cost and path planning optimization
- **Values**: Continuous range 0-255:
  - `0` (White) = Very good to travel (preferred paths)
  - `1-254` (Gray gradients) = Increasing travel cost
  - `255` (Black) = Impossible to travel (blocked)
- **Format**: Navigation-optimized with inflation, gradients, and custom costs
- **Flexibility**: Can be all white (free) by default, or customized with:
  - Wall inflation around obstacles
  - Terrain difficulty costs  
  - Dynamic obstacle penalties
  - Custom navigation preferences

### 3. **GeoJSON Semantic Layer**
- **Purpose**: Tagged semantic elements and regions
- **Format**: GeoJSON polygons with mandatory parameters:
  - `height` (cm) - Height of the element
  - `static` (bool) - Whether element is static or dynamic  
  - `type` (string) - Semantic type (wall, door, equipment, etc.)
- **Flexibility**: Polygons can overlap, fully editable via geoson/zoneout

### 4. **Map Class Integration**
All three layers are encompassed in one **Map class** that:
- Enforces **mandatory dual layers** (O_Map + C_Map) at each height level
- Maintains **height-synchronized** layer pairs
- Provides **unified API** for updates, queries, and exports
- Ensures **format compatibility** (PGM for O_Map, custom for C_Map, GeoJSON for semantics)

## Executive Summary

GridMap is a navigation mapping library built on top of the Zoneout ecosystem. It provides sophisticated multi-layered 2.5D navigation maps using a **mandatory dual-layer system** separating obstacle detection (O_Map) from navigation cost (C_Map) for safe robot navigation in agricultural and indoor environments. The library leverages Zoneout's Zone, Grid, and Poly infrastructure along with its dependencies (geotiv, geoson, concord) to create production-ready navigation maps with direct PGM compatibility.

## Library Structure

GridMap is designed as a separate library that depends on:
- **zoneout**: For Zone management, polygrid operations, and high-level spatial data structures
- **geotiv**: For raster/grid operations and GeoTIFF I/O
- **geoson**: For vector/polygon operations and GeoJSON I/O  
- **concord**: For coordinate systems, transformations, and geometric primitives
- **Other libraries**: Following the same pattern as zoneout for specialized functionality

This modular approach allows GridMap to focus purely on navigation-specific features while leveraging the robust spatial data handling of the Zoneout ecosystem.

## Table of Contents

1. [Background: Robot Mapping Concepts](#background-robot-mapping-concepts)
2. [Mapping Paradigms](#mapping-paradigms)
3. [Proposed Architecture](#proposed-architecture)
4. [Implementation Design](#implementation-design)
5. [Implementation Status](#implementation-status)
6. [PGM Standard Compliance](#pgm-standard-compliance)
7. [Use Case: Barn Navigation](#use-case-barn-navigation)
8. [Integration with ROS2](#integration-with-ros2)
9. [API Examples](#api-examples)
10. [Future Extensions](#future-extensions)

## Background: Robot Mapping Concepts

### Occupancy Grids

**Definition**: Occupancy grids represent the environment as an evenly spaced field where each cell indicates the presence of an obstacle at that location.

**Key Properties**:
- **Value Representation**: 0 = completely free, 255 = fully occupied
- **Gradient Zones**: Values between 0-255 represent probability/uncertainty
- **Inflation**: Obstacles expanded with gradient for safety margins
- **Update Method**: Bayesian filtering with log-odds representation
- **Resolution**: Typically 0.05m - 1.0m per cell

**Safety Through Inflation**: Real-world maps aren't millimeter-accurate. By inflating obstacles with a gradient (rather than binary occupied/free), robots naturally prefer safer routes away from obstacles while still being able to navigate close when necessary.

**In Zoneout Context**: The existing `Grid` class already provides multi-layer grid storage with uint8_t values (0-255), perfect for gradient-based occupancy representation.

### Gray Maps / Grayscale Maps

**Definition**: Maps where cell values represent continuous properties rather than binary occupancy.

**Applications**:
- **Traversability**: 0 = easily traversable, 255 = impassable (inverse of free/occupied)
- **Elevation**: Height encoded as grayscale value
- **Cost**: Navigation cost for path planning (0 = low cost, 255 = high cost)
- **Uncertainty**: Sensor confidence levels
- **Inflation Gradients**: Smooth transitions from free to occupied space

**In Zoneout Context**: The uint8_t grid layers naturally support grayscale representation, with obstacle inflation creating natural cost gradients.

### Voxel Grids

**Definition**: 3D extension of occupancy grids, dividing space into cubic volumes.

**Characteristics**:
- Full 3D representation
- High memory consumption
- Suitable for flying robots or manipulation

**OctoMap Alternative**: Hierarchical octree structure that:
- Stores only occupied/unknown voxels
- Multi-resolution capability
- Probabilistic updates
- Memory efficient through pruning

### 2.5D Elevation Maps

**Definition**: 2D grid where each cell stores height information rather than binary occupancy.

**Advantages**:
- Compact representation vs full 3D
- Suitable for ground robots
- Encodes terrain traversability
- Efficient for real-time processing

**Multi-Layer Extension**: Stack of 2D grids at different heights:
- Each layer represents occupancy at specific elevation
- Captures overhanging obstacles
- Balances detail with efficiency

## Mapping Paradigms

### 1. Pure Grid-Based (Metric)
- **Pros**: Precise, suitable for local navigation
- **Cons**: Memory intensive, difficult global planning
- **Zoneout Implementation**: Use Grid class directly

### 2. Feature-Based (Semantic)
- **Pros**: Compact, meaningful landmarks
- **Cons**: Requires feature extraction
- **Zoneout Implementation**: Use Poly class for polygonal features

### 3. Hybrid Approach (Recommended)
- **Metric Base**: Grid layers for detailed occupancy
- **Semantic Overlay**: Polygons for regions and obstacles
- **Hierarchical Structure**: Multiple resolutions
- **Zoneout Implementation**: Combine Zone's poly_data_ and grid_data_

## Proposed Architecture

### Core Design Principles

1. **Layered Representation**: Multiple elevation layers for 2.5D mapping
2. **Hybrid Storage**: Grids for metric data, polygons for semantic features
3. **Probabilistic Updates**: Support for sensor uncertainty
4. **Efficient Memory**: Only store relevant layers
5. **Real-time Capable**: Optimized for navigation queries

### Class Hierarchy

```
NavigationMap (High-level interface)
    ├── Zone (Existing - base storage)
    │   ├── Grid (Multi-layer occupancy)
    │   └── Poly (Semantic features)
    ├── LayerManager (Height management)
    ├── CostmapGenerator (Path planning)
    └── UpdateManager (Sensor integration)
```

## Implementation Design

### NavigationMap Class

```cpp
#pragma once

#include "zoneout/zoneout.hpp"
#include "geotiv/raster.hpp"
#include "geoson/vector.hpp"
#include "concord/concord.hpp"

namespace gridmap {

// Layer types for different map representations
enum class LayerType {
    O_MAP,          // Obstacle Map - 3 discrete values (0=occupied, 255=free, 128=unknown)
    C_MAP,          // Cost Map - continuous 0-255 (0=good_travel, 255=impossible)
    ELEVATION,      // Height map
    SEMANTIC,       // Object classifications  
    DYNAMIC         // Temporary obstacles
};

// Configuration for each elevation layer
struct ElevationLayer {
    double height_min;    // Lower bound of layer (meters)
    double height_max;    // Upper bound of layer (meters)
    LayerType type;       // Type of data stored
    std::string name;     // Human-readable identifier
    bool active;          // Whether layer is currently used
};

// Navigation-specific metadata
struct NavigationMetadata {
    double robot_height;           // Robot dimensions
    double robot_width;
    double max_traversable_step;   // Maximum step height
    double max_traversable_slope;  // Maximum slope angle
    double inflation_radius;        // Obstacle inflation for safety
    std::vector<ElevationLayer> layers;  // Layer configuration
};

class NavigationMap {
private:
    zoneout::Zone base_zone_;                      // Underlying zone storage from zoneout
    NavigationMetadata nav_metadata_;              // Navigation parameters
    std::vector<double> layer_heights_;            // Height of each layer
    
    // Costmap cache for path planning
    mutable concord::Grid<uint8_t> costmap_cache_;
    mutable bool costmap_dirty_;
    
public:
    // ========== Constructors ==========
    NavigationMap(const std::string& name, 
                  const concord::Polygon& boundary,
                  const NavigationMetadata& metadata,
                  double resolution = 0.1);
    
    // Create from existing Zone
    explicit NavigationMap(const zoneout::Zone& zone, 
                          const NavigationMetadata& metadata);
    
    // ========== Layer Management ==========
    // Add occupancy layer at specific height
    void addOccupancyLayer(double height, 
                           const std::string& name = "");
    
    // Update occupancy at specific position and height
    void updateOccupancy(const concord::Point& position, 
                        double height,
                        uint8_t occupancy_value);
    
    // Get occupancy value at 3D position (0=free, 255=occupied)
    uint8_t getOccupancy(const concord::Point& position, 
                        double height) const;
    
    // ========== Height Queries ==========
    // Check if position is traversable at ground level
    bool isTraversable(const concord::Point& position) const;
    
    // Get maximum obstacle height at position
    double getObstacleHeight(const concord::Point& position) const;
    
    // Get clear height (space between ground and first obstacle)
    double getClearanceHeight(const concord::Point& position) const;
    
    // ========== Semantic Features ==========
    // Add named region (e.g., "feeding_area", "storage")
    void addSemanticRegion(const concord::Polygon& region,
                           const std::string& name,
                           const std::string& type);
    
    // Add obstacle with height information
    void addObstacle(const concord::Polygon& footprint,
                     double height,
                     const std::string& name = "");
    
    // ========== Costmap Generation ==========
    // Generate 2D costmap for path planning
    const concord::Grid<uint8_t>& getCostmap() const;
    
    // Generate costmap for specific height slice
    concord::Grid<uint8_t> getCostmapAtHeight(double height) const;
    
    // ========== Sensor Updates ==========
    // Update from 2D laser scan at known height
    void updateFromLaserScan(const std::vector<concord::Point>& points,
                             double scan_height,
                             const concord::Pose& robot_pose);
    
    // Update from 3D point cloud
    void updateFromPointCloud(const std::vector<concord::Point>& points,
                              const concord::Pose& sensor_pose);
    
    // Update from depth camera
    void updateFromDepthImage(const concord::Grid<float>& depth_image,
                              const concord::Pose& camera_pose);
    
    // ========== Barn-Specific Features ==========
    // Add pen/stall boundaries
    void addPen(const concord::Polygon& boundary,
                const std::string& pen_id,
                double fence_height = 1.2);
    
    // Add feeding trough (obstacle at specific height)
    void addFeedingTrough(const concord::Polygon& footprint,
                          double height = 0.8);
    
    // Add gate (can be open/closed)
    void addGate(const concord::LineString& gate_line,
                 const std::string& gate_id,
                 bool is_open = false);
    
    // ========== Utilities ==========
    // Clear all dynamic obstacles
    void clearDynamicLayers();
    
    // Reset specific layer
    void resetLayer(size_t layer_index);
    
    // Merge with another map
    void mergeMap(const NavigationMap& other);
    
    // ========== File I/O ==========
    void save(const std::filesystem::path& directory) const;
    static NavigationMap load(const std::filesystem::path& directory);
    
    // ========== Visualization ==========
    // Get layer for visualization
    const concord::Grid<uint8_t>& getLayer(size_t index) const;
    
    // Get composite view (max projection)
    concord::Grid<uint8_t> getCompositeView() const;
    
    // Get 3D point cloud representation
    std::vector<concord::Point> toPointCloud() const;
};

// ========== Implementation Helpers ==========

class LayerManager {
public:
    // Find appropriate layer for given height
    static size_t getLayerIndex(double height, 
                                const std::vector<double>& layer_heights);
    
    // Interpolate between layers
    static double interpolateValue(double height,
                                   size_t lower_layer,
                                   size_t upper_layer,
                                   double lower_value,
                                   double upper_value);
    
    // Generate optimal layer heights for environment
    static std::vector<double> generateLayerHeights(double min_height,
                                                    double max_height,
                                                    double robot_height,
                                                    size_t num_layers);
};

class CostmapGenerator {
public:
    // Convert occupancy to navigation cost (0=free, 255=occupied)
    static uint8_t occupancyToCost(uint8_t occupancy_value) {
        // Direct mapping since we use 0=free, 255=occupied convention
        return occupancy_value;
    }
    
    // Inflate obstacles for safety margin with gradient
    static void inflateObstacles(concord::Grid<uint8_t>& costmap,
                                 double robot_radius,
                                 double map_resolution) {
        // Use the ObstacleInflation class for gradient-based inflation
        ObstacleInflation::inflateObstacles(costmap, robot_radius, map_resolution);
    }
    
    // Combine multiple layers into single costmap
    static concord::Grid<uint8_t> combineLayers(
        const std::vector<concord::Grid<uint8_t>>& layers,
        const std::vector<double>& weights) {
        if (layers.empty()) throw std::runtime_error("No layers to combine");
        
        auto result = layers[0];  // Start with first layer
        
        // Take maximum occupancy value at each cell (conservative approach)
        for (size_t i = 1; i < layers.size(); ++i) {
            for (size_t r = 0; r < result.rows(); ++r) {
                for (size_t c = 0; c < result.cols(); ++c) {
                    uint8_t current = result.get_value(r, c);
                    uint8_t layer_val = layers[i].get_value(r, c);
                    
                    // Weight the layer value if weights provided
                    if (i < weights.size()) {
                        layer_val = static_cast<uint8_t>(layer_val * weights[i]);
                    }
                    
                    // Conservative: take maximum (most dangerous)
                    result.set_value(r, c, std::max(current, layer_val));
                }
            }
        }
        
        return result;
    }
    
    // Add traversability constraints based on elevation
    static void applyTraversability(concord::Grid<uint8_t>& costmap,
                                    const concord::Grid<float>& elevation,
                                    double max_slope) {
        for (size_t r = 1; r < costmap.rows() - 1; ++r) {
            for (size_t c = 1; c < costmap.cols() - 1; ++c) {
                // Calculate local slope
                float center = elevation.get_value(r, c);
                float dx = (elevation.get_value(r, c+1) - elevation.get_value(r, c-1)) / 2.0f;
                float dy = (elevation.get_value(r+1, c) - elevation.get_value(r-1, c)) / 2.0f;
                
                float slope = std::sqrt(dx*dx + dy*dy) / elevation.resolution();
                float slope_degrees = std::atan(slope) * 180.0 / M_PI;
                
                // If slope exceeds maximum, mark as high cost
                if (slope_degrees > max_slope) {
                    // Scale cost based on how much slope exceeds limit
                    float excess = (slope_degrees - max_slope) / max_slope;
                    uint8_t slope_cost = 100 + std::min(154.0f, excess * 154.0f);
                    
                    uint8_t current = costmap.get_value(r, c);
                    costmap.set_value(r, c, std::max(current, slope_cost));
                }
            }
        }
    }
};

class UpdateManager {
public:
    // Raytrace for laser updates
    static void raytraceUpdate(concord::Grid<uint8_t>& grid,
                               const concord::Point& origin,
                               const concord::Point& endpoint,
                               bool hit);
    
    // Probabilistic update using log-odds
    static uint8_t probabilisticUpdate(uint8_t current_value,
                                       bool observation,
                                       double hit_odds,
                                       double miss_odds);
    
    // Decay dynamic obstacles over time
    static void temporalDecay(concord::Grid<uint8_t>& grid,
                              double decay_rate);
};

} // namespace gridmap
```

## Implementation Status

### 🚧 Implementation Status - TO BE IMPLEMENTED

#### Core Library Structure  
- **❌ NavigationMap Class** (`include/gridmap/navigation_map.hpp`) - NOT IMPLEMENTED
  - Extends zoneout::Zone pattern with navigation-specific features
  - Constructor with mandatory `concord::Datum` parameter (follows zoneout pattern)
  - **CRITICAL: Mandatory dual-layer system (O_MAP + C_MAP at each height)**
  - Multi-layer occupancy grid management using zoneout::Grid infrastructure  
  - Semantic region and obstacle management using zoneout::Poly infrastructure
  - Sensor integration (point cloud, laser scan, depth image)
  - Navigation queries (traversability, clearance height, obstacle height)
  - Agricultural-specific features (pens, feeding troughs, gates)

#### Helper Classes - TO BE IMPLEMENTED
- **❌ LayerManager** (`include/gridmap/layer_manager.hpp`) - NOT IMPLEMENTED
  - Height-based layer indexing and management
  - Automatic layer height generation for environments
  - Value interpolation between layers

- **❌ CostmapGenerator** (`include/gridmap/costmap_generator.hpp`) - NOT IMPLEMENTED
  - Occupancy to cost conversion
  - Multi-layer combination strategies
  - Traversability analysis based on slope and elevation

- **❌ ObstacleInflation** (`include/gridmap/obstacle_inflation.hpp`) - NOT IMPLEMENTED
  - Gradient-based obstacle inflation
  - Distance-based safety gradients
  - Resolution-aware inflation scaling
  - Gaussian and exponential falloff algorithms

- **❌ UpdateManager** (`include/gridmap/update_manager.hpp`) - NOT IMPLEMENTED
  - Probabilistic Bayesian updates with log-odds
  - Ray-tracing for laser scan integration
  - Temporal decay for dynamic obstacles

#### Export and Visualization - TO BE IMPLEMENTED
- **❌ PgmExporter** (`include/gridmap/pgm_exporter.hpp`) - NOT IMPLEMENTED
  - **CRITICAL: PGM standard compliance (white=free, black=occupied)**
  - Dual-layer aware export (separate handling for O_MAP vs C_MAP)
  - PGM P2 (ASCII) and P5 (binary) format support
  - Individual layer exports with proper type conversion
  - Uses existing zoneout WGS84/ENU conversion workflow

#### Frontend Interface - TO BE IMPLEMENTED
- **❌ Main Header** (`include/gridmap/gridmap.hpp`) - NOT IMPLEMENTED
  - Single-include frontend for entire library
  - Clean namespace organization under `gridmap::`

#### Testing and Examples - TO BE IMPLEMENTED
- **❌ Test Suite** (`test/`) - NOT IMPLEMENTED
  - NavigationMap basic operations tests
  - Obstacle inflation algorithm tests  
  - LayerManager functionality tests

- **❌ Example Applications** (`examples/`) - NOT IMPLEMENTED
  - **ALL must use mandatory dual-layer system**
  - Barn Navigation - Agricultural robot navigation following zoneout Plot->Zone pattern
  - Warehouse Navigation - Indoor logistics robot with multi-level shelving
  - PGM Export Demo - Comprehensive PGM format demonstration
  - Reference Map Demo - Compatible with standard PGM reference maps

#### Build System - TO BE IMPLEMENTED
- **❌ CMake Integration** - Automatic example and test discovery
- **❌ Dependency Management** - Proper integration with Zoneout ecosystem
- **❌ Code Formatting** - `.clang-format` style compliance

### 🎯 CRITICAL Requirements to Implement

1. **MANDATORY DUAL-LAYER SYSTEM**: Every height level MUST have BOTH:
   - O_MAP layer (PGM standard: 255=free/white, 0=occupied/black)
   - C_MAP layer (Navigation standard: 0=free, 255=high_cost)
   
2. **NO SINGLE LAYERS ALLOWED**: System must enforce dual layers automatically

3. **ZONEOUT PATTERN COMPLIANCE**: Follow Plot->Zone->Poly+Grid pattern with WGS84/ENU workflow

4. **PGM STANDARD COMPLIANCE**: Follow robotics conventions exactly

### 📊 Current Status

- **❌ Nothing Implemented** - Starting from scratch after git reset
- **❌ Build System** - Needs complete setup
- **❌ Examples** - All need dual-layer implementation  
- **❌ Tests** - All need implementation with dual-layer system

## PGM Standard Compliance

### Standard PGM Format Convention

Based on analysis of reference maps in `reference_maps/` (copied from `other/pgm_map_creator/`), the standard PGM format uses:

#### Color Convention (Standard Robotics)
- **White (255)** = Free space (traversable)
- **Black (0)** = Occupied space (obstacles) 
- **Gray (1-254)** = Unknown/unscanned areas OR probabilistic occupancy

#### YAML Metadata Standards
```yaml
image: map.pgm              # PGM filename
resolution: 0.050000        # meters per pixel
origin: [-5, -12.0, 0.0]   # x,y,z offset of map origin  
negate: 0                   # 0 = standard colors, 1 = inverted
occupied_thresh: 0.65       # pixels darker than this are occupied
free_thresh: 0.196          # pixels lighter than this are free
```

#### Key Parameters
- **negate: 0** means standard robotics convention (white=free, black=occupied)
- **Thresholds** convert grayscale to ternary (free/occupied/unknown)
- **Resolution** typically 0.05m (5cm) per pixel for indoor environments

### 🎯 New Requirement: Dual Layer System

To comply with PGM standards while maintaining our gradient inflation:

#### **O_Map Layer** (PGM Standard)
- **Value Range**: 0-255 following PGM convention
- **Semantics**: 
  - `255` = Free space (white)
  - `0` = Occupied space (black)  
  - `1-254` = Unscanned/unknown areas (gray)
- **Usage**: Ground truth occupancy data from sensors

#### **C_Map Layer** (Navigation)  
- **Value Range**: 0-255 for path planning
- **Semantics**: 
  - `0` = Low cost (preferred paths)
  - `255` = High cost/impassable
  - Gradient inflation for safety
- **Usage**: Robot navigation and path planning

#### **Same Height, Different Purpose**
Both layers exist at the same elevation but serve different functions:
- **O_Map**: "What does the sensor see?" (PGM standard)
- **C_Map**: "Where should the robot go?" (navigation optimization)

### Implementation Requirements

#### Updated Layer Configuration
```cpp
// Example: Ground level has BOTH O_Map and C_Map
metadata.layers = {
    {0.0, 0.5, LayerType::O_MAP, "ground_occupancy", true},
    {0.0, 0.5, LayerType::C_MAP, "ground_costmap", true},     // Same height!
    {0.5, 1.0, LayerType::O_MAP, "knee_occupancy", true}, 
    {0.5, 1.0, LayerType::C_MAP, "knee_costmap", true},      // Same height!
    // ... more layers
};
```

#### Export Behavior
- **PGM Export**: Use O_MAP layers with standard white=free, black=occupied
- **Navigation**: Use C_MAP layers with gradient inflation for path planning
- **GIS Export**: Export both types with clear naming convention

This dual-layer approach ensures:
1. **Standards Compliance** - PGM files follow robotics conventions
2. **Navigation Efficiency** - C_Maps optimized for path planning  
3. **Data Integrity** - Raw sensor data preserved separately from processed navigation data
4. **Interoperability** - Compatible with existing ROS/robotics toolchains

## Use Case: Barn Navigation

### Scenario Description

A robot navigating inside a barn with:
- **Dimensions**: 30m x 50m x 5m height
- **Features**: Animal pens, feeding areas, storage zones
- **Obstacles**: Gates, troughs, equipment at various heights
- **Dynamic Elements**: Animals, movable equipment

### Layer Configuration

```cpp
NavigationMetadata barn_metadata;
barn_metadata.robot_height = 1.2;     // Robot is 1.2m tall
barn_metadata.robot_width = 0.6;      // 60cm wide
barn_metadata.max_traversable_step = 0.1;  // Can climb 10cm
barn_metadata.max_traversable_slope = 15.0; // 15 degree slopes
barn_metadata.inflation_radius = 0.3;  // 30cm safety margin

// Configure elevation layers (every 50cm up to 5m)
barn_metadata.layers = {
    {0.0,  0.5,  LayerType::O_MAP, "ground_level",  true},
    {0.5,  1.0,  LayerType::O_MAP, "knee_height",   true},
    {1.0,  1.5,  LayerType::O_MAP, "waist_height",  true},
    {1.5,  2.0,  LayerType::O_MAP, "head_height",   true},
    {2.0,  2.5,  LayerType::O_MAP, "overhead_1",    true},
    {2.5,  3.0,  LayerType::O_MAP, "overhead_2",    true},
    {3.0,  3.5,  LayerType::O_MAP, "overhead_3",    false},
    {3.5,  4.0,  LayerType::O_MAP, "overhead_4",    false},
    {4.0,  4.5,  LayerType::O_MAP, "overhead_5",    false},
    {4.5,  5.0,  LayerType::O_MAP, "ceiling",       false}
};
```

### Implementation Example

```cpp
// Create barn boundary
std::vector<concord::Point> barn_corners = {
    {0, 0, 0}, {30, 0, 0}, {30, 50, 0}, {0, 50, 0}
};
concord::Polygon barn_boundary(barn_corners);

// Initialize navigation map
NavigationMap barn_map("main_barn", barn_boundary, barn_metadata, 0.1);

// Add structural features
// Add support pillars (extend through multiple layers)
for (double x = 10; x <= 20; x += 10) {
    for (double y = 10; y <= 40; y += 15) {
        concord::Polygon pillar = createSquarePolygon(x, y, 0.3);
        barn_map.addObstacle(pillar, 5.0, "support_pillar");
    }
}

// Add animal pens
std::vector<concord::Polygon> pens = {
    createRectanglePolygon(2, 2, 8, 12),    // Pen 1
    createRectanglePolygon(2, 15, 8, 12),   // Pen 2
    createRectanglePolygon(2, 30, 8, 12),   // Pen 3
    createRectanglePolygon(22, 2, 8, 12),   // Pen 4
    createRectanglePolygon(22, 15, 8, 12),  // Pen 5
    createRectanglePolygon(22, 30, 8, 12)   // Pen 6
};

for (size_t i = 0; i < pens.size(); ++i) {
    barn_map.addPen(pens[i], "pen_" + std::to_string(i+1), 1.2);
}

// Add feeding troughs (obstacles at 0.8m height)
for (const auto& pen : pens) {
    auto trough = createTroughPolygon(pen);  // Helper to position trough in pen
    barn_map.addFeedingTrough(trough, 0.8);
}

// Add equipment storage area with varying height obstacles
concord::Polygon storage_area = createRectanglePolygon(12, 35, 6, 10);
barn_map.addSemanticRegion(storage_area, "equipment_storage", "storage");

// Add hay bales at different heights
barn_map.addObstacle(createRectanglePolygon(13, 36, 1, 2), 1.5, "hay_bale_1");
barn_map.addObstacle(createRectanglePolygon(13, 39, 1, 2), 1.5, "hay_bale_2");
barn_map.addObstacle(createRectanglePolygon(13, 42, 1, 2), 2.5, "hay_bale_stack");

// Add gates that can change state
barn_map.addGate(createLine(10, 0, 10, 5), "main_entrance", true);
barn_map.addGate(createLine(10, 45, 10, 50), "back_entrance", false);

// Simulate sensor updates
// Update from 2D lidar at robot height
std::vector<concord::Point> lidar_points = simulateLidarScan();
concord::Pose robot_pose({15, 25, 0}, {0, 0, 0});
barn_map.updateFromLaserScan(lidar_points, 0.3, robot_pose);

// Update from ceiling-mounted depth camera
concord::Grid<float> depth_image = simulateDepthCamera();
concord::Pose camera_pose({15, 25, 4.5}, {0, -90, 0}); // Looking down
barn_map.updateFromDepthImage(depth_image, camera_pose);

// Generate costmap for navigation
const auto& costmap = barn_map.getCostmap();

// Query specific locations
bool can_traverse = barn_map.isTraversable({15, 20, 0});
double clearance = barn_map.getClearanceHeight({15, 20, 0});
double obstacle_height = barn_map.getObstacleHeight({13, 36, 0});

// Save the map
barn_map.save("./barn_maps/main_barn");
```

### Occupancy Encoding and Inflation

For navigation safety, we use a gradient-based occupancy representation where obstacles are inflated with a probability gradient:

| Value | Meaning | Navigation Behavior |
|-------|---------|-------------------|
| 0 | Completely free space | Preferred path |
| 1-50 | Safe zone | Normal navigation |
| 51-100 | Inflation gradient | Avoid if possible |
| 101-200 | Near obstacle zone | High cost navigation |
| 201-254 | Very close to obstacle | Emergency only |
| 255 | Occupied/Obstacle | Impassable |

The inflation creates a safety buffer around obstacles, accounting for:
- Map inaccuracies (maps aren't millimeter-precise)
- Robot dimensions and shape uncertainty
- Sensor noise and localization errors
- Dynamic safety margins

### Obstacle Inflation Algorithm

```cpp
class ObstacleInflation {
public:
    // Inflate obstacles with distance-based gradient
    static void inflateObstacles(concord::Grid<uint8_t>& grid, 
                                 double robot_radius,
                                 double map_resolution) {
        // Calculate inflation radius in cells
        // Add extra cells based on resolution (coarser = more inflation)
        double safety_factor = std::max(1.0, map_resolution * 10); // 10cm res = 1.0, 1m res = 10x
        double inflation_radius = robot_radius * safety_factor;
        int inflation_cells = std::ceil(inflation_radius / map_resolution);
        
        // Create a copy for reading original values
        auto original = grid;
        
        // Process each cell
        for (size_t r = 0; r < grid.rows(); ++r) {
            for (size_t c = 0; c < grid.cols(); ++c) {
                // Skip if already occupied
                if (original.get_value(r, c) == 255) continue;
                
                // Find distance to nearest obstacle
                double min_distance = std::numeric_limits<double>::max();
                
                // Check neighborhood within inflation radius
                for (int dr = -inflation_cells; dr <= inflation_cells; ++dr) {
                    for (int dc = -inflation_cells; dc <= inflation_cells; ++dc) {
                        int nr = r + dr;
                        int nc = c + dc;
                        
                        // Check bounds
                        if (nr < 0 || nr >= grid.rows() || 
                            nc < 0 || nc >= grid.cols()) continue;
                        
                        // If this neighbor is an obstacle
                        if (original.get_value(nr, nc) == 255) {
                            double distance = std::sqrt(dr*dr + dc*dc) * map_resolution;
                            min_distance = std::min(min_distance, distance);
                        }
                    }
                }
                
                // Apply gradient based on distance
                if (min_distance <= inflation_radius) {
                    uint8_t new_value = calculateInflationValue(
                        min_distance, robot_radius, inflation_radius);
                    
                    // Only increase occupancy, never decrease
                    uint8_t current = grid.get_value(r, c);
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
            return 200 + (54 * (robot_radius - distance) / robot_radius);
        } else if (distance <= inflation_radius) {
            // Within inflation zone - gradient falloff
            double ratio = (inflation_radius - distance) / 
                          (inflation_radius - robot_radius);
            
            // Exponential falloff for smoother gradient
            double exp_ratio = std::exp(-3.0 * (1.0 - ratio));
            return 51 + (149 * exp_ratio);
        } else {
            // Outside inflation - completely safe
            return 0;
        }
    }
    
    // Advanced inflation considering map uncertainty
    static void adaptiveInflation(concord::Grid<uint8_t>& grid,
                                  double robot_radius,
                                  double map_resolution,
                                  double localization_uncertainty) {
        // Adjust inflation based on multiple factors
        double resolution_uncertainty = map_resolution * 0.5; // Half cell uncertainty
        double total_uncertainty = std::sqrt(
            localization_uncertainty * localization_uncertainty +
            resolution_uncertainty * resolution_uncertainty);
        
        // More uncertainty = more inflation
        double adjusted_radius = robot_radius + total_uncertainty;
        
        inflateObstacles(grid, adjusted_radius, map_resolution);
    }
    
    // Create Gaussian-based inflation for smoother gradients
    static void gaussianInflation(concord::Grid<uint8_t>& grid,
                                  double robot_radius,
                                  double map_resolution) {
        double sigma = robot_radius / 2.0; // Standard deviation
        int kernel_size = std::ceil(3 * sigma / map_resolution);
        
        // Pre-compute Gaussian kernel
        std::vector<std::vector<double>> kernel(
            2 * kernel_size + 1, 
            std::vector<double>(2 * kernel_size + 1));
        
        for (int i = -kernel_size; i <= kernel_size; ++i) {
            for (int j = -kernel_size; j <= kernel_size; ++j) {
                double distance = std::sqrt(i*i + j*j) * map_resolution;
                kernel[i + kernel_size][j + kernel_size] = 
                    std::exp(-(distance * distance) / (2 * sigma * sigma));
            }
        }
        
        // Apply convolution with obstacles
        auto original = grid;
        for (size_t r = 0; r < grid.rows(); ++r) {
            for (size_t c = 0; c < grid.cols(); ++c) {
                if (original.get_value(r, c) == 255) continue;
                
                double influence = 0.0;
                for (int i = -kernel_size; i <= kernel_size; ++i) {
                    for (int j = -kernel_size; j <= kernel_size; ++j) {
                        int nr = r + i;
                        int nc = c + j;
                        
                        if (nr >= 0 && nr < grid.rows() && 
                            nc >= 0 && nc < grid.cols()) {
                            if (original.get_value(nr, nc) == 255) {
                                influence += kernel[i + kernel_size][j + kernel_size];
                            }
                        }
                    }
                }
                
                // Convert influence to occupancy value
                if (influence > 0) {
                    uint8_t inflated = std::min(254.0, influence * 200.0);
                    uint8_t current = grid.get_value(r, c);
                    grid.set_value(r, c, std::max(current, inflated));
                }
            }
        }
    }
};

### Dynamic Updates

```cpp
// Handle moving equipment
void updateMovingEquipment(NavigationMap& map, 
                           const TrackedObject& equipment) {
    // Clear previous position
    map.clearDynamicLayers();
    
    // Add at new position
    map.addObstacle(equipment.footprint, 
                   equipment.height, 
                   "dynamic_equipment");
}

// Handle animals (soft obstacles)
void updateAnimalPositions(NavigationMap& map,
                           const std::vector<AnimalTracker>& animals) {
    for (const auto& animal : animals) {
        // Animals are traversable at low speed
        // Use medium occupancy values (not fully blocked)
        for (double h = 0; h < animal.height; h += 0.5) {
            map.updateOccupancy(animal.position, h, 150);  // Soft obstacle
        }
    }
}
```

## Integration with ROS2

### Message Conversion

```cpp
// Convert to ROS2 OccupancyGrid message
nav_msgs::msg::OccupancyGrid toROSOccupancyGrid(
    const NavigationMap& map, 
    double height = 0.0) {
    nav_msgs::msg::OccupancyGrid msg;
    
    // Get appropriate layer
    auto grid = map.getCostmapAtHeight(height);
    
    // Set metadata
    msg.header.frame_id = "map";
    msg.header.stamp = rclcpp::Clock().now();
    msg.info.resolution = grid.resolution();
    msg.info.width = grid.cols();
    msg.info.height = grid.rows();
    
    // Set origin
    auto pose = grid.shift();
    msg.info.origin.position.x = pose.position.x;
    msg.info.origin.position.y = pose.position.y;
    msg.info.origin.position.z = height;
    
    // Convert data (0-255 to ROS convention: 0=free, 100=occupied, -1=unknown)
    msg.data.resize(grid.rows() * grid.cols());
    for (size_t i = 0; i < msg.data.size(); ++i) {
        uint8_t value = grid.data()[i];
        if (value == 0) {
            msg.data[i] = 0;  // Completely free
        } else if (value <= 50) {
            msg.data[i] = value / 2;  // Safe zone (0-25% occupied)
        } else if (value <= 100) {
            msg.data[i] = 25 + (value - 50) / 2;  // Inflation zone (25-50% occupied)
        } else if (value <= 254) {
            msg.data[i] = 50 + (value - 100) * 50 / 154;  // Near obstacle (50-100% occupied)
        } else {
            msg.data[i] = 100;  // Fully occupied
        }
    }
    
    return msg;
}

// Convert to PointCloud2 for 3D visualization
sensor_msgs::msg::PointCloud2 toROSPointCloud(
    const NavigationMap& map) {
    auto points = map.toPointCloud();
    // ... conversion implementation
}
```

### ROS2 Node Example

```cpp
class NavigationMapNode : public rclcpp::Node {
private:
    NavigationMap map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
public:
    NavigationMapNode() : Node("navigation_map_node") {
        // Initialize publishers
        grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).transient_local());
        
        // Subscribe to sensors
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                updateFromLaserScan(msg);
            });
        
        // Publish map periodically
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { publishMap(); });
    }
    
    void updateFromLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Convert and update map
        // ...
    }
    
    void publishMap() {
        auto grid_msg = toROSOccupancyGrid(map_);
        grid_pub_->publish(grid_msg);
    }
};
```

## API Examples

### Basic Usage

```cpp
// Create a simple warehouse map
concord::Polygon warehouse_boundary = /* ... */;
NavigationMetadata metadata = createWarehouseMetadata();
NavigationMap warehouse("warehouse_1", warehouse_boundary, metadata);

// Add static obstacles
warehouse.addObstacle(shelf1_footprint, 2.5, "shelf_1");
warehouse.addObstacle(shelf2_footprint, 2.5, "shelf_2");

// Define semantic zones
warehouse.addSemanticRegion(loading_zone, "loading", "work_area");
warehouse.addSemanticRegion(storage_zone, "storage", "restricted");

// Update from sensors
warehouse.updateFromPointCloud(point_cloud, sensor_pose);

// Query for navigation
if (warehouse.isTraversable(target_position)) {
    auto costmap = warehouse.getCostmap();
    // Plan path using costmap
}
```

### Advanced Queries

```cpp
// Find all traversable positions at specific height
std::vector<concord::Point> findTraversablePositions(
    const NavigationMap& map,
    double min_clearance) {
    
    std::vector<concord::Point> positions;
    auto base_layer = map.getLayer(0);
    
    for (size_t r = 0; r < base_layer.rows(); ++r) {
        for (size_t c = 0; c < base_layer.cols(); ++c) {
            auto point = base_layer.get_point(r, c);
            if (map.getClearanceHeight(point) >= min_clearance) {
                positions.push_back(point);
            }
        }
    }
    return positions;
}

// Check line-of-sight between two points
bool hasLineOfSight(const NavigationMap& map,
                    const concord::Point& from,
                    const concord::Point& to,
                    double height) {
    
    // Bresenham's algorithm on appropriate layer
    auto layer = map.getCostmapAtHeight(height);
    // ... implementation
}

// Find nearest free space
concord::Point findNearestFreeSpace(
    const NavigationMap& map,
    const concord::Point& position) {
    
    // Breadth-first search on costmap
    // ... implementation
}
```

### Persistence

```cpp
// Save map with metadata
void saveMapWithMetadata(const NavigationMap& map,
                         const std::string& path) {
    map.save(path);
    
    // Save additional metadata
    nlohmann::json metadata;
    metadata["created"] = getCurrentTimestamp();
    metadata["robot_config"] = getRobotConfig();
    metadata["sensor_config"] = getSensorConfig();
    
    std::ofstream file(path + "/metadata.json");
    file << metadata.dump(2);
}

// Load and validate map
NavigationMap loadAndValidateMap(const std::string& path) {
    auto map = NavigationMap::load(path);
    
    // Validate against current robot configuration
    if (!validateMapForRobot(map, current_robot)) {
        throw std::runtime_error("Map incompatible with robot");
    }
    
    return map;
}
```

## Future Extensions

### 1. Temporal Mapping
- Track changes over time
- Predict periodic patterns (feeding times, cleaning schedules)
- Long-term map management

### 2. Multi-Robot Coordination
- Shared map updates
- Conflict resolution
- Distributed mapping

### 3. Semantic Understanding
- Object recognition integration
- Activity zones learning
- Behavioral pattern mapping

### 4. Advanced Sensor Fusion
- Multi-modal sensor integration
- Uncertainty propagation
- Active sensing strategies

### 5. Machine Learning Integration
- Traversability learning
- Dynamic obstacle prediction
- Optimal layer configuration learning

### 6. Path Planning Extensions
- Multi-level path planning
- Time-dependent planning
- Energy-optimal paths

### 7. Visualization Tools
- Real-time 3D visualization
- AR overlay for operators
- Web-based monitoring interface

### 8. Standards Compliance
- OpenDRIVE format support
- Lanelet2 integration
- ROS2 Navigation Stack compatibility

## Performance Considerations

### Memory Optimization
- **Sparse Storage**: Only store occupied cells
- **Compression**: Use octree for 3D data
- **Level-of-Detail**: Variable resolution based on distance
- **Lazy Loading**: Load layers on demand

### Computational Efficiency
- **Parallel Updates**: Multi-threaded sensor processing
- **GPU Acceleration**: CUDA for ray-tracing
- **Caching**: Pre-computed costmaps
- **Incremental Updates**: Only update changed regions

### Scalability
- **Tiling**: Divide large maps into tiles
- **Streaming**: Load/unload map sections
- **Distributed Processing**: Multi-machine mapping
- **Cloud Integration**: Offload heavy processing

## Conclusion

The proposed NavigationMap system extends the Zoneout library's capabilities to create sophisticated multi-layered navigation maps suitable for real-world robotic applications. By leveraging the existing Zone, Grid, and Poly infrastructure, we can build:

1. **2.5D representations** that balance detail with efficiency
2. **Gradient-based occupancy** with 0=free, 255=occupied, and smooth inflation zones
3. **Safety through uncertainty** - inflation gradients account for map inaccuracies
4. **Hybrid metric-semantic maps** combining precise grids with meaningful features
5. **Flexible layer management** adapting to environment characteristics at different heights
6. **ROS2 integration** for standard robotics workflows

### Key Innovation: Gradient Inflation

The critical insight is that **maps are never perfectly accurate**. By using gradient inflation around obstacles:
- Robots naturally maintain safe distances from walls
- Navigation remains possible in tight spaces (following lower gradient values)
- Path planners automatically choose safer routes
- System gracefully handles sensor noise and localization errors

The barn navigation example demonstrates practical application in agricultural robotics, where:
- Obstacles exist at multiple heights (feeding troughs, overhead beams)
- Dynamic elements require adaptive inflation (animals as soft obstacles)
- Resolution-aware inflation adjusts safety margins based on map quality
- Semantic zones provide context for navigation behavior

This architecture provides a solid foundation for both current navigation needs and future extensions into more advanced mapping and planning capabilities.
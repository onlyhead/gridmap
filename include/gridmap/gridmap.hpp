#pragma once

// GridMap - Navigation Mapping Library
// Built on top of the Zoneout ecosystem for agricultural and indoor robot navigation
// 
// Core Features:
// - Mandatory dual-layer system (O_MAP + C_MAP at each height level)
// - Multi-layered 2.5D navigation maps
// - WGS84/ENU coordinate handling via concord::Datum
// - PGM standard compliance for occupancy maps
// - Gradient-based obstacle inflation for safety
// - Agricultural/barn navigation features
// - Integration with zoneout Plot->Zone->Poly+Grid architecture

// Core navigation map class
#include "navigation_map.hpp"

// Helper classes for navigation mapping
#include "layer_manager.hpp"
#include "obstacle_inflation.hpp"
#include "costmap_generator.hpp"
#include "update_manager.hpp"

// Export functionality
#include "pgm_exporter.hpp"

// Convenience namespace alias
namespace gm = gridmap;

// Version information
#define GRIDMAP_VERSION_MAJOR 0
#define GRIDMAP_VERSION_MINOR 1
#define GRIDMAP_VERSION_PATCH 0

namespace gridmap {

// Version information
struct Version {
    static constexpr int major = GRIDMAP_VERSION_MAJOR;
    static constexpr int minor = GRIDMAP_VERSION_MINOR;
    static constexpr int patch = GRIDMAP_VERSION_PATCH;
    
    static std::string getString() {
        return std::to_string(major) + "." + 
               std::to_string(minor) + "." + 
               std::to_string(patch);
    }
};

// Library information
inline std::string getLibraryInfo() {
    return "GridMap Navigation Library v" + Version::getString() + 
           "\nBuilt on Zoneout ecosystem for agricultural and indoor robot navigation";
}

// Utility functions for common navigation mapping tasks

// Create a basic dual-layer navigation map for agricultural environments
inline NavigationMap createBarnNavigationMap(
    const std::string& name,
    const concord::Polygon& barn_boundary,
    const concord::Datum& datum,
    double resolution = 0.1) {
    
    NavigationMetadata metadata;
    metadata.robot_height = 1.2;        // Standard agricultural robot height
    metadata.robot_width = 0.8;         // Standard agricultural robot width  
    metadata.inflation_radius = 0.4;    // Extra safety for barn environment
    metadata.max_traversable_slope = 10.0; // Conservative for barn floors
    
    // Set up barn-optimized layers
    metadata.layers = {
        ElevationLayer(0.0, 0.5, LayerType::O_MAP, "ground_omap", true),
        ElevationLayer(0.0, 0.5, LayerType::C_MAP, "ground_cmap", true),
        ElevationLayer(0.5, 1.0, LayerType::O_MAP, "knee_height_omap", true),
        ElevationLayer(0.5, 1.0, LayerType::C_MAP, "knee_height_cmap", true),
        ElevationLayer(1.0, 1.5, LayerType::O_MAP, "robot_height_omap", true),
        ElevationLayer(1.0, 1.5, LayerType::C_MAP, "robot_height_cmap", true),
        ElevationLayer(1.5, 2.5, LayerType::O_MAP, "overhead_omap", true),
        ElevationLayer(1.5, 2.5, LayerType::C_MAP, "overhead_cmap", true),
        ElevationLayer(2.5, 5.0, LayerType::O_MAP, "ceiling_omap", false),
        ElevationLayer(2.5, 5.0, LayerType::C_MAP, "ceiling_cmap", false)
    };
    
    return NavigationMap(name, "barn_navigation", barn_boundary, datum, resolution, metadata);
}

// Create a basic dual-layer navigation map for warehouse environments
inline NavigationMap createWarehouseNavigationMap(
    const std::string& name,
    const concord::Polygon& warehouse_boundary,
    const concord::Datum& datum,
    double resolution = 0.05) {
    
    NavigationMetadata metadata;
    metadata.robot_height = 1.5;        // Taller warehouse robot
    metadata.robot_width = 0.6;         // Narrow for aisle navigation
    metadata.inflation_radius = 0.2;    // Precise navigation required
    metadata.max_traversable_slope = 5.0;  // Flat warehouse floors
    
    // Set up warehouse-optimized layers  
    metadata.layers = {
        ElevationLayer(0.0, 0.3, LayerType::O_MAP, "floor_omap", true),
        ElevationLayer(0.0, 0.3, LayerType::C_MAP, "floor_cmap", true),
        ElevationLayer(0.3, 1.0, LayerType::O_MAP, "low_shelf_omap", true),
        ElevationLayer(0.3, 1.0, LayerType::C_MAP, "low_shelf_cmap", true),
        ElevationLayer(1.0, 2.0, LayerType::O_MAP, "mid_shelf_omap", true),
        ElevationLayer(1.0, 2.0, LayerType::C_MAP, "mid_shelf_cmap", true),
        ElevationLayer(2.0, 3.0, LayerType::O_MAP, "high_shelf_omap", true),
        ElevationLayer(2.0, 3.0, LayerType::C_MAP, "high_shelf_cmap", true),
        ElevationLayer(3.0, 10.0, LayerType::O_MAP, "ceiling_omap", false),
        ElevationLayer(3.0, 10.0, LayerType::C_MAP, "ceiling_cmap", false)
    };
    
    return NavigationMap(name, "warehouse_navigation", warehouse_boundary, datum, resolution, metadata);
}

// Quick PGM export for both O_MAP and C_MAP layers
inline void exportNavigationMapToPGM(const NavigationMap& nav_map,
                                     const std::filesystem::path& output_directory,
                                     bool export_all_layers = false) {
    
    std::filesystem::create_directories(output_directory);
    
    if (export_all_layers) {
        // Export all individual layers
        auto heights = nav_map.getValidHeights();
        std::vector<concord::Grid<uint8_t>> omap_layers, cmap_layers;
        std::vector<std::string> omap_names, cmap_names;
        
        for (double height : heights) {
            // Get layers for this height
            auto omap_layer = nav_map.getObstacleMapAtHeight(height);
            auto cmap_layer = nav_map.getCostmapAtHeight(height);
            
            omap_layers.push_back(omap_layer);
            cmap_layers.push_back(cmap_layer);
            
            omap_names.push_back("omap");
            cmap_names.push_back("cmap");
        }
        
        // Export O_MAP layers
        PgmExporter::exportLayersToPGM(omap_layers, heights, omap_names,
                                       output_directory / "obstacle_maps", true);
        
        // Export C_MAP layers  
        PgmExporter::exportLayersToPGM(cmap_layers, heights, cmap_names,
                                       output_directory / "cost_maps", false);
    }
    
    // Always export composite maps
    auto composite_omap = nav_map.getCompositeObstacleView();
    auto composite_cmap = nav_map.getCompositeCostView();
    
    PgmExporter::exportObstacleMapToPGM(composite_omap, 
                                        output_directory / "composite_obstacle_map.pgm");
    
    PgmExporter::exportCostMapToPGM(composite_cmap,
                                    output_directory / "composite_cost_map.pgm");
}

} // namespace gridmap
#pragma once

#include "concord/concord.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <cstdint>

namespace gridmap {

class PgmExporter {
public:
    // PGM metadata structure compatible with ROS/robotics standards
    struct PgmMetadata {
        std::string image_file;          // PGM filename (e.g., "map.pgm")
        double resolution;               // meters per pixel
        std::vector<double> origin;      // [x, y, theta] - map origin in world coordinates  
        int negate;                      // 0 = standard colors, 1 = inverted
        double occupied_thresh;          // pixels darker than this are occupied
        double free_thresh;              // pixels lighter than this are free
        
        // Default robotics-compatible values
        PgmMetadata(const std::string& filename = "map.pgm", double res = 0.05) 
            : image_file(filename), resolution(res), origin({0.0, 0.0, 0.0}),
              negate(0), occupied_thresh(0.65), free_thresh(0.196) {}
    };
    
    // Export O_MAP (obstacle map) to PGM format with strict standard compliance
    // O_MAP: 0=occupied/black, 255=free/white, 128=unknown/gray -> PGM standard
    static void exportObstacleMapToPGM(const concord::Grid<uint8_t>& omap_grid,
                                       const std::filesystem::path& pgm_path,
                                       const PgmMetadata& metadata = PgmMetadata{},
                                       bool use_binary = true) {
        
        // Validate O_MAP values (should be PGM-compliant already)
        validateOMapValues(omap_grid);
        
        // Write PGM file
        if (use_binary) {
            writePGMBinary(omap_grid, pgm_path, metadata);
        } else {
            writePGMASCII(omap_grid, pgm_path, metadata);
        }
        
        // Write YAML metadata file
        auto yaml_path = pgm_path;
        yaml_path.replace_extension(".yaml");
        writeYAMLMetadata(metadata, yaml_path);
    }
    
    // Export C_MAP (cost map) to PGM format with proper conversion
    // C_MAP: 0=good_travel, 255=impossible -> Convert to PGM standard
    static void exportCostMapToPGM(const concord::Grid<uint8_t>& cmap_grid,
                                   const std::filesystem::path& pgm_path,
                                   const PgmMetadata& metadata = PgmMetadata{},
                                   bool use_binary = true) {
        
        // Convert C_MAP to O_MAP format for PGM export
        // C_MAP: 0=good -> O_MAP: 255=free (invert values)
        auto converted_grid = cmap_grid; // Copy structure
        
        for (size_t r = 0; r < cmap_grid.rows(); ++r) {
            for (size_t c = 0; c < cmap_grid.cols(); ++c) {
                uint8_t cmap_value = cmap_grid(r, c);
                uint8_t omap_value = 255 - cmap_value; // Invert for PGM standard
                converted_grid.set_value(r, c, omap_value);
            }
        }
        
        // Export converted grid
        exportObstacleMapToPGM(converted_grid, pgm_path, metadata, use_binary);
    }
    
    // Export multiple height layers as separate PGM files
    static void exportLayersToPGM(const std::vector<concord::Grid<uint8_t>>& layers,
                                  const std::vector<double>& heights,
                                  const std::vector<std::string>& layer_names,
                                  const std::filesystem::path& output_directory,
                                  bool export_omaps = true,
                                  const PgmMetadata& base_metadata = PgmMetadata{}) {
        
        if (layers.size() != heights.size() || layers.size() != layer_names.size()) {
            throw std::runtime_error("Layers, heights, and names vectors must have same size");
        }
        
        // Create output directory
        std::filesystem::create_directories(output_directory);
        
        for (size_t i = 0; i < layers.size(); ++i) {
            // Create filename with height information
            std::ostringstream filename;
            filename << layer_names[i] << "_h" << std::fixed << std::setprecision(2) 
                     << heights[i] << "m.pgm";
            
            auto pgm_path = output_directory / filename.str();
            
            // Prepare metadata for this layer
            auto layer_metadata = base_metadata;
            layer_metadata.image_file = filename.str();
            layer_metadata.origin[2] = heights[i]; // Set Z coordinate to height
            
            // Export based on layer type
            if (export_omaps) {
                exportObstacleMapToPGM(layers[i], pgm_path, layer_metadata);
            } else {
                exportCostMapToPGM(layers[i], pgm_path, layer_metadata);
            }
        }
    }
    
    // Create composite map from multiple layers (max projection)
    static concord::Grid<uint8_t> createCompositeMap(
        const std::vector<concord::Grid<uint8_t>>& layers,
        bool use_max_projection = true) {
        
        if (layers.empty()) {
            throw std::runtime_error("No layers to composite");
        }
        
        auto composite = layers[0]; // Copy structure from first layer
        
        // Initialize with first layer values
        for (size_t r = 0; r < composite.rows(); ++r) {
            for (size_t c = 0; c < composite.cols(); ++c) {
                composite.set_value(r, c, layers[0](r, c));
            }
        }
        
        // Combine remaining layers
        for (size_t i = 1; i < layers.size(); ++i) {
            for (size_t r = 0; r < composite.rows(); ++r) {
                for (size_t c = 0; c < composite.cols(); ++c) {
                    uint8_t current = composite(r, c);
                    uint8_t layer_val = layers[i](r, c);
                    
                    if (use_max_projection) {
                        // For O_MAP: max value means more occupied (conservative)
                        // For C_MAP: max value means higher cost (conservative)
                        composite.set_value(r, c, std::max(current, layer_val));
                    } else {
                        // Min projection (optimistic)
                        composite.set_value(r, c, std::min(current, layer_val));
                    }
                }
            }
        }
        
        return composite;
    }
    
private:
    // Validate that O_MAP grid contains proper PGM values
    static void validateOMapValues(const concord::Grid<uint8_t>& grid) {
        // Check for common values - no need to validate every cell
        // Just ensure it's a reasonable O_MAP
        bool has_free = false, has_occupied = false, has_unknown = false;
        
        size_t sample_size = std::min(static_cast<size_t>(100), grid.rows() * grid.cols());
        
        for (size_t i = 0; i < sample_size; ++i) {
            size_t r = i % grid.rows();
            size_t c = (i / grid.rows()) % grid.cols();
            uint8_t val = grid(r, c);
            
            if (val == 0) has_occupied = true;
            else if (val == 255) has_free = true;
            else if (val == 128) has_unknown = true;
        }
        
        // Warning if unusual values (but don't fail - might be valid grayscale)
        if (!has_free && !has_occupied) {
            // Grid might be all unknown - this is valid
        }
    }
    
    // Write PGM in binary format (P5)
    static void writePGMBinary(const concord::Grid<uint8_t>& grid,
                               const std::filesystem::path& path,
                               const PgmMetadata& metadata) {
        
        std::ofstream file(path, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file for writing: " + path.string());
        }
        
        // Write PGM P5 header
        file << "P5\n";
        file << "# Created by GridMap library - O_MAP export\n";
        file << "# Resolution: " << metadata.resolution << " m/pixel\n"; 
        file << "# Origin: [" << metadata.origin[0] << ", " << metadata.origin[1] 
             << ", " << metadata.origin[2] << "]\n";
        file << grid.cols() << " " << grid.rows() << "\n";
        file << "255\n";
        
        // Write pixel data row by row
        // PGM format: top-left origin, row-major order
        for (size_t r = 0; r < grid.rows(); ++r) {
            for (size_t c = 0; c < grid.cols(); ++c) {
                uint8_t pixel = grid(r, c);
                file.write(reinterpret_cast<const char*>(&pixel), 1);
            }
        }
        
        file.close();
    }
    
    // Write PGM in ASCII format (P2)
    static void writePGMASCII(const concord::Grid<uint8_t>& grid,
                              const std::filesystem::path& path,
                              const PgmMetadata& metadata) {
        
        std::ofstream file(path);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file for writing: " + path.string());
        }
        
        // Write PGM P2 header
        file << "P2\n";
        file << "# Created by GridMap library - O_MAP export\n";
        file << "# Resolution: " << metadata.resolution << " m/pixel\n";
        file << "# Origin: [" << metadata.origin[0] << ", " << metadata.origin[1] 
             << ", " << metadata.origin[2] << "]\n";
        file << grid.cols() << " " << grid.rows() << "\n";
        file << "255\n";
        
        // Write pixel data with line breaks for readability
        for (size_t r = 0; r < grid.rows(); ++r) {
            for (size_t c = 0; c < grid.cols(); ++c) {
                if (c > 0) file << " ";
                file << static_cast<int>(grid(r, c));
            }
            file << "\n";
        }
        
        file.close();
    }
    
    // Write YAML metadata file compatible with ROS/robotics standards
    static void writeYAMLMetadata(const PgmMetadata& metadata,
                                  const std::filesystem::path& yaml_path) {
        
        std::ofstream yaml_file(yaml_path);
        if (!yaml_file.is_open()) {
            throw std::runtime_error("Cannot open YAML file for writing: " + yaml_path.string());
        }
        
        yaml_file << std::fixed << std::setprecision(6);
        yaml_file << "image: " << metadata.image_file << "\n";
        yaml_file << "resolution: " << metadata.resolution << "\n";
        yaml_file << "origin: [" << metadata.origin[0] << ", " 
                  << metadata.origin[1] << ", " << metadata.origin[2] << "]\n";
        yaml_file << "negate: " << metadata.negate << "\n";
        yaml_file << "occupied_thresh: " << metadata.occupied_thresh << "\n";
        yaml_file << "free_thresh: " << metadata.free_thresh << "\n";
        
        yaml_file.close();
    }
};

} // namespace gridmap
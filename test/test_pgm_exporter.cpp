#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "gridmap/pgm_exporter.hpp"
#include <fstream>
#include <sstream>

// Helper to create a simple test grid
concord::Grid<uint8_t> createTestGrid(size_t rows, size_t cols, double resolution = 0.1) {
    concord::Pose pose({0, 0, 0}, {0, 0, 0});
    concord::Grid<uint8_t> grid(rows, cols, resolution, true, pose, false);
    
    // Fill with some test pattern
    for (size_t r = 0; r < rows; ++r) {
        for (size_t c = 0; c < cols; ++c) {
            if (r < rows / 4 || r >= 3 * rows / 4 || c < cols / 4 || c >= 3 * cols / 4) {
                grid.set_value(r, c, 0);   // Occupied (black) border
            } else if (r == rows / 2 && c == cols / 2) {
                grid.set_value(r, c, 128); // Unknown (gray) center
            } else {
                grid.set_value(r, c, 255); // Free (white) interior
            }
        }
    }
    
    return grid;
}

TEST_CASE("PgmExporter Basic Functionality") {
    SUBCASE("PGM metadata creation") {
        gridmap::PgmExporter::PgmMetadata metadata("test.pgm", 0.05);
        
        CHECK(metadata.image_file == "test.pgm");
        CHECK(metadata.resolution == doctest::Approx(0.05));
        CHECK(metadata.negate == 0);
        CHECK(metadata.occupied_thresh == doctest::Approx(0.65));
        CHECK(metadata.free_thresh == doctest::Approx(0.196));
        CHECK(metadata.origin.size() == 3);
    }
}

TEST_CASE("PgmExporter O_MAP Export") {
    auto test_grid = createTestGrid(20, 30, 0.1);
    std::filesystem::path temp_dir = "./test_pgm_output";
    std::filesystem::create_directories(temp_dir);
    
    SUBCASE("Binary PGM export") {
        auto pgm_path = temp_dir / "test_binary.pgm";
        gridmap::PgmExporter::PgmMetadata metadata("test_binary.pgm", 0.1);
        
        CHECK_NOTHROW(gridmap::PgmExporter::exportObstacleMapToPGM(test_grid, pgm_path, metadata, true));
        CHECK(std::filesystem::exists(pgm_path));
        CHECK(std::filesystem::exists(temp_dir / "test_binary.yaml"));
        
        // Verify PGM file format
        std::ifstream pgm_file(pgm_path, std::ios::binary);
        REQUIRE(pgm_file.is_open());
        
        std::string magic;
        pgm_file >> magic;
        CHECK(magic == "P5"); // Binary format
        
        pgm_file.close();
    }
    
    SUBCASE("ASCII PGM export") {
        auto pgm_path = temp_dir / "test_ascii.pgm";
        gridmap::PgmExporter::PgmMetadata metadata("test_ascii.pgm", 0.1);
        
        CHECK_NOTHROW(gridmap::PgmExporter::exportObstacleMapToPGM(test_grid, pgm_path, metadata, false));
        CHECK(std::filesystem::exists(pgm_path));
        
        // Verify PGM file format
        std::ifstream pgm_file(pgm_path);
        REQUIRE(pgm_file.is_open());
        
        std::string magic;
        pgm_file >> magic;
        CHECK(magic == "P2"); // ASCII format
        
        pgm_file.close();
    }
    
    SUBCASE("YAML metadata file") {
        auto pgm_path = temp_dir / "test_metadata.pgm";
        auto yaml_path = temp_dir / "test_metadata.yaml";
        
        gridmap::PgmExporter::PgmMetadata metadata("test_metadata.pgm", 0.05);
        metadata.origin = {52.0, 4.0, 1.5};
        
        gridmap::PgmExporter::exportObstacleMapToPGM(test_grid, pgm_path, metadata, true);
        
        CHECK(std::filesystem::exists(yaml_path));
        
        // Verify YAML content
        std::ifstream yaml_file(yaml_path);
        REQUIRE(yaml_file.is_open());
        
        std::string yaml_content((std::istreambuf_iterator<char>(yaml_file)),
                                std::istreambuf_iterator<char>());
        
        CHECK(yaml_content.find("image: test_metadata.pgm") != std::string::npos);
        CHECK(yaml_content.find("resolution: 0.050000") != std::string::npos);
        CHECK(yaml_content.find("origin: [52.000000, 4.000000, 1.500000]") != std::string::npos);
        CHECK(yaml_content.find("negate: 0") != std::string::npos);
        
        yaml_file.close();
    }
    
    // Clean up
    std::filesystem::remove_all(temp_dir);
}

TEST_CASE("PgmExporter C_MAP Export") {
    // Create a C_MAP test grid (inverted values from O_MAP)
    auto cmap_grid = createTestGrid(15, 25, 0.1);
    
    // Invert values to simulate C_MAP (0=good, 255=bad)
    for (size_t r = 0; r < cmap_grid.rows(); ++r) {
        for (size_t c = 0; c < cmap_grid.cols(); ++c) {
            uint8_t val = cmap_grid(r, c);
            cmap_grid.set_value(r, c, 255 - val); // Invert
        }
    }
    
    std::filesystem::path temp_dir = "./test_cmap_output";
    std::filesystem::create_directories(temp_dir);
    
    SUBCASE("C_MAP to PGM conversion") {
        auto pgm_path = temp_dir / "test_cmap.pgm";
        gridmap::PgmExporter::PgmMetadata metadata("test_cmap.pgm", 0.1);
        
        CHECK_NOTHROW(gridmap::PgmExporter::exportCostMapToPGM(cmap_grid, pgm_path, metadata, true));
        CHECK(std::filesystem::exists(pgm_path));
        
        // Verify that the exported file follows PGM standard (should be inverted again)
        std::ifstream pgm_file(pgm_path, std::ios::binary);
        REQUIRE(pgm_file.is_open());
        
        std::string line;
        // Skip header lines
        std::getline(pgm_file, line); // P5
        while (std::getline(pgm_file, line) && line[0] == '#') {} // Comments
        std::getline(pgm_file, line); // 255
        
        // Read some pixel data
        char pixel;
        pgm_file.read(&pixel, 1);
        
        // Should be valid grayscale value
        uint8_t pixel_val = static_cast<uint8_t>(pixel);
        CHECK(pixel_val <= 255);
        
        pgm_file.close();
    }
    
    // Clean up
    std::filesystem::remove_all(temp_dir);
}

TEST_CASE("PgmExporter Multi-Layer Export") {
    // Create multiple test layers
    std::vector<concord::Grid<uint8_t>> layers;
    std::vector<double> heights = {0.0, 0.5, 1.0, 1.5};
    std::vector<std::string> names = {"ground", "knee", "waist", "head"};
    
    for (size_t i = 0; i < heights.size(); ++i) {
        auto layer = createTestGrid(10, 15, 0.1);
        
        // Make each layer slightly different
        for (size_t r = 0; r < layer.rows(); ++r) {
            for (size_t c = 0; c < layer.cols(); ++c) {
                uint8_t val = layer(r, c);
                if (val == 255 && (r + c + i) % 3 == 0) {
                    layer.set_value(r, c, 128); // Add some unknown areas
                }
            }
        }
        
        layers.push_back(layer);
    }
    
    std::filesystem::path temp_dir = "./test_multilayer_output";
    
    SUBCASE("Export multiple layers") {
        gridmap::PgmExporter::PgmMetadata base_metadata("base.pgm", 0.1);
        
        CHECK_NOTHROW(gridmap::PgmExporter::exportLayersToPGM(
            layers, heights, names, temp_dir, true, base_metadata));
        
        // Check that files were created
        for (size_t i = 0; i < heights.size(); ++i) {
            std::ostringstream expected_filename;
            expected_filename << names[i] << "_h" << std::fixed << std::setprecision(2) 
                             << heights[i] << "m.pgm";
            
            auto expected_path = temp_dir / expected_filename.str();
            CHECK(std::filesystem::exists(expected_path));
            
            // Also check for YAML file
            auto yaml_path = temp_dir / (expected_filename.str().substr(0, expected_filename.str().length() - 4) + ".yaml");
            CHECK(std::filesystem::exists(yaml_path));
        }
    }
    
    SUBCASE("Mismatched input sizes") {
        std::vector<double> wrong_heights = {0.0, 0.5}; // Different size
        
        CHECK_THROWS(gridmap::PgmExporter::exportLayersToPGM(
            layers, wrong_heights, names, temp_dir, true));
    }
    
    // Clean up
    std::filesystem::remove_all(temp_dir);
}

TEST_CASE("PgmExporter Composite Maps") {
    std::vector<concord::Grid<uint8_t>> layers;
    
    // Create test layers with different obstacle patterns
    for (int i = 0; i < 3; ++i) {
        auto layer = createTestGrid(10, 10, 0.1);
        
        // Add layer-specific obstacles
        size_t obstacle_r = 2 + i;
        size_t obstacle_c = 3 + i;
        if (obstacle_r < layer.rows() && obstacle_c < layer.cols()) {
            layer.set_value(obstacle_r, obstacle_c, 0); // Add obstacle
        }
        
        layers.push_back(layer);
    }
    
    SUBCASE("Max projection composite") {
        auto composite = gridmap::PgmExporter::createCompositeMap(layers, true);
        
        CHECK(composite.rows() == layers[0].rows());
        CHECK(composite.cols() == layers[0].cols());
        
        // Verify max projection behavior
        // At positions where any layer has an obstacle, composite should have obstacle
        for (size_t r = 0; r < composite.rows(); ++r) {
            for (size_t c = 0; c < composite.cols(); ++c) {
                uint8_t composite_val = composite(r, c);
                
                uint8_t max_val = 0;
                for (const auto& layer : layers) {
                    max_val = std::max(max_val, layer(r, c));
                }
                
                CHECK(composite_val == max_val);
            }
        }
    }
    
    SUBCASE("Min projection composite") {
        auto composite = gridmap::PgmExporter::createCompositeMap(layers, false);
        
        // Verify min projection behavior
        for (size_t r = 0; r < composite.rows(); ++r) {
            for (size_t c = 0; c < composite.cols(); ++c) {
                uint8_t composite_val = composite(r, c);
                
                uint8_t min_val = 255;
                for (const auto& layer : layers) {
                    min_val = std::min(min_val, layer(r, c));
                }
                
                CHECK(composite_val == min_val);
            }
        }
    }
    
    SUBCASE("Empty layer list") {
        std::vector<concord::Grid<uint8_t>> empty_layers;
        CHECK_THROWS(gridmap::PgmExporter::createCompositeMap(empty_layers));
    }
}
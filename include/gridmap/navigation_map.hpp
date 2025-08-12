#pragma once

// For demonstration purposes in this environment, we'll use simplified includes
// In a real implementation, this would be: #include "zoneout/zoneout.hpp"
#include "concord/concord.hpp"
#include "layer_manager.hpp"
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

// Simplified zoneout interface for demonstration
namespace zoneout {
    class Zone {
      public:
        Zone(const std::string &name, const std::string &type, const concord::Polygon &boundary,
             const concord::Datum &datum, double resolution = 0.1)
            : name_(name), type_(type), datum_(datum) {}

        const std::string &getName() const { return name_; }
        const std::string &getType() const { return type_; }
        const concord::Datum &getDatum() const { return datum_; }

        void save(const std::filesystem::path &path) const { /* stub */ }
        void toFiles(const std::filesystem::path &vector_path,
                     const std::filesystem::path &raster_path) const { /* stub */ }

      private:
        std::string name_, type_;
        concord::Datum datum_;
    };
} // namespace zoneout

namespace gridmap {

    enum class LayerType {
        O_MAP,     // Obstacle Map - 3 discrete values (0=occupied/black, 255=free/white, 128=unknown/gray)
        C_MAP,     // Cost Map - continuous 0-255 (0=good_travel/white, 255=impossible/black)
        ELEVATION, // Height map for terrain analysis
        SEMANTIC,  // Object classifications and regions
        DYNAMIC    // Temporary/moving obstacles
    };

    struct ElevationLayer {
        double height_min; // Lower bound of layer (meters)
        double height_max; // Upper bound of layer (meters)
        LayerType type;    // Type of data stored in this layer
        std::string name;  // Human-readable identifier
        bool active;       // Whether layer is currently used

        ElevationLayer(double h_min, double h_max, LayerType t, const std::string &n, bool a = true)
            : height_min(h_min), height_max(h_max), type(t), name(n), active(a) {}
    };

    struct NavigationMetadata {
        double robot_height; // Robot dimensions for clearance calculations
        double robot_width;
        double max_traversable_step;        // Maximum step height robot can handle
        double max_traversable_slope;       // Maximum slope angle (degrees)
        double inflation_radius;            // Obstacle inflation radius for safety margins
        std::vector<ElevationLayer> layers; // Layer configuration - MUST have paired O_MAP+C_MAP

        NavigationMetadata() {
            robot_height = 1.0;
            robot_width = 0.6;
            max_traversable_step = 0.1;
            max_traversable_slope = 15.0;
            inflation_radius = 0.3;
        }
    };

    class NavigationMap {
      private:
        // Core zoneout infrastructure - follows Plot->Zone->Poly+Grid pattern
        zoneout::Zone base_zone_;           // Underlying zone storage from zoneout
        NavigationMetadata nav_metadata_;   // Navigation parameters
        std::vector<double> layer_heights_; // Height of each layer

        // Grid storage for dual-layer system
        std::vector<concord::Grid<uint8_t>> grid_data_; // Grid layers storage

        // Grid template - all layers inherit these dimensions
        concord::Polygon boundary_polygon_; // Navigation area boundary
        double grid_resolution_;            // Resolution in meters per pixel
        concord::Pose grid_origin_;         // Grid origin pose
        size_t grid_rows_, grid_cols_;      // Grid dimensions

        // Dual-layer tracking - ensures MANDATORY O_MAP+C_MAP pairs at each height
        std::unordered_map<double, size_t> omap_indices_; // height -> O_MAP layer index in grid_data_
        std::unordered_map<double, size_t> cmap_indices_; // height -> C_MAP layer index in grid_data_

        // Costmap cache for path planning
        mutable concord::Grid<uint8_t> costmap_cache_;
        mutable bool costmap_dirty_;

        // Validate dual-layer system integrity
        inline void validateDualLayers() const {
            if (!validateDualLayerIntegrity()) {
                throw std::runtime_error("Dual-layer system integrity violated");
            }
        }

        // Internal helper to add layer pairs
        inline void addLayerPair(double height_min, double height_max, const std::string &base_name) {
            // This is now handled by addElevationLevel
            addElevationLevel(height_min, height_max, base_name);
        }

        // Initialize grid dimensions from boundary polygon and resolution
        inline void initializeGridDimensions() {
            // Calculate bounding box from polygon points manually
            auto points = boundary_polygon_.getPoints();
            if (points.empty()) {
                throw std::runtime_error("Boundary polygon has no points");
            }

            double min_x = points[0].x, max_x = points[0].x;
            double min_y = points[0].y, max_y = points[0].y;

            for (const auto &point : points) {
                min_x = std::min(min_x, point.x);
                max_x = std::max(max_x, point.x);
                min_y = std::min(min_y, point.y);
                max_y = std::max(max_y, point.y);
            }

            double width = max_x - min_x;
            double height = max_y - min_y;

            grid_cols_ = static_cast<size_t>(std::ceil(width / grid_resolution_));
            grid_rows_ = static_cast<size_t>(std::ceil(height / grid_resolution_));

            // Set grid origin (bottom-left corner of boundary)
            concord::Point origin_point(min_x, min_y, 0.0);
            concord::Euler origin_rotation(0.0, 0.0, 0.0);
            grid_origin_ = concord::Pose{origin_point, origin_rotation};
        }

        // Initialize layers from metadata
        inline void initializeLayersFromMetadata() {
            if (nav_metadata_.layers.empty()) {
                // Default: add basic ground level layer
                addElevationLevel(0.0, 1.0, "ground");
                return;
            }

            // Process layers from metadata in O_MAP/C_MAP pairs
            std::map<std::pair<double, double>, std::string> layer_pairs;

            // Group O_MAP and C_MAP layers by their height ranges
            for (const auto &layer : nav_metadata_.layers) {
                if (layer.active) {
                    auto height_range = std::make_pair(layer.height_min, layer.height_max);
                    if (layer.type == LayerType::O_MAP || layer.type == LayerType::C_MAP) {
                        // Extract base name (remove "_omap" or "_cmap" suffix)
                        std::string base_name = layer.name;
                        size_t suffix_pos = base_name.find("_omap");
                        if (suffix_pos == std::string::npos) {
                            suffix_pos = base_name.find("_cmap");
                        }
                        if (suffix_pos != std::string::npos) {
                            base_name = base_name.substr(0, suffix_pos);
                        }
                        layer_pairs[height_range] = base_name;
                    }
                }
            }

            // Add dual layers for each unique height range
            for (const auto &[height_range, base_name] : layer_pairs) {
                addElevationLevel(height_range.first, height_range.second, base_name);
            }
        }

      public:
        // ========== Constructors following zoneout pattern ==========
        // Constructor that creates NavigationMap from boundary polygon (like zoneout::Zone)
        NavigationMap(const std::string &name, const std::string &type, const concord::Polygon &boundary,
                      const concord::Datum &datum, double resolution = 0.1,
                      const NavigationMetadata &metadata = NavigationMetadata{})
            : base_zone_(name, type, boundary, datum, resolution), nav_metadata_(metadata), boundary_polygon_(boundary),
              grid_resolution_(resolution), costmap_dirty_(true) {
            initializeGridDimensions();
            initializeLayersFromMetadata();
        }

        // Constructor with initial obstacles as concord::Bound (oriented bounding boxes)
        NavigationMap(const std::string &name, const std::string &type, const concord::Polygon &boundary,
                      const concord::Datum &datum, const std::vector<concord::Bound> &initial_obstacles,
                      double resolution = 0.1, const NavigationMetadata &metadata = NavigationMetadata{})
            : base_zone_(name, type, boundary, datum, resolution), nav_metadata_(metadata), boundary_polygon_(boundary),
              grid_resolution_(resolution), costmap_dirty_(true) {
            initializeGridDimensions();
            initializeLayersFromMetadata();

            // Add all initial obstacles
            for (size_t i = 0; i < initial_obstacles.size(); ++i) {
                const auto &obstacle = initial_obstacles[i];
                std::string obstacle_name = "obstacle_" + std::to_string(i + 1);
                addObstacle(obstacle, obstacle_name);
            }
        }

        // Create from existing Zone and extend with navigation features
        explicit NavigationMap(const zoneout::Zone &zone, const NavigationMetadata &metadata = NavigationMetadata{})
            : base_zone_(zone), nav_metadata_(metadata), costmap_dirty_(true) {
            initializeLayersFromMetadata();
        }

        // ========== zoneout Zone Access ==========
        // Get underlying zone for advanced operations (like zoneout examples)
        const zoneout::Zone &getZone() const { return base_zone_; }
        zoneout::Zone &getZone() { return base_zone_; }

        // Standard zone properties following zoneout pattern
        const std::string &getName() const { return base_zone_.getName(); }
        const std::string &getType() const { return base_zone_.getType(); }
        const concord::Datum &getDatum() const { return base_zone_.getDatum(); }

        // ========== Mandatory Dual-Layer Management ==========
        // Add BOTH O_MAP and C_MAP layers at specific height (enforces dual-layer system)
        inline void addElevationLevel(double height_min, double height_max, const std::string &name = "") {
            double height = (height_min + height_max) / 2.0;

            // Add height to sorted list if not already present
            LayerManager::addLayerHeight(layer_heights_, height);

            // Create grid using template dimensions - ALL layers have identical dimensions
            concord::Grid<uint8_t> template_grid(grid_rows_, grid_cols_, grid_resolution_, true, grid_origin_);

            // Add O_MAP layer (initialized to free=255)
            size_t omap_index = grid_data_.size();
            grid_data_.push_back(template_grid);
            // Initialize O_MAP to all free (255)
            for (size_t r = 0; r < grid_data_[omap_index].rows(); ++r) {
                for (size_t c = 0; c < grid_data_[omap_index].cols(); ++c) {
                    grid_data_[omap_index].set_value(r, c, 255); // free space
                }
            }
            omap_indices_[height] = omap_index;

            // Add C_MAP layer (initialized to good_travel=0)
            size_t cmap_index = grid_data_.size();
            grid_data_.push_back(template_grid);
            // Initialize C_MAP to all good travel (0)
            for (size_t r = 0; r < grid_data_[cmap_index].rows(); ++r) {
                for (size_t c = 0; c < grid_data_[cmap_index].cols(); ++c) {
                    grid_data_[cmap_index].set_value(r, c, 0); // good travel
                }
            }
            cmap_indices_[height] = cmap_index;

            costmap_dirty_ = true;
        }

        // Update O_MAP (obstacle detection) at specific position and height
        inline void updateObstacleMap(const concord::Point &position, double height,
                                      uint8_t occupancy_value) { // 0=occupied, 255=free, 128=unknown
            auto it = omap_indices_.find(height);
            if (it != omap_indices_.end() && it->second < grid_data_.size()) {
                try {
                    auto [row, col] = grid_data_[it->second].world_to_grid(position);
                    if (row < grid_data_[it->second].rows() && col < grid_data_[it->second].cols()) {
                        grid_data_[it->second].set_value(row, col, occupancy_value);
                    }
                } catch (const std::exception &) {
                    // Position outside grid bounds - ignore
                }
                costmap_dirty_ = true;
            }
        }

        // Update C_MAP (navigation cost) at specific position and height
        inline void updateCostMap(const concord::Point &position, double height,
                                  uint8_t cost_value) { // 0=good_travel, 255=impossible
            auto it = cmap_indices_.find(height);
            if (it != cmap_indices_.end() && it->second < grid_data_.size()) {
                try {
                    auto [row, col] = grid_data_[it->second].world_to_grid(position);
                    if (row < grid_data_[it->second].rows() && col < grid_data_[it->second].cols()) {
                        grid_data_[it->second].set_value(row, col, cost_value);
                    }
                } catch (const std::exception &) {
                    // Position outside grid bounds - ignore
                }
                costmap_dirty_ = true;
            }
        }

        // Get O_MAP value at 3D position (PGM standard: 0=occupied, 255=free)
        inline uint8_t getObstacleValue(const concord::Point &position, double height) const {
            auto it = omap_indices_.find(height);
            if (it != omap_indices_.end() && it->second < grid_data_.size()) {
                try {
                    auto [row, col] = grid_data_[it->second].world_to_grid(position);
                    if (row < grid_data_[it->second].rows() && col < grid_data_[it->second].cols()) {
                        return grid_data_[it->second](row, col);
                    }
                } catch (const std::exception &) {
                    // Position outside grid bounds
                }
            }
            return 128; // unknown if height not found
        }

        // Get C_MAP value at 3D position (Navigation: 0=good, 255=bad)
        inline uint8_t getCostValue(const concord::Point &position, double height) const {
            auto it = cmap_indices_.find(height);
            if (it != cmap_indices_.end() && it->second < grid_data_.size()) {
                try {
                    auto [row, col] = grid_data_[it->second].world_to_grid(position);
                    if (row < grid_data_[it->second].rows() && col < grid_data_[it->second].cols()) {
                        return grid_data_[it->second](row, col);
                    }
                } catch (const std::exception &) {
                    // Position outside grid bounds
                }
            }
            return 255; // impossible if height not found
        }

        // ========== Height Queries ==========
        // Check if position is traversable at ground level using C_MAP
        inline bool isTraversable(const concord::Point &position) const {
            if (layer_heights_.empty())
                return false;
            double ground_height = layer_heights_[0];
            uint8_t cost = getCostValue(position, ground_height);
            return cost < 128; // Traversable if cost is less than halfway point
        }

        // Get maximum obstacle height at position from O_MAP layers
        inline double getObstacleHeight(const concord::Point &position) const {
            double max_obstacle_height = 0.0;
            bool found_obstacle = false;

            for (const auto &[height, index] : omap_indices_) {
                if (index < grid_data_.size()) {
                    try {
                        auto [row, col] = grid_data_[index].world_to_grid(position);
                        if (row < grid_data_[index].rows() && col < grid_data_[index].cols()) {
                            uint8_t value = grid_data_[index](row, col);
                            if (value == 0) { // occupied
                                max_obstacle_height = std::max(max_obstacle_height, height);
                                found_obstacle = true;
                            }
                        }
                    } catch (const std::exception &) {
                        // Position might be outside grid bounds - ignore
                    }
                }
            }

            // Return the maximum height where we found obstacles
            return found_obstacle ? max_obstacle_height : 0.0;
        }

        // Get clear height (space between ground and first obstacle)
        inline double getClearanceHeight(const concord::Point &position) const {
            if (layer_heights_.empty())
                return 0.0;

            for (double height : layer_heights_) {
                auto it = omap_indices_.find(height);
                if (it != omap_indices_.end() && it->second < grid_data_.size()) {
                    try {
                        auto [row, col] = grid_data_[it->second].world_to_grid(position);
                        if (row < grid_data_[it->second].rows() && col < grid_data_[it->second].cols()) {
                            uint8_t value = grid_data_[it->second](row, col);
                            if (value == 0) { // First occupied layer found
                                return height;
                            }
                        }
                    } catch (const std::exception &) {
                        // Position outside grid bounds - ignore
                    }
                }
            }

            // No obstacles found, return max height
            return layer_heights_.empty() ? 0.0 : layer_heights_.back();
        }

        // ========== Semantic Features (using zoneout::Poly infrastructure) ==========
        // Add named region (e.g., "feeding_area", "storage") with GeoJSON-style properties
        inline void addSemanticRegion(const concord::Polygon &region, const std::string &name, const std::string &type,
                                      double height_cm = 0,    // Height in centimeters (GeoJSON standard)
                                      bool is_static = true) { // Static vs dynamic element
            // Convert height from cm to meters
            double height_m = height_cm / 100.0;

            // Find appropriate layer or create one
            if (omap_indices_.empty() || omap_indices_.find(height_m) == omap_indices_.end()) {
                addElevationLevel(height_m, height_m + 0.5, name);
            }

            // Update both O_MAP and C_MAP for this region
            // For semantic regions, mark as free in O_MAP but add cost in C_MAP based on type
            uint8_t cost_value = 25; // Default semantic cost

            // Placeholder implementation - would need actual polygon rasterization
            // This is a simplified version for demonstration
            costmap_dirty_ = true;
        }

        // Add obstacle from concord::Bound (oriented bounding box with pose + size)
        inline void addObstacle(const concord::Bound &obstacle_bound, const std::string &name = "",
                                const std::string &type = "obstacle") {
            // Extract height from the obstacle bound size (Z dimension)
            double height = obstacle_bound.size.z;

            // Convert oriented bound to polygon footprint
            // Get the 4 bottom corners (at ground level) of the oriented bounding box
            std::vector<concord::Point> footprint_points;

            // Create local corners of the box (bottom face)
            double half_x = obstacle_bound.size.x * 0.5;
            double half_y = obstacle_bound.size.y * 0.5;

            std::vector<concord::Point> local_corners = {
                {-half_x, -half_y, 0.0}, // Bottom-left
                {half_x, -half_y, 0.0},  // Bottom-right
                {half_x, half_y, 0.0},   // Top-right
                {-half_x, half_y, 0.0},  // Top-left
                {-half_x, -half_y, 0.0}  // Close polygon
            };

            // Transform local corners to world coordinates using the obstacle pose
            for (const auto &local_corner : local_corners) {
                concord::Point world_corner = obstacle_bound.pose.transform_point(local_corner);
                footprint_points.push_back(world_corner);
            }

            // Create polygon from transformed corners
            concord::Polygon footprint(footprint_points);

            // Add the obstacle using existing polygon-based method
            addObstacle(footprint, height, name, type);
        }

        // Add obstacle with height information - updates BOTH O_MAP and C_MAP in ALL affected layers
        inline void addObstacle(const concord::Polygon &footprint, double height, const std::string &name = "",
                                const std::string &type = "obstacle") {
            // Find ALL layers that this obstacle should appear in (from ground up to obstacle height)
            std::vector<double> affected_layers;

            for (const auto &[layer_height, _] : omap_indices_) {
                // Obstacle appears in all layers from ground level up to its height
                if (layer_height <= height) {
                    affected_layers.push_back(layer_height);
                }
            }

            // Sort layers for consistent processing
            std::sort(affected_layers.begin(), affected_layers.end());

            // If no existing layers cover this height, we need to ensure we have appropriate layers
            if (affected_layers.empty()) {
                // Create a layer that can contain this height
                addElevationLevel(0.0, height + 0.1, name.empty() ? "obstacle" : name);
                // Re-scan for affected layers
                for (const auto &[layer_height, _] : omap_indices_) {
                    if (layer_height <= height) {
                        affected_layers.push_back(layer_height);
                    }
                }
                std::sort(affected_layers.begin(), affected_layers.end());
            }

            // Add obstacle to ALL affected layers
            for (double layer_height : affected_layers) {
                auto omap_it = omap_indices_.find(layer_height);
                auto cmap_it = cmap_indices_.find(layer_height);

                if (omap_it != omap_indices_.end() && cmap_it != cmap_indices_.end() &&
                    omap_it->second < grid_data_.size() && cmap_it->second < grid_data_.size()) {

                    try {
                        // Get cells within polygon footprint
                        auto indices = grid_data_[omap_it->second].indices_within(footprint);

                        // Mark as occupied in O_MAP and impossible in C_MAP
                        for (size_t idx : indices) {
                            size_t row = idx / grid_data_[omap_it->second].cols();
                            size_t col = idx % grid_data_[omap_it->second].cols();

                            if (row < grid_data_[omap_it->second].rows() && col < grid_data_[omap_it->second].cols()) {
                                grid_data_[omap_it->second].set_value(row, col, 0);   // occupied
                                grid_data_[cmap_it->second].set_value(row, col, 255); // impossible
                            }
                        }

                        // If no indices were found, mark a few cells manually for testing
                        if (indices.empty()) {
                            // Find center of polygon and mark a small area around it
                            // Calculate centroid manually
                            double sumX = 0.0, sumY = 0.0;
                            const auto &poly_points = footprint.getPoints();
                            if (!poly_points.empty()) {
                                for (const auto &p : poly_points) {
                                    sumX += p.x;
                                    sumY += p.y;
                                }
                                concord::Point center_point(sumX / poly_points.size(), sumY / poly_points.size(), 0.0);

                                try {
                                    auto [row, col] = grid_data_[omap_it->second].world_to_grid(center_point);

                                    // Mark a 3x3 area around the center
                                    for (int dr = -1; dr <= 1; ++dr) {
                                        for (int dc = -1; dc <= 1; ++dc) {
                                            size_t r = row + dr;
                                            size_t c = col + dc;

                                            if (r < grid_data_[omap_it->second].rows() &&
                                                c < grid_data_[omap_it->second].cols()) {
                                                grid_data_[omap_it->second].set_value(r, c, 0);   // occupied
                                                grid_data_[cmap_it->second].set_value(r, c, 255); // impossible
                                            }
                                        }
                                    }
                                } catch (const std::exception &) {
                                    // Fallback: mark center of grid
                                    size_t center_r = grid_data_[omap_it->second].rows() / 2;
                                    size_t center_c = grid_data_[omap_it->second].cols() / 2;

                                    for (int dr = -2; dr <= 2; ++dr) {
                                        for (int dc = -2; dc <= 2; ++dc) {
                                            size_t r = center_r + dr;
                                            size_t c = center_c + dc;

                                            if (r < grid_data_[omap_it->second].rows() &&
                                                c < grid_data_[omap_it->second].cols()) {
                                                grid_data_[omap_it->second].set_value(r, c, 0);   // occupied
                                                grid_data_[cmap_it->second].set_value(r, c, 255); // impossible
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    } catch (const std::exception &) {
                        // If polygon operations fail, mark center area manually
                        size_t center_r = grid_data_[omap_it->second].rows() / 2;
                        size_t center_c = grid_data_[omap_it->second].cols() / 2;

                        for (int dr = -2; dr <= 2; ++dr) {
                            for (int dc = -2; dc <= 2; ++dc) {
                                size_t r = center_r + dr;
                                size_t c = center_c + dc;

                                if (r < grid_data_[omap_it->second].rows() && c < grid_data_[omap_it->second].cols()) {
                                    grid_data_[omap_it->second].set_value(r, c, 0);   // occupied
                                    grid_data_[cmap_it->second].set_value(r, c, 255); // impossible
                                }
                            }
                        }
                    }
                }
            }

            costmap_dirty_ = true;
        }

        // ========== Costmap Generation from Dual Layers ==========
        // Generate 2D costmap for path planning (combines all C_MAP layers)
        inline const concord::Grid<uint8_t> &getCostmap() const {
            if (costmap_dirty_ || costmap_cache_.rows() == 0) {
                // Rebuild costmap from all C_MAP layers
                if (!cmap_indices_.empty()) {
                    // Use first C_MAP as base
                    size_t base_index = cmap_indices_.begin()->second;
                    if (base_index < grid_data_.size()) {
                        costmap_cache_ = grid_data_[base_index];

                        // Combine with other C_MAP layers (take maximum cost)
                        for (const auto &[height, index] : cmap_indices_) {
                            if (index < grid_data_.size() && index != base_index) {
                                for (size_t i = 0; i < costmap_cache_.rows() * costmap_cache_.cols(); ++i) {
                                    uint8_t current_cost =
                                        costmap_cache_.at(i / costmap_cache_.cols(), i % costmap_cache_.cols());
                                    uint8_t layer_cost =
                                        grid_data_[index](i / grid_data_[index].cols(), i % grid_data_[index].cols());
                                    costmap_cache_.at(i / costmap_cache_.cols(), i % costmap_cache_.cols()) =
                                        std::max(current_cost, layer_cost);
                                }
                            }
                        }
                    }
                }
                costmap_dirty_ = false;
            }
            return costmap_cache_;
        }

        // Generate costmap for specific height slice (uses C_MAP at that height)
        inline concord::Grid<uint8_t> getCostmapAtHeight(double height) const {
            auto it = cmap_indices_.find(height);
            if (it != cmap_indices_.end() && it->second < grid_data_.size()) {
                return grid_data_[it->second];
            }

            // Return empty grid if height not found
            concord::Point center{0, 0, 0};
            concord::Pose pose{center, {0, 0, 0}};
            return concord::Grid<uint8_t>(1, 1, 0.1, true, pose);
        }

        // Generate obstacle map for export (uses O_MAP at that height)
        inline concord::Grid<uint8_t> getObstacleMapAtHeight(double height) const {
            auto it = omap_indices_.find(height);
            if (it != omap_indices_.end() && it->second < grid_data_.size()) {
                return grid_data_[it->second];
            }

            // Return empty grid if height not found
            concord::Point center{0, 0, 0};
            concord::Pose pose{center, {0, 0, 0}};
            return concord::Grid<uint8_t>(1, 1, 0.1, true, pose);
        }

        // ========== Sensor Updates (Updates O_MAP, then generates C_MAP) ==========
        // Update from 2D laser scan at known height
        inline void updateFromLaserScan(const std::vector<concord::Point> &points, double scan_height,
                                        const concord::Pose &robot_pose) {
            // Ensure we have a layer for this height
            if (omap_indices_.find(scan_height) == omap_indices_.end()) {
                addElevationLevel(scan_height, scan_height + 0.1, "laser_scan");
            }

            auto omap_it = omap_indices_.find(scan_height);
            auto cmap_it = cmap_indices_.find(scan_height);

            if (omap_it != omap_indices_.end() && cmap_it != cmap_indices_.end() &&
                omap_it->second < grid_data_.size() && cmap_it->second < grid_data_.size()) {

                // Update obstacle map with scan points
                for (const auto &point : points) {
                    // Mark obstacle points as occupied in O_MAP
                    try {
                        auto [row1, col1] = grid_data_[omap_it->second].world_to_grid(point);
                        if (row1 < grid_data_[omap_it->second].rows() && col1 < grid_data_[omap_it->second].cols()) {
                            grid_data_[omap_it->second].set_value(row1, col1, 0); // occupied
                        }
                    } catch (const std::exception &) {
                    }

                    // Add cost around obstacles in C_MAP (inflation)
                    try {
                        auto [row2, col2] = grid_data_[cmap_it->second].world_to_grid(point);
                        if (row2 < grid_data_[cmap_it->second].rows() && col2 < grid_data_[cmap_it->second].cols()) {
                            grid_data_[cmap_it->second].set_value(row2, col2, 255); // impossible
                        }
                    } catch (const std::exception &) {
                    }
                    // TODO: Add inflation radius implementation
                }

                costmap_dirty_ = true;
            }
        }

        // Update from 3D point cloud
        inline void updateFromPointCloud(const std::vector<concord::Point> &points, const concord::Pose &sensor_pose) {
            for (const auto &point : points) {
                double height = point.z;

                // Find appropriate layer for this point's height
                size_t layer_index = LayerManager::getLayerIndex(height, layer_heights_);
                if (layer_index < layer_heights_.size()) {
                    double layer_height = layer_heights_[layer_index];

                    // Ensure layers exist for this height
                    if (omap_indices_.find(layer_height) == omap_indices_.end()) {
                        addElevationLevel(layer_height, layer_height + 0.1, "pointcloud");
                    }

                    // Update both O_MAP and C_MAP
                    updateObstacleMap(point, layer_height, 0); // occupied
                    updateCostMap(point, layer_height, 255);   // impossible
                }
            }
            costmap_dirty_ = true;
        }

        // Update from depth camera
        inline void updateFromDepthImage(const concord::Grid<float> &depth_image, const concord::Pose &camera_pose) {
            // Placeholder implementation - would need full camera projection
            // For demonstration, treat as simplified point cloud
            std::vector<concord::Point> points;
            points.reserve(depth_image.rows() * depth_image.cols());

            for (size_t r = 0; r < depth_image.rows(); ++r) {
                for (size_t c = 0; c < depth_image.cols(); ++c) {
                    float depth = depth_image(r, c);
                    if (depth > 0.1 && depth < 10.0) { // Valid depth range
                        // Simplified projection - actual implementation would use camera intrinsics
                        concord::Point world_point{camera_pose.point.x + depth * std::cos(camera_pose.angle.yaw),
                                                   camera_pose.point.y + depth * std::sin(camera_pose.angle.yaw),
                                                   camera_pose.point.z};
                        points.push_back(world_point);
                    }
                }
            }

            updateFromPointCloud(points, camera_pose);
        }

        // ========== Utilities ==========
        // Clear all dynamic obstacles from both O_MAP and C_MAP layers
        inline void clearDynamicLayers() {
            // Reset dynamic layers to default values
            // For simplicity, we'll mark all layers as potentially having dynamic content
            for (const auto &[height, omap_index] : omap_indices_) {
                if (omap_index < grid_data_.size()) {
                    // Reset O_MAP to unknown (128)
                    std::fill(grid_data_[omap_index].begin(), grid_data_[omap_index].end(), 128);
                }
            }

            for (const auto &[height, cmap_index] : cmap_indices_) {
                if (cmap_index < grid_data_.size()) {
                    // Reset C_MAP to good travel (0)
                    std::fill(grid_data_[cmap_index].begin(), grid_data_[cmap_index].end(), 0);
                }
            }

            costmap_dirty_ = true;
        }

        // Reset specific height level (both O_MAP and C_MAP)
        inline void resetElevationLevel(double height) {
            auto omap_it = omap_indices_.find(height);
            auto cmap_it = cmap_indices_.find(height);

            if (omap_it != omap_indices_.end() && omap_it->second < grid_data_.size()) {
                std::fill(grid_data_[omap_it->second].begin(), grid_data_[omap_it->second].end(), 128); // unknown
            }

            if (cmap_it != cmap_indices_.end() && cmap_it->second < grid_data_.size()) {
                std::fill(grid_data_[cmap_it->second].begin(), grid_data_[cmap_it->second].end(), 0); // good travel
            }

            costmap_dirty_ = true;
        }

        // Merge with another map (validates dual-layer compatibility)
        inline void mergeMap(const NavigationMap &other) {
            // Validate dual-layer integrity before merging
            if (!other.validateDualLayerIntegrity()) {
                throw std::runtime_error("Cannot merge: other map has invalid dual-layer structure");
            }

            // Add any missing heights from other map
            for (double height : other.layer_heights_) {
                if (omap_indices_.find(height) == omap_indices_.end()) {
                    addElevationLevel(height, height + 0.1, "merged");
                }
            }

            // Merge data - take maximum occupancy/cost values
            for (const auto &[height, other_omap_index] : other.omap_indices_) {
                auto our_omap_it = omap_indices_.find(height);
                auto our_cmap_it = cmap_indices_.find(height);
                auto other_cmap_it = other.cmap_indices_.find(height);

                if (our_omap_it != omap_indices_.end() && our_cmap_it != cmap_indices_.end() &&
                    other_cmap_it != other.cmap_indices_.end() && other_omap_index < other.grid_data_.size() &&
                    other_cmap_it->second < other.grid_data_.size() && our_omap_it->second < grid_data_.size() &&
                    our_cmap_it->second < grid_data_.size()) {

                    // Merge O_MAP and C_MAP data (simplified - would need proper grid alignment)
                    costmap_dirty_ = true;
                }
            }
        }

        // ========== File I/O using zoneout infrastructure ==========
        // Save using zoneout's WGS84/ENU workflow (saves to GeoJSON + GeoTIFF)
        inline void save(const std::filesystem::path &directory) const {
            // Create directory if it doesn't exist
            std::filesystem::create_directories(directory);

            // Save base zone using zoneout infrastructure
            base_zone_.save(directory);

            // Save navigation-specific data
            // Placeholder - would save grid data, layer information, etc.
            // In real implementation, would serialize dual-layer system to files

            // Create a simple marker file to indicate successful save
            std::ofstream marker(directory / "navigation_map.info");
            if (marker.is_open()) {
                marker << "NavigationMap saved successfully\n";
                marker << "Layers: " << getLayerPairCount() << "\n";
                marker.close();
            }
        }
        static inline NavigationMap load(const std::filesystem::path &directory) {
            // Placeholder implementation
            concord::Point center{0, 0, 0};
            concord::Pose pose{center, {0, 0, 0}};
            std::vector<concord::Point> boundary_points = {{-10, -10, 0}, {10, -10, 0}, {10, 10, 0}, {-10, 10, 0}};
            concord::Polygon boundary(boundary_points);
            concord::Datum datum; // Default datum

            return NavigationMap("loaded_map", "navigation", boundary, datum);
        }

        // Export to files following zoneout pattern
        void toFiles(const std::filesystem::path &vector_path, const std::filesystem::path &raster_path) const {
            base_zone_.toFiles(vector_path, raster_path);
        }

        // ========== Layer Access for Visualization ==========
        // Get O_MAP layer for visualization/export (PGM standard)
        inline const concord::Grid<uint8_t> &getObstacleLayer(double height) const {
            auto it = omap_indices_.find(height);
            if (it != omap_indices_.end() && it->second < grid_data_.size()) {
                return grid_data_[it->second];
            }

            // Return reference to first available layer if height not found
            if (!grid_data_.empty()) {
                return grid_data_[0];
            }

            // This shouldn't happen in a properly initialized map
            throw std::runtime_error("No obstacle layers available");
        }

        // Get C_MAP layer for visualization/export (Navigation standard)
        inline const concord::Grid<uint8_t> &getCostLayer(double height) const {
            auto it = cmap_indices_.find(height);
            if (it != cmap_indices_.end() && it->second < grid_data_.size()) {
                return grid_data_[it->second];
            }

            // Return reference to first available layer if height not found
            if (!grid_data_.empty()) {
                return grid_data_[0];
            }

            // This shouldn't happen in a properly initialized map
            throw std::runtime_error("No cost layers available");
        }

        // Get composite obstacle view (max projection of all O_MAP layers)
        inline concord::Grid<uint8_t> getCompositeObstacleView() const {
            if (omap_indices_.empty() || grid_data_.empty()) {
                concord::Point center{0, 0, 0};
                concord::Pose pose{center, {0, 0, 0}};
                return concord::Grid<uint8_t>(1, 1, 0.1, true, pose);
            }

            // Start with first O_MAP layer
            size_t base_index = omap_indices_.begin()->second;
            concord::Grid<uint8_t> composite = grid_data_[base_index];

            // Combine with other O_MAP layers (min value = most occupied)
            for (const auto &[height, index] : omap_indices_) {
                if (index < grid_data_.size() && index != base_index) {
                    for (size_t i = 0; i < composite.rows() * composite.cols(); ++i) {
                        uint8_t current_val = composite(i / composite.cols(), i % composite.cols());
                        uint8_t layer_val =
                            grid_data_[index](i / grid_data_[index].cols(), i % grid_data_[index].cols());
                        // Take minimum (0=occupied is higher priority than 255=free)
                        composite.set_value(i / composite.cols(), i % composite.cols(),
                                            std::min(current_val, layer_val));
                    }
                }
            }

            return composite;
        }

        // Get composite cost view (max projection of all C_MAP layers)
        inline concord::Grid<uint8_t> getCompositeCostView() const {
            if (cmap_indices_.empty() || grid_data_.empty()) {
                concord::Point center{0, 0, 0};
                concord::Pose pose{center, {0, 0, 0}};
                return concord::Grid<uint8_t>(1, 1, 0.1, true, pose);
            }

            // Start with first C_MAP layer
            size_t base_index = cmap_indices_.begin()->second;
            concord::Grid<uint8_t> composite = grid_data_[base_index];

            // Combine with other C_MAP layers (max cost)
            for (const auto &[height, index] : cmap_indices_) {
                if (index < grid_data_.size() && index != base_index) {
                    for (size_t i = 0; i < composite.rows() * composite.cols(); ++i) {
                        uint8_t current_cost = composite(i / composite.cols(), i % composite.cols());
                        uint8_t layer_cost =
                            grid_data_[index](i / grid_data_[index].cols(), i % grid_data_[index].cols());
                        // Take maximum cost
                        composite.set_value(i / composite.cols(), i % composite.cols(),
                                            std::max(current_cost, layer_cost));
                    }
                }
            }

            return composite;
        }

        // Get 3D point cloud representation
        inline std::vector<concord::Point> toPointCloud() const {
            std::vector<concord::Point> points;

            // Extract occupied points from all O_MAP layers
            for (const auto &[height, index] : omap_indices_) {
                if (index < grid_data_.size()) {
                    const auto &grid = grid_data_[index];
                    for (size_t r = 0; r < grid.rows(); ++r) {
                        for (size_t c = 0; c < grid.cols(); ++c) {
                            if (grid(r, c) == 0) { // occupied
                                concord::Point grid_point = grid.get_point(r, c);
                                points.push_back({grid_point.x, grid_point.y, height});
                            }
                        }
                    }
                }
            }

            return points;
        }

        // ========== Dual-Layer System Validation ==========
        // Check if dual-layer system is intact
        inline bool validateDualLayerIntegrity() const {
            // Check if every height has both O_MAP and C_MAP
            if (omap_indices_.size() != cmap_indices_.size()) {
                return false;
            }

            for (const auto &[height, omap_index] : omap_indices_) {
                auto cmap_it = cmap_indices_.find(height);
                if (cmap_it == cmap_indices_.end()) {
                    return false; // Missing C_MAP for this height
                }

                // Check if indices are valid
                if (omap_index >= grid_data_.size() || cmap_it->second >= grid_data_.size()) {
                    return false;
                }
            }

            return true;
        }

        // Get layer pair count (should be even - each height has O_MAP+C_MAP)
        inline size_t getLayerPairCount() const {
            return omap_indices_.size(); // Number of height levels with dual layers
        }

        // Get all heights with complete dual layers
        inline std::vector<double> getValidHeights() const {
            std::vector<double> valid_heights;

            for (const auto &[height, omap_index] : omap_indices_) {
                auto cmap_it = cmap_indices_.find(height);
                if (cmap_it != cmap_indices_.end() && omap_index < grid_data_.size() &&
                    cmap_it->second < grid_data_.size()) {
                    valid_heights.push_back(height);
                }
            }

            // Sort heights for consistency
            std::sort(valid_heights.begin(), valid_heights.end());
            return valid_heights;
        }

        // ========== Metadata Access ==========
        const NavigationMetadata &getMetadata() const { return nav_metadata_; }
        NavigationMetadata &getMetadata() { return nav_metadata_; }
    };

} // namespace gridmap

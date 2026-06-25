/**
 * @brief Hybrid Ground Segmentation and Downward Stair/Drop-off Hazard Detection Pipeline.
 * * This algorithm processes unordered 3D point clouds by projecting them into a discretized 2D 
 * elevation grid to perform deterministic, geometry-based hazard mapping and ground estimation.
 * * Pipeline Architecture:
 * * 1. Grid Discretization (Pass 1):
 * Projects the point cloud into a 2D spatial grid, capturing the absolute minimum Z-value 
 * per cell to construct a localized, high-resolution elevation map.
 * * 2. Gradient-Based Drop-off Detection (Pass 4):
 * Operates on the raw, unblurred elevation map. Computes forward (+x) and lateral (+y) spatial 
 * height differentials ($\Delta z$) between adjacent cells. Cells exceeding 'stair_drop_threshold' 
 * are flagged as candidate drop-off edges.
 * * 3. Spatial and Structural Filtering:
 * - Width Filter: Performs a column-wise run-length scan over the grid to verify that the 
 * detected edge forms a continuous lateral structure matching a minimum width parameter, 
 * effectively discarding isolated point noise or small debris.
 * - Lower Surface Validation: Looks ahead (+2 cells) from the edge to verify the existence 
 * of a lower, flat surface within standard architectural limits ($\Delta z \le \text{0.25 m}$), 
 * differentiating stairs/curbs from walls or open voids.
 * * 4. Kinematic BFS Flood-Fill (Pass 2):
 * Propagates ground validation outward via a Breadth-First Search starting from the robot's 
 * immediate footprint. Enforces strict slope/step continuity ($|\Delta z| < \text{max\_step\_height}$) 
 * between neighboring cells, allowing smooth ramps to be validated while halting at unvalidated steps.
 * * 5. Point Classification (Pass 3):
 * Intercepts and segments the raw point cloud into three distinct streams based on the spatial masks:
 * - Stair Edge Points: Diverted immediately to a dedicated hazard cloud.
 * - Ground Points: Evaluated against the flood-fill mask and local thickness bounds.
 * - Non-Ground Points: Classified as standard positive obstacles.
 */

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

struct GridPlane {
    float z_val = 0.0f;
    bool valid = false;
    int age = 0;
};

class GroundDetectionRobust : public rclcpp::Node {
public:
    GroundDetectionRobust() : Node("ground_detection_robust") {
        this->declare_parameter("lidar_topic", "/livox/lidar");
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("cell_size", 0.25);
        this->declare_parameter("ground_thickness", 0.06);
        this->declare_parameter("max_range", 15.0);
        this->declare_parameter("min_range", 1.1); 
        this->declare_parameter("max_step_height", 0.08); 

        // [FEATURE: New Parameters for Stair/Drop-off Detection]
        // [WHY: Allows tuning of the geometric heuristics based on building codes and sensor characteristics.]
        this->declare_parameter("stair_detection", true);           // Master toggle
        this->declare_parameter("stair_drop_threshold", 0.05);      // Min height drop (150mm standard riser)
        this->declare_parameter("stair_min_width", 0.5);            // Min lateral width to ignore noise
        this->declare_parameter("stair_max_width", 4.0);            // Max lateral width (ignores terrain shifts)
        this->declare_parameter("stair_detection_range", 30.0);      // Max range to search (due to occlusion)
        this->declare_parameter("stair_forward_only", true);        // Restrict search space to +x hemisphere

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("lidar_topic").as_string(), qos, 
            std::bind(&GroundDetectionRobust::lidarCallback, this, std::placeholders::_1));

        pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_ground", 10);
        pub_nonground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_nonground", 10);
        
        // [FEATURE: Dedicated Stair Edge Publisher]
        // [WHY: Allows downstream navigation/safety nodes to subscribe to the isolated hazard for immediate stopping, and aids in RViz visualization.]
        pub_stair_edge_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_stair_edge", 10);
        
        pub_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_local", 10);
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::string target_frame = this->get_parameter("target_frame").as_string();
        sensor_msgs::msg::PointCloud2 cloud_tf;
        try {
            auto t = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, rclcpp::Time(0));
            tf2::doTransform(*msg, cloud_tf, t);
        } catch (tf2::TransformException &ex) { return; }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_tf, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obs(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr stair(new pcl::PointCloud<pcl::PointXYZ>);

        // [FEATURE: Added Stair Cloud Output]
        // [WHY: Plumbs the separated points out to the callback for independent publishing.]
        segmentHybrid(cloud, ground, obs, stair);

        publishCloud(ground, pub_ground_, target_frame);
        publishCloud(obs, pub_nonground_, target_frame);
        publishCloud(stair, pub_stair_edge_, target_frame);
    }

   void segmentHybrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_out,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr& obs_out,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr& stair_out) 
    {
        double cell_size = this->get_parameter("cell_size").as_double();
        double thickness = this->get_parameter("ground_thickness").as_double();
        double max_r = this->get_parameter("max_range").as_double();
        double max_step = this->get_parameter("max_step_height").as_double(); 
        
        bool run_stair_detection = this->get_parameter("stair_detection").as_bool();
        double stair_drop_thresh = this->get_parameter("stair_drop_threshold").as_double();
        double stair_min_w = this->get_parameter("stair_min_width").as_double();
        double stair_max_w = this->get_parameter("stair_max_width").as_double();
        double stair_range = this->get_parameter("stair_detection_range").as_double();
        bool stair_fwd_only = this->get_parameter("stair_forward_only").as_bool();

        int grid_dim = std::max(1, static_cast<int>((max_r * 2) / cell_size));
        int center = grid_dim / 2;
        
        if (plane_grid_.size() != static_cast<size_t>(grid_dim * grid_dim)) {
            plane_grid_.assign(grid_dim * grid_dim, {0.0f, false, 0});
        }

        std::vector<float> current_min_z(grid_dim * grid_dim, 100.0f);
        std::vector<bool> ground_mask(grid_dim * grid_dim, false); 
        
        // [FEATURE: Stair Edge Mask Allocation]
        // [WHY: Initialized to false per frame to prevent phantom hazards from lingering incorrectly. Parallel to plane_grid_.]
        std::vector<bool> stair_edge_mask(grid_dim * grid_dim, false);

        // --- Pass 1: Surgical Self-Filter & Grid Population ---
        for (const auto& pt : input->points) {
            float r2 = pt.x*pt.x + pt.y*pt.y;
            
            if (r2 < 1.1 * 1.1 && pt.z > 0.15) {
                obs_out->push_back(pt); continue; 
            }
            if (r2 > max_r*max_r || r2 < 0.04) continue; 

            int gx = static_cast<int>(pt.x / cell_size) + center;
            int gy = static_cast<int>(pt.y / cell_size) + center;

            if (gx >= 0 && gx < grid_dim && gy >= 0 && gy < grid_dim) {
                int idx = gy * grid_dim + gx;
                if (pt.z < current_min_z[idx]) current_min_z[idx] = pt.z;
            }
        }

        // --- New Pass 4: Drop-off Edge Detection ---
        // [FEATURE: Placed BEFORE neighbor-fill (Pass 2)]
        // [WHY: Prevents the subsequent spatial smoothing algorithm from blurring the sharp Z-gradient required to identify the riser.]
        if (run_stair_detection) {
            int max_idx_range = static_cast<int>(stair_range / cell_size);
            std::vector<bool> candidate_mask(grid_dim * grid_dim, false);

            // Step 4b: Per-cell gradient check
            for (int gy = 1; gy < grid_dim - 1; gy++) {
                // If forward_only is true, start gx at center. Otherwise start at 1.
                int start_gx = stair_fwd_only ? center : 1;
                
                for (int gx = start_gx; gx < grid_dim - 1; gx++) {
                    // Check Euclidean range logic
                    float dist_x = (gx - center) * cell_size;
                    float dist_y = (gy - center) * cell_size;
                    if (std::sqrt(dist_x*dist_x + dist_y*dist_y) > stair_range) continue;

                    int idx = gy * grid_dim + gx;
                    if (current_min_z[idx] > 90.0f) continue;

                    // Forward check (+x)
                    int idx_fwd = gy * grid_dim + (gx + 1);
                    if (current_min_z[idx_fwd] < 90.0f) {
                        float dz_fwd = current_min_z[idx] - current_min_z[idx_fwd];
                        if (dz_fwd > stair_drop_thresh) {
                            candidate_mask[idx] = true;
                            continue; // Skip lateral check if already flagged
                        }
                    }

                    // Lateral check (+y)
                    int idx_lat = (gy + 1) * grid_dim + gx;
                    if (current_min_z[idx_lat] < 90.0f) {
                        float dz_lat = current_min_z[idx] - current_min_z[idx_lat];
                        if (dz_lat > stair_drop_thresh) {
                            candidate_mask[idx] = true;
                        }
                    }
                }
            }

            // Step 4c & 4d: Width Filter & Lower Surface Confirmation
            // [FEATURE: Column-wise connected component scan]
            // [WHY: Single cells flagged by noise are ignored. Requires the drop-off to span the width of an actual staircase/curb.]
            int min_cells_width = static_cast<int>(stair_min_w / cell_size);
            int max_cells_width = static_cast<int>(stair_max_w / cell_size);

            for (int gx = 1; gx < grid_dim - 1; gx++) {
                int run_length = 0;
                int run_start_gy = -1;

                for (int gy = 1; gy < grid_dim; gy++) {
                    int idx = gy * grid_dim + gx;
                    if (candidate_mask[idx]) {
                        if (run_length == 0) run_start_gy = gy;
                        run_length++;
                    } else {
                        // End of a run. Validate it.
                        if (run_length >= min_cells_width && run_length <= max_cells_width) {
                            
                            // 4d: Lower Surface Check
                            // [WHY: Verifies the edge isn't an infinite void (like a cliff) or a wall, but an actual step with a lower tread.]
                            bool confirmed = false;
                            for (int r = run_start_gy; r < run_start_gy + run_length; r++) {
                                int r_idx = r * grid_dim + gx;
                                int check_idx_fwd = r * grid_dim + std::min(gx + 2, grid_dim - 1);
                                
                                if (current_min_z[check_idx_fwd] < 90.0f) {
                                    float drop = current_min_z[r_idx] - current_min_z[check_idx_fwd];
                                    if (drop > stair_drop_thresh && drop < 0.25) { // 250mm max riser
                                        confirmed = true;
                                        break;
                                    }
                                }
                            }

                            if (confirmed) {
                                for (int r = run_start_gy; r < run_start_gy + run_length; r++) {
                                    stair_edge_mask[r * grid_dim + gx] = true;
                                }
                            }
                        }
                        run_length = 0; // Reset for next run
                    }
                }
            }
        }

        // --- Pass 2: Kinematic Flood-Fill ---
        // (Unchanged from your previous setup, operates safely because stair pass already ran)
        std::queue<int> bfs;
        int seed_idx = -1;
        float seed_z = 90.0f;
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int cx = center + dx;
                int cy = center + dy;
                if (cx >= 0 && cx < grid_dim && cy >= 0 && cy < grid_dim) {
                    int idx = cy * grid_dim + cx;
                    if (current_min_z[idx] < seed_z) {
                        seed_z = current_min_z[idx];
                        seed_idx = idx;
                    }
                }
            }
        }

        if (seed_idx != -1) {
            ground_mask[seed_idx] = true;
            bfs.push(seed_idx);
        }

        while (!bfs.empty()) {
            int curr = bfs.front();
            bfs.pop();
            int cx = curr % grid_dim;
            int cy = curr / grid_dim;

            int neighbors[4][2] = {{cx-1, cy}, {cx+1, cy}, {cx, cy-1}, {cx, cy+1}};
            for (auto& n : neighbors) {
                int nx = n[0];
                int ny = n[1];
                if (nx >= 0 && nx < grid_dim && ny >= 0 && ny < grid_dim) {
                    int n_idx = ny * grid_dim + nx;
                    if (!ground_mask[n_idx] && current_min_z[n_idx] < 90.0f) {
                        if (std::abs(current_min_z[n_idx] - current_min_z[curr]) < max_step) {
                            ground_mask[n_idx] = true;
                            bfs.push(n_idx); 
                        }
                    }
                }
            }
        }

        // --- Pass 3: Classification ---
        ground_out->reserve(input->size());
        obs_out->reserve(input->size());
        stair_out->reserve(input->size());

        for (const auto& pt : input->points) {
            float r2 = pt.x*pt.x + pt.y*pt.y;
            if (r2 < 0.04) continue; 

            int gx = static_cast<int>(pt.x / cell_size) + center;
            int gy = static_cast<int>(pt.y / cell_size) + center;
            
            if (gx >= 0 && gx < grid_dim && gy >= 0 && gy < grid_dim) {
                int idx = gy * grid_dim + gx;
                
                // [FEATURE: Stair Edge Point Intercept]
                // [WHY: Captures any LiDAR points that land in cells flagged by our Stair Edge Mask, diverting them from the standard ground/obs split.]
                if (run_stair_detection && stair_edge_mask[idx]) {
                    stair_out->push_back(pt);
                    continue; 
                }

                float target_z = 0.0f;
                bool has_ground = false;

                if (ground_mask[idx]) {
                    target_z = current_min_z[idx];
                    has_ground = true;
                    plane_grid_[idx] = {target_z, true, 0}; 
                } 
                else if (plane_grid_[idx].valid && plane_grid_[idx].age < 15) {
                     if (current_min_z[idx] < 90.0f && std::abs(current_min_z[idx] - plane_grid_[idx].z_val) > max_step) {
                         has_ground = false; 
                     } else {
                         target_z = plane_grid_[idx].z_val;
                         has_ground = true;
                     }
                }

                double dynamic_thick = (r2 < 2.25) ? thickness * 1.5 : thickness;

                if (has_ground && (pt.z - target_z) < dynamic_thick) {
                    ground_out->push_back(pt);
                } else {
                    obs_out->push_back(pt);
                }
            } else {
                obs_out->push_back(pt);
            }
        }
        for(auto& p : plane_grid_) if(p.valid) p.age++;
    }

    void publishCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, std::string fid) {
        if (cloud->empty()) return;
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = this->now();
        msg.header.frame_id = fid;
        pub->publish(msg);
    }

    std::vector<GridPlane> plane_grid_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_, pub_nonground_, pub_stair_edge_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundDetectionRobust>());
    rclcpp::shutdown();
    return 0;
}
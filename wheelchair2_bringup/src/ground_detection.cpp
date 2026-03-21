#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

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

using std::placeholders::_1;

class GroundDetectionRobust : public rclcpp::Node {
public:
    GroundDetectionRobust() : Node("ground_detection_robust") {
        // Parameters
        this->declare_parameter("lidar_topic", "/livox/lidar");
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("publish_grid", true);

        this->declare_parameter("min_range", 0.5);
        this->declare_parameter("max_range", 15.0);
        this->declare_parameter("min_z_limit", -2.0);

        this->declare_parameter("cell_size", 0.25);
        this->declare_parameter("ground_thickness", 0.05);
        this->declare_parameter("max_step_height", 0.05);
        this->declare_parameter("out_grid_res", 0.05);
        this->declare_parameter("out_size_xy", 10.0);
        this->declare_parameter("angular_res_deg", 0.5);

        // TF Buffer
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // QoS: Best Effort for high-speed Lidar data
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);

        std::string topic = this->get_parameter("lidar_topic").as_string();
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, qos, std::bind(&GroundDetectionRobust::lidarCallback, this, _1));

        pub_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_local", 10);
        pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_ground", 10);
        pub_nonground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_nonground", 10);

        RCLCPP_INFO(this->get_logger(), "C++ Robust Ground Detection Started.");
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::string target_frame = this->get_parameter("target_frame").as_string();

        // 1. Transform PointCloud
        sensor_msgs::msg::PointCloud2 cloud_transformed;
        try {
            geometry_msgs::msg::TransformStamped t;
            t = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, rclcpp::Time(0));
            tf2::doTransform(*msg, cloud_transformed, t);
        } catch (tf2::TransformException &ex) {
            return; // Wait for TF
        }

        // 2. Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_transformed, *cloud);

        if (cloud->empty()) return;

        // 3. Filter & Segment
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_pts(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pts(new pcl::PointCloud<pcl::PointXYZ>);

        segmentGround(cloud, ground_pts, obs_pts);

        // 4. Occlusion Filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr visible_ground(new pcl::PointCloud<pcl::PointXYZ>);
        filterOcclusion(ground_pts, obs_pts, visible_ground);

        // 5. Publish Clouds
        publishCloud(visible_ground, pub_ground_, target_frame);
        publishCloud(obs_pts, pub_nonground_, target_frame);

        // 6. Grid Generation
        if (this->get_parameter("publish_grid").as_bool()) {
            publishGrid(obs_pts, visible_ground, target_frame);
        }
    }

    void segmentGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_out,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& obs_out) 
    {
        double cell_size = this->get_parameter("cell_size").as_double();
        double thickness = this->get_parameter("ground_thickness").as_double();
        double min_r = this->get_parameter("min_range").as_double();
        double max_r = this->get_parameter("max_range").as_double();
        double min_z = this->get_parameter("min_z_limit").as_double();

        // Find Bounds
        float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
        
        // First pass: Filter range and find bounds
        std::vector<int> valid_indices;
        valid_indices.reserve(input->size());
        
        for(size_t i=0; i<input->size(); ++i) {
            const auto& pt = input->points[i];
            float r2 = pt.x*pt.x + pt.y*pt.y;
            if (r2 >= min_r*min_r && r2 <= max_r*max_r && pt.z > min_z) {
                valid_indices.push_back(i);
                if(pt.x < min_x) min_x = pt.x;
                if(pt.x > max_x) max_x = pt.x;
                if(pt.y < min_y) min_y = pt.y;
                if(pt.y > max_y) max_y = pt.y;
            }
        }

        if (valid_indices.empty()) return;

        int width = std::ceil((max_x - min_x) / cell_size) + 1;
        int height = std::ceil((max_y - min_y) / cell_size) + 1;
        int gridSize = width * height;

        // Grid for lowest Z
        std::vector<float> min_z_grid(gridSize, 100.0f);

        // Fill Grid
        for(int idx : valid_indices) {
            const auto& pt = input->points[idx];
            int cx = (pt.x - min_x) / cell_size;
            int cy = (pt.y - min_y) / cell_size;
            int g_idx = cy * width + cx;
            if (g_idx >= 0 && g_idx < gridSize) {
                if (pt.z < min_z_grid[g_idx]) min_z_grid[g_idx] = pt.z;
            }
        }

        // Classification
        ground_out->reserve(valid_indices.size());
        obs_out->reserve(valid_indices.size());

        for(int idx : valid_indices) {
            const auto& pt = input->points[idx];
            int cx = (pt.x - min_x) / cell_size;
            int cy = (pt.y - min_y) / cell_size;
            int g_idx = cy * width + cx;
            
            bool is_ground = false;
            if (g_idx >= 0 && g_idx < gridSize) {
                float ground_z = min_z_grid[g_idx];
                // Simple check: is this point close to the lowest point in its cell?
                // (Simplified from the complex Python version for speed, but highly effective)
                if (ground_z < 99.0f && (pt.z - ground_z) < thickness) {
                    is_ground = true;
                }
            }

            if (is_ground) ground_out->push_back(pt);
            else obs_out->push_back(pt);
        }
    }

    void filterOcclusion(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ground,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& obs,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_visible)
    {
        if (obs->empty()) {
            *ground_visible = *ground;
            return;
        }

        double ang_res_deg = this->get_parameter("angular_res_deg").as_double();
        double ang_res = ang_res_deg * (M_PI / 180.0);
        int bins = std::ceil(2 * M_PI / ang_res) + 1;

        std::vector<float> min_obs_dist(bins, 999.0f);

        // Map obstacles to radial bins
        for (const auto& pt : obs->points) {
            float dist = std::sqrt(pt.x*pt.x + pt.y*pt.y);
            float ang = std::atan2(pt.y, pt.x);
            int bin = (ang + M_PI) / ang_res;
            if (bin >= 0 && bin < bins) {
                if (dist < min_obs_dist[bin]) min_obs_dist[bin] = dist;
            }
        }

        // Filter ground
        ground_visible->reserve(ground->size());
        for (const auto& pt : ground->points) {
            float dist = std::sqrt(pt.x*pt.x + pt.y*pt.y);
            float ang = std::atan2(pt.y, pt.x);
            int bin = (ang + M_PI) / ang_res;
            
            bool visible = true;
            if (bin >= 0 && bin < bins) {
                // Allow ground to be slightly behind obstacle (buffer)
                if (dist > (min_obs_dist[bin] + 0.2f)) visible = false;
            }
            if (visible) ground_visible->push_back(pt);
        }
    }

    void publishGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& obs,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& ground,
                     const std::string& frame_id)
    {
        double res = this->get_parameter("out_grid_res").as_double();
        double size = this->get_parameter("out_size_xy").as_double();
        
        int width = size / res;
        int height = size / res;
        int cx = width / 2;
        int cy = height / 2;

        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = this->now();
        grid.header.frame_id = frame_id;
        grid.info.resolution = res;
        grid.info.width = width;
        grid.info.height = height;
        grid.info.origin.position.x = -size / 2.0;
        grid.info.origin.position.y = -size / 2.0;
        grid.info.origin.position.z = 0.0;
        
        // Init with -1 (unknown)
        grid.data.assign(width * height, -1);

        auto mark_grid = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int val) {
            for (const auto& pt : cloud->points) {
                int c = (pt.x / res) + cx;
                int r = (pt.y / res) + cy;
                if (c >= 0 && c < width && r >= 0 && r < height) {
                    grid.data[r * width + c] = val;
                }
            }
        };

        // Mark Free (0)
        mark_grid(ground, 0);
        // Mark Occupied (100) - Overwrites free
        mark_grid(obs, 100);

        pub_grid_->publish(grid);
    }

    void publishCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                      std::string frame_id) {
        if (cloud->empty()) return;
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id;
        pub->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_nonground_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundDetectionRobust>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
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
        this->declare_parameter("min_range", 1.1); // Increased for body filter

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("lidar_topic").as_string(), qos, 
            std::bind(&GroundDetectionRobust::lidarCallback, this, std::placeholders::_1));

        pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_ground", 10);
        pub_nonground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_nonground", 10);
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

        segmentHybrid(cloud, ground, obs);

        publishCloud(ground, pub_ground_, target_frame);
        publishCloud(obs, pub_nonground_, target_frame);
    }

   void segmentHybrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_out,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr& obs_out) 
    {
        double cell_size = this->get_parameter("cell_size").as_double();
        double thickness = this->get_parameter("ground_thickness").as_double();
        double max_r = this->get_parameter("max_range").as_double();

        // Ensure grid_dim is at least 1 to avoid division by zero or empty vectors
        int grid_dim = std::max(1, static_cast<int>((max_r * 2) / cell_size));
        int center = grid_dim / 2;
        
        // Safety check for vector allocation
        if (plane_grid_.size() != static_cast<size_t>(grid_dim * grid_dim)) {
            plane_grid_.assign(grid_dim * grid_dim, {0.0f, false, 0});
        }

        std::vector<float> current_min_z(grid_dim * grid_dim, 100.0f);

        // Pass 1: Surgical Self-Filter & Grid Population
        for (const auto& pt : input->points) {
            float r2 = pt.x*pt.x + pt.y*pt.y;
            
            // Body filter: ignore points very close to robot if they are high
            if (r2 < 1.1 * 1.1 && pt.z > 0.15) {
                obs_out->push_back(pt); continue; 
            }
            if (r2 > max_r*max_r || r2 < 0.04) continue; 

            int gx = static_cast<int>(pt.x / cell_size) + center;
            int gy = static_cast<int>(pt.y / cell_size) + center;

            // BOUNDS CHECK: Essential to prevent SegFault
            if (gx >= 0 && gx < grid_dim && gy >= 0 && gy < grid_dim) {
                int idx = gy * grid_dim + gx;
                if (pt.z < current_min_z[idx]) current_min_z[idx] = pt.z;
            }
        }

        // Pass 2: Neighbor-Fill (Spatial Smoothing)
        for (int gy = 1; gy < grid_dim - 1; gy++) {
            for (int gx = 1; gx < grid_dim - 1; gx++) {
                int idx = gy * grid_dim + gx;
                if (current_min_z[idx] > 90.0f) {
                    int neighbors[] = {idx-1, idx+1, idx-grid_dim, idx+grid_dim};
                    for(int n : neighbors) {
                        if (current_min_z[n] < 90.0f) {
                            current_min_z[idx] = current_min_z[n]; break;
                        }
                    }
                }
            }
        }

        // Pass 3: Classification (Now with Bounds Checks)
        ground_out->reserve(input->size());
        obs_out->reserve(input->size());

        for (const auto& pt : input->points) {
            float r2 = pt.x*pt.x + pt.y*pt.y;
            if (r2 < 0.04) continue; 

            int gx = static_cast<int>(pt.x / cell_size) + center;
            int gy = static_cast<int>(pt.y / cell_size) + center;
            
            // CHECK BOUNDS AGAIN
            if (gx >= 0 && gx < grid_dim && gy >= 0 && gy < grid_dim) {
                int idx = gy * grid_dim + gx;
                float target_z = 0.0f;
                bool has_ground = false;

                if (current_min_z[idx] < 90.0f) {
                    target_z = current_min_z[idx];
                    has_ground = true;
                    plane_grid_[idx] = {target_z, true, 0}; 
                } else if (plane_grid_[idx].valid && plane_grid_[idx].age < 15) {
                    target_z = plane_grid_[idx].z_val;
                    has_ground = true;
                }

                double dynamic_thick = (r2 < 2.25) ? thickness * 1.5 : thickness;

                if (has_ground && (pt.z - target_z) < dynamic_thick) {
                    ground_out->push_back(pt);
                } else {
                    obs_out->push_back(pt);
                }
            } else {
                // Points outside the grid dimensions
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_, pub_nonground_;
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
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

struct SectorPlane {
    float a, b, c, d;
    bool valid = false;
    int age = 0;
};

class GroundDetectionHybrid : public rclcpp::Node {
public:
    GroundDetectionHybrid() : Node("ground_detection_hybrid") {
        this->declare_parameter("max_ramp_angle", 10.0); 
        this->declare_parameter("ground_thickness", 0.08);
        this->declare_parameter("voxel_size", 0.05); 
        this->declare_parameter("base_link_frame", "base_link");

        zone_ranges_ = {2.5, 5.0, 8.0, 15.0};
        num_sectors_ = {16, 32, 32, 16};
        
        sector_cache_.resize(4);
        for(int i=0; i<4; ++i) sector_cache_[i].resize(num_sectors_[i]);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto qos = rclcpp::QoS(10).reliable();
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar_inverted", qos, std::bind(&GroundDetectionHybrid::lidarCallback, this, std::placeholders::_1));

        pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_ground", 10);
        pub_nonground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_nonground", 10);
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::string base_frame = this->get_parameter("base_link_frame").as_string();
        sensor_msgs::msg::PointCloud2 cloud_tf;
        try {
            auto t = tf_buffer_->lookupTransform(base_frame, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_nanoseconds(50000000));
            tf2::doTransform(*msg, cloud_tf, t);
        } catch (tf2::TransformException &ex) { return; }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_tf, *cloud);

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        float leaf = this->get_parameter("voxel_size").as_double();
        vg.setLeafSize(leaf, leaf, leaf);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vox(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*cloud_vox);

        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_out(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obs_out(new pcl::PointCloud<pcl::PointXYZ>);

        processHybrid(cloud_vox, ground_out, obs_out);

        publishCloud(ground_out, pub_ground_, base_frame);
        publishCloud(obs_out, pub_nonground_, base_frame);
    }

    void processHybrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& g_out,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& o_out) 
    {
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> bins(4);
        for(int i=0; i<4; i++) {
            bins[i].resize(num_sectors_[i]);
            for(int j=0; j<num_sectors_[i]; j++) bins[i][j].reset(new pcl::PointCloud<pcl::PointXYZ>);
        }

        for (const auto& pt : input->points) {
            double r = std::sqrt(pt.x*pt.x + pt.y*pt.y);
            // Self-filter for wheelchair/body interference (Eliminates "Red Shadow")
            if (r < 1.1 && pt.z > 0.12) { o_out->push_back(pt); continue; }
            if (r > 15.0 || r < 0.2) continue;

            int zi = 0;
            for (int i=0; i<4; i++) { if (r < zone_ranges_[i]) { zi = i; break; } }
            double angle = std::atan2(pt.y, pt.x) + M_PI;
            int si = static_cast<int>(angle / (2 * M_PI) * num_sectors_[zi]) % num_sectors_[zi];
            bins[zi][si]->push_back(pt);
        }

        double max_angle = this->get_parameter("max_ramp_angle").as_double();
        double min_normal_z = std::cos(max_angle * M_PI / 180.0);
        double dist_thresh_base = this->get_parameter("ground_thickness").as_double();

        for (int s_idx = 0; s_idx < 32; s_idx++) { 
            SectorPlane last_valid_plane; // Seed for spatial propagation

            for (int z = 0; z < 4; z++) {
                int s = (s_idx * num_sectors_[z]) / 32;
                auto& bin = bins[z][s];
                
                // Adaptive thickness for outer noisy zones
                double dist_thresh = (z > 1) ? dist_thresh_base * 1.5 : dist_thresh_base;

                if (bin->empty()) { last_valid_plane.valid = false; continue; }

                SectorPlane &cached = sector_cache_[z][s];
                bool ground_found = false;

                std::sort(bin->points.begin(), bin->points.end(), 
                          [](const pcl::PointXYZ& a, const pcl::PointXYZ& b){ return a.z < b.z; });

                // A. SPATIAL PROPAGATION (Variable Corrected to last_valid_plane)
                if (last_valid_plane.valid) {
                    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                    for (size_t i = 0; i < bin->size(); i++) {
                        float dist = std::abs(last_valid_plane.a*bin->points[i].x + last_valid_plane.b*bin->points[i].y + 
                                              last_valid_plane.c*bin->points[i].z + last_valid_plane.d);
                        if (dist < dist_thresh) inliers->indices.push_back(i);
                    }
                    if (inliers->indices.size() >= 2 || (bin->size() < 5 && !inliers->indices.empty())) {
                        extractPoints(bin, inliers, g_out, o_out);
                        ground_found = true;
                    }
                }

                // B. TEMPORAL REUSE
                if (!ground_found && cached.valid && cached.age < 15) {
                    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                    for (size_t i = 0; i < bin->size(); i++) {
                        float dist = std::abs(cached.a*bin->points[i].x + cached.b*bin->points[i].y + 
                                              cached.c*bin->points[i].z + cached.d);
                        if (dist < dist_thresh) inliers->indices.push_back(i);
                    }
                    if (inliers->indices.size() > bin->size() * 0.4) {
                        extractPoints(bin, inliers, g_out, o_out);
                        cached.age = 0; ground_found = true;
                        last_valid_plane = cached;
                    }
                }

                // C. DISCOVERY (Variable Corrected to last_valid_plane)
                if (!ground_found && bin->size() >= 8) { 
                    double d = std::sqrt(bin->points[0].x*bin->points[0].x + bin->points[0].y*bin->points[0].y);
                    double adaptive_z_limit = (d * std::tan(max_angle * M_PI / 180.0)) + 0.12;

                    if (bin->points[0].z < adaptive_z_limit) {
                        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
                        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                        pcl::SACSegmentation<pcl::PointXYZ> seg;
                        seg.setOptimizeCoefficients(true);
                        seg.setModelType(pcl::SACMODEL_PLANE);
                        seg.setMethodType(pcl::SAC_RANSAC);
                        seg.setDistanceThreshold(dist_thresh);
                        seg.setInputCloud(bin);
                        seg.segment(*inliers, *coeff);

                        if (inliers->indices.size() > bin->size() * 0.3 && std::abs(coeff->values[2]) > min_normal_z) {
                            extractPoints(bin, inliers, g_out, o_out);
                            cached = {coeff->values[0], coeff->values[1], coeff->values[2], coeff->values[3], true, 0};
                            last_valid_plane = cached;
                            ground_found = true;
                        }
                    }
                }

                // D. FALLBACK: ABSOLUTE Z-ANCHOR
                if (!ground_found) {
                   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                   for (size_t i = 0; i < bin->size(); i++) {
                       if (std::abs(bin->points[i].z) < (dist_thresh * 0.8)) inliers->indices.push_back(i);
                   }
                   if (!inliers->indices.empty()) {
                       extractPoints(bin, inliers, g_out, o_out);
                       ground_found = true;
                   }
                }

                if (!ground_found) { *o_out += *bin; cached.valid = false; last_valid_plane.valid = false; }
                cached.age++;
            }
        }
    }

    void extractPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& bin, pcl::PointIndices::Ptr inliers, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& g, pcl::PointCloud<pcl::PointXYZ>::Ptr& o) {
        pcl::ExtractIndices<pcl::PointXYZ> ext;
        ext.setInputCloud(bin); ext.setIndices(inliers);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        ext.filter(*tmp); *g += *tmp;
        ext.setNegative(true); ext.filter(*tmp); *o += *tmp;
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

    std::vector<double> zone_ranges_;
    std::vector<int> num_sectors_;
    std::vector<std::vector<SectorPlane>> sector_cache_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_, pub_nonground_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundDetectionHybrid>());
    rclcpp::shutdown();
    return 0;
}
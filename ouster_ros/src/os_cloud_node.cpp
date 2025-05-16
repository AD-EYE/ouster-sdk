/**
 * Copyright (c) 2019, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Example node to publish point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <memory>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using Cloud = ouster_ros::Cloud;
using Point = ouster_ros::Point;
namespace sensor = ouster::sensor;


//============== part of new FOV shrinker ====================
/**
 * @brief Filter point clouds, to keep only the middle horizontal beams in the Field of View (FOV)
 * @param start_beam starting beam index in FOV
 * @param end_beam ending beam index in FOV
 * @param debug Enable debug messages
 */

 sensor_msgs::PointCloud2 FOV_shrinker(
    const sensor_msgs::PointCloud2& cloud_msg,
    uint32_t start_beam,
    uint32_t end_beam,
    bool debug = false){
        try{
            ros::Time start_time = ros::Time::now();

            uint32_t ros_height = end_beam - start_beam + 1;

            if(debug){
                ROS_INFO("Original data size: %zu bytes", cloud_msg.data.size());
                ROS_INFO("Expected row size: %u bytes", cloud_msg.row_step);
                ROS_INFO("Expected total size: %u bytes", cloud_msg.row_step * cloud_msg.height);
            }

            sensor_msgs::PointCloud2 filtered_cloud;

            //copy metadata
            filtered_cloud.header = cloud_msg.header;
            filtered_cloud.height = ros_height;
            filtered_cloud.width = cloud_msg.width;
            filtered_cloud.fields = cloud_msg.fields;
            filtered_cloud.is_bigendian = cloud_msg.is_bigendian;
            filtered_cloud.point_step = cloud_msg.point_step;
            filtered_cloud.row_step = cloud_msg.row_step;
            filtered_cloud.is_dense = cloud_msg.is_dense;

            // allocate memory for the filtered cloud
            filtered_cloud.data.resize(cloud_msg.row_step * ros_height);

            // copy only the rows in the ROI
            for (uint32_t i=0, beam_idx = start_beam; beam_idx <= end_beam; ++i, ++beam_idx){
                uint32_t start = beam_idx * filtered_cloud.row_step;
                uint32_t end = start + filtered_cloud.row_step;
                uint32_t dest = i * filtered_cloud.row_step;

                if (debug && i<2){  //just print first 2 beams to avoid log spam
                    ROS_INFO("Copying beam %u (index %u) from %u to %u ", beam_idx, i, start, end);
                }

                std::copy(cloud_msg.data.begin() + start, cloud_msg.data.begin() + end, filtered_cloud.data.begin() + dest);
            }

            if (debug){
                ros::Duration duration = ros::Time::now() - start_time;
                ROS_INFO("-----------------FOV shrinker performance-----------------");
                ROS_INFO("Input point cloud size: %u", cloud_msg.width * cloud_msg.height);
                ROS_INFO("Input point cloud size: %u x %u", cloud_msg.width, cloud_msg.height);
                ROS_INFO("Filtered point cloud: %u points", filtered_cloud.width * filtered_cloud.height);
                ROS_INFO("Filtered point cloud size: %u x %u", filtered_cloud.width, filtered_cloud.height);
                ROS_INFO("Time taken: %.2f ms", duration.toSec() * 1000.0);
            }

            return filtered_cloud;
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to shrink the FOV: %s", e.what());
            return cloud_msg; // return orignal on error
        }
    }
// ============ FOV shrinker =============


int main(int argc, char** argv) {
    ros::init(argc, argv, "os_cloud_node");
    ros::NodeHandle nh("~");

    auto tf_prefix = nh.param("tf_prefix", std::string{});
    if (!tf_prefix.empty() && tf_prefix.back() != '/') tf_prefix.append("/");
    auto sensor_frame = tf_prefix + "os_sensor";
    auto imu_frame = tf_prefix + "os_imu";
    auto lidar_frame = tf_prefix + "os_lidar";

    ouster_ros::OSConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("os_cloud_node: Calling config service failed");
        return EXIT_FAILURE;
    }

    auto info = sensor::parse_metadata(cfg.response.metadata);
    uint32_t H = info.format.pixels_per_column;
    uint32_t W = info.format.columns_per_frame;
    auto udp_profile_lidar = info.format.udp_profile_lidar;

    const int n_returns =
        (udp_profile_lidar == sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY)
            ? 1
            : 2;
    auto pf = sensor::get_format(info);

    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    auto img_suffix = [](int ind) {
        if (ind == 0) return std::string();
        return std::to_string(ind + 1);  // need second return to return 2
    };

    // auto lidar_pubs = std::vector<ros::Publisher>();
    // for (int i = 0; i < n_returns; i++) {
    //     auto pub = nh.advertise<sensor_msgs::PointCloud2>(
    //         std::string("points") + img_suffix(i), 10);
    //     lidar_pubs.push_back(pub);
    // }

    //============= part of FOV shrinker ===================
    // parameter for beam selection

    uint32_t n_beams = H;
    int start_beam_int = nh.param("start_beam", (int)(n_beams/4));
    int end_beam_int = nh.param("end_beam", (int)(start_beam_int + n_beams/2));
    uint32_t start_beam = static_cast<uint32_t>(std::max(0, start_beam_int));
    uint32_t end_beam = static_cast<uint32_t>(std::max(0, end_beam_int));
    bool debug_mode = nh.param("debug_mode", false);

    std::vector<ros::Publisher> fov_pubs;
    for (int i = 0; i < n_returns; i++){
        auto pub = nh.advertise<sensor_msgs::PointCloud2>(std::string("points")+img_suffix(i), 10);
        fov_pubs.push_back(pub);
    }

    ROS_INFO("FOV point cloud filtering initialized");
    ROS_INFO("Keeping beams %u to %u (of %u beams)", start_beam, end_beam-1, n_beams);
   // ============= FOV shrinker ===================

    auto xyz_lut = ouster::make_xyz_lut(info);

    ouster::LidarScan ls{W, H, udp_profile_lidar};
    Cloud cloud{W, H};

    ouster::ScanBatcher batch(W, pf);

    auto lidar_handler = [&](const PacketMsg& pm) mutable {
        if (batch(pm.buf.data(), ls)) {
            auto h = std::find_if(
                ls.headers.begin(), ls.headers.end(), [](const auto& h) {
                    return h.timestamp != std::chrono::nanoseconds{0};
                });
            if (h != ls.headers.end()) {
                for (int i = 0; i < n_returns; i++) {
                    scan_to_cloud(xyz_lut, h->timestamp, ls, cloud, i);
                    //lidar_pubs[i].publish(ouster_ros::cloud_to_cloud_msg(
                    //    cloud, h->timestamp, sensor_frame));                               // original publisher


                    // =============== part of FOV shrinker =================
                    auto cloud_msg = ouster_ros::cloud_to_cloud_msg(cloud, h->timestamp, sensor_frame);

                    // //publish the os_cloud
                    // lidar_pubs[i].publish(cloud_msg);

                    //publish the FOV_clouds
                    auto filtered_cloud = FOV_shrinker(cloud_msg, start_beam, end_beam, debug_mode);
                    fov_pubs[i].publish(filtered_cloud);
                    // =============== FOV shrinker =======================


                }
            }
        }
    };

    auto imu_handler = [&](const PacketMsg& p) {
        imu_pub.publish(ouster_ros::packet_to_imu_msg(p, imu_frame, pf));
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);

    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    ros::spin();

    return EXIT_SUCCESS;
}

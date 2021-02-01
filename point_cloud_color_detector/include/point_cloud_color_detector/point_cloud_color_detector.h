#ifndef POINT_CLOUD_COLOR_DETECTOR_H_
#define POINT_CLOUD_COLOR_DETECTOR_H_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <string>
#include <vector>

#include "color_detector_msgs/TargetPosition.h"

class PointCloudColorDetector {
 public:
    struct HSV {
        int h, s, v;
    };
    struct ThresholdHSV {
        HSV lower, upper;
    };
    PointCloudColorDetector();
    void sensor_callback(const sensor_msgs::PointCloud2ConstPtr &received_pc);
    pcl::PointCloud<pcl::PointXYZRGB> limit_point_cloud(const ThresholdHSV &thres_hsv, const pcl::PointCloud<pcl::PointXYZRGB> &pc);
    std::vector<pcl::PointIndices> euclidean_clustering(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> &pc);
    pcl::PointCloud<pcl::PointXYZRGB> get_lagest_cluster(const pcl::PointCloud<pcl::PointXYZRGB> &pc, const std::vector<pcl::PointIndices> &pc_indices);
    color_detector_msgs::TargetPosition calc_target_position(const  pcl::PointCloud<pcl::PointXYZRGB> &pc);
    void detect_target_position(const ThresholdHSV &thres_hsv, const std_msgs::Header &header, const ros::Publisher &masked_pc_pub, const ros::Publisher &target_pc_pub, const pcl::PointCloud<pcl::PointXYZRGB> &pc);
    void process();

 private:
    const std::vector<std::string> colors = {"green", "red", "blue", "yellow", "white"};
    std::vector<ThresholdHSV> config_hsvs;
    double TOLERANCE;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber pc_sub;
    ros::Publisher target_position_pub;
    std::vector<ros::Publisher> masked_pc_pubs;
    std::vector<ros::Publisher> target_pc_pubs;
};

#endif  // POINT_CLOUD_COLOR_DETECTOR_H_

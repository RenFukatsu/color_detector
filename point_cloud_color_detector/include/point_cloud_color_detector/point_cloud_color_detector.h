#ifndef POINT_CLOUD_COLOR_DETECTOR_H_
#define POINT_CLOUD_COLOR_DETECTOR_H_

#include <pcl/point_types_conversion.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <string>
#include <vector>

#include "color_detector_msgs/TargetPosition.h"
#include "color_detector_params/HsvConfig.h"
#include "color_detector_params/hsv.h"
#include "color_detector_srvs/ColorEnable.h"

using color_detector_params_hsv::HSV;
using color_detector_params_hsv::ThresholdHSV;

class PointCloudColorDetector {
 public:
    PointCloudColorDetector();
    void set_hsv_params();
    bool enable_color(color_detector_srvs::ColorEnable::Request &req, color_detector_srvs::ColorEnable::Response &res);
    void sensor_callback(const sensor_msgs::PointCloud2ConstPtr &received_pc);
    pcl::PointCloud<pcl::PointXYZRGB> limit_point_cloud(const ThresholdHSV &thres_hsv,
                                                        const pcl::PointCloud<pcl::PointXYZRGB> &pc);
    std::vector<pcl::PointIndices> euclidean_clustering(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> &pc);
    pcl::PointCloud<pcl::PointXYZRGB> get_target_cluster(const pcl::PointCloud<pcl::PointXYZRGB> &pc,
                                                         std::vector<pcl::PointIndices> &pc_indices);
    color_detector_msgs::TargetPosition calc_target_position(const pcl::PointCloud<pcl::PointXYZRGB> &pc);
    pcl::PointCloud<pcl::PointXYZRGB> detect_target_cluster(const ThresholdHSV &thres_hsv,
                                                            const std_msgs::Header &header,
                                                            const ros::Publisher &masked_pc_pub,
                                                            const pcl::PointCloud<pcl::PointXYZRGB> &pc);
    void save_csv(const color_detector_msgs::TargetPosition &target_position);
    void process();
    std::vector<std::string> colors;
    std::vector<ThresholdHSV> param_hsvs;
    std::vector<bool> use_colors;
    bool only_publish_mask_points;
    bool publish_target_points;

 private:
    double TOLERANCE;
    double HIGHEST_TARGET_Y;
    double LOWEREST_TARGET_Y;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber pc_sub;
    ros::Publisher target_position_pub;
    std::vector<ros::Publisher> masked_pc_pubs;
    std::vector<ros::Publisher> target_pc_pubs;
    ros::ServiceServer color_enable_srv;
};

#endif  // POINT_CLOUD_COLOR_DETECTOR_H_

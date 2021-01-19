#ifndef __POINT_CLOUD_COLOR_DETECTOR_HPP
#define __POINT_CLOUD_COLOR_DETECTOR_HPP

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

class PointCloudColorDetector {
public:
    struct HSV { int h, s, v; };
    struct ThresholdHSV { HSV lower, upper; };
    PointCloudColorDetector();
    void sensor_callback(const sensor_msgs::PointCloud2ConstPtr &received_pc);
    void detect_target_position(ThresholdHSV thres_hsv, const std_msgs::Header &header, const ros::Publisher &masked_pc_pub, const ros::Publisher &target_pc_pub, const pcl::PointCloud<pcl::PointXYZRGB> &pc);
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
    std::vector<ros::Publisher> masked_pc_pubs;
    std::vector<ros::Publisher> target_pc_pubs;
};

#endif // __POINT_CLOUD_COLOR_DETECTOR_HPP
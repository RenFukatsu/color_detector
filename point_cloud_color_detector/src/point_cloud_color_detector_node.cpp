#include "point_cloud_color_detector/point_cloud_color_detector.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "point_cloud_color_detector");
    PointCloudColorDetector point_cloud_color_detector;
    point_cloud_color_detector.process();
    return 0;
}

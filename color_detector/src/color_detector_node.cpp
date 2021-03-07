#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include "color_detector_params/hsv.h"
#include "image_color_detector/image_color_detector.h"
#include "point_cloud_color_detector/point_cloud_color_detector.h"

class ColorDetector {
 public:
    ColorDetector() {
        dynamic_reconfigure::Server<color_detector_params::HsvConfig>::CallbackType dr_callback;
        dr_callback = boost::bind(&ColorDetector::update_dr, this, _1, _2);
        dr_server.setCallback(dr_callback);
    }
    void update_dr(color_detector_params::HsvConfig &config, uint32_t level) {
        point_cloud_color_detector.only_publish_mask_points = config.only_publish_mask;
        point_cloud_color_detector.publish_target_points = config.publish_target;
        image_color_detector.only_publish_mask_image = config.only_publish_mask;
        image_color_detector.publish_target_image = config.publish_target;
        color_detector_params_hsv::update_use_colors(point_cloud_color_detector.colors, config,
                                                     point_cloud_color_detector.use_colors);
        color_detector_params_hsv::update_hsv_params(point_cloud_color_detector.colors, config,
                                                     point_cloud_color_detector.param_hsvs);
        color_detector_params_hsv::update_hsv_params(image_color_detector.colors, config,
                                                     image_color_detector.param_hsvs);
    }
    void process() { ros::spin(); }

 private:
    PointCloudColorDetector point_cloud_color_detector;
    ImageColorDetector image_color_detector;
    dynamic_reconfigure::Server<color_detector_params::HsvConfig> dr_server;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "color_detector");
    ColorDetector color_detector;
    color_detector.process();
    return 0;
}

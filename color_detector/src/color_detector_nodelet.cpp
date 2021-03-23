#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "color_detector_params/hsv.h"
#include "image_color_detector/image_color_detector.h"
#include "point_cloud_color_detector/point_cloud_color_detector.h"

namespace color_detector {
class ColorDetector : public nodelet::Nodelet {
 public:
    virtual void onInit() {
        nh_ = getNodeHandle();
        private_nh_ = getPrivateNodeHandle();
        point_cloud_color_detector =
            std::shared_ptr<PointCloudColorDetector>(new PointCloudColorDetector(nh_, private_nh_));
        image_color_detector = std::shared_ptr<ImageColorDetector>(new ImageColorDetector(nh_, private_nh_));

        dynamic_reconfigure::Server<color_detector_params::HsvConfig>::CallbackType dr_callback;
        dr_callback = boost::bind(&ColorDetector::update_dr, this, _1, _2);
        dr_server.setCallback(dr_callback);
    }
    void update_dr(color_detector_params::HsvConfig &config, uint32_t level) {
        point_cloud_color_detector->only_publish_mask_points_ = config.only_publish_mask;
        point_cloud_color_detector->publish_target_points_ = config.publish_target;
        image_color_detector->only_publish_mask_image_ = config.only_publish_mask;
        image_color_detector->publish_target_image_ = config.publish_target;
        color_detector_params_hsv::update_use_colors(point_cloud_color_detector->colors_, config,
                                                     point_cloud_color_detector->use_colors_);
        color_detector_params_hsv::update_hsv_params(point_cloud_color_detector->colors_, config,
                                                     point_cloud_color_detector->param_hsvs_);
        color_detector_params_hsv::update_hsv_params(image_color_detector->colors_, config,
                                                     image_color_detector->param_hsvs_);
    }
    void process() { ros::spin(); }

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::shared_ptr<PointCloudColorDetector> point_cloud_color_detector;
    std::shared_ptr<ImageColorDetector> image_color_detector;
    dynamic_reconfigure::Server<color_detector_params::HsvConfig> dr_server;
};
}  // namespace color_detector
PLUGINLIB_EXPORT_CLASS(color_detector::ColorDetector, nodelet::Nodelet);

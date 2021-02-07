#include "point_cloud_color_detector/point_cloud_color_detector.h"

PointCloudColorDetector::PointCloudColorDetector() : private_nh("~"),
                                                     use_colors(colors.size(), false),
                                                     config_hsvs(colors.size()),
                                                     masked_pc_pubs(colors.size()),
                                                     target_pc_pubs(colors.size()) {
    dynamic_reconfigure::Server<color_detector_params::HsvConfig>::CallbackType hsv_param_callback_type;
    hsv_param_callback_type = boost::bind(&PointCloudColorDetector::hsv_param_changed, this, _1, _2);
    hsv_param_server.setCallback(hsv_param_callback_type);
    pc_sub = nh.subscribe("/camera/depth_registered/points", 1, &PointCloudColorDetector::sensor_callback, this);
    target_position_pub = nh.advertise<color_detector_msgs::TargetPosition>("/target/position", 1);
    for (size_t i = 0; i < colors.size(); i++) {
        masked_pc_pubs[i] = private_nh.advertise<sensor_msgs::PointCloud2>("/cloud/masked/" + colors[i] + "/raw", 1);
        target_pc_pubs[i] = private_nh.advertise<sensor_msgs::PointCloud2>("/cloud/target/" + colors[i] + "/raw", 1);
    }
    color_enable_srv = nh.advertiseService("color_enable", &PointCloudColorDetector::enable_color, this);

    private_nh.param("TOLERANCE", TOLERANCE, 0.20);
    private_nh.param("MIN_CLUSTER_SIZE", MIN_CLUSTER_SIZE, 20);
    private_nh.param("MAX_CLUSTER_SIZE", MAX_CLUSTER_SIZE, 10000);
    update_hsv_params();
}

void PointCloudColorDetector::hsv_param_changed(color_detector_params::HsvConfig &config, uint32_t level) {
    only_show_mask_points = config.only_show_mask_points;
    for (size_t i = 0; i < colors.size(); i++) {
        if (colors[i] == "red") {
            config_hsvs[i].lower.h = config.LOWER_RED_H;
            config_hsvs[i].lower.s = config.LOWER_RED_S;
            config_hsvs[i].lower.v = config.LOWER_RED_V;
            config_hsvs[i].upper.h = config.UPPER_RED_H;
            config_hsvs[i].upper.s = config.UPPER_RED_S;
            config_hsvs[i].upper.v = config.UPPER_RED_V;
        } else if (colors[i] == "green") {
            config_hsvs[i].lower.h = config.LOWER_GREEN_H;
            config_hsvs[i].lower.s = config.LOWER_GREEN_S;
            config_hsvs[i].lower.v = config.LOWER_GREEN_V;
            config_hsvs[i].upper.h = config.UPPER_GREEN_H;
            config_hsvs[i].upper.s = config.UPPER_GREEN_S;
            config_hsvs[i].upper.v = config.UPPER_GREEN_V;
        } else if (colors[i] == "blue") {
            config_hsvs[i].lower.h = config.LOWER_BLUE_H;
            config_hsvs[i].lower.s = config.LOWER_BLUE_S;
            config_hsvs[i].lower.v = config.LOWER_BLUE_V;
            config_hsvs[i].upper.h = config.UPPER_BLUE_H;
            config_hsvs[i].upper.s = config.UPPER_BLUE_S;
            config_hsvs[i].upper.v = config.UPPER_BLUE_V;
        } else if (colors[i] == "yellow") {
            config_hsvs[i].lower.h = config.LOWER_YELLOW_H;
            config_hsvs[i].lower.s = config.LOWER_YELLOW_S;
            config_hsvs[i].lower.v = config.LOWER_YELLOW_V;
            config_hsvs[i].upper.h = config.UPPER_YELLOW_H;
            config_hsvs[i].upper.s = config.UPPER_YELLOW_S;
            config_hsvs[i].upper.v = config.UPPER_YELLOW_V;
        } else if (colors[i] == "white") {
            config_hsvs[i].lower.h = config.LOWER_WHITE_H;
            config_hsvs[i].lower.s = config.LOWER_WHITE_S;
            config_hsvs[i].lower.v = config.LOWER_WHITE_V;
            config_hsvs[i].upper.h = config.UPPER_WHITE_H;
            config_hsvs[i].upper.s = config.UPPER_WHITE_S;
            config_hsvs[i].upper.v = config.UPPER_WHITE_V;
        }
    }
}

void PointCloudColorDetector::update_hsv_params() {
    for (size_t i = 0; i < colors.size(); i++) {
        std::string uppercase_latter;
        uppercase_latter.resize(colors[i].size());
        std::transform(colors[i].begin(), colors[i].end(), uppercase_latter.begin(), toupper);
        private_nh.param("LOWER_" + uppercase_latter + "_H", config_hsvs[i].lower.h, 0);
        private_nh.param("LOWER_" + uppercase_latter + "_S", config_hsvs[i].lower.s, 0);
        private_nh.param("LOWER_" + uppercase_latter + "_V", config_hsvs[i].lower.v, 0);
        private_nh.param("UPPER_" + uppercase_latter + "_H", config_hsvs[i].upper.h, 0);
        private_nh.param("UPPER_" + uppercase_latter + "_S", config_hsvs[i].upper.s, 0);
        private_nh.param("UPPER_" + uppercase_latter + "_V", config_hsvs[i].upper.v, 0);
    }
}

bool PointCloudColorDetector::enable_color(color_detector_srvs::ColorEnable::Request &req, color_detector_srvs::ColorEnable::Response &res) {
    for (size_t i = 0; i < colors.size(); i++) {
        if (colors[i] == req.color) {
            use_colors[i] = req.enable;
        }
    }
    return true;
}

pcl::PointCloud<pcl::PointXYZRGB> PointCloudColorDetector::limit_point_cloud(const ThresholdHSV &thres_hsv, const pcl::PointCloud<pcl::PointXYZRGB> &pc) {
    pcl::PointCloud<pcl::PointXYZRGB> masked_pc;
    for (const auto &rgb_point : pc) {
        pcl::PointXYZHSV hsv_point;
        pcl::PointXYZRGBtoXYZHSV(rgb_point, hsv_point);

        // opencv 0 <= h <= 180, 0 <= s <= 255, 0 <= v <= 255
        // pcl 0 <= h <= 360, 0 <= s <= 1, 0 <= v <= 1
        if (thres_hsv.lower.h <= hsv_point.h / 2    && hsv_point.h / 2    <= thres_hsv.upper.h &&
            thres_hsv.lower.s <= hsv_point.s * 255. && hsv_point.s * 255. <= thres_hsv.upper.s &&
            thres_hsv.lower.v <= hsv_point.v * 255. && hsv_point.v * 255. <= thres_hsv.upper.v &&
            isfinite(hsv_point.x) && isfinite(hsv_point.y) && isfinite(hsv_point.z)) {
            masked_pc.push_back(rgb_point);
        }
    }
    ROS_DEBUG_STREAM("lower hsv param : " << thres_hsv.lower.h << ' ' << thres_hsv.lower.s << ' ' << thres_hsv.lower.v);
    ROS_DEBUG_STREAM("upper hsv param : " << thres_hsv.upper.h << ' ' << thres_hsv.upper.s << ' ' << thres_hsv.upper.v);
    return masked_pc;
}

std::vector<pcl::PointIndices> PointCloudColorDetector::euclidean_clustering(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> &pc) {
    pcl::shared_ptr<pcl::search::KdTree<pcl::PointXYZRGB>> tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(pc);

    std::vector<pcl::PointIndices> pc_indices;
    pcl::shared_ptr<pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>> ec(new pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>);
    ec->setClusterTolerance(TOLERANCE);
    ec->setMinClusterSize(MIN_CLUSTER_SIZE);
    ec->setMaxClusterSize(MAX_CLUSTER_SIZE);
    ec->setSearchMethod(tree);
    ec->setInputCloud(pc);
    ec->extract(pc_indices);

    return pc_indices;
}

pcl::PointCloud<pcl::PointXYZRGB> PointCloudColorDetector::get_lagest_cluster(const pcl::PointCloud<pcl::PointXYZRGB> &pc, const std::vector<pcl::PointIndices> &pc_indices) {
    size_t max_cluster_size = 0;
    pcl::PointIndices point_indices;
    for (const auto &indices : pc_indices) {
        if (max_cluster_size < indices.indices.size()) {
            max_cluster_size = indices.indices.size();
            point_indices = std::move(indices);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB> target_pc;
    for (const auto itr : point_indices.indices) {
        target_pc.push_back(pc.points.at(itr));
    }
    return target_pc;
}

color_detector_msgs::TargetPosition PointCloudColorDetector::calc_target_position(const  pcl::PointCloud<pcl::PointXYZRGB> &pc) {
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    int finite_count = 0;
    for (const auto &point : pc) {
        if (isfinite(point.x) && isfinite(point.y) && isfinite(point.z)) {
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
            finite_count++;
        }
    }

    color_detector_msgs::TargetPosition target_position;
    target_position.x = sum_x / finite_count;
    target_position.y = sum_y / finite_count;
    target_position.z = sum_z / finite_count;
    target_position.cluster_num = finite_count;

    return target_position;
}

pcl::PointCloud<pcl::PointXYZRGB> PointCloudColorDetector::detect_target_cluster(const ThresholdHSV &thres_hsv,
                                                                                 const std_msgs::Header &header,
                                                                                 const ros::Publisher &masked_pc_pub,
                                                                                 const pcl::PointCloud<pcl::PointXYZRGB> &pc) {
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> masked_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    *masked_pc = limit_point_cloud(thres_hsv, pc);

    if (masked_pc->size() < MIN_CLUSTER_SIZE) {
        ROS_WARN_STREAM("The number of points limited by HSV is too small.");
        return pcl::PointCloud<pcl::PointXYZRGB>();
    }

    ROS_INFO_STREAM("masked cluster size : " << masked_pc->size());
    sensor_msgs::PointCloud2 ros_masked_pc;
    pcl::toROSMsg(*masked_pc, ros_masked_pc);
    ros_masked_pc.header = header;
    masked_pc_pub.publish(ros_masked_pc);

    if (only_show_mask_points) return pcl::PointCloud<pcl::PointXYZRGB>();

    std::vector<pcl::PointIndices> pc_indices = euclidean_clustering(masked_pc);
    pcl::PointCloud<pcl::PointXYZRGB> target_pc = get_lagest_cluster(*masked_pc, pc_indices);

    return target_pc;
}

void PointCloudColorDetector::sensor_callback(const sensor_msgs::PointCloud2ConstPtr &received_pc) {
    auto start_time = ros::Time::now();
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> rgb_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*received_pc, *rgb_pc);

    for (size_t i = 0; i < colors.size(); i++) {
        if (!use_colors[i]) continue;
        auto target_pc = detect_target_cluster(config_hsvs[i], received_pc->header, masked_pc_pubs[i], *rgb_pc);
        ROS_INFO_STREAM("target cluster size : " << target_pc.size());
        sensor_msgs::PointCloud2 ros_target_pc;
        pcl::toROSMsg(target_pc, ros_target_pc);
        ros_target_pc.header = received_pc->header;
        target_pc_pubs[i].publish(ros_target_pc);

        color_detector_msgs::TargetPosition target_position = calc_target_position(target_pc);
        target_position.header = received_pc->header;
        ROS_INFO_STREAM("finite cluster size : " << target_position.cluster_num);
        target_position_pub.publish(target_position);
    }

    ROS_INFO_STREAM("[sensor_callback] elasped time : " << (ros::Time::now() - start_time).toSec() << "[sec]");
}

void PointCloudColorDetector::process() {
    ros::spin();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "point_cloud_color_detector");
    PointCloudColorDetector point_cloud_color_detector;
    point_cloud_color_detector.process();
    return 0;
}

#include "point_cloud_color_detector/point_cloud_color_detector.h"

PointCloudColorDetector::PointCloudColorDetector() : PointCloudColorDetector(ros::NodeHandle(), ros::NodeHandle("~")) {}
PointCloudColorDetector::PointCloudColorDetector(ros::NodeHandle nh, ros::NodeHandle private_nh) : nh_(nh), private_nh_(private_nh) {
    color_detector_params_hsv::init(colors_, param_hsvs_);
    use_colors_.assign(colors_.size(), false);

    pc_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PointCloudColorDetector::sensor_callback, this);
    target_position_pub_ = nh_.advertise<color_detector_msgs::TargetPosition>("/target/position", 1);
    masked_pc_pubs_.resize(colors_.size());
    target_pc_pubs_.resize(colors_.size());
    for (size_t i = 0; i < colors_.size(); i++) {
        masked_pc_pubs_[i] = private_nh_.advertise<sensor_msgs::PointCloud2>("/cloud/masked/" + colors_[i] + "/raw", 1);
        target_pc_pubs_[i] = private_nh_.advertise<sensor_msgs::PointCloud2>("/cloud/target/" + colors_[i] + "/raw", 1);
    }
    color_enable_srv_ = nh_.advertiseService("color_enable", &PointCloudColorDetector::enable_color, this);

    private_nh_.param("TOLERANCE", TOLERANCE, 0.20);
    private_nh_.param("HIGHEST_TARGET_Y", HIGHEST_TARGET_Y, 1000.0);
    private_nh_.param("LOWEREST_TARGET_Y", LOWEREST_TARGET_Y, -1000.0);
    private_nh_.param("MIN_CLUSTER_SIZE", MIN_CLUSTER_SIZE, 20);
    private_nh_.param("MAX_CLUSTER_SIZE", MAX_CLUSTER_SIZE, 10000);
    private_nh_.param("ONLY_PUBLISH_MASK_POINTS", only_publish_mask_points_, false);
    private_nh_.param("PUBLISH_TARGET_POINTS", publish_target_points_, false);
    set_hsv_params();
}

void PointCloudColorDetector::set_hsv_params() {
    for (size_t i = 0; i < colors_.size(); i++) {
        std::string uppercase_latter;
        uppercase_latter.resize(colors_[i].size());
        std::transform(colors_[i].begin(), colors_[i].end(), uppercase_latter.begin(), toupper);
        private_nh_.param("LOWER_" + uppercase_latter + "_H", param_hsvs_[i].lower.h, 0);
        private_nh_.param("LOWER_" + uppercase_latter + "_S", param_hsvs_[i].lower.s, 0);
        private_nh_.param("LOWER_" + uppercase_latter + "_V", param_hsvs_[i].lower.v, 0);
        private_nh_.param("UPPER_" + uppercase_latter + "_H", param_hsvs_[i].upper.h, 0);
        private_nh_.param("UPPER_" + uppercase_latter + "_S", param_hsvs_[i].upper.s, 0);
        private_nh_.param("UPPER_" + uppercase_latter + "_V", param_hsvs_[i].upper.v, 0);
    }
}

bool PointCloudColorDetector::enable_color(color_detector_srvs::ColorEnable::Request &req,
                                           color_detector_srvs::ColorEnable::Response &res) {
    for (size_t i = 0; i < colors_.size(); i++) {
        if (colors_[i] == req.color) {
            use_colors_[i] = req.enable;
        }
    }
    return true;
}

pcl::PointCloud<pcl::PointXYZRGB> PointCloudColorDetector::limit_point_cloud(
    const ThresholdHSV &thres_hsv, const pcl::PointCloud<pcl::PointXYZRGB> &pc) {
    pcl::PointCloud<pcl::PointXYZRGB> masked_pc;
    for (const auto &rgb_point : pc) {
        pcl::PointXYZHSV hsv_point;
        pcl::PointXYZRGBtoXYZHSV(rgb_point, hsv_point);

        // opencv 0 <= h <= 180, 0 <= s <= 255, 0 <= v <= 255
        // pcl 0 <= h <= 360, 0 <= s <= 1, 0 <= v <= 1
        if (thres_hsv.lower.h <= hsv_point.h / 2 && hsv_point.h / 2 <= thres_hsv.upper.h &&
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

std::vector<pcl::PointIndices> PointCloudColorDetector::euclidean_clustering(
    const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> &pc) {
    pcl::shared_ptr<pcl::search::KdTree<pcl::PointXYZRGB>> tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(pc);

    std::vector<pcl::PointIndices> pc_indices;
    pcl::shared_ptr<pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>> ec(
        new pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>);
    ec->setClusterTolerance(TOLERANCE);
    ec->setMinClusterSize(MIN_CLUSTER_SIZE);
    ec->setMaxClusterSize(MAX_CLUSTER_SIZE);
    ec->setSearchMethod(tree);
    ec->setInputCloud(pc);
    ec->extract(pc_indices);

    return pc_indices;
}

pcl::PointCloud<pcl::PointXYZRGB> PointCloudColorDetector::get_target_cluster(
    const pcl::PointCloud<pcl::PointXYZRGB> &pc, std::vector<pcl::PointIndices> &pc_indices) {
    static auto comp = [](const pcl::PointIndices &a, const pcl::PointIndices &b) -> bool {
        return a.indices.size() > b.indices.size();
    };
    sort(pc_indices.begin(), pc_indices.end(), comp);

    pcl::PointCloud<pcl::PointXYZRGB> target_pc;
    for (const auto &point_indices : pc_indices) {
        bool is_target = true;
        pcl::PointCloud<pcl::PointXYZRGB> cluster;
        for (const auto &itr : point_indices.indices) {
            cluster.push_back(pc.points.at(itr));
            if (cluster.back().y < LOWEREST_TARGET_Y || HIGHEST_TARGET_Y < cluster.back().y) {
                is_target = false;
                break;
            }
        }
        if (is_target) {
            target_pc = std::move(cluster);
            break;
        }
    }
    return target_pc;
}

color_detector_msgs::TargetPosition PointCloudColorDetector::calc_target_position(
    const pcl::PointCloud<pcl::PointXYZRGB> &pc) {
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

pcl::PointCloud<pcl::PointXYZRGB> PointCloudColorDetector::detect_target_cluster(
    const ThresholdHSV &thres_hsv, const std_msgs::Header &header, const ros::Publisher &masked_pc_pub,
    const pcl::PointCloud<pcl::PointXYZRGB> &pc) {
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> masked_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    *masked_pc = limit_point_cloud(thres_hsv, pc);

    if (masked_pc->size() < MIN_CLUSTER_SIZE) {
        ROS_WARN_STREAM("The number of points limited by HSV is too small.");
        return pcl::PointCloud<pcl::PointXYZRGB>();
    }
    ROS_DEBUG_STREAM("masked cluster size : " << masked_pc->size());

    if (only_publish_mask_points_) {
        sensor_msgs::PointCloud2 ros_masked_pc;
        pcl::toROSMsg(*masked_pc, ros_masked_pc);
        ros_masked_pc.header = header;
        masked_pc_pub.publish(ros_masked_pc);
        return pcl::PointCloud<pcl::PointXYZRGB>();
    }

    std::vector<pcl::PointIndices> pc_indices = euclidean_clustering(masked_pc);
    pcl::PointCloud<pcl::PointXYZRGB> target_pc = get_target_cluster(*masked_pc, pc_indices);

    return target_pc;
}

void PointCloudColorDetector::sensor_callback(const sensor_msgs::PointCloud2ConstPtr &received_pc) {
    auto start_time = ros::Time::now();
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> rgb_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*received_pc, *rgb_pc);

    for (size_t i = 0; i < colors_.size(); i++) {
        if (!use_colors_[i]) continue;
        auto target_pc = detect_target_cluster(param_hsvs_[i], received_pc->header, masked_pc_pubs_[i], *rgb_pc);
        ROS_DEBUG_STREAM("target cluster size : " << target_pc.size());

        if (target_pc.empty()) continue;

        color_detector_msgs::TargetPosition target_position = calc_target_position(target_pc);
        target_position.header = received_pc->header;
        ROS_DEBUG_STREAM("finite cluster size : " << target_position.cluster_num);
        target_position_pub_.publish(target_position);

        if (publish_target_points_) {
            sensor_msgs::PointCloud2 ros_target_pc;
            pcl::toROSMsg(target_pc, ros_target_pc);
            ros_target_pc.header = received_pc->header;
            target_pc_pubs_[i].publish(ros_target_pc);
            save_csv(target_position);
        }
    }

    ROS_INFO_STREAM("[point_cloud_color_detector:sensor_callback] elasped time : " << (ros::Time::now() - start_time).toSec() << "[sec]");
}

void PointCloudColorDetector::save_csv(const color_detector_msgs::TargetPosition &target_position) {
    static auto start_time = ros::Time::now();
    static std::ofstream ofs("/home/amsl/dist.csv");
    double x = target_position.x;
    double y = target_position.y;
    double z = target_position.z;
    if (!only_publish_mask_points_)
        ofs << (ros::Time::now() - start_time).toSec() << ',' << sqrt(x * x + z * z) << ',' << target_position.cluster_num << ',' << x << ',' << y << ',' << z << std::endl;
    return;
}

void PointCloudColorDetector::process() { ros::spin(); }

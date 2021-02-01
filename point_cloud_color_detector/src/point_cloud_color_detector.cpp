#include "point_cloud_color_detector/point_cloud_color_detector.h"

PointCloudColorDetector::PointCloudColorDetector() : private_nh("~"),
                                                     config_hsvs(colors.size()),
                                                     masked_pc_pubs(colors.size()),
                                                     target_pc_pubs(colors.size()) {
    pc_sub = nh.subscribe("/camera/depth_registered/points", 1, &PointCloudColorDetector::sensor_callback, this);
    target_position_pub = nh.advertise<color_detector_msgs::TargetPosition>("/target/position", 1);
    for (size_t i = 0; i < colors.size(); i++) {
        masked_pc_pubs[i] = private_nh.advertise<sensor_msgs::PointCloud2>("/cloud/masked/" + colors[i] + "/raw", 1);
        target_pc_pubs[i] = private_nh.advertise<sensor_msgs::PointCloud2>("/cloud/target/" + colors[i] + "/raw", 1);
    }

    private_nh.param("TOLERANCE", TOLERANCE, 0.20);
    private_nh.param("MIN_CLUSTER_SIZE", MIN_CLUSTER_SIZE, 20);
    private_nh.param("MAX_CLUSTER_SIZE", MAX_CLUSTER_SIZE, 10000);
    for (size_t i = 0; i < colors.size(); i++) {
        std::string uppercase_latter;
        uppercase_latter.resize(colors[i].size());
        std::transform(colors[i].begin(), colors[i].end(), uppercase_latter.begin(), toupper);
        ROS_INFO_STREAM_ONCE(uppercase_latter);
        private_nh.param("LOWER_" + uppercase_latter + "_H", config_hsvs[i].lower.h, 0);
        private_nh.param("LOWER_" + uppercase_latter + "_S", config_hsvs[i].lower.s, 0);
        private_nh.param("LOWER_" + uppercase_latter + "_V", config_hsvs[i].lower.v, 0);
        private_nh.param("UPPER_" + uppercase_latter + "_H", config_hsvs[i].upper.h, 0);
        private_nh.param("UPPER_" + uppercase_latter + "_S", config_hsvs[i].upper.s, 0);
        private_nh.param("UPPER_" + uppercase_latter + "_V", config_hsvs[i].upper.v, 0);
    }
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

void PointCloudColorDetector::detect_target_position(const ThresholdHSV &thres_hsv,
                                                     const std_msgs::Header &header,
                                                     const ros::Publisher &masked_pc_pub,
                                                     const ros::Publisher &target_pc_pub,
                                                     const pcl::PointCloud<pcl::PointXYZRGB> &pc) {
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> masked_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    *masked_pc = limit_point_cloud(thres_hsv, pc);

    if (masked_pc->size() < MIN_CLUSTER_SIZE) {
        ROS_WARN_STREAM("The number of points limited by HSV is too small.");
        return;
    }

    ROS_INFO_STREAM("masked cluster size : " << masked_pc->size());
    sensor_msgs::PointCloud2 ros_masked_pc;
    pcl::toROSMsg(*masked_pc, ros_masked_pc);
    ros_masked_pc.header = header;
    masked_pc_pub.publish(ros_masked_pc);

    std::vector<pcl::PointIndices> pc_indices = euclidean_clustering(masked_pc);
    pcl::PointCloud<pcl::PointXYZRGB> target_pc = get_lagest_cluster(*masked_pc, pc_indices);

    ROS_INFO_STREAM("target cluster size : " << target_pc.size());
    sensor_msgs::PointCloud2 ros_target_pc;
    pcl::toROSMsg(target_pc, ros_target_pc);
    ros_target_pc.header = header;
    target_pc_pub.publish(ros_target_pc);

    color_detector_msgs::TargetPosition target_position = calc_target_position(target_pc);
    target_position.header = header;
    ROS_INFO_STREAM("finite cluster size : " << target_position.cluster_num);
    target_position_pub.publish(target_position);
}

void PointCloudColorDetector::sensor_callback(const sensor_msgs::PointCloud2ConstPtr &received_pc) {
    auto start_time = ros::Time::now();
    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> rgb_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*received_pc, *rgb_pc);

    for (size_t i = 0; i < colors.size(); i++) {
        detect_target_position(config_hsvs[i], received_pc->header, masked_pc_pubs[i], target_pc_pubs[i], *rgb_pc);
        break;
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

#include "pointcloud_segmentation/environment.hpp"

using std::placeholders::_1;

environment::environment() : Node("lidar_environment") {
    RCLCPP_INFO(get_logger(), "Init Lidar Environment Node");

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(5));

    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "livox/lidar", qos_profile, std::bind(&environment::pointcloud_cb, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/environment", 2);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr environment::FilterCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr     cloud,
    float                                   filterRes,
    Eigen::Vector4f                         minPoint,
    Eigen::Vector4f                         maxPoint)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Filter Region of interest
    pcl::CropBox<pcl::PointXYZ> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudFiltered);

    // Filter Points on the Robot Roof Top
    std::vector<int> indices;
    pcl::CropBox<pcl::PointXYZ> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, 0.0, 1));
    roof.setInputCloud(cloudFiltered);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }

    // Remove Roof Points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudFiltered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudFiltered);

    return cloudFiltered;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr environment::detectObject(
    pcl::PointCloud<pcl::PointXYZ>::Ptr         cloud,
    bool                                        view,
    std::pair<Eigen::Vector3f, Eigen::Vector3f> plane1,
    std::pair<Eigen::Vector3f, Eigen::Vector3f> plane2)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr detectPoints (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<Plane> planes;
    std::vector<Eigen::Vector3f> vertices;

    vertices.push_back(plane1.first);
    vertices.push_back(Eigen::Vector3f(plane1.first(0), plane1.first(1), plane1.second(2)));
    vertices.push_back(plane1.second);
    vertices.push_back(Eigen::Vector3f(plane1.first(0), plane1.second(1), plane1.first(2)));
    vertices.push_back(plane2.first);
    vertices.push_back(Eigen::Vector3f(plane2.first(0), plane2.first(1), plane2.second(2)));
    vertices.push_back(plane2.second);
    vertices.push_back(Eigen::Vector3f(plane2.first(0), plane2.second(1), plane2.first(2)));

    planes.push_back(getPlane(vertices[0], vertices[3], vertices[1]));
    planes.push_back(getPlane(vertices[4], vertices[5], vertices[7]));
    planes.push_back(getPlane(vertices[0], vertices[1], vertices[4]));
    planes.push_back(getPlane(vertices[2], vertices[3], vertices[6]));
    planes.push_back(getPlane(vertices[0], vertices[4], vertices[3]));
    planes.push_back(getPlane(vertices[1], vertices[2], vertices[5]));

    for (auto elem : cloud->points) {
        Eigen::Vector3f point(elem.x,elem.y,elem.z);
        if (isPointInside(planes, point)) {
            detectPoints->points.push_back(elem);
        }
    }
    return detectPoints;
}

Plane environment::getPlane(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) {
    Plane plane;
    plane.normal = (p1 - p0).cross(p2 - p0).normalized();
    plane.d = -plane.normal.dot(p0);
    return plane;
}

bool environment::isPointInside(const std::vector<Plane>& planes, Eigen::Vector3f point) {
    for (const auto& plane : planes) {
        if (plane.normal.dot(point) + plane.d < 0){
            return false;
        }
    }
    return true;
}

void environment::pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::msg::PointCloud2 cloud_out;

    pcl::fromROSMsg(*cloud_msg, *cloud_dst);

    cloud_filtered = FilterCloud(
        cloud_dst,
        0.3 ,
        Eigen::Vector4f (-20, -6, -3, 1),
        Eigen::Vector4f (30, 7, 2, 1));
    pcl::toROSMsg(*cloud_filtered,cloud_out);

    std::pair<Eigen::Vector3f, Eigen::Vector3f> plane1;
    std::pair<Eigen::Vector3f, Eigen::Vector3f> plane2;
    plane1 = std::make_pair(Eigen::Vector3f(1, 1, 1), Eigen::Vector3f(1, -1, -1));
    plane2 = std::make_pair(Eigen::Vector3f(3, 1, 1), Eigen::Vector3f(3, -1, -1));
    cloud_filtered = detectObject(
        cloud_filtered,
        false,
        plane1,
        plane2
    );
    pcl::toROSMsg(*cloud_filtered,cloud_out);
    cloud_out.header.frame_id = cloud_msg->header.frame_id;
    cloud_out.header.stamp = cloud_msg->header.stamp;

    publisher_->publish(cloud_out);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<environment>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

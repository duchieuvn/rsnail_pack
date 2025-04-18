#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudToImage : public rclcpp::Node {
public:
    PointCloudToImage() : Node("pointcloud_to_image") {
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_pointcloud", 10,
            std::bind(&PointCloudToImage::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        int width = msg->width;
        int height = msg->height;
        RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", cloud->size());
        RCLPP_INFO(this->get_logger(), "Width: %d, Height: %d", width, height);
        std::vector<std::vector<double>> depth_array(height, std::vector<double>(width, 0.0));

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            const auto &point = cloud->points[i];
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                int u = i % width;  // Column index
                int v = i / width;  // Row index
                if (u >= 0 && u < width && v >= 0 && v < height) {
                    depth_array[v][u] = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Converted point cloud to depth image");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToImage>());
    rclcpp::shutdown();
    return 0;
}

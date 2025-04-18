#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer()
        : Node("pointcloud_transformer"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
	// Declare and retrieve the input topic parameter
        this->declare_parameter<std::string>("input_topic", "input_pointcloud");
	    this->declare_parameter<bool>("do_transform", true);
        input_topic_ = this->get_parameter("input_topic").as_string();
	    do_transform = this->get_parameter("do_transform").as_bool();
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10,
            std::bind(&PointCloudTransformer::pointcloud_callback, this, std::placeholders::_1));

        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(input_topic_ + "/transformed", 10);
    }

private:
    std::string input_topic_;
    bool do_transform;
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try
        {
            std::string target_frame = "base_link";
            auto transform = tf_buffer_.lookupTransform(
                target_frame, msg->header.frame_id, tf2::TimePointZero);

	    if(!do_transform){
	    	transform.transform.translation.x = 0;
		transform.transform.translation.y = 0;
		transform.transform.translation.z = 0;
	    }
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*msg, transformed_cloud, transform);

            transformed_cloud.header.frame_id = target_frame;
            transformed_cloud.header.stamp = this->get_clock()->now();
            pointcloud_pub_->publish(transformed_cloud);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTransformer>());
    rclcpp::shutdown();
    return 0;
}

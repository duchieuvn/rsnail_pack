#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <std_srvs/srv/set_bool.hpp>
#include <can_msgs/msg/frame.hpp>

#include "spline.h" // Using tk::spline for cubic interpolation

#define CAR_LENGTH 2.0

using SetBool = std_srvs::srv::SetBool;

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node"), car_position_{0.0, 0.0} {
        // Create the service and bind the service callback.
        service_ = this->create_service<SetBool>(
          "set_asEngaged",
          std::bind(&PurePursuitNode::handle_set_flag, this,
                    std::placeholders::_1, std::placeholders::_2));

        vsu_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "vsu_distance",10,std::bind(&PurePursuitNode::distance_callback, this, std::placeholders::_1));
        track_subscription_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "path", 10, std::bind(&PurePursuitNode::track_callback, this, std::placeholders::_1));
        car_position_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "car_position_topic", 10, std::bind(&PurePursuitNode::car_position_callback, this, std::placeholders::_1));
        controller_publisher_ = this->create_publisher<can_msgs::msg::Frame>("send_to_sensor", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(2) , std::bind(&PurePursuitNode::timer_callback, this));
    }

private:
    std::vector<std::pair<double, double>> track_;
    std::pair<double, double> car_position_ = {0.0, 0.0};
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vsu_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr track_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr car_position_subscription_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr controller_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool as_engage_flag_ = false;
    bool timer_elapsed = false;
    rclcpp::Service<SetBool>::SharedPtr service_;

    void timer_callback()
    {
        timer_elapsed = true;
    }

    void distance_callback(const std_msgs::msg::Float32 msg)
    {
        //if (!timer_elapsed)
        //{
            if (msg.data > 80.) //Turn AS off after 80m
            {
                as_engage_flag_ = false;

                can_msgs::msg::Frame can_frame = can_msgs::msg::Frame();
                can_frame.id = 0x202;
                can_frame.dlc = 8;
                can_frame.data[0] = -99;
                can_frame.data[2] = 1 << 1; // Send Flags to 1 = Brake Pressed R
                controller_publisher_->publish(can_frame);
            }
        //}
    }

    void handle_set_flag(
    const std::shared_ptr<SetBool::Request> request,
    std::shared_ptr<SetBool::Response> response)
    {
        as_engage_flag_ = request->data;  // Update the flag based on the service request.
        response->success = true;
        response->message = as_engage_flag_ ? "AS engage set to true" : "AS engage set to false";

        RCLCPP_INFO(this->get_logger(), "Service call: %s", response->message.c_str());
    }

    void track_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        track_.clear();

        std::vector<double> x, y;

        for (const auto& point : msg->polygon.points) {
            x.push_back(point.x);
            y.push_back(point.y);
            RCLCPP_INFO(this->get_logger(), "new point: %f %f", point.x, point.y);
        }

        if (x.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Not enough points for spline interpolation.");
            return;
        }

        // Create cubic spline interpolator
        tk::spline spline_x, spline_y;
        std::vector<double> t(x.size());
        std::iota(t.begin(), t.end(), 0);

        spline_x.set_points(t, x);
        spline_y.set_points(t, y);

        // Generate interpolated points
        track_.clear();
        for (double i = 0; i <= t.back(); i += 0.1) {
            track_.emplace_back(spline_x(i), spline_y(i));
            //RCLCPP_INFO(this->get_logger(), "new point: %f %f", spline_x(i), spline_y(i));
        }
        run_pure_pursuit();
    }

    void car_position_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        car_position_ = {msg->x, msg->y};
        RCLCPP_INFO(this->get_logger(), "New car position");
        run_pure_pursuit();
    }

    size_t point_projection(const std::pair<double, double>& position_car) {
        size_t closest_index = 0;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < track_.size(); i++) {
            double dx = track_[i].first - position_car.first;
            double dy = track_[i].second - position_car.second;
            double distance = std::hypot(dx, dy);
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }
        return closest_index;
    }

    std::pair<std::pair<double, double>, double> lookahead_point(size_t projection_car, const std::pair<double, double>& position_car) {
        while (projection_car < track_.size() - 1 &&
               std::hypot(track_[projection_car].first - position_car.first,
                          track_[projection_car].second - position_car.second) < 0.25) {
            projection_car++;
        }
        double lookahead_distance = std::hypot(track_[projection_car].first - position_car.first,
                                               track_[projection_car].second - position_car.second);
        return {track_[projection_car], lookahead_distance};
    }

    double pure_pursuit(const std::pair<double, double>& position_car, const std::pair<double, double>& control_point, double lookahead) {
        double alpha = std::atan2(control_point.second - position_car.second, control_point.first - position_car.first);
        double delta = std::atan2(2.0 * CAR_LENGTH * std::sin(alpha) / lookahead, 1.0);
        return delta;
    }

    void run_pure_pursuit() {
        if (track_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Track is empty, cannot run pure pursuit.");
            return;
        }
        size_t projection = point_projection(car_position_);
        RCLCPP_INFO(this->get_logger(), "new point: %f %f", this->track_[projection], this->track_[projection]);
        auto control_data = lookahead_point(projection, car_position_);
        RCLCPP_INFO(this->get_logger(), "new lookahead: %f %f", control_data.first.first, control_data.first.second);
        const float steering_angle = - static_cast<float>(pure_pursuit(car_position_, control_data.first, control_data.second));

        const float steering_deg = (steering_angle / M_PI) * 180;
        const float steering_min = -120;
        const float steering_max = 120;
        float steering_clipped;
        if (steering_deg < steering_min) {
            steering_clipped = steering_min;
        }
        else if (steering_deg > steering_max) {
            steering_clipped = steering_max;
        }
        else {
            steering_clipped = steering_deg;
        }
        steering_clipped = steering_clipped / 120.f;

        std_msgs::msg::Float64 steering_msg;
        steering_msg.data = steering_clipped;

        can_msgs::msg::Frame can_frame = can_msgs::msg::Frame();
        can_frame.id = 0x202;
        can_frame.dlc = 8;
        can_frame.data[0] = 99;
        can_frame.data[2] = 1 << 3 | 1 << 2 | 1 << 0; // Send Flags to ACU 3 = AS Engaged, 2 = Release Brake NR, 0 Release Brake R
        memcpy(&can_frame.data[4], &steering_clipped, sizeof(steering_clipped));
        if (as_engage_flag_) {
            controller_publisher_->publish(can_frame); // Publish steering angle
        }

        RCLCPP_INFO(this->get_logger(), "Steering Angle clipped: %f", steering_clipped);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

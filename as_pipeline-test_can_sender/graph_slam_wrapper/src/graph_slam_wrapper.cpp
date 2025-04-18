#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>


class GraphSlamWrapper final : public rclcpp::Node {
public:
    GraphSlamWrapper() : Node("graph_slam_wrapper") {
        this->landmarkSubscriber_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/landmarks/filtered",
            10,
            std::bind(&GraphSlamWrapper::landmark_callback, this, std::placeholders::_1)
        );

        this->odometrySubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered",
            10,
            std::bind(&GraphSlamWrapper::odometry_callback, this, std::placeholders::_1)
        );

        this->posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/slam/pose",
            10
        );

        this->mapPublisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/slam/map",
            10
        );

        this->last_pose_odom_ = geometry_msgs::msg::Pose();

        // gtsam initialization
        this->last_pose_slam_ = gtsam::Pose2();
        this->landmark_next_index = 0;
        this->pose_next_index = 1;
        this->graph_ = gtsam::NonlinearFactorGraph();
        const auto initial_pose = gtsam::Pose2(0, 0, 0);
        this->graph_.add(gtsam::PriorFactor<gtsam::Pose2>(
            gtsam::Symbol('x', this->pose_next_index),
            initial_pose,
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.01))
        ));
        this->initialEstimate_.insert(gtsam::Symbol('x', this->pose_next_index), initial_pose);
        this->pose_next_index++;
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr landmarkSubscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr mapPublisher_;

    geometry_msgs::msg::Pose last_pose_odom_;

    // gtsam stuff
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initialEstimate_;
    unsigned pose_next_index;
    unsigned landmark_next_index;
    struct Landmark {
        gtsam::Point2 position;
        unsigned index;
    };
    std::vector<Landmark> landmark_map_{};
    gtsam::Pose2 last_pose_slam_;

    void landmark_callback(const geometry_msgs::msg::PolygonStamped& landmark_msg) {
        RCLCPP_INFO(this->get_logger(), "landmark_callback at %f", this->now().seconds());

        // pose update
        {
            const auto new_pose = gtsam::Pose2(
               this->last_pose_odom_.position.x,
               this->last_pose_odom_.position.y,
               tf2::getYaw(this->last_pose_odom_.orientation)
           );

            const gtsam::Pose2 prev_pose = this->last_pose_slam_;
            const gtsam::Pose2 odometry_delta = prev_pose.between(new_pose);

            // add pose constraint
            const gtsam::Vector3 pose_noise(0.1, 0.1, 0.01);
            this->graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
                gtsam::Symbol('x', this->pose_next_index - 1),
                gtsam::Symbol('x', this->pose_next_index),
                odometry_delta,
                gtsam::noiseModel::Diagonal::Sigmas(pose_noise)
            ));

            RCLCPP_INFO(this->get_logger(), "pose index %u", this->pose_next_index);
            RCLCPP_INFO(this->get_logger(), "poses at x:%f y:%f", new_pose.x(), new_pose.y());


            this->initialEstimate_.insert(gtsam::Symbol('x', this->pose_next_index), new_pose);
            this->pose_next_index++;
        }

        RCLCPP_INFO(this->get_logger(), "pose updated at %f", this->now().seconds());

        // landmark update
        {
            constexpr float new_landmark_threshold = std::pow(0.5, 2);

            for (auto& observed_landmark: landmark_msg.polygon.points) {
                RCLCPP_INFO(this->get_logger(), "landmark: %f, %f", observed_landmark.x, observed_landmark.y);

                std::vector<float> distances;
                distances.emplace_back(INFINITY);
                for (auto&[position, index]: this->landmark_map_) {
                    distances.emplace_back(
                        std::pow(observed_landmark.x - position.x(), 2) +
                        std::pow(observed_landmark.y - position.y(), 2));
                }

                RCLCPP_INFO(this->get_logger(), "Distances calculated");

                // get index of the minimum distance
                const auto min_distance_it = std::min_element(distances.begin(), distances.end());

                auto data_association_inx = std::distance(distances.begin(), min_distance_it) - 1 <= -1 ?
                    0 :
                    this->landmark_map_[std::distance(distances.begin(), min_distance_it) - 1].index;

                RCLCPP_INFO(this->get_logger(), "min_distance: %f", *min_distance_it);

                if (*min_distance_it > new_landmark_threshold) {
                    // new landmark
                    gtsam::Point2 new_landmark(observed_landmark.x, observed_landmark.y);
                    this->initialEstimate_.insert(gtsam::Symbol('l', this->landmark_next_index), new_landmark);
                    data_association_inx = this->landmark_next_index;
                    this->landmark_next_index++;
                }
                else {
                    // old landmark
                    // -> nothing to do
                }

                RCLCPP_INFO(this->get_logger(), "data association: %u", data_association_inx);

                this->graph_.add(
                    gtsam::BetweenFactor<gtsam::Point2>(
                        gtsam::Symbol('x', this->pose_next_index - 1),
                        gtsam::Symbol('l', data_association_inx),
                        gtsam::Point2(observed_landmark.x, observed_landmark.y),
                        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.1))
                    )
                );
            }
        }

        RCLCPP_INFO(this->get_logger(), "landmarks updated at %f", this->now().seconds());

        // run optimization
        {
            gtsam::LevenbergMarquardtOptimizer optimizer(this->graph_, this->initialEstimate_);
            const gtsam::Values result = optimizer.optimize();

            RCLCPP_INFO(this->get_logger(), "optimizer finished at %f", this->now().seconds());


            // get landmark and pose data
            this->last_pose_slam_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', this->pose_next_index - 1));

            this->landmark_map_.clear();
            for (unsigned i = 0; i < this->landmark_next_index; i++) {
                if (result.exists(gtsam::Symbol('l', i))) {
                    this->landmark_map_.emplace_back(Landmark{
                        result.at<gtsam::Point2>(gtsam::Symbol('l', i)),
                        i
                    });
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "optim finished at %f", this->now().seconds());

        // publish results
        {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = this->last_pose_slam_.x();
            pose_msg.pose.position.y = this->last_pose_slam_.y();
            pose_msg.pose.position.z = 0;
            // pose_msg.pose.orientation = tf2::toMsg<tf2::Quaternion, geometry_msgs::msg::Quaternion>(
            //     tf2::Quaternion(tf2::Vector3(0, 0, 1), this->last_pose_slam_.theta())
            // );
            const auto tf_quaternion = tf2::Quaternion(tf2::Vector3(0, 0, 1), this->last_pose_slam_.theta());
            pose_msg.pose.orientation.set__x(tf_quaternion.x());
            pose_msg.pose.orientation.set__y(tf_quaternion.y());
            pose_msg.pose.orientation.set__z(tf_quaternion.z());
            pose_msg.pose.orientation.set__w(tf_quaternion.w());
            this->posePublisher_->publish(pose_msg);

            geometry_msgs::msg::PolygonStamped polygon_msg;
            polygon_msg.header.stamp = this->now();
            polygon_msg.header.frame_id = "map";
            for (const auto&[position, index]: this->landmark_map_) {
                geometry_msgs::msg::Point32 landmark_point32;
                landmark_point32.x = position.x();
                landmark_point32.y = position.y();
                landmark_point32.z = 0;
                polygon_msg.polygon.points.push_back(landmark_point32);
            }
            this->mapPublisher_->publish(polygon_msg);
        }

        RCLCPP_INFO(this->get_logger(), "results published at %f", this->now().seconds());
    }

    void odometry_callback(const nav_msgs::msg::Odometry& odom_msg) {
        this->last_pose_odom_ = odom_msg.pose.pose;
        RCLCPP_DEBUG(this->get_logger(), "Odom received: %f, %f", this->last_pose_odom_.position.x, this->last_pose_odom_.position.y);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraphSlamWrapper>());
    rclcpp::shutdown();
    return 0;
}

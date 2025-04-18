#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <Eigen/Dense>

//Selects whether to filter the landmarks via distance (0) to track or distance by itself(1)
#define FILTERMODE 1

class PerceptionFilter final : public rclcpp::Node {
public:
    PerceptionFilter() : Node("perception_filter") {
        this->landmarkDetectionsSubscriber_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/landmarks/raw", 10,
            std::bind(&PerceptionFilter::landmark_callback, this, std::placeholders::_1)
        );
        this->filteredLandmarkPublisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/landmarks/filtered", 10
        );
    }

private:
    void landmark_callback(const geometry_msgs::msg::PolygonStamped &poly_msg) const {
        //RCLCPP_INFO(get_logger(), "Data received: %zu points", poly_msg.polygon.points.size());
        // Filter the landmarks
        geometry_msgs::msg::PolygonStamped filtered_landmarks;
        filtered_landmarks.header.frame_id = poly_msg.header.frame_id;
        filtered_landmarks.header.stamp = poly_msg.header.stamp;
        if constexpr (FILTERMODE == 0) {
            for (auto &point: poly_msg.polygon.points) {
                if (point.y < 3 && point.y > -3) {
                    filtered_landmarks.polygon.points.push_back(point);
                }
            }
        } else if constexpr (FILTERMODE == 1) {
            Eigen::MatrixXd points(poly_msg.polygon.points.size(), 2);
            for (size_t i = 0; i < poly_msg.polygon.points.size(); ++i) {
                points(i, 0) = poly_msg.polygon.points[i].x;
                points(i, 1) = poly_msg.polygon.points[i].y;
            }
            const double threshold = 3.;
            const unsigned n_neighbours = 2 + 1; // 2 neighbours + the point itself
            const int N = points.rows();

            RCLCPP_DEBUG(get_logger(),"Before filtering: %d points", N);

            // Compute squared Euclidean distance matrix using vectorized operations.
            // First, compute squared norms for each point (each row)
            Eigen::VectorXd sqNorms = points.rowwise().squaredNorm(); // (N x 1)

            // Form the pairwise squared distances:
            // distsSq(i,j) = ||points.row(i)||^2 + ||points.row(j)||^2 - 2*(points.row(i) dot points.row(j))
            Eigen::MatrixXd distsSq = sqNorms.replicate(1, N) +
                                      sqNorms.transpose().replicate(N, 1) -
                                      2 * points * points.transpose();

            // Compute the Euclidean distance matrix (clamping to zero to avoid negative values from numerical errors)
            Eigen::MatrixXd distance_matrix = distsSq.cwiseMax(0).cwiseSqrt();

            RCLCPP_DEBUG(get_logger(),"Distance matrix: %d x %d", distance_matrix.rows(), distance_matrix.cols());

            // Count the number of neighbors within the threshold distance for each point.
            // Using a vectorized row-wise operation: count = (distance < threshold).cast<int>().sum()
            Eigen::VectorXi neighbor_counts(N);
            for (int i = 0; i < N; ++i) {
                neighbor_counts(i) = (distance_matrix.row(i).array() < threshold).cast<int>().sum();
                if (neighbor_counts(i) <= n_neighbours){
                    // geometry_msgs::msg::Point32 point = geometry_msgs::msg::Point32();
                    // point.x = poly_msg.polygon.points[i].x;
                    // point.y = poly_msg.polygon.points[i].y;
                    // point.z = 0;

                    filtered_landmarks.polygon.points.push_back(poly_msg.polygon.points[i]);
                }
            }

        } else {
            RCLCPP_FATAL(get_logger(), "Invalid filter mode");
        }
        filteredLandmarkPublisher_->publish(filtered_landmarks);
    }

    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr landmarkDetectionsSubscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr filteredLandmarkPublisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionFilter>());
    rclcpp::shutdown();
    return 0;
}

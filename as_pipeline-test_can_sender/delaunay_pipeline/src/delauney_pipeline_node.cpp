#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Cartesian.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#define MAX_EDGE_LENGTH 7.0
#define MAX_VIEWING_DISTANCE 10
#define MAX_CAR_DISTANCE 7.0
#define CAR_X 0.0
#define CAR_Y 0.0
#define DEPTH 5
#define FACTOR_ANGLE 5
#define FACTOR_DISTANCE 1
#define MID_DISTANCE 3
#define MAX_DEVIATION 5


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Vector_2 Vector;
typedef K::Point_2 Point;

struct SearchState {
    Delaunay::Face_handle face;
    int edge_index{};
    float cost{};
    std::vector<Point> path;
};

struct CompareState {
    bool operator()(const SearchState &s1, const SearchState &s2) const {
        return s1.cost > s2.cost;
    }
};

// Computes the angle (in degrees) at point 'position', formed by segments (point - position) and (last_position - position).
double get_angle(const Point &point, const Point &position, const Point &last_position) {
    // Compute vectors from 'position' to the other two points.
    Vector v1 = point - position;
    Vector v2 = last_position - position;
    // Compute the dot product.
    double dot = v1 * v2;
    // Compute the lengths.
    double len1 = std::sqrt(v1.squared_length());
    double len2 = std::sqrt(v2.squared_length());
    // Avoid division by zero.
    if (len1 == 0 || len2 == 0)
        return 0.0;
    // Calculate the cosine of the angle.
    double cosine = dot / (len1 * len2);
    // Clamp the cosine value to the interval [-1,1] to avoid precision errors.
    cosine = std::max(-1.0, std::min(1.0, cosine));
    // Compute the angle in radians and convert to degrees.
    double angle_rad = std::acos(cosine);
    double angle_deg = angle_rad * 180.0 / M_PI;
    return angle_deg;
}

// Computes the Euclidean distance between two points.
double get_distance(const Point &a, const Point &b) {
    return std::sqrt(CGAL::squared_distance(a, b));
}

// Applies the cost function as described.
float apply_cost_function(const Point &point, const Point &position, const Point &last_position) {
    float costs = 0.;
    // Normalize the angle term.
    double angle = 180 - get_angle(point, position, last_position); //Straight line is 180 degrees
    if (angle > 70) {
        return INFINITY;
    }
    costs += FACTOR_ANGLE * (angle / 180.0);

    // Compute the distance term.
    double distance = get_distance(point, position);
    double deviation_distance = std::abs(distance - MID_DISTANCE);
    double norm_distance = (deviation_distance > MAX_DEVIATION) ? INFINITY : (deviation_distance / MAX_DEVIATION);
    costs += FACTOR_DISTANCE * norm_distance;

    return costs;
}

float getDistance(const Point &p1, const Point &p2) {
    return std::sqrt(CGAL::squared_distance(p1, p2));
}

float getCost(const Point &p1, const Point &p2) {
    return getDistance(p1, p2);
}

bool isGoal(const SearchState &state) {
    return state.path.size() >= DEPTH;
}

bool contains(const std::vector<Point> &path, const Point &point) {
    for (const auto &p: path) {
        if (p == point) {
            return true;
        }
    }
    return false;
}


class DelaunayPipelineNode : public rclcpp::Node {
public:
    DelaunayPipelineNode()
        : Node("delaunay_pipeline_node") {
        landmark_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "landmarks/filtered", 10,
            std::bind(&DelaunayPipelineNode::landmark_callback, this, std::placeholders::_1));
        path_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "path", 10);
        RCLCPP_DEBUG(get_logger(), "Delauney Pipeline Node gestartet");
    }

private:
    Point candidateFromNeighbor(
        const Delaunay::Face_handle &neighbor_face,
        const std::vector<Point> &nextMidpoints) {
        for (int l = 0; l < 3; l++) {
            Point candidate = CGAL::midpoint(neighbor_face->vertex(l)->point(),
                                             neighbor_face->vertex((l + 1) % 3)->point());

            for (const auto &nextMidpoint: nextMidpoints) {
                if (candidate == nextMidpoint) {
                    return candidate;
                }
            }

        }
    }

    void landmark_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        RCLCPP_DEBUG(get_logger(), "Nachricht empfangen: %d Punkte", static_cast<unsigned>(msg->polygon.points.size()));
        Delaunay dt;

        for (const auto &pt: msg->polygon.points) {
            dt.insert(Point(pt.x, pt.y));
        }
        geometry_msgs::msg::PolygonStamped output = geometry_msgs::msg::PolygonStamped();
        output.header = msg->header;

        visualization_msgs::msg::Marker marker = visualization_msgs::msg::Marker();
        marker.header = msg->header;
        marker.header.stamp = this->get_clock()->now();
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.id = 2;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.points = std::vector<geometry_msgs::msg::Point>();

        std::vector<SearchState> bestStates = std::vector<SearchState>();
        std::stack<SearchState> openList = std::stack<SearchState>();

        for (auto face = dt.finite_faces_begin(); face != dt.finite_faces_end(); ++face) {
            for (int i = 0; i < 3; i++) {
                Point seed = CGAL::midpoint(face->vertex(i)->point(),
                                            face->vertex((i + 1) % 3)->point());
                if (getDistance(seed, Point(CAR_X,CAR_Y)) <= MAX_CAR_DISTANCE) {
                    SearchState initState = SearchState();
                    initState.face = face;
                    initState.cost = 0.0;
                    initState.path.push_back(Point(0,0));
                    initState.path.push_back(seed);
                    openList.push(initState);
                    RCLCPP_DEBUG(get_logger(), "Seed: %f, %f", seed.x(), seed.y());
                }
            }
        }

        while (!openList.empty()) {
            SearchState currentState = openList.top();
            openList.pop();

            //Checks if the wanted depth is reached
            if (isGoal(currentState)) {
                bestStates.push_back(currentState);
                continue;
            }

            //Get midpoints
            Point current = currentState.path.back();
            std::vector<Point> midpoints = std::vector<Point>();
            for (int l = 0; l < 3; l++) {
                Point p = CGAL::midpoint(currentState.face->vertex(l)->point(),
                                         currentState.face->vertex((l + 1) % 3)->point());
                midpoints.push_back(p);
            }

            bool isDeadEndReached = true;
            for (int k = 0; k < 3; k++) {
                Delaunay::Face_handle neighbor_face = currentState.face->neighbor(k);
                if (dt.is_infinite(neighbor_face))
                    continue;

                Point candidate = candidateFromNeighbor(neighbor_face, midpoints);


                if (!contains(currentState.path, candidate) &&
                    getDistance(current, candidate) <= MAX_EDGE_LENGTH) {
                    SearchState newState = SearchState();
                    newState.face = neighbor_face;
                    newState.cost = currentState.cost + apply_cost_function(candidate, current, currentState.path[currentState.path.size() - 2]);
                    newState.path = currentState.path;
                    if (isinf(newState.cost)) {
                        continue;
                    }
                    newState.path.push_back(candidate);
                    openList.push(newState);
                    isDeadEndReached = false;
                }

            }
            if (isDeadEndReached) {
                bestStates.push_back(currentState);
            }

        }
        
        SearchState bestState = SearchState();
        bestState.cost = std::numeric_limits<float>::max();
        for (const SearchState &state: bestStates) {
            if (state.path.size() > bestState.path.size() || (state.path.size() == bestState.path.size() && state.cost < bestState.cost)) {
                bestState = state;
            }
            RCLCPP_DEBUG(get_logger(), "State: %zu, Costs: %f", state.path.size(), state.cost);

        }
        RCLCPP_DEBUG(get_logger(), "Best State: %zu", bestState.path.size());
        while (!bestState.path.empty()) {
            Point p = bestState.path.back();
            output.polygon.points.push_back(convertPointToROSPoint(p));
            bestState.path.pop_back();
        }

        RCLCPP_DEBUG(get_logger(), "Points; %zu", marker.points.size());
        path_pub_->publish(output);
    }

    static geometry_msgs::msg::Point32 convertPointToROSPoint(const Point p) {
        geometry_msgs::msg::Point32 ros_point;
        ros_point.x = p.x();
        ros_point.y = p.y();
        return ros_point;
    }


    // bool checkForStartPoint(const Point& p0, const Point& p1, const Point& p2) {
    //     Point car = Point(CAR_X, CAR_Y);
    //     return (getDistance(p0, car) < MAX_CAR_DISTANCE && getDistance(p1, car) < MAX_CAR_DISTANCE && getDistance(p2, car) < MAX_CAR_DISTANCE) >= 2;
    // }
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr landmark_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr path_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DelaunayPipelineNode>());
    rclcpp::shutdown();
    return 0;
}

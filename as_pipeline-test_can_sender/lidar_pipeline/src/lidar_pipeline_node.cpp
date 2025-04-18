// filter_ground.cpp

#include <memory>
#include <vector>
#include <map>
#include <cmath>
#include <string>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>

// Eigen for linear algebra
#include <Eigen/Dense>

#include "hclust.h"

#define N_ITER 6
#define N_LPR 5
#define TH_SEEDS 0.1
#define TH_DIST 0.1
#define MIN_POINTS 10

using PointT = pcl::PointXYZ;
using PointRGBT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudRGBT = pcl::PointCloud<PointRGBT>;
using PointCloudTPtr = PointCloudT::Ptr;

// // Helper: Convert a PCL PointCloud to a sensor_msgs::msg::PointCloud2 message.
// sensor_msgs::msg::PointCloud2 pointCloudToMsg(const PointCloudT::Ptr &cloud, const std::string &frame_id)
// {
//   sensor_msgs::msg::PointCloud2 output;
//   pcl::toROSMsg(*cloud, output);
//   output.header.frame_id = frame_id;
//   return output;
// }

void groundRemoval(PointCloudT *cloud_ptr, PointCloudT *output_grid, const int index) {
    // If too few points, simply return.
    if (cloud_ptr->points.size() <= MIN_POINTS) {
        return;
    }
    // Convert PCL points to Eigen vectors.
    std::vector<Eigen::Vector3f> points;
    points.reserve(cloud_ptr->points.size());
    for (const auto &pt: cloud_ptr->points) {
        points.emplace_back(pt.x, pt.y, pt.z);
    }

    double d_plane = 0.0; // Will be updated in estimatePlane.
    Eigen::Vector3f model_unit(0, 0, 0);

    // Lambda: Extract initial seeds from the points.
    auto extractInitialSeeds = [&](const std::vector<Eigen::Vector3f> &P,
                                   int n_lpr, double th_seeds) -> std::vector<Eigen::Vector3f> {
        // Copy so we can partially sort by z value.
        std::vector<Eigen::Vector3f> P_sorted = P;
        int num = std::min(n_lpr, static_cast<int>(P_sorted.size()));
        // Partially sort to find the n_lpr-th lowest z value.
        std::nth_element(P_sorted.begin(), P_sorted.begin() + num, P_sorted.end(),
                         [](const Eigen::Vector3f &a, const Eigen::Vector3f &b) {
                             return a.z() < b.z();
                         });
        double sum_z = 0.0;
        for (int i = 0; i < num; ++i)
            sum_z += P_sorted[i].z();
        double lpr = sum_z / num;

        std::vector<Eigen::Vector3f> seeds;
        seeds.reserve(P_sorted.size());
        for (const auto &p: P_sorted) {
            if (p.z() < lpr + th_seeds)
                seeds.push_back(p);
        }
        return seeds;
    };

    // Lambda: Estimate plane parameters using ground seeds.
    auto estimatePlane = [&](const std::vector<Eigen::Vector3f> &P_g) -> Eigen::Vector3f {
        Eigen::Vector3f s_mean = Eigen::Vector3f::Zero();
        for (const auto &p: P_g)
            s_mean += p;
        s_mean /= static_cast<float>(P_g.size());

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (const auto &p: P_g) {
            Eigen::Vector3f demean = p - s_mean;
            cov += demean * demean.transpose();
        }
        if (P_g.size() > 1)
            cov /= static_cast<float>(P_g.size() - 1);

        if (!cov.allFinite())
            return Eigen::Vector3f(0, 0, 0);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(cov);
        if (eigenSolver.info() != Eigen::Success)
            return Eigen::Vector3f(0, 0, 0);

        // Use the eigenvector corresponding to the smallest eigenvalue.
        Eigen::Vector3f n = eigenSolver.eigenvectors().col(0);
        model_unit = n.normalized();
        d_plane = -s_mean.dot(model_unit);
        return n;
    };

    // Lambda: Compute the distance of a point to the plane.
    auto computeDistance = [&](const Eigen::Vector3f &p) -> double {
        return std::abs(model_unit.dot(p) + d_plane);
    };

    // Initialize the set of ground seeds.
    std::vector<Eigen::Vector3f> P_g = extractInitialSeeds(points, N_LPR, TH_SEEDS);
    std::vector<Eigen::Vector3f> P_ng; // non-ground points

    // Iterative refinement.
    for (int iter = 0; iter < N_ITER + index; ++iter) {
        if (P_g.size() <= 2)
            break;
        Eigen::Vector3f model = estimatePlane(P_g);
        if (model.isZero(1e-6))
            break;

        // Reclassify points by computing distance on the fly.
        std::vector<Eigen::Vector3f> new_P_g;
        new_P_g.reserve(points.size());
        P_ng.clear();
        for (const auto &p: points) {
            double dist = computeDistance(p);
            if (dist < TH_DIST)
                new_P_g.push_back(p);
            else
                P_ng.push_back(p);
        }
        P_g.swap(new_P_g);
    }

    // Rebuild the point cloud from non-ground points.
    output_grid->points.reserve(P_ng.size());
    for (const auto &p: P_ng) {
        PointT pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        output_grid->points.push_back(pt);
    }
    output_grid->width = static_cast<uint32_t>(output_grid->points.size());
    output_grid->height = 1;
}

class FilterGround : public rclcpp::Node {
public:
    FilterGround()
        : Node("filter_ground_cpp") {
        // Subscriber: zum Beispiel "/livox/lidar/transformed"
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar/transformed", 1,
            std::bind(&FilterGround::cloudCallback, this, std::placeholders::_1));


        // Publisher: für die verarbeiteten Punktwolken
        publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/landmarks/raw", 10);
        publisher_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/landmarks", 10);
        RCLCPP_INFO(this->get_logger(), "FilterGround C++ Node gestartet");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pointcloud;

    // Aufruf bei Empfang einer neuen Punktwolke
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // log to warn current ros time
        RCLCPP_INFO(this->get_logger(), "Beginning: Current ROS time: %f", now().seconds());
        //RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message with %d points", msg->width * msg->height);
        // Konvertiere sensor_msgs::PointCloud2 in PCL PointCloud
        PointCloudT::Ptr cloud(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud);

        // Vorfiltern: Beispielweise Punkte mit z < 1.3 und x > 1.5
        PointCloudT::Ptr filtered(new PointCloudT);
        for (const auto &pt: cloud->points) {
            //if (pt.z < .5 && pt.x > 1.4 && pt.x < 40. && pt.y < 20. && pt.y > -20) // TODO keine magic numbers
            if (pt.z < 1. && pt.x > 1.4 && pt.x < 40. && pt.y < 3. && pt.y > -3.)
                filtered->points.push_back(pt);
        }
        filtered->width = filtered->points.size();
        filtered->height = 1;
        filtered->is_dense = true;

        // Gitteraufteilung (divideData2Grid): Wir teilen die Punktwolke in radiale und angulare Zellen ein.
        auto grid = divideData2Grid(filtered);
        //RCLCPP_INFO(this->get_logger(), "After Grid: Current ROS time: %f", now().seconds());
        //RCLCPP_INFO(get_logger(), "Divided data into %zu grid cells.", grid.size());

        // // --- New section: Create a colored point cloud for grid visualization ---
        // PointCloudRGBT::Ptr colored_cloud(new PointCloudRGBT);
        // // For each grid cell, assign a unique color based on its (radial, angular) indices.
        // for (const auto &cell : grid) {
        //   int radial = cell.first.first;
        //   int angular = cell.first.second;
        //   // Simple color assignment: you can change these multipliers to get different colors.
        //   uint8_t r = static_cast<uint8_t>((radial * 50) % 256);
        //   uint8_t g = static_cast<uint8_t>((angular * 60) % 256);
        //   uint8_t b = static_cast<uint8_t>(((radial + angular) * 120) % 256);
        //
        //   for (const auto &pt : cell.second->points) {
        //     PointRGBT pt_rgb;
        //     pt_rgb.x = pt.x;
        //     pt_rgb.y = pt.y;
        //     pt_rgb.z = pt.z;
        //     pt_rgb.r = r;
        //     pt_rgb.g = g;
        //     pt_rgb.b = b;
        //     colored_cloud->points.push_back(pt_rgb);
        //   }
        // }
        // colored_cloud->width = colored_cloud->points.size();
        // colored_cloud->height = 1;
        // colored_cloud->is_dense = true;
        //
        // // Convert the colored point cloud to a ROS message and publish.
        // sensor_msgs::msg::PointCloud2 colored_msg;
        // pcl::toROSMsg(*colored_cloud, colored_msg);
        // colored_msg.header.frame_id = msg->header.frame_id;
        // publisher_->publish(colored_msg);
        // RCLCPP_INFO(get_logger(), "Published colored grid point cloud with %zu points.", colored_cloud->points.size());
        PointCloudT nonGround = PointCloudT();
        //Iterate over the grid cells and remove the ground points.
        std::vector<PointCloudT> buffer(grid.size());
        uint countThread = 0;
        std::vector<std::thread> threads;

        // Launch threads.
        for (auto &[ind, snd]: grid) {
            threads.emplace_back(groundRemoval, snd.get(), &buffer[countThread++], ind.first);
        }

        countThread = 0;
        for (auto &t: threads) {
            t.join();
            nonGround += buffer[countThread++];
        }

        //RCLCPP_INFO(this->get_logger(), "Removal: Current ROS time: %f", now().seconds());
        //RCLCPP_INFO(this->get_logger(), "Removed ground, %zu points remaining.", nonGround.points.size());

        // output.data.reserve(nonGround.points.size());
        // // memcpy(&output.data[0], &nonGround.points[0], nonGround.points.size() * sizeof(PointT));
        // pcl::toROSMsg(nonGround, output);
        // output.header.frame_id = msg->header.frame_id;
        // output.header.stamp = msg->header.stamp;
        // publisher_->publish(output);

        //RCLCPP_INFO(this->get_logger(), "Removed ground, %zu points remaining.", nonGround.points.size());
        // Clustering: Verwende Euclidean Cluster Extraction als Alternative zu HDBSCAN
        //geometry_msgs::msg::Polygon centroids = clusterPoints(nonGround.makeShared());

        // Hierarchisches Clustering
        //geometry_msgs::msg::Polygon centroids = clusterPointsHierarchical(nonGround.makeShared());

        // Depth clustering
        // geometry_msgs::msg::Polygon centroids = depth_clustering(nonGround.makeShared(), msg);

        // RBNN clustering
        geometry_msgs::msg::Polygon centroids = RBNN(nonGround.makeShared());

        //RCLCPP_INFO(this->get_logger(), "Cluster: Current ROS time: %f", now().seconds());
        //RCLCPP_INFO(this->get_logger(), "Found %zu clusters.", centroids.points.size());

        // output.data.reserve(centroids->points.size());
        // //memcpy(&output.data[0], &centroids->points[0], centroids->points.size() * sizeof(PointT));
        // pcl::toROSMsg(*centroids, output);
        // output.header.frame_id = msg->header.frame_id;
        // output.header.stamp = msg->header.stamp;
        // publisher_->publish(output);


        // Assuming cloud is a PointCloudT::Ptr
        //RCLCPP_INFO(this->get_logger(), "PointCloud contains %zu points.", cloud->points.size());

        // Log the first 5 points as an example:
        // size_t numPointsToLog = std::min(cloud->points.size(), static_cast<size_t>(5));
        // for (size_t i = 0; i < numPointsToLog; ++i) {
        //     const auto &pt = cloud->points[i];
        //     RCLCPP_INFO(this->get_logger(), "Point %zu: [%.3f, %.3f, %.3f]", i, pt.x, pt.y, pt.z);
        // }
        //
        // // Erstelle die PointCloud2-Nachricht und veröffentliche
        // geometry_msgs::msg::Polygon polygon_output;
        // polygon_output.points.reserve(centroids->points.size());
        // memcpy(&polygon_output.points[0], &centroids->points[0], centroids->points.size() * sizeof(PointT));


        // PointCloudT data;
        // sensor_msgs::msg::PointCloud2 output;
        //
        // for (geometry_msgs::msg::Point32 &pt: centroids.points) {
        //     PointT p;
        //     p.x = pt.x;
        //     p.y = pt.y;
        //     p.z = pt.z;
        //     data.push_back(p);
        // }
        // pcl::toROSMsg(data, output);
        // output.header.frame_id = msg->header.frame_id;


        geometry_msgs::msg::PolygonStamped polygon_output;
        polygon_output.header.frame_id = msg->header.frame_id;
        polygon_output.header.stamp = msg->header.stamp;
        polygon_output.polygon = centroids;
        publisher_->publish(polygon_output);
        RCLCPP_INFO(this->get_logger(), "SEND: Current ROS time: %f", now().seconds());
        //RCLCPP_INFO(get_logger(), "Published Polygon message with %d points", static_cast<unsigned>(polygon_output.points.size()));
    }

    // Teilt die Punktwolke in ein adaptives Gitter (basierend auf radialen und angulare Bins)
    std::map<std::pair<int, int>, PointCloudT::Ptr> divideData2Grid(const PointCloudT::Ptr &cloud) {
        std::map<std::pair<int, int>, PointCloudT::Ptr> grid;
        // Beispielhafte Parameter – diese können an den konkreten Sensor angepasst werden.
        const int num_radial_bins = 5;
        const int num_angular_bins = 4;
        const double lidar_range = 1.0471975511965976; // ca. 60°

        // Initialisiere die Grid-Zellen
        for (int i = 0; i < num_radial_bins; ++i) {
            for (int j = 0; j < num_angular_bins; ++j) {
                grid[std::make_pair(i, j)] = std::make_shared<PointCloudT>();
            }
        }

        // Berechne für jeden Punkt den Abstand r und Winkel theta
        for (const auto &pt: cloud->points) {
            double r = std::hypot(pt.x, pt.y);
            if (r >= 40.0)
                continue; // Punkte außerhalb des Bereichs werden verworfen
            double theta = std::atan2(pt.y, pt.x);

            // Bestimme die Bin-Indizes
            // Für eine logarithmische Skalierung kann man z. B. std::log(r) verwenden.
            int radial_index = std::min(num_radial_bins - 1, static_cast<int>(num_radial_bins * (r / 40.0)));
            double ang_min = -lidar_range;
            double ang_max = lidar_range;
            int angular_index = std::min(num_angular_bins - 1,
                                         static_cast<int>(
                                             num_angular_bins * ((theta - ang_min) / (ang_max - ang_min))));
            grid[std::make_pair(radial_index, angular_index)]->points.push_back(pt);
        }
        return grid;
    }

    double static distance(const PointT &a, const PointT &b) {
        return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2);
    }

    geometry_msgs::msg::Polygon clusterPointsHierarchical(const PointCloudT::Ptr cloud) {
        const unsigned n = cloud->size();
        RCLCPP_INFO(this->get_logger(), "Start clustering %f", this->now().seconds());
        double* distmat = new double[(n*(n-1))/2];
        int k,i,j;
        for (i=k=0; i<n; i++) {
            for (j=i+1; j<n; j++) {
                // compute distance between observables i and j
                distmat[k] = distance(cloud->data()[i], cloud->data()[j]);
                k++;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Distances computed %f", this->now().seconds());

        int* merge = new int[2*(n-1)];
        double* height = new double[n-1];
        hclust_fast(n, distmat, HCLUST_METHOD_SINGLE, merge, height);
        RCLCPP_INFO(this->get_logger(), "Tree computed %f", this->now().seconds());

        int* labels = new int[n];
        // stop clustering at step with custer distance >= cdist
        constexpr double cdist = 2.25; //quadratic distance
        cutree_cdist(n, merge, height, cdist, labels);
        RCLCPP_INFO(this->get_logger(), "Labels computed %f", this->now().seconds());

        struct cluster {
            int n_points;
            PointT centroid;
        };

        std::vector<cluster> clusters;
        for (i=0; i<n; i++) {
            if (labels[i] == static_cast<int>(clusters.size())) {
                cluster c;
                c.n_points = 1;
                c.centroid = cloud->data()[i];
                clusters.push_back(c);
            }
            else if (labels[i] > static_cast<int>(clusters.size())) {
                RCLCPP_FATAL(get_logger(), "Labels does not match the number of clusters");
            }
            else {
                clusters[labels[i]].n_points++;
                clusters[labels[i]].centroid.x += cloud->data()[i].x;
                clusters[labels[i]].centroid.y += cloud->data()[i].y;
                clusters[labels[i]].centroid.z += cloud->data()[i].z;
            }
        }

        for (auto &c: clusters) {
            c.centroid.x /= c.n_points;
            c.centroid.y /= c.n_points;
            c.centroid.z /= c.n_points;
            RCLCPP_INFO(get_logger(), "NUMBER OF POINTS IN CLUSTER: %d; CENTROID DIST: %f", c.n_points, std::hypot(c.centroid.x, c.centroid.y, c.centroid.z));
        }

        RCLCPP_INFO(this->get_logger(), "Clusters computed %f", this->now().seconds());
        geometry_msgs::msg::Polygon polygon_output;
        for (auto &c: clusters) {
            geometry_msgs::msg::Point32 point;
            point.x = c.centroid.x;
            point.y = c.centroid.y;
            point.z = c.centroid.z;
            polygon_output.points.push_back(point);
        }

        return polygon_output;
    }

    geometry_msgs::msg::Polygon depth_clustering(const PointCloudT::Ptr cloud, const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        pcl::fromROSMsg(*msg, *cloud);

        int width = msg->width;
        int height = msg->height;
        RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", cloud->size());
        RCLCPP_INFO(this->get_logger(), "Width: %d, Height: %d", width, height);
        RCLCPP_INFO(this->get_logger(), "Start conversion pointcloud to 2d %f", this->now().seconds());
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
        RCLCPP_INFO(this->get_logger(), "Start depth clustering %f", this->now().seconds());
        std::vector<std::vector<int>> L(height, std::vector<int>(width, 0));
        int label = 1;
        for (int i =0;i<height;i++){
          	connected_components(label, depth_array, L, 0.1, 0.1, height, width);
            label += 1;
        }

        struct cluster {
            int n_points;
            PointT centroid;
        };

        std::vector<cluster> clusters;
        clusters.reserve(label);
        for (int i=0; i<cloud->size(); i++) {
            int u = i % width;  // Column index
            int v = i / width;  // Row index
            clusters[L[v][u]].n_points++;
            clusters[L[v][u]].centroid.x += cloud->data()[i].x;
            clusters[L[v][u]].centroid.y += cloud->data()[i].y;
            clusters[L[v][u]].centroid.z += cloud->data()[i].z;
        }

        for (auto &c: clusters) {
            c.centroid.x /= c.n_points;
            c.centroid.y /= c.n_points;
            c.centroid.z /= c.n_points;
            RCLCPP_INFO(get_logger(), "NUMBER OF POINTS IN CLUSTER: %d; CENTROID DIST: %f", c.n_points, std::hypot(c.centroid.x, c.centroid.y, c.centroid.z));
        }

        geometry_msgs::msg::Polygon polygon_output;
        for (auto &c: clusters) {
            geometry_msgs::msg::Point32 point;
            point.x = c.centroid.x;
            point.y = c.centroid.y;
            point.z = c.centroid.z;
            polygon_output.points.push_back(point);
        }

        RCLCPP_INFO(this->get_logger(), "End conversion pointcloud to 2d %f", this->now().seconds());

        return polygon_output;
    }

    void static connected_components(int& label, const std::vector<std::vector<double>>& depth_array, std::vector<std::vector<int>>& L, double alpha, double threshold, int height, int width){
        for (int i =0;i<height;i++){
          	for (int j=0;j<width;j++){
            	if (L[i][j] == 0 && depth_array[i][j] > 0.0){
              		std::vector<std::pair<int,int>> Q;
              		Q.emplace_back(i,j);
              		while (!Q.empty()) {
                		std::pair<int,int> p = Q.front();
                        L[p.first][p.second] = label;
                        std::vector<std::pair<int,int>> neighbours = get_neighbours(p.first,p.second, width, height);
                        for (auto n: neighbours){
                            double d1 = std::max(depth_array[p.first][p.second],depth_array[n.first][n.second]);
                            double d2 = std::min(depth_array[p.first][p.second],depth_array[n.first][n.second]);
                            if (std::atan(std::sin(alpha)*d2/(d1-d2*std::cos(alpha)) > threshold)){
                                Q.push_back(n);
                            }
                        }
                        Q.erase(Q.begin());
                	}
              	}
            }
        }
    }

    std::vector<std::pair<int,int>> static inline get_neighbours(int i, int j, int width, int height){
        std::vector<std::pair<int,int>> neighbours;
        if (i-1 >= 0){
            neighbours.emplace_back(i-1,j);
        }
        if (i+1 < height){
            neighbours.emplace_back(i+1,j);
        }
        if (j-1 >= 0){
            neighbours.emplace_back(i,j-1);
        }
        if (j+1 < width){
            neighbours.emplace_back(i,j+1);
        }
        return neighbours;
    }

//     @inproceedings{inproceedings,
// author = {Klasing, Klaas and Wollherr, Dirk and Buss, Martin},
// year = {2008},
// month = {05},
// pages = {4043-4048},
// title = {A clustering method for efficient segmentation of 3D laser data},
// journal = {Proceedings - IEEE International Conference on Robotics and Automation},
// doi = {10.1109/ROBOT.2008.4543832}
//     }
    geometry_msgs::msg::Polygon RBNN(const PointCloudT::Ptr cloud) {
        constexpr double radius = 1.;
        constexpr unsigned min_points = 3;
        constexpr unsigned max_points = 200;

        RCLCPP_INFO(this->get_logger(), "Start RBNN %f", this->now().seconds());

        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(cloud);

        std::vector<unsigned int> labels;
        labels.assign(cloud->points.size(), 0);
        unsigned label_counter = 1;

        RCLCPP_INFO(this->get_logger(), "Before loop %f", this->now().seconds());

        for (int i = 0; i < cloud->size(); i++) {
            if (labels[i] != 0) continue;

            // Perform radius search
            std::vector<int> pointIndices;
            std::vector<float> pointDistances;
            kdtree.radiusSearch(cloud->points[i], radius, pointIndices, pointDistances);

            for (const auto& pointIndex : pointIndices) {
                if (labels[i] != 0 && labels[pointIndex] != 0) {
                    if (labels[i] != labels[pointIndex]) {
                        for (int j = 0; j < cloud->size(); j++) if (labels[j] == labels[pointIndex]) labels[j] = labels[i];
                    }
                }
                else if (labels[i] != 0) {
                    labels[pointIndex] = labels[i];
                }
                else if (labels[pointIndex] != 0) {
                    labels[i] = labels[pointIndex];
                }
            }

            if (labels[i] == 0) {
                labels[i] = ++label_counter;
                //RCLCPP_INFO(this->get_logger(), "Found label %d", label_counter);
            }

            for (const auto& pointIndex : pointIndices) {
                labels[pointIndex] = labels[i];
            }

        }

        RCLCPP_INFO(this->get_logger(), "After loop %f", this->now().seconds());

        struct cluster {
            int n_points{};
            PointT centroid;
        };
        std::unordered_map<unsigned, cluster> cluster_map;

        sensor_msgs::msg::PointCloud2 output;
        pcl::PointCloud<pcl::PointXYZI> cloud_alpha;
        //   pcl::toROSMsg(*cloud, output);
        //   output.header.frame_id = frame_id;
        for (unsigned i = 0; i < labels.size(); i++) {
            cluster_map[labels[i]].n_points++;
            cluster_map[labels[i]].centroid.x += cloud->points[i].x;
            cluster_map[labels[i]].centroid.y += cloud->points[i].y;
            cluster_map[labels[i]].centroid.z += cloud->points[i].z;
            // RCLCPP_INFO(get_logger(), "Update label %d; num: %d", i, cluster_map[i].n_points);

            pcl::PointXYZI asdf;
            asdf.x = cloud->points[i].x;
            asdf.y = cloud->points[i].y;
            asdf.z = cloud->points[i].z;
            asdf.intensity = labels[i];
            cloud_alpha.points.push_back(asdf);
        }
        pcl::toROSMsg<pcl::PointXYZI>(cloud_alpha, output);
        // pcl::toROSMsg<pcl::PointXYZ>(*cloud, output);
        output.header.frame_id = "base_link";
        output.header.stamp = this->now();
        publisher_pointcloud->publish(output);

        geometry_msgs::msg::Polygon polygon_output;
        for (auto&[fst, snd]: cluster_map) {
            // RCLCPP_INFO(get_logger(), "In start loop");
            if (snd.n_points < min_points || snd.n_points > max_points) {
                //cluster_map.erase(fst);
                continue;
            }
            snd.centroid.x /= snd.n_points;
            snd.centroid.y /= snd.n_points;
            snd.centroid.z /= snd.n_points;

            const double centroid_dist = std::hypot(snd.centroid.x, snd.centroid.y, snd.centroid.z);

            // constexpr double m = 1.25;
            // constexpr double t = 60;

            constexpr double m_low = - 7. / 30;
            constexpr double m_high = -30. / 5;
            constexpr double t_low = 10 + 7. / 3;
            constexpr double t_high = 70.;

            constexpr double margin = 4;

            if (centroid_dist > 5. &&
                    (snd.n_points > std::max(m_low * centroid_dist + t_low + margin,
                        m_high * centroid_dist + t_high + margin) ||
                        snd.n_points < std::min(m_low * centroid_dist + t_low - margin,
                            m_high * centroid_dist + t_high - margin))) {
                RCLCPP_INFO(get_logger(), "Above line");
                continue;
            }

            geometry_msgs::msg::Point32 point;
            point.x = snd.centroid.x;
            point.y = snd.centroid.y;
            point.z = snd.centroid.z;
            polygon_output.points.push_back(point);
            RCLCPP_INFO(get_logger(), "NUMBER OF POINTS IN CLUSTER: %d; CENTROID DIST: %f",
                snd.n_points,
                centroid_dist);
            // RCLCPP_INFO(get_logger(), "In loop");
        }
        RCLCPP_INFO(this->get_logger(), "End RBNN %f", this->now().seconds());
        return polygon_output;
    }


    // Clusterung der nicht zum Boden gehörenden Punkte mittels Euclidean Cluster Extraction.
    geometry_msgs::msg::Polygon clusterPoints(const PointCloudT::Ptr cloud) {
        // KdTree für die Cluster-Extraktion
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.275); //  TODO: Parameter fitten default = 0.05
        ec.setMinClusterSize(3);
        ec.setMaxClusterSize(1000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // Für jedes Cluster wird der Schwerpunkt berechnet.
        PointCloudT::Ptr centroids(new PointCloudT);
        geometry_msgs::msg::Polygon polygon_output;
        for (const auto &indices: cluster_indices) {
            Eigen::Vector3f centroid(0, 0, 0);
            for (int idx: indices.indices) {
                const auto &pt = cloud->points[idx];
                centroid[0] += pt.x;
                centroid[1] += pt.y;
                centroid[2] += pt.z;
            }
            centroid /= static_cast<float>(indices.indices.size());
            RCLCPP_INFO(get_logger(), "NUMBER OF POINTS IN CLUSTER: %zu; CENTROID DIST: %f", indices.indices.size(), centroid.norm());

            // filter landmarks -> if z is too high -> no cone
            if (centroid[2] < 0.4) {
                // filter landmarks -> size of cluster
                const float dist_to_zero = centroid.norm();
                constexpr float v_res = 0.23;
                constexpr float h_res = 0.18;
                constexpr float height_small = 0.325 * 0.9; //height of the cone
                constexpr float width_small = 0.228; //width of the cone
                const float expected_small = ((2.f * ((std::tan((width_small / 2.f) / dist_to_zero) / M_PI) * 180.)) / h_res) *
                                             ((2. * ((std::tan((height_small / 2.f) / dist_to_zero) / M_PI) * 180.)) / v_res) * 0.5;
                const float expected_small_old = 0.5 * (height_small / (2 * dist_to_zero * std::tan(v_res / 2))) *
                                                 (width_small / (2 * dist_to_zero * std::tan(h_res / 2)));
                constexpr float height_big = 0.505 * 0.9;
                constexpr float width_big = 0.285;
                const float expected_big = ((2.f * ((std::tan((width_big / 2.f) / dist_to_zero) / M_PI) * 180.)) / h_res) *
                                           ((2. * ((std::tan((height_big / 2.) / dist_to_zero) / M_PI) * 180.)) / v_res) * 0.5;

                constexpr float threshold_percent = 0.3;

                RCLCPP_INFO(get_logger(), "Cluster size: %zu; expected small: %f; expected big: %f; expected small old: %f; dist: %f",
                    indices.indices.size(), expected_small, expected_big, expected_small_old, dist_to_zero);

                // if (dist_to_zero < 10. || (indices.indices.size() < expected_small * (1 + threshold_percent) &&
                //         indices.indices.size() > expected_small * (1 - threshold_percent))) {
                //     geometry_msgs::msg::Point32 point;
                //     point.x = centroid[0];
                //     point.y = centroid[1];
                //     point.z = centroid[2];
                //     polygon_output.points.push_back(point);
                // }

                geometry_msgs::msg::Point32 point;
                point.x = centroid[0];
                point.y = centroid[1];
                point.z = centroid[2];
                polygon_output.points.push_back(point);
            }
        }
        return polygon_output;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FilterGround>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

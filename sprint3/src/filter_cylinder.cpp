#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <opencv2/opencv.hpp> // For OpenCV
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <string>

class FilterCylinderNode : public rclcpp::Node
{
public:
    FilterCylinderNode() : Node("filter_cylinder")
    {
        // Declare the parameter for placing markers
        this->declare_parameter<bool>("place_markers", false);
        this->get_parameter("place_markers", place_markers_);

        // Subscribe to the /scan and /odom topics
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&FilterCylinderNode::laserScanCallback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&FilterCylinderNode::odomCallback, this, std::placeholders::_1));

        // Publisher for markers in Gazebo
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

        // Define a threshold for clustering points (in meters)
        clustering_threshold_ = 0.1; // 10 cm threshold for clustering

        // Cylinder diameter to detect (30 cm)
        cylinder_diameter_ = 0.30;

        // Load the map image
        map_image_ = cv::imread("/home/luis/ros2_ws/src/robotics_1_smart_factory/map/nav2/my_map.pgm", cv::IMREAD_GRAYSCALE);

        if (map_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load the map image. Check if the file exists and is accessible.");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Map image loaded successfully. Image size: %dx%d", map_image_.cols, map_image_.rows);
        }

        // Assume these map parameters (Adjust accordingly based on your map setup)
        map_resolution_ = 0.05;  // Map resolution in meters/pixel (adjust as necessary)
        map_origin_x_ = 5.5;   // X origin of the map in meters (adjust as necessary)
        map_origin_y_ = 6;   // Y origin of the map in meters (adjust as necessary)
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    double clustering_threshold_;
    double cylinder_diameter_;
    bool place_markers_;

    cv::Mat map_image_;
    cv::Mat edited_map_image_;
    double map_resolution_;    // Map resolution in meters per pixel
    double map_origin_x_;      // X origin of the map in meters
    double map_origin_y_;      // Y origin of the map in meters

    struct Point2D
    {
        double x;
        double y;
        int index; // To keep track of the original range index
    };

    struct Pose2D
    {
        double x;
        double y;
        double theta;
    } robot_pose_;

    // Callback for odometry to get the robot's pose
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_pose_.x = msg->pose.pose.position.x;
        robot_pose_.y = msg->pose.pose.position.y;

        // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);

        // Extract orientation as yaw angle
        tf2::Matrix3x3 mat(quat);
        double roll, pitch;
        mat.getRPY(roll, pitch, robot_pose_.theta); // Extract yaw (theta)
    }

    // Function to convert a range into a 2D point in the robot's coordinate frame
    Point2D rangeToPoint(double range, double angle, int index)
    {
        Point2D point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);
        point.index = index;
        return point;
    }

    // Callback for processing the laser scan
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // Add error handling to check scan message
        if (!scan_msg || scan_msg->ranges.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid or empty laser scan message.");
            return;
        }

        std::vector<Point2D> points;
        std::vector<int> cylinder_indices;

        // RCLCPP_INFO(this->get_logger(), "Processing scan data...");

        // Convert ranges to 2D points in the robot's coordinate frame
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            double range = scan_msg->ranges[i];
            // Skip invalid or infinite ranges
            if (range < scan_msg->range_min || range > scan_msg->range_max)
                continue;

            // Calculate the angle corresponding to the current range
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;

            // Convert to 2D point and store
            points.push_back(rangeToPoint(range, angle, i));
        }

        // RCLCPP_INFO(this->get_logger(), "Converted %lu points.", points.size());

        // Cluster points based on proximity
        std::vector<std::vector<Point2D>> clusters = clusterPoints(points);

        // Identify the clusters that match the cylinder's width
        for (const auto &cluster : clusters)
        {
            if (cluster.size() >= 2)
            {
                double width = calculateClusterWidth(cluster);
                if (std::abs(width - cylinder_diameter_) < 0.05) // Allow 5 cm tolerance
                {
                    Point2D cylinder_center = calculateClusterCenter(cluster);
                    RCLCPP_INFO(this->get_logger(), "Cylinder center at (%.2f, %.2f)", cylinder_center.x, cylinder_center.y);

                    // Place markers if required
                    if (place_markers_)
                    {
                        publishMarker(cylinder_center);
                    }

                    // Draw cylinder on the map
                    drawCylinderOnMap(cylinder_center);
                }
            }
        }

        // RCLCPP_INFO(this->get_logger(), "Processing completed.");
    }

    // Function to calculate the center of a cluster in the world frame
    Point2D calculateClusterCenter(const std::vector<Point2D> &cluster)
    {
        Point2D center = {0.0, 0.0, 0};

        // Transform each point to the global frame using the robot's pose
        for (const auto &point : cluster)
        {
            // Convert from the robot's local frame to the global frame
            double global_x = robot_pose_.x + std::cos(robot_pose_.theta) * point.x - std::sin(robot_pose_.theta) * point.y;
            double global_y = robot_pose_.y + std::sin(robot_pose_.theta) * point.x + std::cos(robot_pose_.theta) * point.y;

            center.x += global_x;
            center.y += global_y;
        }

        // Compute the average (center) of the transformed points
        center.x /= cluster.size();
        center.y /= cluster.size();

        return center;
    }

    // Function to draw a cylinder on the map
    void drawCylinderOnMap(const Point2D &cylinder_center)
    {
        // Reload the map from the original image to reset any previous drawings
        edited_map_image_ = map_image_.clone();  // Clone the original map image

        // Debugging: Print map parameters and cylinder center
        // RCLCPP_INFO(this->get_logger(), "Map origin: (%.2f, %.2f)", map_origin_x_, map_origin_y_);
        // RCLCPP_INFO(this->get_logger(), "Map resolution: %.2f meters/pixel", map_resolution_);
        // RCLCPP_INFO(this->get_logger(), "Cylinder center: (%.2f, %.2f)", cylinder_center.x, cylinder_center.y);

        // Convert global coordinates to map pixel coordinates
        int map_x = static_cast<int>((map_origin_x_ + cylinder_center.x) / map_resolution_);

        // Adjust the map_y calculation by taking the difference relative to the origin
        int map_y = static_cast<int>((map_origin_y_ + cylinder_center.y) / map_resolution_);

        // Validate map coordinates
        if (map_x < 0 || map_x >= map_image_.cols || map_y < 0 || map_y >= map_image_.rows) {
            RCLCPP_ERROR(this->get_logger(), "Invalid map coordinates: (map_x: %d, map_y: %d)", map_x, map_y);
            return;
        }

        // Debugging: Print the calculated map coordinates
        // RCLCPP_INFO(this->get_logger(), "Map coordinates: (map_x: %d, map_y: %d)", map_x, map_y);

        // Draw a circle representing the cylinder (15 cm radius, 30 cm diameter)
        int radius = static_cast<int>((cylinder_diameter_ / 2) / map_resolution_);
        cv::circle(edited_map_image_, cv::Point(map_x, map_y), radius, cv::Scalar(0), -1); // Black circle

        // Show the map with the drawn cylinder
        cv::imshow("Map with Cylinder", edited_map_image_);
        cv::waitKey(1); // Update display
    }

    // Function to cluster points based on their proximity
    std::vector<std::vector<Point2D>> clusterPoints(const std::vector<Point2D> &points)
    {
        std::vector<std::vector<Point2D>> clusters;
        std::vector<Point2D> current_cluster;

        for (size_t i = 0; i < points.size(); ++i)
        {
            if (current_cluster.empty())
            {
                current_cluster.push_back(points[i]);
            }
            else
            {
                double distance = calculateDistance(current_cluster.back(), points[i]);
                if (distance < clustering_threshold_)
                {
                    current_cluster.push_back(points[i]);
                }
                else
                {
                    clusters.push_back(current_cluster);
                    current_cluster.clear();
                    current_cluster.push_back(points[i]);
                }
            }
        }

        if (!current_cluster.empty())
        {
            clusters.push_back(current_cluster);
        }

        return clusters;
    }

    // Function to calculate the distance between two 2D points
    double calculateDistance(const Point2D &p1, const Point2D &p2)
    {
        return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
    }

    // Function to calculate the width of a cluster (distance between first and last point)
    double calculateClusterWidth(const std::vector<Point2D> &cluster)
    {
        if (cluster.size() < 2)
            return 0.0;

        const Point2D &first_point = cluster.front();
        const Point2D &last_point = cluster.back();
        return calculateDistance(first_point, last_point);
    }

    // Function to publish a marker for visualization in Gazebo
    void publishMarker(const Point2D &point)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cylinder_marker";
        marker.id = point.index;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Convert point to global frame using robot pose
        double global_x = robot_pose_.x + std::cos(robot_pose_.theta) * point.x - std::sin(robot_pose_.theta) * point.y;
        double global_y = robot_pose_.y + std::sin(robot_pose_.theta) * point.x + std::cos(robot_pose_.theta) * point.y;

        marker.pose.position.x = global_x;
        marker.pose.position.y = global_y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_publisher_->publish(marker);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FilterCylinderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

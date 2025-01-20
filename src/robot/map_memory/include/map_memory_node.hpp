#ifndef MAP_MEMORY_NODE_HPP
#define MAP_MEMORY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>

class MapMemoryNode : public rclcpp::Node {
public:
    MapMemoryNode();

private:
    double resolution_;
    double update_threshold_;
    double map_width_;
    double map_height_;
    std::string global_frame_;
    std::string local_costmap_topic_;
    std::string odom_topic_;
    std::string global_map_topic_;

    nav_msgs::msg::OccupancyGrid global_map_;

    geometry_msgs::msg::Pose last_robot_pose_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;

    rclcpp::TimerBase::SharedPtr map_publish_timer_;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void initializeGlobalMap();
    void integrateLocalCostmap(const nav_msgs::msg::OccupancyGrid &local_costmap, const geometry_msgs::msg::Pose &robot_pose);
    bool shouldUpdate(const geometry_msgs::msg::Pose &current_pose);
    void publishGlobalMap();
};

#endif // MAP_MEMORY_NODE_HPP
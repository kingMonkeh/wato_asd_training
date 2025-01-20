#include "map_memory_node.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

MapMemoryNode::MapMemoryNode() : Node("map_memory_node") {
    resolution_ = this->declare_parameter("resolution", 0.05);
    update_threshold_ = this->declare_parameter("update_threshold", 1.0);
    map_width_ = this->declare_parameter("map_width", 100.0);
    map_height_ = this->declare_parameter("map_height", 100.0);
    global_frame_ = this->declare_parameter("global_frame", "map");
    local_costmap_topic_ = this->declare_parameter("local_costmap_topic", "local_costmap");
    odom_topic_ = this->declare_parameter("odom_topic", "odom");
    global_map_topic_ = this->declare_parameter("global_map_topic", "global_map");

    initializeGlobalMap();

    local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        local_costmap_topic_, 10,
        std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&MapMemoryNode::odometryCallback, this, std::placeholders::_1));

    global_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(global_map_topic_, 10);

    map_publish_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MapMemoryNode::publishGlobalMap, this));
}

void MapMemoryNode::initializeGlobalMap() {
    global_map_.info.resolution = resolution_;
    global_map_.info.width = static_cast<uint32_t>(map_width_ / resolution_);
    global_map_.info.height = static_cast<uint32_t>(map_height_ / resolution_);
    global_map_.info.origin.position.x = -map_width_ / 2.0;
    global_map_.info.origin.position.y = -map_height_ / 2.0;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.data.assign(global_map_.info.width * global_map_.info.height, -1);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!last_robot_pose_.orientation.w) {
        RCLCPP_WARN(this->get_logger(), "Robot has not moved!");
        return;
    }
    integrateLocalCostmap(*msg, last_robot_pose_);
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (shouldUpdate(msg->pose.pose)) {
        last_robot_pose_ = msg->pose.pose;
    }
}

bool MapMemoryNode::shouldUpdate(const geometry_msgs::msg::Pose &current_pose) {
    double dx = current_pose.position.x - last_robot_pose_.position.x;
    double dy = current_pose.position.y - last_robot_pose_.position.y;
    return (dx * dx + dy * dy) >= (update_threshold_ * update_threshold_);
}

void MapMemoryNode::integrateLocalCostmap(const nav_msgs::msg::OccupancyGrid &local_costmap, const geometry_msgs::msg::Pose &robot_pose) {
    tf2::Transform global_to_robot;
    tf2::fromMsg(robot_pose, global_to_robot);

    for (uint32_t y = 0; y < local_costmap.info.height; ++y) {
        for (uint32_t x = 0; x < local_costmap.info.width; ++x) {
            int8_t cost = local_costmap.data[y * local_costmap.info.width + x];
            if (cost < 0) continue; // Skip unknown cells

            double local_x = x * local_costmap.info.resolution + local_costmap.info.origin.position.x;
            double local_y = y * local_costmap.info.resolution + local_costmap.info.origin.position.y;

            tf2::Vector3 local_point(local_x, local_y, 0);
            tf2::Vector3 global_point = global_to_robot * local_point;

            int gx = static_cast<int>((global_point.x() - global_map_.info.origin.position.x) / resolution_);
            int gy = static_cast<int>((global_point.y() - global_map_.info.origin.position.y) / resolution_);

            if (gx >= 0 && gx < static_cast<int>(global_map_.info.width) &&
                gy >= 0 && gy < static_cast<int>(global_map_.info.height)) {
                int global_idx = gy * global_map_.info.width + gx;
                global_map_.data[global_idx] = cost;
            }
        }
    }
}

void MapMemoryNode::publishGlobalMap() {
    global_map_.header.stamp = this->now();
    global_map_.header.frame_id = global_frame_;
    global_map_pub_->publish(global_map_);
}
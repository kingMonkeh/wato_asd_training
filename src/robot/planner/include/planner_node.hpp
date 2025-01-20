#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr goal);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void timer_callback();
  void planPath();
  bool reachGoal();
  

private:
  enum class State
  {
    WAITING_FOR_GOAL,
    WAITING_FOR_ROBOT_TO_REACH_GOAL
  };
  State state_;

  robot::PlannerCore planner_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscribe_map_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscribe_goal_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscribe_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publish_path_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  geometry_msgs::msg::Pose robot_pose_;
  nav_msgs::msg::Path path;
  
  bool goal_received_ = false;

  int width;
  int height;

  double odom_x_;
  double odom_y_;
  double goal_x_;
  double goal_y_;
  bool have_goal_;
  bool have_odom_;

  // ------------------- Supporting Structures -------------------

  // 2D grid index
  struct CellIndex
  {
    int x;
    int y;

    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex &other) const
    {
      return (x == other.x && y == other.y);
    }

    bool operator!=(const CellIndex &other) const
    {
      return (x != other.x || y != other.y);
    }
  };


  // Hash function for CellIndex so it can be used in std::unordered_map
  struct CellIndexHash
  {
    std::size_t operator()(const CellIndex &idx) const
    {
      // A simple hash combining x and y
      return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
  };

  // Structure representing a node in the A* open set
  struct AStarNode
  {
    CellIndex index;
    double f_score; // f = g + h

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
  };

  // Comparator for the priority queue (min-heap by f_score)
  struct CompareF
  {
    bool operator()(const AStarNode &a, const AStarNode &b)
    {
      // We want the node with the smallest f_score on top
      return a.f_score > b.f_score;
    }
  };
};

#endif
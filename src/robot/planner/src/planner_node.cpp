#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{

  /*-------------- SUBSCRIBERS, PUBLISHER, TIMER ----------------*/

  // Subscribe -> /map
  subscribe_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10,
    std::bind(&PlannerNode::map_callback, this, std::placeholders::_1));

  // Subscribe -> /goal_point
  subscribe_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10,
    std::bind(&PlannerNode::goal_callback, this, std::placeholders::_1));

  // Subscribe -> /odom/filtered
  subscribe_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10,
    std::bind(&PlannerNode::odom_callback, this, std::placeholders::_1));

  // Publisher -> /path
  publish_path_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timer_callback, this));
}

void PlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  // Uses as the grid for A* pathfinding
  current_map_ = *map;

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    planPath();
  }
}

void PlannerNode::goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr goal) // DONE -------------------------------------------
{

  if (have_goal_)
  {
    RCLCPP_INFO(this->get_logger(), "Already have an active goal");
    return;
  }

  // Specifies the goal point
  goal_x_ = goal->point.x;
  goal_y_ = goal->point.y;
  have_goal_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) // DONE --------------------------------------------------
{
  odom_x_ = odom->pose.pose.position.x;
  odom_y_ = odom->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timer_callback()
{

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    if (reachGoal())
    {
      RCLCPP_INFO(this->get_logger(), "Reached goal succesfully.");
      state_ = State::WAITING_FOR_GOAL;

    }
    else
    {
      planPath();
    }
  }
}

bool PlannerNode::reachGoal()
{
  float goal_threshhold = 0.5;
  float diff_x = goal_x_ - odom_x_;
  float diff_y = goal_y_ - odom_y_;

  return std::sqrt(pow(diff_x, 2) + pow(diff_y, 2)) <= goal_threshhold; // set 0.5 m as the goal threshold
}



void PlannerNode::planPath() {

  if (!have_goal_ || !have_odom_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot Reach Map.");
    return; 
  }

  path = planner_.getPath();
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  publish_path_->publish(path);
}


// DIDNT DEFINE GET PATH


/*-------------- MAIN ----------------*/

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

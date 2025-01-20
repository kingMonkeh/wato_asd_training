#include <chrono>
#include <memory>
#include <sstream>
 
#include "costmap_node.hpp"
 
 
CostmapNode::CostmapNode() : Node("costmap"), costmapCore(robot::CostmapCore(this->get_logger())) {

  //Have to put this here, cant put it in header since the pose class has not yet been initialized
  //compiler will tell you origin has no type, since it has not read the definition of pose
  origin.position.x = -5.0;
  origin.position.y = -5.0;
  origin.orientation.w = 1.0;

  // Subscribe to Lidar
  lidarSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "\lidar", 10, 
    std::bind(
      &CostmapNode::lidarSubscriberCallback, this, 
      std::placeholders::_1));
//note to hy lac: important that the callback function specified matchesthe function i make
    
    //publisher for data
  costmapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("\costmap", 10);
  //timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  //what does this do? no clue, do i need it? probably not, just seems to let us know its working
  RCLCPP_INFO(this->get_logger(), "Initialized ROS Constructs");
  costmapCore.initCostmap(resolution, width, height, origin, inflationRadius);
  RCLCPP_INFO(this->get_logger(), "Initialized Costmap Core"); //tell them its working
}
 
 /*
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}
*/

void CostmapNode::lidarSubscriberCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
  // Update the costmap according to the laser scan
  costmapCore.updateCostmap(msg);
  // publish the costmap
  nav_msgs::msg::OccupancyGrid costmap_msg = *costmapCore.getCostmapData();
  costmap_msg.header = msg->header;
  costmapPublisher->publish(costmap_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
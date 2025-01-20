#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

//note to hy lac:
/*
Although I am essentially just following the answwer key, I am rewriting parts that do not make sense to me
I am also noting any important syntax as well as trying to explain why each step is needed
*/

#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode(); //constructor
    
    // Place callback function here
    void publishMessage();

    //subscriber callback function, hehehehe
    void lidarSubscriberCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

 
  private:
    robot::CostmapCore costmapCore;
    // Place these constructs here
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidarSubscriber;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmapPublisher;
    rclcpp::TimerBase::SharedPtr timer_; //idk what this is for, i dont think i need it

    //I do not understand much,
    //but i am essentially just following the answer key and rewriting it in a way that makes sense to me
    //no clue why we need processParameters() function, seems useless :/
    double resolution = 0.1;
    int width = 100;
    int height = 100;
    geometry_msgs::msg::Pose origin;
    double inflationRadius = 1.0;
};
 
#endif 
#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"



namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    //still dont understand origin lmao
    void initCostmap(
        double resolution, 
        int width, int height, 
        geometry_msgs::msg::Pose origin, 
        double inflation_radius
        );

    //update the costmap, using the info from lidar
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const;

    //no clue what this function does, but it is important
    void inflateObstacle(int origin_x, int origin_y) const;

    //we need this cus the costmap is for some reason private, imo make it public :)
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;




  private:
    nav_msgs::msg::OccupancyGrid::SharedPtr costmapData;
    rclcpp::Logger logger_;

    double inflation_radius_;
    int inflation_cells_;


};

}  

#endif  
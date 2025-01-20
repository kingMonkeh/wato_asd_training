#include <algorithm>
#include <queue>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : costmapData(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void CostmapCore::initCostmap(double resolution, int width, int height, geometry_msgs::msg::Pose origin, double inflation_radius) {
  //note to hy lac: info is a parameter of the CostmapCore class.
  /*
  Copied from the documentation:
  # This represents a 2-D grid map, in which each cell represents the probability of
  # occupancy.

  Header header 

  #MetaData for the map
  MapMetaData info

  # The map data, in row-major order, starting with (0,0).  Occupancy
  # probabilities are in the range [0,100].  Unknown is -1.
  int8[] data

  */
  costmapData->info.resolution = resolution;
  costmapData->info.width = width;
  costmapData->info.height = height;
  costmapData->info.origin = origin;
  //We fill the data array with -1, since documentation says to put -1 if unknown
  //Since we are initializing, we havve to set it to -1
  costmapData->data.assign(width * height, -1);

  inflation_radius_ = inflation_radius;
  inflation_cells_ = static_cast<int>(inflation_radius / resolution);

  RCLCPP_INFO(logger_, "Costmap initialized with resolution: %.2f, width: %d, height: %d", resolution, width, height);
}

//Where the main bulk of work is done
void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const {
  // Reset the costmap to free space
  std::fill(costmapData->data.begin(), costmapData->data.end(), 0);

  for (size_t i = 0; i < laserscan->ranges.size(); i++) {
    //if you use ++i, i think you might be schizo
    double angle = laserscan->angle_min + i * laserscan->angle_increment; //get angle
    double range = laserscan->ranges[i];

    // skip if its out of range, i dont like the either way of logic
    if (range < laserscan->range_min || range > laserscan->range_max){
        continue;
    }

    //Step 3 of costmap, i cant really edit this in anyway, everyone will have this
    // Calculate obstacle position in the map frame
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    // Convert to grid coordinates
    //wtf is this static_cast stuff, probably dont need it so im gonna comment out and just write it my way
    //int grid_x = static_cast<int>((x - costmapData->info.origin.position.x) / costmapData->info.resolution);
    //int grid_y = static_cast<int>((y - costmapData->info.origin.position.y) / costmapData->info.resolution);
    //doesnt it autocast to int anyway? since grid_x is an int

    //divide by resolution to scale it to our grid
    int grid_x = (x - costmapData->info.origin.position.x) / costmapData->info.resolution;
    int grid_y = (y - costmapData->info.origin.position.y) / costmapData->info.resolution;

    if (grid_x >= 0 && grid_x < static_cast<int>(costmapData->info.width) &&
    grid_y >= 0 && grid_y < static_cast<int>(costmapData->info.height)) {
    // Mark the cell as occupied
    //we do this becuase it is a 1-D array now? I assume
    int index = grid_y * costmapData->info.width + grid_x;
    costmapData->data[index] = 100;  // 100 indicates an occupied cell
    //We set these as occupied becuase our scanner is literally telling us there is an obstacle at [index]

    // Inflate around the obstacle
    inflateObstacle(grid_x, grid_y);
    }

  }
}

//obviously the scanner isnt perfect, so we assign slightly lower occupancy values to cells surrounding guranteed obstacles
void CostmapCore::inflateObstacle(int origin_x, int origin_y) const {
  // Use a simple breadth-first search (BFS) to mark cells within the inflation radius
  std::queue<std::pair<int, int>> queue;
  queue.emplace(origin_x, origin_y);

  //initialize 2-D visited array of size [width][height] with false
  std::vector<std::vector<bool>> visited(costmapData->info.width, std::vector<bool>(costmapData->info.height, false));
  visited[origin_x][origin_y] = true; //Set origin as visited

  //Push out of the origin using BFS, queue implementation first in first out
  while (!queue.empty()) {
    auto [x, y] = queue.front();
    queue.pop();

    // Iterate over neighboring cells
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;  // Skip the center cell

        int nx = x + dx;
        int ny = y + dy;

        // Skip if out of bounds
        if (nx < 0 || nx >= costmapData->info.width || ny < 0 || ny >= costmapData->info.height || visited[nx][ny]) {
          continue;
        }

        // Calculate the distance to the original obstacle cell
        //take x and y distances and take hypot for distance, multiply by resolution to rescale to actual length
        double distance = std::hypot(nx - origin_x, ny - origin_y) * costmapData->info.resolution;

        // If within inflation radius, mark as inflated and add to queue
        if (distance <= inflation_radius_) {
            int index = ny * costmapData->info.width + nx;
            if (costmapData->data[index] < (1 - (distance / inflation_radius_)) * 100) {
              costmapData->data[index] = (1 - (distance / inflation_radius_)) * 100;
            }
            queue.emplace(nx, ny);
        }

        visited[nx][ny] = true;

      }
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const {
  return costmapData;
}

}
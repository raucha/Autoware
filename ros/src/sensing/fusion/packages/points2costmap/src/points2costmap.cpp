/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>

static ros::Publisher g_costmap_pub;
static double g_resolution;
static int g_cell_width;
static int g_cell_height;

static int calculateMedian(std::vector<int> cost_vector)
{
  std::sort(cost_vector.begin(),cost_vector.end());
 // for(auto el : cost_vector)
 //   ROS_INFO("cost value : %d",el);

  int cost = 0;
  if (cost_vector.size() % 2 == 0 && cost_vector.size() != 0)
  {
    cost = (cost_vector[cost_vector.size() / 2] + cost_vector[cost_vector.size() / 2 - 1]) / 2;
  }
  else if (cost_vector.size() % 2 != 0)
  {
    cost = cost_vector[cost_vector.size() / 2 - 1];
  }
  else
  {
    cost = 0;
  }

 // ROS_INFO("median cost : %d",cost);
  return cost;
}

static std::vector<int> createCostMap(const pcl::PointCloud<pcl::PointXYZ> &scan)
{
  std::vector<int> cost_map;
  //create cost map by using the number of pointcloud
    for (int y = 0; y < g_cell_height; y++)
    {
      double grid_y = y - g_cell_height / 2;
      //ROS_INFO("range_y : %f ~ %f", grid_y, grid_y + g_resolution);
      for (int x = 0; x < g_cell_width; x++)
      {
        double grid_x = x - g_cell_width / 2;
       // ROS_INFO("range_x : %f ~ %f", grid_x, grid_x + g_resolution);
        int cost = 0;
        for (auto el : scan.points)
        {
          //ROS_INFO("point : %lf %lf", el.x, el.y);
          if (grid_x < el.x && grid_x + g_resolution > el.x && grid_y < el.y && grid_y + g_resolution > el.y)
            cost++;
          if(cost> 100)
            cost = 100;
        }
       cost_map.push_back(cost);
      }
    }
    return cost_map;
}

//filter CostMap with median or average filter
static void filterCostMap(std::vector<int> *cost_map)
{
  //the cell sorrounded by high costed cell make more higher cost
  for (int y = 0; y < g_cell_height; y++)
  {
    for (int x = 0; x < g_cell_width; x++)
    {
      int average_cost = 0;
      int cell = x + g_cell_height * y;
      int cell_up = cell + 1;
      int cell_down = cell - 1;
      int cell_right = cell - g_cell_height;
      int cell_left = cell + g_cell_height;
      int cell_up_right = cell_up - g_cell_height;
      int cell_up_left = cell_up + g_cell_height;
      int cell_down_right = cell_down - g_cell_height;
      int cell_down_left = cell_down + g_cell_height;
      std::vector<int> cost_vector;

      //ROS_INFO("cell %d : ",cell);
      //processing edge of cost map
      if (x == 0 && y == 0) //bottom right corner
      {
        cost_vector.push_back(cost_map->at(cell));
        cost_vector.push_back(cost_map->at(cell_up));
        cost_vector.push_back(cost_map->at(cell_left));
        cost_vector.push_back(cost_map->at(cell_up_left));
        average_cost = calculateMedian(cost_vector);
      }
      else if (x == g_cell_width - 1 && y == 0) //top right corner
      {
        cost_vector.push_back(cost_map->at(cell));
        cost_vector.push_back(cost_map->at(cell_down));
        cost_vector.push_back(cost_map->at(cell_left));
        cost_vector.push_back(cost_map->at(cell_down_left));
        average_cost = calculateMedian(cost_vector);
      }
      else if (x == 0 && y == g_cell_height - 1) //bottom left corner
      {
        cost_vector.push_back(cost_map->at(cell));
        cost_vector.push_back(cost_map->at(cell_up));
        cost_vector.push_back(cost_map->at(cell_right));
        cost_vector.push_back(cost_map->at(cell_up_right));
        average_cost = calculateMedian(cost_vector);
      }
      else if (x == g_cell_width - 1 && y == g_cell_height - 1) //top left corner
      {
        cost_vector.push_back(cost_map->at(cell));
        cost_vector.push_back(cost_map->at(cell_down));
        cost_vector.push_back(cost_map->at(cell_right));
        cost_vector.push_back(cost_map->at(cell_down_right));
        average_cost = calculateMedian(cost_vector);
      }
      else if (x == 0 && y != 0 && y != g_cell_height - 1) //bottom edge
      {
        cost_vector.push_back(cost_map->at(cell));
        cost_vector.push_back(cost_map->at(cell_up));
        cost_vector.push_back(cost_map->at(cell_right));
        cost_vector.push_back(cost_map->at(cell_left));
        cost_vector.push_back(cost_map->at(cell_up_right));
        cost_vector.push_back(cost_map->at(cell_up_left));
        average_cost = calculateMedian(cost_vector);
      }
      else if (x == g_cell_width - 1 && y != 0 && y != g_cell_height - 1) //top edge
      {
        cost_vector.push_back(cost_map->at(cell));
        cost_vector.push_back(cost_map->at(cell_down));
        cost_vector.push_back(cost_map->at(cell_right));
        cost_vector.push_back(cost_map->at(cell_left));
        cost_vector.push_back(cost_map->at(cell_down_right));
        cost_vector.push_back(cost_map->at(cell_down_left));
        average_cost = calculateMedian(cost_vector);
      }
      else if (y == 0 && x != 0 && x != g_cell_width - 1) //right edge
      {
        cost_vector.push_back(cost_map->at(cell));
        cost_vector.push_back(cost_map->at(cell_up));
        cost_vector.push_back(cost_map->at(cell_down));
        cost_vector.push_back(cost_map->at(cell_left));
        cost_vector.push_back(cost_map->at(cell_up_left));
        cost_vector.push_back(cost_map->at(cell_down_left));
        average_cost = calculateMedian(cost_vector);
      }
      else if (y == g_cell_height - 1 && x != 0 && x != g_cell_width - 1)//left edge
      {
        cost_vector.push_back(cost_map->at(cell));
        cost_vector.push_back(cost_map->at(cell_up));
        cost_vector.push_back(cost_map->at(cell_down));
        cost_vector.push_back(cost_map->at(cell_right));
        cost_vector.push_back(cost_map->at(cell_up_right));
        cost_vector.push_back(cost_map->at(cell_down_right));
        average_cost = calculateMedian(cost_vector);

      }
      else //not edge
      {
        cost_vector.push_back(cost_map->at(cell));
        cost_vector.push_back(cost_map->at(cell_up));
        cost_vector.push_back(cost_map->at(cell_down));
        cost_vector.push_back(cost_map->at(cell_right));
        cost_vector.push_back(cost_map->at(cell_left));
        cost_vector.push_back(cost_map->at(cell_up_right));
        cost_vector.push_back(cost_map->at(cell_up_left));
        cost_vector.push_back(cost_map->at(cell_down_right));
        cost_vector.push_back(cost_map->at(cell_down_left));
        average_cost = calculateMedian(cost_vector);

      }
      //ROS_INFO("original cost : %d ,average_cost : %d ", cost_map->at(cell), average_cost);
      if (average_cost > cost_map->at(cell))
        cost_map->at(cell) = average_cost;
      //ROS_INFO("final cost : %d", cost_map->at(cell));
    }
  }
}

static void createOccupancyGrid(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  auto start = std::chrono::system_clock::now();

  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::fromROSMsg(*input, scan);

  nav_msgs::OccupancyGrid og;

  og.header = input->header;
  og.info.resolution = g_resolution;
  og.info.width = g_cell_width;
  og.info.height = g_cell_height;
  og.info.origin.position.x = (-1) * g_cell_width / 2;
  og.info.origin.position.y = (-1) * g_cell_height / 2;

  //create cost map with pointcloud
  std::vector<int> cost_map = createCostMap(scan);
  filterCostMap(&cost_map);
  og.data.insert(og.data.end(),cost_map.begin(),cost_map.end());
  g_costmap_pub.publish(og);

  auto end = std::chrono::system_clock::now(); //end time
  auto dur = end - start; //processing time
  double time = std::chrono::duration_cast<std::chrono::microseconds>(dur).count(); //micro sec
  ROS_INFO("Function : %lf milli sec" ,time * 0.001);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud2gridmap");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<double>("resolution", g_resolution, 1.0);
  private_nh.param<int>("cell_width", g_cell_width, 50);
  private_nh.param<int>("cell_height", g_cell_height, 50);

  g_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("realtime_cost_map", 10);
  ros::Subscriber velodyne_sub = nh.subscribe("points_filtered", 10, createOccupancyGrid);

  ros::spin();

}

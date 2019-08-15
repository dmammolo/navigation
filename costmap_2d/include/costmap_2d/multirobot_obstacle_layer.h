/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_MULTIROBOT_OBSTACLE_LAYER_H_
#define COSTMAP_2D_MULTIROBOT_OBSTACLE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>

#include <nav_msgs/OccupancyGrid.h>

#include <laser_geometry/laser_geometry.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/MultirobotObstaclePluginConfig.h>
#include <costmap_2d/footprint.h>

namespace costmap_2d
{
/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
      index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
class CellInfo
{
public:
  /**
   * @brief  Constructor for a CellInfo objects
   * @param  cost The current cost is saved for later use
   * @param  cell_data The cell data
   * @return
   */
  CellInfo(unsigned int cost, CellData cell_data) :
      prev_cost_(cost), cell_data_(cell_data)
  {
  }
  unsigned int prev_cost_;
  CellData cell_data_;
};

class MultirobotObstacleLayer : public CostmapLayer
{
public:
  MultirobotObstacleLayer();

  virtual ~MultirobotObstacleLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void reset();

  virtual void matchSize();

  /**
   * @brief Change the values of the robot radius parameters
   * @param robot_radius The other robot radius
   */
  void setInflationParameters(double robot_radius_);

protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  std::string global_frame_;  ///< @brief The global frame for the costmap

  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters
  std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor

  bool rolling_window_;
  dynamic_reconfigure::Server<costmap_2d::MultirobotObstaclePluginConfig> *dsrv_;

  int combination_method_;
  std::vector<std::vector<int> > prev_robot_cells_rolling_;
  std::vector<std::vector<int> > prev_robot_cells_;

  boost::recursive_mutex* inflation_access_;

private:
  void reconfigureCB(costmap_2d::MultirobotObstaclePluginConfig &config, uint32_t level);

  /**
   * @brief  Lookup pre-computed distances
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  }

  void computeCaches();
  void deleteKernels();

  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y);

  double robot_radius_;
  unsigned int cell_robot_radius_;
  unsigned int cached_cell_robot_radius_;
  std::map<double, std::vector<CellInfo> > inflation_cells_;
  std::map<double, std::vector<CellInfo> > inflation_cells_rolling_;

  bool* seen_;
  int seen_size_;

  // unsigned char** cached_costs_;
  double** cached_distances_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  bool need_reinflation_;  ///< Indicates that the entire costmap should be reinflated next time around.
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_MULTIROBOT_OBSTACLE_LAYER_H_

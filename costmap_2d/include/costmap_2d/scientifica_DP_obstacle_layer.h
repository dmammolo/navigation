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
#ifndef COSTMAP_2D_SCIENTIFICA_DP_OBSTACLE_LAYER_H_
#define COSTMAP_2D_SCIENTIFICA_DP_OBSTACLE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>

#include <nav_msgs/OccupancyGrid.h>

#include <laser_geometry/laser_geometry.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ScientificaDPObstaclePluginConfig.h>
#include <costmap_2d/footprint.h>

namespace costmap_2d
{

class ScientificaDPObstacleLayer : public CostmapLayer
{
public:
  ScientificaDPObstacleLayer();

  virtual ~ScientificaDPObstacleLayer();
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
  void setParameters(double DP_min_x, double DP_min_y, double DP_max_x, double DP_max_y,
                     double add_obs_min_x, double add_obs_min_y, double add_obs_max_x, double add_obs_max_y);

protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  std::string global_frame_;  ///< @brief The global frame for the costmap

  bool rolling_window_;
  dynamic_reconfigure::Server<costmap_2d::ScientificaDPObstaclePluginConfig> *dsrv_;

  int combination_method_;

  boost::recursive_mutex* inflation_access_;

private:
  void reconfigureCB(costmap_2d::ScientificaDPObstaclePluginConfig &config, uint32_t level);

  double DP_min_x_, DP_min_y_, DP_max_x_, DP_max_y_;
  double add_obs_min_x_, add_obs_min_y_, add_obs_max_x_, add_obs_max_y_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_SCIENTIFICA_DP_OBSTACLE_LAYER_H_

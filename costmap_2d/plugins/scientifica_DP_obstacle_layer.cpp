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
 * Based on obstacle_layer.cpp from
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *
 * Addapted by
 * Author: Dario Mammolo
 *********************************************************************/
#include <costmap_2d/scientifica_DP_obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ScientificaDPObstacleLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

ScientificaDPObstacleLayer::ScientificaDPObstacleLayer()
: DP_min_x_(0)
, DP_min_y_(0)
, DP_max_x_(0)
, DP_max_y_(0)
, add_obs_min_x_(0)
, add_obs_min_y_(0)
, add_obs_max_x_(0)
, add_obs_max_y_(0)
{
  inflation_access_ = new boost::recursive_mutex();
  costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
}

void ScientificaDPObstacleLayer::onInitialize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  ScientificaDPObstacleLayer::matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
}

void ScientificaDPObstacleLayer::matchSize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
          master->getOriginX(), master->getOriginY());
  resolution_ = master->getResolution();
}

void ScientificaDPObstacleLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ScientificaDPObstaclePluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::ScientificaDPObstaclePluginConfig>::CallbackType cb = boost::bind(
      &ScientificaDPObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

ScientificaDPObstacleLayer::~ScientificaDPObstacleLayer()
{
  if (dsrv_)
      delete dsrv_;
}

void ScientificaDPObstacleLayer::reconfigureCB(costmap_2d::ScientificaDPObstaclePluginConfig &config, uint32_t level)
{
  setParameters(config);
  combination_method_ = config.combination_method;

  if (enabled_ != config.enabled) {
    enabled_ = config.enabled;
  }
}

void ScientificaDPObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  bool current = true;

  // update the global current status
  current_ = current;

  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  touch(DP_min_x_ - 0.1, DP_min_y_ - 0.1, min_x, min_y, max_x, max_y);
  touch(DP_min_x_ - 0.1, DP_max_y_ + 0.1, min_x, min_y, max_x, max_y);
  touch(DP_max_x_ + 0.1, DP_min_y_ - 0.1, min_x, min_y, max_x, max_y);
  touch(DP_max_x_ + 0.1, DP_max_y_ + 0.1, min_x, min_y, max_x, max_y);
}

void ScientificaDPObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  if (!enabled_)
    return;

  // Inflate the robots by robot_radius
  unsigned char* costmap_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  tf::StampedTransform transform;
  if(rolling_window_)
  {
    try
    {
      tf_->lookupTransform(global_frame_, "map", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_DEBUG("Failed getting Transform");
      return;
    }
  }

  // Set lethal for horizontal obstacles
  for (double x_iter = DP_min_x_; x_iter < DP_max_x_; x_iter += resolution_)
  {
    if(!rolling_window_)
    {
      // Set border region of DP as LETHAL
      setLethal(master_grid, costmap_array, x_iter, DP_min_y_);
      setLethal(master_grid, costmap_array, x_iter, DP_max_y_);

      // Set additional obstacle region of DP as LETHAL
      if (x_iter <= add_obs_max_x_ && x_iter >= add_obs_min_x_)
      {
        setLethal(master_grid, costmap_array, x_iter, add_obs_min_y_);
        setLethal(master_grid, costmap_array, x_iter, add_obs_max_y_);
      }
    }
    else
    {
      tf::Vector3 temp_vector_min;
      tf::Vector3 temp_vector_max;

      // Set border region of DP as LETHAL
      // Transform to rolling window frame
      temp_vector_min = transform * tf::Vector3(x_iter, DP_min_y_, 0.0);
      temp_vector_max = transform * tf::Vector3(x_iter, DP_max_y_, 0.0);

      setLethal(master_grid, costmap_array, x_iter, DP_min_y_, temp_vector_min);
      setLethal(master_grid, costmap_array, x_iter, DP_max_y_, temp_vector_max);

      // Set additional obstacle region of DP as LETHAL
      if (x_iter <= add_obs_max_x_ && x_iter >= add_obs_min_x_)
      {
        // Transform to rolling window frame
        temp_vector_min = transform * tf::Vector3(x_iter, add_obs_min_y_, 0.0);
        temp_vector_max = transform * tf::Vector3(x_iter, add_obs_max_y_, 0.0);

        setLethal(master_grid, costmap_array, x_iter, add_obs_min_y_, temp_vector_min);
        setLethal(master_grid, costmap_array, x_iter, add_obs_max_y_, temp_vector_max);
      }
    }
  }

  // Set lethal for vertical obstacles
  for (double y_iter = DP_min_y_; y_iter < DP_max_y_; y_iter += resolution_)
  {
    if(!rolling_window_)
    {
      // Set border region of DP as LETHAL
      setLethal(master_grid, costmap_array, DP_min_x_, y_iter);
      setLethal(master_grid, costmap_array, DP_max_x_, y_iter);

      // Set additional obstacle region of DP as LETHAL
      if (y_iter <= add_obs_max_y_ && y_iter >= add_obs_min_y_)
      {
        setLethal(master_grid, costmap_array, add_obs_min_x_, y_iter);
        setLethal(master_grid, costmap_array, add_obs_max_x_, y_iter);
      }
    }
    else
    {
      tf::Vector3 temp_vector_min;
      tf::Vector3 temp_vector_max;

      // Set border region of DP as LETHAL
      // Transform to rolling window frame
      temp_vector_min = transform * tf::Vector3(DP_min_x_, y_iter, 0.0);
      temp_vector_max = transform * tf::Vector3(DP_max_x_, y_iter, 0.0);

      setLethal(master_grid, costmap_array, DP_min_x_, y_iter, temp_vector_min);
      setLethal(master_grid, costmap_array, DP_max_x_, y_iter, temp_vector_max);

      // Set additional obstacle region of DP as LETHAL
      if (y_iter <= add_obs_max_y_ && y_iter >= add_obs_min_y_)
      {
        // Transform to rolling window frame
        temp_vector_min = transform * tf::Vector3(add_obs_min_x_, y_iter, 0.0);
        temp_vector_max = transform * tf::Vector3(add_obs_max_x_, y_iter, 0.0);

        setLethal(master_grid, costmap_array, add_obs_min_x_, y_iter, temp_vector_min);
        setLethal(master_grid, costmap_array, add_obs_max_x_, y_iter, temp_vector_max);
      }
    }
  }
}

void ScientificaDPObstacleLayer::reset()
{
    resetMaps();
    current_ = true;
}

bool ScientificaDPObstacleLayer::checkIfInStagingRegion(double &x, double &y)
{
  if ((x <= staging_max_x_ && x >= staging_min_x_) &&
      (y <= staging_max_y_ && y >= staging_min_y_))
  {
    return true;
  }
  return false;
}

void ScientificaDPObstacleLayer::setLethal(costmap_2d::Costmap2D& master_grid,
                                           unsigned char* costmap_array,
                                           double &x, double &y)
{
  if (!checkIfInStagingRegion(x, y))
  {
    unsigned int mx, my;
    // Necessary to compute in two if conditions, otherwhise function not working
    if (!worldToMap(x, y, mx, my))
    {
      ROS_DEBUG("Computing map coords failed");
    }
    else
    {
      // Set lethal
      costmap_array[master_grid.getIndex(mx, my)] = LETHAL_OBSTACLE;
    }
  }
}

void ScientificaDPObstacleLayer::setLethal(costmap_2d::Costmap2D& master_grid,
                                           unsigned char* costmap_array,
                                           double &x, double &y, tf::Vector3 &vec)
{
  if (!checkIfInStagingRegion(x, y))
  {
    unsigned int mx, my;
    // Necessary to compute in two if conditions, otherwhise function not working
    if (!worldToMap(vec.getX(), vec.getY(), mx, my))
    {
      ROS_DEBUG("Computing map coords failed");
    }
    else
    {
      // Set lethal
      costmap_array[master_grid.getIndex(mx, my)] = LETHAL_OBSTACLE;
    }
  }
}


void ScientificaDPObstacleLayer::setParameters
    (costmap_2d::ScientificaDPObstaclePluginConfig &config)
{
  DP_min_x_ = config.DP_min_x;
  DP_max_x_ = config.DP_max_x;
  DP_min_y_ = config.DP_min_y;
  DP_max_y_ = config.DP_max_y;
  add_obs_min_x_ = config.add_obs_min_x;
  add_obs_max_x_ = config.add_obs_max_x;
  add_obs_min_y_ = config.add_obs_min_y;
  add_obs_max_y_ = config.add_obs_max_y;
  staging_min_x_ = config.staging_min_x;
  staging_max_x_ = config.staging_max_x;
  staging_min_y_ = config.staging_min_y;
  staging_max_y_ = config.staging_max_y;
}

}  // namespace costmap_2d

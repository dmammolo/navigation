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
#include <costmap_2d/multirobot_obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::MultirobotObstacleLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

void MultirobotObstacleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  MultirobotObstacleLayer::matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
}

void MultirobotObstacleLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::MultirobotObstaclePluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::MultirobotObstaclePluginConfig>::CallbackType cb = boost::bind(
      &MultirobotObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

MultirobotObstacleLayer::~MultirobotObstacleLayer()
{
    if (dsrv_)
        delete dsrv_;
}

void MultirobotObstacleLayer::reconfigureCB(costmap_2d::MultirobotObstaclePluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  combination_method_ = config.combination_method;
}

void MultirobotObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  bool current = true;

  // update the global current status
  current_ = current;

  // Set robot cell cost from previous iteration
  if (!rolling_window_)
  {
    for (int i = 0; i < prev_robot_cells_.size(); i++)
    {
      // std::cout << "previous value " << int(costmap_[prev_robot_cells_[i][0]]) << std::endl;
      costmap_[prev_robot_cells_[i][0]] = prev_robot_cells_[i][1];
      // std::cout << "removing index: " << prev_robot_cells_[i][0] << std::endl;
      // std::cout << " with value " << prev_robot_cells_[i][1] << std::endl;

      double px, py;
      unsigned int mx, my;
      indexToCells(prev_robot_cells_[i][0], mx, my);
      mapToWorld(mx, my, px, py);
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }
  else
  {
    for (int i = 0; i < prev_robot_cells_rolling_.size(); i++)
    {
      costmap_[prev_robot_cells_rolling_[i][0]] = prev_robot_cells_rolling_[i][1];
      double px, py;
      unsigned int mx, my;
      indexToCells(prev_robot_cells_rolling_[i][0], mx, my);
      mapToWorld(mx, my, px, py);
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  if (!rolling_window_) prev_robot_cells_.clear();
  else prev_robot_cells_rolling_.clear();

  // Set robot cell cost to LETHAL
  for (int i = 0; i < 10; i++)
  {
    tf::StampedTransform transform;
    std::string out_string;
    std::stringstream ss;
    ss << i;
    out_string = ss.str();
    try
    {
      tf_->lookupTransform(global_frame_, "robot_" + out_string + "/base_link",
                           ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_DEBUG("robot_%i does not exist", i);
      continue;
    }

    double px, py;
    px = transform.getOrigin().getX();
    py = transform.getOrigin().getY();
    // std::cout << "x: " << px << ", y: " << py << std::endl;

    // now we need to compute the map coordinates for the observation
    unsigned int mx, my;
    if (!worldToMap(px, py, mx, my))
    {
      ROS_DEBUG("Computing map coords failed");
      continue;
    }

    // std::cout << "mx: " << mx << ", my: " << my << std::endl;

    unsigned int index = getIndex(mx, my);
    // std::cout << "index: " << index << std::endl;
    std::vector<int> temp_vec;
    if (!rolling_window_)
    {
      temp_vec.push_back(index);
      temp_vec.push_back(FREE_SPACE);
      prev_robot_cells_.push_back(temp_vec);
      costmap_[index] = LETHAL_OBSTACLE;
    }
    else
    {
      temp_vec.push_back(index);
      temp_vec.push_back(FREE_SPACE);
      prev_robot_cells_rolling_.push_back(temp_vec);
      costmap_[index] = LETHAL_OBSTACLE;
    }

    touch(px, py, min_x, min_y, max_x, max_y);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void MultirobotObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    if (!footprint_clearing_enabled_) return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

void MultirobotObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  if (footprint_clearing_enabled_)
  {
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
  }

  switch (combination_method_)
  {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void MultirobotObstacleLayer::reset()
{
    resetMaps();
    current_ = true;
}

}  // namespace costmap_2d

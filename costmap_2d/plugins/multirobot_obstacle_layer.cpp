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

MultirobotObstacleLayer::MultirobotObstacleLayer()
: robot_radius_(0)
, cell_robot_radius_(0)
, cached_cell_robot_radius_(0)
, seen_(NULL)
, cached_distances_(NULL)
, last_min_x_(-std::numeric_limits<float>::max())
, last_min_y_(-std::numeric_limits<float>::max())
, last_max_x_(std::numeric_limits<float>::max())
, last_max_y_(std::numeric_limits<float>::max())
{
  inflation_access_ = new boost::recursive_mutex();
  costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
}

void MultirobotObstacleLayer::onInitialize()
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

  MultirobotObstacleLayer::matchSize();
  current_ = true;

  if (seen_)
    delete[] seen_;
  seen_ = NULL;
  seen_size_ = 0;
  need_reinflation_ = false;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);

}

void MultirobotObstacleLayer::matchSize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
          master->getOriginX(), master->getOriginY());
  resolution_ = master->getResolution();
  cell_robot_radius_ = cellDistance(robot_radius_);
  computeCaches();

  unsigned int size_x = master->getSizeInCellsX(), size_y = master->getSizeInCellsY();
  if (seen_)
    delete[] seen_;
  seen_size_ = size_x * size_y;
  seen_ = new bool[seen_size_];
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
  deleteKernels();
  if (dsrv_)
      delete dsrv_;
}

void MultirobotObstacleLayer::reconfigureCB(costmap_2d::MultirobotObstaclePluginConfig &config, uint32_t level)
{
  setInflationParameters(config.robot_radius);
  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  combination_method_ = config.combination_method;

  if (enabled_ != config.enabled) {
    enabled_ = config.enabled;
    need_reinflation_ = true;
  }
}

void MultirobotObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  bool current = true;

  // update the global current status
  current_ = current;

  // Iterator
  std::map<double, std::vector<CellInfo> >& clearing_cells =
    (rolling_window_) ? inflation_cells_rolling_ : inflation_cells_;

  std::map<double, std::vector<CellInfo> >::iterator bin;

  // Set robot cell cost from previous iteration
  for (bin = clearing_cells.begin(); bin != clearing_cells.end(); ++bin)
  {
    for (int i = 0; i < bin->second.size(); ++i)
    {
      costmap_[bin->second[i].cell_data_.index_] = bin->second[i].prev_cost_;

      double px, py;
      mapToWorld(bin->second[i].cell_data_.x_, bin->second[i].cell_data_.y_, px, py);
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  // Clear cell lists
  if (!rolling_window_)
  {
    inflation_cells_.clear();
    // make sure the inflation list is empty at the beginning of the cycle (should always be true)
    ROS_ASSERT_MSG(inflation_cells_.empty(), "The inflation list must be empty at the beginning of inflation");
  }
  else
  {
    inflation_cells_rolling_.clear();
    // make sure the inflation list is empty at the beginning of the cycle (should always be true)
    ROS_ASSERT_MSG(inflation_cells_rolling_.empty(), "The inflation list must be empty at the beginning of inflation");
  }

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

    // now we need to compute the map coordinates for the observation
    unsigned int mx, my;
    if (!worldToMap(px, py, mx, my))
    {
      ROS_DEBUG("Computing map coords failed");
      continue;
    }

    unsigned int index = getIndex(mx, my);
    std::vector<CellInfo>& obs_bin =
      (rolling_window_) ? inflation_cells_rolling_[0.0] : inflation_cells_[0.0];

    obs_bin.push_back(CellInfo(FREE_SPACE, CellData(index, mx, my, mx, my)));
    costmap_[index] = LETHAL_OBSTACLE;

    touch(px, py, min_x, min_y, max_x, max_y);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  if (need_reinflation_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  }
  else
  {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - robot_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - robot_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + robot_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + robot_radius_;
  }
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
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  if (!enabled_ || (cell_robot_radius_ == 0))
    return;

  // Inflate the robots by robot_radius
  unsigned char* costmap_array = getCharMap();
  unsigned int size_x = getSizeInCellsX(), size_y = getSizeInCellsY();

  if (seen_ == NULL) {
    ROS_WARN("InflationLayer::updateBounds(): seen_ array is NULL");
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  else if (seen_size_ != size_x * size_y)
  {
    ROS_WARN("InflationLayer::updateBounds(): seen_ array size is wrong");
    delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  memset(seen_, false, size_x * size_y * sizeof(bool));

  // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  std::map<double, std::vector<CellInfo> >& cells =
      (rolling_window_) ? inflation_cells_rolling_ : inflation_cells_;
  std::map<double, std::vector<CellInfo> >::iterator bin;
  for (bin = cells.begin(); bin != cells.end(); ++bin)
  {
    for (int i = 0; i < bin->second.size(); ++i)
    {
      // process all cells at distance dist_bin.first
      const CellData& cell = bin->second[i].cell_data_;

      unsigned int index = cell.index_;

      // ignore if already visited
      if (seen_[index])
      {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;

      // assign the cost LETHAL_OBSTACLE
      costmap_array[index] = LETHAL_OBSTACLE;
      bin->second[i].prev_cost_ = FREE_SPACE;

      // attempt to put the neighbors of the current cell onto the inflation list
      if (mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy);
      if (my > 0)
        enqueue(index - size_x, mx, my - 1, sx, sy);
      if (mx < size_x - 1)
        enqueue(index + 1, mx + 1, my, sx, sy);
      if (my < size_y - 1)
        enqueue(index + size_x, mx, my + 1, sx, sy);
    }
  }


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

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
inline void MultirobotObstacleLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index])
  {
    // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
    if (distance > cell_robot_radius_)
      return;

    // push the cell data onto the inflation list and mark
    if (!rolling_window_)
    {
      inflation_cells_[distance].push_back(CellInfo(FREE_SPACE, CellData(index, mx, my, src_x, src_y)));
    }
    else
    {
      inflation_cells_rolling_[distance].push_back(CellInfo(FREE_SPACE, CellData(index, mx, my, src_x, src_y)));
    }
  }
}

void MultirobotObstacleLayer::computeCaches()
{
  if (cell_robot_radius_ == 0)
    return;

  // based on the inflation radius... compute distance and cost caches
  if (cell_robot_radius_ != cached_cell_robot_radius_)
  {
    deleteKernels();

    cached_distances_ = new double*[cell_robot_radius_ + 2];

    for (unsigned int i = 0; i <= cell_robot_radius_ + 1; ++i)
    {
      cached_distances_[i] = new double[cell_robot_radius_ + 2];
      for (unsigned int j = 0; j <= cell_robot_radius_ + 1; ++j)
      {
        cached_distances_[i][j] = hypot(i, j);
      }
    }

    cached_cell_robot_radius_ = cell_robot_radius_;
  }
}

void MultirobotObstacleLayer::deleteKernels()
{
  if (cached_distances_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_robot_radius_ + 1; ++i)
    {
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }
}

void MultirobotObstacleLayer::setInflationParameters(double robot_radius)
{
  if (robot_radius_ != robot_radius)
  {
    // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
    // when accessing the cached arrays
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);

    robot_radius_ = robot_radius;
    cell_robot_radius_ = cellDistance(robot_radius_);
    need_reinflation_ = true;
    computeCaches();
  }
}

}  // namespace costmap_2d

// This file makes substantial reuse of code from the ros-industrial/industrial_calibration repo,
// specifically the file target_finder/src/nodes/target_tracker.cpp.  The following license applies
// to that reused code:

/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2021, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <utility>  // std::pair
#include <vector>   // std::vector

#include <opc/ua/client/client.h>
#include <opc/ua/subscription.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_observer.hpp>
#include <industrial_extrinsic_cal/pose_yaml_parser.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <target_finder/target_finderConfig.h>

#include <cognex_demo/plc_interface.h>

namespace demo
{

std::vector<std::pair<float, float>> extractUV(const industrial_extrinsic_cal::CameraObservations& observations)
{
  std::vector<std::pair<float, float>> uv_coords;
  uv_coords.reserve(observations.size());
  for (const industrial_extrinsic_cal::Observation& obs : observations)
  {
    uv_coords.emplace_back(
        static_cast<float>(obs.image_loc_x),
        static_cast<float>(obs.image_loc_y));
  }
  return uv_coords;
}

} // namespace demo

class TargetTracker
{
public:
  TargetTracker(
      ros::NodeHandle nh,
      const std::string& image_topic,
      const std::string& camera_name);

  void initMCircleTarget(int rows, int cols, double circle_dia, double spacing);

  void dynReConfCallBack(target_finder::target_finderConfig& config, uint32_t level);

  void update();

private:
  ros::NodeHandle nh_;
  boost::shared_ptr<industrial_extrinsic_cal::Target> target_;
  size_t expected_num_observations_;
  double allowable_cost_per_observation_;
  std::shared_ptr<dynamic_reconfigure::Server<target_finder::target_finderConfig>> reconf_srv_;
  dynamic_reconfigure::Server<target_finder::target_finderConfig>::CallbackType reconf_CB_;
  std::shared_ptr<industrial_extrinsic_cal::ROSCameraObserver> camera_observer_;
  int width_, height_;
  industrial_extrinsic_cal::Roi input_roi_;
  demo::PLCInterface plci_;
};

TargetTracker::TargetTracker(
    ros::NodeHandle nh,
    const std::string& image_topic = "/pylon_camera_node/image_rect",
    const std::string& camera_name = "pylon_camera_node")
{
  nh_ = nh;
  ros::NodeHandle pnh("~");

  // ROS parameters

  // Dynamic Reconfigure parameters
  // target_tracker/target_rows, target_tracker/target_cols, target_tracker/target_circle_dia,
  // target_tracker/target_spacing

  if (!pnh.getParam("allowable_cost_per_observation", allowable_cost_per_observation_))
  {
    allowable_cost_per_observation_ = .25;
  }

  // Get camera parameters (we don't need the intrinsics here, so we use throwaway floats)
  double fx, fy, cx, cy, k1, k2, k3, p1, p2;
  camera_observer_ = std::shared_ptr<industrial_extrinsic_cal::ROSCameraObserver>(new industrial_extrinsic_cal::ROSCameraObserver(image_topic, camera_name));
  camera_observer_->pullCameraInfo(fx, fy, cx, cy, k1, k2, k3, p1, p2, width_, height_);
  camera_observer_->use_circle_detector_ = true;
  input_roi_.x_min = 0;
  input_roi_.y_min = 0;
  input_roi_.x_max = width_;
  input_roi_.y_max = height_;

  // initialize the dynamic reconfigure callback
  reconf_srv_.reset(new dynamic_reconfigure::Server<target_finder::target_finderConfig>(nh_));
  dynamic_reconfigure::Server<target_finder::target_finderConfig>::CallbackType f;
  f = boost::bind(&TargetTracker::dynReConfCallBack, this, _1, _2);
  reconf_srv_->setCallback(f);

  // subscribe to image for updating with each image
  camera_observer_->startTargetTrack();
}

void TargetTracker::dynReConfCallBack(target_finder::target_finderConfig& config, uint32_t /*level*/)
{
  // resize target
  initMCircleTarget(config.target_rows, config.target_cols, config.target_circle_dia, config.target_spacing);
}

void TargetTracker::initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
{
  target_ = boost::make_shared<industrial_extrinsic_cal::Target>();

  // constant parameters
  target_->target_type_ = pattern_options::ModifiedCircleGrid;
  target_->is_moving_ = false;
  target_->circle_grid_parameters_.is_symmetric = true;

  // dynamic parameters
  target_->circle_grid_parameters_.pattern_rows = rows;
  target_->circle_grid_parameters_.pattern_cols = cols;
  target_->circle_grid_parameters_.circle_diameter = circle_dia;
  target_->circle_grid_parameters_.spacing = spacing;
  expected_num_observations_ = static_cast<size_t>(rows * cols);
  // create a grid of points
  target_->pts_.clear();
  target_->num_points_ = static_cast<unsigned int>(rows * cols);
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      industrial_extrinsic_cal::Point3d point;
      point.x = j * spacing;
      point.y = (rows - 1 - i) * spacing;
      point.z = 0.0;
      target_->pts_.push_back(point);
    }
  }
  camera_observer_->clearTargets();
  industrial_extrinsic_cal::Cost_function cost_type =
      industrial_extrinsic_cal::cost_functions::CircleCameraReprjErrorPK;
  camera_observer_->addTarget(target_, input_roi_, cost_type);
}

void TargetTracker::update()
{
  if (!camera_observer_->observationsDone()) return;
  camera_observer_->clearObservations();
  industrial_extrinsic_cal::CameraObservations camera_observations;
  if (camera_observer_->getObservations(camera_observations))
  {
    if (!camera_observer_->checkObservationProclivity(camera_observations))
    {
      ROS_ERROR("Proclivity Error");
    }
    else if (camera_observations.size() != expected_num_observations_)
    {
      ROS_ERROR("number of observations != expected %ld != %ld", camera_observations.size(), expected_num_observations_);
    }
    else
    {
      std::vector<std::pair<float, float>> uv_coords = demo::extractUV(camera_observations);
      plci_.writeTag(demo::node_ids::FEATURE_UV_PATH, uv_coords);
    }
  }
  return;  // error is already reported through ROS_ERROR
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_detector_node");
  ros::NodeHandle nh ("~/target_tracker");
  TargetTracker target_tracker(nh);

  while (ros::ok())
  {
    target_tracker.update();
    ros::spinOnce();
  }

  ros::waitForShutdown();
  return 0;
}

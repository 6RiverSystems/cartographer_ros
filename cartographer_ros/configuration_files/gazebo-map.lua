-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "srsnode_motion/odometry",
  odom_frame = "srsnode_motion/odometry",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_point_clouds = 0,
  num_subdivisions_per_laser_scan = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.1,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = .5,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Rangefinder points outside these ranges will be dropped.
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 25.
-- Points beyond max_range will be inserted with this length as empty space.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.

-- Whether to solve the online scan matching first using the correlative scan matcher to generate a good starting point for Ceres.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 30
-- Make the value larger to trust odometry more and better handle long aisle
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 40
-- Number of scans before adding a new submap. Each submap will get twice the number of scans inserted: First for initialization without being matched against, then while being matched.
-- Generate more submaps and thus should have more locally accurate submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

-- Scaling parameter for Huber loss function.
POSE_GRAPH.optimization_problem.huber_scale = 1e2
-- Minimum linear search window in which the best possible scan alignment will be found.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
-- Online loop closure: If positive, will run the loop closure while the map is built
POSE_GRAPH.optimize_every_n_nodes = 60
-- Threshold for the scan match score below which a match is not considered. Low scores indicate that the scan and map do not look similar.
-- Make the loop closure not too aggressive
POSE_GRAPH.constraint_builder.min_score = 0.55

return options

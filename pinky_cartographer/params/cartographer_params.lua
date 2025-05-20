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
-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 0.5,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.05   -- min_range 이하의 라이다 데이터는 무시(m)
TRAJECTORY_BUILDER_2D.max_range = 3.5 -- max_range 이상의 라이다 데이터는 무시(m)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3. -- 라이다가 데이터를 받지 못한 방향에 대한 가상의 거리값(m)
TRAJECTORY_BUILDER_2D.use_imu_data = true  --imu 사용 여부
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 이전 라이다 스캔과 현재 스캔을 비교해 초기위치 예측 알고리즘 사용 여부
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.02  -- 매칭을 시도할 범위. 값이 작을수록 빠르지만 정확도가 떨어짐
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1 -- 스캔매칭의 평행이동
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1 -- 스캔 매칭의 회전
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- 마지막 포즈랑 비교해서 회전이 0.2 이하면 무시. 중복 데이터 제거로 계산효율 높임
-- for current lidar only 1 is good value
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 누적해서 사용할 라이다 데이터의 수. 1이면 바로 처리. 작을수록 실시간성 높아지나 노이즈가 늘어난다.
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 루프 클로저에서 같은 장소라 판단하는 기준값. 이 값 이상이면 그래프에 제약조건(edge)가 추가된다 
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- relocalization시에 올바른 위치로 판단하는 정확도 기준. 스캔매칭 점수 해당 값 이하면 위치를 신뢰하지 않음

-- POSE_GRAPH.optimize_every_n_nodes = 0

return options
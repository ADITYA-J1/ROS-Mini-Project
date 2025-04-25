include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.optimization_problem.huber_scale = 1e1


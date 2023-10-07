#pragma once
#include "maf_interface/maf_actionlib.h"
#include "maf_interface/maf_geometry.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_nav {

struct Odometry {
  maf_std::Header header;
  std::string child_frame_id;
  maf_geometry::PoseWithCovariance pose;
  maf_geometry::TwistWithCovariance twist;
};

struct GetMapGoal {};

struct GetMapFeedback {};

struct Path {
  maf_std::Header header;
  std::vector<maf_geometry::PoseStamped> poses;
};

struct GridCells {
  maf_std::Header header;
  float cell_width;
  float cell_height;
  std::vector<maf_geometry::Point> cells;
};

struct MapMetaData {
  uint64_t map_load_time;
  float resolution;
  uint32_t width;
  uint32_t height;
  maf_geometry::Pose origin;
};

struct OccupancyGrid {
  maf_std::Header header;
  MapMetaData info;
  std::vector<int8_t> data;
};

struct GetMapActionFeedback {
  maf_std::Header header;
  maf_actionlib::GoalStatus status;
  GetMapFeedback feedback;
};

struct GetMapActionGoal {
  maf_std::Header header;
  maf_actionlib::GoalID goal_id;
  GetMapGoal goal;
};

struct GetMapResult {
  OccupancyGrid map;
};

struct GetMapActionResult {
  maf_std::Header header;
  maf_actionlib::GoalStatus status;
  GetMapResult result;
};

struct GetMapAction {
  GetMapActionGoal action_goal;
  GetMapActionResult action_result;
  GetMapActionFeedback action_feedback;
};

} // namespace maf_nav

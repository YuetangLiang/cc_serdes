#pragma once
#include "maf_interface/maf_geometry.h"
#include "maf_interface/maf_nav.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_davinci {

struct vision_fs {
  std::vector<maf_geometry::Point32> d3_border_pts;
  std::vector<int32_t> border_type;
};

struct grid {
  maf_std::Header header;
  maf_std::Header header_pub;
  float cell_size;
  std::vector<maf_geometry::Point32> cells;
  std::vector<int8_t> w;
};

struct vision_human {
  int32_t track_id;
  float d3_x;
  float d3_yl;
  float d3_yr;
  float d3_vx;
  float d3_vy;
};

struct location {
  maf_std::Header header;
  maf_std::Header header_pub;
  maf_geometry::Pose pose;
  double latitude;
  double longitude;
  double altitude;
  double pitch;
  double roll;
  double yaw;
  bool loc_vaild;
  uint64_t status;
  double quality;
  double accuracy;
};

struct paths {
  maf_std::Header header;
  maf_std::Header header_pub;
  std::vector<maf_nav::Path> paths;
};

struct line {
  maf_std::Header header;
  maf_std::Header header_pub;
  std::vector<maf_geometry::Point32> pts;
};

struct lot {
  maf_std::Header header;
  maf_std::Header header_pub;
  int32_t id;
  int32_t lane_id;
  maf_geometry::Point32 cpos;
  maf_geometry::Point32 epos;
  std::vector<maf_geometry::Point32> corners;
  maf_std::Bool is_occupied;
  maf_std::Bool is_selected;
  maf_std::Bool is_checked;
};

struct vision_car {
  int32_t track_id;
  float d3_x;
  float d3_y;
  float theta;
  float d3_vx;
  float d3_vy;
  std::vector<maf_geometry::Point32> corners;
  int32_t closest_corner_idx;
};

struct trajectory {
  maf_std::Header header;
  maf_std::Header header_pub;
  maf_nav::Path path;
  std::vector<float> vel;
};

struct vision {
  maf_std::Header header;
  maf_std::Header header_pub;
  std::vector<vision_human> human_array;
  std::vector<vision_car> car_array;
  vision_fs fs;
};

struct lane {
  maf_std::Header header;
  maf_std::Header header_pub;
  line cline;
  std::vector<line> bline;
};

struct planning {
  maf_std::Header header;
  maf_std::Header header_pub;
  trajectory motion_ref_traj;
  trajectory todo_ref_traj;
  trajectory history_traj;
  float health;
};

} // namespace maf_davinci

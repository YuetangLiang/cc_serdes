#pragma once
#include "maf_interface/maf_geometry.h"
#include "maf_interface/maf_sensor.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_lidar_interface {

struct LidarObjectPolygon {
  uint64_t track_id;
  uint32_t type;
  maf_geometry::Point position;
  maf_geometry::Vector3 sigma_position;
  maf_geometry::Vector3 velocity;
  maf_geometry::Vector3 sigma_velocity;
  float length;
  float width;
  float height;
  float sigma_length;
  float sigma_width;
  float sigma_height;
  float theta;
  float sigma_theta;
  float score;
  std::vector<maf_geometry::Point> polygon_bottom;
  std::vector<maf_geometry::Point> polygon_top;
};

struct LidarObject {
  uint64_t track_id;
  uint32_t type;
  maf_geometry::Point position;
  maf_geometry::Vector3 sigma_position;
  maf_geometry::Vector3 velocity;
  maf_geometry::Vector3 sigma_velocity;
  float length;
  float width;
  float height;
  float sigma_length;
  float sigma_width;
  float sigma_height;
  float theta;
  float sigma_theta;
};

struct LidarObb {
  int32_t id;
  std::vector<float> center;
  std::vector<float> size;
  std::vector<float> rpy;
};

struct LidarObjectPolygonArray {
  maf_std::Header header;
  std::vector<LidarObjectPolygon> object_array;
};

struct LidarObjectArray {
  maf_std::Header header;
  std::vector<LidarObject> object_array;
};

struct LidarDetections {
  maf_std::Header header;
  maf_sensor::PointCloud2 obstacle_cloud;
  std::vector<LidarObb> obb_array;
};

} // namespace maf_lidar_interface

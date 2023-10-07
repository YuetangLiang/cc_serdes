#pragma once
#include "maf_interface/maf_perception_interface.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_v2x {

struct V2xOverHorizonData {
  int32_t intersection_id;
  std::string video_url;
};

struct V2xMeta {
  uint64_t timestamp_us;
  std::string frame_id;
};

struct V2xPhaseState {
  maf_perception_interface::TrafficLightStatusEnum status;
  double start_time;
  double likely_end_time;
};

struct V2xIntersectionData {
  int32_t intersection_id;
  double enter_angle;
  double exit_angle;
};

struct V2xPerceptionObjectType {
  uint32_t value;

  enum : uint32_t {
    UNKNOWN = 0,
    PEDESTRIAN = 1,
    NON_MOTOR = 2,
    MOTOR = 3,
    OTHER = 4,
  };
};

struct V2xCollisionWarningData {
  std::vector<int32_t> id;
  std::string warning;
};

struct V2xOverHorizonInfo {
  uint8_t available;
  V2xOverHorizonData v2x_over_horizon_data;

  enum : uint8_t {
    V2X_OVER_HORIZON_DATA = 1,
  };
};

struct V2xPhase {
  int32_t phase_id;
  V2xPhaseState v2x_phase_state;
};

struct V2xIntersectionInfo {
  uint8_t available;
  V2xIntersectionData v2x_intersection_data;

  enum : uint8_t {
    V2X_INTERSECTION_DATA = 1,
  };
};

struct V2xPerceptionObject {
  V2xPerceptionObjectType type;
  double longitude;
  double latitude;
  double elevation;
  double speed;
  double heading;
  double width;
  double length;
  double height;
};

struct V2xCollisionWarningInfo {
  uint8_t available;
  V2xCollisionWarningData v2x_collision_warning_data;

  enum : uint8_t {
    V2X_COLLISION_WARNING_DATA = 1,
  };
};

struct V2xOverHorizon {
  maf_std::Header header;
  V2xMeta meta;
  V2xOverHorizonInfo v2x_over_horizon_info;
};

struct V2xIntersectionState {
  int32_t intersection_id;
  std::vector<V2xPhase> v2x_phase;
};

struct V2xIntersection {
  maf_std::Header header;
  V2xMeta meta;
  V2xIntersectionInfo v2x_intersection_info;
};

struct V2xPerceptionInfo {
  uint8_t available;
  std::vector<V2xPerceptionObject> v2x_perception_object;

  enum : uint8_t {
    V2X_PERCEPTION_OBJECT = 1,
  };
};

struct V2xCollisionWarning {
  maf_std::Header header;
  V2xMeta meta;
  V2xCollisionWarningInfo v2x_collision_warning_info;
};

struct V2xTrafficLightInfo {
  uint8_t available;
  std::vector<V2xIntersectionState> v2x_intersection_state;

  enum : uint8_t {
    V2X_INTERSECTION_STATE = 1,
  };
};

struct V2xPerception {
  maf_std::Header header;
  V2xMeta meta;
  V2xPerceptionInfo v2x_perception_info;
};

struct V2xTrafficLight {
  maf_std::Header header;
  V2xMeta meta;
  V2xTrafficLightInfo v2x_traffic_light_info;
};

} // namespace maf_v2x

#pragma once
#include "maf_interface/maf_actionlib.h"
#include "maf_interface/maf_geometry.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_tf2 {

struct LookupTransformGoal {
  std::string target_frame;
  std::string source_frame;
  uint64_t source_time;
  int64_t timeout;
  uint64_t target_time;
  std::string fixed_frame;
  bool advanced;
};

struct TF2Error {
  uint8_t error;
  std::string error_string;

  enum : uint8_t {
    NO_ERROR = 0,
    LOOKUP_ERROR = 1,
    CONNECTIVITY_ERROR = 2,
    EXTRAPOLATION_ERROR = 3,
    INVALID_ARGUMENT_ERROR = 4,
    TIMEOUT_ERROR = 5,
    TRANSFORM_ERROR = 6,
  };
};

struct TFMessage {
  std::vector<maf_geometry::TransformStamped> transforms;
};

struct LookupTransformFeedback {};

struct LookupTransformActionGoal {
  maf_std::Header header;
  maf_actionlib::GoalID goal_id;
  LookupTransformGoal goal;
};

struct LookupTransformResult {
  maf_geometry::TransformStamped transform;
  TF2Error error;
};

struct LookupTransformActionFeedback {
  maf_std::Header header;
  maf_actionlib::GoalStatus status;
  LookupTransformFeedback feedback;
};

struct LookupTransformActionResult {
  maf_std::Header header;
  maf_actionlib::GoalStatus status;
  LookupTransformResult result;
};

struct LookupTransformAction {
  LookupTransformActionGoal action_goal;
  LookupTransformActionResult action_result;
  LookupTransformActionFeedback action_feedback;
};

} // namespace maf_tf2

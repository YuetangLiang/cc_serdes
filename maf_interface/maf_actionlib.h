#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_actionlib {

struct TestRequestGoal {
  int32_t terminate_status;
  bool ignore_cancel;
  std::string result_text;
  int32_t the_result;
  bool is_simple_client;
  int64_t delay_accept;
  int64_t delay_terminate;
  int64_t pause_status;

  enum : int32_t {
    TERMINATE_SUCCESS = 0,
    TERMINATE_ABORTED = 1,
    TERMINATE_REJECTED = 2,
    TERMINATE_LOSE = 3,
    TERMINATE_DROP = 4,
    TERMINATE_EXCEPTION = 5,
  };
};

struct TwoIntsResult {
  int64_t sum;
};

struct TestRequestFeedback {};

struct TestGoal {
  int32_t goal;
};

struct TestRequestResult {
  int32_t the_result;
  bool is_simple_server;
};

struct TwoIntsFeedback {};

struct TestResult {
  int32_t result;
};

struct TestFeedback {
  int32_t feedback;
};

struct TwoIntsGoal {
  int64_t a;
  int64_t b;
};

struct GoalID {
  uint64_t stamp;
  std::string id;
};

struct GoalStatus {
  GoalID goal_id;
  uint8_t status;
  std::string text;

  enum : uint8_t {
    PENDING = 0,
    ACTIVE = 1,
    PREEMPTED = 2,
    SUCCEEDED = 3,
    ABORTED = 4,
    REJECTED = 5,
    PREEMPTING = 6,
    RECALLING = 7,
    RECALLED = 8,
    LOST = 9,
  };
};

struct TestActionGoal {
  maf_std::Header header;
  GoalID goal_id;
  TestGoal goal;
};

struct TestRequestActionGoal {
  maf_std::Header header;
  GoalID goal_id;
  TestRequestGoal goal;
};

struct TwoIntsActionGoal {
  maf_std::Header header;
  GoalID goal_id;
  TwoIntsGoal goal;
};

struct TwoIntsActionFeedback {
  maf_std::Header header;
  GoalStatus status;
  TwoIntsFeedback feedback;
};

struct TestRequestActionFeedback {
  maf_std::Header header;
  GoalStatus status;
  TestRequestFeedback feedback;
};

struct TwoIntsActionResult {
  maf_std::Header header;
  GoalStatus status;
  TwoIntsResult result;
};

struct TestActionFeedback {
  maf_std::Header header;
  GoalStatus status;
  TestFeedback feedback;
};

struct TestRequestActionResult {
  maf_std::Header header;
  GoalStatus status;
  TestRequestResult result;
};

struct TestActionResult {
  maf_std::Header header;
  GoalStatus status;
  TestResult result;
};

struct GoalStatusArray {
  maf_std::Header header;
  std::vector<GoalStatus> status_list;
};

struct TwoIntsAction {
  TwoIntsActionGoal action_goal;
  TwoIntsActionResult action_result;
  TwoIntsActionFeedback action_feedback;
};

struct TestAction {
  TestActionGoal action_goal;
  TestActionResult action_result;
  TestActionFeedback action_feedback;
};

struct TestRequestAction {
  TestRequestActionGoal action_goal;
  TestRequestActionResult action_result;
  TestRequestActionFeedback action_feedback;
};

} // namespace maf_actionlib

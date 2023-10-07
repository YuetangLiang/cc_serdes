#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_recorder {

struct EventParamV2 {
  bool sticky;
  int64_t before;
  int64_t after;
  std::vector<std::string> topics;
  std::vector<std::string> tags;
};

struct EventV2 {
  maf_std::Header header;
  std::string name;
  std::string collect_type;
  bool sticky;
  int64_t before;
  int64_t after;
  std::vector<std::string> topics;
  std::vector<std::string> tags;
  bool debug_reject;
  std::string detail;
};

struct ReceivingSample {
  uint64_t stamp;
  int64_t interval;
  double data_rate;
  std::vector<std::string> topics;
  std::vector<std::string> data_types;
  std::vector<uint32_t> msg_counts;
  std::vector<float> msg_rates;
};

struct RecordingV2 {
  maf_std::Header header;
  bool start;
  std::string name;
  std::string collect_type;
  std::vector<std::string> topics;
  std::vector<std::string> tags;
  bool debug_reject;
  uint64_t debug_start;
  std::string detail;
};

struct FilterParamV2 {
  std::vector<std::string> topics;
  std::vector<std::string> aliases;
  std::vector<uint64_t> stamps;
};

struct ImageTopicConfig {
  std::vector<std::string> base_topics;
  std::string in_transport;
  std::string in_suffix;
  std::string out_transport;
  std::string out_suffix;
  std::string out_params;
};

struct Tag {
  maf_std::Header header;
  std::string name;
  std::string detail;
};

struct FilterV2 {
  maf_std::Header header;
  std::string name;
  std::string collect_type;
  std::vector<std::string> topics;
  std::vector<std::string> aliases;
  std::vector<uint64_t> stamps;
  bool debug_reject;
  std::string detail;
};

struct BagInfo {
  uint32_t index;
  std::string filename;
  double start_time;
  double end_time;
  uint64_t size;
  uint32_t message_cnt;
  std::vector<std::string> topic_name;
  std::vector<std::string> topic_date_type;
  std::vector<uint32_t> topic_message_cnt;
};

struct RecordingParamV2 {
  bool start;
  std::vector<std::string> topics;
  std::vector<std::string> tags;
};

struct VersionInfo {
  std::string image_version;
  std::string recorder_version;
  std::string config_variant;
  std::string config_version;
  std::string format_version;
};

struct TopicInfo {
  std::string topic_name;
  std::string topic_date_type;
  uint32_t topic_message_cnt;
};

struct ExportJobInfo {
  uint64_t stamp;
  std::string type;
  std::string status;
  std::string name;
  std::vector<BagInfo> bag_info;
};

struct ExportJobInfoV2 {
  int32_t sequence;
  uint64_t stamp;
  std::string collect_type;
  std::string name;
  std::string export_type;
  std::string status;
  std::string reason;
  std::vector<EventParamV2> event_param;
  std::vector<RecordingParamV2> recording_param;
  std::vector<FilterParamV2> filter_param;
};

} // namespace maf_recorder

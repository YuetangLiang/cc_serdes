#pragma once
#include "maf_interface/maf_geometry.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_framework_status {

struct ManagerNotify {
  std::string message;
};

struct ModuleType {
  uint8_t value;

  enum : uint8_t {
    PREDICTION = 0,
    ROUTING = 1,
    PLANNING = 2,
    CONTROLLER = 3,
    ENDPOINT = 4,
    VEHICLE_REPORTER = 5,
    WORLDMODEL = 6,
    RADAR_PERCEPTION = 7,
    VISION_PERCEPTION = 8,
    LIDAR_PERCEPTION = 9,
    FUSION_PERCEPTION = 10,
    LOCATION = 11,
  };
};

struct StatisticValue {
  float average;
  float standard_deviation;
  float max_value;
  float min_value;
  std::string reserved_info;
};

struct MDKDiRoadId {
  int64_t tile_id;
  int64_t road_count;
  bool dir_reversed;
};

struct RoutingRequestType {
  uint8_t value;

  enum : uint8_t {
    ROUTE_ENDS = 0,
    JSON_FILE = 1,
  };
};

struct CacheBagDetailsData {
  uint8_t health_status;
  uint64_t timestamp_us;
  std::string reserved_info;
  std::string message;

  enum : uint8_t {
    NORM = 0,
    WARNING = 1,
    ERROR = 2,
  };
};

struct SingleDiskStatus {
  std::string sn;
  std::string disk_type;
  std::string status;
  uint64_t space_size;
  uint64_t space_used;
  std::string driver;
  std::string speed;
  int64_t availability;
  std::string reserved_info;
};

struct ExportDetailsData {
  uint8_t health_status;
  uint64_t timestamp_us;
  std::string reserved_info;
  std::string message;

  enum : uint8_t {
    NORM = 0,
    WARNING = 1,
    ERROR = 2,
  };
};

struct OverallStatus {
  maf_std::Header header;
  uint64_t timestamp_us;
  uint8_t status;
  uint8_t detail_status;
  std::string message;

  enum : uint8_t {
    STOP = 0,
    SAFTY_RUNNING = 1,
    NORMAL_RUNNING = 2,
    TAKEOVER = 3,
    NONE = 0,
    SAFTY_RUNNING_WAIT_TAKEOVER = 1,
  };
};

struct SingleGPUStatus {
  uint8_t gpu_id;
  float gpu_current_percent;
  float gpu_average_percent;
  float gpu_memory_current_percent;
  float gpu_memory_average_percent;
  float gpu_temperature;
  float gpu_power;
  std::vector<std::string> top_processes;
  std::string reserved_info;
};

struct DiskStatusData {
  uint8_t health_status;
  float disk_usage;
  std::string disk_io_counters;
  std::string reserved_info;
  std::string message;

  enum : uint8_t {
    NORM = 0,
    WARNING = 1,
    ERROR = 2,
  };
};

struct TimeStatusData {
  uint8_t health_status;
  std::string reserved_info;
  std::string message;

  enum : uint8_t {
    NORM = 0,
    WARNING = 1,
    ERROR = 2,
  };
};

struct CPUStatusData {
  uint8_t health_status;
  uint8_t cpu_count;
  float cpu_current_percent;
  float cpu_average_percent;
  std::vector<float> cpu_average_load;
  float cpu_average_temperature;
  float cpu_average_frequency;
  std::vector<std::string> top_processes;
  std::string reserved_info;
  std::string message;

  enum : uint8_t {
    NORM = 0,
    WARNING = 1,
    ERROR = 2,
  };
};

struct SingleProcessStatus {
  std::string process_name;
  bool is_alive;
  uint8_t num_threads;
  float voluntary_ctxt_switches_rate;
  float nonvoluntary_ctxt_switches_rate;
  float cpu_current_percent;
  float cpu_average_percent;
  float memory_current_percent;
  float memory_average_percent;
  std::string reserved_info;
};

struct MemoryStatusData {
  uint8_t health_status;
  float virtual_memory;
  float swap_memory;
  std::vector<std::string> top_processes;
  std::string reserved_info;
  std::string message;

  enum : uint8_t {
    NORM = 0,
    WARNING = 1,
    ERROR = 2,
  };
};

struct NetAliveStatus {
  std::string name;
  uint32_t loss_count;
  std::string reserved_info;
};

struct NetCardStatus {
  std::string net_card_name;
  float bytes_sent_rate;
  float bytes_recv_rate;
  float packets_sent_rate;
  float packets_recv_rate;
  float errin_rate;
  float errout_rate;
  float dropin_rate;
  float dropout_rate;
  bool net_card_isup;
  uint8_t duplex;
  uint32_t speed;
  uint32_t mtu;
  std::string reserved_info;
};

struct NodeStatus {
  uint64_t timestamp_us;
  uint8_t node_type;
  uint8_t status;
  uint16_t detail_status;
  std::string reserved_info;
  std::string message;
};

struct ModuleStatus {
  maf_std::Header header;
  uint64_t timestamp_us;
  ModuleType module_type;
  uint8_t status;
  uint8_t detail_status;
  double latency_ms;
  std::string message;

  enum : uint8_t {
    STOP = 0,
    STARTING = 1,
    RUNNING = 2,
    STOPPING = 3,
    RUNNING_WARNING = 4,
    RUNNING_ERROR = 5,
    NONE = 0,
    RUNNING_UNKNOWN_ERROR = 1,
    RUNNING_OUT_OF_ROUTE = 2,
    RUNNING_OUT_OF_MAP = 3,
    RUNNING_INVALID_LANE_SCENE = 4,
    RUNNING_TAKING_OVER = 5,
    RUNNING_UNEXPECTED_ERROR = 6,
    RUNNING_IMAGE_BLACK = 7,
    RUNNING_IMAGE_HIGH_LATENCY = 8,
    RUNNING_IMAGE_LOST = 9,
    RUNNING_POINT_CLOUD_EMPTY = 10,
    RUNNING_LOCATION_LOW_ACCURACY = 11,
    RUNNING_MISSING_INPUTS = 12,
  };
};

struct ModuleControl {
  maf_std::Header header;
  uint64_t timestamp_us;
  ModuleType module_type;
  uint8_t action;

  enum : uint8_t {
    STOP = 0,
    START = 1,
  };
};

struct RoutingRequest {
  maf_std::Header header;
  int64_t order_id;
  RoutingRequestType type;
  std::string json_path;
  std::vector<maf_geometry::Point> route_ends;
};

struct SystemStatus {
  maf_std::Header header;
  uint32_t available;
  std::vector<NodeStatus> function_module_status_array;
  OverallStatus overall_status;

  enum : uint32_t {
    MODULE_STATUS = 1,
    OVERALL_STATUS = 2,
  };
};

struct CacheBagDetails {
  uint8_t available;
  CacheBagDetailsData cache_bag_details_data;

  enum : uint8_t {
    CACHE_BAG_DETAILS_DATA = 1,
  };
};

struct FleetDiskStatusData {
  uint8_t health_status;
  uint64_t timestamp_us;
  std::vector<SingleDiskStatus> single_disk_list;
  std::string reserved_info;
  std::string message;

  enum : uint8_t {
    NORM = 0,
    WARNING = 1,
    ERROR = 2,
  };
};

struct ExportDetails {
  uint8_t available;
  ExportDetailsData export_details_data;

  enum : uint8_t {
    EXPORT_DETAILS_DATA = 1,
  };
};

struct SingleMessageStatus {
  uint8_t health_status;
  std::string name;
  uint64_t start_timestamp_us;
  uint64_t end_timestamp_us;
  float frame_rate;
  float standard_frame_rate;
  StatisticValue interval_ms;
  StatisticValue header_delay_ms;
  uint32_t current_loss_frame_count;
  uint32_t total_loss_frame_count;
  bool in_order;
  std::string reserved_info;
  std::string message;

  enum : uint8_t {
    NORM = 0,
    WARNING = 1,
    ERROR = 2,
  };
};

struct TimeStatus {
  uint8_t available;
  TimeStatusData time_status_data;

  enum : uint8_t {
    TIME_STATUS_DATA = 1,
  };
};

struct MemoryStatus {
  uint8_t available;
  MemoryStatusData memory_status_data;

  enum : uint8_t {
    MEMORY_STATUS_DATA = 1,
  };
};

struct DiskStatus {
  uint8_t available;
  DiskStatusData disk_status_data;

  enum : uint8_t {
    DISK_STATUS_DATA = 1,
  };
};

struct ProcessStatusData {
  uint8_t health_status;
  std::vector<SingleProcessStatus> process_status_list;
  std::string reserved_info;
  std::string message;

  enum : uint8_t {
    NORM = 0,
    WARNING = 1,
    ERROR = 2,
  };
};

struct CPUStatus {
  uint8_t available;
  CPUStatusData cpu_status_data;

  enum : uint8_t {
    CPU_STATUS_DATA = 1,
  };
};

struct NetStatusData {
  uint8_t health_status;
  std::vector<NetCardStatus> net_card_status_list;
  std::vector<NetAliveStatus> net_alive_list;
  std::string reserved_info;
  std::string message;

  enum : uint8_t {
    NORM = 0,
    WARNING = 1,
    ERROR = 2,
  };
};

struct GPUStatusData {
  uint8_t health_status;
  std::vector<SingleGPUStatus> gpu_status_list;
  std::string reserved_info;
  std::string message;

  enum : uint8_t {
    NORM = 0,
    WARNING = 1,
    ERROR = 2,
  };
};

struct NodesStatus {
  maf_std::Header header;
  std::vector<NodeStatus> nodes_status;
};

struct RoutingResponse {
  maf_std::Header header;
  RoutingRequest request;
  bool success;
  std::vector<MDKDiRoadId> di_roads;
  std::vector<maf_geometry::Point> road_points;
};

struct FleetDiskStatus {
  uint8_t available;
  FleetDiskStatusData fleet_disk_status_data;

  enum : uint8_t {
    FLEET_DISK_STATUS_DATA = 1,
  };
};

struct MessageStatus {
  maf_std::Header header;
  uint64_t timestamp_us;
  std::vector<SingleMessageStatus> message_status_list;
};

struct GPUStatus {
  uint8_t available;
  GPUStatusData gpu_status_data;

  enum : uint8_t {
    GPU_STATUS_DATA = 1,
  };
};

struct ProcessStatus {
  uint8_t available;
  ProcessStatusData process_status_data;

  enum : uint8_t {
    PROCESS_STATUS_DATA = 1,
  };
};

struct NetStatus {
  uint8_t available;
  NetStatusData net_status_data;

  enum : uint8_t {
    NET_STATUS_DATA = 1,
  };
};

struct ManagerStatus {
  maf_std::Header header;
  uint8_t available;
  uint8_t auto_status;
  uint8_t map_engine_status;
  RoutingResponse map_routing_response;
  std::vector<ManagerNotify> notify;
  bool hdmap_available;

  enum : uint8_t {
    INITIALIZING = 0,
    READY = 1,
    ENGAGE_PENDING = 2,
    ENGAGED = 3,
    DISENGAGE_PENDING = 4,
    FAULT = 5,
    NAVIGATE = 0,
    CRUISE = 1,
    OFF_MAP = 2,
    AUTO_STATUS = 1,
    MAP_ENGINE_STATUS = 2,
    MAP_ROUTING_RESPONSE = 4,
    NOTIFY = 8,
    HDMAP_AVAILABLE = 16,
  };
};

struct FleetStatus {
  maf_std::Header header;
  FleetDiskStatus fleet_disk_status;
  ExportDetails export_details;
  CacheBagDetails cache_bag_details;
};

struct IPCStatus {
  maf_std::Header header;
  CPUStatus cpu_status;
  DiskStatus disk_status;
  GPUStatus gpu_status;
  MemoryStatus memory_status;
  NetStatus net_status;
  ProcessStatus process_status;
  TimeStatus time_status;
};

} // namespace maf_framework_status

#pragma once

#include <string>
#include <vector>
#include <map>


namespace maf_driver_behavior {

struct driver_behavior {
  double db_time_stamp;
  uint8_t db_available;
  uint64_t db_error_sta;
  uint64_t db_driver_sta;
  double db_driver_sta_conf;
  uint8_t db_tk_available;
  double db_tk_available_conf;
  uint64_t db_tk_error_sta;
  double db_tk_error_conf;
  uint64_t db_tk_state;
  double db_tk_sta_conf;
  uint64_t db_tk_time;
  uint8_t db_auto_pilot_mode_error_sta;
  uint8_t db_auto_pilot_mode_available;
  uint8_t db_auto_pilot_mode;
  double db_auto_pilot_mode_conf;
  uint8_t db_spd_mode_error_sta;
  uint8_t db_spd_mode_available;
  uint8_t db_spd_mode_pressed;
  uint8_t db_spd_offset_pressed;
  uint8_t db_lane_Change_mode_error_sta;
  uint8_t db_lane_Change_mode_available;
  uint8_t db_lane_Change_mode;
  uint8_t db_overtaking_laneset;
  uint8_t db_driver_lane_change_set;
  std::vector<double> db_reserved_double;
  std::vector<uint64_t> db_reserved_int;
};

} 
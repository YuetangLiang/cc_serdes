#pragma once

#include <map>
#include <string>
#include <vector>

namespace maf_odd_detection {

struct odd_detection {

  __int64_t odd_res_header;
  bool odd_res_available;
  uint8_t odd_res_error;
  bool odd_static_available;
  bool odd_static_environ_res;
  std::string odd_static_environ_exc_cause;
  bool odd_static_vehicle_sta_res;
  std::string odd_static_vehicle_exc_cause;
  std::vector<bool> odd_static_res;
  std::vector<uint8_t> odd_environ_point_type;
  std::vector<double> odd_environ_point_dis;
  bool odd_dms_risk_available;
  bool odd_dms_risk_is_coverd;
  uint8_t odd_dms_risk_in_loop_sts;
  double odd_dms_risk_hand_start_lost_timstp;
  double odd_dms_risk_hand_end_lost_timstp;
  double odd_dms_risk_cons_end_lost_timstp;
  double odd_dms_risk_cons_start_lost_timstp;
  double odd_dms_risk_eye_start_lost_timstp;
  double odd_dms_risk_eye_end_lost_timstp;
  uint8_t odd_dms_risk_status;
  uint8_t odd_dms_risk_recall_warning_level;
  bool odd_risk_cause_available;
  uint8_t odd_risk_cause_res;
  bool odd_extra_available;
  std::string odd_extra_version;
  std::string odd_extra_json;
  std::vector<uint8_t> odd_reserved_int;
  std::vector<double> odd_reserved_dou;
}; // odd detection

} // namespace maf_odd_detection

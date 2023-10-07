#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_gps_imu {

struct MLAImuMeta {
  uint64_t timestamp_us;
  uint64_t seq;
};

struct MLAImuBasic {
  double accx;
  double accy;
  double accz;
  double gyrox;
  double gyroy;
  double gyroz;
  double temperature;
};

struct MLAGnssBasic {
  double lat;
  double lon;
  double alt;
  double vel_e;
  double vel_n;
  double vel_u;
  double azi_track;
  double speed;
  int8_t status;
  int8_t msg_type;
  int16_t sat_num;
  double hdop;
  double rtk_age;
  double sec_in_gps_week;
  uint16_t gps_week;
  double utc_time;
};

struct MLAGnssMeta {
  uint64_t timestamp_us;
  uint64_t seq;
};

struct MLAGnssExtend {
  int16_t sv_num;
  std::vector<int16_t> sv_id;
  std::vector<int16_t> sv_elv;
  std::vector<int16_t> sv_az;
  std::vector<int16_t> sv_cno;
  double std_pe;
  double std_pn;
  double std_pu;
  double std_ve;
  double std_vn;
  double std_vu;
  double dual_antenna_azimuth;
  double pdop;
  double vdop;
  int8_t fix_type;
  double alt_msl;
};

struct MLAImuInfo {
  uint8_t available;
  MLAImuBasic imu_data_basic;

  enum : uint8_t {
    MLA_IMU_BASIC = 1,
  };
};

struct MLAGnssInfo {
  uint8_t available;
  MLAGnssBasic gnss_data_basic;
  MLAGnssExtend gnss_data_extend;

  enum : uint8_t {
    MLA_GNSS_BASIC = 1,
    MLA_GNSS_EXTEND = 2,
  };
};

struct MLAImu {
  maf_std::Header header;
  MLAImuMeta meta;
  MLAImuInfo info;
};

struct MLAGnss {
  maf_std::Header header;
  MLAGnssMeta meta;
  MLAGnssInfo info;
};

} // namespace maf_gps_imu

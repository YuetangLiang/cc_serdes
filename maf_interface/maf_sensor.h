#pragma once
#include "maf_interface/maf_geometry.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_sensor {

struct JointState {
  maf_std::Header header;
  std::vector<std::string> name;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
};

struct LaserScan {
  maf_std::Header header;
  float angle_min;
  float angle_max;
  float angle_increment;
  float time_increment;
  float scan_time;
  float range_min;
  float range_max;
  std::vector<float> ranges;
  std::vector<float> intensities;
};

struct JoyFeedback {
  uint8_t type;
  uint8_t id;
  float intensity;

  enum : uint8_t {
    TYPE_LED = 0,
    TYPE_RUMBLE = 1,
    TYPE_BUZZER = 2,
  };
};

struct BatteryState {
  maf_std::Header header;
  float voltage;
  float current;
  float charge;
  float capacity;
  float design_capacity;
  float percentage;
  uint8_t power_supply_status;
  uint8_t power_supply_health;
  uint8_t power_supply_technology;
  bool present;
  std::vector<float> cell_voltage;
  std::string location;
  std::string serial_number;

  enum : uint8_t {
    POWER_SUPPLY_STATUS_UNKNOWN = 0,
    POWER_SUPPLY_STATUS_CHARGING = 1,
    POWER_SUPPLY_STATUS_DISCHARGING = 2,
    POWER_SUPPLY_STATUS_NOT_CHARGING = 3,
    POWER_SUPPLY_STATUS_FULL = 4,
    POWER_SUPPLY_HEALTH_UNKNOWN = 0,
    POWER_SUPPLY_HEALTH_GOOD = 1,
    POWER_SUPPLY_HEALTH_OVERHEAT = 2,
    POWER_SUPPLY_HEALTH_DEAD = 3,
    POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4,
    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5,
    POWER_SUPPLY_HEALTH_COLD = 6,
    POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7,
    POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8,
    POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0,
    POWER_SUPPLY_TECHNOLOGY_NIMH = 1,
    POWER_SUPPLY_TECHNOLOGY_LION = 2,
    POWER_SUPPLY_TECHNOLOGY_LIPO = 3,
    POWER_SUPPLY_TECHNOLOGY_LIFE = 4,
    POWER_SUPPLY_TECHNOLOGY_NICD = 5,
    POWER_SUPPLY_TECHNOLOGY_LIMN = 6,
  };
};

struct ChannelFloat32 {
  std::string name;
  std::vector<float> values;
};

struct CompressedImage {
  maf_std::Header header;
  std::string format;
  std::vector<uint8_t> data;
};

struct PointField {
  std::string name;
  uint32_t offset;
  uint8_t datatype;
  uint32_t count;

  enum : uint8_t {
    INT8 = 1,
    UINT8 = 2,
    INT16 = 3,
    UINT16 = 4,
    INT32 = 5,
    UINT32 = 6,
    FLOAT32 = 7,
    FLOAT64 = 8,
  };
};

struct RelativeHumidity {
  maf_std::Header header;
  double relative_humidity;
  double variance;
};

struct Range {
  maf_std::Header header;
  uint8_t radiation_type;
  float field_of_view;
  float min_range;
  float max_range;
  float range;

  enum : uint8_t {
    ULTRASOUND = 0,
    INFRARED = 1,
  };
};

struct Illuminance {
  maf_std::Header header;
  double illuminance;
  double variance;
};

struct LaserEcho {
  std::vector<float> echoes;
};

struct MultiDOFJointState {
  maf_std::Header header;
  std::vector<std::string> joint_names;
  std::vector<maf_geometry::Transform> transforms;
  std::vector<maf_geometry::Twist> twist;
  std::vector<maf_geometry::Wrench> wrench;
};

struct Imu {
  maf_std::Header header;
  maf_geometry::Quaternion orientation;
  std::vector<double> orientation_covariance;
  maf_geometry::Vector3 angular_velocity;
  std::vector<double> angular_velocity_covariance;
  maf_geometry::Vector3 linear_acceleration;
  std::vector<double> linear_acceleration_covariance;
};

struct Temperature {
  maf_std::Header header;
  double temperature;
  double variance;
};

struct FluidPressure {
  maf_std::Header header;
  double fluid_pressure;
  double variance;
};

struct MagneticField {
  maf_std::Header header;
  maf_geometry::Vector3 magnetic_field;
  std::vector<double> magnetic_field_covariance;
};

struct TimeReference {
  maf_std::Header header;
  uint64_t time_ref;
  std::string source;
};

struct Image {
  maf_std::Header header;
  uint32_t height;
  uint32_t width;
  std::string encoding;
  uint8_t is_bigendian;
  uint32_t step;
  std::vector<uint8_t> data;
};

struct RegionOfInterest {
  uint32_t x_offset;
  uint32_t y_offset;
  uint32_t height;
  uint32_t width;
  bool do_rectify;
};

struct NavSatStatus {
  int8_t status;
  uint16_t service;

  enum : int8_t {
    STATUS_NO_FIX = -1,
    STATUS_FIX = 0,
    STATUS_SBAS_FIX = 1,
    STATUS_GBAS_FIX = 2,
    SERVICE_GPS = 1,
    SERVICE_GLONASS = 2,
    SERVICE_COMPASS = 4,
    SERVICE_GALILEO = 8,
  };
};

struct Joy {
  maf_std::Header header;
  std::vector<float> axes;
  std::vector<int32_t> buttons;
};

struct MultiEchoLaserScan {
  maf_std::Header header;
  float angle_min;
  float angle_max;
  float angle_increment;
  float time_increment;
  float scan_time;
  float range_min;
  float range_max;
  std::vector<LaserEcho> ranges;
  std::vector<LaserEcho> intensities;
};

struct PointCloud {
  maf_std::Header header;
  std::vector<maf_geometry::Point32> points;
  std::vector<ChannelFloat32> channels;
};

struct JoyFeedbackArray {
  std::vector<JoyFeedback> array;
};

struct CameraInfo {
  maf_std::Header header;
  uint32_t height;
  uint32_t width;
  std::string distortion_model;
  std::vector<double> D;
  std::vector<double> K;
  std::vector<double> R;
  std::vector<double> P;
  uint32_t binning_x;
  uint32_t binning_y;
  RegionOfInterest roi;
};

struct PointCloud2 {
  maf_std::Header header;
  uint32_t height;
  uint32_t width;
  std::vector<PointField> fields;
  bool is_bigendian;
  uint32_t point_step;
  uint32_t row_step;
  std::vector<uint8_t> data;
  bool is_dense;
};

struct NavSatFix {
  maf_std::Header header;
  NavSatStatus status;
  double latitude;
  double longitude;
  double altitude;
  std::vector<double> position_covariance;
  uint8_t position_covariance_type;

  enum : uint8_t {
    COVARIANCE_TYPE_UNKNOWN = 0,
    COVARIANCE_TYPE_APPROXIMATED = 1,
    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
    COVARIANCE_TYPE_KNOWN = 3,
  };
};

} // namespace maf_sensor

#pragma once
#include "maf_interface/maf_geometry.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_dbw_mkz {

struct Gear {
  uint8_t gear;

  enum : uint8_t {
    NONE = 0,
    PARK = 1,
    REVERSE = 2,
    NEUTRAL = 3,
    DRIVE = 4,
    LOW = 5,
  };
};

struct WatchdogCounter {
  uint8_t source;

  enum : uint8_t {
    NONE = 0,
    OTHER_BRAKE = 1,
    OTHER_THROTTLE = 2,
    OTHER_STEERING = 3,
    BRAKE_COUNTER = 4,
    BRAKE_DISABLED = 5,
    BRAKE_COMMAND = 6,
    BRAKE_REPORT = 7,
    THROTTLE_COUNTER = 8,
    THROTTLE_DISABLED = 9,
    THROTTLE_COMMAND = 10,
    THROTTLE_REPORT = 11,
    STEERING_COUNTER = 12,
    STEERING_DISABLED = 13,
    STEERING_COMMAND = 14,
    STEERING_REPORT = 15,
  };
};

struct TirePressureReport {
  maf_std::Header header;
  float front_left;
  float front_right;
  float rear_left;
  float rear_right;
};

struct AmbientLight {
  uint8_t status;

  enum : uint8_t {
    DARK = 0,
    LIGHT = 1,
    TWILIGHT = 2,
    TUNNEL_ON = 3,
    TUNNEL_OFF = 4,
    NO_DATA = 7,
  };
};

struct GearReject {
  uint8_t value;

  enum : uint8_t {
    NONE = 0,
    SHIFT_IN_PROGRESS = 1,
    OVERRIDE = 2,
    ROTARY_LOW = 3,
    ROTARY_PARK = 4,
    VEHICLE = 5,
  };
};

struct HillStartAssist {
  uint8_t status;
  uint8_t mode;

  enum : uint8_t {
    STAT_INACTIVE = 0,
    STAT_FINDING_GRADIENT = 1,
    STAT_ACTIVE_PRESSED = 2,
    STAT_ACTIVE_RELEASED = 3,
    STAT_FAST_RELEASE = 4,
    STAT_SLOW_RELEASE = 5,
    STAT_FAILED = 6,
    STAT_UNDEFINED = 7,
    MODE_OFF = 0,
    MODE_AUTO = 1,
    MODE_MANUAL = 2,
    MODE_UNDEFINED = 3,
  };
};

struct BrakeCmd {
  maf_std::Header header;
  float pedal_cmd;
  uint8_t pedal_cmd_type;
  bool boo_cmd;
  bool enable;
  bool clear;
  bool ignore;
  uint8_t count;

  enum : uint8_t {
    CMD_NONE = 0,
    CMD_PEDAL = 1,
    CMD_PERCENT = 2,
    CMD_TORQUE = 3,
    TORQUE_BOO = 520,
    TORQUE_MAX = 3412,
  };
};

struct WheelPositionReport {
  maf_std::Header header;
  int16_t front_left;
  int16_t front_right;
  int16_t rear_left;
  int16_t rear_right;

  enum : float {
    COUNTS_PER_REV = 125.5,
  };
};

struct TurnSignal {
  uint8_t value;

  enum : uint8_t {
    NONE = 0,
    LEFT = 1,
    RIGHT = 2,
  };
};

struct TwistCmd {
  maf_geometry::Twist twist;
  float accel_limit;
  float decel_limit;
};

struct SteeringReport {
  maf_std::Header header;
  float steering_wheel_angle;
  float steering_wheel_angle_cmd;
  float steering_wheel_torque;
  float speed;
  bool enabled;
  bool override;
  bool timeout;
  bool fault_wdc;
  bool fault_bus1;
  bool fault_bus2;
  bool fault_calibration;
};

struct SurroundReport {
  maf_std::Header header;
  bool cta_left_alert;
  bool cta_right_alert;
  bool cta_left_enabled;
  bool cta_right_enabled;
  bool blis_left_alert;
  bool blis_right_alert;
  bool blis_left_enabled;
  bool blis_right_enabled;
  bool sonar_enabled;
  bool sonar_fault;
  std::vector<float> sonar;

  enum : uint8_t {
    FRONT_LEFT_SIDE = 0,
    FRONT_LEFT_CORNER = 1,
    FRONT_LEFT_CENTER = 2,
    FRONT_RIGHT_CENTER = 3,
    FRONT_RIGHT_CORNER = 4,
    FRONT_RIGHT_SIDE = 5,
    REAR_LEFT_SIDE = 6,
    REAR_LEFT_CORNER = 7,
    REAR_LEFT_CENTER = 8,
    REAR_RIGHT_CENTER = 9,
    REAR_RIGHT_CORNER = 10,
    REAR_RIGHT_SIDE = 11,
  };
};

struct SteeringCmd {
  maf_std::Header header;
  float steering_wheel_angle_cmd;
  float steering_wheel_angle_velocity;
  bool enable;
  bool clear;
  bool ignore;
  bool quiet;
  uint8_t count;
};

struct ThrottleInfoReport {
  maf_std::Header header;
  float throttle_pc;
  float throttle_rate;
  float engine_rpm;
};

struct ThrottleCmd {
  maf_std::Header header;
  float pedal_cmd;
  uint8_t pedal_cmd_type;
  bool enable;
  bool clear;
  bool ignore;
  uint8_t count;

  enum : uint8_t {
    CMD_NONE = 0,
    CMD_PEDAL = 1,
    CMD_PERCENT = 2,
  };
};

struct ParkingBrake {
  uint8_t status;

  enum : uint8_t {
    OFF = 0,
    TRANS = 1,
    ON = 2,
    FAULT = 3,
  };
};

struct FuelLevelReport {
  maf_std::Header header;
  float fuel_level;
};

struct WheelSpeedReport {
  maf_std::Header header;
  float front_left;
  float front_right;
  float rear_left;
  float rear_right;
};

struct Wiper {
  uint8_t status;

  enum : uint8_t {
    OFF = 0,
    AUTO_OFF = 1,
    OFF_MOVING = 2,
    MANUAL_OFF = 3,
    MANUAL_ON = 4,
    MANUAL_LOW = 5,
    MANUAL_HIGH = 6,
    MIST_FLICK = 7,
    WASH = 8,
    AUTO_LOW = 9,
    AUTO_HIGH = 10,
    COURTESYWIPE = 11,
    AUTO_ADJUST = 12,
    RESERVED = 13,
    STALLED = 14,
    NO_DATA = 15,
  };
};

struct BrakeInfoReport {
  maf_std::Header header;
  float brake_torque_request;
  float brake_torque_actual;
  float wheel_torque_actual;
  float accel_over_ground;
  HillStartAssist hsa;
  bool abs_active;
  bool abs_enabled;
  bool stab_active;
  bool stab_enabled;
  bool trac_active;
  bool trac_enabled;
  ParkingBrake parking_brake;
  bool stationary;
};

struct GearReport {
  maf_std::Header header;
  Gear state;
  Gear cmd;
  GearReject reject;
  bool override;
  bool fault_bus;
};

struct Misc1Report {
  maf_std::Header header;
  TurnSignal turn_signal;
  bool high_beam_headlights;
  Wiper wiper;
  AmbientLight ambient_light;
  bool btn_cc_on;
  bool btn_cc_off;
  bool btn_cc_on_off;
  bool btn_cc_res;
  bool btn_cc_cncl;
  bool btn_cc_res_cncl;
  bool btn_cc_set_inc;
  bool btn_cc_set_dec;
  bool btn_cc_gap_inc;
  bool btn_cc_gap_dec;
  bool btn_la_on_off;
  bool btn_ld_ok;
  bool btn_ld_up;
  bool btn_ld_down;
  bool btn_ld_left;
  bool btn_ld_right;
  bool fault_bus;
  bool door_driver;
  bool door_passenger;
  bool door_rear_left;
  bool door_rear_right;
  bool door_hood;
  bool door_trunk;
  bool passenger_detect;
  bool passenger_airbag;
  bool buckle_driver;
  bool buckle_passenger;
};

struct ThrottleReport {
  maf_std::Header header;
  float pedal_input;
  float pedal_cmd;
  float pedal_output;
  bool enabled;
  bool override;
  bool driver;
  bool timeout;
  WatchdogCounter watchdog_counter;
  bool fault_wdc;
  bool fault_ch1;
  bool fault_ch2;
};

struct BrakeReport {
  maf_std::Header header;
  float pedal_input;
  float pedal_cmd;
  float pedal_output;
  float torque_input;
  float torque_cmd;
  float torque_output;
  bool boo_input;
  bool boo_cmd;
  bool boo_output;
  bool enabled;
  bool override;
  bool driver;
  bool timeout;
  WatchdogCounter watchdog_counter;
  bool watchdog_braking;
  bool fault_wdc;
  bool fault_ch1;
  bool fault_ch2;
  bool fault_boo;
};

struct GearCmd {
  Gear cmd;
  bool clear;
};

struct TurnSignalCmd {
  TurnSignal cmd;
};

} // namespace maf_dbw_mkz

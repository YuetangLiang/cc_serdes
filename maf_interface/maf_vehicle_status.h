#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_vehicle_status {

struct TurnSignalType {
  uint8_t value;

  enum : uint8_t {
    NONE = 0,
    LEFT = 1,
    RIGHT = 2,
    EMERGENCY_FLASHER = 3,
  };
};

struct GearType {
  uint8_t value;

  enum : uint8_t {
    NONE = 0,
    PARK = 1,
    REVERSE = 2,
    NEUTRAL = 3,
    DRIVE = 4,
    LOW = 5,
  };
};

struct HeadingYawRate {
  uint64_t timestamp_us;
  double value_rps;
};

struct Vector3d {
  double x;
  double y;
  double z;
};

struct Quaternion {
  double x;
  double y;
  double z;
  double w;
};

struct VehicleStatusMeta {
  uint64_t timestamp_us;
};

struct ThrottleData {
  uint64_t timestamp_us;
  float pedal_input;
  float pedal_command;
  float pedal_output;
};

struct BrakeData {
  uint64_t timestamp_us;
  float pedal_input;
  float pedal_command;
  float pedal_output;
};

struct SteeringWheelData {
  uint64_t timestamp_us;
  double steering_wheel_rad;
  double steering_wheel_torque;
};

struct BrakeInfoData {
  uint64_t timestamp_us;
  float acceleration_on_vehicle_wheel;
  bool stationary;
};

struct SeatbeltType {
  uint8_t value;

  enum : uint8_t {
    DRIVER = 0,
  };
};

struct LocationGeographic {
  uint64_t timestamp_us;
  double latitude_degree;
  double longitude_degree;
  double altitude_meter;
};

struct CruiseVelocity {
  uint64_t timestamp_us;
  double value_mps;
};

struct HeadingVelocity {
  uint64_t timestamp_us;
  double value_mps;
};

struct WheelVelocity4d {
  uint64_t timestamp_us;
  double front_left;
  double front_right;
  double rear_left;
  double rear_right;
};

struct HeadingYawData {
  uint64_t timestamp_us;
  double value_rad;
};

struct DoorType {
  uint8_t value;

  enum : uint8_t {
    ALL = 0,
  };
};

struct VehicleLightData {
  uint64_t timestamp_us;
  TurnSignalType turn_signal;
};

struct GearData {
  uint64_t timestamp_us;
  GearType gear_status;
};

struct AngularVelocity3d {
  uint64_t timestamp_us;
  Vector3d value;
};

struct Throttle {
  uint8_t available;
  ThrottleData throttle_data;

  enum : uint8_t {
    THROTTLE_DATA = 1,
  };
};

struct Brake {
  uint8_t available;
  BrakeData brake_data;

  enum : uint8_t {
    BRAKE_DATA = 1,
  };
};

struct SteeringWheel {
  uint8_t available;
  SteeringWheelData steering_wheel_data;

  enum : uint8_t {
    STEERING_WHEEL_DATA = 1,
  };
};

struct BrakeInfo {
  uint8_t available;
  BrakeInfoData brake_info_data;

  enum : uint8_t {
    BRAKE_INFO_DATA = 1,
  };
};

struct SeatbeltStatusData {
  uint64_t timestamp_us;
  std::vector<SeatbeltType> enabled_seatbelt_devices;
};

struct LocationENU {
  uint64_t timestamp_us;
  double x;
  double y;
  double z;
  Quaternion orientation;
};

struct Velocity3d {
  uint64_t timestamp_us;
  Vector3d value;
};

struct Acceleration3d {
  uint64_t timestamp_us;
  Vector3d value;
};

struct WheelVelocity {
  uint8_t available;
  WheelVelocity4d wheel_velocity4d;

  enum : uint8_t {
    WHEEL_VELOCITY4D = 1,
  };
};

struct HeadingYaw {
  uint8_t available;
  HeadingYawData heading_yaw_data;

  enum : uint8_t {
    HEADING_YAW_DATA = 1,
  };
};

struct DoorStatusData {
  uint64_t timestamp_us;
  std::vector<DoorType> opened_doors;
};

struct VehicleLight {
  uint8_t available;
  VehicleLightData vehicle_light_data;

  enum : uint8_t {
    VEHICLE_LIGHT_DATA = 1,
  };
};

struct Gear {
  uint8_t available;
  GearData gear_data;

  enum : uint8_t {
    GEAR_DATA = 1,
  };
};

struct AngularVelocity {
  uint8_t available;
  HeadingYawRate heading_yaw_rate;
  AngularVelocity3d angular_velocity3d;

  enum : uint8_t {
    HEADING_YAW_RATE = 1,
    ANGULAR_VELOCITY3D = 2,
  };
};

struct SeatbeltStatus {
  uint8_t available;
  SeatbeltStatusData seatbelt_status_data;

  enum : uint8_t {
    SEATBELT_STATUS_DATA = 1,
  };
};

struct Location {
  uint8_t available;
  LocationGeographic location_geographic;
  LocationENU location_enu;

  enum : uint8_t {
    LOCATION_GEOGRAPHIC = 1,
    LOCATION_ENU = 2,
  };
};

struct Velocity {
  uint8_t available;
  HeadingVelocity heading_velocity;
  Velocity3d velocity3d;
  CruiseVelocity cruise_velocity;

  enum : uint8_t {
    HEADING_VELOCITY = 1,
    VELOCITY3D = 2,
    CRUISE_VELOCITY = 4,
  };
};

struct Acceleration {
  uint8_t available;
  Acceleration3d acceleration3d;

  enum : uint8_t {
    ACCELERATION3D = 1,
  };
};

struct DoorStatus {
  uint8_t available;
  DoorStatusData door_status_data;

  enum : uint8_t {
    DOOR_STATUS_DATA = 1,
  };
};

struct VehicleStatus {
  maf_std::Header header;
  VehicleStatusMeta meta;
  Velocity velocity;
  AngularVelocity angular_velocity;
  WheelVelocity wheel_velocity;
  Acceleration acceleration;
  HeadingYaw heading_yaw;
  Location location;
  SteeringWheel steering_wheel;
  Brake brake;
  Throttle throttle;
  Gear gear;
  BrakeInfo brake_info;
  VehicleLight vehicle_light;
  DoorStatus door_status;
  SeatbeltStatus seatbelt_status;
};

} // namespace maf_vehicle_status

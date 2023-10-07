#ifndef MAF_INTERFACE_MAF_ENDPOINT_H
#define MAF_INTERFACE_MAF_ENDPOINT_H

#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_endpoint {

/*！
 * \brief throttle report info struct
 */
struct ThrottleInfoReportData {
  uint64_t timestamp_us;
  /**
   * @brief Vehicle Engine Speed
   * \unit{rpm}
   * \value_min{0}
   * \value_max{4095.94}
   */
  float output_shaft_rpm;
  /**
   * @brief Actually Throttle Pedal Percent
   * \unit{%}
   * \value_min{0}
   * \value_max{99.9}
   */
  float throttle_pedal_percent;
  /**
   * @brief Actually Throttle Pedal Output Rate
   */
  float throttle_pedal_rate;
  /**
   * @brief Actually average speed of car
   */
  float vehicle_speed_average;
  /**
   * @brief Actually mileage of car
   */
  float vehicle_mileage;
  float reserved1;
  float reserved2;

  serdes6(timestamp_us, output_shaft_rpm, throttle_pedal_percent,
          throttle_pedal_rate, vehicle_speed_average, vehicle_mileage)
};

struct VehicleImuReportData {
  uint64_t timestamp_us;
  /**
   * @brief Lateral acceleration
   * \unit{m/s^2}
   * \value_min{-327.68}
   * \value_max{327.68}
   */
  float linear_acceleration_lat;
  /**
   * @brief Longitudinal Acceleration
   * \unit{m/s^2}
   * \value_min{-327.68}
   * \value_max{327.68}
   */
  float linear_acceleration_lon;
  /**
   * @brief Vertical acceleration
   * \unit{m/s^2}
   * \value_min{-327.68}
   * \value_max{327.68}
   */
  float linear_acceleration_vert;
  /**
   * @brief Roll angular velocity
   * \unit{rad/s}
   * \value_min{-6.5536}
   * \value_max{6.5534}
   */
  float angular_velocity_roll;
  /**
   * @brief Pitch angular velocity
   * \unit{rad/s}
   * \value_min{-6.5536}
   * \value_max{6.5534}
   */
  float angular_velocity_pitch;
  /**
   * @brief Yaw angular velocity
   * \unit{rad/s}
   * \value_min{-6.5536}
   * \value_max{6.5534}
   */
  float angular_velocity_yaw;
  /**
   * @brief whether imu's acceleration is valid
   * \unit{none}
   * \value_valid{true}
   * \value_invalid{false}
   */
  bool valid;

  serdes8(timestamp_us, linear_acceleration_lat,
          linear_acceleration_lon, linear_acceleration_vert,
          angular_velocity_roll, angular_velocity_pitch,
          angular_velocity_yaw, valid)
};

/*!
 * \brief Wheel Speed Report Data Info
 */
struct WheelSpeedReportData {
  uint64_t timestamp_us;
  /**
   * @brief Angular Speed Of Front Left Wheel
   * \unit{rad/s}
   * \value_min{-327.68}
   * \value_max{327.68}
   */
  float front_left;
  /**
   * @brief Angular Speed Of Front Right Wheel
   * \unit{rad/s}
   * \value_min{-327.68}
   * \value_max{327.68}
   */
  float front_right;
  /**
   * @brief Angular Speed Of Rear Left Wheel
   * \unit{rad/s}
   * \value_min{-327.68}
   * \value_max{327.68}
   */
  float rear_left;
  /**
   * @brief Angular Speed Of Rear Right Wheel
   * \unit{rad/s}
   * \value_min{-327.68}
   * \value_max{327.68}
   */
  float rear_right;
  /**
   * @brief Speed Of Front Left Wheel
   * \unit{m/s}
   * \value_min{unknown}
   * \value_max{unknown}
   */
  float front_left_mps;
  /**
   * @brief Speed Of Front Right Wheel
   * \unit{m/s}
   * \value_min{unknown}
   * \value_max{unknown}
   */
  float front_right_mps;
  /**
   * @brief Speed Of Rear Left Wheel
   * \unit{m/s}
   * \value_min{unknown}
   * \value_max{unknown}
   */
  float rear_left_mps;
  /**
   * @brief Speed Of Rear Left Wheel
   * \unit{m/s}
   * \value_min{unknown}
   * \value_max{unknown}
   */
  float rear_right_mps;
  /**
   * @brief Whether Wheel's Speed is Valid
   * \unit{none}
   * \value_valid{true}
   * \value_invalid{false}
   */
  bool valid;
  serdes10(timestamp_us, front_left,
           front_right, rear_left,
           rear_right, front_left_mps,
           front_right_mps, rear_left_mps,
           rear_right_mps, valid)
};

/*!
 * \brief Control Command Extra
 */
struct ControlCommandExtra {
  uint8_t available;
  std::string version;
  std::string json;

  enum : uint8_t {
    VERSION = 1,
    JSON = 2,
  };
  serdes3(available, version, json)
};

/*!
 * \brief Control Command Meta
 */
struct ControlCommandMeta {
  uint64_t timestamp_us;
  serdes1(timestamp_us)
};

/*!
 * \brief Brake Info Report Data Info
 */
struct BrakeInfoReportData {
  uint64_t timestamp_us;
  /**
   * @brief Estimated Accleration from Wheel
   * \unit{rad/s}
   * \value_min{}
   * \value_max{}
   */
  float accleration_on_wheel;
  /**
   * @brief Whether Standstill
   * \Value{ON/OFF}
   * \standstill{ON}
   * \unstandstill{OFF}
   */
  bool stationary;
  /**
   * @brief Whether ABS is Active
   * \Value{ON/OFF}
   * \active{ON}
   * \inactive{OFF}
   */
  bool abs_avtive;
  /**
   * @brief Whether ABS is Fault
   * \Value{ON/OFF}
   * \fault{ON}
   * \no fault{OFF}
   */
  bool abs_fault;
  /**
   * @brief Whether TCS is Active
   * \Value{ON/OFF}
   * \active{ON}
   * \inactive{OFF}
   */
  bool tcs_avtive;
  /**
   * @brief Whether MSR is Active
   * \Value{ON/OFF}
   * \active{ON}
   * \inactive{OFF}
   */
  bool msr_avtive;
  uint8_t reserved;

  serdes8(timestamp_us, accleration_on_wheel, stationary, abs_avtive,
          abs_fault, tcs_avtive, msr_avtive, reserved)
};

/*!
 * \brief Remaining Energ Info Report Data
 */
struct EnergyRemainingReportData {
  uint64_t timestamp_us;
  /**
   * @brief Remaining energ(oil/battery) percentage
   * \unit{%}
   * \value_min{1}
   * \value_max{100}
   */
  float level;
  /**
   * @brief Remaining mileage car can walk
   * \unit{km}
   * \value_min{0}
   * \value_max{unknown}
   */
  float remaining_mileage;
  /**
   * @brief Whether Energ's Report is Valid
   * \unit{none}
   * \value_valid{true}
   * \value_invalid{false}
   */
  bool valid;
  serdes4(timestamp_us, level, remaining_mileage, valid)
};

/*!
 * \brief Swc Status Info
 */
struct SwcStatus {
  uint8_t value;

  enum : uint8_t {
    SWC_STATUS_NOT_ACTIVE = 0, //!< not active
    SWC_STATUS_NORMAL = 1,     //!< normal
    SWC_STATUS_ERROR = 2,      //!< error
  };
  serdes1(value)
};

struct HdcState {
  uint8_t value;

  enum : uint8_t {
    NORMAL = 0,
    ENABLED = 1,
    ACTIVED = 2,
    FAILED = 3,
    TEMPORARILY_INHIBITED = 4,
  };
  serdes1(value)
};

/*!
 * \brief Door Type Info
 */
struct DoorType {
  uint8_t value;

  enum : uint8_t {
    DRIVER = 0,
    PASSENGER = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3,
    HOOD = 4,
    TRUNK = 5,
  };
  serdes1(value)
};

struct WrlsChrgDetResult {
  uint8_t value;

  enum : uint8_t {
    WIRELESS_CHARGER_DETECED_RESULT_DEFAULT = 0,
    WIRELESS_CHARGER_DETECED_RESULT_OK = 1,
    WIRELESS_CHARGER_DETECED_RESULT_OFFSET_X_NOT_OK = 2,
    WIRELESS_CHARGER_DETECED_RESULT_OFFSET_Y_NOT_OK = 3,
    WIRELESS_CHARGER_DETECED_RESULT_OFFSET_XY_NOT_OK = 4,
    WIRELESS_CHARGER_DETECED_RESULT_METAL = 5,
    WIRELESS_CHARGER_DETECED_RESULT_LIVING = 6,
    WIRELESS_CHARGER_DETECED_RESULT_METAL_OFFSET_NOT_OK = 7,
    WIRELESS_CHARGER_DETECED_RESULT_NETA_LIVING = 8,
  };
  serdes1(value)
};

struct FrontLightState {
  uint8_t value;

  enum : uint8_t {
    LIGHT_OFF = 0,
    LOW_BEAM_LIGHT = 1,
    HIGH_BEAM_LIGHT = 2,
    OVERTAKING_FLASH = 3,
    LIGHT_AUTO = 4,
  };
  serdes1(value)
};

struct AebBrkJerkResp {
  uint8_t value;

  enum : uint8_t {
    AEB_BRAKE_JERK_NO_REQUEST = 0,
    AEB_BRAKE_JERK_REQUEST_HONORED = 1,
    AEB_BRAKE_JERK_LOST_ARBITRATION = 2,
    AEB_BRAKE_JERK_CONTROL_NOT_ALLOWED_ERROR = 3,
    AEB_BRAKE_JERK_PRE_CONDITION_NOT_SATISFIED = 4,
    AEB_BRAKE_JERK_RESERVED = 5,
  };
  serdes1(value)
};

struct Gear {
  uint8_t value;

  enum : uint8_t {
    NONE = 0,
    PARK = 1,
    REVERSE = 2,
    NEUTRAL = 3,
    DRIVE = 4,
    LOW = 5,
  };
  serdes1(value)
};

struct RwsSysStatus {
  uint8_t value;

  enum : uint8_t {
    STANDARD_MODE = 0,
    COMFORTABLE_MODE = 1,
    SPORT_MODE = 2,
    AD_MODE = 3,
    ADLOCK_MODE = 4,
    RESERVED = 5,
  };
  serdes1(value)
};

struct WiperCommandType {
  uint8_t value;

  enum : uint8_t {
    OFF = 0,
    LOW = 1,
    HIGH = 2,
    AUTO = 3,
    SPRAY = 4,
    RESERVED = 5,
  };
  serdes1(value)
};

struct Point2d {
  double x;
  double y;
};

struct VseSysState {
  uint8_t value;

  enum : uint8_t {
    INAVTIVE = 0,
    ACTIVE = 1,
    FAULT = 2,
    WARMING_UP = 3,
    NOT_READY = 4,
  };
};

struct LeverStatus {
  uint8_t value;

  enum : uint8_t {
    LEVER_STATE_OFF = 0,
    LEVER_STATE_LEFT = 1,
    LEVER_STATE_RIGHT = 2,
    LEVER_STATE_LEFT_INVALID = 3,
    LEVER_STATE_RIGHT_INVALID = 4,
    LEVER_STATE_RESERVED1 = 5,
    LEVER_STATE_RESERVED3 = 6,
  };
  serdes1(value)
};

struct AebBrkJerkReq {
  uint8_t value;

  enum : uint8_t {
    AEB_BRAKE_JERK_NO_REQUEST = 0,
    AEB_BRAKE_JERK_LOW_LEVEL = 1,
    AEB_BRAKE_JERK_MID_LEVEL = 2,
    AEB_BRAKE_JERK_HIGH_LEVEL = 3,
    AEV_BRAKE_JERK_RESERVED = 4,
  };
  serdes1(value)
};

struct WindowType {
  uint8_t value;

  enum : uint8_t {
    LIFT_STOP = 0,
    LIFT_UP = 1,
    LIFT_DOWN = 2,
  };
  serdes1(value)
};

struct AebSysFaultStatus {
  uint8_t value;

  enum : uint8_t {
    AEB_SYS_FALULT_NO_ERROR = 0,
    AEB_SYS_FALULT_PERFORMANCE_DEGRADATION = 1,
    AEB_SYS_FALULT_SYSTEM_TEMPORARY_UNCAILABLE = 2,
    AEB_SYS_FALULT_SERVICE_REQUIRED = 3,
    AEB_SYS_FALULT_RESERVED1 = 4,
    AEB_SYS_FALULT_RESERVED2 = 5,
    AEB_SYS_FALULT_RESERVED3 = 6,
    AEB_SYS_FALULT_RESERVED4 = 7,
  };
  serdes1(value)
};

struct EpbStatus {
  uint8_t value;

  enum : uint8_t {
    RELEASE = 0,
    PULLING = 1,
    PULL = 2,
    RESERVED = 3,
  };
  serdes1(value)
};

struct SteeringType {
  uint8_t value;

  enum : uint8_t {
    STEERING_ANGLE = 1,
    FRONT_WHEEL_ANGLE = 2,
    YAW_RATE = 3,
    TORQUE = 4,
    RESERVED = 5,
  };
  serdes1(value)
};

struct LdwStatus {
  uint8_t value;

  enum : uint8_t {
    LDW_STATUS_OFF = 0,
    LDW_STATUS_STANDBY = 1,
    LDW_STATUS_DISABLED = 2,
    LDW_STATUS_OVERIDE = 3,
    LDW_STATUS_ACTIVE = 4,
    LDW_STATUS_RESERVED1 = 5,
    LDW_STATUS_RESERVED2 = 6,
    LDW_STATUS_RESERVED3 = 7,
  };
  serdes1(value)
};

struct WiperStateType {
  uint8_t value;

  enum : uint8_t {
    OFF = 0,
    AUTO_OFF = 1,
    AUTO_MOVING = 2,
    MANUAL_OFF = 3,
    MANUAL_ON = 4,
    MANUAL_LOW = 5,
    MANUAL_HIGH = 6,
    MIST_FLICK = 7,
    WASH = 8,
    AUTO_LOW = 9,
    AUTO_HIGH = 10,
    COURTESY_WIPE = 11,
    AUTO_ADJUST = 12,
    RESERVED = 13,
    STALLED = 14,
    NONE = 15,
  };
  serdes1(value)
};

struct CtaState {
  uint8_t value;

  enum : uint8_t {
    NO_WARNING = 0,
    WARNING_LEVE_1 = 1,
    WARNING_LEVE_2 = 2,
  };
  serdes1(value)
};

struct ButtonEvent {
  uint8_t value;

  enum : uint8_t {
    BUTTON1 = 0,
    BUTTON2 = 1,
    BUTTON3 = 2,
    BUTTON4 = 3,
    BUTTON5 = 4,
    BUTTON6 = 5,
    BUTTON7 = 6,
    BUTTON8 = 7,
    BUTTON9 = 8,
    BUTTON10 = 9,
  };
  serdes1(value)
};

struct WorkType {
  uint8_t value;

  enum : uint8_t {
    MANUALPILOT = 0,
    AUTOPILOT_PILOT = 1,
    AUTOPILOT_APA = 2,
    AUTOPILOT_ACC = 3,
    AUTOPILOT_RESERVED1 = 4,
    AUTOPILOT_RESERVED2 = 5,
    AUTOPILOT_RESERVED3 = 6,
    AUTOPILOT_RESERVED4 = 7,
  };
  serdes1(value)
};

struct AebSysStatus {
  uint8_t value;

  enum : uint8_t {
    OFF = 0,
    STANDBY = 1,
    DISABLE = 2,
    OVERRIDE = 3,
    ACTIVED = 4,
    RESERVED1 = 5,
    RESERVED2 = 6,
    RESERVED3 = 7,
  };
};

struct RWhSteerModType {
  uint8_t value;

  enum : uint8_t {
    NO_REQUEST = 0,
    CONTROL_REQUEST = 1,
    LOCK_REQUEST = 2,
    RESERVED1 = 3,
    RESERVED2 = 4,
    RESERVED3 = 5,
  };
  serdes1(value)
};

struct TurnSignal {
  uint8_t value;

  enum : uint8_t {
    NONE = 0,
    LEFT = 1,
    RIGHT = 2,
    EMERGENCY_FLASHER = 3,
  };
  serdes1(value)
};

struct WireControlType {
  uint8_t wire_control_type;

  enum : uint8_t {
    SYETEM_REQ_NONE = 0,
    SYSTEM_REQ_PILOT = 1,
    SYSTEM_REQ_APA = 2,
    SYSTEM_REQ_ACC = 3,
    SYSTEM_REQ_AVP = 4,
    SYSTEM_REQ_LVP = 5,
    SYSTEM_REQ_MRC = 6,
    RESERVED = 7,
  };
  serdes1(wire_control_type)
};

struct HbaStatus {
  uint8_t value;

  enum : uint8_t {
    AEB_HBA_NO_REQUEST = 0,
    AEB_HBA_REQUEST_HONORED = 1,
    AEB_HBA_REQUEST_LOST_ARBITRATION = 2,
    AEB_HBA_REQUEST_CONTROL_NOT_ALLOWED_ERROR = 3,
    AEB_HBA_REQUEST_PRE_CONDITION_NOT_SATISFIED = 4,
    AEB_HBA_REQUEST_RESERVED1 = 5,
    AEB_HBA_REQUEST_RESERVED2 = 6,
    AEB_HBA_REQUEST_RESERVED3 = 7,
  };
  serdes1(value)
};

struct DlpStatus {
  uint8_t value;

  enum : uint8_t {
    NO_WARNING = 0,
    WARNING_LEVEL1 = 1,
    WARNING_LEVEL2 = 2,
    RESERVED = 3,
  };
  serdes1(value)
};

struct BrakeType {
  uint8_t value;

  enum : uint8_t {
    PEDAL = 1,
    DECELERATION = 2,
    MASTER_CYLINDERPRESSURE = 3,
    WHEEL_CYLINDERPRESSURE = 4,
    TORQUE = 5,
    RESERVED = 6,
  };
  serdes1(value)
};

struct HumanProtectedDeviceType {
  uint8_t value;

  enum : uint8_t {
    DRIVER_SEATBELT = 0,
    PASSENGER_SEATBELT = 1,
    PASSENGER_DETECT = 2,
    PASSENGER_AIRBAG = 3,
  };
  serdes1(value)
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
  serdes1(value)
};

struct AutoHoldStatus {
  uint8_t value;

  enum : uint8_t {
    SYS_OFF = 0,
    SYS_INTERVENTION = 1,
    SYS_STANDBY = 2,
    SYS_ERR = 3,
    RESERVED = 4,
  };
  serdes1(value)
};

struct LdpStatus {
  uint8_t value;

  enum : uint8_t {
    LDP_STATUS_OFF = 0,
    LDP_STATUS_STANDBY = 1,
    LDP_STATUS_ACTIVE = 2,
    LDP_STATUS_RESERVED = 3,
  };
  serdes1(value)
};

struct AebDclReqSts {
  uint8_t value;

  enum : uint8_t {
    AEB_DLC_STATE_NO_REQUEST = 0,
    AEB_DLC_STATE_DECELERATION = 1,
    AEB_DLC_STATE_RESERVED = 2,
  };
  serdes1(value)
};

struct ThrottleType {
  uint8_t value;

  enum : uint8_t {
    PEDAL = 0,
    THROTTLE = 1,
    TORQUE = 2,
    ACCELERATION = 3,
    RESERVED = 4,
  };
  serdes1(value)
};

struct AebPrflResp {
  uint8_t value;

  enum : uint8_t {
    AEB_PRFL_NO_REQUEST = 0,
    AEB_PRFL_REQUEST_HONORED = 1,
    AEB_PRFL_REQUEST_LOST_ARBITRATION = 2,
    AEB_PRFL_REQUEST_CONTROL_NOT_ALLOWED_ERROR = 3,
    AEB_PRFL_REQUEST_PRE_CONDITION_NOT_SATISFIED = 4,
    AEB_PRFL_REQUEST_RESERVED1 = 5,
    AEB_PRFL_REQUEST_RESERVED2 = 6,
    AEB_PRFL_REQUEST_RESERVED3 = 7,
  };
  serdes1(value)
};

struct RainSensorLevel {
  uint8_t value;

  enum : uint8_t {
    RAIN_SENSOR_OFF = 0,
    RARIN_SENSOR_LEVEL_LOW = 1,
    RARIN_SENSOR_LEVEL_MID = 2,
    RARIN_SENSOR_LEVEL_HIGH = 3,
    RARIN_SENSOR_LEVEL_RESERVED = 4,
  };
  serdes1(value)
};

struct AebDclResp {
  uint8_t value;

  enum : uint8_t {
    AEB_DCL_NO_REQUEST = 0,
    AEB_DCL_REQUEST_HONORED = 1,
    AEB_DCL_REQUEST_LOST_ARBITRATION = 2,
    AEB_DCL_REQUEST_CONTROL_NOT_ALLOWED_ERROR = 3,
    AEB_DCL_REQUEST_PRE_CONDITION_NOT_SATISFIED = 4,
    AEB_DCL_REQUEST_RESERVED1 = 5,
    AEB_DCL_REQUEST_RESERVED2 = 6,
    AEB_DCL_REQUEST_RESERVED3 = 7,
  };
  serdes1(value)
};

struct FcwStatus {
  uint8_t value;

  enum : uint8_t {
    OFF = 0,
    STANDBY = 1,
    DISABLE = 2,
    OVERRIDE = 3,
    ACTIVED = 4,
    RESERVED1 = 5,
    RESERVED2 = 6,
    RESERVED3 = 7,
  };
  serdes1(value)
};

struct WindowState {
  uint8_t value;

  enum : uint8_t {
    WINDOW_CLOSED = 0,
    WINDOW_OPENED = 1,
    WINDOW_MIDDLE = 2,
  };
  serdes1(value)
};

struct AebHydBrkReq {
  uint8_t value;

  enum : uint8_t {
    AEB_HYD_BRAKE_NO_REQUEST = 0,
    AEB_HYD_BRAKE_LOW_LEVEL = 1,
    AEB_HYD_BRAKE_MID_LEVEL = 2,
    AEB_HYD_BRAKE_HIGH_LEVEL = 3,
    AEB_HYD_BRAKE_RESERVED1 = 4,
    AEB_HYD_BRAKE_RESERVED2 = 5,
    AEB_HYD_BRAKE_RESERVED3 = 6,
    AEB_HYD_BRAKE_RESERVED4 = 7,
  };
  serdes1(value)
};

struct AccStandstillType {
  uint8_t value;

  enum : uint8_t {
    NO_REQUEST = 0,
    REQUEST = 1,
    REMIND_DRIVE_GO = 2,
    RESERVED = 3,
  };
  serdes1(value)
};

struct PowerModeType {
  uint8_t value;

  enum : uint8_t {
    VEHILCE_POWER_MODE_OFF = 0,
    VEHILCE_POWER_MODE_NORMAL = 1,
    VEHILCE_POWER_MODE_AUTODRIVE = 2,
    VEHILCE_POWER_MODE_RESERVED1 = 3,
  };
  serdes1(value)
};

struct WrlsChrgWifiSts {
  uint8_t value;

  enum : uint8_t {
    WIRELESS_CHARGER_WIFI_DEFAULT = 0,
    WIRELESS_CHARGER_WIFI_ON = 1,
    WIRELESS_CHARGER_WIFI_OFF = 2,
    WIRELESS_CHARGER_WIFI_CONNECTED = 3,
    WIRELESS_CHARGER_WIFI_FAILURE = 4,
  };
  serdes1(value)
};

struct BrakeControlType {
  uint8_t value;

  enum : uint8_t {
    NORMAL = 1,
    EMERGENT = 2,
    RESERVED1 = 3,
    RESERVED2 = 4,
  };
  serdes1(value)
};

struct VehicleAlarmReportData {
  uint64_t timestamp_us;
  /**
   * @brief 安全带锁紧报警
   * \unit{NONE}
   * \value{ON, OFF}
   */
  bool seat_belt_alarm;
  /**
   * @brief Whether seatbelt is tightening
   * \unit{NONE}
   * \value{ON, OFF}
   */
  bool seat_belt_tightening;
  /**
   * @brief 方向盘震动警报
   * \unit{NONE}
   * \value{ON, OFF}
   */
  bool steering_wheel_alarm;
  /**
   * @brief 车载蜂鸣器警报
   * \unit{NONE}
   * \value{ON, OFF}
   */
  bool buzzer_alarm;
  /**
   * @brief 报警状态是否可用
   * \unit{NONE}
   * \value{true, false}
   */
  bool valid;
  serdes6(timestamp_us, seat_belt_alarm, seat_belt_tightening,
          steering_wheel_alarm, buzzer_alarm, valid)
};

/*!
 * \brief Tire Pressure Report Data Info
 */
struct TirePressureReportData {
  uint64_t timestamp_us;
  /**
   * @brief Tire Pressusre of Front Left Wheel
   * \unit{kPa}
   * \value_min{0}
   * \value_max{65535}
   */
  float front_left;
  /**
   * @brief Tire Pressusre of Front Right Wheel
   * \unit{kPa}
   * \value_min{0}
   * \value_max{65535}
   */
  float front_right;
  /**
   * @brief Tire Pressusre of Rear Left Wheel
   * \unit{kPa}
   * \value_min{0}
   * \value_max{65535}
   */
  float rear_left;
  /**
   * @brief Tire Pressusre of Rear Right Wheel
   * \unit{kPa}
   * \value_min{0}
   * \value_max{65535}
   */
  float rear_right;
  bool valid;
  serdes6(timestamp_us, front_left, front_right, 
          rear_left, rear_right, valid)
};

struct StateReportMeta {
  uint64_t timestamp_us;
  serdes1(timestamp_us)
};

/*!
 * \brief Wheel Position Report Data Info
 */
struct WheelPositionReportData {
  uint64_t timestamp_us;
  /**
   * @brief wheel speed pulse of Front Left, Forward increment And Decrease
   * Backwards \value_min{-32767} \value_max{32768}
   */
  int16_t front_left;
  /**
   * @brief wheel speed pulse of Front Right, Forward increment And Decrease
   * Backwards \value_min{-32767} \value_max{32768}
   */
  int16_t front_right;
  /**
   * @brief wheel speed pulse of Rear Left, Forward increment And Decrease
   * Backwards \value_min{-32767} \value_max{32768}
   */
  int16_t rear_left;
  /**
   * @brief wheel speed pulse of Rear Right, Forward increment And Decrease
   * Backwards \value_min{-32767} \value_max{32768}
   */
  int16_t rear_right;
  bool valid;
  serdes6(timestamp_us, front_left, front_right,
          rear_left, rear_right, valid)
};

struct WheelAngleReportData {
  uint64_t timestamp_us;
  RwsSysStatus rws_state;
  float front_left;
  float front_right;
  float rear_left;
  float rear_right;
  serdes6(timestamp_us, rws_state, front_left,
          front_right, rear_left, rear_right)
};

/*!
 * \brief Gear Report Data
 */
struct GearReportData {
  uint64_t timestamp_us;
  WorkType driver_work_type;
  Gear command;       //!< Gear From Command
  Gear current_state; //!< Current Gear
  GearReject reject;  //!< GearReject Reason
  bool valid;
  serdes6(timestamp_us, driver_work_type, command, current_state, reject, valid)
};

struct ThrottleInfoReport {
  uint8_t available;
  ThrottleInfoReportData throttle_info_report_data;

  enum : uint8_t {
    THROTTLE_INFO_REPORT_DATA = 1,
  };
  serdes2(available, throttle_info_report_data)
};

/*!
 * \brief Throttle Command Data
 */
struct ThrottleCommandData {
  uint64_t timestamp_us;
  bool throttle_enable;
  ThrottleType throttle_type;
  float throttle_command;

  serdes4(timestamp_us, throttle_enable, throttle_type, throttle_command)
};

struct VehicleImuReport {
  uint8_t available;
  VehicleImuReportData vehicle_imu_report_data;

  enum : uint8_t {
    VEHICLE_IMU_REPORT_DATA = 1,
  };
  serdes2(available, vehicle_imu_report_data)
};

struct WheelSpeedReport {
  uint8_t available;
  WheelSpeedReportData wheel_speed_report_data;

  enum : uint8_t {
    WHEEL_SPEED_REPORT_DATA = 1,
  };
  serdes2(available, wheel_speed_report_data)
};

/*!
 * \brief Vehicle Light Report Data
 */
struct VehicleLightReportData {
  uint64_t timestamp_us;
  WorkType driver_work_type;
  TurnSignal turn_signal_type;
  FrontLightState front_light;
  bool width_light;
  bool reversing_light;
  bool front_fog_light;
  bool rear_fog_light;
  LeverStatus lever_state;
  bool valid;
  serdes10(timestamp_us, driver_work_type, 
           turn_signal_type, front_light,
           width_light, reversing_light,
           front_fog_light, rear_fog_light,
           lever_state, valid)
};

struct WindowReportData {
  uint64_t timestamp_us;
  bool window_lock;
  WindowState window_state_left_front;
  WindowState window_state_right_front;
  WindowState window_state_left_rear;
  WindowState window_state_right_rear;
  WindowState window_state_top;
  bool valid;
  serdes8(timestamp_us, window_lock, window_state_left_front,
          window_state_right_front, window_state_left_rear, window_state_right_rear,
          window_state_top, valid)
};

/*!
 * \brief Steering Report Data Info
 */
struct SteeringReportData {
  uint64_t timestamp_us;
  /**
   * @brief Steering Work Mode
   * \unit{NONE}
   * \value_min{0}
   * \value_max{7}
   * MANUALPILOT = 0
   * AUTOPILOT_PILOT = 1
   * AUTOPILOT_APA = 2
   * AUTOPILOT_ACC = 3
   * AUTOPILOT_RESERVED1 = 4
   * AUTOPILOT_RESERVED2 = 5
   * AUTOPILOT_RESERVED3 = 6
   * AUTOPILOT_RESERVED4 = 7
   */
  WorkType driver_work_type;
  /**
   * @brief Steering Work Mode
   * \unit{NONE}
   * \value:
   * * STEERING_ANGLE：方向盘转角
   * * FRONT_WHEEL_ANGLE：前轮转角
   * * YAW_RATE：航向角
   * * TORQUE：转向管柱力矩
   * * RESERVED：保留字段
   */
  SteeringType steering_type;
  /**
   * @brief 程序输入目标值
   * \unit{NONE}
   */
  float steering_cmd;
  /**
   * @brief 转向反馈值
   * \unit{NONE}
   */
  float steering_report;
  /**
   * @brief 方向盘转向角反馈值
   * \unit{NONE}
   */
  float steering_wheel_angle_report;
  /**
   * @brief 方向盘转向力矩反馈值
   * \unit{NONE}
   */
  float steering_wheel_torque_report;
  /**
   * @brief 跟随速度
   * \unit{NONE}
   */
  float steering_follow_rate;
  /**
   * @brief RWS工作模式反馈
   * \unit{NONE}
   * \value:
   * * NO_REQUEST
   * * CONTROL_REQUEST
   * * LOCK_REQUEST
   * * RESERVED1
   * * RESERVED2
   * * RESERVED3
   */
  RWhSteerModType rws_mode_report;
  /**
   * @brief RWS当前角度
   * \unit{NONE}
   */
  float rws_angle;
  /**
   * @brief 接管状态
   * \unit{NONE}
   */
  bool override;
  /**
   * @brief steering可用状态
   * \unit{NONE}
   */
  bool valid;

  serdes12(timestamp_us, driver_work_type, steering_type, steering_cmd,
           steering_report, steering_wheel_angle_report, steering_wheel_torque_report, steering_follow_rate,
           rws_mode_report, rws_angle, override, valid)
};

struct HumanProtectedDeviceReportData {
  uint64_t timestamp_us;
  std::vector<HumanProtectedDeviceType> enabled_human_protected_device_array;
  bool valid;
  serdes3(timestamp_us, enabled_human_protected_device_array, valid)
};

struct BrakeInfoReport {
  uint8_t available;
  BrakeInfoReportData brake_info_report_data;

  enum : uint8_t {
    BRAKE_INFO_REPORT_DATA = 1,
  };
  serdes2(available, brake_info_report_data)
};

/*!
 * \brief Steering Command Data
 */
struct SteeringCommandData {
  uint64_t timestamp_us;
  /**
   * @brief Steering Enable
   * \unit{NONE}
   * \value:
   * * true：开启转向线控
   * * false：关闭转向线控
   */
  bool steering_enable;
  /**
   * @brief Steering Type
   * \unit{NONE}
   * \value:
   * * STEERING_ANGLE：方向盘转角
   * * FRONT_WHEEL_ANGLE：前轮转角
   * * YAW_RATE：航向角
   * * TORQUE：转向管柱力矩
   * * RESERVED：保留字段
   */
  SteeringType steering_type;
  /**
   * @brief Steering Command
   * \unit{NONE}
   * \value: 不同指令类型对应相应的单位和分辨率 float
   */
  float steering_command;
  /**
   * @brief 跟随速率
   * \unit{NONE}
   * \value: 目标值跟踪速率（转速) float
   */
  float steering_rate;
  /**
   * @brief 后轮转向模式
   * \unit{NONE}
   * \value:
   * * NO_REQUEST
   * * CONTROL_REQUEST
   * * LOCK_REQUEST
   * * RESERVED1
   * * RESERVED2
   * * RESERVED3
   */
  RWhSteerModType rwheel_steer_mode;
  /**
   * @brief 后轮转向命令
   * \unit{NONE}
   * \value: 目标值 float
   */
  float rwheel_command;

  serdes7(timestamp_us, steering_enable, steering_type,
          steering_command, steering_rate, rwheel_steer_mode, 
          rwheel_command)
};

/*!
 * \brief Door Report Data
 */
struct DoorReportData {
  uint64_t timestamp_us;
  /*!
   * \vec_max_size{6}
   */
  std::vector<DoorType> opened_door_array;
  bool valid;
  serdes3(timestamp_us, opened_door_array, valid)
};

/*!
 * \brief Brake Command Data
 */
struct BrakeCommandData {
  uint64_t timestamp_us;
  /**
   * @brief brake_enable
   * \unit{NONE}
   * \value:
   * * true:开启制动线控
   * * false:关闭制动线控
   */
  bool brake_enable;
  /**
   * @brief 后轮转向模式
   * \unit{NONE}
   * \value:
   * * PEDAL: 制动踏板开度
   * * DECELERATION:减速度
   * * MASTER_CYLINDERPRESSURE：制动主缸压力
   * * WHEEL_CYLINDERPRESSURE：轮缸压力
   * * TORQUE：制动力矩
   * * RESERVED: 保留字段
   */
  BrakeType brake_type;
  float brake_command;
  /**
   * @brief 刹车控制模式
   * \unit{NONE}
   * \value:
   * * NORMAL = 1
   * * EMERGENT = 2
   * * RESVERED = 3
   * * RESVERED = 4
   */
  BrakeControlType brake_control_type;
  /**
   * @brief 驻车指令
   * \unit{NONE}
   * \value:
   * * NO_REQUEST = 0
   * * REQUEST = 1
   * * REMIND_DRIVE_GO = 2
   * * RESERVED = 3
   */
  AccStandstillType brake_acc_standstill;
  /**
   * @brief AccGo
   * \unit{NONE}
   * \value:
   * * REQUEST = 1
   * * NO_REQUEST = 0
   */
  bool brake_acc_go;
  /**
   * @brief EPB使能
   * \unit{NONE}
   * \value{true, false}:
   */
  bool epb_enable;
  /**
   * @brief EPB请求
   * \unit{NONE}
   * \value{true, false}:
   */
  bool epb_command;
  /**
   * @brief AutoHold
   * \unit{NONE}
   * \value{true, false}:
   */
  bool auto_hold_command;

  serdes10(timestamp_us, brake_enable,
           brake_type, brake_command,
           brake_control_type, brake_acc_standstill,
           brake_acc_go, epb_enable, 
           epb_command, auto_hold_command)
};

struct AebStatusReportData {
  uint64_t timestamp_us;
  /**
   * @brief AEB状态信息
   * \unit{NONE}
   * \value:
   * * OFF = 0
   * * STANDBY = 1
   * * DISABLE = 2
   * * OVERRIDE = 3
   * * AVTIVED = 4
   * * RESERVED1
   * * RESERVED2
   * * RESERVED3
   */
  AebSysStatus aeb_system_status;
  /**
   * @brief AEB系统错误信息
   * \unit{NONE}
   * \value:
   * * uint8 AEB_SYS_FALULT_NO_ERROR = 0
   * * uint8 AEB_SYS_FALULT_PERFORMANCE_DEGRADATION = 1
   * * uint8 AEB_SYS_FALULT_SYSTEM_TEMPORARY_UNCAILABLE = 2
   * * uint8 AEB_SYS_FALULT_SERVICE_REQUIRED = 3
   * * uint8 AEB_SYS_FALULT_RESERVED1 = 4
   * * uint8 AEB_SYS_FALULT_RESERVED2 = 5
   * * uint8 AEB_SYS_FALULT_RESERVED3 = 6
   * * uint8 AEB_SYS_FALULT_RESERVED4 = 7
   */
  AebSysFaultStatus aeb_system_fault_status;
  /**
   * @brief awb状态反馈
   * \unit{NONE}
   * \value:
   * * uint8 AEB_BRAKE_JERK_NO_REQUEST = 0
   * * uint8 AEB_BRAKE_JERK_REQUEST_HONORED = 1
   * * uint8 AEB_BRAKE_JERK_LOST_ARBITRATION = 2
   * * uint8 AEB_BRAKE_JERK_CONTROL_NOT_ALLOWED_ERROR = 3
   * * uint8 AEB_BRAKE_JERK_PRE_CONDITION_NOT_SATISFIED = 4
   * * uint8 AEB_BRAKE_JERK_RESERVED = 5
   */
  AebBrkJerkResp awb_status;
  /**
   * @brief awb状态反馈
   * \unit{NONE}
   * \value:
   * * uint8 AEB_DCL_NO_REQUEST = 0
   * * uint8 AEB_DCL_REQUEST_HONORED = 1
   * * uint8 AEB_DCL_REQUEST_LOST_ARBITRATION = 2
   * * uint8 AEB_DCL_REQUEST_CONTROL_NOT_ALLOWED_ERROR = 3
   * * uint8 AEB_DCL_REQUEST_PRE_CONDITION_NOT_SATISFIED = 4
   * * uint8 AEB_DCL_REQUEST_RESERVED1 = 5
   * * uint8 AEB_DCL_REQUEST_RESERVED2 = 6
   * * uint8 AEB_DCL_REQUEST_RESERVED3 = 7
   */
  AebDclResp dcl_status;
  /**
   * @brief awb状态反馈
   * \unit{NONE}
   * \value:
   * * uint8 AEB_HBA_NO_REQUEST = 0
   * * uint8 AEB_HBA_REQUEST_HONORED = 1
   * * uint8 AEB_HBA_REQUEST_LOST_ARBITRATION = 2
   * * uint8 AEB_HBA_REQUEST_CONTROL_NOT_ALLOWED_ERROR = 3
   * * uint8 AEB_HBA_REQUEST_PRE_CONDITION_NOT_SATISFIED = 4
   * * uint8 AEB_HBA_REQUEST_RESERVED1 = 5
   * * uint8 AEB_HBA_REQUEST_RESERVED2 = 6
   * * uint8 AEB_HBA_REQUEST_RESERVED3 = 7
   */
  HbaStatus hba_status;
  /**
   * @brief prfl状态反馈
   * \unit{NONE}
   * \value:
   * * uint8 AEB_PRFL_NO_REQUEST = 0
   * * uint8 AEB_PRFL_REQUEST_HONORED = 1
   * * uint8 AEB_PRFL_REQUEST_LOST_ARBITRATION = 2
   * * uint8 AEB_PRFL_REQUEST_CONTROL_NOT_ALLOWED_ERROR = 3
   * * uint8 AEB_PRFL_REQUEST_PRE_CONDITION_NOT_SATISFIED = 4
   * * uint8 AEB_PRFL_REQUEST_RESERVED1 = 5
   * * uint8 AEB_PRFL_REQUEST_RESERVED2 = 6
   * * uint8 AEB_PRFL_REQUEST_RESERVED3 = 7
   */
  AebPrflResp prfl_status;
  /**
   * @brief whether aeb is valid
   * \unit{NONE}
   * \value{true, false}:
   */
  bool aeb_valid;
  /**
   * @brief fcw status
   * \unit{NONE}
   * \value:
   * * uint8 OFF = 0
   * * uint8 STANDBY = 1
   * * uint8 DISABLE = 2
   * * uint8 OVERRIDE = 3
   * * uint8 ACTIVED = 4
   * * uint8 RESERVED1 = 5
   * * uint8 RESERVED2 = 6
   * * uint8 RESERVED3 = 7
   */
  FcwStatus fcw_status;
  /**
   * @brief whether fcw is valid
   * \unit{NONE}
   * \value{true, false}:
   */
  bool fcw_valid;
  /**
   * @brief vse status
   * \unit{NONE}
   * \value:
   * * uint8 INAVTIVE = 0
   * * uint8 ACTIVE = 2
   * * uint8 FAULT = 3
   * * uint8 WARMING_UP = 4
   * * uint8 NOT_READY = 5
   */
  VseSysState vse_state;
  /**
   * @brief whether vse is valid
   * \unit{NONE}
   * \value{true, false}:
   */
  bool vse_valid;
  /**
   * @brief swc status
   * \unit{NONE}
   * \value:
   * * uint8 SWC_STATUS_NOT_ACTIVE = 0
   * * uint8 SWC_STATUS_NORMAL = 1
   * * uint8 SWC_STATUS_ERROR = 2
   */
  SwcStatus swc_status;
  /**
   * @brief whether swc is valid
   * \unit{NONE}
   * \value{true, false}:
   */
  bool swc_valid;
};

struct VehicleLightCommandData {
  uint64_t timestamp_us;
  bool light_enable;
  TurnSignal turn_signal;
  /**
   * @brief 前照灯控制
   * \unit{NONE}
   * \value:
   * * LIGHT_OFF,关闭前照灯
   * * LOW_BEAM_LIGHT，近光灯
   * * HIGH_BEAM_LIGHT，远光灯
   * * OVERTAKING_FLASH，超车闪烁
   * * AUTO，自动大灯
   */
  FrontLightState front_light;
  /**
   * @brief 示宽灯
   * \unit{NONE}
   * \value:
   * * OFF，关灯
   * * ON，开灯
   */
  bool width_light;
  /**
   * @brief 倒车灯
   * \unit{NONE}
   * \value:
   * * OFF，关灯
   * * ON，开灯
   */
  bool reversing_light;
  /**
   * @brief 前雾灯
   * \unit{NONE}
   * \value:
   * * OFF，关灯
   * * ON，开灯
   */
  bool front_fog_light;
  /**
   * @brief 后雾灯
   * \unit{NONE}
   * \value:
   * * OFF，关灯
   * * ON，开灯
   */
  bool rear_fog_light;
  serdes8(timestamp_us, light_enable, 
          turn_signal, front_light,
          width_light, reversing_light,
          front_fog_light, rear_fog_light)
};

struct EnergyRemainingReport {
  uint8_t available;
  EnergyRemainingReportData energy_remaining_report_data;

  enum : uint8_t {
    ENERGY_REMAINING_REPORT_DATA = 1,
  };
  serdes2(available, energy_remaining_report_data)
};

/*!
 * \brief Brake Report Data Info
 */
struct BrakeReportData {
  uint64_t timestamp_us;
  /**
   * @brief 制动工作模式
   * \unit{NONE}
   * \value:
   * * MANUALPILOT = 0
   * * AUTOPILOT_PILOT = 1
   * * AUTOPILOT_APA = 2
   * * AUTOPILOT_ACC = 3
   * * AUTOPILOT_RESERVED1 = 4
   * * AUTOPILOT_RESERVED2 = 5
   * * AUTOPILOT_RESERVED3 = 6
   * * AUTOPILOT_RESERVED4 = 7
   */
  WorkType driver_work_type;
  /**
   * @brief 执行值类型
   * \unit{NONE}
   * \value:
   * * PEDAL = 1
   * * DECELERATION = 2
   * * MASTER_CYLINDERPRESSURE = 3
   * * WHEEL_CYLINDERPRESSURE = 4
   * * TORQUE = 5
   * * RESERVED = 6
   */
  BrakeType brake_type;
  /**
   * @brief Brake Pedal Stroke Input From Driver
   * \unit{%}
   * \value_min{0}
   * \value_max{100}
   */
  float pedal_input;
  /**
   * @brief Brake Pedal Input Stroke  From Command
   * \unit{%}
   * \value_min{0}
   * \value_max{100}
   */
  float brake_command;
  /**
   * @brief Actually Brake Pedal Output Stroke
   * \unit{%}
   * \value_min{0}
   * \value_max{100}
   */
  float brake_output;
  /**
   * @brief 接管状态
   * \unit{%}
   * \value
   * * 1:接管状态
   * * 0:未接管状态
   */
  bool override;
  /**
   * @brief brake可用状态
   * \unit{%}
   * \value
   * * true：可用状态
   * * false：不可用状态
   */
  bool valid;
  serdes8(timestamp_us, driver_work_type, brake_type, pedal_input, 
          brake_command, brake_output, override, valid)
};

struct VehicleAlarmReport {
  uint8_t available;
  VehicleAlarmReportData vehicle_alarm_report_data;

  enum : uint8_t {
    VEHICLE_ALARM_REPORT_DATA = 1,
  };
  serdes2(available, vehicle_alarm_report_data)
};

struct TirePressureReport {
  uint8_t available;
  TirePressureReportData tire_pressure_report_data;

  enum : uint8_t {
    TIRE_PRESSURE_REPORT_DATA = 1,
  };
  serdes2(available, tire_pressure_report_data)
};

struct WirelessChargerReportData {
  uint64_t timestamp_us;
  WrlsChrgWifiSts link_state;
  WrlsChrgDetResult detected_state;
  bool is_offset_x_vaild;
  bool is_offset_y_vaild;
  bool is_yaw_valid;
  int32_t offset_x;
  int32_t offset_y;
  float yaw;
  int32_t rltv_pos_of_charge_board_x;
  int32_t rltv_pos_of_charge_board_y;
  int32_t charge_board_size_x;
  int32_t charge_board_size_y;
  std::string reserved;
};

struct ButtonEventReportData {
  uint64_t timestamp_us;
  std::vector<ButtonEvent> button_event_array;
  bool valid;
  serdes3(timestamp_us, button_event_array, valid)
};

struct WireControlTypeCommandData {
  uint64_t timestamp_us;
  WireControlType wire_type;

  serdes2(timestamp_us, wire_type)
};

struct AuxiliaryReportData {
  uint64_t timestamp_us;
  /**
   * @brief 后视镜折叠状态
   * \unit{NONE}
   * \value{ON, OFF}
   */
  bool rearview_mirror;
  /**
   * @brief 车辆电源状态
   * \unit{%}
   * \value
   * * VEHILCE_POWER_MODE_NORMAL
   * * VEHILCE_POWER_MODE_AUTODRIVE
   * * VEHILCE_POWER_MODE_RESERVED
   */
  PowerModeType vehicle_power_mode;
  /**
   * @brief 车辆电源状态是否可用
   * \unit{%}
   * \value
   * * true：可用状态
   * * false：不可用状态
   */
  bool vehicle_power_valid;
  /**
   * @brief 车辆落锁状态
   * \unit{NONE}
   * \value{ON, OFF}
   */
  bool vehicle_lock;
  bool vehicle_horn;
  /**
   * @brief 车辆电源状态是否可用
   * \unit{%}
   * \value
   * * RAIN_SENSOR_OFF
   * * RARIN_SENSOR_LEVEL_LOW
   * * RARIN_SENSOR_LEVEL_MID
   * * RARIN_SENSOR_LEVEL_HIGH
   * * RARIN_SENSOR_LEVEL_RESERVED
   */
  RainSensorLevel rain_sensor_level;
  /**
   * @brief 表显车速
   * \unit{km/h}
   * \value
   * \value_min{0}
   * \value_max{TBD}
   */
  float vehicle_speed_on_dashboard;
  /**
   * @brief 车外温度
   * \unit{TBD}
   * \value
   * \value_min{TBD}
   * \value_max{TBD}
   */
  float vehicle_outside_temp;
  /**
   * @brief 设备舱温度
   * \unit{TBD}
   * \value
   * \value_min{TBD}
   * \value_max{TBD}
   */
  float vechile_inside_temp;

  serdes10(timestamp_us, rearview_mirror,
           vehicle_power_mode, vehicle_power_valid,
           vehicle_lock, vehicle_horn,
           rain_sensor_level, vehicle_speed_on_dashboard,
           vehicle_outside_temp, vechile_inside_temp)
};

struct VehicleExtraReportData {
  uint64_t timestamp_us;
  CtaState cta_lf;
  CtaState cta_rf;
  CtaState cta_lr;
  CtaState cta_rr;
  bool cta_valid;
  DlpStatus dlp_state;
  bool dlp_valid;
  HdcState hdc_state;
  bool hdc_valid;
  LdpStatus ldp_status_left_lane;
  LdpStatus ldp_status_right_lane;
  bool ldp_valid;
  LdwStatus ldw_status;
  bool ldw_valid;

  serdes15(timestamp_us, cta_lf, cta_rf, cta_lr, cta_rr, 
           cta_valid, dlp_state, dlp_valid, hdc_state, hdc_valid, 
           ldp_status_left_lane, ldp_status_right_lane, ldp_valid, ldw_status, ldw_valid)
};

struct AuxiliaryCommandData {
  uint64_t timestamp_us;
  /**
   * @brief 雨刷控制使能
   * \unit{NONE}
   * \value
   * enable:开启雨刷线控
   * disable:关闭雨刷线控
   */
  bool wiper_enable;
  /**
   * @brief 前雨刷控制
   * \unit{NONE}
   * \value
   * * OFF：不开启雨刷
   * * LOW：雨刷低速运行
   * * HIGH：雨刷快速运行
   * * AUTO：雨刷自动运行
   * * SPRAY：清洗剂喷射
   * * RESERVE：保留字段
   */
  WiperCommandType front_wiper;
  /**
   * @brief 后雨刷控制
   * \unit{NONE}
   * \value
   * * OFF：不开启雨刷
   * * LOW：雨刷低速运行
   * * HIGH：雨刷快速运行
   * * AUTO：雨刷自动运行
   * * SPRAY：清洗剂喷射
   * * RESERVE：保留字段
   */
  WiperCommandType rear_wiper;
  /**
   * @brief 后视镜控制
   * \unit{NONE}
   * \value
   * * OFF，关闭
   * * ON，打开
   */
  bool rearview_mirror;
  /**
   * @brief 车辆电源控制
   * \unit{NONE}
   * \value
   * * VEHILCE_POWER_MODE_OFF = 0
   * * VEHILCE_POWER_MODE_NORMAL = 1
   * * VEHILCE_POWER_MODE_AUTODRIVE = 2
   * * VEHILCE_POWER_MODE_RESERVED1 = 3
   */
  PowerModeType vehicle_power;
  /**
   * @brief 车辆锁控制
   * \unit{NONE}
   * \value
   * * OFF，关闭
   * * ON，打开
   */
  bool vehicle_lock;
  /**
   * @brief 安全带震动警报
   * \unit{NONE}
   * \value
   * * OFF，关闭
   * * ON，打开
   */
  bool seat_belt_alarm;
  /**
   * @brief 安全带收紧警报
   * \unit{NONE}
   * \value
   * * OFF，关闭
   * * ON，打开
   */
  bool seat_belt_tightening;
  /**
   * @brief 方向盘震动报警
   * \unit{NONE}
   * \value
   * * OFF，关闭
   * * ON，打开
   */
  bool steering_wheel_alarm;
  /**
   * @brief 蜂鸣器报警
   * \unit{NONE}
   * \value
   * * OFF，关闭
   * * ON，打开
   */
  bool buzzer_alarm;
  /**
   * @brief 喇叭使能
   * \unit{NONE}
   * \value
   * * OFF，关闭
   * * ON，打开
   */
  bool horn_enable;

  serdes12(timestamp_us, wiper_enable, front_wiper,
           rear_wiper, rearview_mirror, vehicle_power, 
           vehicle_lock, seat_belt_alarm, seat_belt_tightening,
           steering_wheel_alarm,buzzer_alarm, horn_enable)
};

/*!
 * \brief Gear Command Data
 */
struct GearCommandData {
  uint64_t timestamp_us;
  /**
   * @brief 档位使能
   * \unit{NONE}
   * \value
   * * true：开启换档线控
   * * false：关闭换档线控
   */
  bool gear_enable;
  /**
   * @brief 目标档位
   * \unit{NONE}
   * \value
   * * NONE：无
   * * PARK：P档
   * * REVERSE：R档
   * * NEUTRAL：N档
   * * DRIVE：D档
   * * LOW：L档
   * * RESERVED：保留字段
   */
  Gear gear_type;
  serdes3(timestamp_us, gear_enable, gear_type)
};

struct WiperReportData {
  uint64_t timestamp_us;
  /**
   * @brief 雨刷工作模式
   * \unit{NONE}
   * \value
   * * MANUALPILOT = 0
   * * AUTOPILOT_PILOT = 1
   * * AUTOPILOT_APA = 2
   * * AUTOPILOT_ACC = 3
   * * AUTOPILOT_RESERVED1 = 4
   * * AUTOPILOT_RESERVED2 = 5
   * * AUTOPILOT_RESERVED3 = 6
   * * AUTOPILOT_RESERVED4 = 7
   */
  WorkType driver_work_type;
  /**
   * @brief 前雨刷状态
   * \unit{NONE}
   * \value
   * * OFF
   * * AUTO_OFF
   * * AUTO_MOVING
   * * MANUAL_OFF
   * * MANUAL_ON
   * * MANUAL_LOW
   * * MANUAL_HIGH
   * * MIST_FLICK
   * * WASH
   * * AUTO_LOW
   * * AUTO_HIGH
   * * COURTESY_WIPE
   * * AUTO_ADJUST
   * * RESERVED
   * * STALLED
   * * NONE
   */
  WiperStateType front_wiper_state;
  /**
   * @brief 后雨刷状态
   * \unit{NONE}
   * \value
   * * OFF
   * * AUTO_OFF
   * * AUTO_MOVING
   * * MANUAL_OFF
   * * MANUAL_ON
   * * MANUAL_LOW
   * * MANUAL_HIGH
   * * MIST_FLICK
   * * WASH
   * * AUTO_LOW
   * * AUTO_HIGH
   * * COURTESY_WIPE
   * * AUTO_ADJUST
   * * RESERVED
   * * STALLED
   * * NONE
   */
  WiperStateType rear_wiper_state;
  /**
   * @brief 雨刷可用状态
   * \unit{NONE}
   * \value
   * * true：可用状态
   * * false：不可用状态
   */
  bool valid;
  serdes5(timestamp_us, driver_work_type,front_wiper_state,
          rear_wiper_state, valid)
};

struct WindowCommandData {
  uint64_t timestamp_us;
  bool window_lock;
  /**
   * @brief 左前车窗控制
   * \unit{NONE}
   * \value
   * * UP：左前车窗上升
   * * DOWN：左前车窗下降
   * * STOP：停止
   */
  WindowType window_left_front;
  /**
   * @brief 右前车窗控制
   * \unit{NONE}
   * \value
   * * UP：左前车窗上升
   * * DOWN：左前车窗下降
   * * STOP：停止
   */
  WindowType window_right_front;
  /**
   * @brief 左后车窗控制
   * \unit{NONE}
   * \value
   * * UP：左前车窗上升
   * * DOWN：左前车窗下降
   * * STOP：停止
   */
  WindowType window_left_rear;
  /**
   * @brief 右后车窗控制
   * \unit{NONE}
   * \value
   * * UP：左前车窗上升
   * * DOWN：左前车窗下降
   * * STOP：停止
   */
  WindowType window_right_rear;
  /**
   * @brief 天窗控制
   * \unit{NONE}
   * \value
   * * UP：左前车窗上升
   * * DOWN：左前车窗下降
   * * STOP：停止
   */
  WindowType window_top;
  serdes7(timestamp_us, window_lock, window_left_front,
          window_right_front, window_left_rear, window_right_rear,
          window_top)
};

struct EpbReportData {
  uint64_t timestamp_us;
  /**
   * @brief EPB工作模式
   * \unit{NONE}
   * \value
   * * MANUALPILOT = 0
   * * AUTOPILOT_PILOT = 1
   * * AUTOPILOT_APA = 2
   * * AUTOPILOT_ACC = 3
   * * AUTOPILOT_RESERVED1 = 4
   * * AUTOPILOT_RESERVED2 = 5
   * * AUTOPILOT_RESERVED3 = 6
   * * AUTOPILOT_RESERVED4 = 7
   */
  WorkType driver_work_type;
  /**
   * @brief 当前位置
   * \unit{NONE}
   * \value
   * * pull：拉起
   * * pullng：拉起过程中
   * * release：松开
   */
  EpbStatus epb_report;
  /**
   * @brief AutoHold状态
   * \unit{NONE}
   * \value
   * * SYS_OFF
   * * SYS_INTERVENTION
   * * SYS_STANDBY
   * * SYS_ERR
   * * RESERVED
   */
  AutoHoldStatus auto_hold_state;
  /**
   * @brief epb可用状态
   * \unit{NONE}
   * \value
   * * true：可用状态
   * * false：不可用状态
   */
  bool valid;

  serdes5(timestamp_us, driver_work_type, epb_report, auto_hold_state, valid)
};

struct AebCommandData {
  uint64_t timestamp_us;
  /**
   * @brief aeb brake jerk request
   * \unit{NONE}
   * \value
   * * AEB_BRAKE_JERK_NO_REQUEST = 0
   * * AEB_BRAKE_JERK_LOW_LEVEL = 1
   * * AEB_BRAKE_JERK_MID_LEVEL = 2
   * * AEB_BRAKE_JERK_HIGH_LEVEL = 3
   * * AEV_BRAKE_JERK_RESERVED = 4
   */
  AebBrkJerkReq awb_request;
  /**
   * @brief DCL Req
   * \unit{NONE}
   * \value
   * * AEB_DLC_STATE_NO_REQUEST = 0
   * * AEB_DLC_STATE_DECELERATION = 1
   * * AEB_DLC_STATE_RESERVED = 2
   */
  AebDclReqSts dcl_req_state;
  float dcl_req_command;
  /**
   * @brief HBA Request
   * \unit{NONE}
   * \value
   * * uint8 AEB_HYD_BRAKE_NO_REQUEST = 0
   * * uint8 AEB_HYD_BRAKE_LOW_LEVEL = 1
   * * uint8 AEB_HYD_BRAKE_MID_LEVEL = 2
   * * uint8 AEB_HYD_BRAKE_HIGH_LEVEL = 3
   * * uint8 AEB_HYD_BRAKE_RESERVED1 = 4
   * * uint8 AEB_HYD_BRAKE_RESERVED2 = 5
   * * uint8 AEB_HYD_BRAKE_RESERVED3 = 6
   * * uint8 AEB_HYD_BRAKE_RESERVED4 = 7
   */
  AebHydBrkReq hba_request_level;
  /**
   * @brief HBA Request
   * \unit{NONE}
   * \value
   * * AEB_PERFILL_REQUEST_FALSE = 0
   * * AEB_PERFILL_REQUEST_TRUE = 1
   */
  bool prefill_request;

  serdes6(timestamp_us, awb_request, dcl_req_state, 
          dcl_req_command, hba_request_level, prefill_request)
};

struct WheelPositionReport {
  uint8_t available;
  WheelPositionReportData wheel_position_report_data;

  enum : uint8_t {
    WHEEL_POSITION_REPORT_DATA = 1,
  };
  serdes2(available, wheel_position_report_data)
};

struct WheelAngleReport {
  uint8_t available;
  WheelAngleReportData wheel_angle_report_data;

  enum : uint8_t {
    WHEEL_ANGLE_REPORT_DATA = 1,
  };
  serdes2(available, wheel_angle_report_data)
};

/*!
 * \brief Throttle Report Data Info
 */
struct ThrottleReportData {
  uint64_t timestamp_us;
  ThrottleType type;
  /**
   * @brief 油门工作模式
   * \unit{NONE}
   * \value
   * * MANUALPILOT = 0
   * * AUTOPILOT_PILOT = 1
   * * AUTOPILOT_APA = 2
   * * AUTOPILOT_ACC = 3
   * * AUTOPILOT_RESERVED1 = 4
   * * AUTOPILOT_RESERVED2 = 5
   * * AUTOPILOT_RESERVED3 = 6
   * * AUTOPILOT_RESERVED4 = 7
   */
  WorkType driver_work_type;
  /**
   * @brief pedal 物理输入量
   */
  float pedal_input;
  /**
   * @brief 程序输入量
   */
  float command;
  /**
   * @brief 车辆执行量
   */
  float throttle_output;
  /**
   * @brief 接管状态
   * \unit{NONE}
   * \value
   * * 1:接管状态
   * * 0:未接管状态
   */
  bool override;
  /**
   * @brief throttle可用状态
   * \unit{NONE}
   * \value
   * * true：可用状态
   * * false：不可用状态
   */
  bool valid;
  serdes8(timestamp_us, type, driver_work_type, pedal_input, 
          command, throttle_output, override, valid)
};

struct GearReport {
  uint8_t available;
  GearReportData gear_report_data;

  enum : uint8_t {
    GEAR_REPORT_DATA = 1,
  };
  serdes2(available, gear_report_data)
};

struct ThrottleCommand {
  uint8_t available;
  ThrottleCommandData throttle_command_data;

  enum : uint8_t {
    THROTTLE_COMMAND_DATA = 1,
  };

  serdes2(available, throttle_command_data)
};

struct VehicleLightReport {
  uint8_t available;
  VehicleLightReportData vehicle_light_report_data;

  enum : uint8_t {
    VEHICLE_LIGHT_REPORT_DATA = 1,
  };
  serdes2(available, vehicle_light_report_data)
};

struct WindowReport {
  uint8_t available;
  WindowReportData window_report_data;

  enum : uint8_t {
    WINDOW_REPORT_DATA = 1,
  };
  serdes2(available, window_report_data)
};

struct SteeringReport {
  uint8_t available;
  SteeringReportData steering_report_data;

  enum : uint8_t {
    STEERING_REPORT_DATA = 1,
  };
  serdes2(available, steering_report_data)
};

struct HumanProtectedDeviceReport {
  uint8_t available;
  HumanProtectedDeviceReportData human_protected_device_report_data;

  enum : uint8_t {
    HUMAN_PROTECTED_DEVICE_REPORT_DATA = 1,
  };
  serdes2(available, human_protected_device_report_data)
};

struct SteeringCommand {
  uint8_t available;
  SteeringCommandData steering_command_data;

  enum : uint8_t {
    STEERING_COMMAND_DATA = 1,
  };
  serdes2(available, steering_command_data)
};

struct DoorReport {
  uint8_t available;
  DoorReportData door_report_data;

  enum : uint8_t {
    DOOR_REPORT_DATA = 1,
  };
  serdes2(available, door_report_data)
};

struct BrakeCommand {
  uint8_t available;
  BrakeCommandData brake_command_data;

  enum : uint8_t {
    BRAKE_COMMAND_DATA = 1,
  };

  serdes2(available, brake_command_data)
};

struct AebStatusReport {
  uint8_t available;
  AebStatusReportData aeb_status_data;

  enum : uint8_t {
    AEB_STATUS_REPORT_DATA = 1,
  };
};

struct VehicleLightCommand {
  uint8_t available;
  VehicleLightCommandData vehicle_light_command_data;

  enum : uint8_t {
    VEHICLE_LIGHT_COMMAND_DATA = 1,
  };
  serdes2(available, vehicle_light_command_data)
};

struct BrakeReport {
  uint8_t available;
  BrakeReportData brake_report_data;

  enum : uint8_t {
    BRAKE_REPORT_DATA = 1,
  };
  serdes2(available, brake_report_data)
};

struct WirelessChargerReport {
  uint8_t available;
  WirelessChargerReportData wireless_charger_report_data;

  enum : uint8_t {
    WIRELESS_CHARGER_REPORT_DATA = 1,
  };
};

struct ButtonEventReport {
  uint8_t available;
  ButtonEventReportData button_event_report_data;

  enum : uint8_t {
    BUTTON_EVENT_REPORT_DATA = 1,
  };
  serdes2(available, button_event_report_data)
};

struct WireControlTypeCommand {
  uint8_t available;
  WireControlTypeCommandData wire_control_command_data;

  enum : uint8_t {
    WIRE_CONTROL_TYPE_COMMAND_DATA = 1,
  };
  serdes2(available, wire_control_command_data)
};

struct WheelReport {
  maf_std::Header header;
  StateReportMeta meta;
  WheelPositionReport wheel_position_report;
  WheelSpeedReport wheel_speed_report;
  WheelAngleReport wheel_angle_report;
  VehicleImuReport vehicle_imu_report;

  serdes6(header, meta, wheel_position_report, 
          wheel_speed_report, wheel_angle_report, vehicle_imu_report)
};

struct AuxiliaryReport {
  uint8_t available;
  AuxiliaryReportData auxiliary_report_data;

  enum : uint8_t {
    AUXILIARY_REPORT_DATA = 1,
  };
  serdes2(available, auxiliary_report_data)
};

struct VehicleExtraReport {
  uint8_t available;
  VehicleExtraReportData vehicle_extra_report_data;

  enum : uint8_t {
    VEHICLE_EXTRA_REPORT_DATA = 1,
  };
  serdes2(available, vehicle_extra_report_data)
};

struct AuxiliaryCommand {
  uint8_t available;
  AuxiliaryCommandData auxiliary_command_data;

  enum : uint8_t {
    AUXILIARY_COMMAND_DATA = 1,
  };
  serdes2(available, auxiliary_command_data)
};

struct GearCommand {
  uint8_t available;
  GearCommandData gear_command_data;

  enum : uint8_t {
    GEAR_COMMAND_DATA = 1,
  };
  serdes2(available, gear_command_data)
};

struct WiperReport {
  uint8_t available;
  WiperReportData wiper_report_data;

  enum : uint8_t {
    WIPER_REPORT_DATA = 1,
  };
  serdes2(available, wiper_report_data)
};

struct WindowCommand {
  uint8_t available;
  WindowCommandData window_command_data;

  enum : uint8_t {
    WINDOW_COMMAND_DATA = 1,
  };
  serdes2(available, window_command_data)
};

struct EpbReport {
  uint8_t available;
  EpbReportData epb_report_data;

  enum : uint8_t {
    EPB_REPORT_DATA = 1,
  };
  serdes2(available, epb_report_data)
};

struct AebCommand {
  uint8_t available;
  AebCommandData aeb_command_data;

  enum : uint8_t {
    AEB_COMMAND_DATA = 1,
  };

  serdes2(available, aeb_command_data)
};

struct ThrottleReport {
  uint8_t available;
  ThrottleReportData throttle_report_data;

  enum : uint8_t {
    THROTTLE_REPORT_DATA = 1,
  };
  serdes2(available, throttle_report_data)
};

struct ControlCommand {
  maf_std::Header header;  //!< Header
  ControlCommandMeta meta; //!< Control Command Meta
  WireControlTypeCommand wire_type_command;
  BrakeCommand brake_command;       //!< Brake Command
  GearCommand gear_command;         //!< Gear Command
  SteeringCommand steering_command; //!< Steering Command
  ThrottleCommand throttle_command; //!< Throttle Command
  AebCommand aeb_command;
  VehicleLightCommand vehicle_light_command;
  WindowCommand window_command;
  AuxiliaryCommand auxiliary_command;
  ControlCommandExtra extra; //!< Control Command Extra

  serdes12(header, meta, wire_type_command, brake_command, 
           gear_command, steering_command, throttle_command, aeb_command,
           vehicle_light_command, window_command, auxiliary_command, extra)
};

struct ChassisReport {
  maf_std::Header header;
  StateReportMeta meta;
  BrakeReport brake_report;
  BrakeInfoReport brake_info_report;
  ThrottleReport throttle_report;
  ThrottleInfoReport throttle_info_report;
  SteeringReport steering_report;
  GearReport gear_report;
  EpbReport epb_report;
  serdes9(header,meta,brake_report,
          brake_info_report,throttle_report, throttle_info_report, 
          steering_report, gear_report, epb_report)
};

struct WirelessChargeReport {
  maf_std::Header header;
  StateReportMeta meta;
  WirelessChargerReport wireless_charger_report;
};

struct BodyReport {
  maf_std::Header header;
  StateReportMeta meta;
  VehicleLightReport vehicle_light_report;
  ButtonEventReport button_report;
  DoorReport door_report;
  TirePressureReport tire_report;
  WindowReport window_report;
  WiperReport wiper_report;
  EnergyRemainingReport energy_remaining_report;
  VehicleAlarmReport vehicle_alarm_report;
  VehicleExtraReport vehicle_extra_report;
  HumanProtectedDeviceReport human_protected_device_report;
  AuxiliaryReport auxiliary_report;

  serdes13(header, meta, vehicle_light_report, button_report,
           door_report, tire_report, window_report, wiper_report,
           energy_remaining_report, vehicle_alarm_report, vehicle_extra_report, human_protected_device_report,
            auxiliary_report)
};

} // namespace maf_endpoint

#endif

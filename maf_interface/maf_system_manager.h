#ifndef MAF_INTERFACE_MAF_SYSTEM_MANAGER_H
#define MAF_INTERFACE_MAF_SYSTEM_MANAGER_H

#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_system_manager {

/*!
 * \brief Enum of mode switch
 */
struct RunningModeEnum {
  uint8_t value;

  enum : uint8_t {
    HIGHWAY = 0,
    PARKING = 1,
    MANUAL_DRIVING = 2,
    PRODUCTION_LINE = 3,
  };
};

/*!
 * \brief Enum of module control cmd
 */
struct ModuleControlCmdEnum {
  uint8_t value;

  enum : uint8_t {
    PAUSE = 0,
    RESUME = 1,
  };
};

/*!
 * \brief Module control command request
 */
struct ModuleControlCmdRequest {
  maf_std::Header header;
  RunningModeEnum running_mode;
  ModuleControlCmdEnum module_control_cmd; //!< command of pause/resume
};

/*!
 * \brief Module control command response
 */
struct ModuleControlCmdResponse {
  maf_std::Header header;
  uint32_t request_seq; //!< sequence of module_control_cmd_request msg
  bool success; //!< whether the module_control_cmd is successfully executed
};

/**
 * @brief system command from system_manager to modules
 *
 */
struct SystemCmdTypeEnum {
  uint32_t value;
  enum : uint32_t {
    /**
     * @brief product-line calibration
     *
     */
    PLAC_CAMERA_START = 0x00010001,
    PLAC_CAMERA_CANCEL,
    PLAC_CAMERA_STOP,

    /**
     * @brief online calibration
     *
     */
    ACE_CAMERA_DETECT_START = 0x00020001,
    ACE_CAMERA_DETECT_CANCEL,
    ACE_CAMERA_DETECT_STOP,
    ACE_CAMERA_CALIBRATION_START,
    ACE_CAMERA_CALIBRATION_CANCEL,
    ACE_CAMERA_CALIBRATION_STOP,
    ACE_LIDAR_DETECT_START,
    ACE_LIDAR_DETECT_CANCEL,
    ACE_LIDAR_DETECT_STOP,
    ACE_LIDAR_CALIBRATION_START,
    ACE_LIDAR_CALIBRATION_CANCEL,
    ACE_LIDAR_CALIBRATION_STOP,
    ACE_IMU_CALIBRATION_START,
    ACE_IMU_CALIBRATION_CANCEL,
    ACE_IMU_CALIBRATION_STOP,

    /**
     * @brief camera service command
     *
     */
    CAMERA_PLAC_IMAGE_GENERATE_START = 0x00030001,
    CAMERA_PLAC_IMAGE_GENERATE_CANCEL,
    CAMERA_PLAC_IMAGE_GENERATE_STOP,

    /**
     * @brief vehicle service command
     *
     */
    VEHICLE_SERVICE_HIGHWAY_FUNCTION_MODE = 0x00040001,

    /**
     * @brief vision percpetion command
     *
     */
    PERCEPTION_VISION_APA_START = 0x00050001,
    PERCEPTION_VISION_APA_STOP,
    PERCEPTION_VISION_SVP_START,
    PERCEPTION_VISION_SVP_STOP,
    PERCEPTION_VISION_LVP_START,
    PERCEPTION_VISION_LVP_STOP,

    /**
     * @brief fusion command
     *
     */
    FUSION_PARKING_APA_START = 0x00060001,
    FUSION_PARKING_APA_STOP,
    FUSION_PARKING_SVP_START,
    FUSION_PARKING_SVP_STOP,
    FUSION_PARKING_LVP_START,
    FUSION_PARKING_LVP_STOP,

    /**
     * @brief map manager command
     *
     */
    MAP_MANAGER_HIGHWAY_SET_NAVI_PATH = 0x00070001,

    /**
     * @brief worldmodel command
     *
     */
    WORLDMODEL_PARKING_APA_START = 0x00080001,
    WORLDMODEL_PARKING_APA_STOP,
    WORLDMODEL_PARKING_SVP_START,
    WORLDMODEL_PARKING_SVP_STOP,
    WORLDMODEL_PARKING_LVP_START,
    WORLDMODEL_PARKING_LVP_STOP,
    WORLDMODEL_HIGHWAY_FUNCTION_MODE,

    /**
     * @brief planning command
     *
     */
    PLANNING_CHANGE_MODE =
        0x000a0001,            //!< parking planning mode, include APA, SVP. LVP
    PLANNING_RANDOM_PARKIN,    //!< random park
    PLANNING_DESIGNATE_PARKIN, //!< park to the specified space
    PLANNING_PARKOUT,          //!< park out
    PLANNING_STOP,             //!< stop park process
    PLANNING_EXIT,             //!< exit park process, exit parking
    PLANNING_HIGHWAY_FUNCTION_MODE,
    PLANNING_HIGHWAY_NAVI_SETTINGS,
    PLANNING_HIGHWAY_DRIVING_STYLE,
    PLANNING_HIGHWAY_LANE_CHANGE,
    PLANNING_HIGHWAY_START_STOP,

    /**
     * @brief controller command
     *
     */
    CONTROLLER_FUNCTION_MODE = 0x000b0001, //!< function mode

    /**
     * @brief egopose_manager command
     *
     */
    EGOPOSE_MANAGER_FUNCTION_MODE = 0x000c0001, //!< function mode
  };
};

/*!
 * \brief Enum of function mode
 */
struct FunctionModeEnum {
  uint8_t value;

  enum : uint8_t {
    ACC = 0,
    PILOT = 1,
    HNP = 2,
    APA = 3,
    SVP = 4,
    PLAC = 5,
    ACE = 6,
  };
};

/*!
 * \brief navigation settings
 */
struct NaviSettings {
  float navi_max_speed;     //!< max speed of navigation
  float navi_time_distance; //!< time distance of navigation
};

/*!
 * \brief Enum of driving style
 */
struct DrivingStyleEnum {
  uint8_t value;

  enum : uint8_t {
    AGGRESIVE = 0,
    NORMAL = 1,
    CONSERVATIVE = 2,
  };
};

/*!
 * \brief Enum of lane change type
 */
struct LaneChangeTypeEnum {
  uint8_t value;

  enum : uint8_t {
    FORBIDDEN = 0,
    INTERACTIVE = 1,
  };
};

/*!
 * \brief Enum of lane change direction
 */
struct LaneChangeDirectionEnum {
  uint8_t value;

  enum : uint8_t {
    LEFT = 0,
    RIGHT = 1,
  };
};

/*!
 * \brief Lane change cmd
 */
struct LaneChangeCmd {
  LaneChangeTypeEnum type;
  LaneChangeDirectionEnum direction;
};

/*!
 * \brief Enum of start stop command
 */
struct StartStopCmdEnum {
  uint8_t value;

  enum : uint8_t {
    START = 0,
    STOP = 1,
  };
};

/**
 * @brief camera service info
 *
 */
struct SysCameraServiceInfo {
  std::string path; //!< the dir of bmp image saved
};

/**
 * @brief request from system_manager to camera_service
 *
 */
struct SysCameraServiceRequest {
  maf_std::Header header;
  SystemCmdTypeEnum cmd;
  SysCameraServiceInfo info;
};

/**
 * @brief response from camera_service to system_manager
 *
 */
struct SysCameraServiceResponse {
  maf_std::Header header;
  uint32_t request_seq; //!< sequence of request
  bool success; //!< whether the sys_camera_service_request is successfully
                //! executed
};

/**
 * @brief world model info for parking
 *
 */
struct SysWorldModelInfoForParking {
  uint32_t id; //!< parking slot ID
};

/**
 * @brief world model info for highway
 *
 */
struct SysWorldModelInfoForHighway {
  FunctionModeEnum function_mode; //!< function mode
};

/**
 * @brief request from system manager to worldmodel
 *
 */
struct SysWorldModelRequest {
  maf_std::Header header;
  SystemCmdTypeEnum cmd;
  SysWorldModelInfoForParking parking_info;
  SysWorldModelInfoForHighway highway_info;
};

/**
 * @brief response from worldmodel to system_manager
 *
 */
struct SysWorldModelResponse {
  maf_std::Header header;
  uint32_t request_seq; //!< sequence of request
  bool success; //!< whether the sys_worldmodel_request is successfully executed
};

/**
 * @brief vehicle_service info for highway
 *
 */
struct SysVehicleServiceInfoForHighway {
  FunctionModeEnum function_mode; //!< function mode
};

/**
 * @brief request from system manager to vehicle_service
 *
 */
struct SysVehicleServiceRequest {
  maf_std::Header header;
  SystemCmdTypeEnum cmd;
  SysVehicleServiceInfoForHighway highway_info;
};

/**
 * @brief response from vehicle_service to system_manager
 *
 */
struct SysVehicleServiceResponse {
  maf_std::Header header;
  uint32_t request_seq; //!< sequence of request
  bool success; //!< whether the sys_vehicle_service_request is successfully
                //! executed
};

/**
 * @brief direction of parking-out
 *
 */
struct ParkOutDirectionType {
  uint8_t value;

  enum : uint8_t {
    AUTO_SELECT_DIRECTION_TYPE = 0,      //!< random direction
    FRONT_SIDE_LEFT_DIRECTION_TYPE = 1,  //!< front-side left
    FRONT_SIDE_RIGHT_DIRECTION_TYPE = 2, //!< front-side right
    LEFT_SIDE_FRONT_DIRECTION_TYPE = 3,  //!< left-side front
    RIGHT_SIDE_FRONT_DIRECTION_TYPE = 4, //!< right-side front
    FRONT_SIDE_FRONT_DIRECTION_TYPE = 5, //!< front side
    REAR_SIDE_REAR_DIRECTION_TYPE = 6,   //!< rear side
  };
};

/**
 * @brief planning mode for parking
 *
 */
struct SysPlanningModeTypeForParking {
  uint8_t value;

  enum : uint8_t {
    PLANNING_APA = 1,
    PLANNING_LVP = 2,
    PLANNING_SVP = 3,
  };
};

/**
 * @brief parking out info
 *
 */
struct SysPlanningInfoForParking {
  SysPlanningModeTypeForParking mode;
  uint32_t id;          //!< 用户选择的车位 ID
  bool wireless_charge; //!< is this parking slot support wireless charge
  ParkOutDirectionType parking_out_direction;
};

/**
 * @brief planning info for highway
 *
 */
struct SysPlanningInfoForHighway {
  FunctionModeEnum function_mode;  //!< function mode
  NaviSettings navi_settings;      //!< navigation settings
  DrivingStyleEnum driving_style;  //!< driving style
  LaneChangeCmd lane_change_cmd;   //!< lane change command
  StartStopCmdEnum start_stop_cmd; //!< start stop command
};

/**
 * @brief request from system manager to planning
 *
 */
struct SysPlanningRequest {
  maf_std::Header header;
  SystemCmdTypeEnum cmd;
  SysPlanningInfoForParking parking_info;
  SysPlanningInfoForHighway highway_info;
};

/**
 * @brief response from planning to system_manager
 *
 */
struct SysPlanningResponse {
  maf_std::Header header;
  uint32_t request_seq; //!< sequence of request
  bool success; //!< whether the sys_planning_request is successfully executed
};

/**
 * @brief controller info
 *
 */
struct SysControllerInfo {
  FunctionModeEnum function_mode; //!< function mode
};

/**
 * @brief request from system_manager to controller
 *
 */
struct SysControllerRequest {
  maf_std::Header header;
  SystemCmdTypeEnum cmd;
  SysControllerInfo info;
};

/**
 * @brief response from controller to system_manager
 *
 */
struct SysControllerResponse {
  maf_std::Header header;
  uint32_t request_seq; //!< sequence of request
  bool success; //!< whether the sys_controller_request is successfully executed
};

/**
 * @brief egopose_manager info
 *
 */
struct SysEgoposeManagerInfo {
  FunctionModeEnum function_mode; //!< function mode
};

/**
 * @brief request from system_manager to egopose_manager
 *
 */
struct SysEgoposeManagerRequest {
  maf_std::Header header;
  SystemCmdTypeEnum cmd;
  SysEgoposeManagerInfo info;
};

/**
 * @brief response from egopose_manager to system_manager
 *
 */
struct SysEgoposeManagerResponse {
  maf_std::Header header;
  uint32_t request_seq; //!< sequence of request
  bool success; //!< whether the sys_egopose_manager_request is successfully
                //!< executed
};

/**
 * @brief request from system manager to map_manager
 *
 */
struct SysMapManagerRequest {
  maf_std::Header header;
  SystemCmdTypeEnum cmd;
  std::string route; //!< sdmap route, a protobuf string
};

/**
 * @brief response from map_manager to system_manager
 *
 */
struct SysMapManagerResponse {
  maf_std::Header header;
  uint32_t request_seq;       //!< sequence of request
  std::string route_response; //!< route response of map_manager
};

/**
 * @brief plac info
 *
 */
struct SysPlacInfo {
  std::string path; //!< the dir of plac saves calibration file
};

/**
 * @brief request from system manager to plac
 *
 */
struct SysPlacRequest {
  maf_std::Header header;
  SystemCmdTypeEnum cmd_type;
  SysPlacInfo info;
};

/**
 * @brief response from plac to system_manager
 *
 */
struct SysPlacResponse {
  maf_std::Header header;
  uint32_t request_seq; //!< sequence of request
  bool success; //!< whether the sys_plac_request is successfully executed
};

/**
 * @brief ace info
 *
 */
struct SysAceInfo {
  uint32_t id;      //!< camera id
  std::string path; //!< the dir of ace saves calibration file
};

/**
 * @brief system manager to ace request
 *
 */
struct SysAceRequest {
  maf_std::Header header;
  SystemCmdTypeEnum cmd_type;
  SysAceInfo info;
};

/**
 * @brief response from ace to system_manager
 *
 */
struct SysAceResponse {
  maf_std::Header header;
  uint32_t request_seq; //!< sequence of request
  bool success; //!< whether the sys_ace_request is successfully executed
};

/**
 * @brief maf module task info
 *
 */
struct SysModuleTaskInfo {
  uint64_t timestamp_us;
  SystemCmdTypeEnum latest_cmd;
  uint32_t task; //!< inner state machine
  uint32_t status;
  uint32_t info_code; //!< reminder info
  uint32_t resvered1; //!< resvered position 1
  uint32_t resvered2; //!< resvered position 2
};
} // namespace maf_system_manager

#endif
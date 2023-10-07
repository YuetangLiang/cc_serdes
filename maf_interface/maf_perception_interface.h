#ifndef MAF_INTERFACE_MAF_PERCEPTION_INTERFACE_H
#define MAF_INTERFACE_MAF_PERCEPTION_INTERFACE_H

#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_perception_interface {

struct Rect4f {
  float left;
  float top;
  float right;
  float bottom;
  serdes4(left, top, right, bottom)
};

struct Shape2d {
  double height;
  double width;
};

/*!
 * \brief  velocity  in three-dimensional coordinates
 * \unit{m/s}
 */
struct Vector3d {
  double x;
  double y;
  double z;
  serdes3(x,y,z)
};

struct Shape2f {
  float height;
  float width;
};

struct Point3f {
  float x;
  float y;
  float z;
  serdes3(x,y,z)
};

/*!
 * \brief Enum to the camera source
 */
struct CameraSourceEnum {
  uint8_t value;

  enum : uint8_t {
    CAMERA_SOURCE_FRONT_FAR = 0,      //!< front far camera
    CAMERA_SOURCE_FRONT_MID = 1,      //!< front mid camera
    CAMERA_SOURCE_FRONT_WIDE = 2,     //!< front wide camera
    CAMERA_SOURCE_LEFT_FRONT = 3,     //!< left front camera
    CAMERA_SOURCE_LEFT_REAR = 4,      //!< left rear camera
    CAMERA_SOURCE_REAR_MID = 5,       //!< rear mid camera
    CAMERA_SOURCE_REAR_MID_UP = 6,    //!< rear mid up camera
    CAMERA_SOURCE_REAR_MID_DOWN = 7,  //!< rear mid down camera
    CAMERA_SOURCE_RIGHT_FRONT = 8,    //!< right front camera
    CAMERA_SOURCE_RIGHT_REAR = 9,     //!< right rear camera
    CAMERA_SOURCE_FRONT_FISHEYE = 10, //!< front fisheye camera
    CAMERA_SOURCE_LEFT_FISHEYE = 11,  //!< left fisheye camera
    CAMERA_SOURCE_RIGHT_FISHEYE = 12, //!< right fisheye camera
    CAMERA_SOURCE_REAR_FISHEYE = 13,  //!< rear fisheye camera
    CAMERA_SOURCE_FISHEYE = 14,       //!< fisheye camera
  };
  serdes1(value)
};

/*!
 * \brief  point position in three-dimensional coordinates(m)
 * \unit{m}
 */
struct Point3d {
  double x;
  double y;
  double z;
  serdes3(x,y,z)
};

/*!
 * \brief Enum to the camera model
 */
struct CameraModelEnum {
  uint8_t value;

  enum : uint8_t {
    CAMERA_MODEL_UNKNOWN = 0,   //!< Unknown camera model
    CAMERA_MODEL_RAW = 1,       //!< raw images
    CAMERA_MODEL_CYLINDER = 2,  //!< cylinder images
    CAMERA_MODEL_DEDISTORT = 3, //!< undistorted images
    CAMERA_MODEL_PSTITCH = 4,   //!< pstitch images
    CAMERA_MODEL_RESERVED_A = 5,
    CAMERA_MODEL_RESERVED_B = 6,
    CAMERA_MODEL_RESERVED_C = 7,
  };
  serdes1(value)
};

struct Shape3d {
  double height;
  double length;
  double width;
  serdes3(height, length, width)
};

struct Point2d {
  double x;
  double y;
  serdes2(x,y)
};

struct Vector2f {
  float x;
  float y;
  serdes2(x,y)
};

struct Point2f {
  float x;
  float y;
  serdes2(x,y)
};

struct Shape3f {
  float length;
  float height;
  float width;
  serdes3(length, height, width)
};

struct Vector2d {
  double x;
  double y;
  serdes2(x,y)
};

struct Vector3f {
  float x;
  float y;
  float z;
  serdes3(x,y,z)
};

/*!
 * \brief Enum to the detailed subtype of VRU
 */
struct VRUSubTypeEnum {
  uint8_t value;

  enum : uint8_t {
    VRU_SUB_TYPE_UNKNOWN_VRU = 0, //!< Unknown VRU subtype
    VRU_SUB_TYPE_UNKNOWN_PEDESTRIAN =
        1, //!< Known as pedestrian, but the detailed subtype is unknown
    VRU_SUB_TYPE_STAND_PEDESTRIAN = 2,    //!< Stand pedestrian
    VRU_SUB_TYPE_SQUATTER_PEDESTRIAN = 3, //!< Squatter pedestrian
    VRU_SUB_TYPE_LYING_PEDESTRIAN = 4,    //!< Lying pedestrian
    VRU_SUB_TYPE_UNKNOWN_BICYCLIST =
        5, //!< Known as bicyclist, but the detailed subtype is unknown
    VRU_SUB_TYPE_RIDER_MANUAL_BICYCLIST =
        6, //!< Human-powered bicycle with a rider
    VRU_SUB_TYPE_NO_RIDER_MANUAL_BICYCLIST =
        7, //!< Human-powered bicycle without a rider
    VRU_SUB_TYPE_RIDER_ELECTRIC_BICYCLIST =
        8, //!< Electric bicycle with a rider
    VRU_SUB_TYPE_NO_RIDER_ELECTRIC_BICYCLIST =
        9, //!< Electric bicycle without a rider
    VRU_SUB_TYPE_RIDER_MOTOR_BICYCLIST = 10, //!< Motor bicycle without a rider
    VRU_SUB_TYPE_NO_RIDER_MOTOR_BICYCLIST =
        11, //!< Motor bicycle without a rider
    VRU_SUB_TYPE_UNKNOWN_TRICYCLIST =
        12, //!< Known as tricyclsit, but the detailed subtype is unknown
    VRU_SUB_TYPE_RIDER_MANUAL_TRICYCLIST =
        13, //!< Human-powered tricycle with a rider
    VRU_SUB_TYPE_NO_RIDER_MANUAL_TRICYCLIST =
        14, //!< Human-powered tricycle without a rider
    VRU_SUB_TYPE_RIDER_ELECTRIC_TRICYCLIST =
        15, //!< Electric tricycle with a rider
    VRU_SUB_TYPE_NO_RIDER_ELECTRIC_TRICYCLIST =
        16, //!< Electric tricycle without a rider
    VRU_SUB_TYPE_RIDER_MOTOR_TRICYCLIST = 17, //!< Motor tricycle with a rider
    VRU_SUB_TYPE_NO_RIDER_MOTOR_TRICYCLIST =
        18, //!< Motor tricycle with a rider
    VRU_SUB_TYPE_UNKNOWN_TROLLEY =
        19, //!< Known as trolley, but the detailed subtype is unknown
    VRU_SUB_TYPE_FLAT_TROLLEY = 20,        //!< Flat trolley
    VRU_SUB_TYPE_SHOPPING_TROLLEY = 21,    //!< Shopping Cart
    VRU_SUB_TYPE_STROLLER_TROLLEY = 22,    //!< Stroller
    VRU_SUB_TYPE_GARBAGE_TROLLEY = 23,     //!< Garbage bin
    VRU_SUB_TYPE_WHEEL_CHAIR_TROLLEY = 24, //!< WheelChair
    VRU_SUB_TYPE_ANIMAL = 25,              //!< Animal
  };
  serdes1(value)
};

struct TrafficLightTypeEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_TYPE_UNKNOWN = 0,   //!< unknown trafficlight
    TRAFFIC_LIGHT_TYPE_GROUP = 1,     //!< trafficlight group type
    TRAFFIC_LIGHT_TYPE_GLIM_BULB = 2, //!< trafficlight bulb type
    TRAFFIC_LIGHT_TYPE_GLIM_HALO = 3, //!< trafficlight halo type
  };
  serdes1(value)
};

/*!
 * \brief Enum to the type of object
 */
struct ObjectTypePropertyEnum {
  uint8_t value;

  enum : uint8_t {
    PROPERTY_OBJECT_TYPE_ENUM = 0, //!< Object type
    PROPERTY_GENERAL_OBJECT_TYPE_ENUM =
        1, //!< Type of general object (usually means road debris)
    PROPERTY_VEHICLE_TYPE_ENUM = 2,         //!< Type of vehicle
    PROPERTY_VEHICLE_SUB_TYPE_ENUM = 3,     //!< Subtype of vehicle
    PROPERTY_VRU_TYPE_ENUM = 4,             //!< Type of VRU
    PROPERTY_VRU_SUB_TYPE_ENUM = 5,         //!< Subtype of VRU
    PROPERTY_TRAFFIC_BARRIER_TYPE_ENUM = 6, //!< Type of traffic barrier
    PROPERTY_TRAFFIC_LIGHT_TYPE_ENUM = 7,   //!< Type of traffic light
  };
  serdes1(value)
};

/*!
 * \brief Enum to the type of general object (usually means road debris)
 */
struct GeneralObjectTypeEnum {
  uint8_t value;

  enum : uint8_t {
    GENERAL_OBJECT_TYPE_UNKNOWN = 0,      //!< Unknown type
    GENERAL_OBJECT_TYPE_POINT = 1,        //!< Point type object
    GENERAL_OBJECT_TYPE_BOUNDING_BOX = 2, //!< Bbox type object
  };
  serdes1(value)
};

/*!
 * \brief Enum to the type of VRU
 */
struct VRUTypeEnum {
  uint8_t value;

  enum : uint8_t {
    VRU_TYPE_UNKNOWN = 0,    //!< Unkown VRU type
    VRU_TYPE_PEDESTRIAN = 1, //!< Pedestrian
    VRU_TYPE_BICYCLIST = 2,  //!< Bicyclist
    VRU_TYPE_TRICYCLIST = 3, //!< Tricyclist
    VRU_TYPE_TROLLEY = 4,    //!< Trolley
    VRU_TYPE_ANIMAL = 5,     //!< Animal
  };
  serdes1(value)
};

/*!
 * \brief Enum to the detailed subtype of vehicle
 */
struct VehicleSubTypeEnum {
  uint8_t value;

  enum : uint8_t {
    VEHICLE_SUB_TYPE_UNKNOWN_VEHICLE = 0, //!< Unknown vehicle type
    VEHICLE_SUB_TYPE_LAMP_UNKNOWN_VEHICLE =
        1, //!< Vehicles with only front or rear lamp visible from the image in
           //!< the night or dark scenario
    VEHICLE_SUB_TYPE_UNKNOWN_CAR =
        2, //!< Known as car, but the detailed subtype is unknown
    VEHICLE_SUB_TYPE_SEDAN_CAR = 3,  //!< Sedan car
    VEHICLE_SUB_TYPE_SUV_CAR = 4,    //!< SUV
    VEHICLE_SUB_TYPE_POLICE_CAR = 5, //!< Police car
    VEHICLE_SUB_TYPE_PICKUP_CAR = 6, //!< Pickup
    VEHICLE_SUB_TYPE_UNKNOWN_TRUCK =
        7, //!< Known as truck, but the detailed subtype is unknown
    VEHICLE_SUB_TYPE_VAN_TRUCK = 8,   //!< Van type truck
    VEHICLE_SUB_TYPE_PLATE_TRUCK = 9, //!< Board(Plate) type truck
    VEHICLE_SUB_TYPE_DANGER_TRANSPORT_TRUCK =
        10, //!< Trucks transporting dangerous goods, also known as tank truck
    VEHICLE_SUB_TYPE_TRAILER_TRUCK = 11,         //!< Trailer
    VEHICLE_SUB_TYPE_VEHICLE_CARRIER_TRUCK = 12, //!< Carrier truck
    VEHICLE_SUB_TYPE_ENGINEERING_TRUCK = 13,     //!< Engineering truck
    VEHICLE_SUB_TYPE_AMBULANCE_TRUCK = 14,       //!< Ambulance truck
    VEHICLE_SUB_TYPE_FIRE_ENGINE_TRUCK = 15,     //!< Fire engine truck
    VEHICLE_SUB_TYPE_EXTREMELY_LONG_TRUCK =
        16,                                //!< Extremly long truck(over 20m)
    VEHICLE_SUB_TYPE_SWEEPER_TRUCK = 17,   //!< Sweeper truck
    VEHICLE_SUB_TYPE_FORK_LIFT_TRUCK = 18, //!< Fork lift
    VEHICLE_SUB_TYPE_UNKNOWN_BUS =
        19, //!< Known as bus, but the detailed subtype is unknown
    VEHICLE_SUB_TYPE_PASSENGER_BUS = 20,   //!< Passenger bus
    VEHICLE_SUB_TYPE_SCHOOL_BUS = 21,      //!< School bus
    VEHICLE_SUB_TYPE_CITY_BUS = 22,        //!< City bus
    VEHICLE_SUB_TYPE_ARTICULATED_BUS = 23, //!< Articulated bus
    VEHICLE_SUB_TYPE_TROLLEY_BUS = 24,     //!< Trolley bus
    VEHICLE_SUB_TYPE_MINI_BUS = 25,        //!< Minibus(van)
    VEHICLE_SUB_TYPE_RV_BUS = 26,          //!< RV(motorhome, limo)
    VEHICLE_SUB_TYPE_WHEEL_UNKNOWN_VEHICLE =
        27, //!< Vehicles detected by their wheels, so the detailed type is
            //!< unknown
  };
  serdes1(value)
};

/*!
 * \brief Enum to the type of object
 */
struct ObjectTypeEnum {
  uint8_t value;

  enum : uint8_t {
    OBJECT_TYPE_GENERAL_OBJECT = 0,  //!< General Object
    OBJECT_TYPE_VEHICLE = 1,         //!< Vehicle
    OBJECT_TYPE_VRU = 2,             //!< VRU
    OBJECT_TYPE_TRAFFIC_BARRIER = 3, //!< Traffic Barrier (Static Object)
    OBJECT_TYPE_TRAFFIC_LIGHT = 4,   //!< Traffic Light
  };
  serdes1(value)
};

/*!
 * \brief Enum to the the traffic barrier(Static Objects) type
 */
struct TrafficBarrierTypeEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_BARRIER_TYPE_UNKNOWN = 0,          //!< Unknown traffic barrier type
    TRAFFIC_BARRIER_TYPE_CONE = 1,             //!< Cone
    TRAFFIC_BARRIER_TYPE_WARNING_TRIANGLE = 2, //!< Warining triangle
    TRAFFIC_BARRIER_TYPE_BOLLARD_SLEEVE = 3,   //!< Bollard sleeve
    TRAFFIC_BARRIER_TYPE_PARKING_LOCK = 4,     //!< Lock of parking slot
    TRAFFIC_BARRIER_TYPE_AFRAME_SIGN = 5,      //!< Aframe sign
    TRAFFIC_BARRIER_TYPE_ANTICRASH_BUCKET = 6, //!< Anticrash bucket
    TRAFFIC_BARRIER_TYPE_DUSTBIN = 7,          //!< Dustbin(Garbage Can)
    TRAFFIC_BARRIER_TYPE_BARRIER_STONE = 8,    //!< Stone barrier
    TRAFFIC_BARRIER_TYPE_WATER_BARRIER = 9,    //!< The water horse
    TRAFFIC_BARRIER_TYPE_PARKING_STOPPER = 10, //!< The parking stopper
  };
  serdes1(value)
};

/*!
 * \brief Enum to the type of vehicle
 */
struct VehicleTypeEnum {
  uint8_t value;

  enum : uint8_t {
    VEHICLE_TYPE_UNKNOWN = 0, //!< Unknown vehicle type
    VEHICLE_TYPE_CAR = 1,     //!< Car
    VEHICLE_TYPE_TRUCK = 2,   //!< Truck
    VEHICLE_TYPE_BUS = 3,     //!< Bus
  };
  serdes1(value)
};

/*!
 * \brief Enum to the vehicle beam light status types
 */
struct VehicleBeamLightEnum {
  uint8_t value;

  enum : uint8_t {
    VEHICLE_BEAMLIGHT_UNKNOWN = 0, //!< Unknown beam light status
    VEHICLE_BEAMLIGHT_OFF = 1,     //!< Beam light off
    VEHICLE_BEAMLIGHT_LOW = 2,     //!< Beam light on
    VEHICLE_BEAMLIGHT_HIGH = 3,    //!< High beam light on
  };
  serdes1(value)
};

/*!
 * \brief Enum to the vehicle brake light status types
 */
struct VehicleBrakeLightEnum {
  uint8_t value;

  enum : uint8_t {
    VEHICLE_BRAKELIGHT_UNKNOWN = 0, //!< Unknown brake light status
    VEHICLE_BRAKELIGHT_OFF = 1,     //!< Brake light off
    VEHICLE_BRAKELIGHT_ON = 2,      //!< Brake light on
  };
  serdes1(value)
};

/*!
 * \brief Enum to the vehicle door status types
 */
struct VehicleDoorStateEnum {
  uint8_t value;

  enum : uint8_t {
    VEHICLE_DOORSTATE_UNKNOWN = 0, //!< Unknown door status
    VEHICLE_DOORSTATE_OFF = 1,     //!< Door closed
    VEHICLE_DOORSTATE_ON = 2,      //!< Door opening
  };
  serdes1(value)
};

/*!
 * \brief Enum to the vehicle bink light status types
 */
struct VehicleBlinkLightEnum {
  uint8_t value;

  enum : uint8_t {
    VEHICLE_BLINKLIGHT_UNKNOWN = 0, //!< Unknown blink light status
    VEHICLE_BLINKLIGHT_OFF = 1,     //!< Blink light off
    VEHICLE_BLINKLIGHT_LEFT = 2,    //!< Blink light left (indicate left turn)
    VEHICLE_BLINKLIGHT_RIGHT = 3,   //!< Blink light right (indicate right turn)
    VEHICLE_BLINKLIGHT_BOTH = 4,    //!< Blink light both (vehicle double flash)
  };
  serdes1(value)
};

/*!
 * \brief Enum to the relation between the target vehicle and ego lane lines
 * (overlap or not observed from the image)
 */
struct VehicleEgoLaneRelationEnum {
  uint8_t value;

  enum : uint8_t {
    VEHICLE_EGO_LANE_RELATION_OVERLAP =
        0, //!< The vehicle has overlap with ego lane lines (usually cut in
           //!< cases)
    VEHICLE_EGO_LANE_RELATION_NO_OVERLAP =
        1, //!< The vehicle doesn't have overlap with ego lane lines (usually
           //!< normal cases)
    VEHICLE_EGO_LANE_RELATION_OTHERS =
        2, //!< Other relations between the vehicle and ego lane lines
    VEHICLE_EGO_LANE_RELATION_UNKNOWN =
        3, //!< Unknown relation between vehicle and ego lane lines
  };
  serdes1(value)
};

/*!
 * \brief Conti Radar Existence Probability type
 */
struct ContiRadarExistenceProbabilityType {
  uint8_t value; //!< existence probablity type value

  enum : uint8_t {
    INVALID = 0,          //!< invalid
    PROBABILITY_25 = 1,   //!< 25%
    PROBABILITY_50 = 2,   //!< 50%
    PROBABILITY_75 = 3,   //!< 75%
    PROBABILITY_90 = 4,   //!< 90%
    PROBABILITY_99 = 5,   //!< 99%
    PROBABILITY_99_9 = 6, //!< 99.9%
    PROBABILITY_100 = 7,  //!< 100%
  };
  serdes1(value)
};

/*!
 * \brief Bosch Radar sensor status
 */
struct BoschRadarSensorStatus {
  std::string sensor_tag;    //!< radar sensor tag
  int16_t hardware_failure;  //!< radar hardware failure
  int16_t alignment_failure; //!< radar alignmentfailure
  int16_t sgu_failure;       //!< sgu failure
  int16_t sensor_dirty;      //!< sensor dirty
  int16_t other_failure;     //!< other failure
  uint64_t status;           //!< reserved for radar status
  std::string details;       //!< reserved details
};

/*!
 * \brief Conti Radar Dynamic Property
 */
struct ContiRadarDynamicProperty {
  uint8_t value; //!< conti radar Dynamic Property value

  enum : uint8_t {
    MOVING = 0,               //!< moving
    STATIONARY = 1,           //!< stationary
    ON_COMING = 2,            //!<  on coming
    STATIONARY_CANDIDATE = 3, //!< stationary candidate
    UNKNOWN = 4,              //!<  unknown
    CROSSING_STATIONARY = 5,  //!< crossing stationary
    CROSSING_MOVING = 6,      //!< crossing moving
    STOPPED = 7,              //!<  stopped
  };
};

/*!
 * \brief RadarPerceptionResult meta data
 */
struct RadarPerceptionResultMeta {
  uint64_t sensor_timestamp_us; //!< timestamp corresponding to radarperception
                                //!< result   \unit(us)
  uint64_t
      pipeline_start_timestamp_us; //!< timestamp when pipeline start \unit(us)
  uint64_t pipeline_finish_timestamp_us; //!< timestamp when pipeline finish
                                         //!< \unit(us)
  std::string version_tag;               //!< version tag
};

/*!
 * \brief  Conti Radar Measurement State
 */
struct ContiRadarMeasurementState {
  uint8_t value; //!< measurement state value

  enum : uint8_t {
    DELETED = 0,           //!<  deleted
    NEW = 1,               //!<  new
    MEASURED = 2,          //!< measured
    PREDICTED = 3,         //!< predicted
    DELETED_FOR_MERGE = 4, //!< deleted for merge
    NEW_FROM_MERGE = 5,    //!< new form merge
  };
};

struct PerceptionFusionResultMeta {
  uint64_t sensor_timestamp_us;          //!< Time Stamp of Sensor Msg
  uint64_t pipeline_start_timestamp_us;  //!< Time Stamp of Pipeline Start
  uint64_t pipeline_finish_timestamp_us; //!< Time Stamp of Pipeline End
  std::string version_tag;
  serdes4(sensor_timestamp_us, pipeline_start_timestamp_us,
          pipeline_finish_timestamp_us, version_tag)
};

struct EgoParkingSlot {
  uint64_t id;
  serdes1(id)
};

/*!
 * \brief Enum to the perception fusion AEB state
 */
struct PerceptionFusionAEBStateEnum {
  uint8_t value;

  enum : uint8_t {
    AEB_STATE_OFF = 0, //!< Off
    AEB_STATE_FCW = 1, //!< FCW, where ttc is only got from fusion s3 (usually
                       //!< available when 1.2s<ttc<2.5s)
    AEB_STATE_AEB = 3, //!< AEB, where ttc is got by combining s1, s2 and s3
                       //!< (usually available when ttc<1.2s)
  };
  serdes1(value)
};

/*!
 * \brief Trafficlight ONOFF enum
 */
struct TrafficLightOnOffEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_ONOFF_ON = 0,     //!< traffic_light_group is on
    TRAFFIC_LIGHT_ONOFF_OFF = 1,    //!< traffic_light_group is off
    TRAFFIC_LIGHT_ONOFF_UNKNOW = 2, //!< traffic_light_group is unknow
    TRAFFIC_LIGHT_ONOFF_UNMARKED =
        3, //!< Unmarked OnOff is used if such type of traffic light has no
           //!< OnOff property like bulb or halo
    TRAFFIC_LIGHT_ONOFF_RESERVE1 = 4, //!< Reserve1
    TRAFFIC_LIGHT_ONOFF_RESERVE2 = 5, //!< Reserve2
    TRAFFIC_LIGHT_ONOFF_RESERVE3 = 6, //!< Reserve3
  };
  serdes1(value)
};

/*!
 * \brief Trafficlight status enum
 */
struct TrafficLightStatusEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_STATUS_RED = 0,             //!< Always red
    TRAFFIC_LIGHT_STATUS_YELLOW = 1,          //!< Always yellow
    TRAFFIC_LIGHT_STATUS_GREEN = 2,           //!< Always green
    TRAFFIC_LIGHT_STATUS_YELLOW_BLINKING = 3, //!< Yellow blinking
    TRAFFIC_LIGHT_STATUS_GREEN_BLINKING = 4,  //!< Green blinking
    TRAFFIC_LIGHT_STATUS_OFF =
        5, //!< Always off for dd-fusion out and multi-frame
    TRAFFIC_LIGHT_STATUS_OTHER = 6,        //!< Other status
    TRAFFIC_LIGHT_STATUS_UNKNOWN = 7,      //!< Unknow status
    TRAFFIC_LIGHT_STATUS_RED_BLINKING = 8, //!< Red blinking
    TRAFFIC_LIGHT_STATUS_UNMARKED =
        9, //!< Unmarked status is used if such type of traffic light has no
           //!< Status property like group
    TRAFFIC_LIGHT_STATUS_RESERVE1 = 10, //!< Reserve1
    TRAFFIC_LIGHT_STATUS_RESERVE2 = 11, //!< Reserve2
    TRAFFIC_LIGHT_STATUS_RESERVE3 = 12, //!< Reserve3
  };
  serdes1(value)
};

/*!
 * \brief Trafficlight pattern enum
 */
struct TrafficLightPatternEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_PATTERN_SOLID_CIRCLE = 0,        //!< Solid circle
    TRAFFIC_LIGHT_PATTERN_LEFT_ARROW = 1,          //!< Left arrow
    TRAFFIC_LIGHT_PATTERN_RIGHT_ARROW = 2,         //!< Right arrow
    TRAFFIC_LIGHT_PATTERN_UP_ARROW = 3,            //!< Up arrow
    TRAFFIC_LIGHT_PATTERN_DOWN_ARROW = 4,          //!< Down arrow
    TRAFFIC_LIGHT_PATTERN_PEDESTRIAN = 5,          //!< Pedestrian
    TRAFFIC_LIGHT_PATTERN_NON_MOTOR = 6,           //!< Non motor
    TRAFFIC_LIGHT_PATTERN_LEFT_UTURN = 7,          //!< Left UTurn
    TRAFFIC_LIGHT_PATTERN_RIGHT_UTURN = 8,         //!< Right UTurn
    TRAFFIC_LIGHT_PATTERN_NUMBER = 9,              //!< Number
    TRAFFIC_LIGHT_PATTERN_PROGRESS_BAR = 10,       //!< Process Bar
    TRAFFIC_LIGHT_PATTERN_TEXT_LIGHT = 11,         //!< Text bulb
    TRAFFIC_LIGHT_PATTERN_X_SHAPED_LIGHT = 12,     //!< X shape
    TRAFFIC_LIGHT_PATTERN_LEFTUP_ARROW = 13,       //!< Leftup 45 degree arrow
    TRAFFIC_LIGHT_PATTERN_RIGHTUP_ARROW = 14,      //!< Rightup 45 degree arrow
    TRAFFIC_LIGHT_PATTERN_LEFTDOWN_ARROW = 15,     //!< Leftdown 45 degree arrow
    TRAFFIC_LIGHT_PATTERN_RIGHTDOWN_ARROW = 16,    //!< Rightup 45 degree arrow
    TRAFFIC_LIGHT_PATTERN_LEFT_AND_UP_ARROW = 17,  //!< Left arrow and Up arrow
    TRAFFIC_LIGHT_PATTERN_RIGHT_AND_UP_ARROW = 18, //!< Right arrow and Up arrow
    TRAFFIC_LIGHT_PATTERN_LEFT_AND_RIGHT_ARROW =
        19, //!< Left arrow and Right arrow
    TRAFFIC_LIGHT_PATTERN_LEFT_AND_RIGHT_AND_UP_ARROW =
        20, //!< Left arrow and Right arrow and Up arrow
    TRAFFIC_LIGHT_PATTERN_LEFT_AND_LEFT_UTURN_ARROW =
        21, //!< Left arrow and Left_uturn arrow
    TRAFFIC_LIGHT_PATTERN_RIGHT_AND_RIGHT_UTURN_ARROW =
        22,                            //!< Right and Right_uturn arrow
    TRAFFIC_LIGHT_PATTERN_OTHER = 23,  //!< Other
    TRAFFIC_LIGHT_PATTERN_UNKNOW = 24, //!< Unknow
    TRAFFIC_LIGHT_PATTERN_OFF = 25,    //!< OFF
    TRAFFIC_LIGHT_PATTERN_UNMARKED =
        26, //!< Unmarked pattern is used if such type of traffic light has no
            //!< Pattern property like group
    TRAFFIC_LIGHT_PATTERN_RESERVE1 = 27, //!< Reserve1
    TRAFFIC_LIGHT_PATTERN_RESERVE2 = 28, //!< Reserve2
    TRAFFIC_LIGHT_PATTERN_RESERVE3 = 29, //!< Reserve3
  };
  serdes1(value)
};

/*!
 * \brief roadmark label map struct
 */
struct RoadMarkLabelMap {
  uint32_t height;   //!< label map height
  uint32_t width;    //!< label map width
  uint32_t channels; //!< label map channel
  /*!
   * \brief roadmark label map data
   * \vec_max_size{1000000}
   */
  std::vector<uint8_t> data;
};

/*!
 * \brief Enum to the type of ground line
 */
struct GroundLineTypeEnum {
  uint8_t value;

  enum : uint8_t {
    GROUND_LINE_TYPE_UNKNOWN = 0, //!< Unknown ground line type
    GROUND_LINE_TYPE_WALL = 1,    //!< Wall
    GROUND_LINE_TYPE_PILLAR = 2,  //!< Pillar
    GROUND_LINE_TYPE_FENCE = 3,   //!< Fence
    GROUND_LINE_TYPE_STEP = 4,    //!< Step
    GROUND_LINE_TYPE_SPECIAL = 5, //!< Special building
  };
};

/*!
 * \brief 3D visible polygon boundary of vehicles from image view
 */
struct VehicleVPBInfo {
  uint8_t vpb_point_num; //!< valid vpb point number
  /*!
   * \brief x coordinate sequences in bev
   * \vec_max_size{32}
   */
  std::vector<float> vpb_bv_x;
  /*!
   * \brief y coordinate sequences in bev
   * \vec_max_size{32}
   */
  std::vector<float> vpb_bv_y;
  /*!
   * \brief z coordinate sequences in bev
   * \vec_max_size{32}
   */
  std::vector<float> vpb_bv_z;
};

/*!
 * \brief object tracking information struct
 */
struct ObjectTrackInfo {
  uint64_t track_id;    //!< Track id
  uint64_t track_times; //!< Num of frames the object has been tracked
  /*!
   * \brief Time the object has been tracked
   * \unit{s}
   */
  float age_in_seconds;
};

/*!
 * \brief object AEB feature information struct
 */
struct ObjectAEBFeature {
  /*!
   * \brief AEB feature data for object
   * \vec_max_size{64}
   */
  std::vector<float> data;
};

/*!
 * \brief Vehicle ReID feature struct
 */
struct ObjectReIDFeature {
  /*!
   * \brief ReID embedding feature data for vehicle
   * \vec_max_size{64}
   */
  std::vector<float> data;
};

/*!
 * \brief Enum to the type of parking slot point
 */
struct ParkingSlotPointTypeEnum {
  uint8_t value;

  enum : uint8_t {
    PARKING_SLOT_POINT_TYPE_UNKNOWN = 0, //!< Unknown parking slot point type
    PARKING_SLOT_POINT_TYPE_REAL = 1,    //!< Real parking slot point type
    PARKING_SLOT_POINT_TYPE_FAKE = 2,    //!< Fake parking slot point type
  };
};

/*!
 * \brief Enum to the type of parking slot
 */
struct ParkingSlotTypeEnum {
  uint8_t value;

  enum : uint8_t {
    PARKING_SLOT_TYPE_UNKNOWN = 0,   //!< Unknown parking slot type
    PARKING_SLOT_TYPE_AVAILABLE = 1, //!< Available parking slot
    PARKING_SLOT_TYPE_OCCUPIED = 2,  //!< Occupied parking slot
  };
};

/*!
 * \brief Enum to the charge type of parking slot
 */
struct ParkingSlotChargeTypeEnum {
  uint8_t value;

  enum : uint8_t {
    PARKING_SLOT_CHARGE_TYPE_UNKNOWN = 0, //!< Unknown parking slot charge type
    PARKING_SLOT_CHARGE_TYPE_AVAILABLE =
        1, //!< Parking slot charge type available
    PARKING_SLOT_CHARGE_TYPE_NOT_AVAILABLE =
        2, //!< Parking slot charge type not available
  };
};

struct ImageRetrievalDescriptorPropertyEnum {
  uint8_t value;

  enum : uint8_t {
    IMAGE_RETRIEVAL_DESCRIPTOR_PROPERTY_TAG = 0,
    IMAGE_RETRIEVAL_DESCRIPTOR_PROPERTY_AVAILABLE = 1,
    IMAGE_RETRIEVAL_DESCRIPTOR_PROPERTY_DESCRIPTOR = 2,
    IMAGE_RETRIEVAL_DESCRIPTOR_PROPERTY_LOG_INFO_2D = 3,
    IMAGE_RETRIEVAL_DESCRIPTOR_PROPERTY_RESERVED_INFO = 4,
  };
};

/*!
 * \brief Enum to the type of pole line
 */
struct PoleLineTypeEnum {
  uint8_t value;

  enum : uint8_t {
    POLE_LINE_TYPE_UNKNOWN = 0,      //!< Unknown pole line type
    POLE_LINE_TYPE_GENERAL_POLE = 1, //!< General pole line
    POLE_LINE_TYPE_LIGHT_POLE = 2,   //!< Light pole line
  };
};

struct LocationInfo {
  maf_std::Header header;
  bool is_valid;
  float qw;
  float qx;
  float qy;
  float qz;
  float tx;
  float ty;
  float tz;
};

struct WheelSpeed {
  maf_std::Header header;
  float front_left;
  float front_right;
  float rear_left;
  float rear_right;
};

/*!
 * \brief Enum to the num of speed sign
 */
struct SpeedSignSpeedEnum {
  uint8_t value;

  enum : uint8_t {
    SPEED_SIGN_SPEED_UNKNOWN = 0, //!< Unknown speed num
    SPEED_SIGN_SPEED_5 = 1,       //!< Speed 5
    SPEED_SIGN_SPEED_10 = 2,      //!< Speed 10
    SPEED_SIGN_SPEED_15 = 3,      //!< Speed 15
    SPEED_SIGN_SPEED_20 = 4,      //!< Speed 20
    SPEED_SIGN_SPEED_25 = 5,      //!< Speed 25
    SPEED_SIGN_SPEED_30 = 6,      //!< Speed 30
    SPEED_SIGN_SPEED_35 = 7,      //!< Speed 35
    SPEED_SIGN_SPEED_40 = 8,      //!< Speed 40
    SPEED_SIGN_SPEED_50 = 9,      //!< Speed 50
    SPEED_SIGN_SPEED_60 = 10,     //!< Speed 60
    SPEED_SIGN_SPEED_70 = 11,     //!< Speed 70
    SPEED_SIGN_SPEED_80 = 12,     //!< Speed 80
    SPEED_SIGN_SPEED_90 = 13,     //!< Speed 90
    SPEED_SIGN_SPEED_100 = 14,    //!< Speed 100
    SPEED_SIGN_SPEED_110 = 15,    //!< Speed 110
    SPEED_SIGN_SPEED_120 = 16,    //!< Speed 120
  };
};

/*!
 * \brief Enum to the shape of traffic sign
 */
struct TrafficSignShapeEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_SIGN_SHAPE_UNKNOWN = 0,           //!< Unknown traffic sign shape
    TRAFFIC_SIGN_SHAPE_BACKGROUND = 1,        //!< Background
    TRAFFIC_SIGN_SHAPE_HEXAGON = 2,           //!< Hexagon
    TRAFFIC_SIGN_SHAPE_DIAMOND = 3,           //!< Diamond
    TRAFFIC_SIGN_SHAPE_RECTANGLE = 4,         //!< Rectangle
    TRAFFIC_SIGN_SHAPE_TRIANGLE = 5,          //!< Triangle
    TRAFFIC_SIGN_SHAPE_INVERTED_TRIANGLE = 6, //!< Inverted triangle
    TRAFFIC_SIGN_SHAPE_POLYGON = 7,           //!< Polygon
    TRAFFIC_SIGN_SHAPE_CIRCLE = 8,            //!< Circle
  };
};

/*!
 * \brief Enum to the type of road mark bounding box
 */
struct RoadMarkTypeEnum {
  uint8_t value;

  enum : uint8_t {
    ROAD_MARK_TYPE_UNKNOWN = 0,                   //!< Unknown road mark type
    ROAD_MARK_TYPE_GO_STRAIGHT = 1,               //!< Go straight
    ROAD_MARK_TYPE_GO_STRAIGHT_OR_TURN_LEFT = 2,  //!< Go straight or turn left
    ROAD_MARK_TYPE_GO_STRAIGHT_OR_TURN_RIGHT = 3, //!< Go straight or turn right
    ROAD_MARK_TYPE_GO_STRAIGHT_OR_TURN_BACK = 4,  //!< Go straight or turn back
    ROAD_MARK_TYPE_TURN_LEFT = 5,                 //!< Turn left
    ROAD_MARK_TYPE_TURN_RIGHT = 6,                //!< Turn right
    ROAD_MARK_TYPE_TURN_BACK = 7,                 //!< Turn back
    ROAD_MARK_TYPE_TURN_LEFT_OR_TURN_BACK = 8,    //!< Turn left or turn back
    ROAD_MARK_TYPE_TURN_LEFT_OR_TURN_RIGHT = 9,   //!< Turn left or turn right
    ROAD_MARK_TYPE_LEFT_CONFLUENCE = 10,          //!< Left confluence
    ROAD_MARK_TYPE_RIGHT_CONFLUENCE = 11,         //!< Right confluence
  };
};

/*!
 * \brief Enum to the speed type of speed sign
 */
struct SpeedSignTypeEnum {
  uint8_t value;

  enum : uint8_t {
    SPEED_SIGN_TYPE_UNKNOWN = 0,    //!< Unknown speed sign type
    SPEED_SIGN_TYPE_HIGH_SPEED = 1, //!< High speed limit
    SPEED_SIGN_TYPE_LOW_SPEED = 2,  //!< Low speed limit
    SPEED_SIGN_TYPE_DESPEED = 3,    //!< Cancel speed limit
  };
};

struct FeaturePointArrayPropertyEnum {
  uint8_t value;

  enum : uint8_t {
    FEATURE_POINT_ARRAY_PROPERTY_TAG = 0,
    FEATURE_POINT_ARRAY_PROPERTY_AVAILABLE = 1,
    FEATURE_POINT_ARRAY_PROPERTY_NUM_POINT = 2,
    FEATURE_POINT_ARRAY_PROPERTY_TRACK_IDS = 3,
    FEATURE_POINT_ARRAY_PROPERTY_POINT_XS = 4,
    FEATURE_POINT_ARRAY_PROPERTY_POINT_YS = 5,
    FEATURE_POINT_ARRAY_PROPERTY_DESCRIPTOR_SIZE = 6,
    FEATURE_POINT_ARRAY_PROPERTY_DESCRIPTORS = 7,
    FEATURE_POINT_ARRAY_PROPERTY_LOG_INFO_2D = 8,
    FEATURE_POINT_ARRAY_PROPERTY_RESERVED_INFO = 9,
  };
};

/*!
 * \brief ADAS Perception result struct
 */
struct ADASPerceptionResult {
  uint8_t ihc_highbeam_on; //!< Whether ihc highbeam should be on. (0:off, 1:on)
  double ihc_highbeam_score; //!< Score of ihc_highbeam_on to be 1, range from 0
                             //!< to 1
};

/*!
 * \brief Enum to the type of lane segment point
 */
struct LaneSegmentPointEnum {
  uint8_t value;

  enum : uint8_t {
    LANE_SEGMENT_POINT_UNKNOWN = 0,         //!< Unknown lane segment point type
    LANE_SEGMENT_POINT_FAKE_POINT = 1,      //!< Fake point type
    LANE_SEGMENT_POINT_REAL_POINT = 2,      //!< Real point type
    LANE_SEGMENT_POINT_FARTHEST_POINT = 3,  //!< Farthest point type
    LANE_SEGMENT_POINT_OCCLUSION_POINT = 4, //!< Occlusion point type
  };
};

/*!
 * \brief Enum to the type of lane line
 */
struct LaneTypeEnum {
  uint8_t value;

  enum : uint8_t {
    LANE_TYPE_UNKNOWN = 0,             //!< Unknown lane line type
    LANE_TYPE_DASHED = 1,              //!< Dashed lane line type
    LANE_TYPE_SOLID = 2,               //!< Solid lane line type
    LANE_TYPE_DOUBLE_SOLID = 3,        //!< Double solid lane line type
    LANE_TYPE_DOUBLE_DASHED_SOLID = 4, //!< Double dashed solid lane line type
  };
};

/*!
 * \brief Enum to the color of lane line
 */
struct LaneColorEnum {
  uint8_t value;

  enum : uint8_t {
    LANE_COLOR_UNKNOWN = 0, //!< Unknown lane line color
    LANE_COLOR_WHITE = 1,   //!< White
    LANE_COLOR_YELLOW = 2,  //!< Yellow
    LANE_COLOR_RED = 3,     //!< Red
    LANE_COLOR_BLUE = 4,    //!< Blue
  };
};

/*!
 * \brief Enum to the horizontal line type of lane line
 */
struct LaneHorizontalTypeEnum {
  uint8_t value;

  enum : uint8_t {
    LANE_HORIZENTAL_TYPE_UNKNOWN = 0,    //!< Unknown horizontal line type
    LANE_HORIZENTAL_TYPE_STOP_LINE = 1,  //!< Stop line
    LANE_HORIZENTAL_TYPE_ZEBRA_LINE = 2, //!< Zebra line
    LANE_HORIZENTAL_TYPE_SPEED_BUMP = 3, //!< Speed bump
  };
};

struct LidarPerceptionResultMeta {
  uint64_t sensor_timestamp_us;
  uint64_t pipeline_start_timestamp_us;
  uint64_t pipeline_finish_timestamp_us;
  std::string version_tag;
};

struct FrameMeta {
  uint64_t timestamp_us;
  uint64_t sequence;
  CameraSourceEnum camera_source;
  CameraModelEnum camera_model;
  serdes4(timestamp_us, sequence,
          camera_source, camera_model)
};

struct Polygon3f {
  std::vector<Point3f> points;
  serdes1(points)
};

/*!
 * \brief Object type information struct
 */
struct ObjectTypeInfo {
  std::vector<ObjectTypePropertyEnum>
      type_properties;                         //!< object properties type
  ObjectTypeEnum type;                         //!< object type
  GeneralObjectTypeEnum general_object_type;   //!< general object type
  VehicleTypeEnum vehicle_type;                //!< vehicle type
  VehicleSubTypeEnum vehicle_sub_type;         //!< vehicle subtype
  VRUTypeEnum vru_type;                        //!< VRU type
  VRUSubTypeEnum vru_sub_type;                 //!< VRU subtype
  TrafficBarrierTypeEnum traffic_barrier_type; //!< Traffic Barrier  type
  TrafficLightTypeEnum traffic_light_type;     //!< Traffic Light  type
  serdes9(type_properties, type, general_object_type,
          vehicle_type, vehicle_sub_type, vru_type,
          vru_sub_type, traffic_barrier_type, traffic_light_type)
};

/*!
 * \brief Vehicle beam light information struct
 */
struct VehicleBeamLightInfo {
  float beam_light_on_score; //!< Score of the high_beam_light_on of the target
                             //!< vehicle
  VehicleBeamLightEnum
      beam_light_state; //!< State enum of the high_beam_light_on of the target
                        //!< vehicle
  serdes2(beam_light_on_score, beam_light_state)
};

/*!
 * \brief Vehicle blink light information struct
 */
struct VehicleBlinkLightInfo {
  float blink_light_left_score;  //!< Score of the target vehicle to be
                                 //!< blink_light_left
  float blink_light_right_score; //!< Score of the target vehicle to be
                                 //!< blink_light_right
  float blink_light_both_score;  //!< Score of the target vehicle to be
                                 //!< blink_light_both
  VehicleBlinkLightEnum
      blink_light_state; //!< State enum of the blink_light_state of the target
                         //!< vehicle
  serdes4(blink_light_left_score, blink_light_right_score, blink_light_both_score, blink_light_state)
};

/*!
 * \brief Vehicle brake light information struct
 */
struct VehicleBrakeLightInfo {
  float brake_light_on_score; //!< Score of the target vehicle to be
                              //!< brake_light_on
  VehicleBrakeLightEnum brake_light_state; //!< State enum of the brake_light of
                                           //!< the target vehicle
  serdes2(brake_light_on_score, brake_light_state)
};

/*!
 * \brief Vehicle door state information struct
 */
struct VehicleDoorStateInfo {
  float
      door_state_on_score; //!< Score of the target vehicle to be door_state_on
  VehicleDoorStateEnum
      door_state; //!< State enum of the door_state of the target vehicle
  serdes2(door_state_on_score, door_state)
};

/*!
 * \brief Information for the relation of target vehicle and ego lane lines
 */
struct VehicleEgoLaneRelationInfo {
  float ego_lane_overlap_score; //!< Score of the target vehicle having ovelap
                                //!< with ego lane
  float ego_lane_no_overlap_score; //!< Score of the target vehicle having no
                                   //!< ovelap with ego lane
  VehicleEgoLaneRelationEnum
      ego_lane_relation; //!< State enum of the ego_lane_relation of the target
                         //!< vehicle
  serdes3(ego_lane_overlap_score, ego_lane_no_overlap_score, ego_lane_relation)
};

struct FusionGroundLineData {
  uint8_t available;
  GroundLineTypeEnum type;
  uint64_t track_id;
  float score;
  std::vector<Point3f> points_fusion;
  std::vector<float> points_fusion_confidence;
  std::vector<Point3f> local_points_fusion;
  std::vector<float> local_points_fusion_confidence;

  enum : uint8_t {
    FUSION_GROUND_LINE_SOURCE_VISON = 1,
    FUSION_GROUND_LINE_SOURCE_ULTRASONIC = 2,
  };
};

struct FusionParkingSlotData {
  uint8_t available;
  uint8_t charge_property;
  uint64_t track_id;
  int32_t empty_votes;
  std::vector<Point3f> points_fusion;
  std::vector<float> points_fusion_confidence;
  std::vector<ParkingSlotPointTypeEnum> points_fusion_type;
  std::vector<Point3f> local_points_fusion;
  std::vector<float> local_points_fusion_confidence;
  std::vector<ParkingSlotPointTypeEnum> local_points_fusion_type;
  std::vector<Point2f> points_fusion_in_image;
  std::vector<float> points_fusion_in_image_confidence;
  std::vector<ParkingSlotPointTypeEnum> points_fusion_in_image_type;

  enum : uint8_t {
    FUSION_PARKING_SLOT_SOURCE_VISON = 1,
    FUSION_PARKING_SLOT_SOURCE_ULTRASONIC = 2,
    FUSION_PARKING_SLOT_SOURCE_MAP = 4,
  };
};

/*!
 * \brief Structure to represent information of Traffic Light Decision Result
 */
struct TrafficLightDecision {
  CameraSourceEnum camera_source;  //!< camera source
  CameraModelEnum camera_model;    //!< camera model
  TrafficLightPatternEnum pattern; //!< Pattern
  TrafficLightStatusEnum status;   //!< Status
  float duration;                  //!< Current status duration
  float remaining;                 //!< Current status remaining
  uint8_t count_number;            //!< count_number from number_light
  float score;                     //!< confidence
  std::string reserved_info;       //!< reserved info
  serdes9(camera_source, camera_model, pattern, 
          status, duration, remaining,
          count_number, score, reserved_info)
};

/*!
 * \brief  TrafficLight associated information
 */
struct TrafficLightAssociatedInfo {
  CameraSourceEnum camera_source; //!< camera source
  CameraModelEnum camera_model;   //!< camera model
  uint32_t track_id;              //!< trackid
  serdes3(camera_source, camera_model, track_id)
};

/*!
 * \brief Traffic light ROI
 */
struct TrafficLightROI {
  CameraSourceEnum camera_source; //!< camera source
  CameraModelEnum camera_model;   //!< camera model
  Rect4f roi;                     //!< ROI bbox
  serdes3(camera_source, camera_model, roi)
};

/*!
 * \brief RoadMark struct
 */
struct RoadMark {
  CameraSourceEnum camera_source; //!< camera_source
  CameraModelEnum camera_model;   //!< camera_model
  uint8_t available;              //!< Availability of roadmark perception
  RoadMarkLabelMap label_map_2d;  //!< label map of roadmark
};

/*!
 * \brief ground line struct
 */
struct GroundLine {
  CameraSourceEnum camera_source; //!< camera_source
  CameraModelEnum camera_model;   //!< camera_model
  uint8_t available;              //!< Availability of ground line perception
  float score; //!< Score indicates the ground line existing confidence
  GroundLineTypeEnum type; //!< Subtype of ground line
  /*!
   * \brief perception groundline 2d points
   * \vec_max_size{1000000}
   */
  std::vector<Point2f> points;
  /*!
   * \brief perception groundline 3d points
   * \vec_max_size{10000}
   */
  std::vector<Point3f> points_3d;
};

/*!
 * \brief parking slot 2d point struct
 */
struct ParkingSlotPoint {
  Point2f
      position; //!< parking slot corner point position in image, \unit{pixel}
  float confidence;              //!< parking slot corner confidence
  ParkingSlotPointTypeEnum type; //!< parking slot corner type
};

/*!
 * \brief parking slot center point struct
 */
struct ParkingSlotCenterPoint {
  Point2f
      position; //!< parking slot center point position in image, \unit{pixel}
  ParkingSlotTypeEnum available_type;    //!< parking slot available type
  float confidence;                      //!< parking slot confidence
  float center_line_angle;               //!< parking slot center line angle
  ParkingSlotChargeTypeEnum charge_type; //!< parking slot charge type
};

/*!
 * \brief parking slot 3d point struct
 */
struct ParkingSlotPoint3d {
  Point3f position; //!< parking slot corner 3d point position, \unit{m}
  float confidence; //!< parking slot corner confidence
  ParkingSlotPointTypeEnum type; //!< parking slot corner point type
};

struct ImageRetrievalDescriptor {
  /*!
   * \brief camera name
   */
  std::string tag;
  /*!
   * \brief data valid flag
   */
  uint8_t available;
  /*!
   * \brief descriptor vector
   */
  std::vector<float> feature_descriptor;
  /*!
   * \brief used to record log information
   */
  std::string log_info_2d;
  /*!
   * \brief reserved information
   */
  std::string reserved_info;
};

/*!
 * \brief pole line 2d point struct
 */
struct PoleLinePoint {
  Point2f position; //!< Position of pole line 2d point
  float visibility; //!< Visible score
};

/*!
 * \brief HDmap trafficlight
 */
struct TrafficLightIndication {
  std::vector<Point3f> bounding_box_3d;
  TrafficLightPatternEnum pattern;
};

/*!
 * \brief Road edge struct
 */
struct RoadEdge {
  CameraSourceEnum camera_source; //!< camera_source
  CameraModelEnum camera_model;   //!< camera_model
  float score;   //!< Score indicates the road edge existing confidence
  int32_t index; //!< Index indicates the road edge relative position id
  std::vector<float>
      points_2d_x; //!< X coordinate of road edge 2d point, \unit{pixel}
  std::vector<float>
      points_2d_y; //!< Y coordinate of road edge 2d point, \unit{pixel}
  std::vector<float> points_2d_v; //!< Visibility of road edge 2d point
  uint32_t track_id;              //!< Track id
  /*!
   * \brief coefficient of 3d road edge
   * \vec_fix_size{4}
   */
  std::vector<float> coefficient_bv;
  /*!
   * \brief standard deviation of 3d road edge
   * \vec_fix_size{4}
   */
  std::vector<float> coefficient_bv_sigma;
  bool is_failed_3d; //!< Flag brief if road edge 3d result is available
  std::vector<float> points_3d_x; //!< X coordinate of road edge 3d point,
                                  //!< self-car coordinate system, \unit{m}
  std::vector<float> points_3d_y; //!< Y coordinate of road edge 3d point,
                                  //!< self-car coordinate system, \unit{m}
  std::vector<float> points_3d_z; //!< Z coordinate of road edge 3d point,
                                  //!< self-car coordinate system, \unit{m}
};

/*!
 * \brief traffic sign 2d point struct
 */
struct TrafficSignPoint {
  Point2f position; //!< Position of traffic sign 2d point
  float visibility; //!< Visible score
};

struct FeaturePointArray {
  /*!
   * \brief Camera name
   */
  std::string tag;
  /*!
   * \brief Data valid flag
   */
  uint8_t available;
  /*!
   * \brief Number of feature points
   */
  int32_t num_point;
  /*!
   * \brief Feature id
   */
  std::vector<uint32_t> track_ids;
  /*!
   * \brief x component of 2D pixel coordinates
   */
  std::vector<float> point_xs;
  /*!
   * \brief y component of 2D pixel coordinates
   */
  std::vector<float> point_ys;
  /*!
   * \brief Dimension of descriptor
   */
  int32_t descriptor_size;
  /*!
   * \brief Descriptor vector
   */
  std::vector<float> descriptors;
  /*!
   * \brief used to record log information
   */
  std::string log_info_2d;
  /*!
   * \brief reserved information
   */
  std::string reserved_info;
};

/*!
 * \brief Lane segment struct
 */
struct LaneSegment {
  CameraSourceEnum camera_source; //!< camera_source
  CameraModelEnum camera_model;   //!< camera_model
  /*!
   * \Frame results of lane segment expressed by 2d points
   * \vec_max_size{20}
   */
  std::vector<Point2f> points_2d;
  /*!
   * \Track id of lane segment
   * \vec_max_size{20}
   */
  std::vector<uint32_t> points_track_id;
  /*!
   * \Score of lane segment points
   * \vec_max_size{20}
   */
  std::vector<float> points_score;
  /*!
   * \Type of lane segment points
   * \vec_max_size{20}
   */
  std::vector<LaneSegmentPointEnum> points_type;
};

/*!
 * \brief Lane line struct
 */
struct Lane {
  CameraSourceEnum camera_source; //!< camera_source
  CameraModelEnum camera_model;   //!< camera_model
  float score;   //!< Score indicates the lane line existing confidence
  int32_t index; //!< Index indicates the lane line relative position id
  std::vector<float>
      points_2d_x; //!< X coordinate of lane 2d point, \unit{pixel}
  std::vector<float>
      points_2d_y; //!< Y coordinate of lane 2d point, \unit{pixel}
  std::vector<float> points_2d_v; //!< Visibility of lane 2d point
  uint32_t track_id;              //!< Track id
  /*!
   * \brief coefficient of 3d lane line
   * \vec_fix_size{4}
   */
  std::vector<float> coefficient_bv;
  /*!
   * \brief standard deviation of 3d lane line
   * \vec_fix_size{4}
   */
  std::vector<float> coefficient_bv_sigma;
  bool is_failed_3d;        //!< Flag brief if lane line 3d result is available
  LaneTypeEnum lane_type;   //!< Subtype of lane line, solid or dashed
  LaneColorEnum lane_color; //!< Color of lane line
  float lane_width;         //!< Width of lane line, \unit{m}
  bool is_centerline;       //!< Is centerline or not
  std::vector<float> points_3d_x; //!< X coordinate of lane 3d point, self-car
                                  //!< coordinate system, \unit{m}
  std::vector<float> points_3d_y; //!< Y coordinate of lane 3d point, self-car
                                  //!< coordinate system, \unit{m}
  std::vector<float> points_3d_z; //!< Z coordinate of lane 3d point, self-car
                                  //!< coordinate system, \unit{m}
  bool is_horizontal_line;        //!< Is horizontal line or not
  LaneHorizontalTypeEnum
      lane_horizontal_type; //!< Horizontal line type of lane line
  float stopline_depth;     //!< Distance to stopline, \unit{m}
};

/*!
 * \brief Vehicle state information struct
 */
struct VehicleStateInfo {
  VehicleBrakeLightInfo brake_light_info; //!< Brake light state
  VehicleBlinkLightInfo blink_light_info; //!< Blink light state
  VehicleBeamLightInfo beam_light_info;   //!< Beam light state
  VehicleDoorStateInfo door_state_info;   //!< Door state
  VehicleEgoLaneRelationInfo
      ego_lane_relation_info; //!< The relation with ego lane lines

  serdes5(brake_light_info, blink_light_info, beam_light_info, door_state_info, ego_lane_relation_info)
};

/*!
 * \brief Conti Radar Perception data  infomation
 */
struct ContiRadarPerceptionObjectData {
  uint16_t object_id;        //!< object ID of this detection generated by radar
  Point3d relative_position; //!< relative position of the object(x,y,z)
  Vector3d relative_velocity; //!< relative velocity of the object(x,y,z)
  float orientation_angle;    //!<  orientation angle(deg)
  float longitudinal_relative_acceleration; //!< longitudinal relative
                                            //!< acceleration(m/s2)
  float lateral_relative_acceleration; //!< lateral relative acceleration(m/s2)
  float length;                        //!< object length(m)
  float width;                         //!< object width(m)
  float radar_cross_section;           //!< radar cross section(dBm2)
  uint8_t longitudinal_relative_distance_rms; //!< longitudinal  distance
                                              //!< Standard deviation (m)
  uint8_t lateral_relative_distance_rms;      //!<  lateral  distance Standard
                                              //!<  deviation (m)
  uint8_t
      longitudinal_relative_velocity_rms; //!<  longitudinal relative velocity
                                          //!<  Standard deviation (m/s)
  uint8_t lateral_relative_velocity_rms;  //!<  lateral relative velocity
                                          //!<  Standard deviation (m/s)
  uint8_t longitudinal_relative_acceleration_rms; //!< longitudinal relative
                                                  //!< acceleration Standard
                                                  //!< deviation (m/s2)
  uint8_t lateral_relative_acceleration_rms;  //!< lateral relative acceleration
                                              //!< Standard deviation (m/s2)
  uint8_t relative_orientation_angle_rms;     //!<   relative orientation angle
                                              //!<   Standard deviation (deg)
  ObjectTypeInfo type_info;                   //!< object type information
  ContiRadarDynamicProperty dynamic_property; //!< ContiRadar Dynamic Property
  ContiRadarMeasurementState
      measurement_state; //!< ContiRadar  Measurement  State
  ContiRadarExistenceProbabilityType
      existence_probability_type; //!< ContiRadar Existence Probability Type
  std::string reserved_info;      //!< reserved information
};

/*!
 * \brief Basic Radar Perception data  infomation
 */
struct BoschRadarPerceptionObjectData {
  uint16_t object_id;        //!< object ID of this detection generated by radar
  ObjectTypeInfo type_info;  //!< object type information
  Point3d relative_position; //!< relative position of the object(x,y,z)
  Vector3d relative_velocity; //!< relative velocity of the object(x,y,z)
  float obstacle_prob;        //!< dynamic or static (obstacle) probability
  float exist_prob;           //!< object exist probability
  BoschRadarSensorStatus sensor_status; //!< radar status
  std::string reserved_info;            //!< reserved information
};

/*!
 * \brief Basic Radar Perception data  infomation
 */
struct BasicRadarPerceptionObjectData {
  uint16_t object_id;        //!< object ID of this detection generated by radar
  ObjectTypeInfo type_info;  //!< object type information
  Point3d relative_position; //!< relative position of the object(x,y,z)
  Point3d relative_velocity; //!< relative velocity of the object(x,y,z)
  float obstacle_prob;       //!< dynamic or static (obstacle) probability
  float exist_prob;          //!< object exist probability
};

struct FusionGroundLine {
  uint8_t available;
  std::vector<FusionGroundLineData> ground_line_data;
  std::vector<std::string> reserved_info;

  enum : uint8_t {
    FUSION_GROUND_LINE_AVAILABLE_DATA = 1,
    FUSION_GROUND_LINE_AVAILABLE_RESERVED_INFO = 2,
  };
};

struct FusionParkingSlot {
  uint8_t available;
  std::vector<FusionParkingSlotData> parking_slot_data;
  EgoParkingSlot ego_parking_slot;
  std::vector<std::string> reserved_info;

  enum : uint8_t {
    FUSION_PARKING_SLOT_AVAILABLE_DATA = 1,
    FUSION_PARKING_SLOT_AVAILABLE_EGO_PARKING_SLOT = 2,
    FUSION_PARKING_SLOT_AVAILABLE_RESERVED_INFO = 4,
  };
};

/*!
 * \brief AEB Object data information struct
 */
struct PerceptionFusionAEBObjectData {
  uint64_t track_id;        //!< Track id
  ObjectTypeInfo type_info; //!< Type
  float score;              //!< Score of existing
  /*!
   * @brief ttc(time to collision)
   * \unit{s}
   */
  float ttc;
  /*!
   * @brief Object 3d shape
   * \unit{m}
   */
  Shape3d shape;
  /*!
   * @brief Object center location in ego-vehicle coordinate at bird's-eye view
   * \unit{m}
   */
  Point3d relative_position;
  /*!
   * @brief Relative velocity of object center at bird's-eye view
   * \unit{m/s}
   */
  Vector3d relative_velocity;
  /*!
   * @brief Relative acceleration of object center at bird's-eye view
   * \unit{m/s^2}
   */
  Vector3d relative_acceleration;
  /*!
   * @brief Heading angle from the object heading to the x axis under local
   * world coordinate \unit{radian} \value_min{-2*pi} \value_max{2*pi}
   */
  float relative_heading_yaw;
  /*!
   * @brief Angular velocity of object center at bird's-eye view
   * \unit{radian/s}
   */
  float relative_angular_velocity;
  /*!
   * @brief Velocity of object center to the ground at bird's-eye view
   * \unit{m/s}
   */
  Vector3d velocity_relative_to_ground;
  /*!
   * @brief Acceleration of object center to the ground at bird's-eye view
   * \unit{m/s}
   */
  Vector3d acceleration_relative_to_ground;

    serdes12(track_id, type_info, score,
             ttc, shape, relative_position,
             relative_velocity,relative_acceleration,relative_heading_yaw,
             relative_angular_velocity,velocity_relative_to_ground,acceleration_relative_to_ground)

};

struct PerceptionMeta {
  std::string algorithm_name;
  std::string instance_name;
  uint64_t sensor_timestamp_us;
  uint64_t pipeline_start_timestamp_us;
  uint64_t pipeline_finish_timestamp_us;
  std::vector<FrameMeta> relevant_frames;
  std::string version_tag;
  std::string ts_log;
  uint64_t sequence;
  std::string task_tag;
  serdes10(algorithm_name, instance_name,
           sensor_timestamp_us, pipeline_start_timestamp_us,
           pipeline_finish_timestamp_us, relevant_frames,
           version_tag, ts_log,
           sequence, task_tag)
};
/*!
 * \brief Structure to represent information of fusion result of the traffic
 * light
 */
struct TrafficLightMultiFrame {
  CameraSourceEnum camera_source; //!< Camera source
  CameraModelEnum camera_model;   //!< Camera model
  TrafficLightTypeEnum type;      //!< Traffic light Type information
  int32_t track_id;               //!< Track id
  std::vector<TrafficLightPatternEnum> pattern; //!< Pattern
  std::vector<TrafficLightStatusEnum> status;   //!< Status
  std::vector<float> pattern_score;             //!< Confidence of pattern
  std::vector<float> status_score;              //!< Confidence of status
  float duration;                               //!< Current status duration
  float remaining;                              //!< Current status remaining
  uint8_t count_number; //!< count_number from number_light
  Shape3f shape_3d;     //!< 3D shape
  Point3f location_bv;  //!< Object center location in ego-vehicle coordinate at
                        //!< bird's-eye view
  float heading_theta_bv; //!< Heading angle from the object heading to the x
                          //!< axis in ego-vehicle coordinate
  Vector2f
      velocity_bv; //!< Relative velocity of object center at bird's-eye view
  std::vector<TrafficLightAssociatedInfo>
      associated_infos;      //!< Association information
  std::string reserved_info; //!< reserved info
  serdes17(camera_source, camera_model, type, track_id, pattern, 
           status, pattern_score, status_score, duration, remaining,
           count_number, shape_3d, location_bv, heading_theta_bv, velocity_bv,
           associated_infos, reserved_info)
};

/*!
 * \brief Structure to represent information of a single frame result of the
 * traffic light
 */
struct TrafficLightSingleFrame {
  CameraSourceEnum camera_source; //!< Camera source
  CameraModelEnum camera_model;   //!< Camera model
  float score;                 //!< Confidence on whether it is a traffic light
  TrafficLightTypeEnum type;   //!< Traffic light Type information
  Rect4f bounding_box_2d;      //!< Bounding box of a traffic light
  std::vector<float> features; //!< Feature from feature extractor
  int32_t track_id;            //!< Track id
  std::vector<TrafficLightPatternEnum>
      pattern; //!< Pattern list in a light bulb or a traffic light halo
  std::vector<TrafficLightStatusEnum>
      status; //!< Status list in a light bulb or a traffic light halo
  std::vector<TrafficLightOnOffEnum> onoff; //!< OnOff state of a light group
  uint8_t count_number;                     //!< count_number from number_light
  Shape3f shape_3d;                         //!< 3D shape
  Point3f location_bv; //!< Object center location in ego-vehicle coordinate at
                       //!< bird's-eye view
  float heading_theta_bv; //!< Heading angle from the object heading to the x
                          //!< axis in ego-vehicle coordinate
  Vector2f
      velocity_bv; //!< Relative velocity of object center at bird's-eye view
  std::vector<TrafficLightAssociatedInfo>
      associated_infos;      //!< Association information
  std::string reserved_info; //!< Reserved info

  serdes17(camera_source, camera_model, score, type, bounding_box_2d,
           features, track_id, pattern, status, onoff,
           count_number, shape_3d, location_bv, heading_theta_bv, velocity_bv, 
           associated_infos, reserved_info)
};

/*!
 * \brief Vision AEB object struct
 */
struct VisionAEBObject {
  CameraSourceEnum camera_source; //!< Camera source
  CameraModelEnum camera_model;   //!< Camera model
  ObjectTypeInfo type_info;       //!< Type
  float score;                    //!< Confidence on whether it is a AEB object
  float visibility_2d;            //!< Visibility
  Rect4f bounding_box_2d;         //!< Bbox in the image
  ObjectTrackInfo track_info;     //!< Tracking info
  /*!
   * @brief Object 3d shape
   * \unit{m}
   */
  Shape3f shape_3d;
  /*!
   * @brief Heading angle from the object heading to the x axis in ego-vehicle
   * coordinate \unit{radian} \value_min{-2*pi} \value_max{2*pi}
   */
  float heading_theta_bv;
  /*!
   * @brief Relative velocity of object center at bird's-eye view
   * \unit{m/s}
   */
  Vector3f velocity_bv;
  /*!
   * @brief Object center location in ego-vehicle coordinate at bird's-eye view
   * \unit{m}
   */
  Point3f location_bv;
  /*!
   * @brief Time to collision
   * \unit{s}
   */
  float ttc;
};

/*!
 * \brief parking slot struct
 */
struct ParkingSlot {
  CameraSourceEnum camera_source; //!< camera_source
  CameraModelEnum camera_model;   //!< camera_model
  uint8_t available;              //!< Available Bits
  /*!
   * \brief parking slot point in image(uv coordinate,pixel)
   * \vec_max_size{1000}
   */
  std::vector<ParkingSlotPoint> corner_points;
  ParkingSlotCenterPoint center_point; //!< parking slot center point and info
  /*!
   * \Frame results of a parking slot expressed by 3d points
   * \vec_max_size{8}
   */
  std::vector<ParkingSlotPoint3d> points_3d;
};

/*!
 * \brief Pole line struct
 */
struct PoleLine {
  CameraSourceEnum camera_source; //!< camera_source
  CameraModelEnum camera_model;   //!< camera_model
  float score; //!< Score indicates the pole line existing confidence
  /*!
   * \Frame results of a pole line expressed by 2d points
   * \vec_max_size{192}
   */
  std::vector<PoleLinePoint> points_2d;
  PoleLineTypeEnum type; //!< Subtype of pole line
  uint32_t track_id;     //!< Track id
};

/*!
 * \brief HDMap trafficlight trigger
 */
struct TrafficLightDetectTrigger {
  maf_std::Header header;
  uint64_t id;                                     //!< cross id
  int32_t num;                                     //!< HDMap trafficlight num
  std::vector<TrafficLightIndication> indications; //!< HDMap trafficlight
};

/*!
 * \brief road edge perception struct
 */
struct RoadEdgePerception {
  uint64_t available; //!< Availability of road edge perception
  /*!
   * \brief Perception results of road edges
   * \vec_max_size{10000}
   */
  std::vector<RoadEdge> road_edges;
  /*!
   * \brief Reserved infos of road edges
   * \vec_max_size{10000}
   */
  std::vector<std::string> reserved_infos;

  enum : uint8_t {
    VISION_PERCEPTION_ROAD_EDGE_RESULT =
        1, //!< Road edges from vision perception
    VISION_PERCEPTION_ROAD_EDGE_RESERVED_INFOS =
        2, //!< Reserved infos of road edges
  };
};

/*!
 * \brief traffic sign struct
 */
struct TrafficSign {
  CameraSourceEnum camera_source; //!< Camera source
  CameraModelEnum camera_model;   //!< Camera model
  float score; //!< Score indicates the traffic sign existing confidence
  /*!
   * \Frame results of a traffic sign expressed by 2d points
   * \vec_max_size{20}
   */
  std::vector<TrafficSignPoint> points_2d;
  uint32_t track_id;               //!< Track id
  TrafficSignShapeEnum shape_info; //!< Traffic sign shape information
  Rect4f bounding_box_2d;          //!< 2d bbox in the image
  bool is_speed_sign;              //!< Is speed sign or not
  SpeedSignTypeEnum speed_type;    //!< Speed type of speed sign
  SpeedSignSpeedEnum speed_num;    //!< Speed num of speed sign
  bool is_road_mark;               //!< Is road mark or not
  RoadMarkTypeEnum road_mark_type; //!< Type of road mark
};

struct FeaturePointPerception {
  maf_std::Header header;
  PerceptionMeta meta;
  /*!
   * \brief Data valid flag
   */
  uint8_t available;
  /*!
   * \brief Feature point vector, used to save the descriptor, 2D pixel
   * coordinates and etc
   */
  std::vector<FeaturePointArray> feature_point_arrays;

  enum : uint8_t {
    VISION_PERCEPTION_FEATURE_POINT_RESULT = 1,
  };
};

/*!
 * \brief lane segment perception struct
 */
struct LaneSegmentPerception {
  uint64_t available; //!< Availability of lane segment perception
  /*!
   * \brief Perception results of lane segment
   * \vec_max_size{10000}
   */
  std::vector<LaneSegment> lane_segments;
  /*!
   * \brief reserved infos of lane segment
   * \vec_max_size{10000}
   */
  std::vector<std::string> reserved_infos;

  enum : uint8_t {
    VISION_PERCEPTION_LANE_SEGMENT_RESULT =
        1, //!< lane segments from vision perception
    VISION_PERCEPTION_LANE_SEGMENT_RESERVED_INFOS =
        2, //!< reserved_infos of lane segments
  };
};

/*!
 * \brief lane line perception struct
 */
struct LanePerception {
  uint64_t available; //!< Availability of lane line perception
  /*!
   * \brief Perception results of lane lines
   * \vec_max_size{10000}
   */
  std::vector<Lane> lanes;
  /*!
   * \brief Reserved infos of lane lines
   * \vec_max_size{10000}
   */
  std::vector<std::string> reserved_infos;

  enum : uint8_t {
    VISION_PERCEPTION_LANE_RESULT = 1, //!< Lane lines from vision perception
    VISION_PERCEPTION_LANE_RESERVED_INFOS = 2, //!< Reserved infos of lane lines
  };
};

struct LidarPerceptionObjectData {
  uint64_t track_id;
  ObjectTypeInfo type_info;
  Shape3d shape;
  Shape3d shape_sigma;
  Point3d position;
  Point3d position_sigma;
  Vector3d velocity;
  Vector3d velocity_sigma;
  Vector3d acceleration;
  Vector3d acceleration_sigma;
  float heading_yaw;
  float heading_yaw_sigma;
  float angular_velocity;
  float angular_velocity_sigma;
  Point3d relative_position;
  Point3d relative_position_sigma;
  Vector3d relative_velocity;
  Vector3d relative_velocity_sigma;
  Vector3d relative_acceleration;
  Vector3d relative_acceleration_sigma;
  float relative_heading_yaw;
  float relative_heading_yaw_sigma;
  float relative_angular_velocity;
  float relative_angular_velocity_sigma;
  Vector3d velocity_relative_to_ground;
  Vector3d velocity_relative_to_ground_sigma;
  Vector3d acceleration_relative_to_ground;
  Vector3d acceleration_relative_to_ground_sigma;
  float score;
  Polygon3f polygon_bottom;
  Polygon3f polygon_top;
  std::string reserved_info;
};

/*!
 * \brief RadarPerceptionObjects information
 */
struct RadarPerceptionObjects {
  uint8_t available; //!< Is data available
  std::vector<BasicRadarPerceptionObjectData>
      basic_radar_perception_object_data; //!< basic radar data
  std::vector<BoschRadarPerceptionObjectData>
      bosch_radar_perception_object_data; //!< bosch radar data
  std::vector<ContiRadarPerceptionObjectData>
      conti_radar_perception_object_data; //!< conti radar data

  enum : uint8_t {
    BASIC_RADAR_PERCEPTION_OBJECT_DATA = 1, //!< basic radar
    BOSCH_RADAR_PERCEPTION_OBJECT_DATA = 2, //!< bosch radar
    CONTI_RADAR_PERCEPTION_OBJECT_DATA = 4, //!< conti radar
  };
};

struct FusionGroundLineResult {
  maf_std::Header header;
  PerceptionFusionResultMeta meta;
  FusionGroundLine ground_line;
};

struct PerceptionFusionObjectData {
  uint64_t track_id;        //!< fusion objects track id
  ObjectTypeInfo type_info; //!< fusion objects type info
  bool is_static;           //!< fusion objects whether or not be static
  float static_confidence;  //!< fusion objects static confidence score
  VehicleStateInfo vehicle_state_info; //!< vehicle state information
  Shape3d shape;     //!< fusion objects shape in world coordinate(m)
  Point3d position;  //!< fusion objects position in world coordinate(m)
  Vector3d velocity; //!< fusion objects velocity in world coordinate(m/s)
  Vector3d
      acceleration;  //!< fusion objects acceleration in world coordinate(m/s^2)
  float heading_yaw; //!< Heading angle from fusion objects to the x axis in
                     //!< world coordinate(radian)
  float angular_velocity;         //!< fusion objects angular velocity in world
                                  //!< coordinate(radian/s)
  Point3d relative_position;      //!< fusion objects position in ego-vehicle
                                  //!< coordinate(m)
  Vector3d relative_velocity;     //!< fusion objects velocity in ego-vehicle
                                  //!< coordinate(m/s)
  Vector3d relative_acceleration; //!< fusion objects acceleration in
                                  //!< ego-vehicle coordinate(m/s^2)
  float relative_heading_yaw;     //!< fusion objects heading yaw in ego-vehicle
                                  //!< coordinate(rad, range:-2*pi~2*pi)
  float relative_angular_velocity;      //!< fusion objects angular velocity in
                                        //!< ego-vehicle coordinate(rad/s)
  Vector3d velocity_relative_to_ground; //!< fusion objects velocity relative to
                                        //!< ground(m/s)
  Vector3d acceleration_relative_to_ground; //!< fusion objects acceleration
                                            //!< relative to ground(m/s^2)
  float score;                              //!< existence score of the object
  float crash_risk_ttc;                     //!< fusion objects crash ris ttc(s)
  Polygon3f polygon_bottom; //!< fusion objects shape in ego-vehicle coordinate
  Polygon3f polygon_top;    //!< fusion objects shape in ego-vehicle coordinate

  serdes22(track_id, type_info, is_static, static_confidence, vehicle_state_info,
           shape, position, velocity, acceleration, heading_yaw, 
           angular_velocity, relative_position, relative_velocity, relative_acceleration, relative_heading_yaw, 
           relative_angular_velocity, velocity_relative_to_ground, acceleration_relative_to_ground, score, crash_risk_ttc, 
           polygon_bottom, polygon_top)
};

struct FusionParkingSlotResult {
  maf_std::Header header;
  PerceptionFusionResultMeta meta;
  FusionParkingSlot parking_slot;
};

/*!
 * \brief AEB objects of perception fusion struct
 */
struct PerceptionFusionAEBObjects {
  uint8_t available; //!< Available or not, 0: not available; 1: available
  std::vector<PerceptionFusionAEBObjectData>
      perception_fusion_aeb_objects_data;  //!< AEB objects data for
                                           //!< perception_fusion
  std::vector<std::string> reserved_infos; //!< Reserved info

  enum : uint8_t {
    PERCEPTION_FUSION_AEB_OBJECTS_DATA = 1,
    PERCEPTION_FUSION_AEB_OBJECTS_RESERVED = 2,
  };

    serdes3(available, perception_fusion_aeb_objects_data, reserved_infos)
};

/*!
 * \brief Trafficlight payload
 */
struct TrafficLightPerceptionResult {
  uint64_t cross_id; //!< itersection track id, from worldmodel
  std::vector<TrafficLightROI> traffic_light_roi; //!< trafficlights ROI
  std::vector<TrafficLightSingleFrame>
      traffic_light_single_frame; //!< Single frame detection result and single
                                  //!< frame classification result
  std::vector<TrafficLightMultiFrame>
      traffic_light_multi_frame; //!< Results after multi-frame
  std::vector<TrafficLightDecision>
      traffic_light_decision; //!< The final result used to make decisions for
                              //!< PNC
  serdes5(cross_id, traffic_light_roi,
          traffic_light_single_frame, traffic_light_multi_frame,
          traffic_light_decision)
};

/*!
 * \brief road mark perception struct
 */
struct RoadMarkPerception {
  maf_std::Header header; //!< TimeStamp to public this message
  PerceptionMeta
      meta; //!< TimeStamp according to perception meta.sensor_timestamp_us
  uint8_t available; //!< Availability of roadmark perception
  /*!
   * \brief Perception results of road marks
   * \vec_max_size{10000}
   */
  std::vector<RoadMark> road_marks;
  /*!
   * \brief reserved infos of road marks
   * \vec_max_size{10000}
   */
  std::vector<std::string> reserved_infos;

  enum : uint8_t {
    VISION_PERCEPTION_ROAD_MARK_RESULT =
        1, //!< road mark from vision perception
    VISION_PERCEPTION_ROAD_MARK_RESERVED_INFOS =
        2, //!< reserved infos of road mark
  };
};

/*!
 * \brief Vision only AEB perception
 */
struct VisionAEBPerception {
  maf_std::Header header;
  PerceptionMeta meta;
  std::vector<VisionAEBObject> vision_aeb_objects;
  float vision_min_ttc;
};

/*!
 * \brief ground line perception struct
 */
struct GroundLinePerception {
  maf_std::Header header; //!< TimeStamp to public this message
  PerceptionMeta
      meta; //!< TimeStamp according to perception meta.sensor_timestamp_us
  uint64_t available; //!< Availability of ground line perception
  /*!
   * \brief Perception results of ground lines
   * \vec_max_size{10000}
   */
  std::vector<GroundLine> ground_lines;
  /*!
   * \brief reserved infos of ground lines
   * \vec_max_size{10000}
   */
  std::vector<std::string> reserved_infos;

  enum : uint8_t {
    VISION_PERCEPTION_GROUND_LINE_RESULT =
        1, //!< ground lines from vision perception
    VISION_PERCEPTION_GROUND_LINE_RESERVED_INFOS =
        2, //!< reserved infos of ground lines
  };
};

struct Object {
  CameraSourceEnum camera_source;
  CameraModelEnum camera_model;
  ObjectTypeInfo type_info;
  float score;
  float visibility_2d;
  Rect4f bounding_box_2d;
  ObjectTrackInfo track_info;
  ObjectReIDFeature reid_feature;
  VehicleStateInfo vehicle_state_info;
  Shape3f shape_3d;
  float heading_theta_bv;
  Vector3f velocity_bv;
  Point3f location_bv;
  ObjectAEBFeature aeb_feature;
  float ttc;
  bool is_cipv;
  VehicleVPBInfo vehicle_vpb_info;
};

/*!
 * \brief parking slot perception struct
 */
struct ParkingSlotPerception {
  maf_std::Header header; //!< TimeStamp to public this message
  PerceptionMeta
      meta; //!< TimeStamp according to perception meta.sensor_timestamp_us
  uint64_t available; //!< Availability of parking slot perception
  /*!
   * \brief Perception results of parking slots
   * \vec_max_size{1000}
   */
  std::vector<ParkingSlot> parking_slots;
  /*!
   * \brief reserved infos of parking slots
   * \vec_max_size{1000}
   */
  std::vector<std::string> reserved_infos;

  enum : uint8_t {
    VISION_PERCEPTION_PARKING_SLOT_RESULT =
        1, //!< parking slots from vision perception
    VISION_PERCEPTION_PARKING_SLOT_RESERVED_INFOS =
        2, //!< reserved infos of parking slots
  };
};

struct ImageRetrievalDescriptorPerception {
  maf_std::Header header;
  PerceptionMeta meta;
  /*!
   * \brief data valid flag
   */
  uint64_t available;

  /*!
   * \brief image retrieval descriptor struct
   */
  ImageRetrievalDescriptor image_descriptor;

  enum : uint8_t {
    VISION_PERCEPTION_IMAGE_RETRIEVAL_DESCRIPTOR_RESULT = 1,
  };
};

/*!
 * \brief pole line perception struct
 */
struct PoleLinePerception {
  uint64_t available; //!< Availability of pole line perception
  /*!
   * \brief Perception results of pole lines
   * \vec_max_size{10000}
   */
  std::vector<PoleLine> pole_lines;
  /*!
   * \brief reserved infos of pole lines
   * \vec_max_size{10000}
   */
  std::vector<std::string> reserved_infos;

  enum : uint8_t {
    VISION_PERCEPTION_POLE_LINE_RESULT =
        1, //!< pole lines from vision perception
    VISION_PERCEPTION_POLE_LINE_RESERVED_INFOS =
        2, //!< reserved infos of pole lines
  };
};

/*!
 * \brief traffic sign perception struct
 */
struct TrafficSignPerception {
  uint64_t available; //!< Availability of traffic sign perception
  /*!
   * \brief Perception results of traffic signs
   * \vec_max_size{10000}
   */
  std::vector<TrafficSign> traffic_signs;
  /*!
   * \brief Reserved infos of traffic signs
   * \vec_max_size{10000}
   */
  std::vector<std::string> reserved_infos;

  enum : uint8_t {
    VISION_PERCEPTION_TRAFFIC_SIGN_RESULT =
        1, //!< traffic signs from vision perception
    VISION_PERCEPTION_TRAFFIC_SIGN_RESERVED_INFOS =
        2, //!< reserved infos of traffic signs
  };
};

struct ADASPerception {
  maf_std::Header header;
  PerceptionMeta meta;
  uint64_t available;
  ADASPerceptionResult adas_perception_result;

  enum : uint8_t {
    VISION_PERCEPTION_ADAS_PERCEPTION_RESULT = 1,
  };
};

/*!
 * \brief road line perception struct
 */
struct RoadLinePerception {
  maf_std::Header header; //!< TimeStamp to public this message
  PerceptionMeta
      meta; //!< TimeStamp according to perception meta.sensor_timestamp_us
  /*!
   * \brief lane line perception result
   */
  LanePerception lane_perception;
  /*!
   * \brief road edge perception result
   */
  RoadEdgePerception road_edge_perception;
};

struct LidarPerceptionObjects {
  uint8_t available;
  std::vector<LidarPerceptionObjectData> lidar_perception_object_data;

  enum : uint8_t {
    LIDAR_PERCEPTION_OBJECT_DATA = 1,
  };
};

/*!
 * \brief radar perception information
 */
struct RadarPerceptionResult {
  maf_std::Header header;         //!< header
  RadarPerceptionResultMeta meta; //!< metadata
  RadarPerceptionObjects
      radar_perception_objects; //!< radar_perception_objects(basic,bosch,conti)
};

/*!
 * \brief perception fusion result Msg
 */
struct PerceptionFusionObjectResult {
  maf_std::Header header; //!< TimeStamp to public this message
  PerceptionFusionResultMeta
      meta; //!< TimeStamp according to perception meta.sensor_timestamp_us
  /*!
   * \brief objects (vehicle, VRU, etc.) fusion result
   * \vec_max_size{1000000}
   */
  std::vector<PerceptionFusionObjectData> perception_fusion_objects_data;
  std::vector<std::string>
      reserved_infos; //!< reserved information to describe the objects. e.g.
                      //!< is_ignore=True/False

    serdes4(header, meta, perception_fusion_objects_data, reserved_infos)
};

/*!
 * \brief perception fusion AEB result
 */
struct PerceptionFusionAEBResult {
  maf_std::Header header; //!< TimeStamp to public this message
  PerceptionFusionResultMeta
      meta; //!< TimeStamp according to perception meta.sensor_timestamp_us
  PerceptionFusionAEBStateEnum
      perception_fusion_aeb_state; //!< State enum of AEB
  float min_ttc;                   //!< The min_ttc of all aeb objects
  PerceptionFusionAEBObjects
      perception_fusion_aeb_objects; //!< AEB objects of perception fusion

    serdes5(header, meta, perception_fusion_aeb_state,
            min_ttc, perception_fusion_aeb_objects)
};

/*!
 * \brief Trafficlight perception result
 */
struct TrafficLightPerception {
  maf_std::Header header; //!< Header
  PerceptionMeta meta;    //!< Meta
  uint64_t available;     //!< Header
  TrafficLightPerceptionResult
      traffic_light_perception_result; //!< Trafficlight real payload

  enum : uint8_t {
    VISION_PERCEPTION_TRAFFIC_LIGHT_RESULT = 1,
  };

  serdes4(header,meta,available,traffic_light_perception_result)
};

/*!
 * \brief General feature perception struct
 * \note
 * * ImageRetrievalDescriptorPerception: image descriptor, used for global image
 * retrieval
 * * FeaturePointPerception: visual feature point information
 */
struct GeneralFeaturePerception {
  maf_std::Header header;
  PerceptionMeta meta;
  ImageRetrievalDescriptorPerception image_retrieval_descriptor_perception;
  FeaturePointPerception feature_point_perception;
};

/*!
 * \brief Vision object perception
 */
struct ObjectPerception {
  maf_std::Header header; //!< TimeStamp to public this message
  PerceptionMeta
      meta; //!< TimeStamp according to perception meta.sensor_timestamp_us
  uint64_t available; //!< Available or not. 0: not available, 1: available
  std::vector<Object> objects;             //!< All objects
  std::vector<std::string> reserved_infos; //!< Reserved info

  enum : uint8_t {
    VISION_PERCEPTION_OBJECT_RESULT = 1,
    VISION_PERCEPTION_OBJECT_RESERVED = 2,
  };
};

/*!
 * \brief Vision mloc perception
 */
struct MLocPerception {
  maf_std::Header header; //!< TimeStamp to public this message
  PerceptionMeta
      meta; //!< TimeStamp according to perception meta.sensor_timestamp_us
  /*!
   * \brief lane segment perception result
   */
  LaneSegmentPerception lane_segment_perception;
  /*!
   * \brief pole line perception result
   */
  PoleLinePerception pole_line_perception;
  /*!
   * \brief traffic sign perception result
   */
  TrafficSignPerception traffic_sign_perception;
};

struct LidarPerceptionResult {
  maf_std::Header header;
  LidarPerceptionResultMeta meta;
  LidarPerceptionObjects lidar_perception_objects;
};

} // namespace maf_perception_interface

#endif

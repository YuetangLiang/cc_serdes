#ifndef MAF_INTERFACE_MAF_WORLDMODEL_H
#define MAF_INTERFACE_MAF_WORLDMODEL_H

#include "maf_interface/maf_perception_interface.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_worldmodel {

/*!
 * \brief  The orientation of the lane merging of splitting point
 */
struct Orientation {
  uint8_t value; //!< Different Type of Orientation

  enum : uint8_t {
    UNKNOWN = 0,   //!< Unknown Orientation
    LEFT = 1,      //!< Left Orientation
    RIGHT = 2,     //!< Right Orientation
    INTERSECT = 3, //!< Intersect Orientation
    IGNORED = 4,   //!< Ignored Orientation
  };
  serdes1(value)
};

/*!
 * \brief Information on parking slots that have been traversed
 */
struct TraversedParkingLot {
  uint16_t parking_lot_id; //!< ID of the traversed parking slot
  maf_perception_interface::Polygon3f
      parking_lot_position; //!< Corners of the traversed parking slot, the
                            //!< first and last point is the line close to road
  maf_perception_interface::Point3f
      parking_lot_center; //!< Center point of the traversed parking slot
  maf_perception_interface::Point3f
      parking_lot_entrance_center; //!< Entrance center point of the traversed
                                   //!< parking slot
  double available_inner_width;  //!< Inner width of the traversed parking slot
  double available_outter_width; //!< Outter width of the traversed parking slot
  uint16_t next_parking_lot_id;  //!< ID of the next traversed parking slot
  uint16_t
      previous_parking_lot_id; //!< ID of the previous traversed parking slot
  double left_extended_distance;
  double right_extended_distance;
  double score;
  serdes11(parking_lot_id, parking_lot_position,
           parking_lot_center, parking_lot_entrance_center,
           available_inner_width, available_outter_width,
           next_parking_lot_id, previous_parking_lot_id,
           left_extended_distance, right_extended_distance,
           score)

};

/*!
 * \brief Information to describe parking out process
 */
struct ParkingOutPosition {
  uint16_t source_parking_lot_id; //!< ID of the parking slot in which we begin
                                  //!< to park out
  maf_perception_interface::Polygon3f
      source_parking_lot_position; //!< Corners of the parking slot in which we
                                   //!< begin to park out
  maf_perception_interface::Point3f
      target_position;       //!< Destination point where we park to
  double target_heading_yaw; //!< Destination yaw where we park to
  serdes4(source_parking_lot_id, source_parking_lot_position,
          target_position, target_heading_yaw)
};

/*!
 * \brief Second-order Matrix composed of Four float
 */
struct Matrix2f {
  float x00;
  float x01;
  float x10;
  float x11;
  serdes4(x00,x01,x10,x11)
};

/*!
 * \brief The interval of lane change
 */

struct Interval {
  /**
   * @brief Begin of Interval
   * \unit{m}
   * \value_min{0}
   */
  double begin;
  /**
   * @brief End of Interval
   * \unit{m}
   * \value_min{0}
   */
  double end;
  serdes2(begin,end)
};

/*!
 * \brief the direction of lane
 */
struct Direction {
  uint8_t value;

  enum : uint8_t {
    UNKNOWN = 0,     //!< Unknown Direction
    GO_STRAIGHT = 1, //!< Go Straight
    TURN_RIGHT = 2,  //!< Turning Right
    TURN_LEFT = 4,   //!< Turning Left
    U_TURN_LEFT = 8, //!< U Turn Left, Not applied in Right-hand Driving Region
    U_TURN_RIGHT =
        16,         //!< U Turn Right, Not applied in Left-hand Driving Region
    OFF_ROUTE = 32, // when the successor lanes all offroute in certain distance
  };
  serdes1(value)
};

/*!
 * \brief Points to describe a polyline
 */
struct Polyline3f {
  std::vector<maf_perception_interface::Point3f> points;
  serdes1(points)
};

/*!
 * \brief The type of Map POI
 */
struct MapPOIType {
  uint8_t value;
  enum : uint8_t {
    UNKNOWN = 0,                    //!< Unknown Map POI
    PARKING_LOT = 1,                //!< Parking lot of Map POI
    HUMAN_ACCESS = 2,               //!< Human Access of Map POI
    DESTINATION = 3,                //!< Destination of Map POI
    PARKING = 4,                    //!< Parking of Map POI
    BARRIER_GAP = 5,                //!< Barrier Gap of Map POI
    FACILITY_ENTRANCE = 6,          //!< Facility Entrance of Map POI
    FACILITY_EXIT = 7,              //!< Facility Exit of Map POI
    FACILITY_EXIT_AND_ENTRANCE = 8, //!< Facility Exit & Entrance of Map POI
    BUS_STOP = 9,                   //!< Bus Stop of Map POI
    GARAGE_ENTRANCE = 10,           //!< Garage Entrance of Map POI
    GARAGE_EXIT = 11,               //!< Garage Exit of Map POI
    SPEED_BUMP = 12,                //!< Speed Bump of Map POI
    CROSS_WALK = 13,                //!< Cross Walk of Map POI
    DASHED_SEGMENT = 14,            //!< Dashed Segment of Map POI
    CENTRAL_CIRCLE = 15,            //!< Central Circle of Map POI
    NO_PARKING_ZONE = 16,           //!< No Parking Zone of Map POI
    ROAD_MERGE = 17,                //!< Road Merge of Map POI
    ROAD_SPLIT = 18,                //!< Road Split of Map POI
    TUNNEL = 19,                    //!< Tunnel of Map POI
    TOLL_STATION = 20,              //!< Toll Station of Map POI
  };
  serdes1(value)
};

/*!
 * \brief Information to describe one traffic light
 */
struct MapTrafficLight {
  std::vector<maf_perception_interface::Point3d>
      boundary; //!< Boundary of traffic light
  /*!
   * \brief 3D boundary points in CAR axis
   * \vec_fix_size{4}
   */
  maf_perception_interface::TrafficLightPatternEnum
      pattern;   //!< Traffic Light Pattern
  bool on_route; //!< whether this traffic light on route

  serdes3(boundary, pattern, on_route)
};

/*!
 * \brief Information to describe left waiting area
 */
struct LeftWaitingArea {
  bool existence; //!< whether left waiting area exists
  /**
   * @brief length of left waiting area
   * \unit{m}
   * \value_min{0}
   */
  double length;
  /**
   * @brief distance of left waiting area
   * \unit{m}
   * \value_min{0}
   */
  double distance;

  serdes3(existence, length, distance)
};

struct LaneChangeState {
  uint8_t value; //!< Different Type Lane Change Status

  enum : uint8_t {
    FOLLOW = 0,     //!< Lane Change Status of Follow
    TURN_LEFT = 1,  //!< Lane Change Status of Turning Left
    TURN_RIGHT = 2, //!< Lane Change Status of Turning Right
  };
  serdes1(value)
};

/*!
 * \brief Information to describe the fusion object area
 */
struct ObjectWorldModelData {
  bool filtered; //!< Whether Filtered
  std::vector<maf_perception_interface::Point3f>
      occlusion_area;     //!< Occlusion Area
  std::string extra_json; //!< Extra json info
  serdes3(filtered, occlusion_area, extra_json)
};

/*!
 * \brief ObjectsInterface Meta Info
 */
struct ObjectsInterfaceMeta {
  uint64_t sensor_timestamp_us;          //!< Time Stamp of Sensor Msg
  uint64_t pipeline_start_timestamp_us;  //!< Time Stamp of Pipeline Start
  uint64_t pipeline_finish_timestamp_us; //!< Time Stamp of Pipeline End
  std::string frame_id;
  serdes4(sensor_timestamp_us, pipeline_start_timestamp_us,
          pipeline_finish_timestamp_us,frame_id)
};

struct FusionAPAMeta {
  uint64_t timestamp_us;
  uint64_t pipeline_start_timestamp_us;
  uint64_t pipeline_finish_timestamp_us;
};

/*!
 * \brief Information to describe parking slot status in APA mode
 */
struct APAInfo {
  uint32_t status;                //!< Status of the parking slot
  int32_t map_id;                 //!< Global Map ID of the parking slot
  uint64_t last_update_timestamp; //!< Time Stamp of Last Update

  enum : uint32_t {
    UNKNOWN = 0,
    OCCUPIED = 1,
    VACANT = 2,
    APA_FAIL = 3,
    APA_SUCCEED = 4,
    REVERSED = 5,
  };
};

/*!
 * \brief Type of Intention
 */
struct IntentionType {
  int32_t value;

  enum : int32_t {
    INVALID = -1,              //!< Invalid Intention Type
    FREE_MOVE = 0,             //!< Free Move Intention Type
    KEEP_LANE = 1,             //!< Keep Lane Intention Type
    LEFT_CHANGE_LANE_OUT = 2,  //!< Left Change Lane Out Intention Type
    LEFT_CHANGE_LANE_IN = 3,   //!< Left Change Lane In Intention Type
    RIGHT_CHANGE_LANE_OUT = 4, //!< Right Change Lane Out Intention Type
    RIGHT_CHANGE_LANE_IN = 5,  //!< Right Change Lane In Intention Type
    GO_STRAIGHT = 11,          //!< Go Straight Intention Type
    TURN_LEFT = 12,            //!< Turn Left Intention Type
    TURN_RIGHT = 13,           //!< Turn Right Intention Type
    U_TURN = 14,               //!< U Turn Intention Type
  };
  serdes1(value)
};

struct PredictionResultMeta {
  uint64_t sensor_timestamp_us;
  uint64_t pipeline_start_timestamp_us;
  uint64_t pipeline_finish_timestamp_us;
  std::string frame_id;
  serdes4(sensor_timestamp_us, pipeline_start_timestamp_us,
          pipeline_finish_timestamp_us, frame_id)
};

struct SceneObjectsMeta {
  uint64_t timestamp_us;
  uint64_t pipeline_start_timestamp_us;
  uint64_t pipeline_finish_timestamp_us;
};

/*!
 * \brief Type of SceneObject
 */
struct SceneObjectType {
  uint8_t value;

  enum : uint8_t {
    UNKNOWN = 0,                   //!< Unknown Scene Object
    TRAFFIC_BARRIER_TYPE_CONE = 1, //!< Traffic Barrier of Cone
    TRAFFIC_BARRIER_TYPE_WARNING_TRIANGLE =
        2, //!< Traffic Barrier of Warning Triangle
    TRAFFIC_BARRIER_TYPE_BOLLARD_SLEEVE =
        3, //!< Traffic Barrier of Bollard Sleeve
    TRAFFIC_BARRIER_TYPE_PARKING_LOCK = 4, //!< Traffic Barrier of Parking Lock
    TRAFFIC_BARRIER_TYPE_AFRAME_SIGN = 5,  //!< Traffic Barrier of Aframe Sign
    TRAFFIC_BARRIER_TYPE_ANTICRASH_BUCKET =
        6,                            //!< Traffic Barrier of Anti-Crash Bucket
    TRAFFIC_BARRIER_TYPE_DUSTBIN = 7, //!< Traffic Barrier of Dustbin
    TRAFFIC_BARRIER_TYPE_BARRIER_STONE =
        8, //!< Traffic Barrier of Stone Barrier
    TRAFFIC_BARRIER_TYPE_WATER_BARRIER =
        9,                      //!< Traffic Barrier of Water Barrier
    MAP_FIRE_EXIT_DOOR = 10,    //!< Fire Exit Door
    MAP_PILLAR = 11,            //!< Pillar
    MAP_TOLL_GATE = 12,         //!< Toll Gate
    MAP_BORDER_SOLID_LINE = 13, //!< Border of Solid Line
    MAP_BORDER_DASH_LINE = 14,  //!< Border of Dashed Line
    MAP_BORDER_PHYSICAL = 15,   //!< Physical Border
  };
};

/*!
 * \brief information to describe the nearest ramp
 */
struct RampManeuver {
  float dis_to_ramp; //!< distance to the nearest ramp
  float length;      //!< the length of the nearest ramp
  serdes2(dis_to_ramp, length)
};

/*!
 * \brief Map Meta Info
 */
struct ProcessedMapMeta {
  uint64_t egopose_timestamp_us; //!< timestamp when the map message trigger
  uint64_t pipeline_start_timestamp_us;  //!< timestamp when the algorithm
                                         //! pipeline start
  uint64_t pipeline_finish_timestamp_us; //!< timestamp when the algorithm
                                         //! pipeline end
  serdes3(egopose_timestamp_us, pipeline_start_timestamp_us, pipeline_finish_timestamp_us)
};

/*!
 * \brief Current Car Position Data indicating current car status
 */
struct SelfPositionData {
  bool in_map_area;     //!< whether current car is in map area
  bool in_intersection; //!< whether current car is in intersection area
  bool on_ramp;         //!< whether current car is on ramp
  int32_t current_parking_slot_id;

  serdes4(in_map_area, in_intersection, on_ramp, current_parking_slot_id)
};

/*!
 * \brief Way Point type
 */
struct WayPointType {
  uint8_t value;
  enum : uint8_t {
    NORMAL_WAYPOINT = 1,       //!< Way Point after certain distance
    INTERSECTION_WAYPOINT = 2, //!< Way Point after certain intersection
    RAMP_WAYPOINT = 4,         //!< Way Point after a ramp
  };
  serdes1(value)
};

/*!
 * \brief Way Point Position for rerouting
 */
struct WayPointData {
  WayPointType way_point_type; //!< different type of way point
  double longtitude;           //!< longtitude of the way point
  double latitude;             //!< latitude of the way point
  double altitude;             //!< altitude of the way point
  serdes4(way_point_type, longtitude, latitude, altitude)
};

/*!
 * \brief Type of Lane
 */
struct LaneType {
  uint8_t value;
  enum : uint8_t {
    UNKNOWN = 0,                 //!< Unknown Lane Type
    NORMAL = 1,                  //!< Normal Lane Type
    VIRTUAL = 2,                 //!< Virtual Lane Type
    PARKING = 3,                 //!< Parking Lane Type
    ACCELERATE = 4,              //!< Accelerate Lane Type
    DECELERATE = 5,              //!< Decelerate Lane TYpe
    BUS = 6,                     //!< Bus Lane Type
    EMERGENCY = 7,               //!< Emergency Lane Type
    ACCELERATE_DECELERATE = 8,   //!< Accelerate and Decelerate Lane Type
    LEFT_TURN_WAITTING_AREA = 9, //!< Left Turning Waiting Area
    NON_MOTOR = 10,              //!< Non-Motor Lane Type
  };
  serdes1(value)
};

/*!
 * \brief Different Connection Direction between two connected lanes
 */
struct LaneConnectDirection {
  uint8_t value;

  enum : uint8_t {
    NORMAL = 0,           //!< Normal Connection
    SPLIT_TO_LEFT = 1,    //!< Split to left
    SPLIT_TO_RIGHT = 2,   //!< Split to right
    MERGE_FROM_LEFT = 3,  //!< Merge from left
    MERGE_FROM_RIGHT = 4, //!< Merge from right
  };
  serdes1(value)
};

/*!
 * \brief Form of Lane Boundary Type
 */
struct LaneBoundaryForm {
  uint8_t value;
  enum : uint8_t {
    UNKNOWN = 0,      //!< Unknown Lane Boundary Type
    SOLID = 1,        //!< Solid Lane Boundary
    DASH = 2,         //!< Dashed Lane Boundary
    VIRTUAL = 3,      //!< Virtual Lane Boundary
    PHYSICAL = 4,     //!< Physical Lane Boundary
    DOUBLE_SOLID = 5, //!< Double Solid Lane Boundary
    DASH_SOLID =
        6, //!< Dash and Solid Lane Boundary, can be crossed by this side
    SOLID_DASH =
        7, //!< Dash and Solid Lane Boundary, cant be crossed by this side
  };
  serdes1(value)
};

/*!
 * \brief Form of Lane Boundary Color
 */
struct LaneBoundaryColor {
  uint8_t value;
  enum : uint8_t {
    WHITE = 0,  //!< while color
    YELLOW = 1, //!< yellow color
    OTHER = 2,  //!< other color
  };
  serdes1(value)
};

/*!
 * \brief Type of Lane Boundary Type
 */
struct LaneBoundaryType {
  LaneBoundaryForm value;  //!< Form of Lane Boundary
  LaneBoundaryColor color; //!< Color of Lane Boundary Type
  serdes2(value,color)
};

/*!
 * \brief Merge Type of road
 */
struct MergeType {
  uint8_t value;

  enum : uint8_t {
    UNKNOWN = 0,
    UNRELATED = 1,
    FROM_RIGHT = 2,
    FROM_LEFT = 3,
    FROM_BOTH = 4,
    LANE_END = 5,
  };
  serdes1(value)
};

/*!
 * \brief Information about Road Edge
 */
struct RoadEdge {
  bool existence; //!< whether road edge exists
  std::vector<maf_perception_interface::Point3f>
      points;           //! vector of point on road edge
  uint8_t type;         //! Type of road edge
  std::string reserved; //!< reserved information

  serdes4(existence, points, type, reserved)
};

/*!
 * \brief Information of a lane merging/splitting point
 */
struct LaneMergingSplittingPointData {
  /**
   * @brief current Distance to the point
   * \unit{m}
   * \value_min{0}
   */
  double distance;
  bool is_split;           //!< when this point is splitting point
  bool is_continue;        //!< when this point is merging point
  Orientation orientation; //!< orientation of this point
  /**
   * @brief length of merging/splitting maneuver,normally means the lengths
   * between the point that two lanes start to split/merge and the point that
   * two lanes finish \unit{m} \value_min{0}
   */
  double length;

  serdes5(distance, is_split, is_continue, orientation, length)
};

/*!
 * \brief Information to describe Aimed POI
 */
struct TargetMapPOI {
  uint16_t id;                                  //!< ID of Aimed POI
  MapPOIType type;                              //!< Type of Aimed POI
  maf_perception_interface::Polygon3f position; //!< Corners of Aimed POI
  double distance;                              //!< Distance to Aimed POI
  serdes4(id,type,position,distance)
};

/*!
 * \brief Point of Interest obtained from HDMAP data
 */
struct MapPOIInfoData {
  /**
   * @brief Distance to the MAP POI
   * \unit{m}
   * \value_min{0}
   */
  double distance;
  /**
   * @brief Length of the MAP POI
   * \unit{m}
   * \value_min{0}
   */
  double length;
  MapPOIType type; //!< Type of the MAP POI
  std::vector<maf_perception_interface::Point3f>
      key_points_enu; //!< Key point of MAP POI in ENU axis

  serdes4(distance, length, type, key_points_enu)
};

/*!
 * \brief Traffic Light Group in an Intersection
 */
struct MapTrafficLightGroup {
  uint64_t track_id; //!< Track Id of the traffic light group
  std::vector<MapTrafficLight> traffic_lights; //!< Groups of traffic lights

  serdes2(track_id, traffic_lights)
};

/*!
 * \brief Vector of Lane Changing Strategy
 */
struct LaneStrategyData {
  std::vector<LaneChangeState>
      strategy_start_with_current_lane; //!< Lane Changing Strategy of current
                                        //!< lane
  std::vector<LaneChangeState>
      strategy_start_with_left_lane; //!< Lane Changing Strategy of left lane
  std::vector<LaneChangeState>
      strategy_start_with_right_lane; //!< Lane Changing Strategy of right lane
  std::vector<Interval>
      current_lane_change_available_interval; //!< Lane Changeable Interval of
                                              //!< current lane
  std::vector<Interval>
      left_lane_change_available_interval; //!<  Lane Changeable Interval of
                                           //!<  left lane
  std::vector<Interval>
      right_lane_change_available_interval; //!<  Lane Changeable Interval of
                                            //!<  right lane
  serdes6(strategy_start_with_current_lane, strategy_start_with_left_lane,
          strategy_start_with_right_lane, current_lane_change_available_interval,
          left_lane_change_available_interval, right_lane_change_available_interval)
};

/*!
 * \brief Information to describe a fusion object
 */
struct ObjectInterface {
  maf_perception_interface::PerceptionFusionObjectData
      object_fusion_data;                       //!< fusion object data
  ObjectWorldModelData object_world_model_data; //!< occlusion area
  bool is_ignore; //!< whether to ignore the fusion object
  serdes3(object_fusion_data, object_world_model_data, is_ignore)
};

/*!
 * \brief Information to describe a parking slot in FusionAPA
 */
struct ParkingSlotFusionAPAData {
  uint8_t available;
  maf_perception_interface::FusionParkingSlotData
      parking_slot; //!< Parking Slot information in PSD Fusion
  APAInfo apa_info; //!< Parking Slot information in APA, associated with map

  enum : uint8_t {
    PARKING_SLOT = 1,
    APA_INFO = 2,
  };
};

/*!
 * \brief Information of the points on prediction trajectories
 */
struct PredictionTrajectoryPoint {
  maf_perception_interface::Point2f position; //!< the location of the point
  /**
   * @brief The current vehicle speed direction
   * \unit{rad}
   * \value_min{-pi}
   * \value_max{pi}
   */
  float yaw;
  /**
   * @brief The current vehicle heading direction
   * \unit{rad}
   * \value_min{-pi}
   * \value_max{pi}
   */
  float theta;
  /**
   * @brief The velocity on this point
   * \unit{m/s}
   * \value_min{0}
   */
  float velocity;
  float confidence;       //!< the confidence of this point
  Matrix2f covariance_xy; //!< the two-dimensional sigma of this point
  serdes6(position, yaw, theta,
          velocity, confidence, covariance_xy)
};

struct SceneCircleObject {
  uint64_t object_id;
  SceneObjectType object_type;
  maf_perception_interface::Point2d center_point;
  double radius;
  double height;
};

/*!
 * \brief Information to describe SceneObject
 */
struct ScenePolygonObject {
  uint64_t object_id;          //!< ID of ScenePolygonObject
  SceneObjectType object_type; //!< Type of ScenePolygonObject
  maf_perception_interface::Polygon3f
      object_polygon; //!< Corners of ScenePolygonObject
};

/*!
 * \brief Information to describe ExtraInfo
 */
struct ExtraInfo {
  uint8_t available;          //!< Available Bit
  RampManeuver ramp_maneuver; //!< information to describe nearest ramp_maneuver
  std::string
      scene_type; //!< information to describe current scene highway or urban
  std::string reserved_info; //!< reserved field

  enum : uint8_t {
    RAMP_MANEUVER = 1, //!< when ramp_maneuver filed  is available
    SCENE_TYPE = 2,    //!< when scene_type filed  is available
    RESERVED_INFO = 4, //!< when reserved_info filed  is available
  };

  serdes4(available, ramp_maneuver, scene_type, reserved_info)
};

/*!
 * \brief Information of a reference line segment
 */
struct ReferenceLineSegment {
  uint16_t
      begin_index;    //!< starting point of this segment on this reference line
  uint16_t end_index; //!< ending point of this segment on this reference line
  LaneConnectDirection direction; //!< Lane Direction of this reference segment
  LaneType lane_type;             //!< Lane Type of this reference segment
  bool is_in_intersection;        //!< whether this lane segment in intersection
  bool is_in_route;               //!< whether this lane segment on route

  serdes6(begin_index, end_index, direction,
          lane_type, is_in_intersection, is_in_route)
};

/*!
 * \brief Information of a lane boundary segment
 */
struct LaneBoundarySegment {
  /**
   * @brief length of the lane boundary
   * \unit{m}
   * \value_min{0}
   */
  double length;         //!< length of the lane boundary
  LaneBoundaryType type; //!< type of the lane boundary

  serdes2(length, type)
};

/*!
 * \brief Lane Boundary Polyline Information
 */
struct LaneBoundaryPolyline {
  uint64_t track_id;                    //!< Identification of Lane Boundary
  std::vector<double> poly_coefficient; //!< polyline coefficient
                                        /*!
                                         * \vec_fix_size{4}
                                         */
  Interval available_interval;          //!< Interval of Lane Boundary

  serdes3(track_id, poly_coefficient, available_interval)
};

/*!
 * \brief Information about Lane Merging Point in front
 */
struct MergePoint {
  /**
   * @brief Distance to current car position
   * \unit{m}
   * \value_min{0}
   */
  double distance;
  MergeType type; //!< Merge Type of the Merge Point
  serdes2(distance, type)
};

/*!
 * \brief Information to describe the point in reference line
 */
struct ReferenceLinePoint {
  maf_perception_interface::Point3f
      enu_point; //!< Point on Reference Line in ENU axis
  /**
   * @brief Curvature of the reference line on this point
   * \unit{rad}
   * \value_min{0}
   */
  float curvature;
  /**
   * @brief distance between reference line point and left road border
   * \unit{m}
   * \value_min{0}
   */
  float distance_to_left_road_border;
  /**
   * @brief distance between reference line point and right road border
   * \unit{m}
   * \value_min{0}
   */
  float distance_to_right_road_border;
  /**
   * @brief distance between reference line point and left lane border
   * \unit{m}
   * \value_min{0}
   */
  float distance_to_left_lane_border;
  /**
   * @brief distance between reference line point and right lane border
   * \unit{m}
   * \value_min{0}
   */
  float distance_to_right_lane_border;
  /**
   * @brief distance between reference line point and left physical obstacle
   * \unit{m}
   * \value_min{0}
   */
  float distance_to_left_obstacle;
  /**
   * @brief distance between reference line point and right physical obstacle
   * \unit{m}
   * \value_min{0}
   */
  float distance_to_right_obstacle;
  /**
   * @brief lane width, equals distance_to_left_lane_border +
   * distance_to_right_lane_border if both exist \unit{m} \value_min{0}
   */
  float lane_width;
  /**
   * @brief speed limit
   * \unit{km/h}
   * \value_min{0}
   */
  int32_t max_velocity;    //!< speed limit
  uint64_t track_id;       //!< track id for this point
  bool is_experience_data; //!< It tells whether the ReferenceLinePoint is
                           //!< generated from experience data
  LaneBoundaryType left_road_border_type;  //!< left road border type
  LaneBoundaryType right_road_border_type; //!< right road border type
  bool on_route;                           //!< whether this point on route
  double longitudinal_slope;
  /**
   * @brief slope in longtitude direction
   * \unit{rad}
   * \value_min{-1.57}
   * \value_max{ 1.57}
   */
  double horizontal_slope;
  /**
   * @brief slope in horizontal direction
   * \unit{rad}
   * \value_min{-1.57}
   * \value_max{ 1.57}
   */

  serdes17(enu_point, curvature, distance_to_left_road_border, distance_to_right_road_border, distance_to_left_lane_border,
           distance_to_right_lane_border, distance_to_left_obstacle, distance_to_right_obstacle, lane_width, max_velocity,
           track_id, is_experience_data, left_road_border_type, right_road_border_type, on_route,
           longitudinal_slope, horizontal_slope)
};

/*!
 * \brief Information to describe the different targets in parking
 */
struct ParkingTargetPosition {
  uint64_t available;
  TargetMapPOI target_map_poi; //!< Aimed POI
  std::vector<TraversedParkingLot>
      traversed_parking_lots;              //!< Traversed parking slots
  ParkingOutPosition parking_out_position; //!< Parking out position

  enum : uint64_t {
    TARGET_MAP_POI = 1,
    TRAVERSED_PARKING_LOTS = 2,
    PARKING_OUT_POSITION = 4,
  };
  serdes4(available, target_map_poi, traversed_parking_lots, parking_out_position)
};

/*!
 * \brief Lane Change Strategy according to Lane path
 */
struct LaneStrategy {
  uint8_t available;                   //!< Available Bits
  LaneStrategyData lane_strategy_data; //!< Lane Strategy of the lane

  enum : uint8_t {
    LANE_STRATEGY_DATA = 1, //!< when lane strategy is available
  };
};

/*!
 * \brief post-processed fusion data sent by Worldmodel Module
 */
struct ObjectsInterface {
  maf_std::Header header;
  ObjectsInterfaceMeta meta;                     //!< Meta information
  std::vector<ObjectInterface> object_interface; //!< Array of objects
  serdes3(header, meta, object_interface)
};

/*!
 * \brief Environment used in parking in&out
 */
struct FusionAPA {
  maf_std::Header header;
  FusionAPAMeta meta;
  uint8_t available;
  std::vector<ParkingSlotFusionAPAData> parking_slots; //!< Parking slots
  int32_t ego_parking_slot_track_id; //!< Track ID of parking slot in which self
                                     //!< car is
  int32_t
      ego_parking_slot_map_id; //!< Map ID of parking slot in which self car is
  int32_t
      suggested_parking_slot_track_id;   //!< Track ID of suggested parking slot
  int32_t suggested_parking_slot_map_id; //!< Map ID of suggested parking slot
  std::vector<std::string> reserved_info; // array of reversed_info

  enum : uint8_t {
    PARKING_SLOTS = 1,
    EGO_PARKING_SLOT_TRACK_ID = 2,
    EGO_PARKING_SLOT_MAP_ID = 4,
    SUGGESTED_PARKING_SLOT_TRACK_ID = 8,
    SUGGESTED_PARKING_SLOT_MAP_ID = 16,
    RESERVED_INFO = 32,
  };
};

/*!
 * \brief Information of the prediction trajectories
 */
struct PredictionTrajectory {
  /**
   * @brief The confidence of this trajectory
   * \value_min{0}
   * \value_max{1}
   */
  float confidence;
  /**
   * @brief The time interval between points of the prediction trajectory
   * \unit{s}
   * \value_min{0}
   */
  float prediction_interval;
  IntentionType intention; //!< The intention of this prediction trajectory
  std::vector<PredictionTrajectoryPoint>
      trajectory_points;  //!< The points of this prediction trajectory
  std::string extra_json; //!< Extra json
  serdes5(confidence, prediction_interval,
          intention, trajectory_points, 
          extra_json)
};

struct SceneObjects {
  maf_std::Header header;
  SceneObjectsMeta meta;
  uint8_t available;
  std::vector<ScenePolygonObject>
      on_path_polygon_objects; //!< SceneObjects on route
  std::vector<ScenePolygonObject>
      surround_polygon_objects; //!< SceneObjects nearby
  std::vector<SceneCircleObject>
      on_path_circle_objects; //!< SceneObjects on route
  std::vector<SceneCircleObject>
      surround_circle_objects; //!< SceneObjects nearby

  enum : uint8_t {
    ON_PATH_POLYGON_OBJECTS = 1,
    SURROUND_POLYGON_OBJECTS = 2,
    ON_PATH_CIRCLE_OBJECTS = 4,
    SURROUND_CIRCLE_OBJECTS = 8,
  };
};

/*!
 * \brief Information to Describe a reference line
 */
struct ReferenceLine {
  bool available; //!< whether exists
  std::vector<ReferenceLinePoint>
      reference_line_points; //!< points on the reference line
  std::vector<ReferenceLineSegment>
      reference_line_segments; //!< Segments of reference line

  serdes3(available, reference_line_points, reference_line_segments)
};

/*!
 * \brief Information to describe lane boundary
 */
struct LaneBoundary {
  bool existence;                            //!< whether exists
  std::vector<LaneBoundarySegment> segments; //!< Segments of Lane Boundary
  LaneBoundaryPolyline polyline;             //!< Fitted Polyline
  std::vector<maf_perception_interface::Point3f>
      points;           //!< Points on the Lane Boundary
  std::string reserved; //!< Reserved Information

  serdes5(existence, segments, polyline, points, reserved)
};

/*!
 * \brief Information on parking target
 */
struct TargetPosition {
  uint8_t available;
  ParkingTargetPosition parking_target_position;

  enum : uint8_t {
    PARKING_TARGET_POSITION = 1,
  };
  serdes2(available, parking_target_position)
};
/*!
 * \brief Object after Prediction, not applied in World Model Module
 */
struct ObjectPredictionData {
  uint64_t object_id;
  std::vector<PredictionTrajectory>
      trajectories;       //!< Predicted Trajectory of objects
  std::string extra_json; //!< Extra json
  serdes3(object_id, trajectories, extra_json)
};

/*!
 * \brief Information to describe a lane
 */
struct LaneData {
  int32_t relative_id; //!< Relative Count to Current Lane, minus for left, plus
                       //!< for right
  int32_t track_id;
  Direction lane_marks;             //!< Direction of the lane
  LaneType lane_type;               //!< Type of the lane
  LaneBoundary left_lane_boundary;  //!< Left Lane Boundary of the lane
  LaneBoundary right_lane_boundary; //!< Right Lane Boundary of the lane
  RoadEdge left_road_edge;          //!< Left Road Edge of the lane
  RoadEdge right_road_edge;         //!< Right Road Edge of the lane
  ReferenceLine reference_line;     //!< Reference Line of the lane
  /**
   * @brief  entrance width
   * \unit{m}
   * \value_min{0}
   */
  double entrance_width;
  MergePoint merge_point; //!< Merge Point of the lane
  MergePoint y_point;     //!< Y point of the lane
  int8_t bias;
  
  serdes13(relative_id, track_id, lane_marks, lane_type, 
           left_lane_boundary, right_lane_boundary, left_road_edge, right_road_edge, 
           reference_line, entrance_width, merge_point, y_point, 
           bias)
};

/*!
 * \brief Information to Describe an Intersection
 */
struct IntersectionData {
  Direction direction; //!< Car's Intention Direction in cur
  /**
   * @brief  Length of the intersection
   * \unit{m}
   * \value_min{0}
   */
  double length;
  std::vector<MapTrafficLightGroup> traffic_light_groups;
  LeftWaitingArea left_waiting_area; //!< Information of Left Waiting area in
                                     //!< this intersection
  /**
   * @brief current distance to the stop line of the intersection
   * \unit{m}
   * \value_min{0}
   */
  double distance_to_stop_line;
  std::vector<LaneData> lanes_out; //! Lanes out in the intersection

  serdes6(direction, length, traffic_light_groups, 
          left_waiting_area, distance_to_stop_line, lanes_out)
};

struct PredictionResult {
  maf_std::Header header;
  PredictionResultMeta meta;
  std::vector<ObjectPredictionData>
      object_prediction_data; //!< array of prediction data
  serdes3(header, meta, object_prediction_data)
};

/*!
 * \brief Surrounding Environmental Information passed by map manager
 */
struct ProcessedMapData {
  uint16_t available; //!< Available Bits
  TargetPosition target_position;
  SelfPositionData self_position;  //!<  Self Position Data
  WayPointData way_point_position; //!< Way Point Position Data
  std::vector<LaneData> lanes;     //!<  Lanes Data Generated around car
  /*!
   * \vec_max_size{5}
   */
  std::vector<MapPOIInfoData> map_poi_info;    //!< Map POI Data
  std::vector<IntersectionData> intersections; //!< Intersections Data
  /*!
   *\brief intersections[0] is the intersection before (or the current one if in
   *intersection), insection[1] is the one after, intersection[2] is the one
   *after intesection [1]. \vec_fix_size{3}
   */
  std::vector<LaneMergingSplittingPointData>
      lane_merging_splitting_points; //!< Lane Merging or Splitting Points Data
  LaneStrategyData lane_strategy;    //!< Lane Changing Strategy Data
  ExtraInfo extra_info;              //!< Extra Info Data

  enum : uint16_t {
    TARGET_POTISION = 1,   //!< when Target Position Data is available
    SELF_POSITION = 2,     //!< when Self Position Data is available
    WAYPOINT_POSITION = 4, //!< when Way Point Data is available
    LANE = 8,              //!<  when Lane Data is available
    MAP_POI_INFO = 16,     //!<  when MAP POI Data is available
    INTERSECTION = 32,     //!<  when Intersection Data is available
    LANE_MERGING_SPLITTING_POINT =
        64, //!<  when Lane Merging Splitting Point Data is available
    LANE_STRATEGY = 128, //!<  when Lane Changing Strategy Data is available
    EXTRA_INFO = 256     //!< when extra info is available

  };

  serdes10(available, target_position, 
           self_position, way_point_position,
           lanes, map_poi_info,
           intersections, lane_merging_splitting_points,
           lane_strategy, extra_info)
};

/*!
 * \brief Map Message sent by Worldmodel Module
 */
struct ProcessedMap {
  maf_std::Header header;
  ProcessedMapMeta meta;               //!< Meta Info
  ProcessedMapData processed_map_data; //!< the detail processed map data
  serdes3(header, meta, processed_map_data)
};

} // namespace maf_worldmodel

#endif

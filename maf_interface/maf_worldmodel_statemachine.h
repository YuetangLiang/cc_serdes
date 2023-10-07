#pragma once
#include "maf_interface/maf_mla_localization.h"
#include "maf_interface/maf_perception_interface.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_worldmodel {

struct Matrix2f {
  float x00;
  float x01;
  float x10;
  float x11;
};

struct Direction {
  uint8_t value;

  enum : uint8_t {
    UNKNOWN = 0,
    GO_STRAIGHT = 1,
    TURN_RIGHT = 2,
    TURN_LEFT = 4,
    U_TURN_LEFT = 8,
    U_TURN_RIGHT = 16,
  };
};

struct Interval {
  double begin;
  double end;
};

struct MapPOIType {
  uint8_t value;

  enum : uint8_t {
    UNKNOWN = 0,
    PARKING_LOT = 1,
    HUMAN_ACCESS = 2,
    DESTINATION = 3,
    PARKING = 4,
    BARRIER_GAP = 5,
    FACILITY_ENTRANCE = 6,
    FACILITY_EXIT = 7,
    FACILITY_EXIT_AND_ENTRANCE = 8,
    BUS_STOP = 9,
    GARAGE_ENTRANCE = 10,
    GARAGE_EXIT = 11,
    SPEED_BUMP = 12,
    CROSS_WALK = 13,
    DASHED_SEGMENT = 14,
    CENTRAL_CIRCLE = 15,
    NO_PARKING_ZONE = 16,
    ROAD_MERGE = 17,
    ROAD_SPLIT = 18,
  };
};

struct Polyline3f {
  std::vector<maf_perception_interface::Point3f> points;
};

struct ParkingOutPosition {
  uint16_t source_parking_lot_id;
  maf_perception_interface::Polygon3f source_parking_lot_position;
  maf_perception_interface::Point3f target_position;
  double target_heading_yaw;
};

struct TraversedParkingLot {
  uint16_t parking_lot_id;
  maf_perception_interface::Polygon3f parking_lot_position;
  maf_perception_interface::Point3f parking_lot_center;
  maf_perception_interface::Point3f parking_lot_entrance_center;
  double available_inner_width;
  double available_outter_width;
  uint16_t next_parking_lot_id;
  uint16_t previous_parking_lot_id;
  double left_extended_distance;
  double right_extended_distance;
  double score;
};

struct WorldModelMeta {
  uint64_t timestamp_us;
  uint64_t ego_pose_timestamp_us;
  uint64_t perception_fusion_timestamp_us;
  uint64_t perception_traffic_light_timestamp_us;
  uint64_t ehp_timestamp_us;
  uint64_t pipeline_start_timestamp_us;
  uint64_t pipeline_finish_timestamp_us;
    serdes3(timestamp_us,ego_pose_timestamp_us,perception_fusion_timestamp_us)
};

struct WorldModelExtra {
  uint8_t available;
  std::string version;
  std::string json;

  enum : uint8_t {
    VERSION = 1,
    JSON = 2,
  };
};

struct SceneObjectType {
  uint8_t value;

  enum : uint8_t {
    UNKNOWN = 0,
    TRAFFIC_BARRIER_TYPE_CONE = 1,
    TRAFFIC_BARRIER_TYPE_WARNING_TRIANGLE = 2,
    TRAFFIC_BARRIER_TYPE_BOLLARD_SLEEVE = 3,
    TRAFFIC_BARRIER_TYPE_PARKING_LOCK = 4,
    TRAFFIC_BARRIER_TYPE_AFRAME_SIGN = 5,
    TRAFFIC_BARRIER_TYPE_ANTICRASH_BUCKET = 6,
    TRAFFIC_BARRIER_TYPE_DUSTBIN = 7,
    TRAFFIC_BARRIER_TYPE_BARRIER_STONE = 8,
    TRAFFIC_BARRIER_TYPE_WATER_BARRIER = 9,
    MAP_FIRE_EXIT_DOOR = 10,
    MAP_PILLAR = 11,
    MAP_TOLL_GATE = 12,
    MAP_BORDER_SOLID_LINE = 13,
    MAP_BORDER_DASH_LINE = 14,
    MAP_BORDER_PHYSICAL = 15,
  };
};

struct APAInfo {
  uint32_t status;
  int32_t map_id;
  uint64_t last_update_timestamp;
  std::string reserved_info;

  enum : uint32_t {
    UNKNOWN = 0,
    OCCUPIED = 1,
    VACANT = 2,
    APA_FAIL = 3,
    APA_SUCCEED = 4,
    REVERSED = 5,
  };
};

struct ObjectWorldModelData {
  bool filtered;
  std::vector<maf_perception_interface::Point3f> occlusion_area;
  std::string extra_json;
};

struct ObjectFusionResult {
  uint8_t available;
  maf_perception_interface::PerceptionFusionObjectData object_fusion_data;

  enum : uint8_t {
    OBJECT_FUSION_DATA = 1,
  };
};

struct ObjectsInterfaceMeta {
  uint64_t sensor_timestamp_us;
  uint64_t pipeline_start_timestamp_us;
  uint64_t pipeline_finish_timestamp_us;
  std::string frame_id;
};

struct MapTrafficLight {
  std::vector<maf_perception_interface::Point3d> boundary;
  maf_perception_interface::TrafficLightPatternEnum pattern;
};

struct LeftWaitingArea {
  bool existence;
  double length;
  double distance;
};

struct MergePoint {
  double distance;
  uint8_t merge_type;
};

struct LaneConnectDirection {
  uint8_t value;

  enum : uint8_t {
    NORMAL = 0,
    SPLIT_TO_LEFT = 1,
    SPLIT_TO_RIGHT = 2,
    MERGE_FROM_LEFT = 3,
    MERGE_FROM_RIGHT = 4,
  };
};

struct RoadEdge {
  bool existence;
  std::vector<maf_perception_interface::Point3f> points;
  uint8_t type;
  std::string reserved;
};

struct LaneBoundaryType {
  uint8_t value;

  enum : uint8_t {
    UNKNOWN = 0,
    SOLID = 1,
    DASH = 2,
    VIRTUAL = 3,
    PHYSICAL = 4,
  };
};

struct LaneType {
  uint8_t value;

  enum : uint8_t {
    UNKNOWN = 0,
    NORMAL = 1,
    VIRTUAL = 2,
    PARKING = 3,
    ACCELERATE = 4,
    DECELERATE = 5,
    BUS = 6,
    EMERGENCY = 7,
    ACCELERATE_DECELERATE = 8,
    LEFT_TURN_WAITTING_AREA = 9,
    NON_MOTOR = 10,
  };
};

struct SelfPositionData {
  bool in_map_area;
  bool in_intersection;
  bool on_ramp;
  int32_t current_parking_slot_id;
};

struct IntentionType {
  int32_t value;

  enum : int32_t {
    INVALID = -1,
    FREE_MOVE = 0,
    KEEP_LANE = 1,
    LEFT_CHANGE_LANE_OUT = 2,
    LEFT_CHANGE_LANE_IN = 3,
    RIGHT_CHANGE_LANE_OUT = 4,
    RIGHT_CHANGE_LANE_IN = 5,
    GO_STRAIGHT = 11,
    TURN_LEFT = 12,
    TURN_RIGHT = 13,
    U_TURN = 14,
  };
};

struct EgoPose {
  uint8_t available;
  maf_mla_localization::MLALocalization mla_localization;

  enum : uint8_t {
    MLA_LOCALIZATION = 1,
  };
    serdes2(available,mla_localization)
};

struct Orientation {
  uint8_t value;

  enum : uint8_t {
    UNKNOWN = 0,
    LEFT = 1,
    RIGHT = 2,
    INTERSECT = 3,
    IGNORED = 4,
  };
};

struct LaneChangeState {
  uint8_t value;

  enum : uint8_t {
    FOLLOW = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
  };
};

struct TargetMapPOI {
  uint16_t id;
  MapPOIType type;
  maf_perception_interface::Polygon3f position;
  double distance;
};

struct ScenePolygonObject {
  uint64_t object_id;
  SceneObjectType object_type;
  maf_perception_interface::Polygon3f object_polygon;
};

struct MapPOIInfoData {
  double distance;
  double length;
  MapPOIType type;
  std::vector<maf_perception_interface::Point3f> key_points_enu;
};

struct ParkingSlotFusionAPAData {
  uint8_t available;
  maf_perception_interface::ParkingSlot parking_slot;
  APAInfo apa_info;

  enum : uint8_t {
    PARKING_SLOT = 1,
    APA_INFO = 2,
  };
};

struct ObjectWorldModelResult {
  uint8_t available;
  ObjectWorldModelData object_world_model_data;

  enum : uint8_t {
    OBJECT_WORLD_MODEL_DATA = 1,
  };
};

struct MapTrafficLightGroup {
  uint64_t track_id;
  std::vector<MapTrafficLight> traffic_lights;
};

struct LaneBoundaryPolyline {
  uint64_t track_id;
  std::vector<double> poly_coefficient;
  Interval available_interval;
};

struct ReferenceLinePoint {
  maf_perception_interface::Point3d car_point;
  maf_perception_interface::Point3d enu_point;
  double curvature;
  double yaw;
  double distance_to_left_road_border;
  double distance_to_right_road_border;
  double distance_to_left_lane_border;
  double distance_to_right_lane_border;
  double distance_to_left_obstacle;
  double distance_to_right_obstacle;
  double lane_width;
  double max_velocity;
  std::string track_id;
  LaneBoundaryType left_road_border_type;
  LaneBoundaryType right_road_border_type;
  bool on_route;
};

struct LaneBoundarySegment {
  double length;
  LaneBoundaryType type;
};

struct ReferenceLineSegment {
  uint16_t begin_index;
  uint16_t end_index;
  LaneConnectDirection direction;
  LaneType lane_type;
  bool is_in_intersection;
  bool is_in_route;
};

struct PredictionTrajectoryPoint {
  maf_perception_interface::Point2f position;
  float yaw;
  float theta;
  float velocity;
  float confidence;
  Matrix2f covariance_xy;
};

struct LaneMergingSplittingPointData {
  double distance;
  bool is_split;
  bool is_continue;
  Orientation orientation;
  double length;
};

struct LaneStrategyData {
  std::vector<LaneChangeState> strategy_start_with_current_lane;
  std::vector<LaneChangeState> strategy_start_with_left_lane;
  std::vector<LaneChangeState> strategy_start_with_right_lane;
  std::vector<Interval> current_lane_change_available_interval;
};

struct ParkingTargetPosition {
  uint64_t available;
  TargetMapPOI target_map_poi;
  std::vector<TraversedParkingLot> traversed_parking_lots;
  ParkingOutPosition parking_out_position;

  enum : uint64_t {
    TARGET_MAP_POI = 1,
    TRAVERSED_PARKING_LOTS = 2,
    PARKING_OUT_POSITION = 4,
  };
};

struct SceneObjects {
  uint8_t available;
  std::vector<ScenePolygonObject> on_path_polygon_objects;
  std::vector<ScenePolygonObject> surround_polygon_objects;

  enum : uint8_t {
    ON_PATH_POLYGON_OBJECTS = 1,
    SURROUND_POLYGON_OBJECTS = 2,
  };
};

struct FusionAPA {
  uint8_t available;
  std::vector<ParkingSlotFusionAPAData> parking_slots;
  maf_perception_interface::GroundLinePerception static_groundlines;
  maf_perception_interface::PerceptionFusionObjects static_objects;
  int32_t ego_parking_slot_track_id;
  int32_t ego_parking_slot_map_id;
  int32_t suggested_parking_slot_track_id;
  int32_t suggested_parking_slot_map_id;
  std::string reserved_info;

  enum : uint8_t {
    PARKING_SLOTS = 1,
    STATIC_GROUNDLINES = 2,
    STATIC_OBJECTS = 4,
    EGO_PARKING_SLOT_TRACK_ID = 8,
    EGO_PARKING_SLOT_MAP_ID = 16,
    SUGGESTED_PARKING_SLOT_TRACK_ID = 32,
    SUGGESTED_PARKING_SLOT_MAP_ID = 64,
    RESERVED_INFO = 128,
  };
};

struct LaneBoundary {
  bool existence;
  std::vector<LaneBoundarySegment> segments;
  LaneBoundaryPolyline polyline;
  std::vector<maf_perception_interface::Point3f> points;
  std::string reserved;
};

struct ReferenceLine {
  bool available;
  std::vector<ReferenceLinePoint> reference_line_points;
  std::vector<ReferenceLineSegment> reference_line_segments;
};

struct PredictionTrajectory {
  float confidence;
  float prediction_interval;
  maf_perception_interface::ObjectStatus status;
  IntentionType intention;
  std::vector<PredictionTrajectoryPoint> trajectory_points;
  std::string extra_json;
};

struct LaneStrategy {
  uint8_t available;
  LaneStrategyData lane_strategy_data;

  enum : uint8_t {
    LANE_STRATEGY_DATA = 1,
  };
};

struct TargetPosition {
  uint8_t available;
  ParkingTargetPosition parking_target_position;

  enum : uint8_t {
    PARKING_TARGET_POSITION = 1,
  };
};

struct ObjectPredictionData {
  std::vector<PredictionTrajectory> trajectories;
  std::string extra_json;
};

struct LaneData {
  int32_t relative_id;
  int32_t track_id;
  Direction lane_marks;
  LaneType lane_type;
  LaneBoundary left_lane_boundary;
  LaneBoundary right_lane_boundary;
  RoadEdge left_road_edge;
  RoadEdge right_road_edge;
  ReferenceLine reference_line;
  double entrance_width;
  MergePoint merge_point;
  MergePoint y_point;
  int8_t bias;
};

struct ObjectPredictionResult {
  uint8_t available;
  ObjectPredictionData object_prediction_data;

  enum : uint8_t {
    OBJECT_PREDICTION_DATA = 1,
  };
};

struct IntersectionData {
  Direction direction;
  double length;
  std::vector<MapTrafficLightGroup> traffic_light_groups;
  LeftWaitingArea left_waiting_area;
  double distance_to_stop_line;
  std::vector<LaneData> lanes_out;
};

struct ObjectInterface {
  ObjectFusionResult object_fusion_result;
  ObjectPredictionResult object_prediction_result;
  ObjectWorldModelResult object_world_model_result;
  std::string extra_json;
};

struct ProcessedMapData {
  uint8_t available;
  TargetPosition target_position;
  SelfPositionData self_position;
  std::vector<LaneData> lanes;
  std::vector<MapPOIInfoData> map_poi_info;
  std::vector<IntersectionData> intersections;
  std::vector<LaneMergingSplittingPointData> lane_merging_splitting_points;
  LaneStrategyData lane_strategy;

  enum : uint8_t {
    TARGET_POTISION = 1,
    SELF_POSITION = 2,
    LANE = 4,
    MAP_POI_INFO = 8,
    INTERSECTION = 16,
    LANE_MERGING_SPLITTING_POINT = 32,
    LANE_STRATEGY = 64,
  };
};

struct ObjectsResult {
  uint8_t available;
  std::vector<ObjectInterface> object_interface;

  enum : uint8_t {
    OBJECTS_RESULT = 1,
  };
};

struct EnvironmentData {
  uint8_t available;
  ProcessedMapData processed_map_data;
  FusionAPA fusion_apa;
  maf_perception_interface::TrafficLightPerception perception_traffic_light;
  SceneObjects scene_objects;

  enum : uint8_t {
    PROCESSED_MAP_DATA = 1,
    FUSION_APA = 2,
    TRAFFIC_LIGHT_PERCEPTION = 4,
    SCENE_OBJECTS = 8,
  };
};

struct WorldModel {
  maf_std::Header header;
  WorldModelMeta meta;
  EgoPose ego_pose;
  ObjectsResult perception_objects_result;
  EnvironmentData environment_data;
  WorldModelExtra extra;
    serdes3(header,meta,ego_pose)
};

struct ObjectsInterface {
  maf_std::Header header;
  ObjectsInterfaceMeta meta;
  ObjectsResult objects_result;
};

} // namespace maf_worldmodel

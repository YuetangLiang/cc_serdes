#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_mpa {

struct FreespacePropertyEnum {
  uint8_t value;

  enum : uint8_t {
    FREESPACE_PROPERTY_TAG = 0,
    FREESPACE_PROPERTY_BORDER_POINTS_2D = 1,
    FREESPACE_PROPERTY_LABEL_MAP_2D = 2,
    FREESPACE_PROPERTY_IS_FAILED_3D = 3,
    FREESPACE_PROPERTY_BORDER_POINTS_BV = 4,
  };
};

struct FreespacePointTypeEnum {
  uint8_t value;

  enum : uint8_t {
    FREESPACE_UNKNOWN = 0,
    FREESPACE_NON_GROUNDING_POINT = 1,
    FREESPACE_ROAD_SURFACE_POINT = 2,
    FREESPACE_STATIC_BORDER = 3,
    FREESPACE_DYNAMIC_BORDER = 4,
  };
};

struct FreespaceLabelMapEnum {
  uint8_t value;

  enum : uint8_t {
    FREESPACE_LABEL_MAP_BACKGROUND = 0,
    FREESPACE_LABEL_MAP_GROUND_SURFACE = 1,
  };
};

struct FreespaceWheelLinePropertyEnum {
  uint8_t value;

  enum : uint8_t {
    FREESPACE_WHEEL_LINE_PROPERTY_TAG = 0,
    FREESPACE_WHEEL_LINE_PROPERTY_SEGMENTS_2D = 1,
    FREESPACE_WHEEL_LINE_PROPERTY_IS_FAILED_3D = 2,
    FREESPACE_WHEEL_LINE_PROPERTY_SEGMENTS_BV = 3,
  };
};

struct Point3f {
  double x;
  double y;
  double z;
};

struct Vector3f {
  double x;
  double y;
  double z;
};

struct Point2f {
  double x;
  double y;
};

struct Vector2f {
  float x;
  float y;
};

struct FrameMeta {
  uint64_t timestamp_us;
  uint64_t sequence;
  std::string tag;
};

struct Rect4f {
  float left;
  float top;
  float right;
  float bottom;
};

struct Shape2f {
  float height;
  float width;
};

struct Shape3f {
  float length;
  float height;
  float width;
};

struct TrafficLightStatusEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_STATUS_BACKGROUND = 0,
    TRAFFIC_LIGHT_STATUS_RED = 1,
    TRAFFIC_LIGHT_STATUS_YELLOW = 2,
    TRAFFIC_LIGHT_STATUS_GREEN = 3,
    TRAFFIC_LIGHT_STATUS_OTHER = 4,
    TRAFFIC_LIGHT_STATUS_OFF = 5,
    TRAFFIC_LIGHT_STATUS_COUNT = 6,
  };
};

struct TrafficLightDecisionResultEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_DECISION_RESULT_PASS_WITH_OFF = 0,
    TRAFFIC_LIGHT_DECISION_RESULT_PASS_WITH_GREEN = 1,
    TRAFFIC_LIGHT_DECISION_RESULT_STOP_WITH_RED = 2,
    TRAFFIC_LIGHT_DECISION_RESULT_WARNING_WITH_GREEN_BLINKING = 3,
    TRAFFIC_LIGHT_DECISION_RESULT_WARNING_WITH_YELLOW = 4,
    TRAFFIC_LIGHT_DECISION_RESULT_WARNING_WITH_YELLOW_BLINKING = 5,
    TRAFFIC_LIGHT_DECISION_RESULT_NO_VISION_RESULT = 6,
    TRAFFIC_LIGHT_DECISION_RESULT_EXCEPTIONAL_VISION_RESULT = 7,
    TRAFFIC_LIGHT_DECISION_RESULT_NO_TRAFFIC_LIGHT = 8,
  };
};

struct TrafficLightDecisionPropertyEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_DECISION_PROPERTY_TAG = 0,
    TRAFFIC_LIGHT_DECISION_PROPERTY_ACTION = 1,
    TRAFFIC_LIGHT_DECISION_PROPERTY_RESULT = 2,
    TRAFFIC_LIGHT_DECISION_PROPERTY_DURATION = 3,
    TRAFFIC_LIGHT_DECISION_PROPERTY_REMAINING = 4,
  };
};

struct TrafficLightFusionPropertyEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_FUSION_PROPERTY_TAG = 0,
    TRAFFIC_LIGHT_FUSION_PROPERTY_PATTERN = 1,
    TRAFFIC_LIGHT_FUSION_PROPERTY_STATUS = 2,
    TRAFFIC_LIGHT_FUSION_PROPERTY_DURATION = 3,
    TRAFFIC_LIGHT_FUSION_PROPERTY_REMAINING = 4,
  };
};

struct TrafficLightPropertyEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_PROPERTY_TAG = 0,
    TRAFFIC_LIGHT_PROPERTY_SCORE = 1,
    TRAFFIC_LIGHT_PROPERTY_BOUNDING_BOX_2D = 2,
    TRAFFIC_LIGHT_PROPERTY_PATTERN = 3,
    TRAFFIC_LIGHT_PROPERTY_STATUS = 4,
    TRAFFIC_LIGHT_PROPERTY_TRACK_ID = 5,
  };
};

struct TrafficLightPatternEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_PATTERN_UNKNOWN = 0,
    TRAFFIC_LIGHT_PATTERN_SOLID_CIRCLE = 1,
    TRAFFIC_LIGHT_PATTERN_LEFT_ARROW = 2,
    TRAFFIC_LIGHT_PATTERN_STRAIGHT_ARROW = 3,
    TRAFFIC_LIGHT_PATTERN_RIGHT_ARROW = 4,
    TRAFFIC_LIGHT_PATTERN_NUMBER = 5,
    TRAFFIC_LIGHT_PATTERN_OFF = 6,
    TRAFFIC_LIGHT_PATTERN_OTHER = 7,
  };
};

struct TrafficLightFusionStatusEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_FUSION_STATUS_OFF = 0,
    TRAFFIC_LIGHT_FUSION_STATUS_GREEN = 1,
    TRAFFIC_LIGHT_FUSION_STATUS_GREEN_BLINKING = 2,
    TRAFFIC_LIGHT_FUSION_STATUS_YELLOW = 3,
    TRAFFIC_LIGHT_FUSION_STATUS_RED = 4,
    TRAFFIC_LIGHT_FUSION_STATUS_YELLOW_BLINKING = 5,
    TRAFFIC_LIGHT_FUSION_STATUS_UNKNOWN = 6,
  };
};

struct TrafficLightActionEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_LIGHT_ACTION_TURN_LEFT = 0,
    TRAFFIC_LIGHT_ACTION_GO_STRAIGHT = 1,
    TRAFFIC_LIGHT_ACTION_TURN_RIGHT = 2,
    TRAFFIC_LIGHT_ACTION_U_TURN = 3,
  };
};

struct HumanAssociatedInfo {
  std::string tag;
  uint64_t track_id;
};

struct HumanTypeEnum {
  uint8_t value;

  enum : uint8_t {
    HUMAN_TYPE_OFO = 0,
    HUMAN_TYPE_PEDESTRIAN = 1,
    HUMAN_TYPE_RIDER = 2,
  };
};

struct HumanRelationTypeEnum {
  uint8_t value;

  enum : uint8_t {
    HUMAN_RELATION_MATCH_TO_OFO = 0,
    HUMAN_RELATION_MATCH_TO_RIDER = 1,
  };
};

struct HumanTrackInfo {
  uint32_t track_id;
};

struct HumanPropertyEnum {
  uint8_t value;

  enum : uint8_t {
    HUMAN_PROPERTY_TAG = 0,
    HUMAN_PROPERTY_SCORE = 1,
    HUMAN_PROPERTY_VISIBILITY_2D = 2,
    HUMAN_PROPERTY_BOUNDING_BOX_2D = 3,
    HUMAN_PROPERTY_KEYPOINTS_2D = 4,
    HUMAN_PROPERTY_WAIST_HEIGHT_2D = 5,
    HUMAN_PROPERTY_TYPE = 6,
    HUMAN_PROPERTY_REID_FEATURE = 7,
    HUMAN_PROPERTY_TRACK_INFO = 8,
    HUMAN_PROPERTY_SHAPE_3D = 9,
    HUMAN_PROPERTY_SHAPE_3D_SIGMA = 10,
    HUMAN_PROPERTY_VELOCITY_BV = 11,
    HUMAN_PROPERTY_VELOCITY_BV_SIGMA = 12,
    HUMAN_PROPERTY_LOCATION_BV = 13,
    HUMAN_PROPERTY_LOCATION_BV_SIGMA = 14,
    HUMAN_PROPERTY_IS_FAILED_3D = 15,
    HUMAN_PROPERTY_RELATIONS = 16,
    HUMAN_PROPERTY_ASSOCIATED_INFO = 17,
  };
};

struct HumanReIDFeature {
  std::vector<float> data;
};

struct LaneTrackInfo {
  uint32_t track_id;
};

struct LanePropertyEnum {
  uint8_t value;

  enum : uint8_t {
    LANE_PROPERTY_TAG = 0,
    LANE_PROPERTY_SCORE = 1,
    LANE_PROPERTY_INDEX = 2,
    LANE_PROPERTY_POINTS_2D = 3,
    LANE_PROPERTY_TRACK_INFO = 4,
    LANE_PROPERTY_SOLID_SCORE = 5,
    LANE_PROPERTY_WIDTH_SCORE = 6,
    LANE_PROPERTY_COLOR_YELLOW_SCORE = 7,
    LANE_PROPERTY_COEFFICIENT_BV = 8,
    LANE_PROPERTY_IS_FAILED_3D = 9,
  };
};

struct TrafficBarrierReIDFeature {
  std::vector<float> data;
};

struct TrafficBarrierTypeEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_BARRIER_TYPE_UNKNOWN = 0,
    TRAFFIC_BARRIER_TYPE_CONE_BUCKET = 1,
    TRAFFIC_BARRIER_TYPE_WARNING_TRIANGLE = 2,
  };
};

struct TrafficBarrierPropertyEnum {
  uint8_t value;

  enum : uint8_t {
    TRAFFIC_BARRIER_PROPERTY_TAG = 0,
    TRAFFIC_BARRIER_PROPERTY_SCORE = 1,
    TRAFFIC_BARRIER_PROPERTY_VISIBILITY_2D = 2,
    TRAFFIC_BARRIER_PROPERTY_BOUNDING_BOX_2D = 3,
    TRAFFIC_BARRIER_PROPERTY_KEYPOINTS_2D = 4,
    TRAFFIC_BARRIER_PROPERTY_TYPE = 5,
    TRAFFIC_BARRIER_PROPERTY_REID_FEATURE = 6,
    TRAFFIC_BARRIER_PROPERTY_TRACK_INFO = 7,
    TRAFFIC_BARRIER_PROPERTY_SHAPE_3D = 8,
    TRAFFIC_BARRIER_PROPERTY_SHAPE_3D_SIGMA = 9,
    TRAFFIC_BARRIER_PROPERTY_VELOCITY_BV = 10,
    TRAFFIC_BARRIER_PROPERTY_VELOCITY_BV_SIGMA = 11,
    TRAFFIC_BARRIER_PROPERTY_LOCATION_BV = 12,
    TRAFFIC_BARRIER_PROPERTY_LOCATION_BV_SIGMA = 13,
    TRAFFIC_BARRIER_PROPERTY_IS_FAILED_3D = 14,
    TRAFFIC_BARRIER_PROPERTY_ASSOCIATED_INFO = 15,
  };
};

struct TrafficBarrierTrackInfo {
  uint32_t track_id;
};

struct TrafficBarrierAssociatedInfo {
  std::string tag;
  uint64_t track_id;
};

struct CarPropertyEnum {
  uint8_t value;

  enum : uint8_t {
    CAR_PROPERTY_TAG = 0,
    CAR_PROPERTY_SCORE = 1,
    CAR_PROPERTY_VISIBILITY_2D = 2,
    CAR_PROPERTY_BOUNDING_BOX_2D = 3,
    CAR_PROPERTY_WHEELS_2D = 4,
    CAR_PROPERTY_PILLARS_2D = 5,
    CAR_PROPERTY_HEADING_YAW_2D = 6,
    CAR_PROPERTY_TRACK_INFO = 7,
    CAR_PROPERTY_BACKLAMP = 8,
    CAR_PROPERTY_TYPE = 9,
    CAR_PROPERTY_REID_FEATURE = 10,
    CAR_PROPERTY_BOUNDING_BOX_BV_CLOSEST_POINT_INDEX = 11,
    CAR_PROPERTY_SHAPE_3D = 12,
    CAR_PROPERTY_SHAPE_3D_SIGMA = 13,
    CAR_PROPERTY_HEADING_THETA_BV = 14,
    CAR_PROPERTY_HEADING_THETA_BV_SIGMA = 15,
    CAR_PROPERTY_VELOCITY_BV = 16,
    CAR_PROPERTY_VELOCITY_BV_SIGMA = 17,
    CAR_PROPERTY_HIDE_3D = 18,
    CAR_PROPERTY_LOCATION_BV = 19,
    CAR_PROPERTY_LOCATION_BV_SIGMA = 20,
    CAR_PROPERTY_IS_FAILED_3D = 21,
    CAR_PROPERTY_ASSOCIATED_INFO = 22,
  };
};

struct CarTypeEnum {
  uint8_t value;

  enum : uint8_t {
    CAR_TYPE_UNKNOWN = 0,
    CAR_TYPE_COUPE = 1,
    CAR_TYPE_TRUCK = 2,
    CAR_TYPE_BUS = 3,
    CAR_TYPE_WATER_TRUCK = 4,
    CAR_TYPE_TRICYCLE = 5,
  };
};

struct CarScene {
  std::string tag;
};

struct CarTrackInfo {
  uint64_t track_id;
  uint64_t track_times;
  int8_t live;
};

struct CarBacklampEnum {
  uint8_t value;

  enum : uint8_t {
    CAR_BACKLAMP_UNKNOWN = 0,
    CAR_BACKLAMP_OFF = 1,
    CAR_BACKLAMP_BRAKE_LIGHT = 2,
    CAR_BACKLAMP_WIDTH_LIGHT = 3,
  };
};

struct CarAssociatedInfo {
  std::string tag;
  uint64_t track_id;
};

struct CarPillar {
  float position_x;
  float visibility;
};

struct CarReIDFeature {
  std::vector<float> data;
};

struct VruTrackInfo {
  uint32_t track_id;
};

struct VruPropertyEnum {
  uint8_t value;

  enum : uint8_t {
    VRU_PROPERTY_TAG = 0,
    VRU_PROPERTY_SCORE = 1,
    VRU_PROPERTY_VISIBILITY_2D = 2,
    VRU_PROPERTY_BOUNDING_BOX_2D = 3,
    VRU_PROPERTY_KEYPOINTS_2D = 4,
    VRU_PROPERTY_WHEELS_2D = 5,
    VRU_PROPERTY_WAIST_HEIGHT_2D = 6,
    VRU_PROPERTY_TYPE = 7,
    VRU_PROPERTY_REID_FEATURE = 8,
    VRU_PROPERTY_TRACK_INFO = 9,
    VRU_PROPERTY_SHAPE_3D = 10,
    VRU_PROPERTY_SHAPE_3D_SIGMA = 11,
    VRU_PROPERTY_VELOCITY_BV = 12,
    VRU_PROPERTY_VELOCITY_BV_SIGMA = 13,
    VRU_PROPERTY_LOCATION_BV = 14,
    VRU_PROPERTY_LOCATION_BV_SIGMA = 15,
    VRU_PROPERTY_IS_FAILED_3D = 16,
    VRU_PROPERTY_ASSOCIATED_INFO = 17,
  };
};

struct VruReIDFeature {
  std::vector<float> data;
};

struct VruTypeEnum {
  uint8_t value;

  enum : uint8_t {
    VRU_TYPE_ONLY_OFO = 0,
    VRU_TYPE_WITH_RIDER = 1,
  };
};

struct VruAssociatedInfo {
  std::string tag;
  uint64_t track_id;
};

struct FreespaceSegmentPoint {
  Point2f position;
  FreespacePointTypeEnum type;
  float visibility;
};

struct FreespaceBorderPoint {
  Point2f position;
  FreespacePointTypeEnum type;
  float confidence;
};

struct FreespaceLabelMap {
  uint32_t height;
  uint32_t width;
  uint32_t channels;
  std::vector<FreespaceLabelMapEnum> data;
};

struct TrafficLightFusion {
  std::string tag;
  std::vector<TrafficLightFusionPropertyEnum> properties;
  TrafficLightPatternEnum pattern;
  TrafficLightFusionStatusEnum status;
  double duration;
  double remaining;
};

struct TrafficLightScene {
  Rect4f crop_roi;
};

struct TrafficLightDecision {
  std::string tag;
  std::vector<TrafficLightDecisionPropertyEnum> properties;
  TrafficLightActionEnum action;
  TrafficLightDecisionResultEnum decision_result;
  double duration;
  double remaining;
};

struct TrafficLight {
  std::string tag;
  std::vector<TrafficLightPropertyEnum> properties;
  float score;
  Rect4f bounding_box_2d;
  TrafficLightPatternEnum pattern;
  TrafficLightStatusEnum status;
  int32_t track_id;
  std::string log_info_2d;
  std::string reserved_info;
};

struct HumanRelation {
  HumanRelationTypeEnum type;
  uint32_t relative_id;
};

struct LanePoint {
  Point2f position;
  float visibility;
};

struct TrafficLightIndication {
  std::vector<Point3f> bounding_box_3d;
  TrafficLightPatternEnum pattern;
};

struct TrafficBarrier {
  std::string tag;
  std::vector<TrafficBarrierPropertyEnum> properties;
  float score;
  float visibility_2d;
  Rect4f bounding_box_2d;
  std::vector<Point2f> keypoints_2d;
  TrafficBarrierReIDFeature reid_feature;
  TrafficBarrierTypeEnum type;
  TrafficBarrierTrackInfo track_info;
  std::string log_info_2d;
  std::string reserved_info;
  Shape2f shape_3d;
  Shape2f shape_3d_sigma;
  Vector2f velocity_bv;
  Vector2f velocity_bv_sigma;
  Point2f location_bv;
  Point2f location_bv_sigma;
  int8_t is_failed_3d;
  std::vector<TrafficBarrierAssociatedInfo> associated_infos;
};

struct ExtendedPredictionMeta {
  std::string algorithm_name;
  std::string instance_name;
  uint64_t timestamp_us;
  std::vector<FrameMeta> relevant_frames;
  std::string ts_log;
};

struct PredictionMeta {
  std::string algorithm_name;
  std::string instance_name;
  uint64_t timestamp_us;
  std::vector<FrameMeta> relevant_frames;
};

struct CarWheel {
  Point2f position;
  float visibility;
};

struct VruWheel {
  Point2f position;
  float visibility;
};

struct Freespace {
  std::string tag;
  std::vector<FreespacePropertyEnum> properties;
  std::vector<FreespaceBorderPoint> border_points_2d;
  FreespaceLabelMap label_map_2d;
  std::string log_info_2d;
  std::string reserved_info;
  bool is_failed_3d;
  std::vector<Point2f> border_points_bv;
};

struct FreespaceSegment {
  FreespaceSegmentPoint start_point;
  FreespaceSegmentPoint end_point;
  float score;
};

struct TrafficLightPredictions {
  std::vector<TrafficLight> traffic_lights;
  TrafficLightScene scene;
  std::vector<TrafficLightFusion> traffic_light_fusion;
  std::vector<TrafficLightDecision> traffic_light_decision;
};

struct Human {
  std::string tag;
  std::vector<HumanPropertyEnum> properties;
  float score;
  float visibility_2d;
  Rect4f bounding_box_2d;
  std::vector<Point2f> keypoints_2d;
  float waist_height_2d;
  HumanReIDFeature reid_feature;
  HumanTypeEnum type;
  HumanTrackInfo track_info;
  std::string log_info_2d;
  std::string reserved_info;
  Shape2f shape_3d;
  Shape2f shape_3d_sigma;
  Vector2f velocity_bv;
  Vector2f velocity_bv_sigma;
  Point2f location_bv;
  Point2f location_bv_sigma;
  int8_t is_failed_3d;
  std::vector<HumanRelation> relations;
  std::vector<HumanAssociatedInfo> associated_infos;
};

struct Lane {
  std::string tag;
  std::vector<LanePropertyEnum> properties;
  float score;
  int32_t index;
  std::vector<LanePoint> points_2d;
  LaneTrackInfo track_info;
  float solid_score;
  float width_score;
  float color_yellow_score;
  std::vector<float> coefficient_bv;
  bool is_failed_3d;
  std::string log_info_2d;
  std::string reserved_info;
};

struct TrafficLightDetectTrigger {
  maf_std::Header header;
  uint64_t id;
  int32_t num;
  std::vector<TrafficLightIndication> indications;
};

struct TrafficBarrierPredictions {
  std::vector<TrafficBarrier> traffic_barriers;
};

struct Car {
  std::string tag;
  std::vector<CarPropertyEnum> properties;
  float score;
  float visibility_2d;
  Rect4f bounding_box_2d;
  std::vector<CarWheel> wheels_2d;
  std::vector<CarPillar> pillars_2d;
  float heading_yaw_2d;
  CarTrackInfo track_info;
  CarBacklampEnum backlamp;
  CarTypeEnum type;
  CarReIDFeature reid_feature;
  std::string log_info_2d;
  std::string reserved_info;
  std::vector<Point2f> bounding_box_bv;
  int32_t bounding_box_bv_closest_point_index;
  Shape3f shape_3d;
  Shape3f shape_3d_sigma;
  float heading_theta_bv;
  float heading_theta_bv_sigma;
  Vector2f velocity_bv;
  Vector2f velocity_bv_sigma;
  bool hide_3d;
  Point2f location_bv;
  Point2f location_bv_sigma;
  int8_t is_failed_3d;
  std::vector<CarAssociatedInfo> associated_infos;
};

struct Vru {
  std::string tag;
  std::vector<VruPropertyEnum> properties;
  float score;
  float visibility_2d;
  Rect4f bounding_box_2d;
  std::vector<Point2f> keypoints_2d;
  std::vector<VruWheel> wheels_2d;
  float waist_height_2d;
  VruReIDFeature reid_feature;
  VruTypeEnum type;
  VruTrackInfo track_info;
  std::string log_info_2d;
  std::string reserved_info;
  Shape2f shape_3d;
  Shape2f shape_3d_sigma;
  Vector2f velocity_bv;
  Vector2f velocity_bv_sigma;
  Point2f location_bv;
  Point2f location_bv_sigma;
  int8_t is_failed_3d;
  std::vector<VruAssociatedInfo> associated_infos;
};

struct FreespaceWheelEarthLine {
  std::string tag;
  std::vector<FreespaceWheelLinePropertyEnum> properties;
  std::vector<FreespaceSegment> segments_2d;
  bool is_failed_3d;
  std::vector<FreespaceSegment> segments_bv;
};

struct HumanPredictions {
  std::vector<Human> humans;
};

struct LanePredictions {
  std::vector<Lane> lanes;
};

struct CarPredictions {
  std::vector<Car> cars;
};

struct VruPredictions {
  std::vector<Vru> vrus;
};

struct FreespacePredictions {
  std::vector<Freespace> freespaces;
  std::vector<FreespaceWheelEarthLine> wheel_earth_lines;
};

struct ExtendedPerceptionPrediction {
  maf_std::Header header;
  ExtendedPredictionMeta meta;
  std::string version_tag;
  CarPredictions car_predictions;
  LanePredictions lane_predictions;
  HumanPredictions human_predictions;
  VruPredictions vru_predictions;
  FreespacePredictions freespace_predictions;
  TrafficBarrierPredictions traffic_barrier_predictions;
  TrafficLightPredictions traffic_light_predictions;
};

struct PerceptionPrediction {
  maf_std::Header header;
  PredictionMeta meta;
  std::string version_tag;
  CarPredictions car_predictions;
  LanePredictions lane_predictions;
  HumanPredictions human_predictions;
  VruPredictions vru_predictions;
  FreespacePredictions freespace_predictions;
  TrafficBarrierPredictions traffic_barrier_predictions;
  TrafficLightPredictions traffic_light_predictions;
};

} // namespace maf_mpa

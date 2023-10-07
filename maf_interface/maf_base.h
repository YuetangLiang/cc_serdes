#pragma once
#include "maf_interface/maf_geometry.h"
#include "maf_interface/maf_recorder.h"
#include "maf_interface/maf_sensor.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_base {

struct LaneInfo {
  std::vector<uint8_t> lane_marks;
  uint8_t lane_type;
  uint8_t lane_boundary_existence;
};

struct MergePoint {
  double distance;
  uint8_t merge_type;
};

struct MapTaskRange {
  double begin_pos;
  double end_pos;
};

struct MapLight {
  float dis2crossing;
  int32_t id;
};

struct BoundaryInfo {
  double length;
  uint8_t type;
};

struct MTPTrafficLight {
  std::vector<float> rect;
  float score;
  int32_t state_enum;
  int32_t pattern_enum;
  int32_t track_id;
};

struct AsyncDiskEjectOutput {
  int64_t key_num;
  bool service_response;
  bool success;
  std::string message;
};

struct ArsObject {
  uint16_t detection_id;
  maf_geometry::Point position;
  maf_geometry::Vector3 velocity;
  uint8_t dyn_prop;
  float rcs;
  uint8_t dislong_rms;
  uint8_t dislat_rms;
  uint8_t verllong_rms;
  uint8_t verllat_rms;
  uint8_t arellong_rms;
  uint8_t arellat_rms;
  uint8_t orientation_rms;
  uint8_t meas_state;
  uint8_t probofexist;
  float are_long;
  uint8_t obj_class;
  float are_lat;
  float orientation_angle;
  float length;
  float width;
};

struct PredictionJunctionFeatures {
  maf_std::Header header;
  bool is_feature_invalid;
  int32_t id;
  int32_t type;
  float x;
  float y;
  float yaw;
  float diff_theta_car_lane;
  float v;
  float v_x;
  float v_y;
  float acc;
  float acc_x;
  float acc_y;
  float diff_previous_yaw;
  float diff_latest_yaw;
  float diff_previous_x;
  float diff_previous_y;
  float diff_latest_x;
  float diff_latest_y;
  float diff_previous_v_x;
  float diff_previous_v_y;
  float diff_latest_v_x;
  float diff_latest_v_y;
};

struct AsyncCommonInput {
  int64_t key_num;
};

struct AsyncDiskTypeInput {
  int64_t key_num;
  std::string sn;
  std::string disk_type;
};

struct SrrCluster {
  uint16_t detection_id;
  float range;
  float azimuth;
  float vrel;
  uint8_t dyn_prop;
  float rcs;
};

struct PredictionTrajectoryPoint {
  float x;
  float y;
  float yaw;
  float theta;
  float speed;
  float prob;
  float std_dev_x;
  float std_dev_y;
  float sigma_xy;
  float std_dev_yaw;
  float std_dev_speed;
  float relative_ego_x;
  float relative_ego_y;
  float relative_ego_yaw;
  float relative_ego_speed;
  float relative_ego_std_dev_x;
  float relative_ego_std_dev_y;
  float relative_ego_std_dev_yaw;
  float relative_ego_std_dev_speed;
};

struct AsyncCheckRecordingOutput {
  int64_t key_num;
  bool service_response;
  bool collection_disk_recording;
  bool export_disk_recording;
  bool any_disk_recording;
};

struct SpeedChangePoint {
  double x;
  double y;
  double speed;
};

struct AsyncDiskListOutput {
  int64_t key_num;
  bool service_response;
  std::vector<std::string> sn;
  std::vector<std::string> disk_type;
  std::vector<std::string> status;
  std::vector<int64_t> space_size;
  std::vector<int64_t> space_used;
};

struct Mdf {
  int32_t id;
  int32_t idx;
  float speed_limit;
  int32_t p1;
  int32_t p2;
  bool traffic_light;
};

struct FusionTrafficLight {
  int32_t pattern;
  int32_t status;
  double duration;
  double remaining;
};

struct Event {
  maf_std::Header header;
  std::string name;
  int64_t before;
  int64_t after;
  std::string detail;
};

struct ParamAggregatorConfig {
  std::string name;
  std::string data;
  std::string type;
};

struct Lanemark {
  uint32_t id;
  float prob;
  bool is_solid;
  bool is_boundary;
  uint8_t color;
  bool is_Yline;
  bool is_biline;
  std::vector<maf_geometry::Pose2D> pts_on_img;
  std::vector<maf_geometry::Pose2D> pts_on_bv;
};

struct Tag {
  maf_std::Header header;
  std::string name;
  std::string detail;
};

struct AvdMsg {
  int32_t Id;
  std::string Property;
  bool Ignore;
  std::string AvdDirection;
};

struct PredictionAroundCarFeature {
  float relative_s;
  float relative_r;
  float relative_v;
};

struct AimedPoiInfo {
  maf_std::Header header;
  uint16_t id;
  std::string type;
  double distance;
  std::vector<maf_geometry::Point> corners;
};

struct CenterLine {
  std::vector<double> polycoeff;
  double head;
  double tail;
  bool is_recommended;
};

struct ParkingOutInfo {
  maf_std::Header header;
  uint16_t id;
  std::string type;
  std::vector<maf_geometry::Point> corners;
  double apoa_yaw;
  maf_geometry::Point apoa_enu;
};

struct FusionPolygon {
  uint64_t track_id;
  uint32_t type;
  float const_vel_prob;
  float const_acc_prob;
  float still_prob;
  float coord_turn_prob;
  maf_geometry::Point position;
  maf_geometry::Vector3 velocity;
  maf_geometry::Vector3 accelaration;
  float theta;
  float theta_sigma;
  float omega;
  float omega_sigma;
  float d_omega;
  float d_omega_sigma;
  maf_geometry::Vector3 sigma_position;
  maf_geometry::Vector3 sigma_velocity;
  maf_geometry::Point relative_position;
  maf_geometry::Vector3 relative_velocity;
  maf_geometry::Vector3 relative_accelaration;
  float relative_theta;
  float length;
  float width;
  float height;
  std::vector<maf_geometry::Point> polygon_bottom;
  std::vector<maf_geometry::Point> polygon_top;
};

struct IntersectionManeuver {
  uint8_t direction;
  double distance;
  double length;
};

struct AimedPoi {
  maf_std::Header header;
  uint16_t id;
  std::string type;
  std::vector<maf_geometry::Point> corners;
};

struct TrafficLightInfo {
  std::vector<uint8_t> patterns;
  uint8_t direction;
};

struct AsyncDiskTypeOutput {
  int64_t key_num;
  bool service_response;
  bool success;
  std::string message;
};

struct DecisionReport {
  maf_std::Header header;
  uint8_t current_direction;
  uint8_t light_status;
  uint8_t light_pattern;
  double dist2stopline;
  maf_std::String current_state;
  maf_std::String current_lane_id;
};

struct LightDetectTrigger {
  maf_std::Header header;
  std::vector<maf_geometry::Point> boundary;
  int32_t num_signals;
  std::vector<int32_t> pattern_signals;
  uint64_t light_id;
};

struct DbwStatus {
  maf_std::Header header;
  bool status;
};

struct ArsCluster {
  uint16_t detection_id;
  maf_geometry::Point position;
  maf_geometry::Vector3 velocity;
  uint8_t dyn_prop;
  float rcs;
  uint8_t dislong_rms;
  uint8_t dislat_rms;
  uint8_t verllong_rms;
  uint8_t verllat_rms;
  uint8_t arellong_rms;
  uint8_t arellat_rms;
  uint8_t pdh0;
  uint8_t ambigstate;
  uint8_t invalidstate;
};

struct PredictionDisplayConfig {
  std::string prediction_topic;
  std::string judge_topic;
  std::vector<uint64_t> track_ids;
};

struct AsyncExportDetailsV2Output {
  int64_t key_num;
  bool service_response;
  bool success;
  std::string message;
  std::vector<maf_recorder::ExportJobInfoV2> export_jobs;
};

struct RefLinePoint {
  double x;
  double y;
  double z;
  double curvature;
  double yaw;
  double left_road_border_distance;
  double right_road_border_distance;
  double left_lane_border_distance;
  double right_lane_border_distance;
  double left_obstacle_distance;
  double right_obstacle_distance;
  double lane_width;
  double speed_limit;
  std::string track_id;
};

struct Object {
  uint64_t track_id;
  int32_t det_trunc;
  maf_geometry::Point position;
  maf_geometry::Vector3 sigma_position;
  maf_geometry::Vector3 velocity;
  maf_geometry::Vector3 sigma_velocity;
  float length;
  float width;
  float height;
  float sigma_length;
  float sigma_width;
  float sigma_height;
  float theta;
  float sigma_theta;
  uint32_t type;
  uint64_t track_times;
  int32_t visibility;
  int32_t back_lamp;
  float det_score;
  std::vector<int32_t> bbox_pts;
  std::vector<float> reid_feature;
};

struct NaviFusion {
  maf_std::Header header;
  maf_geometry::Pose enu;
  maf_geometry::Pose2D pose;
  maf_sensor::NavSatFix wgs84_position;
  maf_sensor::Imu imu;
  maf_geometry::Twist vel;
  double vel_scalar;
  uint8_t gps_status;
  uint8_t sat_cnt1;
  uint8_t sat_cnt2;
};

struct AsyncTopicsV2Input {
  int64_t key_num;
  bool set;
  std::vector<maf_recorder::ImageTopicConfig> image_topic_list;
  std::vector<std::string> non_image_topic_list;
  std::vector<std::string> latched_topic_list;
};

struct NodesTopics {
  maf_std::Header header;
  std::vector<std::string> topics;
};

struct FusionObject {
  uint64_t track_id;
  uint32_t type;
  float const_vel_prob;
  float const_acc_prob;
  float still_prob;
  float coord_turn_prob;
  maf_geometry::Point position;
  maf_geometry::Vector3 velocity;
  maf_geometry::Vector3 accelaration;
  float theta;
  float theta_sigma;
  float omega;
  float omega_sigma;
  float d_omega;
  float d_omega_sigma;
  maf_geometry::Vector3 sigma_position;
  maf_geometry::Vector3 sigma_velocity;
  maf_geometry::Point relative_position;
  maf_geometry::Vector3 relative_velocity;
  maf_geometry::Vector3 relative_accelaration;
  float relative_theta;
  float length;
  float width;
  float height;
};

struct AsyncDiskEjectInput {
  int64_t key_num;
  std::string sn;
};

struct LaneBoundary {
  std::vector<double> polycoeff;
  double head;
  double tail;
  int8_t relative_id;
  uint64_t track_id;
};

struct LanePoint {
  maf_geometry::Pose2D pose;
  float vel_limit;
  float width_left;
  bool is_left_cross;
  float width_right;
  bool is_right_cross;
};

struct AsyncTopicsV2Output {
  int64_t key_num;
  bool service_response;
  bool success;
  std::string message;
  std::vector<maf_recorder::ImageTopicConfig> image_topic_list;
  std::vector<std::string> non_image_topic_list;
  std::vector<std::string> latched_topic_list;
};

struct MTPCarObject {
  uint64_t available;
  float left;
  float top;
  float right;
  float bottom;
  float score;
  int32_t visibility;
  std::vector<float> wheels_position_x;
  std::vector<float> wheels_position_y;
  std::vector<float> wheels_visibility;
  int32_t wheels_count;
  std::vector<float> pillars_position_x;
  std::vector<float> pillars_visibility;
  int32_t pillars_count;
  uint32_t heading;
  uint64_t track_id;
  uint64_t track_times;
  int32_t live;
  float angle_yaw;
  uint32_t backlamp;
  uint32_t type;
  uint32_t reid_feature_length;
  std::vector<float> reid_feature;
};

struct DebugInfo {
  maf_std::Header header;
  uint8_t level;
  std::string related_topic;
  std::string data;

  enum : uint8_t {
    LEVEL_DEBUG = 0,
    LEVEL_INFO = 1,
    LEVEL_WARN = 2,
    LEVEL_ERROR = 3,
    LEVEL_FATAL = 4,
  };
};

struct Line {
  std::vector<maf_geometry::Pose2D> pose_array;
};

struct SystemMonitor {
  maf_std::Header header;
  uint8_t system_status;
  std::vector<std::string> warning_nodes_list;
  std::vector<std::string> error_nodes_list;

  enum : uint8_t {
    SYSTEM_STATUS_NORMAL_TYPE = 0,
    SYSTEM_STATUS_WARNING_TYPE = 1,
    SYSTEM_STATUS_ERROR_TYPE = 2,
  };
};

struct AsyncExportDetailsV2Input {
  int64_t key_num;
  std::vector<std::string> collect_type;
  std::vector<std::string> name;
  std::vector<std::string> export_type;
  uint64_t since;
  uint32_t latest;
};

struct CarConfig {
  maf_std::Header header;
  std::string lidar_type;
  std::string camera_type;
  std::string location_type;
};

struct BagRecorderTrigger {
  maf_std::Header header;
  std::vector<std::string> scenario_flag;
};

struct Segment {
  uint16_t begin_index;
  uint16_t end_index;
  uint8_t connection_type;
  uint8_t direction;
  uint8_t lane_type;
  int64_t lane_original_id;
  bool is_in_intersection;
  bool is_on_route;
};

struct TrafficLight {
  maf_std::Header header;
  int8_t left;
  int8_t right;
  int8_t mid;
  bool can_go_straight;
  bool can_right_turn;
  bool can_left_turn;
  bool can_u_turn;
  std::vector<maf_geometry::Pose2D> bbox_pts;
  uint8_t bbox_cnt;
};

struct ObjectArray {
  maf_std::Header header;
  std::vector<Object> object_array;
};

struct MapLane {
  std::vector<LanePoint> lane;
  uint32_t cur_idx;
  uint32_t global_idx;
};

struct ParamAggregatorConfigArray {
  maf_std::Header header;
  std::vector<ParamAggregatorConfig> config_array;
};

struct Refline {
  std::vector<Segment> segments;
  std::vector<RefLinePoint> points;
};

struct Lane {
  maf_std::Header header;
  Lanemark left_lanemark;
  Lanemark right_lanemark;
  Line mid_line;
  float width;
};

struct PredictionTrajectory {
  maf_std::Header header;
  float prob;
  float prediction_interval;
  uint16_t num_of_points;
  std::string intention;
  std::string source;
  std::vector<std::string> reserved;
  float const_vel_prob;
  float const_acc_prob;
  float still_prob;
  float coord_turn_prob;
  std::vector<PredictionTrajectoryPoint> trajectory;
};

struct FusionTrafficLightArray {
  maf_std::Header header;
  std::vector<FusionTrafficLight> traffic_light_array;
};

struct PredictionJunctionFeaturesArray {
  maf_std::Header header;
  std::vector<PredictionJunctionFeatures> junction_features_array;
};

struct ArsClusterArray {
  maf_std::Header header;
  std::vector<ArsCluster> cluster_array;
};

struct Trajectory {
  maf_std::Header header;
  Line path;
  std::vector<float> vel_array;
  std::vector<float> curvature;
  std::vector<float> accleration;
  std::vector<float> s;
  std::vector<float> relative_time;
  bool is_replan;
  int32_t type;
};

struct MTPCarObjectArray {
  maf_std::Header header;
  std::vector<MTPCarObject> object_array;
};

struct ArsObjectArray {
  maf_std::Header header;
  std::vector<ArsObject> object_array;
};

struct PredictionNonIntersectionFeatures {
  maf_std::Header header;
  bool is_feature_invalid;
  int32_t id;
  int32_t type;
  float x;
  float y;
  float s;
  float r;
  float yaw;
  float diff_theta_car_lane;
  float v;
  float v_s;
  float v_r;
  float acc;
  float acc_s;
  float acc_r;
  int32_t lane_index;
  float lane_width;
  float distance_left_lane;
  float distance_right_lane;
  float distance_mid_lane;
  float distance_left_road;
  float distance_right_road;
  float is_left_lane_solid;
  float is_right_lane_solid;
  float is_most_left_lane;
  float is_most_right_lane;
  std::vector<PredictionAroundCarFeature> around_car_feature;
  float diff_prev_yaw;
  float diff_prev_velocity;
  float diff_prev_velocity_s;
  float diff_prev_velocity_r;
  float diff_prev_acc;
  float diff_prev_acc_s;
  float diff_prev_acc_r;
  float diff_prev_distance_mid_lane;
  float diff_prev_multi_yaw;
  float diff_prev_multi_velocity;
  float diff_prev_multi_velocity_s;
  float diff_prev_multi_velocity_r;
  float diff_prev_multi_acc;
  float diff_prev_multi_acc_s;
  float diff_prev_multi_acc_r;
  float diff_prev_multi_distance_mid_lane;
  float is_left_half_lane;
  float is_right_half_lane;
  float previous_intention;
};

struct FusionObjectArray {
  maf_std::Header header;
  std::vector<FusionObject> object_array;
};

struct OptimalPlannerTrigger {
  maf_std::Header header;
  std::vector<float> v_target;
  std::vector<float> a_target;
  bool enable;
  bool stop_flag;
  float v_limit;
  float stop_distance;
  std::string which_lane;
  int32_t track_id;
  std::string lc_request;
  std::string lc_status;
  bool lane_borrow;
  int32_t lane_borrow_range;
  std::vector<AvdMsg> avd_info;
  float lat_offset;
};

struct FusionPolygonArray {
  maf_std::Header header;
  std::vector<FusionPolygon> object_array;
};

struct SrrClusterArray {
  maf_std::Header header;
  std::vector<SrrCluster> cluster_array;
};

struct MTPTrafficLightArray {
  maf_std::Header header;
  std::vector<MTPTrafficLight> items;
  std::vector<float> bbox_points;
};

struct PlanningDebug {
  maf_std::Header header;
  float lat_error;
  float lon_error;
  uint32_t flag_closelead;
  uint32_t flag_invalid;
  uint32_t flag_type0;
  uint32_t flag_softbrake;
  uint32_t flag_fastcutin;
  float v_set;
  float ds_set;
  float t_set;
  float v_limit;
  uint32_t num_yield;
  float vl_yield;
  float dsl_yield;
  uint32_t type_yield;
  int32_t id_yield;
  uint32_t tag_yield;
  uint32_t type_merge;
  float dis2cross;
  float dsl_overtake;
  uint32_t type_overtake;
  int32_t id_overtake;
  float dis2merge;
  float cutin_score;
  int32_t id_nudge;
  std::vector<AvdMsg> avd_info;
  double lead_one_drel;
  double lead_one_vrel;
  float v_target;
  float a_target;
  float dis_to_stop;
  std::vector<uint8_t> traffic_light_decision;
  std::vector<uint32_t> lon_follow_obstacles;
  std::vector<uint32_t> lon_overtake_obstacles;
  std::vector<uint32_t> lat_nudge_obstacles;
  uint32_t blocking_obstacle_id;
};

struct MapTask {
  std::vector<int8_t> current_tasks;
  std::vector<int8_t> left_lane_tasks;
  std::vector<int8_t> right_lane_tasks;
  std::vector<MapTaskRange> first_task_ranges;
};

struct MapPlanning {
  maf_std::Header header;
  std::vector<LaneBoundary> lane_boundaries;
  IntersectionManeuver next_maneuver;
  IntersectionManeuver last_maneuver;
  MergePoint next_merge;
  MapTask tasks;
  Refline current_refline;
  Refline left_left_refline;
  Refline left_refline;
  Refline right_refline;
  Refline right_right_refline;
  std::vector<BoundaryInfo> left_boundary_info;
  std::vector<BoundaryInfo> right_boundary_info;
  TrafficLightInfo traffic_light;
  std::vector<LaneInfo> lanes_info;
  double distance_to_stop_line;
  double distance_to_car_park;
  double distance_to_destination;
  double distance_to_left_waiting_area;
  double distance_to_left_waiting_area_stop_line;
  double distance_to_barrier_gap;
  double current_lane_speed_limit;
  SpeedChangePoint speed_change_point;
  bool is_in_map_area;
  bool is_in_intersection;
  uint8_t current_lane_index;
  int64_t mono_time;
  maf_geometry::Pose2D pose;
  maf_geometry::Pose enu;
  maf_sensor::NavSatFix wgs84_position;
};

struct PredictionNonIntersectionFeaturesArray {
  maf_std::Header header;
  std::vector<PredictionNonIntersectionFeatures>
      non_intersection_features_array;
};

struct MapLanes {
  maf_std::Header header;
  std::vector<MapLane> lanes;
  int32_t cur_lane_id;
  Mdf next_mdf;
};

struct LaneArray {
  maf_std::Header header;
  std::vector<Lane> lane_array;
  uint8_t current_lane_idx;
};

struct PredictionObject {
  uint32_t id;
  uint32_t type;
  float position_x;
  float position_y;
  float length;
  float width;
  float speed;
  float yaw;
  float velocity_x;
  float velocity_y;
  float acc;
  float pos_x_sigma;
  float pos_y_sigma;
  float rel_position_x;
  float rel_position_y;
  float rel_speed_x;
  float rel_speed_y;
  float rel_theta;
  std::vector<std::string> reserved;
  std::vector<PredictionTrajectory> trajectory_array;
};

struct Planning {
  maf_std::Header header;
  float v_target;
  float a_target;
  float dis_to_stop;
  std::vector<float> v_array;
  std::vector<float> a_array;
  std::vector<uint8_t> traffic_light_decision;
  Trajectory traj;
  std::vector<maf_geometry::Pose2D> pose_array;
  std::vector<uint32_t> lon_follow_obstacles;
  std::vector<uint32_t> lon_overtake_obstacles;
  std::vector<uint32_t> lat_nudge_obstacles;
  uint32_t blocking_obstacle_id;
};

struct PredictionObjectArray {
  maf_std::Header header;
  std::vector<PredictionObject> object_prediction_array;
};

} // namespace maf_base

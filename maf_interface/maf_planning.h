#ifndef MAF_INTERFACE_MAF_PLANNING_H
#define MAF_INTERFACE_MAF_PLANNING_H

#include "maf_interface/maf_actionlib.h"
#include "maf_interface/maf_geometry.h"
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_planning {

/*!
 * \brief Gear Type
 */
struct Gear {
  uint8_t value;

  enum : uint8_t {
    NONE = 0,    //!< Gear Type None
    PARK = 1,    //!< Gear Type Park
    REVERSE = 2, //!< Gear Type Reverse
    NEUTRAL = 3, //!< Gear Type Netural
    DRIVE = 4,   //!< Gear Type Drive
    LOW = 5,     //!< Gear Type Low
  };
  serdes1(value)
};

/*!
 * \brief Struct of Polygon2f
 */
struct Polygon2f {
  /*!
   * @brief Vector of Polygon2f
   * \vec_max_size{100}
   */
  std::vector<maf_geometry::Point32> points;
  serdes1(points)
};

/*!
 * \brief Failure Reason Enum Info
 */
struct FailureReasonEnum {
  uint8_t value;

  enum : uint8_t {
    INFEASIBLE = 1, //!< Infeasible
    TIMEOUT = 2,    //!< TimeOut
    EXCEPTION = 4,  //!< Exeception
  };
};

/*!
 * \brief SBP Request Meta Info
 */
struct SBPResultMeta {
  uint64_t timestamp_us; //!< Timestamp info
};

/*!
 * \brief SBP Result Extra Info
 */
struct SBPResultExtra {
  uint8_t available;
  std::string json;

  enum : uint8_t {
    JSON = 1, //!< When Json is available
  };
};

/*!
 * \brief Obstacle Type Enum Info
 */
struct ObstacleTypeEnum {
  uint8_t value;

  enum : uint8_t {
    PHYSICAL = 1,     //!< Physical Obstacle
    LOGICAL = 2,      //!< Logical Obstacle
    PARKING_SLOT = 4, //!< parking Lot
  };
};

/*!
 * \brief Struct of Box2f, Describing center and size
 */
struct Box2f {
  maf_geometry::Point32 center; //!< Center point ox box
  float heading;                //!< Heading point ox box
  float length;                 //!< Length point ox box
  float width;                  //!< Width point ox box
};

/*!
 * \brief Struct of Task Params
 */
struct TaskParams {
  uint8_t verbose;                 //!< Verbose value
  uint8_t planning_core;           //!< Planning core
  uint8_t footprint_model;         //!< Foot print model value
  uint8_t footprint_model_precise; //!< Foot print model precise value
  float lon_inflation;             //!< Lon inflation value
  float lat_inflation;             //!< Lat inflation value
  float inflation_rearview_mirror; //!< Rearview mirror inflation value
  float shrink_ratio_for_lines;    //!< Shrink ratio for lines
  float max_steer_angle;           //!< Max steer angle
  float max_steer_angle_rate;      //!< Max steer angle rate
  float max_steer_angle_rear;      //!< Max steer angle rear
  float max_steer_angle_rate_rear; //!< Max steer angle rate rear
  bool enable_analytic_expansion;  //!< Whether enable analytic expansion
  float analytic_expansion_end_size_threshold; //!< Analytic expansion end size
                                               //!< threshold
  int8_t force_analytic_expansion_end_direction; //!< Force analytic expansion
                                                 //!< end direction
  bool enable_multiple_steer_modes;          //!< Enable multiple steer modes
  float xy_grid_resolution;                  //!< X and y grid resolution
  float phi_grid_resolution;                 //!< Phi grid resolution
  float grid_dijkstra_xy_resolution;         //!< Grid dijkstra xy resolution
  float traj_forward_penalty;                //!< Traj forward penalty
  float traj_back_penalty;                   //!< Traj back penalty
  float traj_gear_switch_penalty;            //!< Traj gear switch penalty
  float traj_steer_penalty;                  //!< Traj steer penalty
  float traj_steer_change_penalty;           //!< Traj steer change penalty
  float step_size;                           //!< Step size
  uint8_t next_node_num;                     //!< Num of next node
  float max_iteration;                       //!< Max iteration
  float holonomic_with_obs_heuristic;        //!< Holonomic with obs heuristic
  float non_holonomic_without_obs_heuristic; //!< Non holonomic without obs
                                             //!< heuristic
};

/*!
 * \brief SBP Request Meta Info
 */
struct SBPRequestMeta {
  uint64_t timestamp_us;
};

/*!
 * \brief Struct of Polyline2f
 */
struct Polyline2f {
  /*!
   * @brief Vector of Polyline2f
   * \vec_max_size{100}
   */
  std::vector<maf_geometry::Point32> points;
};

/*!
 * \brief Request Enum Info
 */
struct RequestEnum {
  uint8_t value;

  enum : uint8_t {
    START_TASK = 1, //!< Start Task
    ABORT_TASK = 2, //!< Abort Task
  };
};

/*!
 * \brief SBP Request Extra Info
 */
struct SBPRequestExtra {
  uint8_t available;
  std::string json;

  enum : uint8_t {
    JSON = 1, //!< When Json is available
  };

  serdes2(available, json)
};

/*!
 * \brief Stationary Obstacle Car Info
 */
struct StationaryObstacleCarInfo {
  bool close_to_obstacle;          //!< Whether front car is accident
  double steering_wheel_rad_limit; //!< Reference Steering Wheel When Accident
                                   //!< Car (rad)
  serdes2(close_to_obstacle, steering_wheel_rad_limit)
};

/*!
 * \brief Point2d of Location
 */
struct Point2d {
  double x; //!< x axis
  double y; //!< y axis
  serdes2(x,y)
};

/*!
 * \brief Acceleration Range of Desired Acceleration
 */
struct AccelerationRange {
  /**
   * @brief Max acceleration
   * \unit{m/s2}
   * \value_min{0}
   * * \value_max{10}
   */
  double max;
  /**
   * @brief Min acceleration
   * \unit{m/s2}
   * \value_min{-10}
   * \value_max{0}
   */
  double min;

  serdes2(max,min)
};

/*!
 * \brief Polynomial of Desired Trajectory
 */
struct Curve {
  /*!
   * @brief Polynomials
   * \vec_max_size{10}
   */
  std::vector<double> polynomial;
  serdes1(polynomial)
};

/*!
 * \brief Acceleration Point of Desired Acceleration
 */
struct AccelerationPoint {
  /**
   * @brief Desired acceleration of point
   * \unit{m/s2}
   * \value_min{-10}
   * \value_min{10}
   */
  double acc;
  /**
   * @brief Desired jerk of point
   * \unit{m/s3}
   * \value_min{-20}
   * \value_min{20}
   */
  double jerk;

  serdes2(acc,jerk)
};

struct VelocityPoint {
  /**
   * @brief target velocity
   * \unit{m/s}
   * \value_min{0}
   */
  double target_velocity;
  /**
   * @brief relative time from planning start
   * \unit{s}
   * \value_min{0}
   */
  double relative_time;
  /**
   * @brief relative distance from planning start
   * \unit{m}
   * \value_min{0}
   */
  double distance;

  serdes3(target_velocity, relative_time, distance)
};

/*!
 * \brief Longitudinal Decision Info
 */
struct LongitudinalDecision {
  /*!
   * @brief Follow obstacles
   * \vec_max_size{10}
   */
  std::vector<int32_t> follow_obstacles;
  /*!
   * @brief Overtake obstacles
   * \vec_max_size{10}
   */
  std::vector<int32_t> overtake_obstacles;
  /*!
   * @brief Longitudinal nudge obstacles
   * \vec_max_size{10}
   */
  std::vector<int32_t> lon_nudge_obstacles;
  /*!
   * @brief Lead obstacles
   * \vec_max_size{10}
   */
  std::vector<int32_t> lead_obstacles;
  /*!
   * @brief Temp lead obstacles
   * \vec_max_size{10}
   */
  std::vector<int32_t> tlead_obstacles;
  /*!
   * @brief Lane change gap info
   * \vec_max_size{10}
   */
  std::vector<int32_t> lc_gap_info;
  int32_t id_yield_closest;    //!< Closest yield target id
  int32_t id_overtake_closest; //!< Closest overtake target id
  int32_t tag_yield;           //!< Yield target tag
  int32_t N_hard;              //!< Variable of N_hard
  float dt_yield;              //!< Variable of dt_yield
  float dt_set;                //!< Variable of dt_set
  float ds_set;                //!< Variable of ds_set
  float ds_yield;              //!< Variable of ds_yield
  float ds_overtake;           //!< Variable of ds_overtake
  float v_set;                 //!< Variable of v_set
  float v_yield_closest;       //!< Cloest yield target velocity
  float v_overtake_closest;    //!< Cloest overtake target velocity
  float v_lim;                 //!< velocity limit
  float v_lim_map;             //!< velocity limit of map
  float a_set;                 //!< Variable of a_set
  float dis_close;             //!< Variable of dis_close
  /*!
   * @brief Debug info of ds_lon
   * \vec_max_size{10}
   */
  std::vector<float> ds_lon;
  /*!
   * @brief Debug info of ttc_lon
   * \vec_max_size{10}
   */
  std::vector<float> ttc_lon;
  /*!
   * @brief Debug info of dt_lon
   * \vec_max_size{10}
   */
  std::vector<float> dt_lon;
  /*!
   * @brief Debug info of s_max
   * \vec_max_size{10}
   */
  std::vector<float> s_max;
  /*!
   * @brief Debug info of d_safe
   * \vec_max_size{10}
   */
  std::vector<float> d_safe;
  /*!
   * @brief Debug info of ds_flag
   * \vec_max_size{10}
   */
  std::vector<float> ds_flag;
  /*!
   * @brief Debug info of v_ref
   * \vec_max_size{10}
   */
  std::vector<float> v_ref;
  /*!
   * @brief Debug info of weights
   * \vec_max_size{10}
   */
  std::vector<float> weights;
  bool enable_prebrake; //!< Whether prebrake enabled
  bool enable_preacc;   //!< Whether prearcc enabled
  bool need_aeb;        //!< Whether aeb enabled
  bool pnc_start;       //!< Whether need start
  bool pnc_stop;        //!< Whether need start
  bool blocked;         //!< Whether is blocked

  //3 x 12
  serdes36(follow_obstacles, overtake_obstacles,lon_nudge_obstacles, //1
           lead_obstacles, tlead_obstacles, lc_gap_info,             //2
           id_yield_closest, id_overtake_closest, tag_yield,         //3
           N_hard, dt_yield, dt_set,                                 //4
           ds_set, ds_yield,ds_overtake,                             //5
           v_set, v_yield_closest, v_overtake_closest,               //6
           v_lim, v_lim_map, a_set,                                  //7
           dis_close, ds_lon, ttc_lon,                               //8
           dt_lon, s_max, d_safe,                                    //9
           ds_flag, v_ref, weights,                                  //a
           enable_prebrake, enable_preacc, need_aeb,                 //b
           pnc_start, pnc_stop, blocked)                             //c
};

/*!
 * \brief TrafficLight Decision Info
 */
struct TrafficLightDecision {
  int32_t traffic_light_state; //!< Traffic light state
  int32_t stop_reason;         //!< Traffic light stop reason
  bool stop_flag;              //!< Traffic light stop flag
  bool is_passed_stop_line;    //!< Whether passed stop line
  bool faster_pass;            //!< Whether faster pass
  serdes5(traffic_light_state, stop_reason,
          stop_flag, is_passed_stop_line,
          faster_pass)
};

/*!
 * \brief Turn Signal Source
 */
struct TurnSignalSource {
  uint8_t value;

  enum : uint8_t {
    NONE = 0, //!< No Lane Change
    MAP = 1,  //!< Map Lane Change
    LC = 2,   //!< Active Lane Change
    LB = 3,   //!< Lane Borrow
  };
  serdes1(value)
};

/*!
 * \brief Turn Signal Command
 */
struct TurnSignal {
  uint8_t value;

  enum : uint8_t {
    NONE = 0,              //!< No Turn Signal
    LEFT = 1,              //!< Left Turn Signal
    RIGHT = 2,             //!< Right Turn Signal
    EMERGENCY_FLASHER = 3, //!< Emergent Turn Signal
  };
  serdes1(value)
};

/*!
 * \brief Maneuver Gear of Lane Changing
 */
struct ManeuverGear {
  uint8_t value;

  enum : uint8_t {
    SLOW = 0,     //!< Slow Gear
    NORMAL = 1,   //!< Normal Gear
    FAST = 2,     //!< Fast Gear
    ACCIDENT = 3, //!< Accident Gear
  };
  serdes1(value)
};

/*!
 * \brief Plan Meta Info
 */
struct PlanMeta {
  uint64_t timestamp_us;          //!< Timestamp when plan result is sent
  uint64_t plan_timestamp_us;     //!< Timestamp when plan result is generated
  std::string plan_strategy_name; //!< Plan Strategy Name
  serdes3(timestamp_us, plan_timestamp_us, plan_strategy_name)
};

/*!
 * \brief Plan Extra Info
 */
struct PlanExtra {
  uint8_t available;
  std::string version;
  std::string json;

  enum : uint8_t {
    VERSION = 1, //!< When Version is available
    JSON = 2,    //!< When Json is available
  };

  serdes3(available, version, json)
};

/*!
 * \brief Solver Status Info
 */
struct SolverStatus {
  /*!
   * @brief Debug info of lateral montion status
   * \vec_max_size{20}
   */
  std::vector<uint8_t> lat_motion_status;
  /*!
   * @brief Debug info of lateral solver status
   * \vec_max_size{20}
   */
  std::vector<uint8_t> lat_solver_status;
  /*!
   * @brief Debug info of longitudinal montion status
   * \vec_max_size{20}
   */
  std::vector<uint8_t> lon_motion_status;
  /*!
   * @brief Debug info of longitudinal solver status
   * \vec_max_size{20}
   */
  std::vector<uint8_t> lon_solver_status;

  serdes4(lat_motion_status, lat_solver_status,
          lon_motion_status, lon_solver_status)
};

/*!
 * \brief Warning Status Info
 */
struct WarningStatus {
  uint64_t warning_status;

  enum : uint64_t {
    NARROW_ROAD = 1,                 //!< Narrow Road Passable
    OVERTAKE_CUTIN = 2,              //!< Overtake Cutin Car Warning
    OVERTAKE_WISDOM_DODGE = 4,       //!< Overtake When Wisdom Dodge
    LANE_CHANGE_REMIND_LANE_END = 8, //!< Lane Change Remind When Lane End
    LANE_CHANGE_REMIND_CONSTRUCTION =
        16, //!< Lane Change Reminder When Construction
    LANE_CHANGE_REMIND_ACCIDENT =
        32,                         //!< Lane Change Reminder When Accident Car
    DECEL_WARNING = 64,             //!< Decel Warning for Dlp
    PEDESTRIAN_WARNING = 128,       //!< Warning for Pedestrian
    LANE_CHANGE_START = 256,        //!< Lane Change Start Warning
    LANE_CHANGE_STOP = 512,         //!< Lane Change End Warning
    FROINT_COLLISION = 1024,        //!< Front Collision Warning
    REAR_COLLISION = 2048,          //!< Rear Collision Warning
    LANE_DEPARTURE = 4096,          //!< Lane Departure Warning
    PARKING_LOT_UNAVALIABLE = 8192, //!< Parking Lot Unavailable Warning
    AVOID_IN_LANE = 16384,          //!< Avoid in Lane
  };

  serdes1(warning_status)
};

/*!
 * \brief Plan Algorithm Status Info
 */
struct PlanAlgorithmStatus {
  uint64_t scene;
  uint64_t action;
  uint64_t action_status;
  uint64_t function;
  uint64_t function_status;
  uint64_t Reason;
  uint64_t source;

  enum : uint64_t {
    NORMAL_ROAD = 1,            //!< driving in normal road
    INTERSECT = 2,              //!< driving in intersect
    PARKING_SVP = 4,            //!< parking with hdmap
    PARKING_LVP = 8,            //!< parking with learned map
    PARKING_APA = 16,           //!< parking with only perception
    REMOTE_CONTROL = 32,        //!< vehicle is under remote control
    LANE_CHANGE_LEFT = 1,       //!< lane change left
    LANE_CHANGE_RIGHT = 2,      //!< lane change right
    LANE_BORROW_LEFT = 4,       //!< lane borrow left
    LANE_BORROW_RIGHT = 8,      //!< lane borrow right
    INTERSECT_GO_STRAIGHT = 16, //!< go straight in intersect scene
    INTERSECT_TURN_LEFT = 32,   //!< turn left in intersect scene
    INTERSECT_TURN_RIGHT = 64,  //!< turn right in intersect scene
    INTERSECT_U_TURN = 128,     //!< u-turn in intersect scene
    LANE_BORROW_IN_NON_MOTORIZED_LANE =
        256,                    //!< lane borrow in non-motorized-lane
    LANE_KEEP = 512,            //!< lane keep
    PULL_OVER = 1024,           //!< pull over
    OPEN_SPACE_PASS = 2048,     //!< openspace pass
    PARKING = 4096,             //!< parking in progress
    SEARCH = 1,                 //!< search vacant parking slot
    CRUISE = 2,                 //!< cruise to a destination
    APA = 4,                    //!< park in
    APOA = 8,                   //!< park out
    WAIT = 16,                  //!< wait for further request
    LANE_CHANGE_WAITING = 1,    //!< waiting the lane changing
    LANE_CHANGEING = 2,         //!< changing lane
    LANE_CHANGE_BACK = 4,       //!< lane changing back
    LANE_BORROWING = 8,         //!< borrowing lane
    LANE_BORROW_BACK = 16,      //!< lane borrow back to the back of target car
    LANE_BORROW_RETURN = 32,    //!< lane borrow back to the front of target car
    LANE_BORROW_SUSPEND = 64,   //!< lane borrow suspend
    OPEN_SPACE_STANDBY = 128,   //!< hybrid A* standby
    OPEN_SPACE_PLANNING = 256,  //!< hybrid A* plan
    OPEN_SPACE_RUNNING = 512,   //!< hybrid A* run
    OPEN_SPACE_FALLBACK = 1024, //!< hybrid A* fallback
    LANE_KEEPING = 2048,        //!< lane keeping
    SIDE_PASS = 4096,           //!< side pass
    RUNNING = 1,                //!< function status running
    SUCCEEDED = 2,              //!< function status successed
    FAILED = 4,                 //!< function status failed
    PAUSED = 8,                 //!< function status paused
    NONE = 1,                   //!< reason none
    NO_FRONT_VIEW = 2,          //!< reason no front view
    NO_SIDE_VIEW = 4,           //!< reason no side view
    SIDE_VIEW_BACK = 8,         //!< reason side view back
    FRONT_VIEW_BACK = 16,       //!< reason front view back
    BACK_CNT_BELOW_THRESHOLD = 32,      //!< reason back cnt below threshold
    EXCEED_MOVE_THRESHOLD = 64,         //!< reason exceed move threshold
    NOT_OPTIMAL = 128,                  //!< reason not optimal
    INVALID_GAP_SELECTION = 256,        //!< reason invalid gap selection
    INVALID_REQUEST = 512,              //!< reason invalid request
    CANCELED = 1024,                    //!< reason canceled
    FRONT_VIEW_FORBID = 2048,           //!< reason front view forbid
    SIDE_VIEW_FORBID = 4096,            //!< reason side view forbid
    EXCEED_GAP_REMOTE_CAR = 1024,       //!< reason exceed gap remote car
    NOT_EXCEED_GAP_PROXIMAL_CAR = 2048, //!< reason not exceed gap proximal car
    NO_VALID_GAP = 4096,                //!< reason no valid gap
    TARGET_LANE_INVALID = 8192,         //!< reason target lane invalid
    MAP_REQUEST = 2,                    //!< map request
    ALC_REQUEST = 4,                    //!< act request
    ALC_BY_MERGE_REQUEST = 8,           //!< act by merge request
    ALC_BY_YPOINT_REQUEST = 16,         //!< act by y point request
    INT_REQUEST = 32,                   //!< int request
    LB_REQUEST = 64,                    //!< lane borrow request
  };

  serdes7(scene,action,action_status,
          function, function_status, Reason,
          source)
};

/*!
 * \brief Steer Mode Enum Info
 */
struct SteerModeEnum {
  uint8_t value;

  enum : uint8_t {
    NORMAL = 0,      //!< Normal Mode
    TWIST = 1,       //!< Twist Mode
    TRANSLATION = 2, //!< Translation Mode
  };
  serdes1(value)
};

/*!
 * \brief Gear Command
 */
struct GearCommand {
  uint8_t available;
  Gear gear_data; //!< Gear data

  enum : uint8_t {
    GEAR_DATA = 1, //!< When Gear Data is Available
  };
  serdes2(available, gear_data)
};

/*!
 * \brief Boundary Info, Describing Boundary
 */
struct Boundary {
  uint8_t available;
  Box2f box;         //!< Box2f info of boundary
  Polygon2f polygon; //!< Polygon2f info of boundary

  enum : uint8_t {
    BOX = 1,     //!< When Box is Available
    POLYGON = 2, //!< When Polygon is Available
  };
};

/*!
 * \brief Task Info, Including Request Enum and Goal ID
 */
struct TaskInfo {
  uint8_t available;
  RequestEnum request;           //!< Request Enum
  maf_actionlib::GoalID goal_id; //!< Goal ID

  enum : uint8_t {
    REQUEST = 1, //!< When Request Enum is Available
    GOAL_ID = 2, //!< When Goal ID is Available
  };
};

/*!
 * \brief Obstacle Info, Including Obstacle Type, Box, Polygon, Polyline and
 * Point
 */
struct Obstacle {
  uint8_t available;
  ObstacleTypeEnum obstacle_type; //!< Obstacle Type Enum
  Box2f box;                      //!< Box2f info of Obstacle
  Polygon2f polygon;              //!< Polygon2f info of Obstacle
  Polyline2f polyline;            //!< Polyline2f info of Obstacle
  maf_geometry::Point32 point;    //!< Point32 info of Obstacle

  enum : uint8_t {
    BOX = 1,      //!< When Box is Available
    POLYGON = 2,  //!< When Polygon is Available
    POLYLINE = 4, //!< When Polyline is Available
    POINT = 8,    //!< When Point is Available
  };
};

/*!
 * \brief SBP Vehicle State, Including Pose, Steer, Steer Mode and Gear State
 */
struct SBPVehicleState {
  maf_geometry::Pose2D pose; //!< Ego pose
  float steer;               //!< Ego steer
  SteerModeEnum steer_mode;  //!< Current steer mode
  int8_t gear;               //!< Ego gear state
};

/*!
 * \brief Emergency Info, Including Jerk Factor and Stationary Obstacle Car Info
 */
struct Emergency {
  uint8_t available;
  double jerk_factor; //!< Jerk Factor
  StationaryObstacleCarInfo
      stationary_obstacle_car_info; //!< Stationary obstacle car info

  enum : uint8_t {
    JERK_FACTOR = 1, //!< When Jerk Factor is Available
    STATIONARY_OBSTACLE_CAR_INFO =
        2, //!< When Stationary Obstacle Car Infois Available
  };

  serdes3(available, jerk_factor, stationary_obstacle_car_info)
};

/*!
 * \brief Struct of Pose2d
 */
struct Pose2d {
  Point2d position; //!< Position of pose
  double theta;     //!< Theta of pose
  serdes2(position, theta)
};

/*!
 * \brief Desired Acceleration, Including Range and Sequence
 */
struct Acceleration {
  uint8_t available;
  AccelerationRange range_limit; //!< Range Limit of Real Time Planning
  /*!
   * @brief Target Sequence of Long Term Planning
   * \vec_max_size{320}
   */
  std::vector<AccelerationPoint> acc_points;

  enum : uint8_t {
    RANGE_LIMIT = 1, //!< When Range Limit is Available
    ACC_POINTS = 2,  //!< When Acceleration Point is Available
  };

  serdes3(available, range_limit, acc_points)
};

/*!
 * \brief Desired Velocity, Including Value, Sequence and Cruise
 */
struct Velocity {
  uint8_t available;
  double target_value; //!< Target Value of Real Time Planning
  /*!
   * @brief Target Sequence of Long Term Planning
   * \vec_max_size{320}
   */
  std::vector<VelocityPoint> vel_points;
  double cruise_velocity; //!< Cruise Velocity

  enum : uint8_t {
    TARGET_VALUE = 1,    //!< When Target Value is Available
    VEL_POINTS = 2,      //!< When Velocity Point is Available
    CRUISE_VELOCITY = 4, //!< When Cruise Velocity is Available
  };

  serdes4(available, target_value, vel_points, cruise_velocity)
};

/*!
 * \brief Path Point of Desired Trajectory
 */
struct PathPoint {
  Point2d position_enu;        //!< Enu position of path point
  double heading_yaw;          //!< Heading yaw of path point
  double curvature;            //!< Curvature of path point
  double path_follow_strength; //!< Path follow strength of path point
  serdes4(position_enu, heading_yaw, curvature, path_follow_strength)
};

/*!
 * \brief Frenet State Info
 */
struct FrenetState {
  Point2d position_frenet; //!< Frenet position
  double dl;               //!< Dl of frenet
  double ddl;              //!< Ddl of frenet

  serdes3(position_frenet, dl,ddl)
};

/*!
 * \brief Avoid Messege
 */
struct AvdMsg {
  int32_t id;                //!< Avoid target id
  std::string property;      //!< Avoid target property
  bool ignore;               //!< Whether avoid target is ignored
  std::string avd_direction; //!< Avoid target direction
  int32_t avd_priority;      //!< Avoid target priority
  float blocked_time_begin;  //!< Avoid target blocked time begin
  float blocked_time_end;    //!< Avoid target blocked time end
  /*!
   * @brief Polygon of avoid target
   * \vec_max_size{100}
   */
  std::vector<Point2d> polygon;

  serdes8(id,property,ignore,avd_direction,
          avd_priority,blocked_time_begin,blocked_time_end,polygon)
};

/*!
 * \brief Turn Signal Command
 */
struct TurnSignalCommand {
  uint8_t available;
  TurnSignal turn_signal_data;         //!< Turn signal data
  TurnSignalSource turn_signal_source; //!< Turn signal source

  enum : uint8_t {
    TURN_SIGNAL_DATA = 1,   //!< When Turn Signal is Available
    TURN_SIGNAL_SOURCE = 2, //!< When Turn Signal Source is Available
  };

  serdes3(available, turn_signal_data, turn_signal_source)
};

/*!
 * \brief Comfort Info
 */
struct Comfort {
  uint8_t available;
  ManeuverGear maneuver_gear; //!< Maneuver gear

  enum : uint8_t {
    MANEUVER_GEAR = 1, //!< When Maneuver Gear is Available
  };

  serdes2(available, maneuver_gear)
};

/*!
 * \brief Plan Status Info, Including Plan Algorithm Status, Warning Status,
 * Solver Status, Steer Mode Enum and Hdmap Valid
 */
struct PlanStatus {
  uint8_t available;
  PlanAlgorithmStatus algorithm_status; //!< Plan Algorithm Status
  WarningStatus warning_status;         //!< Warning Status
  SolverStatus solver_status;           //!< Solver Status
  SteerModeEnum steer_mode;             //!< Steer Mode Enum
  bool hdmap_valid;

  enum : uint8_t {
    ALGORITHM_STATUS = 1, //!< When Plan Algorithm Status is Available
    WARNING_STATUS = 2,   //!< When Warning Status is Available
    SOLVER_STATUS = 4,    //!< When Solver Status is Available
    STEER_MODE = 8,       //!< When Steer Mode Enum is Available
  };

  serdes6(available, algorithm_status,
          warning_status, solver_status,
          steer_mode, hdmap_valid)
};

/*!
 * \brief Problem Config Info
 */
struct ProblemConfig {
  uint8_t available;
  TaskParams params;          //!< Task Params
  std::string params_string;  //!< String of params
  SBPVehicleState init_state; //!< Init state
  SBPVehicleState goal_state; //!< Goal state

  enum : uint8_t {
    PARAMS = 1,        //!< When Task Params is Available
    PARAMS_STRING = 2, //!< When Params String is Available
    INIT_STATE = 4,    //!< When Init State is Available
    TARGET_STATE = 8,  //!< When Goal State is Available
  };
};

/*!
 * \brief Result Info of SBP Result
 */
struct Result {
  uint8_t available;
  int64_t iteration_times; //!< Iteration times
  /*!
   * @brief Target trajectory
   * \vec_max_size{256}
   */
  std::vector<SBPVehicleState> trajectory;
  uint8_t num_segments;             //!< Num of segments
  FailureReasonEnum failure_reason; //!< Failure reason

  enum : uint8_t {
    ITERATION_TIMES = 1, //!< When Iteration Times is Available
    TRAJECTORY = 2,      //!< When Trajectory is Available
    NUM_SEGMENTS = 4,    //!< When Num Segments is Available
    FAILURE_REASON = 8,  //!< When Failure Reason is Available
  };
};

/*!
 * \brief Environment Data Info, Including Obstacles
 */
struct EnvironmentData {
  uint8_t available;
  /*!
   * @brief Obstacles info
   * \vec_max_size{100}
   */
  std::vector<Obstacle> obstacles;

  enum : uint8_t {
    OBSTACLES = 1, //!< When Obstacles is Available
  };
};

/*!
 * \brief Desired Trajectory Info, Including Curve, Path, Velocity, Acceleration
 * and Frenet State
 */
struct Trajectory {
  uint8_t available;
  Curve polynomial_curve; //!< Polynomial curve of trajectory
  /*!
   * @brief Path points of trajectory
   * \vec_max_size{320}
   */
  std::vector<PathPoint> path;
  Velocity velocity;         //!< Target velocity of trajectory
  Acceleration acceleration; //!< Target acceleration of trajectory
  /*!
   * @brief Target frenet state of trajectory
   * \vec_max_size{320}
   */
  std::vector<FrenetState> frenet_state;

  enum : uint8_t {
    POLYNOMIAL_CURVE = 1, //!< When Polynomial Curve is Available
    PATH = 2,             //!< When Path is Available
    VELOCITY = 4,         //!< When Velocity is Available
    ACCELERATION = 8,     //!< When Acceleration is Available
    FRENET_STATE = 16,    //!< When Frenet State is Available
  };

  serdes6(available, polynomial_curve, path, 
          velocity, acceleration, frenet_state)
};

/*!
 * \brief Lateral Decision Info
 */
struct LateralDecision {
  /*!
   * @brief Nudge obstacles
   * \vec_max_size{20}
   */
  std::vector<int32_t> nudge_obstacles;
  /*!
   * @brief Latral motion decider info
   * \vec_max_size{50}
   */
  std::vector<int32_t> lat_motion_decider;
  /*!
   * @brief Desired offset
   * \vec_max_size{50}
   */
  std::vector<float> desire_offset;
  /*!
   * @brief Avoid info
   * \vec_max_size{20}
   */
  std::vector<AvdMsg> avd_info;
  bool accident_ahead;          //!< Whether accident ahead
  bool disable_left;            //!< Whether disable left
  bool disable_right;           //!< Whether disable right
  bool enable_left;             //!< Whether enable left
  bool enable_right;            //!< Whether enable right
  bool lane_borrow;             //!< Whether lane borrow
  int32_t lane_borrow_range;    //!< Lane borrow range
  float lat_offset;             //!< Lat offset
  int32_t fix_refline_index;    //!< Fix refline index
  int32_t origin_refline_index; //!< Origin refline index
  int32_t target_refline_index; //!< Target refline index
  bool sb_Blane;                //!< Whether sb_Blane
  bool sb_lane;                 //!< Whether sb_lane
  /*!
   * @brief Avd_car_past_fisrt info
   * \vec_max_size{20}
   */
  std::vector<float> avd_car_past_fisrt;
  /*!
   * @brief Avd_car_past_second info
   * \vec_max_size{20}
   */
  std::vector<float> avd_car_past_second;
  /*!
   * @brief Avd_sp_car_past_first info
   * \vec_max_size{20}
   */
  std::vector<float> avd_sp_car_past_first;
  /*!
   * @brief Avd_sp_car_past_second info
   * \vec_max_size{20}
   */
  std::vector<float> avd_sp_car_past_second;
  /*!
   * @brief Alc car id info
   * \vec_max_size{20}
   */
  std::vector<int32_t> alc_car_id;
  /*!
   * @brief Lb car id
   * \vec_max_size{20}
   */
  std::vector<int32_t> lb_car_id;
  int32_t lc_back_id;    //!< Lc back target id
  int32_t lc_invalid_id; //!< Lc invalid target id
  float dl_lat;          //!< Value of dl_lat
  float ttc_lat;         //!< Value of ttc_lat
  float dt_lat;          //!< Value of dt_lat
  bool fvfdead;          //!< Whether fvf is dead
  bool svfdead;          //!< Whether svf is dead
  bool premoving;        //!< Whether premoving
  float premove_dist;    //!< Premoving dist
  /*!
   * @brief Initial val info
   * \vec_max_size{50}
   */
  std::vector<Pose2d> init_val;
  /*!
   * @brief Via points
   * \vec_max_size{50}
   */
  std::vector<Pose2d> via_points;

  // 3 x 11 + 1
  serdes34(nudge_obstacles, lat_motion_decider, desire_offset,     //1
           avd_info, accident_ahead, disable_left,                 //2
           disable_right, enable_left, enable_right,               //3
           lane_borrow, lane_borrow_range, lat_offset,             //4
           fix_refline_index,origin_refline_index, target_refline_index,       //5
           sb_Blane, sb_lane, avd_car_past_fisrt,                              //6
           avd_car_past_second, avd_sp_car_past_first, avd_sp_car_past_second, //7
           alc_car_id, lb_car_id, lc_back_id,                      //8
           lc_invalid_id, dl_lat, ttc_lat,                         //9
           dt_lat, fvfdead, svfdead,
           premoving, premove_dist, init_val,
           via_points)
};

/*!
 * \brief TaskStatus, Including Goal ID, Goal Status and Result
 */
struct TaskStatus {
  uint8_t available;
  uint64_t plan_timestamp_us;
  maf_actionlib::GoalID goal_id;         //!< Goal ID
  maf_actionlib::GoalStatus goal_status; //!< Goal Status
  Result result;                         //!< Planning Result

  enum : uint8_t {
    PLAN_TIMESTAMP_US = 1, //!< When Plan Timestamp Us is Available
    GOAL_ID = 2,           //!< When Goal ID is Available
    GOAL_STATUS = 4,       //!< When Goal Status is Available
    RESULT = 8,            //!< When Result is Available
  };
};

/*!
 * \brief TaskConfig Info, Including ProblemConfig, Environment Data and
 * Boundary
 */
struct TaskConfig {
  uint8_t available;
  ProblemConfig problem_config;     //!< Problem Config
  EnvironmentData environment_data; //!< Environment Data
  Boundary boundary;                //!< Boundary info

  enum : uint8_t {
    CONFIG = 1,           //!< When Problem Config is Available
    ENVIRONMENT_DATA = 2, //!< When Environment Data is Available
    BOUNDARY = 4,         //!< When Boundary is Available
  };
};

/*!
 * \brief Decision Info, Including Longitudinal Decision, Lateral Decision and
 * Traffic Light Decision
 */
struct Decision {
  uint8_t available;
  LongitudinalDecision lon_decision;           //!< Longitudinal Decision Info
  LateralDecision lat_decision;                //!< Lateral Decision Info
  TrafficLightDecision traffic_light_decision; //!< TrafficLight Decision Info

  enum : uint8_t {
    LON_DECISION = 1,           //!< When Longitudinal Decision is Available
    LAT_DECISION = 2,           //!< When Lateral Decision is Available
    TRAFFIC_LIGHT_DECISION = 4, //!< When Traffic Light Decision is Available
  };

  serdes3(available, lon_decision, lat_decision)//, traffic_light_decision)
};

/*!
 * \brief SBP Request Message sent by Main Planning Module
 */
struct SBPRequest {
  maf_std::Header header;
  SBPRequestMeta meta;    //!< Meta info
  TaskInfo task_info;     //!< Task Info
  TaskConfig task_config; //!< Task Config
};

/*!
 * \brief SBP Result Message Returned by Seach Based Planning Module
 */
struct SBPResult {
  maf_std::Header header;
  SBPResultMeta meta; //!< Meta info
  /*!
   * @brief Task Status
   * \vec_max_size{20}
   */
  std::vector<TaskStatus> task_status;
  SBPResultExtra extra; //!< Extra info
};

/*!
 * \brief Struct of Mpc Trajectory Point
 */
struct MpcTrajectoryPoint {
  Point2d position_enu;
  double heading_yaw;
};

/*!
 * \brief Mpc Trajectory Info
 */
struct MpcTrajectory {
  uint8_t available;
  /*!
   * @brief Mpc Trajectory Point
   * \vec_max_size{101}
   */
  std::vector<MpcTrajectoryPoint> path_points;

  enum : uint8_t {
    PATH_POINTS = 1, //!< when Mpc Trajectory Points is available
  };
};

/*!
 * \brief Mpc Trajectory Result
 */
struct MpcTrajectoryResult {
  maf_std::Header header;
  MpcTrajectory mpc_trajectory;
};

/*!
 * \brief Planning Message sent by Planning Module
 */
struct Planning {
  maf_std::Header header;
  PlanMeta meta;                         //!< Meta Info
  Trajectory trajectory;                 //!< Trajectory Result
  Emergency emergency;                   //!< Emergency Info
  TurnSignalCommand turn_signal_command; //!< Turn Signal Command
  GearCommand gear_command;              //!< Gear Command
  PlanStatus plan_status;                //!< Plan Status
  Comfort comfort;                       //!< Comfort Info
  Decision decision;                     //!< Decision Info
  PlanExtra extra;                       //!< Extra Info

  serdes10(header,meta,
           trajectory,emergency,
           turn_signal_command, gear_command,
           plan_status,comfort,
           decision, extra)
};

} // namespace maf_planning

#endif

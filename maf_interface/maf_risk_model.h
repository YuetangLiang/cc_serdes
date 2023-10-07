#pragma once

#include "maf_interface/maf_perception_interface.h"
#include "maf_interface/maf_std.h"
#include "maf_interface/maf_worldmodel.h"

namespace maf_risk_model {

struct RiskyObject {
  uint64_t track_id;
  maf_perception_interface::Shape3d revised_shape;
  double risk_potential;
};

struct RiskyArea {
  uint8_t area_type;
  maf_worldmodel::MapPOIType poi_type;
  double distance;
  double risk_potential;
  bool in_area;
  std::vector<maf_perception_interface::Point3f> key_points_enu;

  enum : uint8_t {
    INTERSECTION = 1,
    MAP_POI = 2,
  };
};

struct RiskyBlindSpot {
  uint64_t object_id;
  double risk_potential;
  maf_perception_interface::Polygon3f polygon_enu;
  maf_perception_interface::Polygon3f polygon_car;
};

struct RiskyRoad {
  uint8_t edge;
  double distance;
  double relative_velocity;
  double risk_potential;

  enum : uint8_t {
    LEFT = 1,
    RIGHT = 2,
  };
};

struct SystemAbilityData {
  double x_std;
  double y_std;
  double yaw_std;
  double vx_std;
  double vy_std;
  double shape_w_std;
  double shape_l_std;
  double recall;
};

struct RiskModel {

  enum : uint8_t {
    NO_CAUSE = 0,
    CAUSE_OTHER_TRAFFIC_ISSUE = 1,
    CAUSE_OBJECT = 2,
    CAUSE_BLIND_SPOT = 3,
    CAUSE_RISKY_AREA = 4,
    CAUSE_LEFT_ROAD = 5,
    CAUSE_RIGHT_ROAD = 6,
    CAUSE_OTHER_PERCEPTION_ISSUE = 32,
    CAUSE_RAIN = 33,
    CAUSE_LIGHT = 34,
    CAUSE_SPEED = 35,
    CAUSE_UNKNOWN = 255,
  };

  enum : uint8_t {
    NO_RISK = 0,
    NEED_DRIVER_EYES_ON = 1,
    NEED_DRIVER_HANDS_ON = 2,
    NEED_DRIVER_TAKEOVER = 3,
  };

  enum : uint8_t {
    AREA_UNKNOWN = 0,
    AREA_PARKING_LOT = 1,
    AREA_HUMAN_ACCESS = 2,
    AREA_DESTINATION = 3,
    AREA_PARKING = 4,
    AREA_BARRIER_GAP = 5,
    AREA_FACILITY_ENTRANCE = 6,
    AREA_FACILITY_EXIT = 7,
    AREA_FACILITY_EXIT_AND_ENTRANCE = 8,
    AREA_BUS_STOP = 9,
    AREA_GARAGE_ENTRANCE = 10,
    AREA_GARAGE_EXIT = 11,
    AREA_SPEED_BUMP = 12,
    AREA_CROSS_WALK = 13,
    AREA_DASHED_SEGMENT = 14,
    AREA_CENTRAL_CIRCLE = 15,
    AREA_NO_PARKING_ZONE = 16,
    AREA_ROAD_MERGE = 17,
    AREA_ROAD_SPLIT = 18,
    AREA_INTERSECTION = 50,
  };

  maf_std::Header header;
  uint8_t risk_level;
  uint8_t risk_cause;
  uint64_t object_id;
  uint8_t area_type;
  double area_distance;
  std::vector<RiskyBlindSpot> risky_blind_spots;
  std::vector<int64_t> RiskModel_reserved_int;
  std::vector<double> RiskModel_reserved_double;
};

struct RiskModelTest {
  RiskModel risk;
  SystemAbilityData ability;
  std::vector<RiskyRoad> roads; // potential order
  std::vector<RiskyArea> areas;
  std::vector<RiskyBlindSpot> blindspots;
  std::vector<RiskyObject> objects;
  double recall_potential;
  double road_potential_sum;
  double areas_potential_sum;
  double blindspots_potential_sum;
  double objects_potential_sum;
  uint8_t ComAdapter_RainfallAmnt;
  uint16_t ComAdapter_TwliBriRaw;
};

} // namespace maf_risk_model
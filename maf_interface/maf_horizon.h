#ifndef MAF_INTERFACE_MAF_HORIZON_H
#define MAF_INTERFACE_MAF_HORIZON_H

#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_horizon {

struct HorizonLayer {
  std::string data;
    serdes1(data)
};

struct RouteData {
  std::string data;
};

struct HorizonData {
  uint8_t available;
  HorizonLayer map_offline;
  HorizonLayer map_online;
  HorizonLayer map_match_info;
  HorizonLayer other;

  enum : uint8_t {
    MAP_OFFLINE = 1,
    MAP_ONLINE = 2,
    MAP_MATCH_INFO = 4,
    HORIZON_LAYER_OTHER = 8,
  };

    serdes5(available,
            map_offline,
            map_online,
            map_match_info,
            other)
};

struct Route {
  maf_std::Header header;
  RouteData route_data;
};

struct Horizon {
  maf_std::Header header;
  HorizonData horizon_data;
    serdes2(header,horizon_data)
};

/*!
 * \brief Defines type of ODD event
 */
struct ODDType {
  uint8_t value;

  enum : uint8_t {
    ODD_UNKNOWN = 0,           //!< ODD event triggered by unknown reason
    ODD_ROAD_CONSTRUCTION = 1, //!< ODD event triggered by road construction
    ODD_TOLL = 2,              //!< ODD event triggered by toll
    ODD_GATE_MACHINE = 3,      //!< ODD event triggered by gate machine
    ODD_MAP_COLLECT = 4,       //!< ODD event triggered by map data collection
  };
};

/*!
 * \brief Defines the ODD event
 */
struct ODDEvent {
  uint32_t id;      //!> Each event has a unique id
  ODDType type;     //!> The type of the ODD event
  int32_t distance; //!> The distance from the vehicle's current position to the
                    //! event. Negative means the vehicle is still in the ODD's
                    //! district
};

/*!
 * \brief Container for all ODD events
 */
struct ODDData {
  maf_std::Header header;
  std::vector<ODDEvent> odd_events;
};

} // namespace maf_horizon

#endif

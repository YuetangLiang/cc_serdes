#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_timedomain_interface {

struct timedomain_msg {
  maf_std::Header header;
  uint64_t local_time;
  uint64_t gps_time;
  int32_t state;
};

} // namespace maf_timedomain_interface

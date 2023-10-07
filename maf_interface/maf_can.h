#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_can {

struct Frame {
  maf_std::Header header;
  uint32_t id;
  bool is_rtr;
  bool is_extended;
  bool is_error;
  uint8_t dlc;
  std::vector<uint8_t> data;
};

} // namespace maf_can

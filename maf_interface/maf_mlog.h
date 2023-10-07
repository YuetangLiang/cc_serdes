#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_mlog {

struct Entry {
  uint8_t type;
  std::string data;

  enum : uint8_t {
    DATA_TYPE_NORMAL = 0,
    DATA_TYPE_MSG_TIMESTAMP = 1,
    DATA_TYPE_MSG_RELATION = 2,
  };
};

struct Logs {
  maf_std::Header header;
  std::vector<Entry> entries;
};

} // namespace maf_mlog

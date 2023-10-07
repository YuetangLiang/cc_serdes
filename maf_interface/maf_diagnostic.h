#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_diagnostic {

struct KeyValue {
  std::string key;
  std::string value;
};

struct DiagnosticStatus {
  int8_t level;
  std::string name;
  std::string message;
  std::string hardware_id;
  std::vector<KeyValue> values;

  enum : int8_t {
    OK = 0,
    WARN = 1,
    ERROR = 2,
    STALE = 3,
  };
};

struct DiagnosticArray {
  maf_std::Header header;
  std::vector<DiagnosticStatus> status;
};

} // namespace maf_diagnostic

#pragma once

#include <cstdint>
#include <string>

namespace librmcs_hardware {

enum class CanBus {
  Can1,
  Can2,
};

struct JointConfig {
  enum class Vendor {
    Dji,
    Lk,
    Dm,
    Bm,
  };

  std::string name;
  Vendor vendor = Vendor::Dji;
  std::string model;
  CanBus can_bus = CanBus::Can1;
  uint32_t can_id = 0;
  int encoder_zero_point = 0;
  double reduction_ratio = 0.0;
  bool reversed = false;
  bool multi_turn_angle = false;
};

struct JointState {
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
  bool online = false;
};

}  // namespace librmcs_hardware

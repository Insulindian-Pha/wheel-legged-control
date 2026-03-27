#pragma once

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "librmcs_hardware/devices/motor_device_base.hpp"

namespace librmcs_hardware {

class GroupedCanCommandAggregator {
public:
  void clear();
  void ingest(const GroupedCanCommand & command);
  std::vector<CanFrame> flush() const;

private:
  struct GroupKey {
    CanBus can_bus = CanBus::Can1;
    uint32_t tx_can_id = 0;
    std::size_t slot_width_bytes = 0;
    std::size_t slot_count = 0;

    auto operator<=>(const GroupKey &) const = default;
  };

  struct GroupState {
    std::array<uint8_t, 8> data{};
  };

  std::map<GroupKey, GroupState> groups_;
};

}  // namespace librmcs_hardware

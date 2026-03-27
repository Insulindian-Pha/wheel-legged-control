#include "librmcs_hardware/driver/grouped_can_command.hpp"

#include <bit>
#include <stdexcept>

namespace librmcs_hardware {

void GroupedCanCommandAggregator::clear() {
  groups_.clear();
}

void GroupedCanCommandAggregator::ingest(const GroupedCanCommand & command) {
  if (!command.valid) {
    return;
  }

  if (command.slot_width_bytes == 0 || command.slot_count == 0) {
    throw std::runtime_error("Grouped CAN command must define non-zero slot width and slot count.");
  }
  if (command.slot_index >= command.slot_count) {
    throw std::runtime_error("Grouped CAN command slot index is out of range.");
  }
  if (command.slot_width_bytes * command.slot_count > 8) {
    throw std::runtime_error("Grouped CAN command exceeds 8-byte CAN payload size.");
  }

  auto & state = groups_[GroupKey{
    .can_bus = command.can_bus,
    .tx_can_id = command.tx_can_id,
    .slot_width_bytes = command.slot_width_bytes,
    .slot_count = command.slot_count,
  }];
  const auto offset = command.slot_index * command.slot_width_bytes;
  for (std::size_t byte_index = 0; byte_index < command.slot_width_bytes; ++byte_index) {
    state.data[offset + byte_index] =
      static_cast<uint8_t>((command.payload >> (8 * byte_index)) & 0xFF);
  }
}

std::vector<CanFrame> GroupedCanCommandAggregator::flush() const {
  std::vector<CanFrame> frames;
  frames.reserve(groups_.size());
  for (const auto & [key, state] : groups_) {
    frames.push_back(CanFrame{key.tx_can_id, std::bit_cast<uint64_t>(state.data), key.can_bus});
  }
  return frames;
}

}  // namespace librmcs_hardware

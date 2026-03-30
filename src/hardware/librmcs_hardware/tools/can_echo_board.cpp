#include "can_echo_board.hpp"

namespace librmcs_hardware::tools {

CanEchoBoard::CanEchoBoard(int32_t usb_pid)
: librmcs::client::CBoard(usb_pid), transmit_buffer_(*this, 16) {}

void CanEchoBoard::set_rx_callback(CanEchoRxCallback cb) {
  std::scoped_lock lock(callback_mutex_);
  rx_callback_ = std::move(cb);
}

bool CanEchoBoard::send_frame(
  bool can1, uint32_t can_id, uint64_t can_data, bool is_extended, bool is_remote_transmission,
  uint8_t can_data_length) {
  std::scoped_lock lock(transmit_mutex_);
  const bool queued = can1 ? transmit_buffer_.add_can1_transmission(
                                 can_id, can_data, is_extended, is_remote_transmission,
                                 can_data_length)
                           : transmit_buffer_.add_can2_transmission(
                                 can_id, can_data, is_extended, is_remote_transmission,
                                 can_data_length);
  if (!queued) {
    return false;
  }
  return transmit_buffer_.trigger_transmission();
}

void CanEchoBoard::can1_receive_callback(
  uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
  uint8_t can_data_length) {
  CanEchoRxCallback cb;
  {
    std::scoped_lock lock(callback_mutex_);
    cb = rx_callback_;
  }
  if (cb) {
    cb(1, can_id, can_data, is_extended_can_id, is_remote_transmission, can_data_length);
  }
}

void CanEchoBoard::can2_receive_callback(
  uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
  uint8_t can_data_length) {
  CanEchoRxCallback cb;
  {
    std::scoped_lock lock(callback_mutex_);
    cb = rx_callback_;
  }
  if (cb) {
    cb(2, can_id, can_data, is_extended_can_id, is_remote_transmission, can_data_length);
  }
}

}  // namespace librmcs_hardware::tools

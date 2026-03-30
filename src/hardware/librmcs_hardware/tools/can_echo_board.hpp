#pragma once

#include <librmcs/client/cboard.hpp>

#include <cstdint>
#include <functional>
#include <mutex>

namespace librmcs_hardware::tools {

using CanEchoRxCallback = std::function<void(
  int bus, uint32_t can_id, uint64_t can_data, bool is_extended, bool is_remote_transmission,
  uint8_t can_data_length)>;

class CanEchoBoard : public librmcs::client::CBoard {
public:
  explicit CanEchoBoard(int32_t usb_pid);

  void set_rx_callback(CanEchoRxCallback cb);

  bool send_frame(
    bool can1, uint32_t can_id, uint64_t can_data, bool is_extended, bool is_remote_transmission,
    uint8_t can_data_length);

private:
  void can1_receive_callback(
    uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    uint8_t can_data_length) override;

  void can2_receive_callback(
    uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    uint8_t can_data_length) override;

  std::mutex callback_mutex_;
  CanEchoRxCallback rx_callback_;
  std::mutex transmit_mutex_;
  TransmitBuffer transmit_buffer_;
};

}  // namespace librmcs_hardware::tools

#include "can_echo_board.hpp"

#include <librmcs_hardware/msg/librmcs_can_frame.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <cinttypes>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

namespace {

std::string trim_copy(std::string s) {
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}

bool parse_uint32_from_string(const std::string & raw, uint32_t & out) {
  const std::string s = trim_copy(raw);
  if (s.empty()) {
    return false;
  }
  char * end = nullptr;
  const unsigned long v = std::strtoul(s.c_str(), &end, 0);
  if (end == s.c_str() || *end != '\0') {
    return false;
  }
  out = static_cast<uint32_t>(v);
  return true;
}

template <typename DataArray>
uint64_t pack_le(const DataArray & data, uint8_t dlc) {
  uint64_t v = 0;
  for (uint8_t i = 0; i < dlc; ++i) {
    v |= static_cast<uint64_t>(data[i]) << (8 * i);
  }
  return v;
}

librmcs_hardware::msg::LibrmcsCanFrame make_frame_base(
  rclcpp::Node & node, uint8_t bus, uint32_t id, uint64_t raw_data, uint8_t dlc, bool ext,
  bool rtr) {
  librmcs_hardware::msg::LibrmcsCanFrame m;
  m.header.stamp = node.get_clock()->now();
  m.bus = bus;
  m.id = id;
  {
    std::ostringstream oss;
    oss << "0x" << std::uppercase << std::hex << id;
    m.id_hex = oss.str();
  }
  m.dlc = dlc;
  m.is_extended = ext;
  m.is_remote = rtr;
  for (uint8_t i = 0; i < dlc; ++i) {
    m.data[i] = static_cast<uint8_t>((raw_data >> (8 * i)) & 0xFF);
  }
  for (uint8_t i = dlc; i < 8; ++i) {
    m.data[i] = 0;
  }
  return m;
}

class LibrmcsCanBridgeNode final : public rclcpp::Node {
public:
  LibrmcsCanBridgeNode()
  : Node("librmcs_can_bridge") {
    const int32_t usb_pid = declare_parameter("usb_pid", -1);
    const std::string tx_topic = declare_parameter("tx_topic", std::string("can/tx"));
    const std::string rx_topic = declare_parameter("rx_topic", std::string("can/rx"));
    const std::string tx_echo_topic =
      declare_parameter("tx_echo_topic", std::string("can/tx_echo"));
    log_throttle_ms_ = declare_parameter("log_throttle_ms", 500);
    enable_verbose_log_ = declare_parameter("enable_verbose_log", true);

    board_ = std::make_unique<librmcs_hardware::tools::CanEchoBoard>(usb_pid);

    rclcpp::QoS qos{rclcpp::KeepLast(64)};
    qos.reliable();

    pub_rx_ = create_publisher<librmcs_hardware::msg::LibrmcsCanFrame>(rx_topic, qos);
    pub_tx_echo_ = create_publisher<librmcs_hardware::msg::LibrmcsCanFrame>(tx_echo_topic, qos);

    board_->set_rx_callback(
      [this](
        int bus, uint32_t can_id, uint64_t can_data, bool is_extended, bool is_remote_transmission,
        uint8_t can_data_length) { on_board_rx(bus, can_id, can_data, is_extended, is_remote_transmission, can_data_length); });

    usb_thread_ = std::thread([this]() {
      try {
        board_->handle_events();
      } catch (...) {
      }
    });

    sub_tx_ = create_subscription<librmcs_hardware::msg::LibrmcsCanFrame>(
      tx_topic, qos,
      std::bind(&LibrmcsCanBridgeNode::on_tx, this, std::placeholders::_1));

    setup_periodic_tx();

    RCLCPP_INFO(get_logger(), "librmcs CAN bridge: sub %s pub %s %s", tx_topic.c_str(),
      rx_topic.c_str(), tx_echo_topic.c_str());
  }

  LibrmcsCanBridgeNode(const LibrmcsCanBridgeNode &) = delete;
  LibrmcsCanBridgeNode & operator=(const LibrmcsCanBridgeNode &) = delete;

  ~LibrmcsCanBridgeNode() override {
    sub_tx_.reset();
    if (board_) {
      board_->stop_handling_events();
    }
    if (usb_thread_.joinable()) {
      usb_thread_.join();
    }
    board_.reset();
  }

private:
  void stop_periodic_tx() {
    if (periodic_tx_timer_) {
      periodic_tx_timer_->cancel();
      periodic_tx_timer_.reset();
    }
    periodic_tx_msg_.reset();
  }

  void configure_periodic_tx(
    const librmcs_hardware::msg::LibrmcsCanFrame::SharedPtr & msg, double rate_hz) {
    auto periodic_msg = std::make_shared<librmcs_hardware::msg::LibrmcsCanFrame>(*msg);
    periodic_msg->periodic_rate_hz = 0.0F;
    periodic_tx_msg_ = periodic_msg;

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    periodic_tx_timer_ = create_wall_timer(period, [this]() {
      if (!periodic_tx_msg_) {
        return;
      }
      on_tx(periodic_tx_msg_);
    });

    RCLCPP_INFO(
      get_logger(),
      "更新周期发送: rate=%.3f Hz bus=%u id=%u id_hex=\"%s\" dlc=%u%s%s",
      rate_hz,
      static_cast<unsigned>(periodic_tx_msg_->bus),
      static_cast<unsigned>(periodic_tx_msg_->id),
      periodic_tx_msg_->id_hex.c_str(),
      static_cast<unsigned>(periodic_tx_msg_->dlc),
      periodic_tx_msg_->is_extended ? " ext" : "",
      periodic_tx_msg_->is_remote ? " rtr" : "");
  }

  void setup_periodic_tx() {
    const double rate_hz = declare_parameter("periodic_tx_rate_hz", 0.0);
    if (rate_hz <= 0.0) {
      return;
    }

    auto msg = std::make_shared<librmcs_hardware::msg::LibrmcsCanFrame>();
    msg->bus = static_cast<uint8_t>(declare_parameter("periodic_tx_bus", 1));
    msg->id = static_cast<uint32_t>(declare_parameter("periodic_tx_id", 0));
    msg->id_hex = declare_parameter("periodic_tx_id_hex", std::string(""));
    msg->dlc = static_cast<uint8_t>(declare_parameter("periodic_tx_dlc", 8));
    msg->is_extended = declare_parameter("periodic_tx_is_extended", false);
    msg->is_remote = declare_parameter("periodic_tx_is_remote", false);

    {
      const auto data = declare_parameter(
        "periodic_tx_data",
        std::vector<int64_t>{0, 0, 0, 0, 0, 0, 0, 0});
      for (size_t i = 0; i < 8; ++i) {
        const int64_t v = i < data.size() ? data[i] : 0;
        msg->data[i] = static_cast<uint8_t>(std::clamp<int64_t>(v, 0, 255));
      }
    }

    periodic_tx_msg_ = msg;
    configure_periodic_tx(periodic_tx_msg_, rate_hz);
  }

  void on_tx(const librmcs_hardware::msg::LibrmcsCanFrame::SharedPtr msg) {
    if (!board_) {
      return;
    }
    if (msg->bus != 1 && msg->bus != 2) {
      RCLCPP_WARN(get_logger(), "忽略无效 bus=%u（应为 1 或 2）", msg->bus);
      return;
    }
    if (msg->dlc < 1 || msg->dlc > 8) {
      RCLCPP_WARN(get_logger(), "忽略无效 dlc=%u", msg->dlc);
      return;
    }

    uint32_t resolved_id = msg->id;
    if (!trim_copy(msg->id_hex).empty()) {
      if (!parse_uint32_from_string(msg->id_hex, resolved_id)) {
        RCLCPP_WARN(get_logger(), "id_hex 解析失败: \"%s\"", msg->id_hex.c_str());
        return;
      }
    }

    if (!msg->is_extended && resolved_id > 0x7FFU) {
      RCLCPP_WARN(get_logger(), "标准帧 id 0x%X 超出 11 位", resolved_id);
      return;
    }
    if (msg->is_extended && resolved_id > 0x1FFFFFFFU) {
      RCLCPP_WARN(get_logger(), "扩展帧 id 超出 29 位");
      return;
    }

    if (msg->periodic_rate_hz < 0.0F) {
      stop_periodic_tx();
      RCLCPP_INFO(get_logger(), "已停止周期发送");
      return;
    }
    if (msg->periodic_rate_hz > 0.0F) {
      stop_periodic_tx();
      auto periodic_msg = std::make_shared<librmcs_hardware::msg::LibrmcsCanFrame>(*msg);
      periodic_msg->id = resolved_id;
      periodic_msg->id_hex = "";
      configure_periodic_tx(periodic_msg, static_cast<double>(msg->periodic_rate_hz));
    }

    const uint64_t raw = pack_le(msg->data, msg->dlc);
    const bool can1 = msg->bus == 1;
    const bool ok = board_->send_frame(
      can1, resolved_id, raw, msg->is_extended, msg->is_remote, msg->dlc);

    if (!ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "CAN 发送失败（USB 缓冲区忙或未就绪）");
      return;
    }

    auto echoed = *msg;
    echoed.header.stamp = get_clock()->now();
    echoed.id = resolved_id;
    echoed.id_hex = "";
    echoed.periodic_rate_hz = 0.0F;
    pub_tx_echo_->publish(echoed);

    if (enable_verbose_log_) {
      if (log_throttle_ms_ > 0) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), static_cast<uint64_t>(log_throttle_ms_),
          "[TX] bus=%u id=0x%" PRIX32 "%s data=0x%016llX dlc=%u", msg->bus, resolved_id,
          msg->is_extended ? " ext" : "", static_cast<unsigned long long>(raw),
          static_cast<unsigned>(msg->dlc));
      } else {
        RCLCPP_INFO(
          get_logger(), "[TX] bus=%u id=0x%" PRIX32 "%s data=0x%016llX dlc=%u", msg->bus,
          resolved_id, msg->is_extended ? " ext" : "", static_cast<unsigned long long>(raw),
          static_cast<unsigned>(msg->dlc));
      }
    }
  }

  void on_board_rx(
    int bus, uint32_t can_id, uint64_t can_data, bool is_extended, bool is_remote_transmission,
    uint8_t can_data_length) {
    auto m = make_frame_base(*this, static_cast<uint8_t>(bus), can_id, can_data, can_data_length,
      is_extended, is_remote_transmission);
    pub_rx_->publish(m);

    if (enable_verbose_log_) {
      if (log_throttle_ms_ > 0) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), static_cast<uint64_t>(log_throttle_ms_),
          "[RX] bus=%d id=0x%" PRIX32 "%s%s data=0x%016llX dlc=%u", bus, can_id,
          is_extended ? " ext" : "", is_remote_transmission ? " rtr" : "",
          static_cast<unsigned long long>(can_data), static_cast<unsigned>(can_data_length));
      } else {
        RCLCPP_INFO(
          get_logger(), "[RX] bus=%d id=0x%" PRIX32 "%s%s data=0x%016llX dlc=%u", bus, can_id,
          is_extended ? " ext" : "", is_remote_transmission ? " rtr" : "",
          static_cast<unsigned long long>(can_data), static_cast<unsigned>(can_data_length));
      }
    }
  }

  std::unique_ptr<librmcs_hardware::tools::CanEchoBoard> board_;
  std::thread usb_thread_;

  rclcpp::Publisher<librmcs_hardware::msg::LibrmcsCanFrame>::SharedPtr pub_rx_;
  rclcpp::Publisher<librmcs_hardware::msg::LibrmcsCanFrame>::SharedPtr pub_tx_echo_;
  rclcpp::Subscription<librmcs_hardware::msg::LibrmcsCanFrame>::SharedPtr sub_tx_;
  rclcpp::TimerBase::SharedPtr periodic_tx_timer_;
  librmcs_hardware::msg::LibrmcsCanFrame::SharedPtr periodic_tx_msg_;

  int log_throttle_ms_ = 500;
  bool enable_verbose_log_ = true;
};

}  // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LibrmcsCanBridgeNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("librmcs_can_bridge"), "Fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

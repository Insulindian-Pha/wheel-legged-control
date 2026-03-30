// 通过 RM 控制板 USB（librmcs CBoard）发送 CAN 并打印收到的帧。
// 用法见 print_usage() 或运行 --help。

#include "can_echo_board.hpp"

#include <atomic>
#include <cctype>
#include <charconv>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>

namespace {

using librmcs_hardware::tools::CanEchoBoard;

void print_usage(const char * prog) {
  std::fprintf(
    stderr,
    "用法:\n"
    "  %s [--pid <hex>]     可选 USB 产品 ID（十进制或 0x 十六进制），默认自动选择唯一 0xa11c 设备\n"
    "\n"
    "交互命令（stdin，每行一条）:\n"
    "  <bus> <can_id> <data_hex>   bus=1 为 CAN1，bus=2 为 CAN2；\n"
    "                              can_id 支持十进制或 0x 十六进制；\n"
    "                              data_hex 为偶数个十六进制字符（1~16 个字符 = 1~8 字节），可带 0x 前缀。\n"
    "  <bus> -e <can_id> <data_hex>  扩展帧（29 位 ID）\n"
    "  q | quit                      退出\n"
    "  h | help                      显示本说明\n"
    "\n"
    "示例:\n"
    "  1 0x200 0x0102030405060708\n"
    "  2 -e 0x12345678 0x00ff\n",
    prog);
}

bool parse_int32_pid(std::string_view s, int32_t & out) {
  int base = 10;
  if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
    base = 16;
    s.remove_prefix(2);
  }
  const auto * first = s.data();
  const auto * last = s.data() + s.size();
  long long v = 0;
  auto r = std::from_chars(first, last, v, base);
  if (r.ec != std::errc{} || r.ptr != last || v < 0 || v > 0x7FFFFFFF) {
    return false;
  }
  out = static_cast<int32_t>(v);
  return true;
}

bool parse_hex_u32(std::string_view s, uint32_t & out) {
  if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
    s.remove_prefix(2);
  }
  const auto * first = s.data();
  const auto * last = s.data() + s.size();
  unsigned long long v = 0;
  auto r = std::from_chars(first, last, v, 16);
  if (r.ec != std::errc{} || r.ptr != last) {
    return false;
  }
  out = static_cast<uint32_t>(v);
  return true;
}

bool parse_hex_data(std::string_view s, uint64_t & out, uint8_t & dlc) {
  if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
    s.remove_prefix(2);
  }
  if (s.empty() || (s.size() % 2) != 0 || s.size() > 16) {
    return false;
  }
  for (unsigned char c : s) {
    if (!std::isxdigit(c)) {
      return false;
    }
  }
  out = 0;
  dlc = static_cast<uint8_t>(s.size() / 2);
  for (unsigned i = 0; i < dlc; ++i) {
    char hi = static_cast<char>(s[i * 2]);
    char lo = static_cast<char>(s[i * 2 + 1]);
    auto nibble = [](char ch) -> uint8_t {
      if (ch >= '0' && ch <= '9') {
        return static_cast<uint8_t>(ch - '0');
      }
      if (ch >= 'a' && ch <= 'f') {
        return static_cast<uint8_t>(10 + ch - 'a');
      }
      if (ch >= 'A' && ch <= 'F') {
        return static_cast<uint8_t>(10 + ch - 'A');
      }
      return 0;
    };
    const uint8_t byte = static_cast<uint8_t>((nibble(hi) << 4) | nibble(lo));
    out |= static_cast<uint64_t>(byte) << (8 * i);
  }
  return true;
}

void print_tx(bool can1, uint32_t can_id, uint64_t can_data, bool is_extended, uint8_t dlc) {
  const char * bus = can1 ? "can1" : "can2";
  std::printf(
    "[TX] bus=%s id=0x%" PRIX32 "%s data=0x%016llX dlc=%u\n", bus, can_id,
    is_extended ? " ext" : "", static_cast<unsigned long long>(can_data),
    static_cast<unsigned>(dlc));
  std::fflush(stdout);
}

void print_rx(
  int bus, uint32_t can_id, uint64_t can_data, bool is_extended, bool is_rtr, uint8_t dlc) {
  const char * bus_name = bus == 1 ? "can1" : "can2";
  std::printf(
    "[RX] bus=%s id=0x%" PRIX32 "%s%s data=0x%016llX dlc=%u\n", bus_name, can_id,
    is_extended ? " ext" : "", is_rtr ? " rtr" : "",
    static_cast<unsigned long long>(can_data), static_cast<unsigned>(dlc));
  std::fflush(stdout);
}

bool handle_line(CanEchoBoard & board, std::string_view line) {
  while (!line.empty() && (line.front() == ' ' || line.front() == '\t')) {
    line.remove_prefix(1);
  }
  while (!line.empty() && (line.back() == ' ' || line.back() == '\t' || line.back() == '\r')) {
    line.remove_suffix(1);
  }
  if (line.empty() || line[0] == '#') {
    return true;
  }

  std::string line_copy(line);
  char bus_str[16];
  char id_str[32];
  char data_str[40];
  char flag[8];

  if (std::sscanf(line_copy.c_str(), "%15s %7s %31s %39s", bus_str, flag, id_str, data_str) == 4) {
    if (std::strcmp(flag, "-e") == 0) {
      int bus = std::atoi(bus_str);
      if (bus != 1 && bus != 2) {
        std::fprintf(stderr, "无效 bus，应为 1 或 2\n");
        return true;
      }
      uint32_t can_id = 0;
      if (!parse_hex_u32(id_str, can_id)) {
        std::fprintf(stderr, "无效 can_id\n");
        return true;
      }
      uint64_t data = 0;
      uint8_t dlc = 8;
      if (!parse_hex_data(data_str, data, dlc)) {
        std::fprintf(stderr, "无效 data（偶数个十六进制字符，最多 16 个）\n");
        return true;
      }
      if (can_id > 0x1FFFFFFFU) {
        std::fprintf(stderr, "扩展帧 id 超出 29 位\n");
        return true;
      }
      if (!board.send_frame(bus == 1, can_id, data, true, false, dlc)) {
        std::fprintf(stderr, "发送失败（缓冲区忙或 USB 未就绪）\n");
      } else {
        print_tx(bus == 1, can_id, data, true, dlc);
      }
      return true;
    }
  }

  if (std::sscanf(line_copy.c_str(), "%15s %31s %39s", bus_str, id_str, data_str) != 3) {
    std::string cmd(line_copy);
    if (cmd == "q" || cmd == "quit") {
      return false;
    }
    if (cmd == "h" || cmd == "help") {
      print_usage("librmcs_can_echo");
      return true;
    }
    std::fprintf(stderr, "无法解析行，输入 h 查看格式\n");
    return true;
  }

  int bus = std::atoi(bus_str);
  if (bus != 1 && bus != 2) {
    std::fprintf(stderr, "无效 bus，应为 1 或 2\n");
    return true;
  }

  uint32_t can_id = 0;
  {
    std::string_view id_sv(id_str);
    int base = 10;
    if (id_sv.size() > 2 && id_sv[0] == '0' && (id_sv[1] == 'x' || id_sv[1] == 'X')) {
      base = 16;
    }
    const auto * first = id_sv.data();
    const auto * last = id_sv.data() + id_sv.size();
    unsigned long long v = 0;
    auto r = std::from_chars(first, last, v, base);
    if (r.ec != std::errc{} || r.ptr != last || v > 0x7FF) {
      std::fprintf(stderr, "无效标准帧 can_id（0~0x7FF）\n");
      return true;
    }
    can_id = static_cast<uint32_t>(v);
  }

  uint64_t data = 0;
  uint8_t dlc = 8;
  if (!parse_hex_data(data_str, data, dlc)) {
    std::fprintf(stderr, "无效 data（偶数个十六进制字符，最多 16 个）\n");
    return true;
  }

  if (!board.send_frame(bus == 1, can_id, data, false, false, dlc)) {
    std::fprintf(stderr, "发送失败（缓冲区忙或 USB 未就绪）\n");
  } else {
    print_tx(bus == 1, can_id, data, false, dlc);
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv) {
  int32_t usb_pid = -1;

  for (int i = 1; i < argc; ++i) {
    const char * a = argv[i];
    if (std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0) {
      print_usage(argv[0]);
      return 0;
    }
    if (std::strcmp(a, "--pid") == 0) {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "--pid 需要参数\n");
        return 1;
      }
      if (!parse_int32_pid(argv[i + 1], usb_pid)) {
        std::fprintf(stderr, "无效 --pid\n");
        return 1;
      }
      ++i;
      continue;
    }
    std::fprintf(stderr, "未知参数: %s\n", a);
    print_usage(argv[0]);
    return 1;
  }

  try {
    CanEchoBoard board(usb_pid);
    board.set_rx_callback(
      [](int bus, uint32_t can_id, uint64_t can_data, bool ext, bool rtr, uint8_t dlc) {
        print_rx(bus, can_id, can_data, ext, rtr, dlc);
      });

    std::atomic<bool> running{true};

    std::thread event_thread([&board, &running]() {
      try {
        board.handle_events();
      } catch (...) {
      }
      running.store(false, std::memory_order_relaxed);
    });

    std::fprintf(
      stderr,
      "已连接控制板，输入 CAN 行发送（h 帮助，q 退出）。首包 USB 上行会被固件丢弃一条，属正常。\n");

    std::string line;
    while (running.load(std::memory_order_relaxed) && std::getline(std::cin, line)) {
      if (!handle_line(board, line)) {
        break;
      }
    }

    board.stop_handling_events();
    if (event_thread.joinable()) {
      event_thread.join();
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "错误: %s\n", e.what());
    return 1;
  }

  return 0;
}

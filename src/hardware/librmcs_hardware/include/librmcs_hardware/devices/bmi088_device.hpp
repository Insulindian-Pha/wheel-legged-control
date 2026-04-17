#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <librmcs/device/bmi088.hpp>

namespace librmcs_hardware {

class Bmi088Device {
public:
  struct ImuRawData
  {
    int16_t acc_x{0};
    int16_t acc_y{0};
    int16_t acc_z{0};
    int16_t gyro_x{0};
    int16_t gyro_y{0};
    int16_t gyro_z{0};
  };

  bool read_raw_data(ImuRawData & imu_raw_data) const;
  void handle_accelerometer_data(int16_t x, int16_t y, int16_t z);
  void handle_gyroscope_data(int16_t x, int16_t y, int16_t z);

private:
  std::atomic<int16_t> acc_x_{0};
  std::atomic<int16_t> acc_y_{0};
  std::atomic<int16_t> acc_z_{0};
  std::atomic<int16_t> gyro_x_{0};
  std::atomic<int16_t> gyro_y_{0};
  std::atomic<int16_t> gyro_z_{0};
  std::atomic<bool> acc_ready_{false};
  std::atomic<bool> gyro_ready_{false};
};

class Bmi088Processor {
public:
  struct Config
  {
    double sample_freq{200.0};
    double kp{1.0};
    double ki{0.0};
    std::array<int, 3> gyro_axes{1, 2, 3};
    std::array<int, 3> acc_axes{1, 2, 3};
    bool calibration_enable{false};
    int calibration_window_samples{6000};
    double calibration_accel_norm_span{0.5};
    double calibration_gyro_span{0.15};
    double gyro_offset_x{0.0};
    double gyro_offset_y{0.0};
    double gyro_offset_z{0.0};
    double accel_scale{1.0};
    double accel_offset_x{0.0};
    double accel_offset_y{0.0};
    double accel_offset_z{0.0};
    std::string calibration_file;
  };

  struct State
  {
    double orientation_x{0.0};
    double orientation_y{0.0};
    double orientation_z{0.0};
    double orientation_w{1.0};
    double angular_velocity_x{0.0};
    double angular_velocity_y{0.0};
    double angular_velocity_z{0.0};
    double linear_acceleration_x{0.0};
    double linear_acceleration_y{0.0};
    double linear_acceleration_z{0.0};
  };

  static std::string default_calibration_file_path();
  bool initialize(const Config & config, std::string * warning_message = nullptr);
  bool update(const Bmi088Device::ImuRawData & imu_raw_data, State & state, std::string * info_message = nullptr);

private:
  Config config_{};
  bool calibration_loaded_from_file_{false};
  bool calibration_saved_to_file_{false};
  std::unique_ptr<librmcs::device::Bmi088> bmi088_;
};

}  // namespace librmcs_hardware

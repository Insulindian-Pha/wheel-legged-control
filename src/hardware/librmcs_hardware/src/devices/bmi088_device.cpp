#include "librmcs_hardware/devices/bmi088_device.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <tuple>

namespace librmcs_hardware {

namespace {

constexpr double kGravityMps2 = 9.80665;

std::string trim_copy(const std::string & value) {
  std::size_t begin = 0;
  while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }
  std::size_t end = value.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(begin, end - begin);
}

double map_axis(const std::array<int, 3> & mapping, double x, double y, double z, std::size_t axis) {
  const std::array<double, 3> source = {x, y, z};
  const int mapped_axis = mapping[axis];
  const int source_index = std::abs(mapped_axis) - 1;
  const double sign = mapped_axis >= 0 ? 1.0 : -1.0;
  return sign * source[source_index];
}

bool load_calibration_yaml(
  const std::filesystem::path & file_path, librmcs::device::Bmi088::CalibrationResult & result)
{
  std::ifstream input(file_path);
  if (!input.is_open()) {
    return false;
  }

  bool has_gyro_x = false;
  bool has_gyro_y = false;
  bool has_gyro_z = false;
  bool has_acc_scale = false;
  bool has_acc_x = false;
  bool has_acc_y = false;
  bool has_acc_z = false;

  std::string line;
  while (std::getline(input, line)) {
    auto parsed = trim_copy(line);
    if (parsed.empty() || parsed[0] == '#') {
      continue;
    }

    const auto comment_pos = parsed.find('#');
    if (comment_pos != std::string::npos) {
      parsed = trim_copy(parsed.substr(0, comment_pos));
      if (parsed.empty()) {
        continue;
      }
    }

    const auto delimiter_pos = parsed.find(':');
    if (delimiter_pos == std::string::npos) {
      continue;
    }
    const auto key = trim_copy(parsed.substr(0, delimiter_pos));
    const auto value = trim_copy(parsed.substr(delimiter_pos + 1));
    if (key.empty() || value.empty()) {
      continue;
    }

    try {
      if (key == "gyro_offset_x") {
        result.gyro_offset_x = std::stod(value);
        has_gyro_x = true;
      } else if (key == "gyro_offset_y") {
        result.gyro_offset_y = std::stod(value);
        has_gyro_y = true;
      } else if (key == "gyro_offset_z") {
        result.gyro_offset_z = std::stod(value);
        has_gyro_z = true;
      } else if (key == "accel_scale") {
        result.accel_scale = std::stod(value);
        has_acc_scale = true;
      } else if (key == "accel_offset_x") {
        result.accel_offset_x = std::stod(value);
        has_acc_x = true;
      } else if (key == "accel_offset_y") {
        result.accel_offset_y = std::stod(value);
        has_acc_y = true;
      } else if (key == "accel_offset_z") {
        result.accel_offset_z = std::stod(value);
        has_acc_z = true;
      }
    } catch (const std::exception &) {
      return false;
    }
  }

  return has_gyro_x && has_gyro_y && has_gyro_z && has_acc_scale && has_acc_x && has_acc_y &&
         has_acc_z;
}

bool save_calibration_yaml(
  const std::filesystem::path & file_path, const librmcs::device::Bmi088::CalibrationResult & result)
{
  std::error_code error;
  std::filesystem::create_directories(file_path.parent_path(), error);
  if (error) {
    return false;
  }

  std::ofstream output(file_path, std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    return false;
  }

  output << "# Auto-generated IMU calibration file.\n";
  output << std::setprecision(10);
  output << "gyro_offset_x: " << result.gyro_offset_x << "\n";
  output << "gyro_offset_y: " << result.gyro_offset_y << "\n";
  output << "gyro_offset_z: " << result.gyro_offset_z << "\n";
  output << "accel_scale: " << result.accel_scale << "\n";
  output << "accel_offset_x: " << result.accel_offset_x << "\n";
  output << "accel_offset_y: " << result.accel_offset_y << "\n";
  output << "accel_offset_z: " << result.accel_offset_z << "\n";
  output.flush();

  return output.good();
}

}  // namespace

bool Bmi088Device::read_raw_data(ImuRawData & imu_raw_data) const {
  if (!acc_ready_.load(std::memory_order::relaxed) || !gyro_ready_.load(std::memory_order::relaxed)) {
    return false;
  }

  imu_raw_data.acc_x = acc_x_.load(std::memory_order::relaxed);
  imu_raw_data.acc_y = acc_y_.load(std::memory_order::relaxed);
  imu_raw_data.acc_z = acc_z_.load(std::memory_order::relaxed);
  imu_raw_data.gyro_x = gyro_x_.load(std::memory_order::relaxed);
  imu_raw_data.gyro_y = gyro_y_.load(std::memory_order::relaxed);
  imu_raw_data.gyro_z = gyro_z_.load(std::memory_order::relaxed);
  return true;
}

void Bmi088Device::handle_accelerometer_data(int16_t x, int16_t y, int16_t z) {
  acc_x_.store(x, std::memory_order::relaxed);
  acc_y_.store(y, std::memory_order::relaxed);
  acc_z_.store(z, std::memory_order::relaxed);
  acc_ready_.store(true, std::memory_order::relaxed);
}

void Bmi088Device::handle_gyroscope_data(int16_t x, int16_t y, int16_t z) {
  gyro_x_.store(x, std::memory_order::relaxed);
  gyro_y_.store(y, std::memory_order::relaxed);
  gyro_z_.store(z, std::memory_order::relaxed);
  gyro_ready_.store(true, std::memory_order::relaxed);
}

std::string Bmi088Processor::default_calibration_file_path() {
  return (std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
          "msg/imu_calibration.yaml")
    .string();
}

bool Bmi088Processor::initialize(const Config & config, std::string * warning_message) {
  config_ = config;
  calibration_loaded_from_file_ = false;
  calibration_saved_to_file_ = false;
  bmi088_ = std::make_unique<librmcs::device::Bmi088>(config_.sample_freq, config_.kp, config_.ki);

  if (config_.gyro_axes != config_.acc_axes) {
    if (warning_message) {
      *warning_message = "imu_gyro_axes and imu_acc_axes must be identical.";
    }
    return false;
  }

  if (config_.gyro_axes != std::array<int, 3>{1, 2, 3}) {
    const auto axes = config_.gyro_axes;
    bmi088_->set_coordinate_mapping(
      [axes](double x, double y, double z) -> std::tuple<double, double, double> {
        return {map_axis(axes, x, y, z, 0), map_axis(axes, x, y, z, 1), map_axis(axes, x, y, z, 2)};
      });
  }

  bmi088_->set_calibration_window(static_cast<std::size_t>(std::max(1, config_.calibration_window_samples)));
  bmi088_->set_calibration_thresholds(config_.calibration_accel_norm_span, config_.calibration_gyro_span);

  if (!config_.calibration_enable) {
    librmcs::device::Bmi088::CalibrationResult calibration_result = {
      config_.gyro_offset_x,
      config_.gyro_offset_y,
      config_.gyro_offset_z,
      config_.accel_scale,
      config_.accel_offset_x,
      config_.accel_offset_y,
      config_.accel_offset_z};
    if (!config_.calibration_file.empty() &&
        load_calibration_yaml(config_.calibration_file, calibration_result))
    {
      calibration_loaded_from_file_ = true;
    } else if (warning_message && !config_.calibration_file.empty()) {
      *warning_message =
        "IMU calibration file unavailable or invalid, fallback to hardware parameters.";
    }
    bmi088_->set_calibration_result(
      calibration_result.gyro_offset_x, calibration_result.gyro_offset_y,
      calibration_result.gyro_offset_z, calibration_result.accel_scale,
      calibration_result.accel_offset_x, calibration_result.accel_offset_y,
      calibration_result.accel_offset_z);
  }

  return true;
}

bool Bmi088Processor::update(
  const Bmi088Device::ImuRawData & imu_raw_data, State & state, std::string * info_message)
{
  if (!bmi088_) {
    return false;
  }
  const bool calibrated_before_update = bmi088_->is_calibrated();
  bmi088_->store_accelerometer_status(imu_raw_data.acc_x, imu_raw_data.acc_y, imu_raw_data.acc_z);
  bmi088_->store_gyroscope_status(imu_raw_data.gyro_x, imu_raw_data.gyro_y, imu_raw_data.gyro_z);
  bmi088_->update_status();

  if (config_.calibration_enable && !calibrated_before_update && bmi088_->is_calibrated() &&
      !calibration_saved_to_file_)
  {
    const auto calibration_result = bmi088_->calibration_result();
    if (!config_.calibration_file.empty() &&
        save_calibration_yaml(config_.calibration_file, calibration_result))
    {
      calibration_saved_to_file_ = true;
      if (info_message) {
        *info_message = "Saved IMU calibration result to file.";
      }
    } else if (info_message && !config_.calibration_file.empty()) {
      *info_message = "Failed to save IMU calibration result to file.";
    }
  }

  state.orientation_w = bmi088_->q0();
  state.orientation_x = bmi088_->q1();
  state.orientation_y = bmi088_->q2();
  state.orientation_z = bmi088_->q3();
  state.angular_velocity_x = bmi088_->gx();
  state.angular_velocity_y = bmi088_->gy();
  state.angular_velocity_z = bmi088_->gz();
  state.linear_acceleration_x = bmi088_->ax() * kGravityMps2;
  state.linear_acceleration_y = bmi088_->ay() * kGravityMps2;
  state.linear_acceleration_z = bmi088_->az() * kGravityMps2;
  return true;
}

}  // namespace librmcs_hardware

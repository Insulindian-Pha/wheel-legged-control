#pragma once

#include <algorithm>
#include <cmath>

#include <atomic>
#include <cstddef>
#include <functional>
#include <numbers>
#include <utility>

namespace librmcs::device {

class Bmi088 {
public:
    struct CalibrationResult {
        double gyro_offset_x;
        double gyro_offset_y;
        double gyro_offset_z;
        double accel_scale;
        double accel_offset_x;
        double accel_offset_y;
        double accel_offset_z;
    };

    explicit Bmi088(
        double sample_freq, double kp, double ki, double q0 = 1, double q1 = 0, double q2 = 0,
        double q3 = 0)
        : inv_sample_freq_(1.0 / sample_freq)
        , double_kp_(2.0 * kp)
        , double_ki_(2.0 * ki)
        , q0_(q0)
        , q1_(q1)
        , q2_(q2)
        , q3_(q3) {
        reset_calibration();
    };

    void set_coordinate_mapping(
        std::function<std::tuple<double, double, double>(double, double, double)>
            mapping_function) {
        coordinate_mapping_function_ = std::move(mapping_function);
    }

    void store_accelerometer_status(int16_t x, int16_t y, int16_t z) {
        accelerometer_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void store_gyroscope_status(int16_t x, int16_t y, int16_t z) {
        gyroscope_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void update_status() {
        auto acc = accelerometer_data_.load(std::memory_order::relaxed);
        auto gyro = gyroscope_data_.load(std::memory_order::relaxed);

        auto solve_acc = [](int16_t value) { return value / 32767.0 * 6.0; };
        auto solve_gyro = [](int16_t value) {
            return value / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
        };

        gx_ = solve_gyro(gyro.x), gy_ = solve_gyro(gyro.y), gz_ = solve_gyro(gyro.z);
        ax_ = solve_acc(acc.x), ay_ = solve_acc(acc.y), az_ = solve_acc(acc.z);

        if (coordinate_mapping_function_) {
            std::tie(gx_, gy_, gz_) = coordinate_mapping_function_(gx_, gy_, gz_);
            std::tie(ax_, ay_, az_) = coordinate_mapping_function_(ax_, ay_, az_);
        }

        if (!calibrated_) {
            update_calibration(gx_, gy_, gz_, ax_, ay_, az_);
            return;
        }

        gx_ -= gyro_offset_x_;
        gy_ -= gyro_offset_y_;
        gz_ -= gyro_offset_z_;

        ax_ -= accel_offset_x_;
        ay_ -= accel_offset_y_;
        az_ -= accel_offset_z_;

        ax_ *= accel_scale_;
        ay_ *= accel_scale_;
        az_ *= accel_scale_;

        mahony_ahrs_update_imu(ax_, ay_, az_, gx_, gy_, gz_);
    }

    double ax() const { return ax_; }
    double ay() const { return ay_; }
    double az() const { return az_; }

    double gx() const { return gx_; }
    double gy() const { return gy_; }
    double gz() const { return gz_; }

    double& q0() { return q0_; }
    double& q1() { return q1_; }
    double& q2() { return q2_; }
    double& q3() { return q3_; }
    bool is_calibrated() const { return calibrated_; }
    void reset_calibration() {
        calibrated_ = false;
        gyro_offset_x_ = 0.0;
        gyro_offset_y_ = 0.0;
        gyro_offset_z_ = 0.0;
        accel_scale_ = 1.0;
        accel_offset_x_ = 0.0;
        accel_offset_y_ = 0.0;
        accel_offset_z_ = 0.0;
        reset_calibration_window();
    }
    void set_calibration_window(std::size_t samples) {
        calibration_window_samples_ = samples > 0 ? samples : 1;
        reset_calibration();
    }
    void set_calibration_thresholds(double accel_norm_span, double gyro_span) {
        accel_norm_span_threshold_ = accel_norm_span;
        gyro_span_threshold_ = gyro_span;
        reset_calibration();
    }
    void set_calibration_result(
        double gyro_offset_x, double gyro_offset_y, double gyro_offset_z, double accel_scale = 1.0,
        double accel_offset_x = 0.0, double accel_offset_y = 0.0, double accel_offset_z = 0.0) {
        gyro_offset_x_ = gyro_offset_x;
        gyro_offset_y_ = gyro_offset_y;
        gyro_offset_z_ = gyro_offset_z;
        accel_scale_ = accel_scale;
        accel_offset_x_ = accel_offset_x;
        accel_offset_y_ = accel_offset_y;
        accel_offset_z_ = accel_offset_z;
        calibrated_ = true;
        reset_calibration_window();
    }
    CalibrationResult calibration_result() const {
        return {
            gyro_offset_x_,
            gyro_offset_y_,
            gyro_offset_z_,
            accel_scale_,
            accel_offset_x_,
            accel_offset_y_,
            accel_offset_z_};
    }

private:
    void reset_calibration_window() {
        calibration_sample_count_ = 0;
        sum_gx_ = sum_gy_ = sum_gz_ = 0.0;
        sum_ax_ = sum_ay_ = sum_az_ = 0.0;
        sum_acc_norm_ = 0.0;
        gyro_min_x_ = gyro_min_y_ = gyro_min_z_ = 0.0;
        gyro_max_x_ = gyro_max_y_ = gyro_max_z_ = 0.0;
        acc_norm_min_ = acc_norm_max_ = 0.0;
    }

    void update_calibration(double gx, double gy, double gz, double ax, double ay, double az) {
        const auto acc_norm = std::sqrt(ax * ax + ay * ay + az * az);
        if (calibration_sample_count_ == 0) {
            gyro_min_x_ = gyro_max_x_ = gx;
            gyro_min_y_ = gyro_max_y_ = gy;
            gyro_min_z_ = gyro_max_z_ = gz;
            acc_norm_min_ = acc_norm_max_ = acc_norm;
        } else {
            gyro_min_x_ = std::min(gyro_min_x_, gx);
            gyro_min_y_ = std::min(gyro_min_y_, gy);
            gyro_min_z_ = std::min(gyro_min_z_, gz);
            gyro_max_x_ = std::max(gyro_max_x_, gx);
            gyro_max_y_ = std::max(gyro_max_y_, gy);
            gyro_max_z_ = std::max(gyro_max_z_, gz);
            acc_norm_min_ = std::min(acc_norm_min_, acc_norm);
            acc_norm_max_ = std::max(acc_norm_max_, acc_norm);
        }

        sum_gx_ += gx;
        sum_gy_ += gy;
        sum_gz_ += gz;
        sum_ax_ += ax;
        sum_ay_ += ay;
        sum_az_ += az;
        sum_acc_norm_ += acc_norm;
        ++calibration_sample_count_;

        if (calibration_sample_count_ < calibration_window_samples_) {
            return;
        }

        const double gyro_span_x = gyro_max_x_ - gyro_min_x_;
        const double gyro_span_y = gyro_max_y_ - gyro_min_y_;
        const double gyro_span_z = gyro_max_z_ - gyro_min_z_;
        const double acc_norm_span = acc_norm_max_ - acc_norm_min_;

        const bool steady =
            acc_norm_span <= accel_norm_span_threshold_ && gyro_span_x <= gyro_span_threshold_ &&
            gyro_span_y <= gyro_span_threshold_ && gyro_span_z <= gyro_span_threshold_;
        if (!steady) {
            reset_calibration_window();
            return;
        }

        const double inv_samples = 1.0 / static_cast<double>(calibration_sample_count_);
        gyro_offset_x_ = sum_gx_ * inv_samples;
        gyro_offset_y_ = sum_gy_ * inv_samples;
        gyro_offset_z_ = sum_gz_ * inv_samples;
        const double avg_acc_norm = sum_acc_norm_ * inv_samples;
        if (avg_acc_norm > 1e-6) {
            accel_scale_ = 1.0 / avg_acc_norm;
        } else {
            accel_scale_ = 1.0;
        }
        // Keep legacy behavior: estimate X/Y accel offsets, keep Z as 0
        // because Z contains gravity when the robot is upright.
        accel_offset_x_ = (sum_ax_ * inv_samples) * accel_scale_;
        accel_offset_y_ = (sum_ay_ * inv_samples) * accel_scale_;
        accel_offset_z_ = 0.0;

        calibrated_ = true;
        reset_calibration_window();
    }

    void mahony_ahrs_update_imu(double ax, double ay, double az, double gx, double gy, double gz) {
        // Madgwick's implementation of Mayhony's AHRS algorithm.
        // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms

        double recip_norm;
        double halfvx, halfvy, halfvz;
        double halfex, halfey, halfez;
        double qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer
        // normalization)
        if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

            // Normalize accelerometer measurement
            recip_norm = 1 / std::sqrt(ax * ax + ay * ay + az * az);
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            // Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = q1_ * q3_ - q0_ * q2_;
            halfvy = q0_ * q1_ + q2_ * q3_;
            halfvz = q0_ * q0_ - 0.5 + q3_ * q3_;

            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = ay * halfvz - az * halfvy;
            halfey = az * halfvx - ax * halfvz;
            halfez = ax * halfvy - ay * halfvx;

            // Compute and apply integral feedback if enabled
            if (double_ki_ > 0.0) {
                // integral error scaled by Ki
                integral_fbx_ += double_ki_ * halfex * (inv_sample_freq_);
                integral_fby_ += double_ki_ * halfey * (inv_sample_freq_);
                integral_fbz_ += double_ki_ * halfez * (inv_sample_freq_);
                // apply integral feedback
                gx += integral_fbx_;
                gy += integral_fby_;
                gz += integral_fbz_;
            } else {
                // prevent integral windup
                integral_fbx_ = 0.0;
                integral_fby_ = 0.0;
                integral_fbz_ = 0.0;
            }

            // Apply proportional feedback
            gx += double_kp_ * halfex;
            gy += double_kp_ * halfey;
            gz += double_kp_ * halfez;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5 * (inv_sample_freq_)); // pre-multiply common factors
        gy *= (0.5 * (inv_sample_freq_));
        gz *= (0.5 * (inv_sample_freq_));
        qa = q0_;
        qb = q1_;
        qc = q2_;
        q0_ += (-qb * gx - qc * gy - q3_ * gz);
        q1_ += (qa * gx + qc * gz - q3_ * gy);
        q2_ += (qa * gy - qb * gz + q3_ * gx);
        q3_ += (qa * gz + qb * gy - qc * gx);

        // Normalize quaternion
        recip_norm = 1 / std::sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
        q0_ *= recip_norm;
        q1_ *= recip_norm;
        q2_ *= recip_norm;
        q3_ *= recip_norm;
    }

    double inv_sample_freq_; // The reciprocal of sampling frequency
    double double_kp_;       // 2 * proportional gain (Kp)
    double double_ki_;       // 2 * integral gain (Ki)

    struct alignas(8) ImuData {
        int16_t x, y, z;
    };
    std::atomic<ImuData> accelerometer_data_, gyroscope_data_;
    static_assert(std::atomic<ImuData>::is_always_lock_free);

    double ax_ = 0.0, ay_ = 0.0, az_ = 1.0, gx_ = 0.0, gy_ = 0.0, gz_ = 0.0;

    std::function<std::tuple<double, double, double>(double, double, double)>
        coordinate_mapping_function_;

    // Quaternion of sensor frame relative to auxiliary frame
    double q0_, q1_, q2_, q3_;

    // Integral error terms scaled by Ki
    double integral_fbx_ = 0.0, integral_fby_ = 0.0, integral_fbz_ = 0.0;

    bool calibrated_ = false;
    std::size_t calibration_window_samples_ = 6000;
    std::size_t calibration_sample_count_ = 0;
    double accel_norm_span_threshold_ = 0.5;
    double gyro_span_threshold_ = 0.15;

    double sum_gx_ = 0.0, sum_gy_ = 0.0, sum_gz_ = 0.0;
    double sum_ax_ = 0.0, sum_ay_ = 0.0, sum_az_ = 0.0;
    double sum_acc_norm_ = 0.0;
    double gyro_min_x_ = 0.0, gyro_min_y_ = 0.0, gyro_min_z_ = 0.0;
    double gyro_max_x_ = 0.0, gyro_max_y_ = 0.0, gyro_max_z_ = 0.0;
    double acc_norm_min_ = 0.0, acc_norm_max_ = 0.0;
    double gyro_offset_x_ = 0.0, gyro_offset_y_ = 0.0, gyro_offset_z_ = 0.0;
    double accel_scale_ = 1.0;
    double accel_offset_x_ = 0.0, accel_offset_y_ = 0.0, accel_offset_z_ = 0.0;
};

} // namespace librmcs::device
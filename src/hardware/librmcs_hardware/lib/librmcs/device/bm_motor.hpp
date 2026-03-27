#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <stdexcept>
#include <string>

namespace librmcs::device {

class BmMotor {
public:
    enum class Type : uint8_t { P1010B_111, M1505B_111 };

    struct Config {
        explicit Config(Type motor_type)
            : motor_type(motor_type) {
            encoder_zero_point = 0;
            reversed = false;
        }

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }

        Type motor_type;
        int encoder_zero_point;
        bool reversed;
    };

    BmMotor()
        : angle_(0.0)
        , velocity_(0.0)
        , torque_(0.0) {}

    explicit BmMotor(const Config& config)
        : BmMotor() {
        configure(config);
    }

    void configure(const Config& config) {
        sign_ = config.reversed ? -1.0 : 1.0;
        encoder_zero_point_ = config.encoder_zero_point;
        angle_initialized_ = false;
        last_calibrated_position_ = 0;
        multi_turn_position_ = 0;
        angle_ = 0.0;
        velocity_ = 0.0;
        torque_ = 0.0;
        feedback_ = Feedback{};

        switch (config.motor_type) {
        case Type::P1010B_111:
            feedback_base_id_ = 0x50;
            command_can_id_low_ = 0x32;
            command_can_id_high_ = 0x33;
            raw_position_max_ = 32768;
            raw_velocity_to_rpm_coefficient_ = 0.1;
            feedback_current_to_ampere_coefficient_ = 0.01;
            torque_per_ampere_coefficient_ = 1.2 / 1.414;
            command_raw_per_ampere_coefficient_ = 100.0;
            min_command_ = -7500;
            max_command_ = 7500;
            has_state_command_ = true;
            needs_startup_warmup_ = false;
            break;
        case Type::M1505B_111:
            feedback_base_id_ = 0x96;
            command_can_id_low_ = 0x32;
            command_can_id_high_ = 0x33;
            raw_position_max_ = 32767;
            raw_velocity_to_rpm_coefficient_ = 0.1;
            feedback_current_to_ampere_coefficient_ = 55.0 / 32767.0;
            torque_per_ampere_coefficient_ = 0.8;
            command_raw_per_ampere_coefficient_ = 32767.0 / 55.0;
            min_command_ = -16383;
            max_command_ = 16383;
            has_state_command_ = false;
            needs_startup_warmup_ = true;
            break;
        }
    }

    void set_feedback_can_id(uint32_t can_id) {
        if (can_id <= feedback_base_id_) {
            throw std::runtime_error("BM feedback CAN ID must be greater than base ID.");
        }

        const auto motor_id = can_id - feedback_base_id_;
        if (motor_id < 1 || motor_id > 8) {
            throw std::runtime_error("BM feedback CAN ID must map to motor ID in [1, 8].");
        }

        motor_id_ = static_cast<uint8_t>(motor_id);
        command_can_id_ = motor_id_ <= 4 ? command_can_id_low_ : command_can_id_high_;
        command_slot_ = (motor_id_ - 1) % 4;
    }

    void store_status(uint64_t can_data) { can_data_.store(can_data, std::memory_order::relaxed); }

    void update_status() {
        const auto data = std::bit_cast<std::array<uint8_t, 8>>(can_data_.load(std::memory_order::relaxed));
        feedback_.velocity = read_be_i16(data, 0);
        feedback_.current = read_be_i16(data, 2);
        feedback_.position = read_be_u16(data, 4);

        if (has_state_command_) {
            feedback_.voltage = read_be_u16(data, 6);
            feedback_.fault = 0;
            feedback_.mode = 0;
        } else {
            feedback_.voltage = 0;
            feedback_.fault = data[6];
            feedback_.mode = data[7];
        }

        const auto calibrated = calibrated_position(feedback_.position);
        if (!angle_initialized_) {
            last_calibrated_position_ = calibrated;
            multi_turn_position_ = calibrated;
            angle_initialized_ = true;
        } else {
            auto diff = calibrated - last_calibrated_position_;
            const auto half_turn = raw_position_max_ / 2;
            if (diff <= -half_turn)
                diff += raw_position_max_;
            else if (diff > half_turn)
                diff -= raw_position_max_;

            multi_turn_position_ += diff;
            last_calibrated_position_ = calibrated;
        }

        const auto position_scale = 2.0 * std::numbers::pi / static_cast<double>(raw_position_max_);
        angle_ = sign_ * static_cast<double>(multi_turn_position_) * position_scale;

        const auto velocity_rpm = static_cast<double>(feedback_.velocity) * raw_velocity_to_rpm_coefficient_;
        velocity_ = sign_ * velocity_rpm / 60.0 * 2.0 * std::numbers::pi;

        const auto current = static_cast<double>(feedback_.current) * feedback_current_to_ampere_coefficient_;
        torque_ = sign_ * current * torque_per_ampere_coefficient_;
    }

    uint32_t feedback_base_id() const { return feedback_base_id_; }
    uint8_t motor_id() const { return motor_id_; }
    uint32_t command_can_id() const { return command_can_id_; }
    std::size_t command_slot() const { return command_slot_; }
    std::size_t group_slot_count() const { return 4; }
    bool has_state_command() const { return has_state_command_; }
    bool needs_startup_warmup() const { return needs_startup_warmup_; }
    uint8_t fault() const { return feedback_.fault; }
    uint8_t mode() const { return feedback_.mode; }
    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }

    uint16_t generate_group_command_word(double control_torque) const {
        const auto raw_command =
            static_cast<uint16_t>(compute_raw_command(control_torque));
        return static_cast<uint16_t>((raw_command >> 8) | (raw_command << 8));
    }

    uint64_t generate_torque_command(double control_torque) const {
        return pack_command_frame(command_slot_, compute_raw_command(control_torque));
    }

    uint64_t generate_disable_command() const {
        if (has_state_command_)
            return 0x0101010101010101ULL;
        return pack_command_frame(command_slot_, 0);
    }

    uint64_t generate_enable_command() const {
        if (!has_state_command_)
            return 0;
        return 0x0202020202020202ULL;
    }

private:
    int16_t compute_raw_command(double control_torque) const {
        if (!std::isfinite(control_torque))
            return 0;

        const auto current = control_torque / torque_per_ampere_coefficient_;
        const auto raw_command = std::lround(sign_ * current * command_raw_per_ampere_coefficient_);
        return clamp_command(raw_command, min_command_, max_command_);
    }

    struct Feedback {
        int16_t velocity = 0;
        int16_t current = 0;
        uint16_t position = 0;
        uint16_t voltage = 0;
        uint8_t fault = 0;
        uint8_t mode = 0;
    };

    static int16_t read_be_i16(const std::array<uint8_t, 8>& data, std::size_t index) {
        return static_cast<int16_t>((static_cast<uint16_t>(data[index]) << 8) | data[index + 1]);
    }

    static uint16_t read_be_u16(const std::array<uint8_t, 8>& data, std::size_t index) {
        return static_cast<uint16_t>((static_cast<uint16_t>(data[index]) << 8) | data[index + 1]);
    }

    static uint64_t pack_command_frame(std::size_t slot, int16_t command) {
        std::array<uint8_t, 8> data{};
        data[slot * 2] = static_cast<uint8_t>((static_cast<uint16_t>(command) >> 8) & 0xFF);
        data[slot * 2 + 1] = static_cast<uint8_t>(static_cast<uint16_t>(command) & 0xFF);
        return std::bit_cast<uint64_t>(data);
    }

    static uint64_t pack_state_frame(uint8_t motor_id, uint8_t command) {
        std::array<uint8_t, 8> data{};
        data[motor_id - 1] = command;
        return std::bit_cast<uint64_t>(data);
    }

    static int16_t clamp_command(long value, int16_t min_value, int16_t max_value) {
        return static_cast<int16_t>(std::clamp(
            value, static_cast<long>(min_value), static_cast<long>(max_value)));
    }

    int calibrated_position(uint16_t raw_position) const {
        auto calibrated = static_cast<int>(raw_position) - encoder_zero_point_;
        calibrated %= raw_position_max_;
        if (calibrated < 0)
            calibrated += raw_position_max_;
        return calibrated;
    }

    std::atomic<uint64_t> can_data_ = 0;
    Feedback feedback_{};

    uint32_t feedback_base_id_ = 0;
    uint32_t command_can_id_low_ = 0;
    uint32_t command_can_id_high_ = 0;
    uint32_t command_can_id_ = 0;
    int raw_position_max_ = 0;
    int encoder_zero_point_ = 0;
    std::size_t command_slot_ = 0;
    uint8_t motor_id_ = 0;
    int16_t min_command_ = 0;
    int16_t max_command_ = 0;
    bool has_state_command_ = false;
    bool needs_startup_warmup_ = false;
    bool angle_initialized_ = false;
    int last_calibrated_position_ = 0;
    int64_t multi_turn_position_ = 0;
    double sign_ = 1.0;
    double raw_velocity_to_rpm_coefficient_ = 0.0;
    double feedback_current_to_ampere_coefficient_ = 0.0;
    double torque_per_ampere_coefficient_ = 0.0;
    double command_raw_per_ampere_coefficient_ = 0.0;
    double angle_ = 0.0;
    double velocity_ = 0.0;
    double torque_ = 0.0;
};

}  // namespace librmcs::device

/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file device_manager.cpp
 * @brief Motor and IMU peripheral adapters
 */

#include "device_manager.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace whole_body {
namespace {

double MonotonicTime() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

class PeripheralDevices final : public DeviceManager {
public:
    explicit PeripheralDevices(const RuntimeConfig &config) : config_(config) {
        std::unordered_map<std::string, BusConfig> buses;
        for (const auto &bus : config_.buses) buses.emplace(bus.name, bus);

        motors_.reserve(config_.motors.size());
        motor_feedback_valid_.assign(config_.motors.size(), false);
        motor_feedback_time_s_.assign(config_.motors.size(), 0.0);
        for (const auto &motor : config_.motors) {
            const auto bus = buses.find(motor.bus);
            if (bus == buses.end()) throw std::runtime_error("unknown motor bus");
            const std::vector<DriverOption> configured_options =
                BuildMotorDriverOptions(motor);
            std::vector<motor_option> options;
            options.reserve(configured_options.size());
            for (const auto &option : configured_options)
                options.push_back({option.name.c_str(), option.value.c_str()});
            motor_dev *device = motor_alloc_can_with_options(motor.driver.c_str(),
                bus->second.device.c_str(), motor.command_id, options.data(), options.size());
            if (!device) {
                ReleaseAllocated();
                throw std::runtime_error("failed to allocate motor '" + motor.name + "'");
            }
            motors_.push_back(device);
        }

        const std::string imu_instance = config_.imu.driver + ":whole_body";
        imu_ = imu_alloc_uart(
            imu_instance.c_str(), config_.imu.device.c_str(), config_.imu.baud, nullptr);
        if (!imu_) {
            ReleaseAllocated();
            throw std::runtime_error("failed to allocate IMU driver '" + config_.imu.driver + "'");
        }
    }

    ~PeripheralDevices() override { Shutdown(); }

    int Init() override {
        if (initialized_) return 0;
        if (motor_init(motors_.data(), motors_.size()) < 0) return -1;
        imu_config imu_config{};
        std::copy(config_.imu.mounting_matrix.begin(), config_.imu.mounting_matrix.end(),
            imu_config.mounting_matrix);
        std::copy(config_.imu.acceleration_bias.begin(), config_.imu.acceleration_bias.end(),
            imu_config.acc_offset);
        std::copy(
            config_.imu.gyro_bias.begin(), config_.imu.gyro_bias.end(), imu_config.gyro_offset);
        imu_config.sample_rate = static_cast<uint32_t>(1.0 / config_.cycle_s);
        if (imu_init(imu_, &imu_config) < 0) return -1;
        initialized_ = true;
        feedback_start_time_s_ = MonotonicTime();
        return 0;
    }

    int Read(std::vector<motor_state> *motors, imu_data *imu,
        DeviceFeedbackStatus *status) override {
        if (!initialized_ || !motors || !imu || !status ||
            motors->size() != motors_.size()) {
            return -1;
        }
        const double now = MonotonicTime();
        status->motor_received.assign(motors_.size(), 0);
        status->motor_fresh.assign(motors_.size(), 0);
        status->motor_age_s.assign(motors_.size(), 0.0);
        bool waiting_for_first_sample = false;
        bool feedback_timed_out = false;
        for (size_t i = 0; i < motors_.size(); ++i) {
            motor_state current{};
            if (motor_get_state_one(motors_[i], &current) == 0) {
                (*motors)[i] = current;
                motor_feedback_valid_[i] = true;
                motor_feedback_time_s_[i] = now;
            } else if (!motor_feedback_valid_[i]) {
                if (now - feedback_start_time_s_ > config_.startup_feedback_timeout_s)
                    feedback_timed_out = true;
                else
                    waiting_for_first_sample = true;
            } else if (now - motor_feedback_time_s_[i] > config_.feedback_timeout_s) {
                feedback_timed_out = true;
            }
            status->motor_received[i] = motor_feedback_valid_[i];
            status->motor_age_s[i] = motor_feedback_valid_[i]
                ? now - motor_feedback_time_s_[i]
                : now - feedback_start_time_s_;
            status->motor_fresh[i] = motor_feedback_valid_[i] &&
                status->motor_age_s[i] <= config_.feedback_timeout_s;
        }

        imu_data current_imu{};
        if (imu_read(imu_, &current_imu) == 0) {
            *imu = current_imu;
            imu_feedback_valid_ = true;
            imu_feedback_time_s_ = now;
        } else if (!imu_feedback_valid_) {
            if (now - feedback_start_time_s_ > config_.startup_feedback_timeout_s)
                feedback_timed_out = true;
            else
                waiting_for_first_sample = true;
        } else if (now - imu_feedback_time_s_ > config_.feedback_timeout_s) {
            feedback_timed_out = true;
        }
        status->imu_received = imu_feedback_valid_;
        status->imu_age_s = imu_feedback_valid_
            ? now - imu_feedback_time_s_
            : now - feedback_start_time_s_;
        status->imu_fresh = imu_feedback_valid_ &&
            status->imu_age_s <= config_.feedback_timeout_s;
        if (feedback_timed_out) return DEVICE_READ_ERROR;
        return waiting_for_first_sample ? DEVICE_READ_WAITING : DEVICE_READ_OK;
    }

    int Write(const std::vector<motor_cmd> &commands) override {
        if (!initialized_ || commands.size() != motors_.size()) return -1;
        wrote_command_ = true;
        return motor_set_cmds(motors_.data(), commands.data(), motors_.size());
    }

    void Shutdown() override {
        if (motors_.empty() && !imu_) return;
        if (initialized_ && wrote_command_) {
            std::vector<motor_cmd> idle(motors_.size());
            for (auto &command : idle) command.mode = MOTOR_MODE_IDLE;
            motor_set_cmds(motors_.data(), idle.data(), motors_.size());
        }
        if (!motors_.empty()) {
            motor_free(motors_.data(), motors_.size());
            motors_.clear();
        }
        if (imu_) {
            imu_free(imu_);
            imu_ = nullptr;
        }
        initialized_ = false;
    }

private:
    void ReleaseAllocated() {
        if (!motors_.empty()) {
            motor_free(motors_.data(), motors_.size());
            motors_.clear();
        }
        if (imu_) {
            imu_free(imu_);
            imu_ = nullptr;
        }
    }

    RuntimeConfig config_;
    std::vector<motor_dev *> motors_;
    std::vector<bool> motor_feedback_valid_;
    std::vector<double> motor_feedback_time_s_;
    imu_dev *imu_ = nullptr;
    double feedback_start_time_s_ = 0.0;
    double imu_feedback_time_s_ = 0.0;
    bool imu_feedback_valid_ = false;
    bool initialized_ = false;
    bool wrote_command_ = false;
};

}  // namespace

std::vector<DriverOption> BuildMotorDriverOptions(const MotorConfig &motor) {
    std::vector<DriverOption> options = motor.driver_options;
    options.push_back({"model", motor.model});
    options.push_back({"feedback_id", std::to_string(motor.feedback_id)});
    return options;
}

std::unique_ptr<DeviceManager> CreatePeripheralDevices(const RuntimeConfig &config) {
    return std::make_unique<PeripheralDevices>(config);
}

}  // namespace whole_body

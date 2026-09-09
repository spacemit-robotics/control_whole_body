/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file device_manager.h
 * @brief Internal motor and IMU device abstraction
 */

#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

extern "C" {
#include "imu.h"
#include "motor.h"
}

#include "whole_body.h"
#include "whole_body_config.h"

namespace whole_body {

enum DeviceReadResult {
    DEVICE_READ_INCOMPATIBLE = -2,
    DEVICE_READ_ERROR = -1,
    DEVICE_READ_OK = 0,
    DEVICE_READ_WAITING = 1,
};

struct DeviceFeedbackStatus {
    std::vector<uint8_t> motor_received;
    std::vector<uint8_t> motor_fresh;
    std::vector<double> motor_age_s;
    std::vector<double> motor_timestamp_s;
    std::vector<whole_body_feedback_timestamp_source> motor_timestamp_source;
    bool imu_received = false;
    bool imu_fresh = false;
    double imu_age_s = 0.0;
    double imu_sample_timestamp_s = 0.0;
    double imu_receive_timestamp_s = 0.0;
    double feedback_window_s = 0.0;
    imu_diagnostics imu_parser{};
};

class DeviceManager {
public:
    virtual ~DeviceManager() = default;
    virtual int Init() = 0;
    virtual int Read(std::vector<motor_state> *motors, imu_data *imu,
        DeviceFeedbackStatus *status) = 0;
    virtual int Write(const std::vector<motor_cmd> &commands) = 0;
    virtual void Shutdown() = 0;
    const std::string &LastWriteError() const { return last_write_error_; }

protected:
    std::string last_write_error_;
};

using MotorCommandWriter = int (*)(motor_dev *, const motor_cmd *);
int WriteMotorCommands(const RuntimeConfig &config, const std::vector<motor_dev *> &motors,
    const std::vector<motor_cmd> &commands, std::string *error,
    MotorCommandWriter write_command = motor_set_cmd_one);

std::vector<DriverOption> BuildMotorDriverOptions(const MotorConfig &motor);
std::unique_ptr<DeviceManager> CreatePeripheralDevices(const RuntimeConfig &config);

}  // namespace whole_body

#endif  // DEVICE_MANAGER_H

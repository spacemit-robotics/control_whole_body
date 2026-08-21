/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body_core.h
 * @brief Internal whole-body control core
 */

#ifndef WHOLE_BODY_CORE_H
#define WHOLE_BODY_CORE_H

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "device_manager.h"
#include "parallel_ankle.h"
#include "whole_body.h"
#include "whole_body_config.h"

namespace whole_body {

class WholeBodyCore {
    public:
    WholeBodyCore(RuntimeConfig config, std::unique_ptr<DeviceManager> devices);
    ~WholeBodyCore();

    int Init();
    int Read(whole_body_state *state);
    int Write(const whole_body_joint_command &command, double monotonic_time_s);
    int Tick(double monotonic_time_s);
    int SetMode(whole_body_mode mode);
    whole_body_health GetHealth() const;
    whole_body_diagnostics GetDiagnostics() const;
    double CycleSeconds() const;
    const std::string &LastError() const;

    private:
    struct CouplingRuntime {
        ParallelAnkle mapping;
        std::array<size_t, 2> joint_indices;
        std::array<size_t, 2> motor_indices;
    };

    int Fail(int error, const std::string &message);
    bool ValidateCommand(
        const whole_body_joint_command &command, std::string *reason) const;
    bool ValidateJointPositions(
        const whole_body_state &state, std::string *reason) const;
    int BuildMotorCommands(
        const whole_body_joint_command &command, std::vector<motor_cmd> *motor_commands);
    int SendIdle();
    int EnterSafety(int error, const std::string &message);
    std::string DescribeFeedbackProblem(const std::string &prefix) const;
    std::string DescribeMotorErrors() const;

    RuntimeConfig config_;
    std::unique_ptr<DeviceManager> devices_;
    std::unordered_map<std::string, size_t> motor_indices_;
    std::unordered_map<std::string, size_t> joint_indices_;
    std::vector<CouplingRuntime> couplings_;
    std::vector<motor_state> motor_states_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<std::array<double, 2>> reference_position_limits_;
    DeviceFeedbackStatus feedback_status_;
    imu_data imu_state_{};
    whole_body_state last_state_{};
    whole_body_health health_{};
    whole_body_mode mode_ = WHOLE_BODY_MODE_POWER_OFF;
    std::string last_error_;
    double last_command_time_s_ = 0.0;
    double last_idle_time_s_ = 0.0;
    bool initialized_ = false;
    bool has_command_ = false;
    bool has_joint_state_ = false;
    bool watchdog_active_ = false;
    bool fault_latched_ = false;
};

}  // namespace whole_body

#endif  // WHOLE_BODY_CORE_H

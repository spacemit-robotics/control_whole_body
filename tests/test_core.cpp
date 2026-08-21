/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_core.cpp
 * @brief Offline tests for whole-body mapping and safety gates
 */

#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "whole_body_core.h"

namespace {

class DummyDevices final : public whole_body::DeviceManager {
    public:
    int Init() override {
        initialized = true;
        return 0;
    }

    int Read(std::vector<motor_state> *motors, imu_data *imu,
        whole_body::DeviceFeedbackStatus *status) override {
        if (!initialized || !motors || motors->size() != feedback.size() || !imu ||
            !status) {
            return -1;
        }
        status->motor_received.assign(feedback.size(), read_result == whole_body::DEVICE_READ_OK);
        status->motor_fresh.assign(feedback.size(), read_result == whole_body::DEVICE_READ_OK);
        status->motor_age_s.assign(feedback.size(), 0.001);
        status->imu_received = read_result == whole_body::DEVICE_READ_OK;
        status->imu_fresh = read_result == whole_body::DEVICE_READ_OK;
        status->imu_age_s = 0.001;
        if (read_result != whole_body::DEVICE_READ_OK) return read_result;
        *motors = feedback;
        std::memset(imu, 0, sizeof(*imu));
        imu->timestamp_us = 1000000;
        imu->quat[0] = 1.0f;
        return 0;
    }

    int Write(const std::vector<motor_cmd> &commands) override {
        if (!initialized) return -1;
        last_commands = commands;
        ++write_count;
        return 0;
    }

    void Shutdown() override { initialized = false; }

    std::vector<motor_state> feedback = std::vector<motor_state>(2);
    std::vector<motor_cmd> last_commands;
    int write_count = 0;
    int read_result = 0;
    bool initialized = false;
};

whole_body::RuntimeConfig MakeConfig(bool read_only) {
    whole_body::RuntimeConfig config;
    config.num_dof = 2;
    config.cycle_s = 0.001;
    config.startup_feedback_timeout_s = 1.0;
    config.feedback_timeout_s = 0.04;
    config.command_timeout_s = 0.1;
    config.read_only = read_only;
    config.allow_actuation = !read_only;
    config.joint_names = {"joint_0", "joint_1"};
    config.buses.push_back({"bus_0", "socketcan", "can0", 1000000});
    config.motors.resize(2);
    config.motors[0].name = "motor_0";
    config.motors[0].driver = "driver_0";
    config.motors[0].model = "model_0";
    config.motors[0].bus = "bus_0";
    config.motors[0].command_id = 1;
    config.motors[0].feedback_id = 17;
    config.motors[0].polarity = -1.0;
    config.motors[0].zero_offset = 0.1;
    config.motors[1].name = "motor_1";
    config.motors[1].driver = "driver_1";
    config.motors[1].model = "model_1";
    config.motors[1].bus = "bus_0";
    config.motors[1].command_id = 2;
    config.motors[1].feedback_id = 18;
    config.motors[1].polarity = 1.0;
    config.motors[1].zero_offset = -0.2;
    config.joints.resize(2);
    for (size_t i = 0; i < 2; ++i) {
        config.joints[i].name = config.joint_names[i];
        config.joints[i].mapping = "direct";
        config.joints[i].motors = {config.motors[i].name};
        config.joints[i].position_limit = {-2.0, 2.0};
        config.joints[i].velocity_limit = 5.0;
        config.joints[i].torque_limit = 10.0;
    }
    return config;
}

whole_body::ParallelAnkleConfig MakeParallelAnkleConfig() {
    whole_body::ParallelAnkleConfig config;
    config.joints = {"joint_0", "joint_1"};
    config.motors = {"motor_0", "motor_1"};
    config.ankle_pivots = {
        whole_body::Vec3{-0.066, 0.03575, 0.0},
        whole_body::Vec3{-0.066, -0.03575, 0.0},
    };
    config.ball_positions = {
        whole_body::Vec3{-0.066, 0.046, 0.297},
        whole_body::Vec3{-0.066, -0.046, 0.186},
    };
    config.motor_positions = {
        whole_body::Vec3{0.0, 0.046, 0.297},
        whole_body::Vec3{0.0, -0.046, 0.186},
    };
    config.motor_limit = 1.5;
    config.ankle_limit = 1.5;
    config.motor_difference_limit = 1.3;
    config.max_iterations = 50;
    config.squared_tolerance = 1.0e-10;
    return config;
}

whole_body::RuntimeConfig MakeParallelConfig() {
    auto config = MakeConfig(false);
    for (size_t i = 0; i < config.motors.size(); ++i) {
        config.motors[i].polarity = 1.0;
        config.motors[i].zero_offset = 0.0;
        config.joints[i].mapping = "parallel_ankle";
        config.joints[i].motors = {"motor_0", "motor_1"};
        config.joints[i].position_limit = {-0.25, 0.25};
        config.joints[i].impedance.mode = whole_body::ImpedanceMode::kSoftware;
    }
    config.couplings = {MakeParallelAnkleConfig()};
    return config;
}

whole_body_joint_command MakeCommand() {
    whole_body_joint_command command{};
    command.num_dof = 2;
    command.enable = true;
    command.mode = WHOLE_BODY_MODE_RL;
    command.actuation_mode = WHOLE_BODY_ACTUATION_HYBRID;
    command.position[0] = 0.5;
    command.position[1] = -0.4;
    command.velocity[0] = 0.2;
    command.velocity[1] = -0.3;
    command.torque[0] = 1.0;
    command.torque[1] = -2.0;
    command.kp[0] = command.kp[1] = 20.0;
    command.kd[0] = command.kd[1] = 1.0;
    return command;
}

}  // namespace

int main() {
    auto devices = std::make_unique<DummyDevices>();
    DummyDevices *devices_ptr = devices.get();
    devices_ptr->feedback[0].pos = 1.1f;
    devices_ptr->feedback[1].pos = 0.3f;
    whole_body::WholeBodyCore core(MakeConfig(false), std::move(devices));
    assert(core.Init() == WHOLE_BODY_OK);
    whole_body_state state{};
    devices_ptr->read_result = whole_body::DEVICE_READ_WAITING;
    assert(core.Read(&state) == WHOLE_BODY_ERR_STATE);
    assert(core.LastError().find("motor_0") != std::string::npos);
    assert(core.LastError().find("imu") != std::string::npos);
    assert(devices_ptr->write_count == 1);
    assert(devices_ptr->last_commands[0].mode == MOTOR_MODE_IDLE);
    devices_ptr->read_result = whole_body::DEVICE_READ_OK;
    assert(core.Read(&state) == WHOLE_BODY_OK);
    assert(std::abs(state.position[0] + 1.0) < 1.0e-6);
    assert(std::abs(state.position[1] - 0.5) < 1.0e-6);
    const auto diagnostics = core.GetDiagnostics();
    assert(diagnostics.motor_count == 2);
    assert(diagnostics.joint_count == 2);
    assert(std::string(diagnostics.motors[0].name) == "motor_0");
    assert(std::string(diagnostics.motors[0].joint_names) == "joint_0");
    assert(std::string(diagnostics.motors[0].device) == "can0");
    assert(diagnostics.motors[0].feedback_received);
    assert(diagnostics.motors[0].feedback_fresh);
    assert(std::abs(diagnostics.motors[0].raw_position - 1.1) < 1.0e-6);
    assert(std::abs(diagnostics.motors[0].calibrated_position + 1.0) < 1.0e-6);
    assert(std::string(diagnostics.joints[0].name) == "joint_0");
    assert(diagnostics.joints[0].feedback_valid);
    assert(diagnostics.imu.feedback_received);
    assert(diagnostics.imu.feedback_fresh);

    const auto command = MakeCommand();
    assert(core.Write(command, 1.0) == WHOLE_BODY_OK);
    assert(devices_ptr->write_count == 2);
    assert(devices_ptr->last_commands[0].mode == MOTOR_MODE_HYBRID);
    assert(std::abs(devices_ptr->last_commands[0].pos_des + 0.4f) < 1.0e-6f);
    assert(std::abs(devices_ptr->last_commands[0].trq_des + 1.0f) < 1.0e-6f);
    assert(std::abs(devices_ptr->last_commands[1].pos_des + 0.6f) < 1.0e-6f);
    assert(core.Tick(1.2) == WHOLE_BODY_ERR_TIMEOUT);
    assert(devices_ptr->write_count == 3);
    assert(devices_ptr->last_commands[0].mode == MOTOR_MODE_IDLE);
    assert(core.GetHealth().watchdog_events == 1);
    assert(core.Read(&state) == WHOLE_BODY_OK);
    assert(core.GetHealth().state == WHOLE_BODY_HEALTH_WATCHDOG);
    assert(core.Write(command, 1.3) == WHOLE_BODY_ERR_TIMEOUT);
    assert(devices_ptr->write_count == 3);
    assert(core.SetMode(WHOLE_BODY_MODE_POWER_OFF) == WHOLE_BODY_OK);
    assert(devices_ptr->write_count == 4);

    auto damp_command = MakeCommand();
    damp_command.mode = WHOLE_BODY_MODE_DAMP;
    damp_command.actuation_mode = WHOLE_BODY_ACTUATION_TORQUE;
    assert(core.Write(damp_command, 1.4) == WHOLE_BODY_OK);
    assert(devices_ptr->last_commands[0].mode == MOTOR_MODE_HYBRID);
    assert(devices_ptr->last_commands[0].kp == 0.0f);
    assert(devices_ptr->last_commands[0].trq_des == 0.0f);
    assert(std::abs(devices_ptr->last_commands[0].pos_des - 0.1f) < 1.0e-6f);

    devices_ptr->read_result = -1;
    assert(core.Read(&state) == WHOLE_BODY_ERR_DEVICE);
    assert(devices_ptr->write_count == 6);
    assert(devices_ptr->last_commands[0].mode == MOTOR_MODE_IDLE);
    assert(core.GetHealth().state == WHOLE_BODY_HEALTH_ERROR);
    devices_ptr->read_result = 0;
    assert(core.Read(&state) == WHOLE_BODY_OK);
    assert(core.GetHealth().state == WHOLE_BODY_HEALTH_ERROR);
    assert(core.Write(command, 1.5) == WHOLE_BODY_ERR_DEVICE);
    assert(devices_ptr->write_count == 6);
    assert(core.SetMode(WHOLE_BODY_MODE_POWER_OFF) == WHOLE_BODY_OK);

    assert(core.Write(command, 1.6) == WHOLE_BODY_OK);
    auto out_of_range_command = MakeCommand();
    out_of_range_command.position[1] = -2.1;
    assert(core.Write(out_of_range_command, 1.7) == WHOLE_BODY_ERR_COMMAND);
    assert(core.LastError().find("joint_1.position") != std::string::npos);
    assert(core.LastError().find("reference range") != std::string::npos);
    assert(devices_ptr->write_count == 9);
    assert(devices_ptr->last_commands[0].mode == MOTOR_MODE_IDLE);
    assert(core.Write(command, 1.8) == WHOLE_BODY_ERR_COMMAND);
    assert(devices_ptr->write_count == 9);
    assert(core.SetMode(WHOLE_BODY_MODE_POWER_OFF) == WHOLE_BODY_OK);

    assert(core.Write(command, 1.9) == WHOLE_BODY_OK);
    assert(core.SetMode(WHOLE_BODY_MODE_SAFETY) == WHOLE_BODY_OK);
    assert(devices_ptr->write_count == 12);
    assert(devices_ptr->last_commands[0].mode == MOTOR_MODE_IDLE);
    assert(core.Write(command, 2.0) == WHOLE_BODY_ERR_STATE);
    assert(devices_ptr->write_count == 12);
    assert(core.SetMode(WHOLE_BODY_MODE_POWER_OFF) == WHOLE_BODY_OK);

    devices_ptr->feedback[0].pos = NAN;
    assert(core.Read(&state) == WHOLE_BODY_ERR_STATE);
    assert(devices_ptr->write_count == 14);
    devices_ptr->feedback[0].pos = 1.1f;
    assert(core.SetMode(WHOLE_BODY_MODE_POWER_OFF) == WHOLE_BODY_OK);
    assert(core.Read(&state) == WHOLE_BODY_OK);

    auto no_feedback_devices = std::make_unique<DummyDevices>();
    DummyDevices *no_feedback_ptr = no_feedback_devices.get();
    whole_body::WholeBodyCore no_feedback_core(
        MakeParallelConfig(), std::move(no_feedback_devices));
    assert(no_feedback_core.Init() == WHOLE_BODY_OK);
    assert(no_feedback_core.Write(MakeCommand(), 1.0) == WHOLE_BODY_ERR_STATE);
    assert(no_feedback_ptr->last_commands[0].mode == MOTOR_MODE_IDLE);

    const whole_body::ParallelAnkle parallel_mapping(MakeParallelAnkleConfig());
    const std::array<double, 2> actual_joint_position = {0.0, 0.0};
    const std::array<double, 2> actual_joint_velocity = {0.1, -0.2};
    std::array<double, 2> actual_motor_velocity;
    assert(parallel_mapping.JointVelocityToMotor(
        actual_joint_position, actual_joint_velocity, &actual_motor_velocity));
    auto parallel_devices = std::make_unique<DummyDevices>();
    DummyDevices *parallel_devices_ptr = parallel_devices.get();
    parallel_devices_ptr->feedback[0].vel = static_cast<float>(actual_motor_velocity[0]);
    parallel_devices_ptr->feedback[1].vel = static_cast<float>(actual_motor_velocity[1]);
    whole_body::WholeBodyCore parallel_core(
        MakeParallelConfig(), std::move(parallel_devices));
    assert(parallel_core.Init() == WHOLE_BODY_OK);
    whole_body_state parallel_state{};
    assert(parallel_core.Read(&parallel_state) == WHOLE_BODY_OK);
    assert(std::abs(parallel_state.velocity[0] - actual_joint_velocity[0]) < 1.0e-6);
    assert(std::abs(parallel_state.velocity[1] - actual_joint_velocity[1]) < 1.0e-6);

    auto parallel_command = MakeCommand();
    parallel_command.kp[0] = 4.0;
    parallel_command.kp[1] = 6.0;
    parallel_command.kd[0] = 1.0;
    parallel_command.kd[1] = 2.0;
    assert(parallel_core.Write(parallel_command, 1.0) == WHOLE_BODY_OK);
    const std::array<double, 2> target_joint_position = {
        parallel_command.position[0], parallel_command.position[1]};
    const std::array<double, 2> target_joint_velocity = {
        parallel_command.velocity[0], parallel_command.velocity[1]};
    const std::array<double, 2> expected_joint_torque = {
        parallel_command.torque[0] + parallel_command.kp[0] *
                (target_joint_position[0] - parallel_state.position[0]) +
            parallel_command.kd[0] *
                (target_joint_velocity[0] - parallel_state.velocity[0]),
        parallel_command.torque[1] + parallel_command.kp[1] *
                (target_joint_position[1] - parallel_state.position[1]) +
            parallel_command.kd[1] *
                (target_joint_velocity[1] - parallel_state.velocity[1]),
    };
    std::array<double, 2> expected_motor_position;
    std::array<double, 2> expected_motor_velocity;
    std::array<double, 2> expected_motor_torque;
    assert(parallel_mapping.JointToMotor(actual_joint_position, &expected_motor_position));
    assert(parallel_mapping.JointVelocityToMotor(
        actual_joint_position, target_joint_velocity, &expected_motor_velocity));
    assert(parallel_mapping.JointTorqueToMotor(
        actual_joint_position, expected_joint_torque, &expected_motor_torque));
    for (size_t i = 0; i < 2; ++i) {
        const auto &output = parallel_devices_ptr->last_commands[i];
        assert(output.mode == MOTOR_MODE_HYBRID);
        assert(std::abs(output.pos_des - expected_motor_position[i]) < 1.0e-6);
        assert(std::abs(output.vel_des - expected_motor_velocity[i]) < 1.0e-6);
        assert(std::abs(output.trq_des - expected_motor_torque[i]) < 1.0e-5);
        assert(output.kp == 0.0f);
        assert(output.kd == 0.0f);
    }

    std::array<double, 2> outside_motor_position;
    assert(parallel_mapping.JointToMotor({0.3, 0.0}, &outside_motor_position));
    auto limit_devices = std::make_unique<DummyDevices>();
    DummyDevices *limit_devices_ptr = limit_devices.get();
    limit_devices_ptr->feedback[0].pos = static_cast<float>(outside_motor_position[0]);
    limit_devices_ptr->feedback[1].pos = static_cast<float>(outside_motor_position[1]);
    whole_body::WholeBodyCore limit_core(
        MakeParallelConfig(), std::move(limit_devices));
    assert(limit_core.Init() == WHOLE_BODY_OK);
    whole_body_state limit_state{};
    assert(limit_core.Read(&limit_state) == WHOLE_BODY_OK);
    auto neutral_command = MakeCommand();
    neutral_command.position[0] = 0.0;
    neutral_command.position[1] = 0.0;
    assert(limit_core.Write(neutral_command, 1.0) == WHOLE_BODY_ERR_STATE);
    assert(limit_core.LastError().find("joint_0 measured position") != std::string::npos);

    auto parallel_position_command = parallel_command;
    parallel_position_command.actuation_mode = WHOLE_BODY_ACTUATION_POSITION;
    assert(parallel_core.Write(parallel_position_command, 1.01) == WHOLE_BODY_OK);
    const std::array<double, 2> position_joint_torque = {
        parallel_position_command.kp[0] *
                (target_joint_position[0] - parallel_state.position[0]) -
            parallel_position_command.kd[0] * parallel_state.velocity[0],
        parallel_position_command.kp[1] *
                (target_joint_position[1] - parallel_state.position[1]) -
            parallel_position_command.kd[1] * parallel_state.velocity[1],
    };
    assert(parallel_mapping.JointTorqueToMotor(
        actual_joint_position, position_joint_torque, &expected_motor_torque));
    for (size_t i = 0; i < 2; ++i) {
        const auto &output = parallel_devices_ptr->last_commands[i];
        assert(output.mode == MOTOR_MODE_HYBRID);
        assert(std::abs(output.trq_des - expected_motor_torque[i]) < 1.0e-6);
        assert(output.kp == 0.0f);
        assert(output.kd == 0.0f);
    }

    auto parallel_velocity_command = parallel_command;
    parallel_velocity_command.actuation_mode = WHOLE_BODY_ACTUATION_VELOCITY;
    assert(parallel_core.Write(parallel_velocity_command, 1.02) == WHOLE_BODY_OK);
    const std::array<double, 2> velocity_joint_torque = {
        parallel_velocity_command.kd[0] *
            (target_joint_velocity[0] - parallel_state.velocity[0]),
        parallel_velocity_command.kd[1] *
            (target_joint_velocity[1] - parallel_state.velocity[1]),
    };
    assert(parallel_mapping.JointTorqueToMotor(
        actual_joint_position, velocity_joint_torque, &expected_motor_torque));
    for (size_t i = 0; i < 2; ++i) {
        const auto &output = parallel_devices_ptr->last_commands[i];
        assert(output.mode == MOTOR_MODE_HYBRID);
        assert(std::abs(output.trq_des - expected_motor_torque[i]) < 1.0e-6);
        assert(output.kp == 0.0f);
        assert(output.kd == 0.0f);
    }

    auto parallel_damp_command = parallel_command;
    parallel_damp_command.mode = WHOLE_BODY_MODE_DAMP;
    parallel_damp_command.actuation_mode = WHOLE_BODY_ACTUATION_TORQUE;
    assert(parallel_core.Write(parallel_damp_command, 1.03) == WHOLE_BODY_OK);
    const std::array<double, 2> damp_joint_torque = {
        -parallel_damp_command.kd[0] * parallel_state.velocity[0],
        -parallel_damp_command.kd[1] * parallel_state.velocity[1],
    };
    assert(parallel_mapping.JointTorqueToMotor(
        actual_joint_position, damp_joint_torque, &expected_motor_torque));
    for (size_t i = 0; i < 2; ++i) {
        const auto &output = parallel_devices_ptr->last_commands[i];
        assert(output.mode == MOTOR_MODE_HYBRID);
        assert(std::abs(output.trq_des - expected_motor_torque[i]) < 1.0e-6);
        assert(output.kp == 0.0f);
        assert(output.kd == 0.0f);
    }

    auto parallel_torque_command = parallel_command;
    parallel_torque_command.actuation_mode = WHOLE_BODY_ACTUATION_TORQUE;
    assert(parallel_core.Write(parallel_torque_command, 1.04) == WHOLE_BODY_OK);
    const std::array<double, 2> direct_joint_torque = {
        parallel_torque_command.torque[0], parallel_torque_command.torque[1]};
    assert(parallel_mapping.JointTorqueToMotor(
        actual_joint_position, direct_joint_torque, &expected_motor_torque));
    for (size_t i = 0; i < 2; ++i) {
        const auto &output = parallel_devices_ptr->last_commands[i];
        assert(output.mode == MOTOR_MODE_TRQ);
        assert(std::abs(output.trq_des - expected_motor_torque[i]) < 1.0e-6);
        assert(output.kp == 0.0f);
        assert(output.kd == 0.0f);
    }

    auto invalid_parallel_reference = parallel_command;
    invalid_parallel_reference.position[0] = 1.51;
    assert(parallel_core.Write(invalid_parallel_reference, 1.05) == WHOLE_BODY_ERR_COMMAND);
    assert(parallel_core.LastError().find("joint_0.position") != std::string::npos);
    assert(parallel_core.LastError().find("[-1.5, 1.5]") != std::string::npos);

    auto read_only_devices = std::make_unique<DummyDevices>();
    DummyDevices *read_only_ptr = read_only_devices.get();
    whole_body::WholeBodyCore read_only_core(MakeConfig(true), std::move(read_only_devices));
    assert(read_only_core.Init() == WHOLE_BODY_OK);
    assert(read_only_core.Write(command, 1.0) == WHOLE_BODY_ERR_READ_ONLY);
    assert(read_only_core.SetMode(WHOLE_BODY_MODE_RL) == WHOLE_BODY_ERR_READ_ONLY);
    assert(read_only_core.SetMode(WHOLE_BODY_MODE_SAFETY) == WHOLE_BODY_OK);
    assert(read_only_ptr->write_count == 0);

    auto mode_devices = std::make_unique<DummyDevices>();
    DummyDevices *mode_devices_ptr = mode_devices.get();
    whole_body::WholeBodyCore mode_core(MakeConfig(false), std::move(mode_devices));
    assert(mode_core.Init() == WHOLE_BODY_OK);
    assert(mode_devices_ptr->write_count == 1);
    assert(mode_devices_ptr->last_commands[0].mode == MOTOR_MODE_IDLE);
    assert(mode_core.Tick(1.0) == WHOLE_BODY_OK);
    assert(mode_devices_ptr->write_count == 2);
    auto mode_command = MakeCommand();
    mode_command.mode = WHOLE_BODY_MODE_HOME;
    assert(mode_core.Write(mode_command, 1.0) == WHOLE_BODY_OK);
    assert(mode_devices_ptr->last_commands[0].mode == MOTOR_MODE_HYBRID);
    mode_command.mode = WHOLE_BODY_MODE_RL;
    mode_command.actuation_mode = WHOLE_BODY_ACTUATION_POSITION;
    assert(mode_core.Write(mode_command, 1.0) == WHOLE_BODY_OK);
    assert(mode_devices_ptr->last_commands[0].mode == MOTOR_MODE_POS);
    mode_command.actuation_mode = WHOLE_BODY_ACTUATION_VELOCITY;
    assert(mode_core.Write(mode_command, 1.01) == WHOLE_BODY_OK);
    assert(mode_devices_ptr->last_commands[0].mode == MOTOR_MODE_VEL);
    mode_command.actuation_mode = WHOLE_BODY_ACTUATION_TORQUE;
    assert(mode_core.Write(mode_command, 1.02) == WHOLE_BODY_OK);
    assert(mode_devices_ptr->last_commands[0].mode == MOTOR_MODE_TRQ);
    assert(std::abs(mode_devices_ptr->last_commands[0].trq_des + 1.0f) < 1.0e-6f);

    auto split_config = MakeConfig(false);
    split_config.motors[0].polarity = 1.0;
    split_config.motors[0].zero_offset = 0.0;
    split_config.joints[0].torque_limit = 100.0;
    split_config.joints[0].impedance.mode = whole_body::ImpedanceMode::kSplit;
    split_config.joints[0].impedance.motor_kp_max = 10.0;
    split_config.joints[0].impedance.motor_kd_max = 5.0;
    auto split_devices = std::make_unique<DummyDevices>();
    DummyDevices *split_devices_ptr = split_devices.get();
    split_devices_ptr->feedback[0].pos = 0.1f;
    split_devices_ptr->feedback[0].vel = -0.2f;
    whole_body::WholeBodyCore split_core(std::move(split_config), std::move(split_devices));
    assert(split_core.Init() == WHOLE_BODY_OK);
    whole_body_state split_state{};
    assert(split_core.Read(&split_state) == WHOLE_BODY_OK);
    auto split_command = MakeCommand();
    split_command.kd[0] = 12.0;
    assert(split_core.Write(split_command, 1.0) == WHOLE_BODY_OK);
    const auto &split_output = split_devices_ptr->last_commands[0];
    const double requested_torque = 1.0 + 20.0 * (0.5 - 0.1) +
        12.0 * (0.2 - (-0.2));
    const double motor_pd_torque = 10.0 * (0.5 - 0.1) +
        5.0 * (0.2 - (-0.2));
    assert(split_output.mode == MOTOR_MODE_HYBRID);
    assert(std::abs(split_output.kp - 10.0f) < 1.0e-6f);
    assert(std::abs(split_output.kd - 5.0f) < 1.0e-6f);
    assert(std::abs(split_output.trq_des -
        static_cast<float>(requested_torque - motor_pd_torque)) < 1.0e-5f);

    auto split_damp_command = split_command;
    split_damp_command.mode = WHOLE_BODY_MODE_DAMP;
    assert(split_core.Write(split_damp_command, 1.01) == WHOLE_BODY_OK);
    const auto &split_damp_output = split_devices_ptr->last_commands[0];
    assert(split_damp_output.mode == MOTOR_MODE_HYBRID);
    assert(split_damp_output.kp == 0.0f);
    assert(std::abs(split_damp_output.kd - 5.0f) < 1.0e-6f);
    assert(std::abs(split_damp_output.trq_des - 1.4f) < 1.0e-5f);

    auto software_config = MakeConfig(false);
    software_config.motors[0].polarity = 1.0;
    software_config.motors[0].zero_offset = 0.0;
    software_config.joints[0].torque_limit = 100.0;
    software_config.joints[0].impedance.mode = whole_body::ImpedanceMode::kSoftware;
    auto software_devices = std::make_unique<DummyDevices>();
    DummyDevices *software_devices_ptr = software_devices.get();
    software_devices_ptr->feedback[0].pos = 0.1f;
    software_devices_ptr->feedback[0].vel = -0.2f;
    whole_body::WholeBodyCore software_core(
        std::move(software_config), std::move(software_devices));
    assert(software_core.Init() == WHOLE_BODY_OK);
    whole_body_state software_state{};
    assert(software_core.Read(&software_state) == WHOLE_BODY_OK);
    assert(software_core.Write(split_command, 1.0) == WHOLE_BODY_OK);
    const auto &software_output = software_devices_ptr->last_commands[0];
    assert(software_output.mode == MOTOR_MODE_HYBRID);
    assert(software_output.kp == 0.0f);
    assert(software_output.kd == 0.0f);
    assert(std::abs(software_output.trq_des - static_cast<float>(requested_torque)) < 1.0e-5f);

    auto safety_unload_command = MakeCommand();
    safety_unload_command.mode = WHOLE_BODY_MODE_SAFETY;
    assert(mode_core.Write(safety_unload_command, 1.03) == WHOLE_BODY_OK);
    assert(mode_devices_ptr->last_commands[0].mode == MOTOR_MODE_HYBRID);
    assert(mode_core.Write(safety_unload_command, 1.04) == WHOLE_BODY_OK);
    safety_unload_command.enable = false;
    assert(mode_core.Write(safety_unload_command, 1.05) == WHOLE_BODY_OK);
    assert(mode_devices_ptr->last_commands[0].mode == MOTOR_MODE_IDLE);
    assert(mode_core.Write(mode_command, 1.06) == WHOLE_BODY_ERR_STATE);
    assert(mode_core.SetMode(WHOLE_BODY_MODE_POWER_OFF) == WHOLE_BODY_OK);

    mode_command.actuation_mode = static_cast<whole_body_actuation_mode>(-1);
    assert(mode_core.Write(mode_command, 1.07) == WHOLE_BODY_ERR_COMMAND);
    assert(mode_devices_ptr->last_commands[0].mode == MOTOR_MODE_IDLE);
    std::cout << "Whole-body core tests passed\n";
    return 0;
}

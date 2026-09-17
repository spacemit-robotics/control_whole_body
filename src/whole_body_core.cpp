/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body_core.cpp
 * @brief Internal whole-body control core implementation
 */

#include "whole_body_core.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace whole_body {
namespace {

constexpr double kPositionLimitTolerance = 1.0e-3;

double MonotonicTime() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool IsValidMode(whole_body_mode mode) {
    return mode >= WHOLE_BODY_MODE_POWER_OFF && mode <= WHOLE_BODY_MODE_HOME;
}

bool IsValidActuationMode(whole_body_actuation_mode mode) {
    return mode >= WHOLE_BODY_ACTUATION_HYBRID && mode <= WHOLE_BODY_ACTUATION_TORQUE;
}

bool MotorModeUsesPosition(uint32_t mode) {
    return mode == MOTOR_MODE_HYBRID || mode == MOTOR_MODE_POS;
}

bool MotorModeUsesVelocity(uint32_t mode) {
    return mode == MOTOR_MODE_HYBRID || mode == MOTOR_MODE_POS ||
        mode == MOTOR_MODE_VEL;
}

bool MotorModeUsesTorque(uint32_t mode) {
    return mode == MOTOR_MODE_HYBRID || mode == MOTOR_MODE_TRQ;
}

bool MotorModeUsesGains(uint32_t mode) {
    return mode == MOTOR_MODE_HYBRID;
}

motor_mode ToMotorMode(whole_body_actuation_mode mode) {
    switch (mode) {
        case WHOLE_BODY_ACTUATION_HYBRID:
            return MOTOR_MODE_HYBRID;
        case WHOLE_BODY_ACTUATION_POSITION:
            return MOTOR_MODE_POS;
        case WHOLE_BODY_ACTUATION_VELOCITY:
            return MOTOR_MODE_VEL;
        case WHOLE_BODY_ACTUATION_TORQUE:
            return MOTOR_MODE_TRQ;
    }
    return MOTOR_MODE_IDLE;
}

bool UsesPositionTarget(const whole_body_joint_command &command) {
    return command.mode != WHOLE_BODY_MODE_DAMP &&
        (command.actuation_mode == WHOLE_BODY_ACTUATION_HYBRID ||
            command.actuation_mode == WHOLE_BODY_ACTUATION_POSITION);
}

bool UsesVelocityTarget(const whole_body_joint_command &command) {
    return command.mode != WHOLE_BODY_MODE_DAMP &&
        (command.actuation_mode == WHOLE_BODY_ACTUATION_HYBRID ||
            command.actuation_mode == WHOLE_BODY_ACTUATION_VELOCITY);
}

bool UsesDamping(const whole_body_joint_command &command) {
    return command.mode == WHOLE_BODY_MODE_DAMP ||
        command.actuation_mode != WHOLE_BODY_ACTUATION_TORQUE;
}

double ComputeJointTorque(const whole_body_joint_command &command, size_t joint_index,
    double current_position, double current_velocity) {
    if (command.mode == WHOLE_BODY_MODE_DAMP)
        return -command.kd[joint_index] * current_velocity;
    switch (command.actuation_mode) {
        case WHOLE_BODY_ACTUATION_HYBRID:
            return command.torque[joint_index] +
                command.kp[joint_index] *
                    (command.position[joint_index] - current_position) +
                command.kd[joint_index] *
                    (command.velocity[joint_index] - current_velocity);
        case WHOLE_BODY_ACTUATION_POSITION:
            return command.kp[joint_index] *
                    (command.position[joint_index] - current_position) -
                command.kd[joint_index] * current_velocity;
        case WHOLE_BODY_ACTUATION_VELOCITY:
            return command.kd[joint_index] *
                (command.velocity[joint_index] - current_velocity);
        case WHOLE_BODY_ACTUATION_TORQUE:
            return command.torque[joint_index];
    }
    return 0.0;
}

bool MotorFeedbackIsFinite(const motor_state &state) {
    return std::isfinite(state.pos) && std::isfinite(state.vel) &&
        std::isfinite(state.trq) && std::isfinite(state.temp);
}

std::string BusDevice(const RuntimeConfig &config, const std::string &bus_name) {
    for (const auto &bus : config.buses) {
        if (bus.name == bus_name) return bus.device;
    }
    return {};
}

void CopyText(const std::string &source, char *destination, size_t capacity) {
    if (!destination || capacity == 0) return;
    std::snprintf(destination, capacity, "%s", source.c_str());
}

bool ImuFeedbackIsFinite(const imu_data &state) {
    for (float value : state.acc) {
        if (!std::isfinite(value)) return false;
    }
    for (float value : state.gyro) {
        if (!std::isfinite(value)) return false;
    }
    double quaternion_norm_squared = 0.0;
    for (float value : state.quat) {
        if (!std::isfinite(value)) return false;
        quaternion_norm_squared += static_cast<double>(value) * value;
    }
    return quaternion_norm_squared > 1.0e-12;
}

}  // namespace

WholeBodyCore::WholeBodyCore(RuntimeConfig config, std::unique_ptr<DeviceManager> devices)
    : config_(std::move(config)), devices_(std::move(devices)) {
    if (!devices_) throw std::runtime_error("whole-body device manager is null");
    for (size_t i = 0; i < config_.motors.size(); ++i)
        motor_indices_.emplace(config_.motors[i].name, i);
    for (size_t i = 0; i < config_.joints.size(); ++i)
        joint_indices_.emplace(config_.joints[i].name, i);
    reference_position_limits_.reserve(config_.joints.size());
    for (const auto &joint : config_.joints)
        reference_position_limits_.push_back(joint.position_limit);
    for (const auto &coupling : config_.couplings) {
        const std::array<size_t, 2> joint_indices = {
            joint_indices_.at(coupling.joints[0]),
            joint_indices_.at(coupling.joints[1]),
        };
        couplings_.push_back({
            ParallelAnkle(coupling),
            joint_indices,
            {motor_indices_.at(coupling.motors[0]), motor_indices_.at(coupling.motors[1])},
        });
        for (size_t joint_index : joint_indices) {
            reference_position_limits_[joint_index] = {
                -coupling.ankle_limit, coupling.ankle_limit};
        }
    }
    motor_states_.resize(config_.motors.size());
    motor_position_branch_offsets_.resize(config_.motors.size());
    previous_motor_raw_positions_.resize(config_.motors.size());
    motor_position_branch_initialized_.resize(config_.motors.size());
    last_motor_commands_.resize(config_.motors.size());
    last_motor_command_metrics_.resize(config_.motors.size());
    joint_position_.resize(config_.joints.size());
    joint_velocity_.resize(config_.joints.size());
    health_.state = WHOLE_BODY_HEALTH_CREATED;
}

WholeBodyCore::~WholeBodyCore() {
    if (devices_) devices_->Shutdown();
}

int WholeBodyCore::Fail(int error, const std::string &message) {
    health_.last_error = error;
    health_.state = WHOLE_BODY_HEALTH_ERROR;
    last_error_ = message;
    return error;
}

int WholeBodyCore::Init() {
    if (initialized_) return WHOLE_BODY_OK;
    if (devices_->Init() < 0)
        return Fail(WHOLE_BODY_ERR_DEVICE, "failed to initialize motor or IMU devices");
    std::fill(motor_position_branch_offsets_.begin(),
        motor_position_branch_offsets_.end(), 0.0);
    std::fill(previous_motor_raw_positions_.begin(),
        previous_motor_raw_positions_.end(), 0.0);
    std::fill(motor_position_branch_initialized_.begin(),
        motor_position_branch_initialized_.end(), false);
    initialized_ = true;
    if (!config_.read_only && config_.allow_actuation && SendIdle() != WHOLE_BODY_OK) {
        const std::string message = DescribeWriteFailure("failed to establish disabled startup state");
        devices_->Shutdown();
        initialized_ = false;
        return Fail(WHOLE_BODY_ERR_DEVICE, message);
    }
    mode_ = WHOLE_BODY_MODE_POWER_OFF;
    has_command_ = false;
    has_joint_state_ = false;
    last_idle_time_s_ = 0.0;
    health_.last_error = WHOLE_BODY_OK;
    health_.state = config_.read_only ? WHOLE_BODY_HEALTH_READ_ONLY : WHOLE_BODY_HEALTH_READY;
    last_error_.clear();
    return WHOLE_BODY_OK;
}

void WholeBodyCore::UpdateMotorPositionBranches() {
    for (size_t i = 0; i < config_.motors.size(); ++i) {
        const auto &motor = config_.motors[i];
        if (motor.position_period <= 0.0) continue;

        const double raw_position = motor_states_[i].pos;
        double new_offset = motor_position_branch_offsets_[i];
        if (!motor_position_branch_initialized_[i]) {
            new_offset = std::round(
                (raw_position - motor.zero_offset) / motor.position_period) *
                motor.position_period;
            motor_position_branch_initialized_[i] = true;
        } else {
            const double raw_delta = raw_position - previous_motor_raw_positions_[i];
            if (std::abs(raw_delta) > motor.position_period * 0.5) {
                new_offset += std::round(raw_delta / motor.position_period) *
                    motor.position_period;
            }
        }
        previous_motor_raw_positions_[i] = raw_position;

        const double offset_delta = new_offset - motor_position_branch_offsets_[i];
        if (offset_delta != 0.0 && has_motor_command_ &&
            MotorModeUsesPosition(last_motor_commands_[i].mode)) {
            last_motor_commands_[i].pos_des += static_cast<float>(offset_delta);
        }
        motor_position_branch_offsets_[i] = new_offset;
    }
}

double WholeBodyCore::NormalizedMotorPosition(size_t index) const {
    return static_cast<double>(motor_states_[index].pos) -
        motor_position_branch_offsets_[index];
}

int WholeBodyCore::Read(whole_body_state *state) {
    if (!initialized_ || !state)
        return Fail(WHOLE_BODY_ERR_STATE, "whole-body backend is not initialized");
    const int read_result =
        devices_->Read(&motor_states_, &imu_state_, &feedback_status_);
    if (read_result == DEVICE_READ_WAITING) {
        has_joint_state_ = false;
        return Fail(WHOLE_BODY_ERR_STATE,
            DescribeFeedbackProblem("waiting for initial feedback"));
    }
    if (read_result == DEVICE_READ_INCOMPATIBLE) {
        return EnterSafety(WHOLE_BODY_ERR_CONFIG,
            DescribeFeedbackProblem("motor feedback timestamp requirement not met"));
    }
    if (read_result < 0) {
        return EnterSafety(WHOLE_BODY_ERR_TIMEOUT,
            DescribeFeedbackProblem("feedback timed out"));
    }
    if (std::any_of(motor_states_.begin(), motor_states_.end(),
            [](const motor_state &motor) { return !MotorFeedbackIsFinite(motor); }) ||
        !ImuFeedbackIsFinite(imu_state_)) {
        return EnterSafety(
            WHOLE_BODY_ERR_STATE, "motor or IMU feedback contains non-finite data");
    }
    for (size_t i = 0; i < motor_states_.size(); ++i) {
        if (FatalMotorError(i) != 0)
            return EnterSafety(WHOLE_BODY_ERR_DEVICE, DescribeMotorErrors());
    }
    UpdateMotorPositionBranches();

    std::memset(state, 0, sizeof(*state));
    state->num_dof = config_.num_dof;
    state->timestamp_s = static_cast<double>(imu_state_.timestamp_us) * 1.0e-6;
    std::vector<double> motor_position(config_.motors.size());
    std::vector<double> motor_velocity(config_.motors.size());
    std::vector<double> motor_torque(config_.motors.size());
    for (size_t i = 0; i < config_.motors.size(); ++i) {
        const auto &motor = config_.motors[i];
        motor_position[i] =
            motor.polarity * (NormalizedMotorPosition(i) - motor.zero_offset);
        motor_velocity[i] = motor.polarity * motor_states_[i].vel;
        motor_torque[i] = motor.polarity * motor_states_[i].trq;
    }

    for (size_t joint_index = 0; joint_index < config_.joints.size(); ++joint_index) {
        const auto &joint = config_.joints[joint_index];
        if (joint.mapping != "direct") continue;
        const size_t motor_index = motor_indices_.at(joint.motors[0]);
        state->position[joint_index] = motor_position[motor_index];
        state->velocity[joint_index] = motor_velocity[motor_index];
        state->torque[joint_index] = motor_torque[motor_index];
        state->temperature[joint_index] = motor_states_[motor_index].temp;
        state->motor_error[joint_index] = motor_states_[motor_index].err;
    }

    for (auto &coupling : couplings_) {
        const std::array<double, 2> coupled_motor_position = {
            motor_position[coupling.motor_indices[0]],
            motor_position[coupling.motor_indices[1]],
        };
        const std::array<double, 2> coupled_motor_velocity = {
            motor_velocity[coupling.motor_indices[0]],
            motor_velocity[coupling.motor_indices[1]],
        };
        const std::array<double, 2> coupled_motor_torque = {
            motor_torque[coupling.motor_indices[0]],
            motor_torque[coupling.motor_indices[1]],
        };
        std::array<double, 2> joint_position;
        std::array<double, 2> joint_velocity;
        std::array<double, 2> joint_torque;
        if (!coupling.mapping.MotorToJoint(coupled_motor_position, &joint_position) ||
            !coupling.mapping.MotorVelocityToJoint(
                joint_position, coupled_motor_velocity, &joint_velocity) ||
            !coupling.mapping.MotorTorqueToJoint(
                joint_position, coupled_motor_torque, &joint_torque)) {
            return EnterSafety(
                WHOLE_BODY_ERR_STATE, "parallel ankle feedback is outside its solvable domain");
        }
        coupling.metrics_valid = coupling.mapping.Metrics(joint_position,
            &coupling.jacobian_condition, &coupling.torque_amplification);
        if (!coupling.metrics_valid) {
            return EnterSafety(
                WHOLE_BODY_ERR_STATE, "parallel ankle Jacobian is singular");
        }
        const auto &coupling_config = coupling.mapping.Config();
        if ((coupling_config.jacobian_condition_limit > 0.0 &&
                coupling.jacobian_condition > coupling_config.jacobian_condition_limit) ||
            (coupling_config.torque_amplification_limit > 0.0 &&
                coupling.torque_amplification > coupling_config.torque_amplification_limit)) {
            std::ostringstream message;
            message << "parallel ankle mapping exceeds safety limit: condition="
                    << coupling.jacobian_condition << ", torque_amplification="
                    << coupling.torque_amplification;
            return EnterSafety(WHOLE_BODY_ERR_STATE, message.str());
        }
        for (size_t side = 0; side < 2; ++side) {
            const size_t joint_index = coupling.joint_indices[side];
            const size_t motor_index = coupling.motor_indices[side];
            state->position[joint_index] = joint_position[side];
            state->velocity[joint_index] = joint_velocity[side];
            state->torque[joint_index] = joint_torque[side];
            state->temperature[joint_index] = motor_states_[motor_index].temp;
            state->motor_error[joint_index] = motor_states_[motor_index].err;
        }
    }

    if (has_command_) {
        std::string reason;
        if (!ValidateJointPositions(*state, &reason))
            return EnterSafety(WHOLE_BODY_ERR_STATE, reason);
    }

    for (size_t i = 0; i < 4; ++i) state->base_quat[i] = imu_state_.quat[i];
    for (size_t i = 0; i < 3; ++i) {
        state->gyro[i] = imu_state_.gyro[i];
        state->acceleration[i] = imu_state_.acc[i];
    }
    for (size_t i = 0; i < config_.joints.size(); ++i) {
        joint_position_[i] = state->position[i];
        joint_velocity_[i] = state->velocity[i];
    }
    has_joint_state_ = true;
    last_state_ = *state;
    ++health_.read_cycles;
    if (!watchdog_active_ && !fault_latched_) {
        health_.last_error = WHOLE_BODY_OK;
        health_.state = config_.read_only ? WHOLE_BODY_HEALTH_READ_ONLY : WHOLE_BODY_HEALTH_READY;
        last_error_.clear();
    }
    return WHOLE_BODY_OK;
}

bool WholeBodyCore::ValidateCommand(
    const whole_body_joint_command &command, std::string *reason) const {
    if (command.num_dof != config_.num_dof) {
        if (reason) {
            *reason = "num_dof=" + std::to_string(command.num_dof) +
                " does not match configured num_dof=" + std::to_string(config_.num_dof);
        }
        return false;
    }
    if (!IsValidMode(command.mode)) {
        if (reason) *reason = "invalid whole-body mode";
        return false;
    }
    if (!IsValidActuationMode(command.actuation_mode)) {
        if (reason) *reason = "invalid whole-body actuation mode";
        return false;
    }
    if (!command.enable || command.mode == WHOLE_BODY_MODE_POWER_OFF)
        return true;
    for (size_t i = 0; i < config_.joints.size(); ++i) {
        const auto &joint = config_.joints[i];
        const std::array<std::pair<const char *, double>, 5> fields = {{
            {"position", command.position[i]},
            {"velocity", command.velocity[i]},
            {"torque", command.torque[i]},
            {"kp", command.kp[i]},
            {"kd", command.kd[i]},
        }};
        for (const auto &field : fields) {
            if (std::isfinite(field.second)) continue;
            if (reason) *reason = joint.name + "." + field.first + " is not finite";
            return false;
        }
        const auto &reference_limit = reference_position_limits_[i];
        if (UsesPositionTarget(command) &&
            (command.position[i] < reference_limit[0] ||
                command.position[i] > reference_limit[1])) {
            if (reason) {
                std::ostringstream message;
                message << joint.name << ".position=" << command.position[i]
                        << " is outside reference range [" << reference_limit[0] << ", "
                        << reference_limit[1] << "]";
                *reason = message.str();
            }
            return false;
        }
        if (std::abs(command.velocity[i]) > joint.velocity_limit) {
            if (reason) {
                std::ostringstream message;
                message << joint.name << ".velocity=" << command.velocity[i]
                        << " exceeds limit +/-" << joint.velocity_limit;
                *reason = message.str();
            }
            return false;
        }
        if (std::abs(command.torque[i]) > joint.torque_limit) {
            if (reason) {
                std::ostringstream message;
                message << joint.name << ".torque=" << command.torque[i]
                        << " exceeds limit +/-" << joint.torque_limit;
                *reason = message.str();
            }
            return false;
        }
        if (command.kp[i] < 0.0 || command.kd[i] < 0.0) {
            if (reason) {
                *reason = joint.name +
                    (command.kp[i] < 0.0 ? ".kp is negative" : ".kd is negative");
            }
            return false;
        }
    }
    return true;
}

bool WholeBodyCore::ValidateJointPositions(
    const whole_body_state &state, std::string *reason) const {
    for (size_t i = 0; i < config_.joints.size(); ++i) {
        const auto &joint = config_.joints[i];
        const double position = state.position[i];
        if (position >= joint.position_limit[0] - kPositionLimitTolerance &&
            position <= joint.position_limit[1] + kPositionLimitTolerance) {
            continue;
        }
        if (reason) {
            std::ostringstream message;
            message << joint.name << " measured position=" << position
                    << " is outside physical range [" << joint.position_limit[0] << ", "
                    << joint.position_limit[1] << "]";
            *reason = message.str();
        }
        return false;
    }
    return true;
}

int WholeBodyCore::BuildMotorCommands(
    const whole_body_joint_command &command, std::vector<motor_cmd> *motor_commands) {
    if (!motor_commands) return WHOLE_BODY_ERR_COMMAND;
    motor_commands->assign(config_.motors.size(), motor_cmd{});
    if (!command.enable || command.mode == WHOLE_BODY_MODE_POWER_OFF) {
        for (auto &motor_command : *motor_commands) motor_command.mode = MOTOR_MODE_IDLE;
        return WHOLE_BODY_OK;
    }
    for (size_t i = 0; i < config_.motors.size(); ++i) {
        if (config_.motors[i].position_period > 0.0 &&
            !motor_position_branch_initialized_[i]) {
            return WHOLE_BODY_ERR_STATE;
        }
    }

    std::vector<double> position(config_.motors.size());
    std::vector<double> velocity(config_.motors.size());
    std::vector<double> torque(config_.motors.size());
    std::vector<double> kp(config_.motors.size());
    std::vector<double> kd(config_.motors.size());
    std::vector<motor_mode> actuation_mode(
        config_.motors.size(), ToMotorMode(command.actuation_mode));
    std::vector<bool> software_controlled(config_.motors.size());
    std::vector<bool> assigned(config_.motors.size());
    for (size_t joint_index = 0; joint_index < config_.joints.size(); ++joint_index) {
        const auto &joint = config_.joints[joint_index];
        if (joint.mapping != "direct") continue;
        const size_t motor_index = motor_indices_.at(joint.motors[0]);
        if (joint.impedance.mode == ImpedanceMode::kMotor) {
            position[motor_index] = command.position[joint_index];
            velocity[motor_index] = command.velocity[joint_index];
            torque[motor_index] = command.torque[joint_index];
            kp[motor_index] = command.kp[joint_index];
            kd[motor_index] = command.kd[joint_index];
            assigned[motor_index] = true;
            continue;
        }
        if (!has_joint_state_) return WHOLE_BODY_ERR_STATE;

        const double current_position = joint_position_[joint_index];
        const double current_velocity = joint_velocity_[joint_index];
        const double target_position = UsesPositionTarget(command)
            ? command.position[joint_index]
            : current_position;
        const double target_velocity = UsesVelocityTarget(command)
            ? command.velocity[joint_index]
            : 0.0;
        const double desired_torque = std::clamp(
            ComputeJointTorque(command, joint_index, current_position, current_velocity),
            -joint.torque_limit, joint.torque_limit);

        double motor_kp = 0.0;
        double motor_kd = 0.0;
        if (joint.impedance.mode == ImpedanceMode::kSplit) {
            if (UsesPositionTarget(command)) {
                motor_kp = std::min(command.kp[joint_index],
                    joint.impedance.motor_kp_max);
            }
            if (UsesDamping(command)) {
                motor_kd = std::min(command.kd[joint_index],
                    joint.impedance.motor_kd_max);
            }
        }
        const double hardware_torque = motor_kp * (target_position - current_position) +
            motor_kd * (target_velocity - current_velocity);
        double residual_torque = desired_torque - hardware_torque;
        if (std::abs(residual_torque) > joint.torque_limit) {
            motor_kp = 0.0;
            motor_kd = 0.0;
            residual_torque = desired_torque;
        }

        position[motor_index] = target_position;
        velocity[motor_index] = target_velocity;
        torque[motor_index] = residual_torque;
        kp[motor_index] = motor_kp;
        kd[motor_index] = motor_kd;
        actuation_mode[motor_index] =
            command.mode != WHOLE_BODY_MODE_DAMP &&
                command.actuation_mode == WHOLE_BODY_ACTUATION_TORQUE
            ? MOTOR_MODE_TRQ
            : MOTOR_MODE_HYBRID;
        software_controlled[motor_index] = true;
        assigned[motor_index] = true;
    }

    if (!couplings_.empty() && !has_joint_state_) return WHOLE_BODY_ERR_STATE;
    for (const auto &coupling : couplings_) {
        const std::array<double, 2> target_joint_velocity = {
            command.velocity[coupling.joint_indices[0]],
            command.velocity[coupling.joint_indices[1]],
        };
        const std::array<double, 2> current_joint_position = {
            joint_position_[coupling.joint_indices[0]],
            joint_position_[coupling.joint_indices[1]],
        };
        const std::array<double, 2> current_joint_velocity = {
            joint_velocity_[coupling.joint_indices[0]],
            joint_velocity_[coupling.joint_indices[1]],
        };
        const bool damp_mode = command.mode == WHOLE_BODY_MODE_DAMP;
        const std::array<double, 2> mapped_joint_velocity =
            UsesVelocityTarget(command)
            ? target_joint_velocity
            : std::array<double, 2>{0.0, 0.0};
        std::array<double, 2> joint_torque{};
        for (size_t side = 0; side < 2; ++side) {
            const size_t joint_index = coupling.joint_indices[side];
            joint_torque[side] = ComputeJointTorque(command, joint_index,
                current_joint_position[side], current_joint_velocity[side]);
            const double torque_limit = config_.joints[joint_index].torque_limit;
            joint_torque[side] = std::clamp(joint_torque[side], -torque_limit, torque_limit);
        }
        std::array<double, 2> coupled_position;
        std::array<double, 2> coupled_velocity;
        std::array<double, 2> coupled_torque;
        if (!coupling.mapping.JointToMotor(current_joint_position, &coupled_position) ||
            !coupling.mapping.JointVelocityToMotor(
                current_joint_position, mapped_joint_velocity, &coupled_velocity) ||
            !coupling.mapping.JointTorqueToMotor(
                current_joint_position, joint_torque, &coupled_torque)) {
            return WHOLE_BODY_ERR_COMMAND;
        }
        for (size_t side = 0; side < 2; ++side) {
            const size_t motor_index = coupling.motor_indices[side];
            position[motor_index] = coupled_position[side];
            velocity[motor_index] = coupled_velocity[side];
            torque[motor_index] = coupled_torque[side];
            kp[motor_index] = 0.0;
            kd[motor_index] = 0.0;
            actuation_mode[motor_index] =
                !damp_mode && command.actuation_mode == WHOLE_BODY_ACTUATION_TORQUE
                ? MOTOR_MODE_TRQ
                : MOTOR_MODE_HYBRID;
            software_controlled[motor_index] = true;
            assigned[motor_index] = true;
        }
    }

    for (size_t i = 0; i < config_.motors.size(); ++i) {
        if (!assigned[i]) return WHOLE_BODY_ERR_CONFIG;
        const auto &motor = config_.motors[i];
        auto &output = (*motor_commands)[i];
        const bool damp_mode = command.mode == WHOLE_BODY_MODE_DAMP;
        const bool hardware_damp = damp_mode && !software_controlled[i];
        output.mode = damp_mode ? MOTOR_MODE_HYBRID : actuation_mode[i];
        const double canonical_position =
            hardware_damp ? motor.zero_offset : motor.polarity * position[i] + motor.zero_offset;
        const double mapped_position = canonical_position + motor_position_branch_offsets_[i];
        const double mapped_velocity = hardware_damp ? 0.0 : motor.polarity * velocity[i];
        const double mapped_torque = hardware_damp ? 0.0 : motor.polarity * torque[i];
        const double mapped_kp = damp_mode ? 0.0 : kp[i];
        const double mapped_kd = kd[i];
        output.pos_des = static_cast<float>(mapped_position);
        output.vel_des = static_cast<float>(mapped_velocity);
        output.trq_des = static_cast<float>(mapped_torque);
        output.kp = static_cast<float>(mapped_kp);
        output.kd = static_cast<float>(mapped_kd);
    }
    return WHOLE_BODY_OK;
}

bool WholeBodyCore::ValidateMotorCommands(const std::vector<motor_cmd> &commands,
    double monotonic_time_s, std::vector<MotorCommandMetrics> *metrics,
    std::string *reason) const {
    if (!metrics || commands.size() != config_.motors.size()) {
        if (reason) *reason = "motor command count does not match configuration";
        return false;
    }
    metrics->assign(commands.size(), MotorCommandMetrics{});
    const double command_interval_s = monotonic_time_s - last_motor_command_time_s_;
    for (size_t i = 0; i < commands.size(); ++i) {
        const auto &command = commands[i];
        const auto &motor = config_.motors[i];
        const auto &limits = motor.command_limits;
        auto fail_limit = [&](const char *field, double value, double limit) {
            if (reason) {
                std::ostringstream message;
                message << motor.name << "." << field << "=" << value
                        << " exceeds limit " << limit;
                *reason = message.str();
            }
            return false;
        };
        const std::array<std::pair<const char *, double>, 5> fields = {{
            {"position", command.pos_des},
            {"velocity", command.vel_des},
            {"torque", command.trq_des},
            {"kp", command.kp},
            {"kd", command.kd},
        }};
        for (const auto &field : fields) {
            if (std::isfinite(field.second)) continue;
            if (reason) *reason = motor.name + "." + field.first + " is not finite";
            return false;
        }
        if (command.mode > MOTOR_MODE_HYBRID || command.kp < 0.0f || command.kd < 0.0f) {
            if (reason) *reason = motor.name + " has an invalid mode or negative gain";
            return false;
        }
        if (command.mode == MOTOR_MODE_IDLE) continue;
        const bool uses_position = MotorModeUsesPosition(command.mode);
        const bool uses_velocity = MotorModeUsesVelocity(command.mode);
        const bool uses_torque = MotorModeUsesTorque(command.mode);
        const bool uses_gains = MotorModeUsesGains(command.mode);
        if (uses_gains && limits.kp_max > 0.0 && command.kp > limits.kp_max)
            return fail_limit("kp", command.kp, limits.kp_max);
        if (uses_gains && limits.kd_max > 0.0 && command.kd > limits.kd_max)
            return fail_limit("kd", command.kd, limits.kd_max);

        const bool has_feedback = i < feedback_status_.motor_received.size() &&
            i < feedback_status_.motor_fresh.size() &&
            feedback_status_.motor_received[i] != 0 &&
            feedback_status_.motor_fresh[i] != 0;
        double estimated_torque = std::numeric_limits<double>::quiet_NaN();
        if (command.mode == MOTOR_MODE_TRQ) {
            estimated_torque = command.trq_des;
        } else if (command.mode == MOTOR_MODE_HYBRID) {
            estimated_torque = command.trq_des;
            if (has_feedback) {
                estimated_torque += command.kp * (command.pos_des - motor_states_[i].pos) +
                    command.kd * (command.vel_des - motor_states_[i].vel);
            } else if (limits.estimated_torque_max > 0.0) {
                if (reason) *reason = motor.name + " has no feedback for torque estimation";
                return false;
            }
        }
        (*metrics)[i].estimated_torque = estimated_torque;
        if (std::isfinite(estimated_torque) && limits.estimated_torque_max > 0.0 &&
            std::abs(estimated_torque) > limits.estimated_torque_max) {
            return fail_limit("estimated_torque", std::abs(estimated_torque),
                limits.estimated_torque_max);
        }

        if (!has_motor_command_ || last_motor_commands_[i].mode == MOTOR_MODE_IDLE ||
            last_motor_commands_[i].mode != command.mode) {
            continue;
        }
        const bool check_position_rate = uses_position && limits.position_rate_max > 0.0;
        const bool check_velocity_rate = uses_velocity && limits.velocity_rate_max > 0.0;
        const bool check_torque_rate = uses_torque && limits.torque_rate_max > 0.0;
        const bool rate_limit_enabled =
            check_position_rate || check_velocity_rate || check_torque_rate;
        if (rate_limit_enabled && command_interval_s <= 0.0) {
            if (reason) *reason = "motor command timestamp did not advance";
            return false;
        }
        if (command_interval_s <= 0.0) continue;
        auto &output = (*metrics)[i];
        if (uses_position) {
            output.position_rate =
                std::abs(command.pos_des - last_motor_commands_[i].pos_des) /
                command_interval_s;
        }
        if (uses_velocity) {
            output.velocity_rate =
                std::abs(command.vel_des - last_motor_commands_[i].vel_des) /
                command_interval_s;
        }
        if (uses_torque) {
            output.torque_rate =
                std::abs(command.trq_des - last_motor_commands_[i].trq_des) /
                command_interval_s;
        }
        if (check_position_rate && output.position_rate > limits.position_rate_max)
            return fail_limit("position_rate", output.position_rate, limits.position_rate_max);
        if (check_velocity_rate && output.velocity_rate > limits.velocity_rate_max)
            return fail_limit("velocity_rate", output.velocity_rate, limits.velocity_rate_max);
        if (check_torque_rate && output.torque_rate > limits.torque_rate_max)
            return fail_limit("torque_rate", output.torque_rate, limits.torque_rate_max);
    }
    return true;
}

int WholeBodyCore::Write(const whole_body_joint_command &command, double monotonic_time_s) {
    if (!initialized_) return Fail(WHOLE_BODY_ERR_STATE, "whole-body backend is not initialized");
    if (config_.read_only || !config_.allow_actuation) {
        health_.last_error = WHOLE_BODY_ERR_READ_ONLY;
        health_.state = WHOLE_BODY_HEALTH_READ_ONLY;
        last_error_ = "actuation is blocked by the local read-only gate";
        return WHOLE_BODY_ERR_READ_ONLY;
    }
    if (!std::isfinite(monotonic_time_s))
        return EnterSafety(WHOLE_BODY_ERR_COMMAND, "command timestamp is not finite");
    std::string validation_error;
    if (!ValidateCommand(command, &validation_error)) {
        return EnterSafety(
            WHOLE_BODY_ERR_COMMAND, "joint command failed validation: " + validation_error);
    }
    const bool resets_safety = command.mode == WHOLE_BODY_MODE_POWER_OFF;
    const bool accepts_latched_fault = resets_safety ||
        (command.mode == WHOLE_BODY_MODE_SAFETY && !command.enable);
    const bool fault_was_latched = watchdog_active_ || fault_latched_;
    if (fault_was_latched && !accepts_latched_fault) {
        return health_.last_error != WHOLE_BODY_OK ? health_.last_error : WHOLE_BODY_ERR_STATE;
    }
    if (command.enable && command.mode != WHOLE_BODY_MODE_POWER_OFF && has_joint_state_ &&
        !ValidateJointPositions(last_state_, &validation_error)) {
        return EnterSafety(
            WHOLE_BODY_ERR_STATE, "joint feedback failed validation: " + validation_error);
    }
    std::vector<motor_cmd> motor_commands;
    const int build_result = BuildMotorCommands(command, &motor_commands);
    if (build_result != WHOLE_BODY_OK)
        return EnterSafety(build_result, "failed to map joint command to motor space");
    std::vector<MotorCommandMetrics> command_metrics;
    if (!ValidateMotorCommands(
            motor_commands, monotonic_time_s, &command_metrics, &validation_error)) {
        return EnterSafety(WHOLE_BODY_ERR_COMMAND,
            "motor command failed validation: " + validation_error);
    }
    if (SendMotorCommands(motor_commands, monotonic_time_s, &command_metrics) != WHOLE_BODY_OK)
        return EnterSafety(WHOLE_BODY_ERR_DEVICE, DescribeWriteFailure("failed to write motor commands"));
    mode_ = command.mode;
    const bool idle_request = !command.enable || command.mode == WHOLE_BODY_MODE_POWER_OFF;
    if (idle_request) {
        last_idle_time_s_ = monotonic_time_s;
    }
    has_command_ = !idle_request;
    if (has_command_) last_command_time_s_ = monotonic_time_s;
    ++health_.write_cycles;
    if (command.mode == WHOLE_BODY_MODE_SAFETY && !command.enable) {
        watchdog_active_ = false;
        fault_latched_ = true;
        if (!fault_was_latched) {
            health_.last_error = WHOLE_BODY_ERR_STATE;
            health_.state = WHOLE_BODY_HEALTH_ERROR;
            last_error_ = "whole-body safety mode requested";
        }
    } else if (resets_safety) {
        watchdog_active_ = false;
        fault_latched_ = false;
        health_.last_error = WHOLE_BODY_OK;
        health_.state = WHOLE_BODY_HEALTH_READY;
        last_error_.clear();
    } else {
        health_.last_error = WHOLE_BODY_OK;
        health_.state = WHOLE_BODY_HEALTH_READY;
        last_error_.clear();
    }
    return WHOLE_BODY_OK;
}

int WholeBodyCore::SendMotorCommands(
    const std::vector<motor_cmd> &commands, double monotonic_time_s,
    const std::vector<MotorCommandMetrics> *metrics) {
    if (devices_->Write(commands) < 0) return WHOLE_BODY_ERR_DEVICE;
    last_motor_commands_ = commands;
    if (metrics && metrics->size() == commands.size())
        last_motor_command_metrics_ = *metrics;
    else
        last_motor_command_metrics_.assign(commands.size(), MotorCommandMetrics{});
    last_motor_command_time_s_ = monotonic_time_s;
    has_motor_command_ = true;
    return WHOLE_BODY_OK;
}

int WholeBodyCore::SendIdle() {
    std::vector<motor_cmd> commands(config_.motors.size());
    for (auto &command : commands) command.mode = MOTOR_MODE_IDLE;
    return SendMotorCommands(commands, MonotonicTime());
}

int WholeBodyCore::EnterSafety(int error, const std::string &message) {
    const bool can_write = initialized_ && !config_.read_only && config_.allow_actuation;
    has_command_ = false;
    has_joint_state_ = false;
    watchdog_active_ = false;
    fault_latched_ = true;
    mode_ = WHOLE_BODY_MODE_SAFETY;
    if (can_write && SendIdle() != WHOLE_BODY_OK) {
        return Fail(WHOLE_BODY_ERR_DEVICE,
            message + "; " + DescribeWriteFailure("failed to disable motors"));
    }
    return Fail(error, message);
}

int WholeBodyCore::Tick(double monotonic_time_s) {
    if (!initialized_ || !std::isfinite(monotonic_time_s))
        return Fail(WHOLE_BODY_ERR_STATE, "invalid whole-body tick");
    if (config_.read_only || !config_.allow_actuation) return WHOLE_BODY_OK;
    const bool disabled_mode = mode_ == WHOLE_BODY_MODE_POWER_OFF ||
        (mode_ == WHOLE_BODY_MODE_SAFETY && !has_command_);
    if (disabled_mode &&
        monotonic_time_s - last_idle_time_s_ >= config_.cycle_s) {
        if (SendIdle() != WHOLE_BODY_OK)
            return Fail(WHOLE_BODY_ERR_DEVICE,
                DescribeWriteFailure("failed to maintain disabled motor state"));
        last_idle_time_s_ = monotonic_time_s;
    }
    if (!has_command_ || watchdog_active_ || fault_latched_ ||
        monotonic_time_s - last_command_time_s_ <= config_.command_timeout_s) {
        return WHOLE_BODY_OK;
    }
    if (SendIdle() != WHOLE_BODY_OK)
        return Fail(WHOLE_BODY_ERR_DEVICE, DescribeWriteFailure("watchdog failed to disable motors"));
    watchdog_active_ = true;
    has_command_ = false;
    mode_ = WHOLE_BODY_MODE_SAFETY;
    last_idle_time_s_ = monotonic_time_s;
    ++health_.watchdog_events;
    health_.last_error = WHOLE_BODY_ERR_TIMEOUT;
    health_.state = WHOLE_BODY_HEALTH_WATCHDOG;
    last_error_ = "motor command watchdog expired";
    return WHOLE_BODY_ERR_TIMEOUT;
}

int WholeBodyCore::SetMode(whole_body_mode mode) {
    if (!initialized_) return Fail(WHOLE_BODY_ERR_STATE, "whole-body backend is not initialized");
    if (!IsValidMode(mode))
        return EnterSafety(WHOLE_BODY_ERR_COMMAND, "invalid whole-body control mode");
    if (config_.read_only || !config_.allow_actuation) {
        if (mode != WHOLE_BODY_MODE_POWER_OFF && mode != WHOLE_BODY_MODE_SAFETY) {
            health_.last_error = WHOLE_BODY_ERR_READ_ONLY;
            health_.state = WHOLE_BODY_HEALTH_READ_ONLY;
            last_error_ = "actuation is blocked by the local read-only gate";
            return WHOLE_BODY_ERR_READ_ONLY;
        }
        mode_ = mode;
        return WHOLE_BODY_OK;
    }
    const bool fault_was_latched = watchdog_active_ || fault_latched_;
    if (fault_was_latched && mode != WHOLE_BODY_MODE_POWER_OFF &&
        mode != WHOLE_BODY_MODE_SAFETY) {
        return health_.last_error != WHOLE_BODY_OK ? health_.last_error : WHOLE_BODY_ERR_STATE;
    }
    if ((mode == WHOLE_BODY_MODE_POWER_OFF || mode == WHOLE_BODY_MODE_SAFETY) &&
        SendIdle() != WHOLE_BODY_OK) {
        return EnterSafety(WHOLE_BODY_ERR_DEVICE,
            DescribeWriteFailure("failed to enter a safe whole-body mode"));
    }
    mode_ = mode;
    if (mode == WHOLE_BODY_MODE_POWER_OFF || mode == WHOLE_BODY_MODE_SAFETY)
        last_idle_time_s_ = 0.0;
    if (mode == WHOLE_BODY_MODE_POWER_OFF) {
        has_command_ = false;
        watchdog_active_ = false;
        fault_latched_ = false;
        health_.last_error = WHOLE_BODY_OK;
        health_.state = WHOLE_BODY_HEALTH_READY;
        last_error_.clear();
    } else if (mode == WHOLE_BODY_MODE_SAFETY) {
        has_command_ = false;
        watchdog_active_ = false;
        fault_latched_ = true;
        if (!fault_was_latched) {
            health_.last_error = WHOLE_BODY_ERR_STATE;
            health_.state = WHOLE_BODY_HEALTH_ERROR;
            last_error_ = "whole-body safety mode requested";
        }
    }
    return WHOLE_BODY_OK;
}

std::string WholeBodyCore::DescribeFeedbackProblem(const std::string &prefix) const {
    std::ostringstream message;
    message << prefix;
    bool has_detail = false;
    for (size_t i = 0; i < config_.motors.size(); ++i) {
        const bool received = i < feedback_status_.motor_received.size() &&
            feedback_status_.motor_received[i] != 0;
        const bool fresh = i < feedback_status_.motor_fresh.size() &&
            feedback_status_.motor_fresh[i] != 0;
        const auto timestamp_source = i < feedback_status_.motor_timestamp_source.size()
            ? feedback_status_.motor_timestamp_source[i]
            : WHOLE_BODY_FEEDBACK_TIMESTAMP_NONE;
        const bool incompatible_timestamp = config_.require_motor_receive_timestamps &&
            timestamp_source != WHOLE_BODY_FEEDBACK_TIMESTAMP_HARDWARE_RECEIVE;
        if (received && fresh && !incompatible_timestamp) continue;
        const auto &motor = config_.motors[i];
        const double age_s = i < feedback_status_.motor_age_s.size()
            ? feedback_status_.motor_age_s[i]
            : 0.0;
        message << (has_detail ? ", " : ": ") << motor.name << "(" << motor.bus
                << "/" << BusDevice(config_, motor.bus)
                << ",cmd=0x" << std::hex << motor.command_id << ",fb=0x"
                << motor.feedback_id << std::dec << ",age_ms=" << std::fixed
                << std::setprecision(1) << age_s * 1000.0;
        if (incompatible_timestamp) message << ",timestamp=read_completion";
        message << ")";
        has_detail = true;
    }
    if (!feedback_status_.imu_received || !feedback_status_.imu_fresh) {
        message << (has_detail ? ", " : ": ") << "imu(" << config_.imu.driver << ","
                << config_.imu.device << ",age_ms=" << std::fixed << std::setprecision(1)
                << feedback_status_.imu_age_s * 1000.0 << ")";
        has_detail = true;
    }
    return has_detail ? message.str() : prefix;
}

std::string WholeBodyCore::DescribeWriteFailure(const std::string &prefix) const {
    const std::string &detail = devices_->LastWriteError();
    return detail.empty() ? prefix : prefix + ": " + detail;
}

std::string WholeBodyCore::DescribeMotorErrors() const {
    std::ostringstream message;
    message << "motor hardware error";
    bool has_detail = false;
    for (size_t i = 0; i < motor_states_.size(); ++i) {
        const uint32_t fatal_error = FatalMotorError(i);
        if (fatal_error == 0) continue;
        const auto &motor = config_.motors[i];
        message << (has_detail ? ", " : ": ") << motor.name << "(" << motor.bus
                << "/" << BusDevice(config_, motor.bus)
                << ",fb=0x" << std::hex << motor.feedback_id << ",raw=0x"
                << motor_states_[i].err << ",fatal=0x" << fatal_error << std::dec << ")";
        has_detail = true;
    }
    return message.str();
}

uint32_t WholeBodyCore::FatalMotorError(size_t index) const {
    if (index >= motor_states_.size() || index >= config_.motors.size()) return 0;
    return IsNonFatalMotorError(index) ? 0 : motor_states_[index].err;
}

bool WholeBodyCore::IsNonFatalMotorError(size_t index) const {
    if (index >= motor_states_.size() || index >= config_.motors.size()) return false;
    const uint32_t error = motor_states_[index].err;
    const auto &codes = config_.motors[index].non_fatal_error_codes;
    return error != 0 && std::find(codes.begin(), codes.end(), error) != codes.end();
}

whole_body_health WholeBodyCore::GetHealth() const { return health_; }

void WholeBodyCore::GetDiagnosticsV2(whole_body_diagnostics_v2 *diagnostics) const {
    if (!diagnostics) return;
    *diagnostics = {};
    diagnostics->timestamp_s = last_state_.timestamp_s;
    diagnostics->feedback_window_s = feedback_status_.feedback_window_s;
    diagnostics->motor_count = static_cast<uint32_t>(config_.motors.size());
    diagnostics->joint_count = static_cast<uint32_t>(config_.joints.size());
    diagnostics->coupling_count = static_cast<uint32_t>(couplings_.size());
    diagnostics->health = health_;

    for (size_t i = 0; i < config_.motors.size(); ++i) {
        const auto &config = config_.motors[i];
        auto &output = diagnostics->motors[i];
        CopyText(config.name, output.name, sizeof(output.name));
        CopyText(config.driver, output.driver, sizeof(output.driver));
        CopyText(config.model, output.model, sizeof(output.model));
        CopyText(config.bus, output.bus, sizeof(output.bus));
        for (const auto &bus : config_.buses) {
            if (bus.name == config.bus) {
                CopyText(bus.device, output.device, sizeof(output.device));
                break;
            }
        }
        std::string joint_names;
        for (const auto &joint : config_.joints) {
            if (std::find(joint.motors.begin(), joint.motors.end(), config.name) ==
                joint.motors.end()) {
                continue;
            }
            if (!joint_names.empty()) joint_names += "+";
            joint_names += joint.name;
        }
        CopyText(joint_names, output.joint_names, sizeof(output.joint_names));
        output.command_id = config.command_id;
        output.feedback_id = config.feedback_id;
        output.feedback_received = i < feedback_status_.motor_received.size() &&
            feedback_status_.motor_received[i] != 0;
        output.feedback_fresh = i < feedback_status_.motor_fresh.size() &&
            feedback_status_.motor_fresh[i] != 0;
        if (i < feedback_status_.motor_age_s.size())
            output.feedback_age_s = feedback_status_.motor_age_s[i];
        if (i < feedback_status_.motor_timestamp_s.size())
            output.feedback_timestamp_s = feedback_status_.motor_timestamp_s[i];
        if (i < feedback_status_.motor_timestamp_source.size())
            output.feedback_timestamp_source = feedback_status_.motor_timestamp_source[i];
        if (output.feedback_received) {
            output.raw_position = motor_states_[i].pos;
            output.raw_velocity = motor_states_[i].vel;
            output.raw_torque = motor_states_[i].trq;
            output.calibrated_position =
                config.polarity * (NormalizedMotorPosition(i) - config.zero_offset);
            output.calibrated_velocity = config.polarity * motor_states_[i].vel;
            output.calibrated_torque = config.polarity * motor_states_[i].trq;
            output.temperature = motor_states_[i].temp;
            output.error = motor_states_[i].err;
            output.warning_error = IsNonFatalMotorError(i) ? motor_states_[i].err : 0;
            output.fatal_error = FatalMotorError(i);
        }
    }

    for (size_t i = 0; i < config_.joints.size(); ++i) {
        auto &output = diagnostics->joints[i];
        CopyText(config_.joints[i].name, output.name, sizeof(output.name));
        output.feedback_valid = has_joint_state_;
        if (!has_joint_state_) continue;
        output.position = last_state_.position[i];
        output.velocity = last_state_.velocity[i];
        output.torque = last_state_.torque[i];
        output.temperature = last_state_.temperature[i];
        output.motor_error = last_state_.motor_error[i];
    }

    CopyText(config_.imu.driver, diagnostics->imu.driver, sizeof(diagnostics->imu.driver));
    CopyText(config_.imu.device, diagnostics->imu.device, sizeof(diagnostics->imu.device));
    diagnostics->imu.feedback_received = feedback_status_.imu_received;
    diagnostics->imu.feedback_fresh = feedback_status_.imu_fresh;
    diagnostics->imu.feedback_age_s = feedback_status_.imu_age_s;
    diagnostics->imu.sample_timestamp_s = feedback_status_.imu_sample_timestamp_s;
    diagnostics->imu.receive_timestamp_s = feedback_status_.imu_receive_timestamp_s;
    diagnostics->imu.valid_frames = feedback_status_.imu_parser.valid_frames;
    diagnostics->imu.crc_errors = feedback_status_.imu_parser.crc_errors;
    diagnostics->imu.decode_errors = feedback_status_.imu_parser.decode_errors;
    diagnostics->imu.superseded_frames = feedback_status_.imu_parser.superseded_frames;
    diagnostics->imu.resync_discarded_bytes =
        feedback_status_.imu_parser.resync_discarded_bytes;
    diagnostics->imu.overflow_discarded_bytes =
        feedback_status_.imu_parser.overflow_discarded_bytes;
    if (feedback_status_.imu_received) {
        for (size_t i = 0; i < 4; ++i) diagnostics->imu.quaternion[i] = imu_state_.quat[i];
        for (size_t i = 0; i < 3; ++i) {
            diagnostics->imu.gyro[i] = imu_state_.gyro[i];
            diagnostics->imu.acceleration[i] = imu_state_.acc[i];
        }
    }
    for (size_t i = 0; i < couplings_.size(); ++i) {
        auto &output = diagnostics->couplings[i];
        const auto &coupling = couplings_[i];
        const std::string names = config_.joints[coupling.joint_indices[0]].name + "+" +
            config_.joints[coupling.joint_indices[1]].name;
        CopyText(names, output.joint_names, sizeof(output.joint_names));
        output.valid = coupling.metrics_valid;
        output.jacobian_condition = coupling.jacobian_condition;
        output.torque_amplification = coupling.torque_amplification;
    }
}

void WholeBodyCore::GetMotorCommandDiagnosticsV2(
    whole_body_motor_command_diagnostics_v2 *diagnostics) const {
    if (!diagnostics) return;
    *diagnostics = {};
    diagnostics->motor_count = static_cast<uint32_t>(config_.motors.size());
    if (!has_motor_command_) return;

    const double command_age_s =
        std::max(0.0, MonotonicTime() - last_motor_command_time_s_);
    for (size_t i = 0; i < last_motor_commands_.size(); ++i) {
        const auto &command = last_motor_commands_[i];
        auto &output = diagnostics->motors[i];
        output.valid = true;
        output.age_s = command_age_s;
        output.mode = command.mode;
        output.position = command.pos_des;
        output.velocity = command.vel_des;
        output.torque = command.trq_des;
        output.kp = command.kp;
        output.kd = command.kd;
        if (i < last_motor_command_metrics_.size()) {
            output.estimated_torque = last_motor_command_metrics_[i].estimated_torque;
            output.position_rate = last_motor_command_metrics_[i].position_rate;
            output.velocity_rate = last_motor_command_metrics_[i].velocity_rate;
            output.torque_rate = last_motor_command_metrics_[i].torque_rate;
        }
    }
}

void WholeBodyCore::GetDiagnostics(whole_body_diagnostics *diagnostics) const {
    if (!diagnostics) return;
    whole_body_diagnostics_v2 extended{};
    GetDiagnosticsV2(&extended);
    *diagnostics = {};
    diagnostics->timestamp_s = extended.timestamp_s;
    diagnostics->motor_count = extended.motor_count;
    diagnostics->joint_count = extended.joint_count;
    diagnostics->health = extended.health;
    for (uint32_t i = 0; i < extended.motor_count; ++i) {
        const auto &source = extended.motors[i];
        auto &output = diagnostics->motors[i];
        std::memcpy(output.name, source.name, sizeof(output.name));
        std::memcpy(output.joint_names, source.joint_names, sizeof(output.joint_names));
        std::memcpy(output.driver, source.driver, sizeof(output.driver));
        std::memcpy(output.model, source.model, sizeof(output.model));
        std::memcpy(output.bus, source.bus, sizeof(output.bus));
        std::memcpy(output.device, source.device, sizeof(output.device));
        output.command_id = source.command_id;
        output.feedback_id = source.feedback_id;
        output.feedback_received = source.feedback_received;
        output.feedback_fresh = source.feedback_fresh;
        output.feedback_age_s = source.feedback_age_s;
        output.raw_position = source.raw_position;
        output.raw_velocity = source.raw_velocity;
        output.raw_torque = source.raw_torque;
        output.calibrated_position = source.calibrated_position;
        output.calibrated_velocity = source.calibrated_velocity;
        output.calibrated_torque = source.calibrated_torque;
        output.temperature = source.temperature;
        output.error = source.error;
    }
    for (uint32_t i = 0; i < extended.joint_count; ++i)
        diagnostics->joints[i] = extended.joints[i];
    std::memcpy(diagnostics->imu.driver, extended.imu.driver,
        sizeof(diagnostics->imu.driver));
    std::memcpy(diagnostics->imu.device, extended.imu.device,
        sizeof(diagnostics->imu.device));
    diagnostics->imu.feedback_received = extended.imu.feedback_received;
    diagnostics->imu.feedback_fresh = extended.imu.feedback_fresh;
    diagnostics->imu.feedback_age_s = extended.imu.feedback_age_s;
    std::copy(std::begin(extended.imu.quaternion), std::end(extended.imu.quaternion),
        diagnostics->imu.quaternion);
    std::copy(std::begin(extended.imu.gyro), std::end(extended.imu.gyro),
        diagnostics->imu.gyro);
    std::copy(std::begin(extended.imu.acceleration), std::end(extended.imu.acceleration),
        diagnostics->imu.acceleration);
}

void WholeBodyCore::GetMotorCommandDiagnostics(
    whole_body_motor_command_diagnostics *diagnostics) const {
    if (!diagnostics) return;
    whole_body_motor_command_diagnostics_v2 extended{};
    GetMotorCommandDiagnosticsV2(&extended);
    *diagnostics = {};
    diagnostics->motor_count = extended.motor_count;
    for (uint32_t i = 0; i < extended.motor_count; ++i) {
        const auto &source = extended.motors[i];
        auto &output = diagnostics->motors[i];
        output.valid = source.valid;
        output.age_s = source.age_s;
        output.mode = source.mode;
        output.position = source.position;
        output.velocity = source.velocity;
        output.torque = source.torque;
        output.kp = source.kp;
        output.kd = source.kd;
    }
}

double WholeBodyCore::CycleSeconds() const { return config_.cycle_s; }

const std::string &WholeBodyCore::LastError() const { return last_error_; }

}  // namespace whole_body

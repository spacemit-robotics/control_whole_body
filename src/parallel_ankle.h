/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file parallel_ankle.h
 * @brief Generic two-link parallel ankle mapping
 */

#ifndef PARALLEL_ANKLE_H
#define PARALLEL_ANKLE_H

#include <array>

#include "whole_body_config.h"

namespace whole_body {

class ParallelAnkle {
    public:
    explicit ParallelAnkle(ParallelAnkleConfig config);

    bool JointToMotor(const std::array<double, 2> &joint, std::array<double, 2> *motor,
        std::array<double, 4> *jacobian = nullptr) const;
    bool MotorToJoint(const std::array<double, 2> &motor, std::array<double, 2> *joint,
        std::array<double, 4> *jacobian = nullptr) const;
    bool JointVelocityToMotor(const std::array<double, 2> &joint,
        const std::array<double, 2> &joint_velocity, std::array<double, 2> *motor_velocity) const;
    bool MotorVelocityToJoint(const std::array<double, 2> &joint,
        const std::array<double, 2> &motor_velocity, std::array<double, 2> *joint_velocity) const;
    bool JointTorqueToMotor(const std::array<double, 2> &joint,
        const std::array<double, 2> &joint_torque, std::array<double, 2> *motor_torque) const;
    bool MotorTorqueToJoint(const std::array<double, 2> &joint,
        const std::array<double, 2> &motor_torque, std::array<double, 2> *joint_torque) const;

    private:
    bool Compute(const std::array<double, 2> &joint, std::array<double, 2> *motor,
        std::array<double, 4> *jacobian) const;

    ParallelAnkleConfig config_;
    std::array<double, 2> rod_lengths_{};
    std::array<double, 2> crank_lengths_{};
};

}  // namespace whole_body

#endif  // PARALLEL_ANKLE_H

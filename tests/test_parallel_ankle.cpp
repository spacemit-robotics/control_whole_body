/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_parallel_ankle.cpp
 * @brief Offline tests for the parallel ankle mapping
 */

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

#include "parallel_ankle.h"

namespace {

whole_body::ParallelAnkleConfig MakeConfig() {
    whole_body::ParallelAnkleConfig config;
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
    config.motor_bias = {0.0, 0.0};
    config.motor_limit = 1.5;
    config.ankle_limit = 1.5;
    config.motor_difference_limit = 1.3;
    config.max_iterations = 50;
    config.squared_tolerance = 1.0e-10;
    return config;
}

}  // namespace

int main() {
    const whole_body::ParallelAnkle mapping(MakeConfig());
    std::array<double, 2> motor;
    std::array<double, 4> jacobian;
    assert(mapping.JointToMotor({0.0, 0.0}, &motor, &jacobian));
    assert(std::abs(motor[0]) < 1.0e-8);
    assert(std::abs(motor[1]) < 1.0e-8);
    assert(std::abs(jacobian[0] - 1.0) < 1.0e-6);
    assert(std::abs(jacobian[1] - 0.541666667) < 1.0e-6);
    assert(std::abs(jacobian[2] - 1.0) < 1.0e-6);
    assert(std::abs(jacobian[3] + 0.541666667) < 1.0e-6);

    const std::array<double, 2> joint = {0.1, -0.1};
    assert(mapping.JointToMotor(joint, &motor));
    assert(std::abs(motor[0] - 0.045948864) < 1.0e-7);
    assert(std::abs(motor[1] - 0.154098040) < 1.0e-7);
    std::array<double, 2> recovered_joint;
    assert(mapping.MotorToJoint(motor, &recovered_joint));
    assert(std::abs(recovered_joint[0] - joint[0]) < 1.0e-4);
    assert(std::abs(recovered_joint[1] - joint[1]) < 1.0e-4);

    const std::array<double, 2> joint_velocity = {0.2, -0.3};
    const std::array<double, 2> joint_torque = {4.0, -2.0};
    std::array<double, 2> motor_velocity;
    std::array<double, 2> motor_torque;
    assert(mapping.JointVelocityToMotor(joint, joint_velocity, &motor_velocity));
    assert(mapping.JointTorqueToMotor(joint, joint_torque, &motor_torque));
    const double joint_power =
        joint_velocity[0] * joint_torque[0] + joint_velocity[1] * joint_torque[1];
    const double motor_power =
        motor_velocity[0] * motor_torque[0] + motor_velocity[1] * motor_torque[1];
    assert(std::abs(joint_power - motor_power) < 1.0e-8);
    std::cout << "Parallel ankle tests passed\n";
    return 0;
}

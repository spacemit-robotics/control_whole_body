/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file parallel_ankle.cpp
 * @brief Generic two-link parallel ankle mapping implementation
 */

#include "parallel_ankle.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace whole_body {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumDenominator = 1.0e-10;

double Norm(const Vec3 &value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 Subtract(const Vec3 &lhs, const Vec3 &rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

bool Invert2x2(const std::array<double, 4> &matrix, std::array<double, 4> *inverse) {
    const double determinant = matrix[0] * matrix[3] - matrix[1] * matrix[2];
    if (!inverse || std::abs(determinant) < kMinimumDenominator) return false;
    *inverse = {matrix[3] / determinant, -matrix[1] / determinant, -matrix[2] / determinant,
        matrix[0] / determinant};
    return true;
}

std::array<double, 2> Multiply(
    const std::array<double, 4> &matrix, const std::array<double, 2> &vector) {
    return {matrix[0] * vector[0] + matrix[1] * vector[1],
        matrix[2] * vector[0] + matrix[3] * vector[1]};
}

}  // namespace

ParallelAnkle::ParallelAnkle(ParallelAnkleConfig config) : config_(std::move(config)) {
    for (size_t side = 0; side < 2; ++side) {
        rod_lengths_[side] =
            Norm(Subtract(config_.ball_positions[side], config_.ankle_pivots[side]));
        crank_lengths_[side] =
            Norm(Subtract(config_.ball_positions[side], config_.motor_positions[side]));
    }
}

bool ParallelAnkle::Compute(const std::array<double, 2> &joint, std::array<double, 2> *motor,
    std::array<double, 4> *jacobian) const {
    if (!motor || !jacobian || !std::isfinite(joint[0]) || !std::isfinite(joint[1]) ||
        std::abs(joint[0]) > config_.ankle_limit || std::abs(joint[1]) > config_.ankle_limit) {
        return false;
    }

    const double pitch_sin = std::sin(joint[0]);
    const double pitch_cos = std::cos(joint[0]);
    const double roll_sin = std::sin(joint[1]);
    const double roll_cos = std::cos(joint[1]);
    for (size_t side = 0; side < 2; ++side) {
        const Vec3 &pivot = config_.ankle_pivots[side];
        const Vec3 &motor_position = config_.motor_positions[side];
        const double rotated_x = pitch_cos * pivot.x + pitch_sin * pivot.y * roll_sin;
        const double rotated_y = pivot.y * roll_cos;
        const double rotated_z = -pitch_sin * pivot.x + pitch_cos * pivot.y * roll_sin;
        const Vec3 displacement = {motor_position.x - rotated_x, motor_position.y - rotated_y,
            motor_position.z - rotated_z};
        const double rho_squared =
            displacement.x * displacement.x + displacement.z * displacement.z;
        const double rho = std::sqrt(rho_squared);
        const double crank = crank_lengths_[side];
        const double rod = rod_lengths_[side];
        if (rho < kMinimumDenominator || crank < kMinimumDenominator) return false;
        const double numerator =
            rho_squared + crank * crank - rod * rod + displacement.y * displacement.y;
        const double cosine = numerator / (2.0 * crank * rho);
        if (cosine < -1.0 - 1.0e-8 || cosine > 1.0 + 1.0e-8) return false;
        const double clipped_cosine = std::clamp(cosine, -1.0, 1.0);
        const double crank_angle =
            std::atan2(displacement.x, displacement.z) + std::acos(clipped_cosine);
        (*motor)[side] = crank_angle - kPi * 0.5 - config_.motor_bias[side];

        const std::array<Vec3, 2> rotated_derivative = {
            Vec3{rotated_z, 0.0, -rotated_x},
            Vec3{pitch_sin * pivot.y * roll_cos, -pivot.y * roll_sin,
                pitch_cos * pivot.y * roll_cos},
        };
        for (size_t axis = 0; axis < 2; ++axis) {
            const Vec3 displacement_derivative = {
                -rotated_derivative[axis].x,
                -rotated_derivative[axis].y,
                -rotated_derivative[axis].z,
            };
            const double rho_numerator =
                displacement.x * displacement_derivative.x +
                displacement.z * displacement_derivative.z;
            const double rho_derivative = rho_numerator / rho;
            const double numerator_derivative =
                2.0 * rho * rho_derivative + 2.0 * displacement.y * displacement_derivative.y;
            const double cosine_derivative =
                numerator_derivative / (2.0 * crank * rho) -
                numerator * rho_derivative / (2.0 * crank * rho_squared);
            const double acos_denominator =
                std::sqrt(std::max(0.0, 1.0 - clipped_cosine * clipped_cosine));
            if (acos_denominator < kMinimumDenominator) return false;
            const double atan_numerator =
                displacement.z * displacement_derivative.x -
                displacement.x * displacement_derivative.z;
            const double atan_derivative = atan_numerator / rho_squared;
            (*jacobian)[side * 2 + axis] = atan_derivative - cosine_derivative / acos_denominator;
        }
    }

    const bool motors_in_range =
        std::abs((*motor)[0]) <= config_.motor_limit &&
        std::abs((*motor)[1]) <= config_.motor_limit;
    const bool difference_in_range =
        std::abs((*motor)[0] - (*motor)[1]) <= config_.motor_difference_limit;
    return motors_in_range && difference_in_range;
}

bool ParallelAnkle::JointToMotor(const std::array<double, 2> &joint, std::array<double, 2> *motor,
    std::array<double, 4> *jacobian) const {
    std::array<double, 4> local_jacobian;
    return Compute(joint, motor, jacobian ? jacobian : &local_jacobian);
}

bool ParallelAnkle::MotorToJoint(const std::array<double, 2> &motor, std::array<double, 2> *joint,
    std::array<double, 4> *jacobian) const {
    if (!joint || !std::isfinite(motor[0]) || !std::isfinite(motor[1]) ||
        std::abs(motor[0]) > config_.motor_limit || std::abs(motor[1]) > config_.motor_limit ||
        std::abs(motor[0] - motor[1]) > config_.motor_difference_limit) {
        return false;
    }

    std::array<double, 2> zero_motor;
    std::array<double, 4> zero_jacobian;
    const std::array<double, 2> zero_joint = {0.0, 0.0};
    if (!Compute(zero_joint, &zero_motor, &zero_jacobian)) return false;
    std::array<double, 4> inverse;
    if (!Invert2x2(zero_jacobian, &inverse)) return false;
    *joint = Multiply(inverse, {motor[0] - zero_motor[0], motor[1] - zero_motor[1]});

    std::array<double, 2> estimate;
    std::array<double, 4> current_jacobian;
    for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
        if (!Compute(*joint, &estimate, &current_jacobian) ||
            !Invert2x2(current_jacobian, &inverse))
            return false;
        const std::array<double, 2> error = {estimate[0] - motor[0], estimate[1] - motor[1]};
        if (error[0] * error[0] + error[1] * error[1] <= config_.squared_tolerance) {
            if (jacobian) *jacobian = current_jacobian;
            return true;
        }
        const auto correction = Multiply(inverse, error);
        (*joint)[0] -= correction[0];
        (*joint)[1] -= correction[1];
    }
    return false;
}

bool ParallelAnkle::JointVelocityToMotor(const std::array<double, 2> &joint,
    const std::array<double, 2> &joint_velocity, std::array<double, 2> *motor_velocity) const {
    std::array<double, 2> motor;
    std::array<double, 4> jacobian;
    if (!motor_velocity || !Compute(joint, &motor, &jacobian)) return false;
    *motor_velocity = Multiply(jacobian, joint_velocity);
    return true;
}

bool ParallelAnkle::MotorVelocityToJoint(const std::array<double, 2> &joint,
    const std::array<double, 2> &motor_velocity, std::array<double, 2> *joint_velocity) const {
    std::array<double, 2> motor;
    std::array<double, 4> jacobian;
    std::array<double, 4> inverse;
    if (!joint_velocity || !Compute(joint, &motor, &jacobian) || !Invert2x2(jacobian, &inverse))
        return false;
    *joint_velocity = Multiply(inverse, motor_velocity);
    return true;
}

bool ParallelAnkle::JointTorqueToMotor(const std::array<double, 2> &joint,
    const std::array<double, 2> &joint_torque, std::array<double, 2> *motor_torque) const {
    std::array<double, 2> motor;
    std::array<double, 4> jacobian;
    std::array<double, 4> transpose = {};
    std::array<double, 4> inverse;
    if (!motor_torque || !Compute(joint, &motor, &jacobian)) return false;
    transpose = {jacobian[0], jacobian[2], jacobian[1], jacobian[3]};
    if (!Invert2x2(transpose, &inverse)) return false;
    *motor_torque = Multiply(inverse, joint_torque);
    return true;
}

bool ParallelAnkle::MotorTorqueToJoint(const std::array<double, 2> &joint,
    const std::array<double, 2> &motor_torque, std::array<double, 2> *joint_torque) const {
    std::array<double, 2> motor;
    std::array<double, 4> jacobian;
    if (!joint_torque || !Compute(joint, &motor, &jacobian)) return false;
    *joint_torque = {jacobian[0] * motor_torque[0] + jacobian[2] * motor_torque[1],
        jacobian[1] * motor_torque[0] + jacobian[3] * motor_torque[1]};
    return true;
}

}  // namespace whole_body

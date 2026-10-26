/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file coupled_mapping.cpp
 * @brief Coupling Jacobian transforms preserving mechanical power.
 */
#include "coupled_mapping.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace whole_body {
namespace {

bool MatrixMetrics(const std::array<double, 4> &matrix, double *condition, double *amplification) {
    if (!condition || !amplification) return false;
    const double trace = matrix[0] * matrix[0] + matrix[1] * matrix[1] +
        matrix[2] * matrix[2] + matrix[3] * matrix[3];
    const double det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
    const double high = (trace + std::sqrt(std::max(0.0, trace * trace - 4 * det * det))) * 0.5;
    const double low = high > 0.0 ? det * det / high : 0.0;
    if (!std::isfinite(high) || !std::isfinite(low) || low <= 1.0e-10) return false;
    *condition = std::sqrt(high / low);
    *amplification = 1.0 / std::sqrt(low);
    return std::isfinite(*condition) && std::isfinite(*amplification);
}

}  // namespace

CoupledMapping::CoupledMapping(const CouplingConfig &config) : config_(config) {
    if (config.type == "table_2d") table_ = std::make_unique<TableMapping>(config);
    else if (config.type == "parallel_ankle") geometry_ = std::make_unique<ParallelAnkle>(config);
    else throw std::runtime_error("unsupported coupling type");
}

bool CoupledMapping::JointToMotor(const std::array<double, 2> &joint,
    std::array<double, 2> *motor, std::array<double, 4> *jacobian) const {
    return table_ ? table_->JointToMotor(joint, motor, jacobian) :
        geometry_->JointToMotor(joint, motor, jacobian);
}

bool CoupledMapping::MotorToJoint(const std::array<double, 2> &motor,
    std::array<double, 2> *joint, std::array<double, 4> *jacobian) const {
    return table_ ? table_->MotorToJoint(motor, joint, jacobian) :
        geometry_->MotorToJoint(motor, joint, jacobian);
}

bool CoupledMapping::Transform(const std::array<double, 2> &joint,
    const std::array<double, 2> &value, bool transpose, bool inverse,
    std::array<double, 2> *result) const {
    std::array<double, 2> motor;
    std::array<double, 4> matrix;
    if (!result || !std::isfinite(value[0]) || !std::isfinite(value[1]) ||
        !JointToMotor(joint, &motor, &matrix)) return false;
    if (transpose) std::swap(matrix[1], matrix[2]);
    if (inverse) {
        const double det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
        if (!std::isfinite(det) || std::abs(det) < 1.0e-10) return false;
        matrix = {matrix[3] / det, -matrix[1] / det, -matrix[2] / det, matrix[0] / det};
    }
    *result = {matrix[0] * value[0] + matrix[1] * value[1],
        matrix[2] * value[0] + matrix[3] * value[1]};
    return std::isfinite((*result)[0]) && std::isfinite((*result)[1]);
}

bool CoupledMapping::JointVelocityToMotor(const std::array<double, 2> &joint,
    const std::array<double, 2> &velocity, std::array<double, 2> *result) const {
    if (geometry_) return geometry_->JointVelocityToMotor(joint, velocity, result);
    return Transform(joint, velocity, false, false, result);
}

bool CoupledMapping::MotorVelocityToJoint(const std::array<double, 2> &joint,
    const std::array<double, 2> &velocity, std::array<double, 2> *result) const {
    if (geometry_) return geometry_->MotorVelocityToJoint(joint, velocity, result);
    return Transform(joint, velocity, false, true, result);
}

bool CoupledMapping::JointTorqueToMotor(const std::array<double, 2> &joint,
    const std::array<double, 2> &torque, std::array<double, 2> *result) const {
    if (geometry_) return geometry_->JointTorqueToMotor(joint, torque, result);
    return Transform(joint, torque, true, true, result);
}

bool CoupledMapping::MotorTorqueToJoint(const std::array<double, 2> &joint,
    const std::array<double, 2> &torque, std::array<double, 2> *result) const {
    if (geometry_) return geometry_->MotorTorqueToJoint(joint, torque, result);
    return Transform(joint, torque, true, false, result);
}

bool CoupledMapping::Metrics(const std::array<double, 2> &joint,
    double *condition, double *amplification) const {
    if (geometry_) return geometry_->Metrics(joint, condition, amplification);
    std::array<double, 2> motor;
    std::array<double, 4> matrix;
    if (!condition || !amplification || !JointToMotor(joint, &motor, &matrix)) return false;
    return MatrixMetrics(matrix, condition, amplification);
}

bool CoupledMapping::JointGainToMotor(const std::array<double, 2> &joint,
    const std::array<double, 2> &gain, std::array<double, 2> *result, bool lower_bound) const {
    if (!result || !std::isfinite(gain[0]) || !std::isfinite(gain[1]) ||
        gain[0] < 0.0 || gain[1] < 0.0) return false;
    std::array<double, 2> motor;
    std::array<double, 4> matrix;
    if (!JointToMotor(joint, &motor, &matrix)) return false;
    const double det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
    if (!std::isfinite(det) || std::abs(det) < 1.0e-10) return false;
    const std::array<double, 4> inverse{
        matrix[3] / det, -matrix[1] / det, -matrix[2] / det, matrix[0] / det};
    for (size_t i = 0; i < 2; ++i) {
        (*result)[i] = gain[0] * inverse[i] * inverse[i] +
            gain[1] * inverse[2 + i] * inverse[2 + i];
        if (!std::isfinite((*result)[i])) return false;
    }
    if (lower_bound) {
        const double cross = gain[0] * inverse[0] * inverse[1] +
            gain[1] * inverse[2] * inverse[3];
        const double trace = (*result)[0] + (*result)[1];
        const double spread = std::hypot((*result)[0] - (*result)[1], 2.0 * cross);
        const double minimum = std::max(0.0, (trace - spread) * 0.5);
        *result = {minimum, minimum};
    }
    return true;
}

bool CoupledMapping::CheckJointTarget(const std::array<double, 2> &joint,
    std::array<double, 2> *motor_target) const {
    std::array<double, 2> motor;
    std::array<double, 4> matrix;
    const bool reachable = table_ ? table_->JointToMotor(joint, &motor, &matrix, false) :
        geometry_->JointToMotor(joint, &motor, &matrix);
    double condition, amplification;
    const bool valid = reachable && MatrixMetrics(matrix, &condition, &amplification) &&
        (config_.jacobian_condition_limit <= 0.0 || condition <= config_.jacobian_condition_limit) &&
        (config_.torque_amplification_limit <= 0.0 ||
            amplification <= config_.torque_amplification_limit);
    if (valid && motor_target) *motor_target = motor;
    return valid;
}

}  // namespace whole_body

/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file imu_chain.cpp
 * @brief Generic articulated IMU-link orientation and angular-velocity transform.
 */
#include "core/imu_chain.h"

#include <array>
#include <cmath>

namespace whole_body {
namespace {

using Quaternion = std::array<double, 4>;
using Vector = std::array<double, 3>;

Quaternion Multiply(const Quaternion &a, const Quaternion &b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

Quaternion Conjugate(const Quaternion &q) { return {q[0], -q[1], -q[2], -q[3]}; }

Vector Rotate(const Quaternion &q, const Vector &v) {
    auto rotated = Multiply(Multiply(q, {0.0, v[0], v[1], v[2]}), Conjugate(q));
    return {rotated[1], rotated[2], rotated[3]};
}

}  // namespace

bool TransformImuToBase(const std::vector<ImuChainJoint> &chain,
    const std::vector<double> &position, const std::vector<double> &velocity,
    const imu_data &sensor, imu_data *base) {
    if (!base || chain.size() != position.size() || chain.size() != velocity.size()) return false;
    Quaternion base_from_sensor{1.0, 0.0, 0.0, 0.0};
    Vector relative_gyro{};
    for (size_t i = 0; i < chain.size(); ++i) {
        if (!std::isfinite(position[i]) || !std::isfinite(velocity[i])) return false;
        const auto &axis = chain[i].axis;
        const double length = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
        if (!std::isfinite(length) || std::abs(length - 1.0) > 1.0e-6) return false;
        const auto base_axis = Rotate(base_from_sensor, {axis.x, axis.y, axis.z});
        for (size_t j = 0; j < 3; ++j) relative_gyro[j] += base_axis[j] * velocity[i];
        const double half_sin = std::sin(position[i] * 0.5);
        base_from_sensor = Multiply(base_from_sensor,
            {std::cos(position[i] * 0.5), axis.x * half_sin, axis.y * half_sin, axis.z * half_sin});
    }
    Quaternion world_from_sensor;
    double norm_squared = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        world_from_sensor[i] = sensor.quat[i];
        norm_squared += world_from_sensor[i] * world_from_sensor[i];
    }
    if (!std::isfinite(norm_squared) || norm_squared < 1.0e-12) return false;
    for (double &value : world_from_sensor) value /= std::sqrt(norm_squared);
    const auto world_from_base = Multiply(world_from_sensor, Conjugate(base_from_sensor));
    const auto gyro = Rotate(base_from_sensor, {sensor.gyro[0], sensor.gyro[1], sensor.gyro[2]});
    const auto acc = Rotate(base_from_sensor, {sensor.acc[0], sensor.acc[1], sensor.acc[2]});
    *base = sensor;
    for (size_t i = 0; i < 4; ++i) base->quat[i] = static_cast<float>(world_from_base[i]);
    for (size_t i = 0; i < 3; ++i) {
        base->gyro[i] = static_cast<float>(gyro[i] - relative_gyro[i]);
        base->acc[i] = static_cast<float>(acc[i]);
        if (!std::isfinite(base->gyro[i]) || !std::isfinite(base->acc[i])) return false;
    }
    return true;
}

}  // namespace whole_body

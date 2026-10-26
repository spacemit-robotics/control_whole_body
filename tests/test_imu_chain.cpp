/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_imu_chain.cpp
 * @brief Offline articulated IMU pose/gyro tests independent of machine names.
 */
#include "imu_chain.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

int main(void) {
    const double pi = 3.14159265358979323846;
    std::vector<whole_body::ImuChainJoint> chain{{"joint", {0, 0, 1}}};
    imu_data sensor{};
    imu_data base;
    sensor.quat[0] = static_cast<float>(std::cos(pi / 4));
    sensor.quat[3] = static_cast<float>(std::sin(pi / 4));
    sensor.gyro[2] = 1.5f;
    sensor.acc[0] = 1.0f;
    assert(whole_body::TransformImuToBase(chain, {pi / 2}, {1.5}, sensor, &base));
    assert(std::abs(base.quat[0] - 1.0f) < 1.0e-6 && std::abs(base.quat[3]) < 1.0e-6);
    assert(std::abs(base.gyro[2]) < 1.0e-6);
    assert(std::abs(base.acc[1] - 1.0f) < 1.0e-6);
    // Noncommuting parent axes: Rz(90) then Rx(90). Relative rates in the base
    // are (0, second_rate, first_rate), not a sum of unrotated local axes.
    chain.push_back({"second", {1, 0, 0}});
    sensor.quat[0] = sensor.quat[1] = sensor.quat[2] = sensor.quat[3] = 0.5f;
    sensor.gyro[0] = 2.0f;
    sensor.gyro[1] = 1.5f;
    sensor.gyro[2] = 0.0f;
    assert(whole_body::TransformImuToBase(chain, {pi / 2, pi / 2}, {1.5, 2}, sensor, &base));
    for (size_t i = 1; i < 4; ++i) assert(std::abs(base.quat[i]) < 1.0e-6);
    for (size_t i = 0; i < 3; ++i) assert(std::abs(base.gyro[i]) < 1.0e-6);
    assert(!whole_body::TransformImuToBase(chain, {0}, {0}, sensor, &base));
    sensor.quat[0] = std::numeric_limits<float>::quiet_NaN();
    assert(!whole_body::TransformImuToBase(chain, {0, 0}, {0, 0}, sensor, &base));
    return 0;
}

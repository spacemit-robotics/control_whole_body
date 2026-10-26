/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file imu_chain.h
 * @brief Rotate sensor-link feedback to the base and subtract relative joint motion.
 */
#ifndef IMU_CHAIN_H
#define IMU_CHAIN_H

#include <vector>

#include "imu.h"
#include "whole_body_config.h"

namespace whole_body {

// Axes are in each joint's parent frame; joint order is base -> sensor link.
// Acceleration remains measured at the IMU position (no lever-arm compensation).
bool TransformImuToBase(const std::vector<ImuChainJoint> &chain,
    const std::vector<double> &position, const std::vector<double> &velocity,
    const imu_data &sensor, imu_data *base);

}  // namespace whole_body
#endif  // IMU_CHAIN_H

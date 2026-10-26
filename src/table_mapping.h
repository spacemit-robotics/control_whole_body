/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file table_mapping.h
 * @brief Strict two-input/two-output grid mapping with a bounded inverse solver.
 */
#ifndef TABLE_MAPPING_H
#define TABLE_MAPPING_H

#include <array>
#include <vector>

#include "whole_body_config.h"

namespace whole_body {

class TableMapping {
    public:
    explicit TableMapping(const CouplingConfig &config);
    bool JointToMotor(const std::array<double, 2> &joint, std::array<double, 2> *motor,
        std::array<double, 4> *jacobian = nullptr, bool allow_global_search = true) const;
    bool MotorToJoint(const std::array<double, 2> &motor, std::array<double, 2> *joint,
        std::array<double, 4> *jacobian = nullptr) const;

    private:
    bool Lookup(const std::array<double, 2> &joint, std::array<double, 2> *motor,
        std::array<double, 4> *jacobian) const;
    bool InverseLookup(const std::array<double, 2> &value, std::array<double, 2> *input,
        std::array<double, 4> *jacobian, bool allow_global_search = true) const;
    bool Solve(const std::array<double, 2> &motor, std::array<double, 2> seed,
        std::array<double, 2> *joint, std::array<double, 4> *jacobian) const;
    CouplingConfig config_;
    size_t nx_ = 0;
    size_t ny_ = 0;
    double x0_ = 0.0;
    double y0_ = 0.0;
    double step_x_ = 0.0;
    double step_y_ = 0.0;
    std::vector<std::array<double, 2>> values_;
    // Per cell: 0 = unsupported, 1 = lower-left/upper-right, 2 = other diagonal.
    std::vector<uint8_t> diagonals_;
    mutable std::array<double, 2> last_joint_{};
    mutable bool has_last_joint_ = false;
    bool motor_to_joint_ = false;
};

}  // namespace whole_body
#endif  // TABLE_MAPPING_H

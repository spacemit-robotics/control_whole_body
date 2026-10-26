/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file coupled_mapping.h
 * @brief Common velocity/torque semantics for geometric and tabulated couplings.
 */
#ifndef COUPLED_MAPPING_H
#define COUPLED_MAPPING_H

#include <array>
#include <memory>

#include "parallel_ankle.h"
#include "table_mapping.h"

namespace whole_body {

class CoupledMapping {
    public:
    explicit CoupledMapping(const CouplingConfig &config);
    bool JointToMotor(const std::array<double, 2> &joint, std::array<double, 2> *motor,
        std::array<double, 4> *jacobian = nullptr) const;
    bool MotorToJoint(const std::array<double, 2> &motor, std::array<double, 2> *joint,
        std::array<double, 4> *jacobian = nullptr) const;
    bool JointVelocityToMotor(const std::array<double, 2> &joint,
        const std::array<double, 2> &velocity, std::array<double, 2> *result) const;
    bool MotorVelocityToJoint(const std::array<double, 2> &joint,
        const std::array<double, 2> &velocity, std::array<double, 2> *result) const;
    bool JointTorqueToMotor(const std::array<double, 2> &joint,
        const std::array<double, 2> &torque, std::array<double, 2> *result) const;
    bool MotorTorqueToJoint(const std::array<double, 2> &joint,
        const std::array<double, 2> &torque, std::array<double, 2> *result) const;
    // Diagonal approximation of A^T diag(gain) A, where A = d(joint)/d(motor).
    // lower_bound selects an isotropic positive-semidefinite part for split control.
    bool JointGainToMotor(const std::array<double, 2> &joint,
        const std::array<double, 2> &gain, std::array<double, 2> *result,
        bool lower_bound = false) const;
    bool Metrics(const std::array<double, 2> &joint, double *condition, double *amplification) const;
    // For table targets: bounded local solve, no whole-grid recovery or feedback-seed mutation.
    bool CheckJointTarget(const std::array<double, 2> &joint,
        std::array<double, 2> *motor_target = nullptr) const;
    const CouplingConfig &Config() const { return config_; }

    private:
    bool Transform(const std::array<double, 2> &joint, const std::array<double, 2> &value,
        bool transpose, bool inverse, std::array<double, 2> *result) const;
    CouplingConfig config_;
    std::unique_ptr<ParallelAnkle> geometry_;
    std::unique_ptr<TableMapping> table_;
};

}  // namespace whole_body
#endif  // COUPLED_MAPPING_H

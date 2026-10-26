/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_table_mapping.cpp
 * @brief Offline table inverse, finite-difference, power and domain rejection tests.
 */
#include "coupled_mapping.h"
#include "whole_body_core.h"

#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <unistd.h>

namespace {

void MakeTable(const std::string &path, bool hole, bool singular = false,
    double y_scale = 1.0) {
    std::ofstream stream(path);
    stream << std::setprecision(17);
    for (size_t ix = 0; ix < 3; ++ix) {
        for (size_t iy = 0; iy < 3; ++iy) {
            const double x = static_cast<double>(ix) - 1.0;
            const double y = (static_cast<double>(iy) - 1.0) * y_scale;
            if (hole && ix == 2 && iy == 2) continue;  // Missing points remain invalid.
            stream << x << ',' << y << ',' << 2 * x + y << ','
                << (singular ? 4 * x + 2 * y : -x + 3 * y) << '\n';
        }
    }
}

double Dot(const std::array<double, 2> &a, const std::array<double, 2> &b) {
    return a[0] * b[0] + a[1] * b[1];
}

class FakeDevices final : public whole_body::DeviceManager {
    public:
    int Init() override { return 0; }
    int Read(std::vector<motor_state> *motors, imu_data *imu,
        whole_body::DeviceFeedbackStatus *status) override {
        *motors = feedback;
        *imu = {};
        imu->quat[0] = 1.0f;
        status->motor_received.assign(2, 1);
        status->motor_fresh.assign(2, 1);
        status->motor_age_s.assign(2, 0.0);
        status->imu_received = status->imu_fresh = true;
        return 0;
    }
    int Write(const std::vector<motor_cmd> &commands) override {
        written = commands;
        ++writes;
        return 0;
    }
    void Shutdown() override {}
    std::vector<motor_state> feedback = std::vector<motor_state>(2);
    std::vector<motor_cmd> written;
    int writes = 0;
};

void TestCoreProtection(const whole_body::CouplingConfig &coupling, bool hole = false,
    bool native_impedance = false) {
    whole_body::RuntimeConfig config;
    config.num_dof = 2;
    config.cycle_s = 0.001;
    config.feedback_timeout_s = config.command_timeout_s = 0.1;
    config.startup_feedback_timeout_s = 1.0;
    config.joint_names = {"first", "second"};
    for (size_t i = 0; i < 2; ++i) {
        whole_body::MotorConfig motor;
        motor.name = "motor_" + std::to_string(i);
        config.motors.push_back(motor);
        whole_body::JointConfig joint;
        joint.name = config.joint_names[i];
        joint.mapping = "table_2d";
        joint.motors = {"motor_0", "motor_1"};
        joint.position_limit = {-0.5, 0.5};
        joint.velocity_limit = joint.torque_limit = 10;
        joint.impedance.mode = native_impedance ? whole_body::ImpedanceMode::kMotor :
            whole_body::ImpedanceMode::kSoftware;
        config.joints.push_back(joint);
    }
    auto named = coupling;
    named.joints = {"first", "second"};
    named.motors = {"motor_0", "motor_1"};
    config.couplings.push_back(named);
    whole_body::CoupledMapping mapping(named);
    std::array<double, 2> motor;
    assert(mapping.JointToMotor({-0.2, 0.1}, &motor));
    auto devices = std::make_unique<FakeDevices>();
    auto *fake = devices.get();
    for (size_t i = 0; i < 2; ++i) fake->feedback[i].pos = static_cast<float>(motor[i]);
    whole_body::WholeBodyCore read_only(config, std::move(devices));
    assert(read_only.Init() == WHOLE_BODY_OK);
    whole_body_state state;
    assert(read_only.Read(&state) == WHOLE_BODY_OK);
    assert(read_only.SetMode(WHOLE_BODY_MODE_RL) == WHOLE_BODY_ERR_READ_ONLY);
    assert(fake->writes == 0);

    config.read_only = false;
    config.allow_actuation = true;
    devices = std::make_unique<FakeDevices>();
    fake = devices.get();
    for (size_t i = 0; i < 2; ++i) fake->feedback[i].pos = static_cast<float>(motor[i]);
    whole_body::WholeBodyCore actuated(config, std::move(devices));
    assert(actuated.Init() == WHOLE_BODY_OK && actuated.Read(&state) == WHOLE_BODY_OK);
    whole_body_joint_command command{};
    command.num_dof = 2;
    command.mode = WHOLE_BODY_MODE_RL;
    command.enable = true;
    command.position[0] = -0.1;
    command.position[1] = 0.2;
    command.kp[0] = command.kp[1] = 1.0;
    assert(actuated.Write(command, 1.0) == WHOLE_BODY_OK);
    if (!native_impedance) {
        for (const auto &sent : fake->written) assert(sent.kp == 0.0f && sent.kd == 0.0f);
    }
    if (hole) {
        command.position[0] = command.position[1] = 0.2;  // Both axes legal, combined cell absent.
        if (native_impedance) {
            assert(actuated.Write(command, 1.01) == WHOLE_BODY_ERR_COMMAND);
            assert(actuated.LastError().find("coupled motor position target rejected") !=
                std::string::npos);
            for (const auto &sent : fake->written) assert(sent.mode == MOTOR_MODE_IDLE);
            command.position[0] = -0.1;
            assert(actuated.Write(command, 1.02) != WHOLE_BODY_OK);  // Rejection stays latched.
            return;
        }
        assert(actuated.Write(command, 1.01) == WHOLE_BODY_OK);
        std::array<double, 2> expected_torque;
        const std::array<double, 2> measured{state.position[0], state.position[1]};
        assert(mapping.JointTorqueToMotor(measured,
            {0.2 - measured[0], 0.2 - measured[1]}, &expected_torque));
        for (size_t i = 0; i < 2; ++i) {
            assert(fake->written[i].kp == 0.0f && fake->written[i].kd == 0.0f);
            assert(std::abs(fake->written[i].trq_des - expected_torque[i]) < 1.0e-7);
        }
    }
    fake->feedback[0].pos = 9.0f;
    assert(actuated.Read(&state) == WHOLE_BODY_ERR_STATE);
    for (const auto &sent : fake->written) assert(sent.mode == MOTOR_MODE_IDLE);
    assert(actuated.Write(command, 1.01) != WHOLE_BODY_OK);  // Fault remains latched.

    devices = std::make_unique<FakeDevices>();
    fake = devices.get();
    for (size_t i = 0; i < 2; ++i) fake->feedback[i].pos = static_cast<float>(motor[i]);
    whole_body::WholeBodyCore limits(config, std::move(devices));
    assert(limits.Init() == WHOLE_BODY_OK && limits.Read(&state) == WHOLE_BODY_OK);
    command.position[0] = 0.6;  // Within table range, outside configured joint limit.
    assert(limits.Write(command, 1.0) == WHOLE_BODY_ERR_COMMAND);
    for (const auto &sent : fake->written) assert(sent.mode == MOTOR_MODE_IDLE);
}

}  // namespace

int main(int argc, char **argv) {
    whole_body::CouplingConfig config;
    config.type = "table_2d";
    config.motor_limit = 10;
    config.ankle_limit = 2;
    config.motor_difference_limit = 20;
    config.max_iterations = 60;
    config.squared_tolerance = 1.0e-12;
    char path[] = "/tmp/whole-body-table-XXXXXX";
    int fd = mkstemp(path);
    assert(fd >= 0);
    close(fd);
    config.table_file = path;
    MakeTable(path, false);
    whole_body::CoupledMapping mapping(config);
    const std::array<double, 2> joint{-0.2, 0.1};
    std::array<double, 2> motor, recovered, motor_velocity, motor_torque, joint_torque;
    std::array<double, 4> jacobian;
    assert(mapping.JointToMotor(joint, &motor, &jacobian));
    assert(std::abs(motor[0] + 0.3) < 1.0e-12 && std::abs(motor[1] - 0.5) < 1.0e-12);
    assert(mapping.MotorToJoint(motor, &recovered));
    for (size_t axis = 0; axis < 2; ++axis) {
        assert(std::abs(recovered[axis] - joint[axis]) < 1.0e-6);
        auto plus = joint;
        auto minus = joint;
        plus[axis] += 1.0e-6;
        minus[axis] -= 1.0e-6;
        std::array<double, 2> p, m;
        assert(mapping.JointToMotor(plus, &p) && mapping.JointToMotor(minus, &m));
        for (size_t side = 0; side < 2; ++side)
            assert(std::abs((p[side] - m[side]) / 2.0e-6 - jacobian[side * 2 + axis]) < 1.0e-8);
    }
    const std::array<double, 2> velocity{0.3, -0.7};
    const std::array<double, 2> torque{1.2, -3.4};
    assert(mapping.JointVelocityToMotor(joint, velocity, &motor_velocity));
    assert(mapping.JointTorqueToMotor(joint, torque, &motor_torque));
    assert(std::abs(Dot(velocity, torque) - Dot(motor_velocity, motor_torque)) < 1.0e-12);
    assert(mapping.MotorTorqueToJoint(joint, motor_torque, &joint_torque));
    assert(std::abs(joint_torque[0] - torque[0]) < 1.0e-12);
    assert(!mapping.JointToMotor({1.0 + 1.0e-9, 0.0}, &motor));
    assert(!mapping.MotorToJoint({9.0, 9.0}, &recovered));
    config.table_direction = "motor_to_joint";
    whole_body::CoupledMapping reversed(config);
    assert(reversed.MotorToJoint({-0.2, 0.1}, &recovered, &jacobian));
    assert(std::abs(recovered[0] + 0.3) < 1.0e-12 && std::abs(recovered[1] - 0.5) < 1.0e-12);
    assert(reversed.JointToMotor(recovered, &motor));
    assert(reversed.CheckJointTarget(recovered));
    assert(!reversed.CheckJointTarget({2.0, -2.0}));  // Within scalar output bound, not image of grid.
    assert(std::abs(motor[0] + 0.2) < 1.0e-6 && std::abs(motor[1] - 0.1) < 1.0e-6);
    assert(std::abs(jacobian[0] - 3.0 / 7.0) < 1.0e-12);
    assert(reversed.JointVelocityToMotor(recovered, velocity, &motor_velocity));
    assert(reversed.JointTorqueToMotor(recovered, torque, &motor_torque));
    assert(std::abs(Dot(velocity, torque) - Dot(motor_velocity, motor_torque)) < 1.0e-12);
    config.table_output_scale = {-1.0, 1.0};
    whole_body::CoupledMapping signed_output(config);
    assert(signed_output.MotorToJoint({-0.2, 0.1}, &recovered));
    assert(std::abs(recovered[0] - 0.3) < 1.0e-12 && std::abs(recovered[1] - 0.5) < 1.0e-12);
    assert(signed_output.JointVelocityToMotor(recovered, velocity, &motor_velocity));
    assert(signed_output.JointTorqueToMotor(recovered, torque, &motor_torque));
    assert(std::abs(Dot(velocity, torque) - Dot(motor_velocity, motor_torque)) < 1.0e-12);
    config.table_output_scale = {1.0, 1.0};
    config.table_direction = "joint_to_motor";
    TestCoreProtection(config);
    MakeTable(path, true);
    whole_body::CoupledMapping hole(config);
    TestCoreProtection(config, true);
    TestCoreProtection(config, true, true);
    assert(!hole.JointToMotor({0.2, 0.2}, &motor));
    assert(!hole.CheckJointTarget({0.2, 0.2}));
    assert(!hole.JointToMotor({0.0, 0.0}, &motor));  // Zero weight cannot hide a NaN derivative.
    MakeTable(path, false, true);
    whole_body::CoupledMapping singular(config);
    double condition, amplification;
    assert(!singular.Metrics(joint, &condition, &amplification));
    assert(!singular.CheckJointTarget(joint));
    assert(!singular.JointTorqueToMotor(joint, torque, &motor_torque));
    { std::ofstream short_file(path); short_file << "0,0,1"; }
    bool rejected = false;
    try { whole_body::CoupledMapping invalid(config); }
    catch (const std::runtime_error &) { rejected = true; }
    assert(rejected);
    MakeTable(path, false);
    { std::ofstream duplicate(path, std::ios::app); duplicate << "0,0,0,0\n"; }
    rejected = false;
    try { whole_body::CoupledMapping invalid(config); }
    catch (const std::runtime_error &) { rejected = true; }
    assert(rejected);
    MakeTable(path, false);
    config.jacobian_condition_limit = 1.01;
    whole_body::CoupledMapping guarded(config);
    assert(!guarded.CheckJointTarget(joint));
    config.jacobian_condition_limit = 0.0;
    config.table_output_scale = {0.0, 1.0};
    rejected = false;
    try { whole_body::CoupledMapping invalid(config); }
    catch (const std::runtime_error &) { rejected = true; }
    assert(rejected);
    config.table_output_scale = {1.0, 1.0};
    MakeTable(path, false, false, 0.5);
    whole_body::CoupledMapping rectangular(config);
    assert(rectangular.JointToMotor(joint, &motor, &jacobian));
    assert(std::abs(motor[0] + 0.3) < 1.0e-12 && std::abs(motor[1] - 0.5) < 1.0e-12);
    assert(std::abs(jacobian[1] - 1.0) < 1.0e-12 && std::abs(jacobian[3] - 3.0) < 1.0e-12);
    config.table_interpolation = "delaunay";
    whole_body::CoupledMapping triangles(config);
    assert(triangles.JointToMotor(joint, &motor, &jacobian));
    assert(std::abs(motor[0] + 0.3) < 1.0e-12 && std::abs(motor[1] - 0.5) < 1.0e-12);
    assert(std::abs(jacobian[1] - 1.0) < 1.0e-12 && std::abs(jacobian[3] - 3.0) < 1.0e-12);
    assert(triangles.MotorToJoint(motor, &recovered));
    assert(std::abs(recovered[0] - joint[0]) < 1e-6 && std::abs(recovered[1] - joint[1]) < 1e-6);
    // A non-planar cell must use a constant triangle derivative, not bilinear blending.
    { std::ofstream surface(path); surface << "0,0,0,0\n0,1,1,3\n1,0,2,-1\n1,1,4,2\n"; }
    whole_body::CoupledMapping curved(config);
    assert(curved.JointToMotor({0.73, 0.19}, &motor, &jacobian));
    const auto derivative = jacobian;
    assert(curved.JointToMotor({0.74, 0.20}, &motor, &jacobian));
    for (size_t i = 0; i < 4; ++i) assert(std::abs(jacobian[i] - derivative[i]) < 1e-12);
    MakeTable(path, true);
    whole_body::CoupledMapping triangular_hole(config);
    assert(!triangular_hole.JointToMotor({0.2, 0.2}, &motor));
    unlink(path);

    // Optional real table: numeric round trips/derivatives only, not physical calibration.
    if (argc == 2) {
        config.table_file = argv[1];
        config.table_direction = "motor_to_joint";
        for (double sign : {-1.0, 1.0}) {
            config.table_output_scale = {sign, 1.0};
            whole_body::CoupledMapping real(config);
            size_t valid = 0;
            for (double x : {-0.2, -0.1, 0.0, 0.1, 0.2}) {
                for (double y : {-0.1, 0.0, 0.1}) {
                    if (!real.JointToMotor({x, y}, &motor, &jacobian)) continue;
                    assert(real.MotorToJoint(motor, &recovered));
                    assert(std::abs(recovered[0] - x) < 1.0e-4 &&
                        std::abs(recovered[1] - y) < 1.0e-4);
                    ++valid;
                }
            }
            assert(valid == 15);
        }
    }
    return 0;
}

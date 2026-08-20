/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body_config.h
 * @brief Internal whole-body YAML configuration types
 */

#ifndef WHOLE_BODY_CONFIG_H
#define WHOLE_BODY_CONFIG_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace whole_body {

struct BusConfig {
    std::string name;
    std::string type;
    std::string device;
    uint32_t bitrate = 0;
};

struct DriverOption {
    std::string name;
    std::string value;
};

struct MotorConfig {
    std::string name;
    std::string driver;
    std::string bus;
    std::string model;
    uint16_t command_id = 0;
    uint16_t feedback_id = 0;
    double polarity = 1.0;
    double zero_offset = 0.0;
    std::vector<DriverOption> driver_options;
};

enum class ImpedanceMode {
    kMotor,
    kSoftware,
    kSplit,
};

struct ImpedanceConfig {
    ImpedanceMode mode = ImpedanceMode::kMotor;
    double motor_kp_max = 0.0;
    double motor_kd_max = 0.0;
};

struct JointConfig {
    std::string name;
    std::string mapping;
    std::vector<std::string> motors;
    std::array<double, 2> position_limit{};
    double velocity_limit = 0.0;
    double torque_limit = 0.0;
    ImpedanceConfig impedance;
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct ParallelAnkleConfig {
    std::array<std::string, 2> joints;
    std::array<std::string, 2> motors;
    std::array<Vec3, 2> ankle_pivots;
    std::array<Vec3, 2> ball_positions;
    std::array<Vec3, 2> motor_positions;
    std::array<double, 2> motor_bias{};
    double motor_limit = 0.0;
    double ankle_limit = 0.0;
    double motor_difference_limit = 0.0;
    int max_iterations = 0;
    double squared_tolerance = 0.0;
};

struct ImuConfig {
    std::string driver;
    std::string device;
    uint32_t baud = 0;
    std::array<float, 9> mounting_matrix{};
    std::array<float, 3> acceleration_bias{};
    std::array<float, 3> gyro_bias{};
};

struct RuntimeConfig {
    std::string main_config_path;
    std::string hardware_config_path;
    std::string robot_name;
    uint32_t num_dof = 0;
    double cycle_s = 0.0;
    double startup_feedback_timeout_s = 0.0;
    double feedback_timeout_s = 0.0;
    double command_timeout_s = 0.0;
    bool read_only = true;
    bool allow_actuation = false;
    std::vector<std::string> joint_names;
    std::vector<BusConfig> buses;
    std::vector<MotorConfig> motors;
    std::vector<JointConfig> joints;
    std::vector<ParallelAnkleConfig> couplings;
    ImuConfig imu;
};

RuntimeConfig LoadConfig(const std::string &main_config_path);

}  // namespace whole_body

#endif  // WHOLE_BODY_CONFIG_H

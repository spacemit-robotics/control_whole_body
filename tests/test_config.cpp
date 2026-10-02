/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_config.cpp
 * @brief Offline tests for the whole-body YAML contract
 */

#include <yaml-cpp/yaml.h>

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "device_manager.h"
#include "whole_body.h"
#include "whole_body_config.h"

namespace {

void ValidatePublicCreate(const std::string &path, bool has_imu) {
    whole_body_dev *device = nullptr;
    assert(whole_body_create(path.c_str(), &device) == WHOLE_BODY_OK);
    assert(device != nullptr);
    assert(whole_body_has_imu(device) == static_cast<int>(has_imu));
    whole_body_health health{};
    assert(whole_body_get_health(device, &health) == WHOLE_BODY_OK);
    assert(health.state == WHOLE_BODY_HEALTH_CREATED);
    double cycle_s = 0.0;
    assert(whole_body_get_cycle_s(device, &cycle_s) == WHOLE_BODY_OK);
    assert(cycle_s > 0.0);
    whole_body_destroy(device);
}

void ValidateImuConfig(const YAML::Node &hardware, bool valid, bool has_imu = true) {
    const std::string directory = TEST_CONFIG_DIR;
    auto main = YAML::LoadFile(std::string(TEST_DATA_DIR) + "/main.yaml");
    main["whole_body"]["config_file"] = "hardware.yaml";
    {
        std::ofstream output(directory + "/hardware.yaml");
        output << hardware;
        assert(output.good());
    }
    {
        std::ofstream output(directory + "/main.yaml");
        output << main;
        assert(output.good());
    }
    bool rejected = false;
    try {
        const auto config = whole_body::LoadConfig(directory + "/main.yaml");
        assert(config.imu.enabled == has_imu);
        if (!has_imu) {
            assert(config.imu.driver.empty());
            assert(config.imu.device.empty());
        }
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    assert(rejected == !valid);
    if (valid) ValidatePublicCreate(directory + "/main.yaml", has_imu);
}

void TestOptionalImuConfig() {
    auto hardware = YAML::LoadFile(std::string(TEST_DATA_DIR) + "/hardware.yaml");
    hardware["whole_body"]["imu"]["enabled"] = true;
    ValidateImuConfig(hardware, true);
    hardware["whole_body"]["imu"].remove("driver");
    ValidateImuConfig(hardware, false);
    hardware["whole_body"]["imu"].remove("enabled");
    ValidateImuConfig(hardware, false);
    hardware["whole_body"].remove("imu");
    ValidateImuConfig(hardware, false);
    hardware["whole_body"]["imu"] = YAML::Node(YAML::NodeType::Map);
    ValidateImuConfig(hardware, false);
    hardware["whole_body"]["imu"]["enabled"] = false;
    ValidateImuConfig(hardware, true, false);
    hardware["whole_body"]["imu"]["driver"] = "nonexistent_imu_driver";
    hardware["whole_body"]["imu"]["baud"] = "unused";
    ValidateImuConfig(hardware, true, false);
    hardware["whole_body"]["imu"]["enabled"] = "not_a_boolean";
    ValidateImuConfig(hardware, false);
    hardware["whole_body"]["imu"]["enabled"] = YAML::Node();
    ValidateImuConfig(hardware, false);
    hardware["whole_body"]["imu"]["enabled"] = false;
    hardware["whole_body"]["imu"]["unknown"] = true;
    ValidateImuConfig(hardware, false);
    hardware["whole_body"]["imu"].remove("unknown");
    hardware["whole_body"]["motors"][0]["bus"] = "missing_bus";
    ValidateImuConfig(hardware, false);
}

}  // namespace

int main(int argc, char *argv[]) {
    if (argc == 2) {
        const auto config = whole_body::LoadConfig(argv[1]);
        assert(config.num_dof == config.joints.size());
        assert(!config.motors.empty());
        assert(config.read_only != config.allow_actuation);
        ValidatePublicCreate(argv[1], config.imu.enabled);
        std::cout << "Validated whole-body config for " << config.robot_name << "\n";
        return 0;
    }

    const std::string data_dir = TEST_DATA_DIR;
    const auto config = whole_body::LoadConfig(data_dir + "/main.yaml");
    assert(config.num_dof == 2);
    assert(config.joint_names[1] == "joint_1");
    assert(config.motors.size() == 2);
    assert(std::abs(config.motors[0].position_period - 6.283185307) < 1.0e-9);
    assert(config.motors[1].position_period == 0.0);
    assert(config.motors[0].non_fatal_error_codes == std::vector<uint32_t>({0x04U}));
    assert(config.motors[0].command_limits.kp_max == 500.0);
    assert(config.motors[0].command_limits.estimated_torque_max == 90.0);
    assert(config.joints[0].impedance.mode == whole_body::ImpedanceMode::kSplit);
    assert(config.joints[0].impedance.motor_kp_max == 500.0);
    assert(config.joints[0].impedance.motor_kd_max == 5.0);
    assert(config.joints[1].impedance.mode == whole_body::ImpedanceMode::kMotor);
    const auto driver_options = whole_body::BuildMotorDriverOptions(config.motors[0]);
    bool found_model = false;
    bool found_feedback_id = false;
    for (const auto &option : driver_options) {
        if (option.name == "model" && option.value == config.motors[0].model)
            found_model = true;
        if (option.name == "feedback_id" && option.value == "1")
            found_feedback_id = true;
    }
    assert(found_model);
    assert(found_feedback_id);
    assert(!config.read_only);
    assert(config.allow_actuation);
    assert(config.startup_feedback_timeout_s > config.feedback_timeout_s);
    assert(config.require_motor_receive_timestamps);
    assert(config.imu.enabled);
    assert(whole_body_has_imu(nullptr) == WHOLE_BODY_ERR_STATE);
    ValidatePublicCreate(data_dir + "/main.yaml", true);
    TestOptionalImuConfig();

    bool rejected_unknown_field = false;
    try {
        (void)whole_body::LoadConfig(data_dir + "/main_invalid.yaml");
    } catch (const std::runtime_error &) {
        rejected_unknown_field = true;
    }
    assert(rejected_unknown_field);
    std::cout << "Whole-body config tests passed\n";
    return 0;
}

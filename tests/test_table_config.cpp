/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_table_config.cpp
 * @brief Validate explicit coupled impedance modes, feedback limits and table contracts.
 */
#include "whole_body_config.h"

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

namespace {

void WriteYaml(const std::string &path, const YAML::Node &node) {
    std::ofstream stream(path);
    stream << node;
    assert(stream.good());
}

bool Rejected(const std::string &main) {
    try { (void)whole_body::LoadConfig(main); }
    catch (const std::runtime_error &) { return true; }
    return false;
}

}  // namespace

int main(void) {
    char directory[] = "/tmp/whole-body-config-XXXXXX";
    assert(mkdtemp(directory) != nullptr);
    const std::string main_path = std::string(directory) + "/main.yaml";
    const std::string hardware_path = std::string(directory) + "/hardware.yaml";
    const std::string table_path = std::string(directory) + "/local.csv";
    { std::ofstream placeholder(table_path); assert(placeholder.good()); }
    auto main = YAML::LoadFile(std::string(TEST_DATA_DIR) + "/main.yaml");
    auto hardware = YAML::LoadFile(std::string(TEST_DATA_DIR) + "/hardware.yaml");
    main["whole_body"]["config_file"] = "hardware.yaml";
    auto root = hardware["whole_body"];
    root["buses"][0]["type"] = "uart";
    root["buses"][0]["bitrate"] = 6000000;
    for (size_t i = 0; i < 2; ++i) {
        root["joints"][i]["mapping"] = "table_2d";
        root["joints"][i]["motors"] = YAML::Load("[motor_0, motor_1]");
        root["joints"][i]["impedance"] = YAML::Load("{mode: software}");
    }
    root["couplings"] = YAML::Load(
        "[{type: table_2d, joints: [joint_0, joint_1], motors: [motor_0, motor_1], "
        "table_file: local.csv, table_direction: motor_to_joint, motor_limit: 1.0, "
        "ankle_limit: 0.5, motor_difference_limit: 1.0, max_iterations: 60, "
        "squared_tolerance: 1.0e-12}]");
    root["imu"]["base_joint_chain"] = YAML::Load("[{joint: joint_0, axis: [0, 0, 1]}]");
    WriteYaml(main_path, main);
    WriteYaml(hardware_path, hardware);
    const auto config = whole_body::LoadConfig(main_path);
    assert(config.buses[0].type == "uart");
    assert(config.couplings[0].table_file == std::string(directory) + "/local.csv");
    assert(config.imu.base_joint_chain.size() == 1);
    assert(config.joints[0].impedance.velocity_filter_time_constant_s == 0.0);

    root["communication_tolerance_s"] = 0.05;
    WriteYaml(hardware_path, hardware);
    assert(whole_body::LoadConfig(main_path).communication_tolerance_s == 0.05);
    root["communication_tolerance_s"] = root["command_timeout_s"].as<double>();
    root["refresh_last_command"] = true;
    WriteYaml(hardware_path, hardware);
    assert(whole_body::LoadConfig(main_path).refresh_last_command);
    for (const char *invalid : {"-0.01", "0.02", "0.101", ".nan", ".inf"}) {
        root["communication_tolerance_s"] = YAML::Load(invalid);
        WriteYaml(hardware_path, hardware);
        assert(Rejected(main_path));
    }
    root.remove("communication_tolerance_s");

    root["joints"][0]["impedance"]["velocity_filter_time_constant_s"] = 0.01;
    WriteYaml(hardware_path, hardware);
    assert(whole_body::LoadConfig(main_path).joints[0].impedance.velocity_filter_time_constant_s == 0.01);
    for (const char *invalid : {"-0.01", ".nan", ".inf"}) {
        root["joints"][0]["impedance"]["velocity_filter_time_constant_s"] = YAML::Load(invalid);
        WriteYaml(hardware_path, hardware);
        assert(Rejected(main_path));
    }
    root["joints"][0]["impedance"].remove("velocity_filter_time_constant_s");

    const auto valid = YAML::Clone(hardware);
    root["joints"][0]["impedance"]["mode"] = "motor";
    WriteYaml(hardware_path, hardware);
    assert(Rejected(main_path));  // Half of a coupling cannot opt into native PD.
    root["joints"][1]["impedance"]["mode"] = "motor";
    root["motors"][0]["feedback_limits"]["velocity_max"] = 2.0;
    WriteYaml(hardware_path, hardware);
    const auto native = whole_body::LoadConfig(main_path);
    assert(native.joints[0].impedance.mode == whole_body::ImpedanceMode::kMotor);
    assert(native.motors[0].feedback_velocity_max == 2.0);
    assert(native.motors[1].feedback_velocity_max == 0.0);
    for (size_t i = 0; i < 2; ++i)
        root["joints"][i]["impedance"]["mode"] = "motor_projected";
    WriteYaml(hardware_path, hardware);
    assert(whole_body::LoadConfig(main_path).joints[0].impedance.mode ==
        whole_body::ImpedanceMode::kMotorProjected);
    for (size_t i = 0; i < 2; ++i) {
        root["joints"][i]["impedance"]["motor_kp_max"] = 2000.0;
        root["joints"][i]["impedance"]["motor_kd_max"] = 25.0;
        root["joints"][i]["feedback_position_check"] = false;
    }
    root["couplings"][0]["table_interpolation"] = "delaunay";
    WriteYaml(hardware_path, hardware);
    const auto projected = whole_body::LoadConfig(main_path);
    assert(projected.joints[0].impedance.motor_kp_max == 2000.0);
    assert(projected.joints[0].impedance.motor_kd_max == 25.0);
    assert(!projected.joints[0].feedback_position_check);
    assert(projected.couplings[0].table_interpolation == "delaunay");
    root["joints"][0]["impedance"]["motor_kp_max"] = -1.0;
    WriteYaml(hardware_path, hardware);
    assert(Rejected(main_path));
    root["joints"][0]["impedance"]["motor_kp_max"] = 2000.0;
    root["couplings"][0]["table_interpolation"] = "unknown";
    WriteYaml(hardware_path, hardware);
    assert(Rejected(main_path));
    root["couplings"][0]["table_interpolation"] = "delaunay";
    root["joints"][0]["impedance"]["rl_mode"] = "software";
    WriteYaml(hardware_path, hardware);
    assert(Rejected(main_path));
    for (size_t i = 0; i < 2; ++i) {
        root["joints"][i]["impedance"] = YAML::Load(
            "{mode: motor, rl_mode: split, motor_kp_max: 0, motor_kd_max: 5}");
    }
    WriteYaml(hardware_path, hardware);
    const auto split = whole_body::LoadConfig(main_path);
    assert(split.joints[0].impedance.mode == whole_body::ImpedanceMode::kMotor);
    assert(split.joints[0].impedance.rl_mode == whole_body::ImpedanceMode::kSplit);
    root["motors"][0]["feedback_limits"]["velocity_max"] = -1.0;
    WriteYaml(hardware_path, hardware);
    assert(Rejected(main_path));
    hardware = YAML::Clone(valid);
    hardware["whole_body"]["couplings"][0]["table_output_scale"] = YAML::Load("[0, 1]");
    WriteYaml(hardware_path, hardware);
    assert(Rejected(main_path));
    hardware = YAML::Clone(valid);
    root = hardware["whole_body"];
    root["couplings"][0].remove("table_direction");
    WriteYaml(hardware_path, hardware);
    assert(Rejected(main_path));
    hardware = YAML::Clone(valid);
    hardware["whole_body"]["joints"][0]["impedance"] =
        YAML::Load("{mode: split, motor_kp_max: 15, motor_kd_max: 0.1}");
    WriteYaml(hardware_path, hardware);
    assert(Rejected(main_path));
    hardware = YAML::Clone(valid);
    hardware["whole_body"]["imu"]["base_joint_chain"][0]["axis"] = YAML::Load("[0, 0, 2]");
    WriteYaml(hardware_path, hardware);
    assert(Rejected(main_path));
    hardware = YAML::Clone(valid);
    hardware["whole_body"]["imu"]["base_joint_chain"][0]["joint"] = "missing";
    WriteYaml(hardware_path, hardware);
    assert(Rejected(main_path));
    unlink(main_path.c_str());
    unlink(hardware_path.c_str());
    unlink(table_path.c_str());
    rmdir(directory);
    return 0;
}

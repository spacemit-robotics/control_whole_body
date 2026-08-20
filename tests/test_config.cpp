/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_config.cpp
 * @brief Offline tests for the whole-body YAML contract
 */

#include <cassert>
#include <iostream>
#include <stdexcept>
#include <string>

#include "device_manager.h"
#include "whole_body.h"
#include "whole_body_config.h"

namespace {

void ValidatePublicCreate(const std::string &path) {
    whole_body_dev *device = nullptr;
    assert(whole_body_create(path.c_str(), &device) == WHOLE_BODY_OK);
    assert(device != nullptr);
    double cycle_s = 0.0;
    assert(whole_body_get_cycle_s(device, &cycle_s) == WHOLE_BODY_OK);
    assert(cycle_s > 0.0);
    whole_body_destroy(device);
}

}  // namespace

int main(int argc, char *argv[]) {
    if (argc == 2) {
        const auto config = whole_body::LoadConfig(argv[1]);
        assert(config.num_dof == config.joints.size());
        assert(!config.motors.empty());
        assert(config.read_only != config.allow_actuation);
        ValidatePublicCreate(argv[1]);
        std::cout << "Validated whole-body config for " << config.robot_name << "\n";
        return 0;
    }

    const std::string data_dir = TEST_DATA_DIR;
    const auto config = whole_body::LoadConfig(data_dir + "/main.yaml");
    assert(config.num_dof == 2);
    assert(config.joint_names[1] == "joint_1");
    assert(config.motors.size() == 2);
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
    ValidatePublicCreate(data_dir + "/main.yaml");

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

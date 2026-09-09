/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_motor_write_errors.cpp
 * @brief Offline tests for per-motor command failure diagnostics.
 */

#include <array>
#include <cassert>
#include <cerrno>
#include <iostream>
#include <string>
#include <vector>

#include "device_manager.h"

namespace {

struct FakeMotor {
    int result = 0;
    int system_error = 0;
    int calls = 0;
};

int WriteFakeMotor(motor_dev *device, const motor_cmd *) {
    auto *motor = reinterpret_cast<FakeMotor *>(device);
    ++motor->calls;
    if (motor->system_error != 0) errno = motor->system_error;
    return motor->result;
}

}  // namespace

int main() {
    whole_body::RuntimeConfig config;
    config.buses.push_back({"bus_0", "socketcan", "can0", 1000000});
    config.motors.resize(3);
    std::array<FakeMotor, 3> fake_motors{};
    std::vector<motor_dev *> devices;
    std::vector<motor_cmd> commands(3);
    for (size_t i = 0; i < fake_motors.size(); ++i) {
        devices.push_back(reinterpret_cast<motor_dev *>(&fake_motors[i]));
        auto &motor = config.motors[i];
        motor.name = "motor_" + std::to_string(i);
        motor.driver = "fake_driver";
        motor.bus = "bus_0";
        motor.command_id = i + 1;
        motor.feedback_id = i + 17;
        commands[i].mode = MOTOR_MODE_IDLE;
    }

    fake_motors[0].result = -1;
    fake_motors[0].system_error = ENOBUFS;
    fake_motors[1].system_error = EBUSY;
    fake_motors[2].result = -9;
    fake_motors[2].system_error = ENETDOWN;
    std::string error;
    assert(whole_body::WriteMotorCommands(config, devices, commands, &error, WriteFakeMotor) == -1);
    for (const auto &motor : fake_motors) assert(motor.calls == 1);
    assert(error.find("motor_0(driver=fake_driver,bus=bus_0/can0,cmd=0x1,fb=0x11,") !=
        std::string::npos);
    assert(error.find("motor_2(driver=fake_driver,bus=bus_0/can0,cmd=0x3,fb=0x13,") !=
        std::string::npos);
    assert(error.find("motor_1") == std::string::npos);
    assert(error.find("operation=disable,mode=0,ret=-1,errno=" + std::to_string(ENOBUFS)) !=
        std::string::npos);
    assert(error.find("ret=-9,errno=" + std::to_string(ENETDOWN)) != std::string::npos);
    std::cout << error << '\n';

    fake_motors[0].result = 0;
    fake_motors[2].system_error = 0;
    commands[2].mode = MOTOR_MODE_HYBRID;
    assert(whole_body::WriteMotorCommands(config, devices, commands, &error, WriteFakeMotor) == -9);
    assert(error.find("motor_0") == std::string::npos);
    assert(error.find("operation=command,mode=4,ret=-9,errno=unavailable") != std::string::npos);

    fake_motors[2].result = 0;
    assert(whole_body::WriteMotorCommands(config, devices, commands, &error, WriteFakeMotor) == 0);
    assert(error.empty());
    for (const auto &motor : fake_motors) assert(motor.calls == 3);

    commands.pop_back();
    assert(whole_body::WriteMotorCommands(config, devices, commands, &error, WriteFakeMotor) < 0);
    assert(error == "invalid motor command batch");
    for (const auto &motor : fake_motors) assert(motor.calls == 3);
    assert(whole_body::WriteMotorCommands(config, devices, commands, nullptr, WriteFakeMotor) < 0);
    std::cout << "Motor write diagnostics tests passed\n";
    return 0;
}

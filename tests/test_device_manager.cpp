/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_device_manager.cpp
 * @brief Offline peripheral lifecycle and optional IMU feedback tests
 */

#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "device_manager.h"

struct motor_dev {
    size_t index;
};

struct imu_dev {};

namespace {

struct FakePeripherals {
    std::array<motor_dev, 2> motors{{{0}, {1}}};
    std::array<bool, 2> motor_responds{{true, true}};
    std::array<bool, 2> motor_refreshes{{true, true}};
    std::array<bool, 2> motor_has_timestamp{{true, true}};
    std::array<uint64_t, 2> motor_timestamps{};
    imu_dev imu;
    uint64_t imu_receive_timestamp = 0;
    int imu_allocations = 0;
    int imu_initializations = 0;
    int imu_reads = 0;
    int imu_diagnostic_reads = 0;
    int imu_frees = 0;
    int motor_initializations = 0;
    int motor_frees = 0;
    bool imu_allocation_fails = false;
    bool imu_responds = false;
};

FakePeripherals fake;

uint64_t MonotonicMicros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

whole_body::RuntimeConfig MakeConfig(bool enabled) {
    whole_body::RuntimeConfig config;
    config.imu.enabled = enabled;
    config.cycle_s = 0.001;
    config.feedback_timeout_s = 0.04;
    config.startup_feedback_timeout_s = 1.0;
    config.require_motor_receive_timestamps = true;
    config.buses.push_back({"bus", "socketcan", "fake_can", 1000000});
    config.motors.resize(2);
    for (size_t i = 0; i < config.motors.size(); ++i) {
        config.motors[i].bus = "bus";
        config.motors[i].command_id = i;
    }
    return config;
}

void TestDisabledImu() {
    fake = {};
    auto devices = whole_body::CreatePeripheralDevices(MakeConfig(false));
    assert(devices->Init() == 0);
    assert(fake.motor_initializations == 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::vector<motor_state> motors(2);
    imu_data imu{};
    whole_body::DeviceFeedbackStatus status;
    status.imu_received = status.imu_fresh = true;
    status.imu_age_s = 1.0;
    status.imu_parser.valid_frames = 42;
    fake.motor_responds[1] = false;
    assert(devices->Read(&motors, &imu, &status) == whole_body::DEVICE_READ_WAITING);
    assert(status.motor_fresh[0] && !status.motor_received[1]);
    assert(!status.imu_received && !status.imu_fresh);
    assert(status.imu_age_s == 0.0 && status.imu_parser.valid_frames == 0);

    fake.motor_responds[1] = true;
    assert(devices->Read(&motors, &imu, &status) == whole_body::DEVICE_READ_OK);
    assert(status.motor_fresh[0] && status.motor_fresh[1]);
    assert(status.imu_sample_timestamp_s == 0.0 && status.imu_receive_timestamp_s == 0.0);
    assert(status.feedback_window_s ==
        status.motor_timestamp_s[1] - status.motor_timestamp_s[0]);

    fake.motor_refreshes[1] = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    assert(devices->Read(&motors, &imu, &status) == whole_body::DEVICE_READ_ERROR);
    assert(status.motor_fresh[0] && status.motor_received[1] && !status.motor_fresh[1]);
    assert(status.motor_age_s[1] > 0.04);
    fake.motor_refreshes[1] = true;
    assert(devices->Read(&motors, &imu, &status) == whole_body::DEVICE_READ_OK);
    fake.motor_has_timestamp[1] = false;
    assert(devices->Read(&motors, &imu, &status) == whole_body::DEVICE_READ_INCOMPATIBLE);
    assert(status.motor_fresh[0] && !status.motor_fresh[1]);
    devices.reset();
    assert(fake.motor_frees == 2);
    assert(fake.imu_allocations == 0 && fake.imu_initializations == 0);
    assert(fake.imu_reads == 0 && fake.imu_diagnostic_reads == 0 && fake.imu_frees == 0);
}

void TestEnabledImu() {
    fake = {};
    auto devices = whole_body::CreatePeripheralDevices(MakeConfig(true));
    assert(fake.imu_allocations == 1);
    assert(devices->Init() == 0);
    assert(fake.imu_initializations == 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::vector<motor_state> motors(2);
    imu_data imu{};
    whole_body::DeviceFeedbackStatus status;
    assert(devices->Read(&motors, &imu, &status) == whole_body::DEVICE_READ_WAITING);
    assert(status.motor_fresh[0] && status.motor_fresh[1] && !status.imu_received);
    fake.imu_responds = true;
    assert(devices->Read(&motors, &imu, &status) == whole_body::DEVICE_READ_OK);
    assert(status.imu_received && status.imu_fresh);
    assert(imu.timestamp_us == 1234567);
    fake.imu_responds = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    assert(devices->Read(&motors, &imu, &status) == whole_body::DEVICE_READ_ERROR);
    assert(status.motor_fresh[0] && status.motor_fresh[1] && !status.imu_fresh);
    assert(fake.imu_reads == 3 && fake.imu_diagnostic_reads == 3);
    devices.reset();
    assert(fake.imu_frees == 1 && fake.motor_frees == 2);

    fake = {};
    fake.imu_allocation_fails = true;
    bool rejected = false;
    try {
        (void)whole_body::CreatePeripheralDevices(MakeConfig(true));
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    assert(rejected && fake.motor_frees == 2);
}

}  // namespace

extern "C" {

motor_dev *motor_alloc_can_with_options(
    const char *, const char *, uint32_t can_id, const motor_option *, uint32_t) {
    assert(can_id < fake.motors.size());
    return &fake.motors[can_id];
}

int motor_init(motor_dev **, uint32_t count) {
    fake.motor_initializations += count;
    return 0;
}

int motor_set_cmds(motor_dev **, const motor_cmd *, uint32_t) { return 0; }

int motor_get_states(motor_dev **devices, motor_state *states, uint32_t count) {
    assert(count == 1);
    const size_t index = devices[0]->index;
    if (!fake.motor_responds[index]) return -1;
    if (fake.motor_refreshes[index]) fake.motor_timestamps[index] = MonotonicMicros();
    states[0] = {};
    return 0;
}

int motor_get_feedback_timestamps(motor_dev **devices, uint64_t *timestamps, uint32_t count) {
    assert(count == 1);
    const size_t index = devices[0]->index;
    timestamps[0] = fake.motor_has_timestamp[index] ? fake.motor_timestamps[index] : 0;
    return 0;
}

void motor_free(motor_dev **, uint32_t count) { fake.motor_frees += count; }

imu_dev *imu_alloc_uart(const char *, const char *, uint32_t, void *) {
    ++fake.imu_allocations;
    return fake.imu_allocation_fails ? nullptr : &fake.imu;
}

int imu_init(imu_dev *device, const imu_config *) {
    assert(device == &fake.imu);
    ++fake.imu_initializations;
    return 0;
}

int imu_read(imu_dev *device, imu_data *data) {
    assert(device == &fake.imu);
    ++fake.imu_reads;
    if (!fake.imu_responds) return -1;
    *data = {};
    data->quat[0] = 1.0f;
    data->timestamp_us = 1234567;
    fake.imu_receive_timestamp = MonotonicMicros();
    return 0;
}

int imu_get_diagnostics(imu_dev *device, imu_diagnostics *diagnostics) {
    assert(device == &fake.imu);
    ++fake.imu_diagnostic_reads;
    *diagnostics = {};
    diagnostics->receive_timestamp_us = fake.imu_receive_timestamp;
    return 0;
}

void imu_free(imu_dev *device) {
    assert(device == &fake.imu);
    ++fake.imu_frees;
}

}  // extern "C"

int main() {
    TestDisabledImu();
    TestEnabledImu();
    std::cout << "Device manager tests passed\n";
    return 0;
}

/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body.cpp
 * @brief Whole-body public API implementation
 */

#include "whole_body.h"

#include <chrono>
#include <exception>
#include <memory>
#include <new>

#include "device_manager.h"
#include "whole_body_config.h"
#include "whole_body_core.h"

struct whole_body_dev {
    std::unique_ptr<whole_body::WholeBodyCore> core;
};

namespace {

double MonotonicTime() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

extern "C" {

int whole_body_create(const char *main_config_path, struct whole_body_dev **out_dev) {
    if (!main_config_path || !out_dev) return WHOLE_BODY_ERR_CONFIG;
    *out_dev = nullptr;
    try {
        auto config = whole_body::LoadConfig(main_config_path);
        auto devices = whole_body::CreatePeripheralDevices(config);
        auto dev = std::make_unique<whole_body_dev>();
        dev->core =
            std::make_unique<whole_body::WholeBodyCore>(std::move(config), std::move(devices));
        *out_dev = dev.release();
        return WHOLE_BODY_OK;
    } catch (const std::bad_alloc &) {
        return WHOLE_BODY_ERR_ALLOC;
    } catch (const std::exception &) {
        return WHOLE_BODY_ERR_CONFIG;
    }
}

int whole_body_init(struct whole_body_dev *dev) {
    return dev && dev->core ? dev->core->Init() : WHOLE_BODY_ERR_STATE;
}

int whole_body_read(struct whole_body_dev *dev, struct whole_body_state *state) {
    return dev && dev->core ? dev->core->Read(state) : WHOLE_BODY_ERR_STATE;
}

int whole_body_write(struct whole_body_dev *dev, const struct whole_body_joint_command *command) {
    if (!dev || !dev->core || !command) return WHOLE_BODY_ERR_COMMAND;
    return dev->core->Write(*command, MonotonicTime());
}

int whole_body_tick(struct whole_body_dev *dev, double monotonic_time_s) {
    return dev && dev->core ? dev->core->Tick(monotonic_time_s) : WHOLE_BODY_ERR_STATE;
}

int whole_body_set_mode(struct whole_body_dev *dev, enum whole_body_mode mode) {
    return dev && dev->core ? dev->core->SetMode(mode) : WHOLE_BODY_ERR_STATE;
}

int whole_body_get_health(const struct whole_body_dev *dev, struct whole_body_health *health) {
    if (!dev || !dev->core || !health) return WHOLE_BODY_ERR_STATE;
    *health = dev->core->GetHealth();
    return WHOLE_BODY_OK;
}

int whole_body_get_diagnostics(
    const struct whole_body_dev *dev, struct whole_body_diagnostics *diagnostics) {
    if (!dev || !dev->core || !diagnostics) return WHOLE_BODY_ERR_STATE;
    *diagnostics = dev->core->GetDiagnostics();
    return WHOLE_BODY_OK;
}

int whole_body_get_cycle_s(const struct whole_body_dev *dev, double *cycle_s) {
    if (!dev || !dev->core || !cycle_s) return WHOLE_BODY_ERR_STATE;
    *cycle_s = dev->core->CycleSeconds();
    return WHOLE_BODY_OK;
}

const char *whole_body_last_error(const struct whole_body_dev *dev) {
    if (!dev || !dev->core) return "invalid whole-body device";
    return dev->core->LastError().c_str();
}

void whole_body_destroy(struct whole_body_dev *dev) { delete dev; }

}  // extern "C"

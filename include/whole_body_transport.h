/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body_transport.h
 * @brief Optional per-bus transport counters without changing existing diagnostic ABI.
 */
#ifndef WHOLE_BODY_TRANSPORT_H
#define WHOLE_BODY_TRANSPORT_H

#include "motor.h"
#include "whole_body.h"

struct whole_body_bus_diagnostic {
    char name[WHOLE_BODY_NAME_LENGTH];
    char device[WHOLE_BODY_PATH_LENGTH];
    struct motor_transport_diagnostics transport;
};

struct whole_body_transport_diagnostics {
    uint32_t bus_count;
    struct whole_body_bus_diagnostic buses[WHOLE_BODY_MAX_MOTORS];
};

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Read supported bus counters on the same thread as read/write/tick. */
int whole_body_get_transport_diagnostics(const struct whole_body_dev *dev,
    struct whole_body_transport_diagnostics *diagnostics);

#ifdef __cplusplus
}
#endif
#endif  // WHOLE_BODY_TRANSPORT_H

/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body.h
 * @brief Robot-model-independent whole-body hardware control API
 */

#ifndef WHOLE_BODY_H
#define WHOLE_BODY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WHOLE_BODY_MAX_DOF 64
#define WHOLE_BODY_MAX_MOTORS 128
#define WHOLE_BODY_NAME_LENGTH 64
#define WHOLE_BODY_PATH_LENGTH 128
#define WHOLE_BODY_JOINT_LABEL_LENGTH 128

enum whole_body_error {
    WHOLE_BODY_OK = 0,
    WHOLE_BODY_ERR_ALLOC = -1,
    WHOLE_BODY_ERR_CONFIG = -2,
    WHOLE_BODY_ERR_DEVICE = -3,
    WHOLE_BODY_ERR_STATE = -4,
    WHOLE_BODY_ERR_COMMAND = -5,
    WHOLE_BODY_ERR_READ_ONLY = -6,
    WHOLE_BODY_ERR_TIMEOUT = -7,
};

enum whole_body_mode {
    WHOLE_BODY_MODE_POWER_OFF = 0,
    WHOLE_BODY_MODE_DAMP = 1,
    WHOLE_BODY_MODE_ZERO = 2,
    WHOLE_BODY_MODE_RL = 3,
    WHOLE_BODY_MODE_SAFETY = 4,
    WHOLE_BODY_MODE_HOME = 5,
};

/**
 * @brief Interpretation of the joint command fields, independent of whole_body_mode.
 */
enum whole_body_actuation_mode {
    WHOLE_BODY_ACTUATION_HYBRID = 0,    /**< Position/velocity PD plus torque feed-forward. */
    WHOLE_BODY_ACTUATION_POSITION = 1,  /**< Position is the primary target. */
    WHOLE_BODY_ACTUATION_VELOCITY = 2,  /**< Velocity is the primary target. */
    WHOLE_BODY_ACTUATION_TORQUE = 3,    /**< Torque is the direct target. */
};

enum whole_body_health_state {
    WHOLE_BODY_HEALTH_CREATED = 0,
    WHOLE_BODY_HEALTH_READY = 1,
    WHOLE_BODY_HEALTH_READ_ONLY = 2,
    WHOLE_BODY_HEALTH_WATCHDOG = 3,
    WHOLE_BODY_HEALTH_ERROR = 4,
};

/**
 * @brief Canonical joint command.
 *
 * actuation_mode applies uniformly to all num_dof joints. All arrays use
 * canonical joint order; fields not used by a mode remain finite and zeroed.
 */
struct whole_body_joint_command {
    uint32_t num_dof;
    bool enable;
    enum whole_body_mode mode;  /**< FSM behavior and safety state. */
    enum whole_body_actuation_mode actuation_mode;  /**< Interpretation for all joints. */
    double position[WHOLE_BODY_MAX_DOF];
    double velocity[WHOLE_BODY_MAX_DOF];
    /* HYBRID: feed-forward torque; TORQUE: direct target torque (Nm). */
    double torque[WHOLE_BODY_MAX_DOF];
    double kp[WHOLE_BODY_MAX_DOF];
    double kd[WHOLE_BODY_MAX_DOF];
};

struct whole_body_state {
    uint32_t num_dof;
    double timestamp_s;
    double position[WHOLE_BODY_MAX_DOF];
    double velocity[WHOLE_BODY_MAX_DOF];
    double torque[WHOLE_BODY_MAX_DOF];
    double temperature[WHOLE_BODY_MAX_DOF];
    uint32_t motor_error[WHOLE_BODY_MAX_DOF];
    double base_quat[4];
    double gyro[3];
    double acceleration[3];
};

struct whole_body_health {
    enum whole_body_health_state state;
    int32_t last_error;
    uint64_t read_cycles;
    uint64_t write_cycles;
    uint64_t watchdog_events;
};

/**
 * @brief Latest physical-motor feedback and its mapping metadata.
 *
 * raw_* values are reported by the motor driver. calibrated_* values apply the
 * configured polarity and zero offset, but do not apply coupled-joint kinematics.
 */
struct whole_body_motor_diagnostic {
    char name[WHOLE_BODY_NAME_LENGTH];
    char joint_names[WHOLE_BODY_JOINT_LABEL_LENGTH];
    char driver[WHOLE_BODY_NAME_LENGTH];
    char model[WHOLE_BODY_NAME_LENGTH];
    char bus[WHOLE_BODY_NAME_LENGTH];
    char device[WHOLE_BODY_PATH_LENGTH];
    uint16_t command_id;
    uint16_t feedback_id;
    bool feedback_received;
    bool feedback_fresh;
    double feedback_age_s;
    double raw_position;
    double raw_velocity;
    double raw_torque;
    double calibrated_position;
    double calibrated_velocity;
    double calibrated_torque;
    double temperature;
    uint32_t error;
};

/** @brief Latest state after direct or coupled motor-to-joint mapping. */
struct whole_body_joint_diagnostic {
    char name[WHOLE_BODY_NAME_LENGTH];
    bool feedback_valid;
    double position;
    double velocity;
    double torque;
    double temperature;
    uint32_t motor_error;
};

/** @brief Latest body-frame IMU state and feedback freshness. */
struct whole_body_imu_diagnostic {
    char driver[WHOLE_BODY_NAME_LENGTH];
    char device[WHOLE_BODY_PATH_LENGTH];
    bool feedback_received;
    bool feedback_fresh;
    double feedback_age_s;
    double quaternion[4];
    double gyro[3];
    double acceleration[3];
};

/** @brief Read-only diagnostic snapshot; it never changes actuation state. */
struct whole_body_diagnostics {
    double timestamp_s;
    uint32_t motor_count;
    uint32_t joint_count;
    struct whole_body_motor_diagnostic motors[WHOLE_BODY_MAX_MOTORS];
    struct whole_body_joint_diagnostic joints[WHOLE_BODY_MAX_DOF];
    struct whole_body_imu_diagnostic imu;
    struct whole_body_health health;
};

struct whole_body_dev;

int whole_body_create(const char *main_config_path, struct whole_body_dev **out_dev);
int whole_body_init(struct whole_body_dev *dev);
int whole_body_read(struct whole_body_dev *dev, struct whole_body_state *state);
int whole_body_write(struct whole_body_dev *dev, const struct whole_body_joint_command *command);
int whole_body_tick(struct whole_body_dev *dev, double monotonic_time_s);
int whole_body_set_mode(struct whole_body_dev *dev, enum whole_body_mode mode);
int whole_body_get_health(const struct whole_body_dev *dev, struct whole_body_health *health);
int whole_body_get_diagnostics(
    const struct whole_body_dev *dev, struct whole_body_diagnostics *diagnostics);
int whole_body_get_cycle_s(const struct whole_body_dev *dev, double *cycle_s);
const char *whole_body_last_error(const struct whole_body_dev *dev);
void whole_body_destroy(struct whole_body_dev *dev);

#ifdef __cplusplus
}
#endif

#endif  // WHOLE_BODY_H

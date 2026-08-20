/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body_config.cpp
 * @brief Internal whole-body YAML configuration loader
 */

#include "whole_body_config.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "whole_body.h"

namespace whole_body {
namespace {

using KeySet = std::unordered_set<std::string>;

void CheckKeys(const YAML::Node &node, const KeySet &allowed, const std::string &path) {
    if (!node || !node.IsMap()) throw std::runtime_error(path + " must be a map");
    for (const auto &entry : node) {
        const std::string key = entry.first.as<std::string>();
        if (allowed.count(key) == 0)
            throw std::runtime_error(path + " contains unknown field '" + key + "'");
    }
}

template <typename T>
T Required(const YAML::Node &node, const char *key, const std::string &path) {
    if (!node[key]) throw std::runtime_error(path + "." + key + " is required");
    try {
        return node[key].as<T>();
    } catch (const YAML::Exception &error) {
        throw std::runtime_error(path + "." + key + ": " + error.what());
    }
}

void RequireFinite(double value, const std::string &path) {
    if (!std::isfinite(value)) throw std::runtime_error(path + " must be finite");
}

ImpedanceMode ReadImpedanceMode(const YAML::Node &node, const std::string &path) {
    const std::string mode = Required<std::string>(node, "mode", path);
    if (mode == "motor") return ImpedanceMode::kMotor;
    if (mode == "software") return ImpedanceMode::kSoftware;
    if (mode == "split") return ImpedanceMode::kSplit;
    throw std::runtime_error(path + ".mode must be motor, software, or split");
}

std::array<double, 2> ReadPair(const YAML::Node &node, const std::string &path) {
    if (!node || !node.IsSequence() || node.size() != 2)
        throw std::runtime_error(path + " must contain exactly 2 numbers");
    std::array<double, 2> result = {node[0].as<double>(), node[1].as<double>()};
    RequireFinite(result[0], path + "[0]");
    RequireFinite(result[1], path + "[1]");
    return result;
}

Vec3 ReadVec3(const YAML::Node &node, const std::string &path) {
    if (!node || !node.IsSequence() || node.size() != 3)
        throw std::runtime_error(path + " must contain exactly 3 numbers");
    Vec3 result{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
    RequireFinite(result.x, path + "[0]");
    RequireFinite(result.y, path + "[1]");
    RequireFinite(result.z, path + "[2]");
    return result;
}

template <size_t Size>
std::array<std::string, Size> ReadStringArray(const YAML::Node &node, const std::string &path) {
    if (!node || !node.IsSequence() || node.size() != Size)
        throw std::runtime_error(path + " has an invalid length");
    std::array<std::string, Size> result;
    for (size_t i = 0; i < Size; ++i) result[i] = node[i].as<std::string>();
    return result;
}

std::string ResolvePath(const std::string &base, const std::string &path) {
    if (path.empty()) throw std::runtime_error("configuration path must not be empty");
    const std::string candidate = path.front() == '/' ? path : base + "/" + path;
    std::array<char, PATH_MAX> resolved{};
    if (!realpath(candidate.c_str(), resolved.data()))
        throw std::runtime_error("failed to resolve configuration path '" + candidate + "'");
    return resolved.data();
}

std::string ParentPath(const std::string &path) {
    const size_t separator = path.find_last_of('/');
    if (separator == std::string::npos) return ".";
    return separator == 0 ? "/" : path.substr(0, separator);
}

void FlattenDriverOptions(const YAML::Node &node, const std::string &path,
    const std::string &prefix, std::vector<DriverOption> *options) {
    if (!options) throw std::runtime_error(path + " has no output destination");
    if (node.IsScalar()) {
        if (prefix.empty()) throw std::runtime_error(path + " contains an unnamed value");
        options->push_back({prefix, node.Scalar()});
        return;
    }
    if (node.IsSequence()) {
        if (node.size() == 0) throw std::runtime_error(path + " must not be empty");
        for (size_t i = 0; i < node.size(); ++i) {
            FlattenDriverOptions(node[i], path + "[" + std::to_string(i) + "]",
                prefix + "." + std::to_string(i), options);
        }
        return;
    }
    if (!node.IsMap()) throw std::runtime_error(path + " must contain scalar configuration");
    for (const auto &entry : node) {
        if (!entry.first.IsScalar()) throw std::runtime_error(path + " contains a non-scalar key");
        const std::string key = entry.first.Scalar();
        if (key.empty() || key.find('.') != std::string::npos)
            throw std::runtime_error(path + " contains an invalid option name");
        const std::string name = prefix.empty() ? key : prefix + "." + key;
        FlattenDriverOptions(entry.second, path + "." + key, name, options);
    }
}

void ValidateRotation(const std::array<float, 9> &rotation) {
    constexpr double kTolerance = 1.0e-3;
    for (int row = 0; row < 3; ++row) {
        for (int other = row; other < 3; ++other) {
            double dot = 0.0;
            for (int column = 0; column < 3; ++column)
                dot += rotation[row * 3 + column] * rotation[other * 3 + column];
            const double expected = row == other ? 1.0 : 0.0;
            if (std::abs(dot - expected) > kTolerance)
                throw std::runtime_error("whole_body.imu.mounting_matrix is not orthonormal");
        }
    }
    const double determinant =
        rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7]) -
        rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6]) +
        rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6]);
    if (std::abs(determinant - 1.0) > kTolerance)
        throw std::runtime_error("whole_body.imu.mounting_matrix must have determinant 1");
}

void ValidateReferences(const RuntimeConfig &config) {
    std::unordered_set<std::string> buses;
    std::unordered_set<std::string> motors;
    std::unordered_map<std::string, const JointConfig *> joints;
    std::unordered_set<std::string> coupled_joints;
    std::unordered_set<std::string> coupled_motors;
    std::unordered_set<std::string> assigned_motors;

    if (config.robot_name.empty()) throw std::runtime_error("robot_base.name must not be empty");
    for (const auto &joint_name : config.joint_names) {
        if (joint_name.empty())
            throw std::runtime_error("robot_base.joint_names contains an empty name");
    }

    for (const auto &bus : config.buses) {
        if (!buses.insert(bus.name).second)
            throw std::runtime_error("duplicate bus name '" + bus.name + "'");
    }
    for (const auto &motor : config.motors) {
        if (!motors.insert(motor.name).second)
            throw std::runtime_error("duplicate motor name '" + motor.name + "'");
        if (buses.count(motor.bus) == 0)
            throw std::runtime_error("motor '" + motor.name + "' references unknown bus");
    }
    std::set<std::pair<std::string, uint16_t>> command_endpoints;
    std::set<std::pair<std::string, uint16_t>> feedback_endpoints;
    for (const auto &motor : config.motors) {
        if (!command_endpoints.emplace(motor.bus, motor.command_id).second ||
            !feedback_endpoints.emplace(motor.bus, motor.feedback_id).second) {
            throw std::runtime_error("duplicate motor CAN ID on bus '" + motor.bus + "'");
        }
    }
    for (size_t i = 0; i < config.joints.size(); ++i) {
        const auto &joint = config.joints[i];
        if (joint.name != config.joint_names[i])
            throw std::runtime_error("whole_body.joints order differs from robot_base.joint_names");
        if (!joints.emplace(joint.name, &joint).second)
            throw std::runtime_error("duplicate joint name '" + joint.name + "'");
        for (const auto &motor : joint.motors) {
            if (motors.count(motor) == 0)
                throw std::runtime_error("joint '" + joint.name + "' references unknown motor");
        }
        if (joint.mapping == "direct" && joint.motors.size() != 1)
            throw std::runtime_error("direct joint '" + joint.name + "' requires one motor");
        if (joint.mapping == "direct" && !assigned_motors.insert(joint.motors[0]).second)
            throw std::runtime_error("motor '" + joint.motors[0] + "' is assigned more than once");
        if (joint.mapping == "parallel_ankle" && joint.motors.size() != 2)
            throw std::runtime_error(
                "parallel_ankle joint '" + joint.name + "' requires two motors");
        if (joint.mapping == "parallel_ankle" &&
            joint.impedance.mode != ImpedanceMode::kSoftware) {
            throw std::runtime_error(
                "parallel_ankle joint '" + joint.name + "' requires software impedance");
        }
        if (joint.impedance.mode == ImpedanceMode::kSplit && joint.mapping != "direct") {
            throw std::runtime_error(
                "split impedance is only supported for direct joint mappings");
        }
    }
    for (const auto &coupling : config.couplings) {
        for (const auto &joint : coupling.joints) {
            const auto found = joints.find(joint);
            if (found == joints.end() || found->second->mapping != "parallel_ankle")
                throw std::runtime_error("coupling references an invalid joint '" + joint + "'");
            if (!coupled_joints.insert(joint).second)
                throw std::runtime_error("joint '" + joint + "' belongs to multiple couplings");
        }
        for (const auto &motor : coupling.motors) {
            if (motors.count(motor) == 0)
                throw std::runtime_error("coupling references unknown motor '" + motor + "'");
            if (!coupled_motors.insert(motor).second)
                throw std::runtime_error("motor '" + motor + "' belongs to multiple couplings");
            if (!assigned_motors.insert(motor).second)
                throw std::runtime_error("motor '" + motor + "' is assigned more than once");
        }
        for (const auto &joint_name : coupling.joints) {
            const auto &joint_motors = joints.at(joint_name)->motors;
            const std::set<std::string> expected(coupling.motors.begin(), coupling.motors.end());
            const std::set<std::string> actual(joint_motors.begin(), joint_motors.end());
            if (actual != expected)
                throw std::runtime_error("coupling motor list differs from joint mapping");
        }
    }
    for (const auto &joint : config.joints) {
        if (joint.mapping == "parallel_ankle" && coupled_joints.count(joint.name) == 0)
            throw std::runtime_error("parallel ankle joint is missing a coupling");
    }
    if (assigned_motors.size() != config.motors.size())
        throw std::runtime_error("one or more motors are not assigned to a joint mapping");
}

}  // namespace

RuntimeConfig LoadConfig(const std::string &main_config_path) {
    RuntimeConfig config;
    YAML::Node main;
    YAML::Node hardware;

    try {
        const std::string absolute_main = ResolvePath(".", main_config_path);
        main = YAML::LoadFile(absolute_main);
        config.main_config_path = absolute_main;
        const YAML::Node robot = main["robot_base"];
        config.robot_name = Required<std::string>(robot, "name", "robot_base");
        config.num_dof = Required<uint32_t>(robot, "num_dof", "robot_base");
        config.joint_names = Required<std::vector<std::string>>(robot, "joint_names", "robot_base");
        if (config.num_dof == 0 || config.num_dof > 64 ||
            config.joint_names.size() != config.num_dof) {
            throw std::runtime_error("robot_base num_dof/joint_names mismatch");
        }
        std::set<std::string> unique_joints(config.joint_names.begin(), config.joint_names.end());
        if (unique_joints.size() != config.joint_names.size())
            throw std::runtime_error("robot_base.joint_names contains duplicates");

        const YAML::Node selector = main["whole_body"];
        CheckKeys(selector, {"config_file"}, "whole_body selector");
        const std::string hardware_path = ResolvePath(ParentPath(absolute_main),
            Required<std::string>(selector, "config_file", "whole_body selector"));
        config.hardware_config_path = hardware_path;
        hardware = YAML::LoadFile(hardware_path);
    } catch (const YAML::Exception &error) {
        throw std::runtime_error(std::string("failed to load whole-body YAML: ") + error.what());
    }

    const YAML::Node root = hardware["whole_body"];
    CheckKeys(root,
        {"cycle_s", "startup_feedback_timeout_s", "feedback_timeout_s",
            "command_timeout_s", "startup_mode", "allow_actuation", "buses", "motors",
            "joints", "couplings", "imu"},
        "whole_body");
    config.cycle_s = Required<double>(root, "cycle_s", "whole_body");
    config.startup_feedback_timeout_s =
        Required<double>(root, "startup_feedback_timeout_s", "whole_body");
    config.feedback_timeout_s = Required<double>(root, "feedback_timeout_s", "whole_body");
    config.command_timeout_s = Required<double>(root, "command_timeout_s", "whole_body");
    RequireFinite(config.cycle_s, "whole_body.cycle_s");
    RequireFinite(
        config.startup_feedback_timeout_s, "whole_body.startup_feedback_timeout_s");
    RequireFinite(config.feedback_timeout_s, "whole_body.feedback_timeout_s");
    RequireFinite(config.command_timeout_s, "whole_body.command_timeout_s");
    const double cycle_us = config.cycle_s * 1.0e6;
    if (config.cycle_s <= 0.0 || config.startup_feedback_timeout_s <= config.feedback_timeout_s ||
        config.feedback_timeout_s <= config.cycle_s ||
        config.command_timeout_s <= config.cycle_s || cycle_us < 1.0 ||
        cycle_us > static_cast<double>(std::numeric_limits<uint32_t>::max()))
        throw std::runtime_error("whole_body timing values are invalid");
    const std::string startup = Required<std::string>(root, "startup_mode", "whole_body");
    config.allow_actuation = Required<bool>(root, "allow_actuation", "whole_body");
    if (startup == "read_only") {
        config.read_only = true;
        if (config.allow_actuation) {
            throw std::runtime_error(
                "whole_body.allow_actuation must be false in read_only mode");
        }
    } else if (startup == "disabled") {
        config.read_only = false;
        if (!config.allow_actuation) {
            throw std::runtime_error(
                "whole_body.allow_actuation must be true in disabled mode");
        }
    } else {
        throw std::runtime_error(
            "whole_body.startup_mode must be read_only or disabled");
    }

    const YAML::Node buses = root["buses"];
    if (!buses || !buses.IsSequence() || buses.size() == 0)
        throw std::runtime_error("whole_body.buses must not be empty");
    for (size_t i = 0; i < buses.size(); ++i) {
        const std::string path = "whole_body.buses[" + std::to_string(i) + "]";
        CheckKeys(buses[i], {"name", "type", "device", "bitrate"}, path);
        BusConfig bus;
        bus.name = Required<std::string>(buses[i], "name", path);
        bus.type = Required<std::string>(buses[i], "type", path);
        bus.device = Required<std::string>(buses[i], "device", path);
        bus.bitrate = Required<uint32_t>(buses[i], "bitrate", path);
        if (bus.name.empty() || bus.device.empty() || bus.type != "socketcan" || bus.bitrate == 0)
            throw std::runtime_error(path + " is invalid");
        config.buses.push_back(std::move(bus));
    }

    const YAML::Node motors = root["motors"];
    if (!motors || !motors.IsSequence() || motors.size() == 0 ||
        motors.size() > WHOLE_BODY_MAX_MOTORS) {
        throw std::runtime_error("whole_body.motors count is outside the supported range");
    }
    for (size_t i = 0; i < motors.size(); ++i) {
        const std::string path = "whole_body.motors[" + std::to_string(i) + "]";
        CheckKeys(motors[i],
            {"name", "driver", "bus", "model", "command_id", "feedback_id", "polarity",
                "zero_offset", "driver_options"},
            path);
        MotorConfig motor;
        motor.name = Required<std::string>(motors[i], "name", path);
        motor.driver = Required<std::string>(motors[i], "driver", path);
        motor.bus = Required<std::string>(motors[i], "bus", path);
        motor.model = Required<std::string>(motors[i], "model", path);
        motor.command_id = Required<uint16_t>(motors[i], "command_id", path);
        motor.feedback_id = Required<uint16_t>(motors[i], "feedback_id", path);
        motor.polarity = Required<double>(motors[i], "polarity", path);
        motor.zero_offset = Required<double>(motors[i], "zero_offset", path);
        const YAML::Node driver_options = motors[i]["driver_options"];
        if (driver_options) {
            if (!driver_options.IsMap())
                throw std::runtime_error(path + ".driver_options must be a map");
            FlattenDriverOptions(
                driver_options, path + ".driver_options", "", &motor.driver_options);
        }
        std::unordered_set<std::string> option_names;
        for (const auto &option : motor.driver_options) {
            if (option.name == "model" || option.name == "feedback_id" ||
                !option_names.insert(option.name).second)
                throw std::runtime_error(path + ".driver_options contains a duplicate field");
        }
        RequireFinite(motor.zero_offset, path + ".zero_offset");
        if (motor.name.empty() || motor.driver.empty() || motor.model.empty() ||
            (motor.polarity != 1.0 && motor.polarity != -1.0) || motor.command_id > 0x7ff ||
            motor.feedback_id > 0x7ff)
            throw std::runtime_error(path + " is invalid");
        config.motors.push_back(std::move(motor));
    }

    const YAML::Node joints = root["joints"];
    if (!joints || !joints.IsSequence() || joints.size() != config.num_dof)
        throw std::runtime_error("whole_body.joints count must equal robot_base.num_dof");
    for (size_t i = 0; i < joints.size(); ++i) {
        const std::string path = "whole_body.joints[" + std::to_string(i) + "]";
        CheckKeys(joints[i],
            {"name", "mapping", "motors", "position_limit", "velocity_limit", "torque_limit",
                "impedance"},
            path);
        JointConfig joint;
        joint.name = Required<std::string>(joints[i], "name", path);
        joint.mapping = Required<std::string>(joints[i], "mapping", path);
        joint.motors = Required<std::vector<std::string>>(joints[i], "motors", path);
        joint.position_limit = ReadPair(joints[i]["position_limit"], path + ".position_limit");
        joint.velocity_limit = Required<double>(joints[i], "velocity_limit", path);
        joint.torque_limit = Required<double>(joints[i], "torque_limit", path);
        joint.impedance.mode = joint.mapping == "parallel_ankle"
            ? ImpedanceMode::kSoftware
            : ImpedanceMode::kMotor;
        const YAML::Node impedance = joints[i]["impedance"];
        if (impedance) {
            const std::string impedance_path = path + ".impedance";
            CheckKeys(impedance, {"mode", "motor_kp_max", "motor_kd_max"}, impedance_path);
            joint.impedance.mode = ReadImpedanceMode(impedance, impedance_path);
            if (joint.impedance.mode == ImpedanceMode::kSplit) {
                joint.impedance.motor_kp_max =
                    Required<double>(impedance, "motor_kp_max", impedance_path);
                joint.impedance.motor_kd_max =
                    Required<double>(impedance, "motor_kd_max", impedance_path);
                RequireFinite(
                    joint.impedance.motor_kp_max, impedance_path + ".motor_kp_max");
                RequireFinite(
                    joint.impedance.motor_kd_max, impedance_path + ".motor_kd_max");
                if (joint.impedance.motor_kp_max < 0.0 ||
                    joint.impedance.motor_kd_max < 0.0) {
                    throw std::runtime_error(
                        impedance_path + " motor gain limits must be non-negative");
                }
            } else if (impedance["motor_kp_max"] || impedance["motor_kd_max"]) {
                throw std::runtime_error(
                    impedance_path + " motor gain limits are only valid in split mode");
            }
        }
        RequireFinite(joint.velocity_limit, path + ".velocity_limit");
        RequireFinite(joint.torque_limit, path + ".torque_limit");
        if ((joint.mapping != "direct" && joint.mapping != "parallel_ankle") ||
            joint.position_limit[0] >= joint.position_limit[1] || joint.velocity_limit <= 0.0 ||
            joint.torque_limit <= 0.0)
            throw std::runtime_error(path + " is invalid");
        config.joints.push_back(std::move(joint));
    }

    const YAML::Node couplings = root["couplings"];
    if (couplings && !couplings.IsSequence())
        throw std::runtime_error("whole_body.couplings must be a sequence");
    for (size_t i = 0; couplings && i < couplings.size(); ++i) {
        const std::string path = "whole_body.couplings[" + std::to_string(i) + "]";
        CheckKeys(couplings[i],
            {"type", "joints", "motors", "ankle_pivots", "ball_positions", "motor_positions",
                "motor_bias", "motor_limit", "ankle_limit", "motor_difference_limit",
                "max_iterations", "squared_tolerance"},
            path);
        if (Required<std::string>(couplings[i], "type", path) != "parallel_ankle")
            throw std::runtime_error(path + ".type is unsupported");
        ParallelAnkleConfig coupling;
        coupling.joints = ReadStringArray<2>(couplings[i]["joints"], path + ".joints");
        coupling.motors = ReadStringArray<2>(couplings[i]["motors"], path + ".motors");
        const YAML::Node pivots = couplings[i]["ankle_pivots"];
        const YAML::Node balls = couplings[i]["ball_positions"];
        const YAML::Node motor_positions = couplings[i]["motor_positions"];
        if (!pivots || pivots.size() != 2 || !balls || balls.size() != 2 || !motor_positions ||
            motor_positions.size() != 2)
            throw std::runtime_error(path + " geometry must contain two sides");
        for (size_t side = 0; side < 2; ++side) {
            coupling.ankle_pivots[side] = ReadVec3(pivots[side], path + ".ankle_pivots");
            coupling.ball_positions[side] = ReadVec3(balls[side], path + ".ball_positions");
            coupling.motor_positions[side] =
                ReadVec3(motor_positions[side], path + ".motor_positions");
        }
        coupling.motor_bias = ReadPair(couplings[i]["motor_bias"], path + ".motor_bias");
        coupling.motor_limit = Required<double>(couplings[i], "motor_limit", path);
        coupling.ankle_limit = Required<double>(couplings[i], "ankle_limit", path);
        coupling.motor_difference_limit =
            Required<double>(couplings[i], "motor_difference_limit", path);
        coupling.max_iterations = Required<int>(couplings[i], "max_iterations", path);
        coupling.squared_tolerance = Required<double>(couplings[i], "squared_tolerance", path);
        RequireFinite(coupling.motor_limit, path + ".motor_limit");
        RequireFinite(coupling.ankle_limit, path + ".ankle_limit");
        RequireFinite(coupling.motor_difference_limit, path + ".motor_difference_limit");
        RequireFinite(coupling.squared_tolerance, path + ".squared_tolerance");
        if (coupling.motor_limit <= 0.0 || coupling.ankle_limit <= 0.0 ||
            coupling.motor_difference_limit <= 0.0 || coupling.max_iterations <= 0 ||
            coupling.squared_tolerance <= 0.0)
            throw std::runtime_error(path + " solver limits are invalid");
        config.couplings.push_back(std::move(coupling));
    }

    const YAML::Node imu = root["imu"];
    CheckKeys(imu,
        {"driver", "device", "baud", "mounting_matrix", "acceleration_bias", "gyro_bias"},
        "whole_body.imu");
    config.imu.driver = Required<std::string>(imu, "driver", "whole_body.imu");
    config.imu.device = Required<std::string>(imu, "device", "whole_body.imu");
    config.imu.baud = Required<uint32_t>(imu, "baud", "whole_body.imu");
    const auto mounting = Required<std::vector<float>>(imu, "mounting_matrix", "whole_body.imu");
    const auto acceleration_bias =
        Required<std::vector<float>>(imu, "acceleration_bias", "whole_body.imu");
    const auto gyro_bias = Required<std::vector<float>>(imu, "gyro_bias", "whole_body.imu");
    if (mounting.size() != 9 || acceleration_bias.size() != 3 || gyro_bias.size() != 3 ||
        config.imu.driver.empty() || config.imu.device.empty() || config.imu.baud == 0)
        throw std::runtime_error("whole_body.imu has invalid dimensions or values");
    std::copy(mounting.begin(), mounting.end(), config.imu.mounting_matrix.begin());
    std::copy(
        acceleration_bias.begin(), acceleration_bias.end(), config.imu.acceleration_bias.begin());
    std::copy(gyro_bias.begin(), gyro_bias.end(), config.imu.gyro_bias.begin());
    for (size_t i = 0; i < config.imu.mounting_matrix.size(); ++i)
        RequireFinite(config.imu.mounting_matrix[i], "whole_body.imu.mounting_matrix");
    for (size_t i = 0; i < config.imu.acceleration_bias.size(); ++i) {
        RequireFinite(config.imu.acceleration_bias[i], "whole_body.imu.acceleration_bias");
        RequireFinite(config.imu.gyro_bias[i], "whole_body.imu.gyro_bias");
    }
    ValidateRotation(config.imu.mounting_matrix);
    ValidateReferences(config);
    return config;
}

}  // namespace whole_body

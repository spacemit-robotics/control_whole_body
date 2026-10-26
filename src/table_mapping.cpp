/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file table_mapping.cpp
 * @brief Bilinear or Delaunay-plane mapping; no extrapolation or NaN reweighting.
 */
#include "table_mapping.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include <delaunator-cpp.hpp>

namespace whole_body {
namespace {

constexpr size_t kMaxTablePoints = 1000000;

double ParseNumber(const std::string &text) {
    char *end = nullptr;
    const char *begin = text.c_str();
    const double value = std::strtod(begin, &end);
    if (end == begin) throw std::runtime_error("invalid table number");
    while (*end == ' ' || *end == '\t' || *end == '\r') ++end;
    if (*end != '\0' || std::isinf(value))
        throw std::runtime_error("invalid table number");
    return value;
}

std::array<double, 4> ReadRow(const std::string &line) {
    std::array<double, 4> row;
    std::istringstream stream(line);
    std::string field;
    for (double &value : row) {
        if (!std::getline(stream, field, ','))
            throw std::runtime_error("table CSV requires four columns");
        value = ParseNumber(field);
    }
    if (stream.peek() != std::char_traits<char>::eof() || line.back() == ',')
        throw std::runtime_error("table CSV requires four columns");
    if (!std::isfinite(row[0]) || !std::isfinite(row[1]))
        throw std::runtime_error("non-finite table input");
    return row;
}

double GridStep(const std::vector<double> &axis) {
    if (axis.size() < 2 || axis.size() > 4096)
        throw std::runtime_error("invalid table dimensions");
    const double step = (axis.back() - axis.front()) / (axis.size() - 1);
    if (!std::isfinite(step) || step <= 0.0)
        throw std::runtime_error("invalid table spacing");
    for (size_t i = 0; i < axis.size(); ++i) {
        if (std::abs(axis[i] - (axis.front() + i * step)) > step * 1.0e-6)
            throw std::runtime_error("table inputs must form a regular grid");
    }
    return step;
}

bool FinitePair(const std::array<double, 2> &value) {
    return std::isfinite(value[0]) && std::isfinite(value[1]);
}

double Error(const std::array<double, 2> &lhs, const std::array<double, 2> &rhs) {
    return (lhs[0] - rhs[0]) * (lhs[0] - rhs[0]) + (lhs[1] - rhs[1]) * (lhs[1] - rhs[1]);
}

bool Invert(const std::array<double, 4> &matrix, std::array<double, 4> *inverse) {
    const double det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
    if (!std::isfinite(det) || std::abs(det) < 1.0e-10) return false;
    *inverse = {matrix[3] / det, -matrix[1] / det, -matrix[2] / det, matrix[0] / det};
    return true;
}

}  // namespace

TableMapping::TableMapping(const CouplingConfig &config) : config_(config) {
    if (config.table_interpolation != "bilinear" && config.table_interpolation != "delaunay")
        throw std::runtime_error("invalid table interpolation");
    if (config.table_direction != "joint_to_motor" && config.table_direction != "motor_to_joint")
        throw std::runtime_error("invalid table direction");
    motor_to_joint_ = config.table_direction == "motor_to_joint";
    for (double scale : config.table_output_scale) {
        if (!std::isfinite(scale) || scale == 0.0)
            throw std::runtime_error("invalid table output scale");
    }
    std::ifstream stream(config.table_file);
    if (!stream) throw std::runtime_error("cannot open table: " + config.table_file);
    std::vector<std::array<double, 4>> rows;
    std::set<double> inputs_x, inputs_y;
    std::string line;
    while (std::getline(stream, line)) {
        if (line.empty() || line == "\r") continue;
        if (line.size() > 4096 || rows.size() >= kMaxTablePoints)
            throw std::runtime_error("table exceeds supported size");
        auto row = ReadRow(line);
        for (size_t side = 0; side < 2; ++side) {
            row[side + 2] *= config.table_output_scale[side];
            if (std::isinf(row[side + 2])) throw std::runtime_error("table output overflow");
        }
        rows.push_back(row);
        inputs_x.insert(row[0]);
        inputs_y.insert(row[1]);
    }
    if (!stream.eof()) throw std::runtime_error("failed to read table: " + config.table_file);
    const std::vector<double> xs(inputs_x.begin(), inputs_x.end());
    const std::vector<double> ys(inputs_y.begin(), inputs_y.end());
    step_x_ = GridStep(xs);
    step_y_ = GridStep(ys);
    nx_ = xs.size();
    ny_ = ys.size();
    if (nx_ * ny_ > kMaxTablePoints)
        throw std::runtime_error("table exceeds supported grid size");
    x0_ = xs.front();
    y0_ = ys.front();
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    values_.assign(nx_ * ny_, {invalid, invalid});
    std::vector<bool> seen(nx_ * ny_, false);
    for (const auto &row : rows) {
        const size_t ix = std::lower_bound(xs.begin(), xs.end(), row[0]) - xs.begin();
        const size_t iy = std::lower_bound(ys.begin(), ys.end(), row[1]) - ys.begin();
        const size_t index = ix * ny_ + iy;
        if (seen[index]) throw std::runtime_error("duplicate table input");
        seen[index] = true;
        values_[index] = {row[2], row[3]};
    }
    if (config.table_interpolation == "delaunay") {
        // Preserve CSV point order: co-circular grid points can admit either diagonal.
        std::vector<double> coordinates;
        std::vector<std::array<size_t, 2>> grid_indices;
        coordinates.reserve(rows.size() * 2);
        grid_indices.reserve(rows.size());
        for (const auto &row : rows) {
            coordinates.push_back(row[0]);
            coordinates.push_back(row[1]);
            grid_indices.push_back({static_cast<size_t>(std::lower_bound(xs.begin(), xs.end(), row[0]) - xs.begin()),
                static_cast<size_t>(std::lower_bound(ys.begin(), ys.end(), row[1]) - ys.begin())});
        }
        const delaunator::Delaunator<double> triangulation(coordinates);
        diagonals_.assign((nx_ - 1) * (ny_ - 1), 0);
        for (size_t i = 0; i + 2 < triangulation.triangles.size(); i += 3) {
            const auto a = grid_indices[triangulation.triangles[i]];
            const auto b = grid_indices[triangulation.triangles[i + 1]];
            const auto c = grid_indices[triangulation.triangles[i + 2]];
            const size_t lx = std::min({a[0], b[0], c[0]});
            const size_t ly = std::min({a[1], b[1], c[1]});
            const size_t hx = std::max({a[0], b[0], c[0]});
            const size_t hy = std::max({a[1], b[1], c[1]});
            // Do not bridge missing grid samples with large Delaunay triangles.
            if (hx != lx + 1 || hy != ly + 1) continue;
            unsigned corners = 0;
            for (const auto &point : {a, b, c})
                corners |= 1U << ((point[0] - lx) * 2 + point[1] - ly);
            diagonals_[lx * (ny_ - 1) + ly] = (corners & 9U) == 9U ? 1 : 2;
        }
    }
    if (config.max_iterations <= 0 || !std::isfinite(config.squared_tolerance) ||
        config.squared_tolerance <= 0.0)
        throw std::runtime_error("invalid table solver limits");
}

bool TableMapping::Lookup(const std::array<double, 2> &joint, std::array<double, 2> *motor,
    std::array<double, 4> *jacobian) const {
    const double input_limit = motor_to_joint_ ? config_.motor_limit : config_.ankle_limit;
    if (!motor || !FinitePair(joint) || std::abs(joint[0]) > input_limit ||
        std::abs(joint[1]) > input_limit ||
        (motor_to_joint_ && std::abs(joint[0] - joint[1]) > config_.motor_difference_limit) ||
        joint[0] < x0_ || joint[1] < y0_ ||
        joint[0] > x0_ + step_x_ * (nx_ - 1) || joint[1] > y0_ + step_y_ * (ny_ - 1))
        return false;
    const double gx = (joint[0] - x0_) / step_x_;
    const double gy = (joint[1] - y0_) / step_y_;
    const size_t ix = std::min(static_cast<size_t>(gx), nx_ - 2);
    const size_t iy = std::min(static_cast<size_t>(gy), ny_ - 2);
    const double u = gx - ix;
    const double v = gy - iy;
    const auto &a = values_[ix * ny_ + iy];
    const auto &b = values_[(ix + 1) * ny_ + iy];
    const auto &c = values_[ix * ny_ + iy + 1];
    const auto &d = values_[(ix + 1) * ny_ + iy + 1];
    // A cell, including its derivative, needs all four finite corners, even at an edge.
    if (!FinitePair(a) || !FinitePair(b) || !FinitePair(c) || !FinitePair(d)) return false;
    std::array<double, 2> result;
    std::array<double, 4> derivative;
    for (size_t side = 0; side < 2; ++side) {
        if (diagonals_.empty()) {
            result[side] = (1 - u) * (1 - v) * a[side] + u * (1 - v) * b[side] +
                (1 - u) * v * c[side] + u * v * d[side];
            derivative[side * 2] = ((1 - v) * (b[side] - a[side]) + v * (d[side] - c[side])) / step_x_;
            derivative[side * 2 + 1] = ((1 - u) * (c[side] - a[side]) + u * (d[side] - b[side])) / step_y_;
        } else {
            const auto diagonal = diagonals_[ix * (ny_ - 1) + iy];
            if (diagonal == 0) return false;
            double dx, dy;
            if (diagonal == 1) {
                dx = u >= v ? b[side] - a[side] : d[side] - c[side];
                dy = u >= v ? d[side] - b[side] : c[side] - a[side];
                result[side] = a[side] + u * dx + v * dy;
            } else {
                dx = u + v <= 1 ? b[side] - a[side] : d[side] - c[side];
                dy = u + v <= 1 ? c[side] - a[side] : d[side] - b[side];
                result[side] = u + v <= 1 ? a[side] + u * dx + v * dy :
                    d[side] + (u - 1) * dx + (v - 1) * dy;
            }
            derivative[side * 2] = dx / step_x_;
            derivative[side * 2 + 1] = dy / step_y_;
        }
    }
    const double output_limit = motor_to_joint_ ? config_.ankle_limit : config_.motor_limit;
    if (!FinitePair(result) || std::abs(result[0]) > output_limit ||
        std::abs(result[1]) > output_limit ||
        (!motor_to_joint_ && std::abs(result[0] - result[1]) > config_.motor_difference_limit))
        return false;
    for (double value : derivative) if (!std::isfinite(value)) return false;
    *motor = result;
    if (jacobian) *jacobian = derivative;
    return true;
}

bool TableMapping::JointToMotor(const std::array<double, 2> &joint,
    std::array<double, 2> *motor, std::array<double, 4> *jacobian, bool allow_global_search) const {
    if (!motor_to_joint_) return Lookup(joint, motor, jacobian);
    std::array<double, 4> derivative;
    std::array<double, 4> inverse;
    if (!InverseLookup(joint, motor, &derivative, allow_global_search) ||
        !Invert(derivative, &inverse)) return false;
    if (jacobian) *jacobian = inverse;
    return true;
}

bool TableMapping::Solve(const std::array<double, 2> &motor, std::array<double, 2> seed,
    std::array<double, 2> *joint, std::array<double, 4> *jacobian) const {
    for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
        std::array<double, 2> estimate;
        std::array<double, 4> derivative;
        if (!Lookup(seed, &estimate, &derivative)) return false;
        const double determinant = derivative[0] * derivative[3] - derivative[1] * derivative[2];
        if (!std::isfinite(determinant) || std::abs(determinant) < 1.0e-10) return false;
        const double error = Error(estimate, motor);
        if (error <= config_.squared_tolerance) {
            *joint = seed;
            if (jacobian) *jacobian = derivative;
            return true;
        }
        const double e0 = estimate[0] - motor[0];
        const double e1 = estimate[1] - motor[1];
        const std::array<double, 2> correction{
            (derivative[3] * e0 - derivative[1] * e1) / determinant,
            (-derivative[2] * e0 + derivative[0] * e1) / determinant};
        const double max_step = std::max(std::abs(correction[0]), std::abs(correction[1]));
        double scale = max_step > 0.1 ? 0.1 / max_step : 1.0;
        bool advanced = false;
        for (size_t attempt = 0; attempt < 16; ++attempt, scale *= 0.5) {
            const std::array<double, 2> trial{
                seed[0] - scale * correction[0], seed[1] - scale * correction[1]};
            if (Lookup(trial, &estimate, nullptr) && Error(estimate, motor) < error) {
                seed = trial;
                advanced = true;
                break;
            }
        }
        if (!advanced) return false;
    }
    return false;
}

bool TableMapping::MotorToJoint(const std::array<double, 2> &motor,
    std::array<double, 2> *joint, std::array<double, 4> *jacobian) const {
    if (!motor_to_joint_) return InverseLookup(motor, joint, jacobian);
    std::array<double, 4> derivative;
    std::array<double, 4> inverse;
    if (!Lookup(motor, joint, &derivative) || !Invert(derivative, &inverse)) return false;
    last_joint_ = motor;
    has_last_joint_ = true;
    if (jacobian) *jacobian = inverse;
    return true;
}

bool TableMapping::InverseLookup(const std::array<double, 2> &motor,
    std::array<double, 2> *joint, std::array<double, 4> *jacobian, bool allow_global_search) const {
    const double output_limit = motor_to_joint_ ? config_.ankle_limit : config_.motor_limit;
    if (!joint || !FinitePair(motor) || std::abs(motor[0]) > output_limit ||
        std::abs(motor[1]) > output_limit ||
        (!motor_to_joint_ && std::abs(motor[0] - motor[1]) > config_.motor_difference_limit))
        return false;
    bool solved = has_last_joint_ && Solve(motor, last_joint_, joint, jacobian);
    if (!solved) solved = Solve(motor, {0.0, 0.0}, joint, jacobian);
    // Recovery seed search is only used if the cached and zero seeds fail.
    // Sample cell centers (rather than NaN boundary vertices), with strict Lookup.
    if (!solved && allow_global_search) {
        double best_error = std::numeric_limits<double>::infinity();
        std::array<double, 2> best{};
        for (size_t ix = 0; ix + 1 < nx_; ++ix) {
            for (size_t iy = 0; iy + 1 < ny_; ++iy) {
                const std::array<double, 2> seed{x0_ + (ix + 0.5) * step_x_, y0_ + (iy + 0.5) * step_y_};
                std::array<double, 2> estimate;
                if (Lookup(seed, &estimate, nullptr) && Error(estimate, motor) < best_error) {
                    best_error = Error(estimate, motor);
                    best = seed;
                }
            }
        }
        if (std::isfinite(best_error)) solved = Solve(motor, best, joint, jacobian);
    }
    // A command-target check must neither scan the entire asset in the control
    // cycle nor displace the feedback seed used for velocity/torque transforms.
    if (solved && allow_global_search) {
        last_joint_ = *joint;
        has_last_joint_ = true;
    }
    return solved;
}

}  // namespace whole_body

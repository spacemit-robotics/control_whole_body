/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file triangle_table.cpp
 * @brief Load calibrated topology once, without platform-dependent triangulation.
 */
#include "mapping/triangle_table.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace whole_body {
namespace {

constexpr uint32_t kMissing = std::numeric_limits<uint32_t>::max();

}  // namespace

TriangleTable::TriangleTable(const std::string &path,
    const std::vector<std::array<double, 4>> &rows, const std::vector<double> &xs,
    const std::vector<double> &ys) : rows_(rows), xs_(xs), ys_(ys) {
    std::ifstream stream(path);
    std::string line, format, extra;
    size_t row_count = 0, count = 0;
    if (!std::getline(stream, line)) throw std::runtime_error("cannot read triangle table: " + path);
    std::istringstream header(line);
    if (!(header >> format >> row_count >> count) || (header >> extra) ||
        format != "ordered_triangles_v1" || row_count != rows.size() || count == 0 ||
        xs.size() < 2 || ys.size() < 2 || count > 2 * (xs.size() - 1) * (ys.size() - 1))
        throw std::runtime_error("invalid triangle table header: " + path);
    cells_.assign((xs.size() - 1) * (ys.size() - 1), {kMissing, kMissing});
    triangles_.reserve(count);
    std::vector<std::array<size_t, 2>> grid;
    grid.reserve(rows.size());
    for (const auto &row : rows)
        grid.push_back({static_cast<size_t>(std::lower_bound(xs.begin(), xs.end(), row[0]) - xs.begin()),
            static_cast<size_t>(std::lower_bound(ys.begin(), ys.end(), row[1]) - ys.begin())});
    for (size_t i = 0; i < count; ++i) {
        if (!std::getline(stream, line) || line.size() > 128)
            throw std::runtime_error("truncated/invalid triangle table: " + path);
        std::istringstream record(line);
        std::array<uint32_t, 3> triangle;
        char comma1 = 0, comma2 = 0;
        if (!(record >> triangle[0] >> comma1 >> triangle[1] >> comma2 >> triangle[2]) ||
            comma1 != ',' || comma2 != ',' || (record >> extra))
            throw std::runtime_error("invalid triangle indices: " + path);
        for (auto vertex : triangle)
            if (vertex >= rows.size()) throw std::runtime_error("triangle index out of range: " + path);
        const auto a = grid[triangle[0]], b = grid[triangle[1]], c = grid[triangle[2]];
        const size_t lx = std::min({a[0], b[0], c[0]}), ly = std::min({a[1], b[1], c[1]});
        const size_t hx = std::max({a[0], b[0], c[0]}), hy = std::max({a[1], b[1], c[1]});
        if (a == b || a == c || b == c || hx != lx + 1 || hy != ly + 1)
            throw std::runtime_error("triangle must span one regular grid cell: " + path);
        auto &cell = cells_[lx * (ys.size() - 1) + ly];
        if (cell[0] == kMissing) {
            cell[0] = i;
        } else {
            if (cell[1] != kMissing) throw std::runtime_error("too many triangles in cell: " + path);
            unsigned old_mask = 0, mask = 0;
            for (auto vertex : triangles_[cell[0]])
                old_mask |= 1U << ((grid[vertex][0] - lx) * 2 + grid[vertex][1] - ly);
            for (auto vertex : triangle)
                mask |= 1U << ((grid[vertex][0] - lx) * 2 + grid[vertex][1] - ly);
            const unsigned shared = old_mask & mask;
            if ((old_mask | mask) != 15U || (shared != 9U && shared != 6U))
                throw std::runtime_error("overlapping triangle pair: " + path);
            cell[1] = i;
        }
        triangles_.push_back(triangle);
    }
    while (std::getline(stream, line))
        if (line.find_first_not_of(" \t\r") != std::string::npos)
            throw std::runtime_error("excess triangle rows: " + path);
    if (!stream.eof()) throw std::runtime_error("triangle table read failure: " + path);
    for (const auto &cell : cells_)
        if (cell[0] != kMissing && cell[1] == kMissing)
            throw std::runtime_error("incomplete triangle cell: " + path);
}

bool TriangleTable::Contains(uint32_t triangle, const std::array<double, 2> &point) const {
    const auto &indices = triangles_[triangle];
    const auto &a = rows_[indices[0]], &b = rows_[indices[1]], &c = rows_[indices[2]];
    for (size_t axis = 0; axis < 2; ++axis)
        if (point[axis] < std::min({a[axis], b[axis], c[axis]}) ||
            point[axis] > std::max({a[axis], b[axis], c[axis]})) return false;
    int sign = 0;
    for (size_t edge = 0; edge < 3; ++edge) {
        const auto &p = rows_[indices[edge]], &q = rows_[indices[(edge + 1) % 3]];
        const double dx = q[0] - p[0], dy = q[1] - p[1];
        const double px = point[0] - p[0], py = point[1] - p[1];
        const double cross = std::fma(dx, py, -dy * px);
        const double tolerance = std::numeric_limits<double>::epsilon() *
            std::max({std::abs(dx), std::abs(dy), std::abs(px), std::abs(py), 1.0});
        if (std::abs(cross) <= tolerance) continue;
        const int current = cross > 0 ? 1 : -1;
        if (sign != 0 && sign != current) return false;
        sign = current;
    }
    return true;
}

bool TriangleTable::Lookup(const std::array<double, 2> &input, std::array<double, 2> *value,
    std::array<double, 4> *derivative) const {
    if (!std::isfinite(input[0]) || !std::isfinite(input[1]) || input[0] < xs_.front() ||
        input[0] > xs_.back() || input[1] < ys_.front() || input[1] > ys_.back()) return false;
    const size_t ix = std::min(static_cast<size_t>(
        std::upper_bound(xs_.begin(), xs_.end(), input[0]) - xs_.begin() - 1), xs_.size() - 2);
    const size_t iy = std::min(static_cast<size_t>(
        std::upper_bound(ys_.begin(), ys_.end(), input[1]) - ys_.begin() - 1), ys_.size() - 2);
    uint32_t selected = kMissing;
    // Include neighboring cells at shared vertices/edges; first asset row wins.
    for (size_t x = ix == 0 ? 0 : ix - 1; x <= std::min(ix + 1, xs_.size() - 2); ++x) {
        for (size_t y = iy == 0 ? 0 : iy - 1; y <= std::min(iy + 1, ys_.size() - 2); ++y) {
            if (input[0] < xs_[x] || input[0] > xs_[x + 1] ||
                input[1] < ys_[y] || input[1] > ys_[y + 1]) continue;
            for (auto triangle : cells_[x * (ys_.size() - 1) + y])
                if (triangle < selected && Contains(triangle, input)) selected = triangle;
        }
    }
    if (selected == kMissing) return false;
    const auto &indices = triangles_[selected];
    const auto &a = rows_[indices[0]], &b = rows_[indices[1]], &c = rows_[indices[2]];
    const double dx1 = b[0] - a[0], dy1 = b[1] - a[1];
    const double dx2 = c[0] - a[0], dy2 = c[1] - a[1];
    // Explicit FMA fixes plane arithmetic across targets/compiler settings.
    const double nz = std::fma(-dx2, dy1, dy2 * dx1);
    for (size_t side = 0; side < 2; ++side) {
        const double z0 = a[side + 2], dz1 = b[side + 2] - z0, dz2 = c[side + 2] - z0;
        const double normal_y = std::fma(-dz2, dx1, dx2 * dz1);
        const double normal_x = std::fma(-dy2, dz1, dz2 * dy1);
        const double offset = std::fma(normal_x, a[0], std::fma(normal_y, a[1], nz * z0));
        (*derivative)[side * 2] = -normal_x / nz;
        (*derivative)[side * 2 + 1] = -normal_y / nz;
        (*value)[side] = -std::fma(input[1], normal_y, std::fma(input[0], normal_x, -offset)) / nz;
        if (!std::isfinite((*value)[side]) || !std::isfinite((*derivative)[side * 2]) ||
            !std::isfinite((*derivative)[side * 2 + 1])) return false;
    }
    return true;
}

}  // namespace whole_body

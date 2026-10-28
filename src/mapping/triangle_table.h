/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file triangle_table.h
 * @brief Fixed, ordered triangulation for reproducible two-axis calibration.
 */
#ifndef TRIANGLE_TABLE_H
#define TRIANGLE_TABLE_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace whole_body {

class TriangleTable {
    public:
    TriangleTable(const std::string &path, const std::vector<std::array<double, 4>> &rows,
        const std::vector<double> &xs, const std::vector<double> &ys);
    // Shared edges use the first containing triangle in asset order. Lookup
    // allocates nothing and visits at most two triangles in each nearby cell.
    bool Lookup(const std::array<double, 2> &input, std::array<double, 2> *value,
        std::array<double, 4> *derivative) const;

    private:
    bool Contains(uint32_t triangle, const std::array<double, 2> &point) const;
    std::vector<std::array<double, 4>> rows_;
    std::vector<double> xs_;
    std::vector<double> ys_;
    std::vector<std::array<uint32_t, 3>> triangles_;
    std::vector<std::array<uint32_t, 2>> cells_;
};

}  // namespace whole_body
#endif  // TRIANGLE_TABLE_H

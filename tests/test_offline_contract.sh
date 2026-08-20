#!/usr/bin/env bash
#
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#
# Builds and runs the whole-body tests with fake devices only.

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
staging_dir="${SROBOTIS_OUTPUT_STAGING:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/staging}"
if [[ -n "${SROBOTIS_TEST_ARTIFACT_DIR:-}" ]]; then
    artifact_dir="$SROBOTIS_TEST_ARTIFACT_DIR"
else
    artifact_dir="$(mktemp -d "${TMPDIR:-/tmp}/whole-body-offline-contract.XXXXXX")"
    trap 'rm -rf "$artifact_dir"' EXIT
fi
log_dir="$artifact_dir/logs"
log_file="$log_dir/whole_body_offline_contract.log"
build_dir="$artifact_dir/build"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"
    echo "[info] staging_dir=$staging_dir"

    cmake -S "$module_root" -B "$build_dir" \
        -DCMAKE_INSTALL_PREFIX="$staging_dir" \
        -DCMAKE_PREFIX_PATH="$staging_dir" \
        -DBUILD_TESTS=ON
    cmake --build "$build_dir" --parallel
    LD_LIBRARY_PATH="$build_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
        ctest --test-dir "$build_dir" --output-on-failure
} 2>&1 | tee "$log_file"

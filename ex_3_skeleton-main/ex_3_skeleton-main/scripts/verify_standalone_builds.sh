#!/usr/bin/env bash
# Build-structure verification for ex3 -- NOT a gtest, a pre-submission /
# post-CMakeLists-change sanity check (see ex3-test-plan.md, section 7,
# "Build-structure verification").
#
# Confirms the spec's "each part may run independently with another team's
# implementation of the other parts" by building Algorithm/, MissionControl/,
# and Simulator/ standalone -- each from a clean, isolated build dir, with
# its sibling project folders untouched/not visible as a parent build --
# and separately dry-runs what a submission zip would contain (5 folders +
# 4 build files + students.txt + README.md, no binaries).
#
# Usage: ./scripts/verify_standalone_builds.sh
# Requires: VCPKG_ROOT set, same as the root/per-project builds.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ -z "${VCPKG_ROOT:-}" ]]; then
    echo "error: VCPKG_ROOT is not set" >&2
    exit 1
fi

TOOLCHAIN="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
FAILED=0

build_standalone() {
    local project_dir="$1"
    local build_dir
    build_dir="$(mktemp -d)"
    echo "=== Building $project_dir standalone in $build_dir ==="
    if cmake -S "$ROOT_DIR/$project_dir" -B "$build_dir" \
            -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" >/tmp/verify_${project_dir//\//_}_configure.log 2>&1 \
       && cmake --build "$build_dir" --parallel 4 >/tmp/verify_${project_dir//\//_}_build.log 2>&1; then
        echo "OK: $project_dir builds standalone"
    else
        echo "FAIL: $project_dir does not build standalone -- see /tmp/verify_${project_dir//\//_}_{configure,build}.log" >&2
        FAILED=1
    fi
    rm -rf "$build_dir"
}

build_standalone "Algorithm"
build_standalone "MissionControl"
build_standalone "Simulator"

echo
echo "=== Submission zip dry run ==="
REQUIRED_FOLDERS=(Simulator Algorithm MissionControl common UserCommon)
for f in "${REQUIRED_FOLDERS[@]}"; do
    if [[ ! -d "$ROOT_DIR/$f" ]]; then
        echo "FAIL: required folder missing: $f" >&2
        FAILED=1
    fi
done
for f in Simulator/CMakeLists.txt Algorithm/CMakeLists.txt MissionControl/CMakeLists.txt CMakeLists.txt; do
    if [[ ! -f "$ROOT_DIR/$f" ]]; then
        echo "FAIL: required build file missing: $f" >&2
        FAILED=1
    fi
done
for f in students.txt README.md; do
    if [[ ! -f "$ROOT_DIR/$f" ]]; then
        echo "FAIL: required top-level file missing: $f" >&2
        FAILED=1
    fi
done
if [[ -f "$ROOT_DIR/UserCommon/CMakeLists.txt" ]]; then
    echo "FAIL: UserCommon must have NO makefile (per spec, section 4b)" >&2
    FAILED=1
fi
if [[ -f "$ROOT_DIR/common/CMakeLists.txt" ]]; then
    echo "note: common/CMakeLists.txt exists -- spec says no makefiles in common/UserCommon, but" \
         "common/ is staff-provided as-is; confirm this matches what was actually published."
fi

echo
echo "=== Binary-file scan (none should be present outside build/ dirs) ==="
if find "$ROOT_DIR" -type d -name build -prune -o \
        -type f \( -name "*.so" -o -name "*.o" -o -executable \) -print \
        | grep -vE '\.(sh|py)$' \
        | grep -v "/scripts/"; then
    echo "FAIL: binary-looking files found outside build/ -- review the list above" >&2
    FAILED=1
else
    echo "OK: no stray binaries found outside build/ directories"
fi

echo
if [[ "$FAILED" -eq 0 ]]; then
    echo "All build-structure checks passed."
else
    echo "One or more build-structure checks FAILED -- see above." >&2
    exit 1
fi

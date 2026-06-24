#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="${XGC2_OBSERVER_QUALITY_BUILD_DIR:-${repo_root}/.ci/cpp-quality}"

cd "${repo_root}"

sources=(
  include/xgc2_observer/angle.hpp
  include/xgc2_observer/angle_differentiator.hpp
  include/xgc2_observer/array_observer.hpp
  include/xgc2_observer/butterworth_filter.hpp
  include/xgc2_observer/differentiator.hpp
  include/xgc2_observer/exponential_filter.hpp
  include/xgc2_observer/inertial_pose_eskf.hpp
  include/xgc2_observer/luenberger_observer.hpp
  include/xgc2_observer/observer.hpp
  include/xgc2_observer/recursive_least_squares.hpp
  include/xgc2_observer/se3.hpp
  include/xgc2_observer/status.hpp
  include/xgc2_observer/time_delta.hpp
  test/observer_header_test.cpp
)

tidy_sources=(
  test/observer_header_test.cpp
)

for tool in cmake ctest clang-format clang-tidy cppcheck; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    echo "Missing required C++ quality tool: ${tool}" >&2
    exit 1
  fi
done

clang-format --dry-run --Werror "${sources[@]}"

rm -rf "${build_dir}"
cmake -S "${repo_root}" -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual -Werror"
cmake --build "${build_dir}" -- -j"$(nproc)"
(cd "${build_dir}" && ctest --output-on-failure)

clang-tidy --quiet -p "${build_dir}" "${tidy_sources[@]}" 2>&1 | sed -E '/^[0-9]+ warnings generated\.$/d'

cppcheck \
  --enable=warning,performance,portability,style \
  --error-exitcode=1 \
  --std=c++17 \
  --inline-suppr \
  -I include \
  include test

echo "C++ quality checks passed."

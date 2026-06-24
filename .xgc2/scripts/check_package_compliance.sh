#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${repo_root}"

bash -n .xgc2/scripts/*.sh

nested_git="$(
  find . \
    -path ./.git -prune -o \
    -path ./.ci -prune -o \
    -path ./build -prune -o \
    -name .git -print
)"
if [[ -n "${nested_git}" ]]; then
  echo "Nested .git directory found." >&2
  echo "${nested_git}" >&2
  exit 1
fi

if git ls-files 2>/dev/null | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.ci)(/|$)' >/dev/null; then
  echo "Generated build artifacts are tracked." >&2
  git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.ci)(/|$)' >&2
  exit 1
fi

required_files=(
  .clang-format
  .clang-tidy
  README.md
  CMakeLists.txt
  cmake/xgc2_mathConfig.cmake.in
  include/math.hpp
  include/types.hpp
  include/core.hpp
  include/geometry.hpp
  include/filter.hpp
  include/observer.hpp
  include/estimation.hpp
  include/control.hpp
  include/core/angle.hpp
  include/core/status.hpp
  include/core/time_delta.hpp
  include/geometry/se3.hpp
  include/geometry/occupied_sets/sphere_set.h
  include/filter/exponential_filter.hpp
  include/observer/differentiator.hpp
  include/estimation/inertial_pose_eskf.hpp
  test/math_header_test.cpp
  .github/workflows/ci.yml
  .xgc2/product.yml
  .xgc2/scripts/build_deb.sh
  .xgc2/scripts/check_cpp_quality.sh
  .xgc2/scripts/check_package_compliance.sh
  .xgc2/scripts/publish_self_hosted_apt.sh
  .xgc2/scripts/smoke_test_installed.sh
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "Missing required file: ${file}" >&2
    exit 1
  fi
done

for removed_file in package.xml .github/workflows/build-debs.yml include/xgc2_observer cmake/xgc2_observerConfig.cmake.in; do
  if [[ -e "${removed_file}" ]]; then
    echo "xgc2-math package must not keep obsolete file: ${removed_file}" >&2
    exit 1
  fi
done

if grep -R "find_package(catkin\\|catkin_package\\|catkin_add_gtest" CMakeLists.txt cmake 2>/dev/null; then
  echo "system package must not use catkin." >&2
  exit 1
fi

if grep -R "xgc2_observer\\|xgc2_geometry" CMakeLists.txt cmake include test README.md .github .xgc2/product.yml 2>/dev/null; then
  echo "obsolete xgc2_observer/xgc2_geometry identifiers remain." >&2
  exit 1
fi

echo "libxgc2-math-dev package compliance checks passed."

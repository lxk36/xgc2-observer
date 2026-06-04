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
  README.md
  CMakeLists.txt
  cmake/xgc2_observerConfig.cmake.in
  include/xgc2_observer/observer.hpp
  include/xgc2_observer/angle.hpp
  include/xgc2_observer/angle_differentiator.hpp
  include/xgc2_observer/array_observer.hpp
  include/xgc2_observer/butterworth_filter.hpp
  include/xgc2_observer/differentiator.hpp
  include/xgc2_observer/exponential_filter.hpp
  include/xgc2_observer/luenberger_observer.hpp
  include/xgc2_observer/recursive_least_squares.hpp
  include/xgc2_observer/status.hpp
  include/xgc2_observer/time_delta.hpp
  test/observer_header_test.cpp
  .github/workflows/ci.yml
  .xgc2/product.yml
  .xgc2/scripts/build_deb.sh
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

for removed_file in package.xml; do
  if [[ -e "${removed_file}" ]]; then
    echo "system package must not keep ROS/catkin file: ${removed_file}" >&2
    exit 1
  fi
done

if grep -R "find_package(catkin\\|catkin_package\\|catkin_add_gtest" CMakeLists.txt cmake 2>/dev/null; then
  echo "system package must not use catkin." >&2
  exit 1
fi

echo "libxgc2-observer-dev package compliance checks passed."

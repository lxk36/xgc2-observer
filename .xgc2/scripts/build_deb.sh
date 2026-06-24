#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"

package_name="libxgc2-observer-dev"
version="${PACKAGE_VERSION:-0.3.3-1}"
build_dir="${XGC2_OBSERVER_BUILD_DIR:-${repo_root}/.ci/build}"
stage_dir="${XGC2_OBSERVER_STAGE_DIR:-${repo_root}/.ci/stage}"
output_dir="${XGC2_OBSERVER_DEB_OUTPUT_DIR:-${repo_root}/.ci/debs}"
pkg_root="${repo_root}/.ci/pkg/${package_name}"
arch="all"

rm -rf "${build_dir}" "${stage_dir}" "${output_dir}" "${pkg_root}"
mkdir -p "${output_dir}"

cmake -S "${repo_root}" -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "${build_dir}" -- -j"$(nproc)"
(cd "${build_dir}" && ctest --output-on-failure)
DESTDIR="${stage_dir}" cmake --install "${build_dir}" --prefix /usr

mkdir -p \
  "${pkg_root}/DEBIAN" \
  "${pkg_root}/usr/share/doc/${package_name}"

cp -a "${stage_dir}/usr" "${pkg_root}/"

cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${package_name}
Version: ${version}
Section: libdevel
Priority: optional
Architecture: ${arch}
Maintainer: XGC2 <apt@example.com>
Depends: libeigen3-dev
Conflicts: ros-noetic-xgc2-observer
Replaces: ros-noetic-xgc2-observer
Description: XGC2 C++ observer and timing-tolerant signal utilities
 Header-only observer, differentiator, and filtering utilities for
 robotics runtime, simulation, and controller code.
EOF

cp -a "${repo_root}/README.md" "${pkg_root}/usr/share/doc/${package_name}/"

test -f "${pkg_root}/usr/include/xgc2_observer/observer.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/angle.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/angle_differentiator.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/array_observer.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/butterworth_filter.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/differentiator.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/exponential_filter.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/inertial_pose_eskf.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/luenberger_observer.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/recursive_least_squares.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/se3.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/status.hpp"
test -f "${pkg_root}/usr/include/xgc2_observer/time_delta.hpp"
test -f "${pkg_root}/usr/lib/cmake/xgc2_observer/xgc2_observerConfig.cmake"

find "${pkg_root}" -type d -exec chmod 0755 {} +
find "${pkg_root}" -type f -exec chmod 0644 {} +
chmod 0755 "${pkg_root}/DEBIAN"

fakeroot dpkg-deb --build "${pkg_root}" "${output_dir}/${package_name}_${version}_${arch}.deb" >/dev/null
dpkg-deb -I "${output_dir}/${package_name}_${version}_${arch}.deb"
echo "Debian artifacts written to ${output_dir}"

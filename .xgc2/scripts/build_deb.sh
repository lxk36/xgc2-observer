#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"

version="${PACKAGE_VERSION:-0.4.1-1}"
build_dir="${XGC2_MATH_BUILD_DIR:-${repo_root}/.ci/build}"
stage_dir="${XGC2_MATH_STAGE_DIR:-${repo_root}/.ci/stage}"
output_dir="${XGC2_MATH_DEB_OUTPUT_DIR:-${repo_root}/.ci/debs}"
pkg_dir="${repo_root}/.ci/pkg"
arch="all"

rm -rf "${build_dir}" "${stage_dir}" "${output_dir}" "${pkg_dir}"
mkdir -p "${output_dir}" "${pkg_dir}"

cmake -S "${repo_root}" -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "${build_dir}" -- -j"$(nproc)"
(cd "${build_dir}" && ctest --output-on-failure)
DESTDIR="${stage_dir}" cmake --install "${build_dir}" --prefix /usr

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${stage_dir}}")"
    cp -a "${src}" "${dst_root}${src#${stage_dir}}"
  fi
}

write_control() {
  local package_name="$1"
  local depends="$2"
  local description="$3"
  local pkg_root="${pkg_dir}/${package_name}"

  mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${package_name}"
  cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${package_name}
Version: ${version}
Section: libdevel
Priority: optional
Architecture: ${arch}
Maintainer: XGC2 <apt@example.com>
Depends: ${depends}
Description: ${description}
 Header-only XGC2 C++ math development package.
EOF
  cp -a "${repo_root}/README.md" "${pkg_root}/usr/share/doc/${package_name}/"
  chmod 0755 "${pkg_root}/DEBIAN"
}

build_package() {
  local package_name="$1"
  local pkg_root="${pkg_dir}/${package_name}"
  find "${pkg_root}" -type d -exec chmod 0755 {} +
  find "${pkg_root}" -type f -exec chmod 0644 {} +
  chmod 0755 "${pkg_root}/DEBIAN"
  fakeroot dpkg-deb --build "${pkg_root}" "${output_dir}/${package_name}_${version}_${arch}.deb" >/dev/null
  dpkg-deb -I "${output_dir}/${package_name}_${version}_${arch}.deb"
}

base_depends="libeigen3-dev, libc6, libgcc-s1, libstdc++6"

core_pkg="libxgc2-math-core-dev"
write_control "${core_pkg}" "${base_depends}" "XGC2 math core headers"
copy_path "${stage_dir}/usr/include/xgc2_math/core" "${pkg_dir}/${core_pkg}"
copy_path "${stage_dir}/usr/include/xgc2_math/core.hpp" "${pkg_dir}/${core_pkg}"

geometry_pkg="libxgc2-math-geometry-dev"
write_control "${geometry_pkg}" "libxgc2-math-core-dev (= ${version}), ${base_depends}" \
  "XGC2 math geometry headers"
copy_path "${stage_dir}/usr/include/xgc2_math/geometry" "${pkg_dir}/${geometry_pkg}"
copy_path "${stage_dir}/usr/include/xgc2_math/geometry.hpp" "${pkg_dir}/${geometry_pkg}"

filter_pkg="libxgc2-math-filter-dev"
write_control "${filter_pkg}" "libxgc2-math-core-dev (= ${version}), ${base_depends}" \
  "XGC2 math filter headers"
copy_path "${stage_dir}/usr/include/xgc2_math/filter" "${pkg_dir}/${filter_pkg}"
copy_path "${stage_dir}/usr/include/xgc2_math/filter.hpp" "${pkg_dir}/${filter_pkg}"

observer_pkg="libxgc2-math-observer-dev"
write_control "${observer_pkg}" \
  "libxgc2-math-core-dev (= ${version}), libxgc2-math-filter-dev (= ${version}), ${base_depends}" \
  "XGC2 math observer headers"
copy_path "${stage_dir}/usr/include/xgc2_math/observer" "${pkg_dir}/${observer_pkg}"
copy_path "${stage_dir}/usr/include/xgc2_math/observer.hpp" "${pkg_dir}/${observer_pkg}"

estimation_pkg="libxgc2-math-estimation-dev"
write_control "${estimation_pkg}" \
  "libxgc2-math-core-dev (= ${version}), libxgc2-math-geometry-dev (= ${version}), libxgc2-math-filter-dev (= ${version}), libxgc2-math-observer-dev (= ${version}), ${base_depends}" \
  "XGC2 math estimation headers"
copy_path "${stage_dir}/usr/include/xgc2_math/estimation" "${pkg_dir}/${estimation_pkg}"
copy_path "${stage_dir}/usr/include/xgc2_math/estimation.hpp" "${pkg_dir}/${estimation_pkg}"

control_pkg="libxgc2-math-control-dev"
write_control "${control_pkg}" "libxgc2-math-core-dev (= ${version}), ${base_depends}" \
  "XGC2 math control headers"
copy_path "${stage_dir}/usr/include/xgc2_math/control.hpp" "${pkg_dir}/${control_pkg}"

meta_pkg="libxgc2-math-dev"
write_control "${meta_pkg}" \
  "libxgc2-math-core-dev (= ${version}), libxgc2-math-geometry-dev (= ${version}), libxgc2-math-filter-dev (= ${version}), libxgc2-math-observer-dev (= ${version}), libxgc2-math-estimation-dev (= ${version}), libxgc2-math-control-dev (= ${version}), ${base_depends}" \
  "XGC2 math meta development package"
copy_path "${stage_dir}/usr/include/xgc2_math/math.hpp" "${pkg_dir}/${meta_pkg}"
copy_path "${stage_dir}/usr/include/xgc2_math/types.hpp" "${pkg_dir}/${meta_pkg}"
copy_path "${stage_dir}/usr/lib/cmake/xgc2_math" "${pkg_dir}/${meta_pkg}"
copy_path "${stage_dir}/usr/share/doc/libxgc2-math-dev/matlab" "${pkg_dir}/${meta_pkg}"

test -f "${pkg_dir}/${core_pkg}/usr/include/xgc2_math/core/status.hpp"
test -f "${pkg_dir}/${geometry_pkg}/usr/include/xgc2_math/geometry/se2.hpp"
test -f "${pkg_dir}/${geometry_pkg}/usr/include/xgc2_math/geometry/occupied_sets/sphere_set.h"
test -f "${pkg_dir}/${filter_pkg}/usr/include/xgc2_math/filter/exponential_filter.hpp"
test -f "${pkg_dir}/${observer_pkg}/usr/include/xgc2_math/observer/differentiator.hpp"
test -f "${pkg_dir}/${estimation_pkg}/usr/include/xgc2_math/estimation/inertial_pose_eskf.hpp"
test -f "${pkg_dir}/${estimation_pkg}/usr/include/xgc2_math/estimation/planar_inertial_eskf.hpp"
test -f "${pkg_dir}/${control_pkg}/usr/include/xgc2_math/control.hpp"
test -f "${pkg_dir}/${meta_pkg}/usr/include/xgc2_math/math.hpp"
test -f "${pkg_dir}/${meta_pkg}/usr/lib/cmake/xgc2_math/xgc2_mathConfig.cmake"

for package_name in \
  "${core_pkg}" \
  "${geometry_pkg}" \
  "${filter_pkg}" \
  "${observer_pkg}" \
  "${estimation_pkg}" \
  "${control_pkg}" \
  "${meta_pkg}"; do
  build_package "${package_name}"
done

echo "Debian artifacts written to ${output_dir}"

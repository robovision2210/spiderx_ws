#!/usr/bin/env bash
# Build the SpiderX overlay of gz_ros2_control: the pinned upstream release plus one patch,
# into its own colcon workspace. Never writes to the toolchain (/opt/mm) or to this repository.
#
#   source /opt/mm/activate.sh
#   third_party/gz_ros2_control_overlay/build_overlay.sh [WORKSPACE]   # default ~/spiderx_gz_overlay_ws
#
# Fetches with git (network needed once); refuses an existing checkout that is not the pinned
# commit plus exactly this patch. Builds with the environment's own compilers, runs the offline
# gtest, and writes WORKSPACE/OVERLAY_PROVENANCE.txt. Runtime order (see README.md):
#   source /opt/mm/activate.sh; source WORKSPACE/install/local_setup.bash;
#   source <spiderx_ws>/install/local_setup.bash
set -euo pipefail

REPO_URL=https://github.com/ros-controls/gz_ros2_control.git
TAG=0.7.15
COMMIT=bfc5845aa7f0250cbc384773dba5c9847d609b95
PACKAGE_DIR=gz_ros2_control

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH="$HERE/patches/0001-plugin-node-parameter-overrides.patch"
WS="$(realpath -m "${1:-${SPIDERX_GZ_OVERLAY_WS:-$HOME/spiderx_gz_overlay_ws}}")"
SRC="$WS/src/gz_ros2_control"

die() { echo "build_overlay: $*" >&2; exit 1; }

[ -n "${CONDA_PREFIX:-}" ] && command -v colcon >/dev/null \
  || die 'activate the toolchain first: source /opt/mm/activate.sh'
CC_BIN="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc"
CXX_BIN="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++"
[ -x "$CC_BIN" ] && [ -x "$CXX_BIN" ] || die "the environment's compilers are missing: $CXX_BIN"
case "$WS/" in "$(realpath "$HERE/../..")/"*) die "WORKSPACE must be outside the repository";; esac

# Always a fresh workspace: nothing is reused or deleted. To rebuild, remove WORKSPACE yourself.
[ ! -e "$WS" ] || [ -z "$(ls -A "$WS")" ] || die "$WS exists and is not empty; remove it to rebuild"
mkdir -p "$(dirname "$SRC")"
git init -q "$SRC"
git -C "$SRC" remote add origin "$REPO_URL"
git -C "$SRC" fetch -q --depth 1 origin "refs/tags/$TAG"
git -C "$SRC" sparse-checkout set "$PACKAGE_DIR"          # this package only (+ root files)
git -C "$SRC" -c advice.detachedHead=false checkout -q --detach FETCH_HEAD
[ "$(git -C "$SRC" rev-parse HEAD)" = "$COMMIT" ] \
  || die "tag $TAG is not commit $COMMIT any more; refusing to build"
git -C "$SRC" apply --check "$PATCH"
git -C "$SRC" apply "$PATCH"
echo "build_overlay: fetched $TAG ($COMMIT) and applied $(basename "$PATCH")"

cd "$WS"
# -include cstdint: the environment's GCC 15 cannot compile gtest_vendor's googletest 1.10
# (missing <cstdint>); it only affects this build. The install RPATH points at the toolchain's
# lib directory, as the stock libraries' $ORIGIN does for themselves.
CC="$CC_BIN" CXX="$CXX_BIN" colcon --log-base "$WS/log" build \
  --base-paths "$SRC/$PACKAGE_DIR" --build-base "$WS/build" --install-base "$WS/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON "-DCMAKE_CXX_FLAGS=-include cstdint"
colcon --log-base "$WS/log" test \
  --base-paths "$SRC/$PACKAGE_DIR" --build-base "$WS/build" --install-base "$WS/install" \
  --ctest-args -R '^test_parameter_loading$'
colcon test-result --test-result-base "$WS/build/gz_ros2_control" --verbose

LIB="$WS/install/gz_ros2_control/lib"
{
  echo "upstream $REPO_URL tag $TAG commit $COMMIT"
  echo "patch $(sha256sum "$PATCH" | cut -d' ' -f1) $(basename "$PATCH")"
  echo "compiler $("$CXX_BIN" --version | head -1)"
  echo "toolchain $CONDA_PREFIX"
  for f in "$LIB/libgz_ros2_control-system.so" "$LIB/libgz_hardware_plugins.so"; do
    echo "library $(sha256sum "$f" | cut -d' ' -f1) $f"
  done
} > "$WS/OVERLAY_PROVENANCE.txt"
cat "$WS/OVERLAY_PROVENANCE.txt"
echo "build_overlay: done. Source $WS/install/local_setup.bash after /opt/mm/activate.sh."

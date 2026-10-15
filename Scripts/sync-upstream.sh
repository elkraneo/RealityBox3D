#!/bin/sh
# Re-vendors Box3D at a given commit (default: the one pinned in UPSTREAM.md).
# Upstream files are copied unmodified; review the diff, then run the tests, b3-bench and b3-snapshot.
#
#   Scripts/sync-upstream.sh [commit]
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
commit=${1:-$(sed -n 's/^Commit: `\(.*\)`$/\1/p' "$root/UPSTREAM.md")}
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

git clone --quiet https://github.com/erincatto/box3d "$work/box3d"
git -C "$work/box3d" checkout --quiet "$commit"

rm -rf "$root/Sources/CBox3D/include" "$root/Sources/CBox3D/src"
cp -R "$work/box3d/include" "$root/Sources/CBox3D/include"
cp -R "$work/box3d/src" "$root/Sources/CBox3D/src"
rm -f "$root/Sources/CBox3D/src/CMakeLists.txt" "$root/Sources/CBox3D/src/box3d.natvis"
cp "$work/box3d/LICENSE" "$root/Sources/CBox3D/LICENSE.box3d"

for f in benchmarks human utils; do
    cp "$work/box3d/shared/$f.c" "$root/Sources/CBox3DShared/$f.c"
    cp "$work/box3d/shared/$f.h" "$root/Sources/CBox3DShared/include/$f.h"
done
rm -rf "$root/Sources/CBox3DShared/m2air_neon-reference"
cp -R "$work/box3d/benchmark/m2air_neon" "$root/Sources/CBox3DShared/m2air_neon-reference"

sed -i '' "s/^Commit: \`.*\`$/Commit: \`$(git -C "$work/box3d" rev-parse HEAD)\`/" "$root/UPSTREAM.md"
echo "Vendored $(git -C "$work/box3d" rev-parse --short HEAD). Now: git diff, swift test, swift run -c release b3-bench, swift run -c release b3-snapshot <dir>"
echo "If the .inl files in src changed, check the exclude list of the CBox3D target in Package.swift."

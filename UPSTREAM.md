# Upstream

Box3D is vendored unmodified from [erincatto/box3d](https://github.com/erincatto/box3d)
(MIT License, Erin Catto). Upstream calls it alpha, so it is pinned to one commit.

Commit: `0cfc475b88d0e46bea9533a460afdd79a9001868`

- `Sources/CBox3D/include` and `Sources/CBox3D/src`: the library, byte for byte, without its CMake
  files and debugger visualizer.
- `Sources/CBox3DShared`: upstream's `shared/` benchmark scenes (`benchmarks`, `human`, `utils`),
  used only by `b3-bench` as the raw C baseline, and `m2air_neon-reference`, upstream's reference
  timings from `benchmark/m2air_neon`.
- `Sources/CBox3D/LICENSE.box3d`: upstream's license.
- `Samples/Box3DDemoScenes/Meshes`: upstream's `data/meshes/test_map01.obj` and `stairs.obj`, the
  Geometric Mover's level.

Do not edit vendored files. Behavior belongs in the Swift layer, `Sources/RealityBox3D`.
To update, run `Scripts/sync-upstream.sh [commit]`.

## Build settings

SwiftPM does not define `NDEBUG` in release builds, and Box3D keys its assertions and the size of
its benchmark scenes off it, so `Package.swift` defines it for both C targets in release. Without
it, `b3-bench` times the small debug scenes.

## Checking an update

1. `swift test`: the bridge's tests, every demo scene, and three comparisons with raw Box3D:
   upstream's Arch through the C API must match the port, the Weeble's mass data must match
   upstream's `b3Steiner` result, and forecasts must equal the real world step for step.
2. `swift run -c release b3-bench`: the raw C column should stay near upstream's M2 Air reference.
3. `swift run -c release b3-snapshot <dir>`: renders every demo scene.

Box3D's recordings, which `B3World.forecast()` uses, are gated on Box3D's internal struct layout,
so a forecast only works within one build. That is fine for forecasting, which never leaves the process.

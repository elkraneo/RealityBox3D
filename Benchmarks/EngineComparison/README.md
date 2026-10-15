# Engine comparison

The same piles on RealityKit's physics, RealitySDFPhysics and RealityBox3D, headless, with each
engine's defaults and sleeping, 300 steps at 60 Hz.

- **tori**: RealitySDFPhysics's own benchmark, tori 30 cm across. RealityKit and Box3D simulate each
  torus as 12 capsules, their closest convex stand-in; RealitySDFPhysics simulates the real torus.
- **boxes**: 10 cm boxes, which every engine simulates natively.

RealityKit is timed between its `WillSimulate` and `DidSimulate` events with its clock stepped by
hand, RealitySDFPhysics's method. RealitySDFPhysics and RealityBox3D are timed around a whole step,
their entity sync included; "engine" is the engine's own share.

```sh
swift run -c release --scratch-path /tmp/ec engine-comparison [tori|boxes] [--counts 25,100] [--steps 300]
```

RealitySDFPhysics is pinned to an exact release in `Package.swift`, so its later changes never
move these numbers. Builds are large, so put `--scratch-path` on a disk with room.

## Apple M1, 2026-10-15

Measured with RealitySDFPhysics at commit 30ddbb9, from before its rename to RealitySDFPhysics
and its 0.2.0 release, when it was still called SDFCollisionKit. These numbers have not yet been
remeasured on 0.2.0.

Two runs: `results-2026-10-15-m1-loaded.txt` under a load average of about 14 from other work, and
`results-2026-10-15-m1-quiet.txt` at about 6, the reference below. Mean ms per step:

| Pile | Bodies | RealityBox3D, 1 worker | RealityBox3D, 4 workers | RealitySDFPhysics (30ddbb9) | RealityKit |
| --- | ---: | ---: | ---: | ---: | ---: |
| tori | 25 | 0.08 | 0.10 | 0.27 | 0.17 |
| tori | 100 | 0.59 | 0.57 | 1.33 | 1.75 |
| tori | 400 | 4.48 | 3.61 | 9.69 | 12.96 |
| boxes | 100 | 0.10 | 0.12 | 0.38 | 0.11 |
| boxes | 400 | 0.60 | 0.65 | 3.54 | 1.15 |
| boxes | 1,600 | 4.85 | 2.14 | 33.15 | 5.52 |

- Box3D, sync included, is the fastest on every pile: about 3.5 times RealityKit on 400 tori made of
  capsules, and 2.5 times on 1,600 boxes with 4 workers.
- RealitySDFPhysics pays for exact concave contact: on boxes it does work the convex engines skip.
  On tori it is the only one simulating the true shape, and still beats RealityKit's capsule rings.
- Box3D's workers pay off from about a thousand bodies on a quiet machine. Under heavy load they
  added spikes instead, so the demo app starts with one and lets you raise it.
- The 1-worker 1,600-box run had one 208 ms step, a scheduling hiccup; its mean is otherwise in
  line with the loaded run.
- This harness drives `B3World.update(entities:)`, a full scan each frame. Inside RealityKit,
  `B3PhysicsSystem` reads only changed entities, so sleeping bodies cost nothing to sync.

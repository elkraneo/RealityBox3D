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
remeasured on 0.3.0, the release this package now pins.

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

## Newton's cradle

`engine-comparison cradle` runs Apple's "Simulating physics joints in your RealityKit app" scene
with Apple's values on RealityKit, on RealityBox3D (its sample's scene, with `sequentialImpacts`)
and on RealitySDFPhysics 0.3.0 (its Pendulum sample's scene, with the contact margin and
restitution threshold that sample sizes for the balls), using RealitySDFPhysics's metrics: release
the first k balls from 30° and record each ball's farthest swing over the first swing out; push the
first ball with Apple's impulse; and send a free row of five balls, awake and asleep, the first at
1 m/s. These measure behavior, not time, so they do not depend on load.
`results-2026-10-16-m1-cradle.txt`:

| Case | A real cradle | RealityBox3D | RealitySDFPhysics | RealityKit |
| --- | --- | --- | --- | --- |
| Lift 1 | ball 5 out, others still | 29.8°, others ≤ 0.3° | 29.8°, others ≤ 0.1° | all five 5.9–7.7° |
| Lift 2 | balls 4–5 out | 29.8° / 29.8°, others ≤ 0.3° | 29.4° / 30.1°, others ≤ 0.3° | all five 11.9–14.3° |
| Lift 3 | balls 3–5 out | 29.8° / 29.8° / 30.0°, others ≤ 0.3° | 29.0° / 29.9° / 30.5°, others ≤ 0.9° | all five 17.7–17.9° |
| Push | ball 1 to 48°, then ball 5 out | ball 5 48.1°, balls 2–4 ≤ 0.8° | ball 5 48.0°, balls 2–4 ≤ 0.1° | balls 2–5 out together, 18.9–24.3° |
| Free row, awake or asleep | last ball 1.000 m/s, the rest 0 | 1.000, the rest 0.000 | 1.000, the rest 0.000 | −0.630, 0.367, 0.416, 0.416, 0.432 |

Box3D's solver alone shares momentum like RealityKit's (one ball in sends three out at 12°);
`sequentialImpacts` resolves the impacts pair by pair before each step. RealitySDFPhysics 0.3.0
does the same in its own solver with its sequential collision pass. Its bounces don't yet see
joints, a gap its README lists.

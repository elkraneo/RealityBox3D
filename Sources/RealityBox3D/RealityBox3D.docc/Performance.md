# Performance

What the bridge between RealityKit and Box3D costs, and how to keep it small.

## Measured cost

`b3-bench` times Box3D's own benchmark scenes in raw C and through RealityBox3D. Box3D runs at the
same speed in both. The bridge adds about 0.7 µs per body per frame to read entities in, and about
the same per awake body to write poses out, on an Apple M1.

Inside RealityKit, ``B3PhysicsSystem`` reads only the entities RealityKit reports as changed and
ignores the world's own writes, so a scene at rest costs almost nothing: 5,431 sleeping dominoes
sync in 0.004 ms per frame. ``B3World/update(entities:deltaTime:)``, for headless use, reads every
entity instead.

## Keep it small

- Make bodies direct children of the simulation root, or of parents with uniform scale. Each parent
  frame is then computed once per frame and composed with plain math. Non-uniform scale in between
  falls back to RealityKit's per-entity conversions.
- Let bodies sleep. Writing back only touches bodies Box3D reports as moved.
- Avoid editing components every frame. Edits apply through Box3D's setters, but a new shape or a
  new entity scale rebuilds the body's shapes.
- Use ``B3PhysicsSimulationComponent/workerCount`` up to the number of performance cores for large
  scenes; Box3D gains little from efficiency cores.

# RealityBox3D

Box3D rigid body physics for RealityKit, with an API that mirrors RealityKit's physics components.

> Erin Catto. **Box3D**, a 3D physics engine for games, the successor of Box2D. MIT License.
> [Repository](https://github.com/erincatto/box3d) · [Announcement](https://x.com/erin_catto/status/2072326622590730604)

Box3D is vendored unmodified as C (see [UPSTREAM.md](UPSTREAM.md)), so your project needs no C
interoperability settings. All credit for the engine goes to Erin Catto and Box3D's contributors.

Box3D brings what RealityKit's physics doesn't expose: a deterministic soft-step solver with
substeps, continuous collision including bullets, rolling resistance, motion locks, sleep control,
exact forecasting of a simulation, and its own convex hulls, meshes and Box3D's sample scenes.

Status: 0.1, early. Box3D upstream is alpha and pinned to one commit.

### Siblings

[RealitySDFPhysics](https://github.com/elkraneo/RealitySDFPhysics) is the same idea for signed
distance fields: exact contact for concave and interlocking shapes such as gears, chains and meshes
of any shape, which convex engines approximate. RealityBox3D is for many convex and compound bodies,
fast stable joints, vehicles, ragdolls and characters. The two share their conventions (an engine
prefix on RealityKit's own type names, `entity.b3` and `entity.sdf`, the same samples and docs) and
can run side by side in one RealityKit scene, each stepping its own entities.

## Usage

```swift
import RealityBox3D
import RealityKit

RealityBox3D.register() // once, before creating entities

// A root entity runs the simulation for its descendants, in its local space
let root = Entity()
root.components.set(B3PhysicsSimulationComponent())
content.add(root)

// Static ground
let ground = Entity()
ground.position = [0, -0.5, 0]
ground.components.set(B3PhysicsBodyComponent(mode: .static))
ground.components.set(B3CollisionComponent(shapes: [.generateBox(size: [10, 1, 10])]))
root.addChild(ground)

// A die: Box3D's convex hull of its meshes, plastic density
let die = try await Entity(named: "D20")
die.components.set(B3PhysicsBodyComponent(density: 1200, material: .generate(dynamicFriction: 0.5, restitution: 0.35)))
die.components.set(B3CollisionComponent(shapes: [.generateConvex(from: die)]))
die.position = [0, 1, 0]
root.addChild(die)

// Forces and events, as with RealityKit's physics
die.b3.applyLinearImpulse([0, 0.02, 0], relativeTo: nil)
let world = RealityBox3D.world(for: die)!   // after the first frame
subscription = world.subscribe(to: B3CollisionEvents.Began.self, on: die) { event in
    print("hit \(event.entityB.name)")
}

// Where will it land? An exact copy of the world, stepped ahead
if let forecast = world.forecast() {
    forecast.step(120)
    print(forecast.transform(of: die)!.translation)
}
```

| RealityKit | RealityBox3D |
| --- | --- |
| `PhysicsSimulationComponent` | `B3PhysicsSimulationComponent`: gravity, substeps, fixed rate, workers, sleeping, continuous |
| `PhysicsBodyComponent` | `B3PhysicsBodyComponent`: mode, mass or density, material, locks, damping, gravity, CCD |
| `CollisionComponent` | `B3CollisionComponent`: shapes, `.default` / `.trigger`, RealityKit's own `CollisionFilter` |
| `ShapeResource` | `B3ShapeResource`: box, sphere, capsule, convex, static mesh, height field, Box3D's cylinder, cone, rock |
| `PhysicsMotionComponent` | `B3PhysicsMotionComponent` |
| `PhysicsMaterialResource` | `B3PhysicsMaterial`, plus rolling resistance |
| `addForce`, `applyLinearImpulse`, … | `entity.b3.addForce`, … |
| `CollisionEvents`, `PhysicsSimulationEvents` | `B3CollisionEvents` (plus `Hit`), `B3PhysicsSimulationEvents` |
| `PhysicsRevoluteJoint`, `PhysicsPrismaticJoint`, `PhysicsSphericalJoint` | `B3PhysicsRevoluteJoint`, … with Box3D motors and springs |
| `PhysicsFixedJoint`, `PhysicsDistanceJoint`, `PhysicsJointsComponent` | `B3PhysicsFixedJoint`, `B3PhysicsDistanceJoint`, `B3PhysicsJointsComponent` |
| — | `B3PhysicsWheelJoint`, `B3PhysicsParallelJoint`, `B3PhysicsMotorJoint`, `B3PhysicsFilterJoint` |
| `CharacterControllerComponent`, `moveCharacter(by:deltaTime:relativeTo:)` | `B3CharacterControllerComponent`, `entity.b3.moveCharacter(…)`, on Box3D's geometric mover |
| — | `B3CharacterObstacleComponent`: soft obstacles (a push limit), shapes the sweep passes, bodies the character cannot push; `clippedVelocity` in the state; `moveCharacter(by:correction:…)` for positional corrections that are not motion |
| — | `B3PhysicsMaterial.tangentVelocity`: conveyor belts; `joint.constraintHertz`, `joint.constraintDampingRatio`: constraint stiffness |
| `scene.raycast`, `scene.convexCast` | `world.raycast`, `world.convexCast` |
| — | `world.overlaps(of:at:margin:)`: which bodies a shape placed here would touch |
| — | `world.forecast()`: an exact copy of the world to step ahead |
| — | `joint.breakLimit`, `B3JointEvents.Broke`, `world.constraintForce(of:)`, `world.reading(of:)`: breakable joints and live joint state |
| — | `B3CollisionComponent.oneWayDirection`: one-way platforms |
| — | `frictionCombine`, `restitutionCombine`: how touching materials mix, PhysX's average included |
| — | `world.explode(at:radius:impulsePerArea:)` |
| — | `sequentialImpacts`: sphere impacts resolved pair by pair, for Newton's cradles and pool breaks |
| — | `B3ShapeResource.material`, per-triangle `B3TriangleMaterials`, `bakesCompound` for static level geometry |
| — | `entity.b3.contacts`, `overlappingEntities`, `applyWind(_:)` |

`PhysicsBodyMode`, `CollisionFilter`, `CollisionGroup` and `GeometricPin` are RealityKit's own
types. Joints follow RealityKit's axes: revolute joints turn and prismatic joints slide along the
pins' x-axis, and `joint.addToSimulation()` adds them to the simulation root. A pin on a skeletal
joint follows the skeleton's animation; RealityKit resolves such pins only for entities in a
running scene, so they do not attach in a headless `B3World`. The body and
collision components stay separate, as in RealityKit, because Box3D bodies take several shapes.

Disabling an entity takes its body, and the bodies below it, out of the simulation, as RealityKit
does; enabled again, a body continues with the state it had.

## Samples

macOS apps built on the public API, in `Samples/`:

- `Box3DDemos`: 32 scenes of Box3D's sample app rebuilt in the `Box3DDemoScenes` module with
  upstream's positions, sizes, densities, materials and cameras, grouped by upstream category:
  Benchmark (Large Pyramid, Wide Pyramid, Convex Pile, Falling Boxes), Stacking (Card House,
  Jenga Stack, Dominoes, Arch), Shapes (Inclined Plane, Rolling Resistance, Conveyor Belt, Wind), Restitution, Bodies
  (Gyroscopic Torque, Spinning Book, Weeble, Kinematic), Joints (Revolute, Prismatic, Spherical,
  Weld, Bridge, Ball and Chain, Distance Joint, Driving, Door), Ragdoll (upstream's 12-bone humanoid),
  Character (Geometric Mover, on upstream's test level: walk with WASD, jump with space, sprint
  with shift), Continuous (Thin Wall, Bullet vs Stack), Events (Sensor Visit) and Mesh (Grid).
  Drag a body to pull it. `Box3DDemos --scene "Geometric Mover"` opens on one scene.
- `Box3DRealityKitSamples`: Apple's RealityKit physics samples ported to Box3D bodies, keeping Apple's
  structure and values, each listing where it differs:
  - [Simulating physics with collisions in your visionOS app](https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app),
    with a switch from spheres to tori, as in RealitySDFPhysics's port; each torus is a ring of 12
    capsules, since Box3D's bodies are convex.
  - [Simulating dice rolls as a component for your game](https://developer.apple.com/documentation/tabletopkit/simulating-dice-rolls-as-a-component-for-your-game),
    with Apple's seven dice models, each simulated as its exact polyhedron. The roll is forecast with `B3World.forecast()`, so the score
    shows while the dice tumble, and "Max score toss" turns each die's model by a symmetry so its
    highest face lands on top.
  - [Simulating physics joints in your RealityKit app](https://developer.apple.com/documentation/realitykit/simulating-physics-joints-in-your-realitykit-app),
    Apple's Newton's cradle on revolute joints. Box3D's solver, like RealityKit's, shares a striking
    ball's momentum along the row; with `sequentialImpacts`, impacts are resolved pair by pair
    before each step, so one ball in sends one ball out, two send two, and the middle balls stay still.

Both mirror RealitySDFPhysics's `SDFDemos` and `SDFRealityKitSamples`, so the same scenes can be
compared across RealityKit's physics, RealitySDFPhysics and Box3D.

```sh
swift run -c release Box3DDemos
swift run -c release Box3DRealityKitSamples
swift run -c release b3-snapshot out/ [seconds] [scene name...]   # offscreen renders
swift run -c release b3-snapshot out/ --samples                   # the RealityKit sample ports
swift run -c release b3-bench [--threads 1,4] [--runs 3]           # raw C against the bridge
swift test
```

Use release builds: debug builds of Box3D keep its assertions and are much slower.

## Performance

`b3-bench` runs Box3D's own benchmark scenes twice with upstream's method: through upstream's C
scene builders on a bare world, and as RealityKit entities through RealityBox3D, including reading
every entity in and writing every awake body out each frame.

Apple M1, release, 60 Hz with 4 substeps, fastest of 3 runs, total for the run. Measured while
other processes loaded the machine, so absolute times vary by about 20%; the split within a row is
the useful part.

| Scene | Bodies | Threads | Raw C | Bridge | Box3D in bridge | Sync in | Sync out |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Large Pyramid, 200 steps | 5,051 | 1 | 1,441 | 3,332 | 1,709 | 720 | 721 |
| Large Pyramid | 5,051 | 4 | 826 | 2,632 | 992 | 722 | 735 |
| Convex Pile, 500 steps | 5,121 | 1 | 5,510 | 10,901 | 6,443 | 1,972 | 1,975 |
| Convex Pile | 5,121 | 4 | 2,371 | 7,056 | 2,925 | 1,905 | 1,746 |

Times in ms. Upstream's M2 Air reference for Large Pyramid is 1,811 ms on one thread.

- Box3D runs at the same speed inside the bridge as in raw C.
- `b3-bench` drives `B3World.update(entities:)`, which reads every entity each frame: about 0.7 µs
  per body each way, 1.4 ms per frame for 1,000 awake bodies.
- Inside RealityKit, `B3PhysicsSystem` reads only entities that RealityKit's change events report
  as changed, ignoring the world's own pose writes, and writes back only bodies Box3D reports as
  moved. 5,431 sleeping dominoes cost 0.004 ms per frame to sync, against 5.2 ms for a full scan.

`Benchmarks/EngineComparison` runs the same piles on RealityKit's physics, RealitySDFPhysics and
RealityBox3D. On an M1, 400 tori made of capsules take 3.6 ms per step in Box3D with 4 workers,
13 ms in RealityKit and 9.7 ms as true tori in RealitySDFPhysics; 1,600 boxes take 2.1, 5.5 and 33 ms.

## Design

- **Mirrored, not replaced.** RealityKit simulates anything carrying its own physics components,
  so RealityBox3D uses parallel types. Give an entity one or the other.
- **Transforms.** Dynamic bodies write their pose to the entity. Moving a static or dynamic entity
  yourself teleports its body. Kinematic bodies follow their entity through Box3D target transforms.
- **Fixed steps.** 60 Hz by default, 4 substeps, at most 4 steps per frame, with poses interpolated
  for 90 Hz displays. `B3PhysicsSimulationEvents` run around every step.
- **Edits apply in place.** Changing a component calls Box3D's setters, and so does changing a
  joint's limits, motor or springs; only a joint whose pins or type change is rebuilt. Only new geometry or a new
  entity scale replaces the shapes, and the body, its contacts and its velocity survive.
- **Space.** Bodies simulate in the space of the nearest ancestor with `B3PhysicsSimulationComponent`,
  or scene space without one. Entity scale is baked into shapes.
- **Units.** Box3D's tolerances, such as its 5 mm contact slop, suit bodies from about 0.1 to 10
  units across. For small objects such as 2 cm dice, model them in centimeters under a root scaled
  by 0.01, and leave `RealityBox3D.lengthUnitsPerMeter` at 1: Box3D then treats a die like a 2 m
  object, with a slop of 0.05 mm. Raise it only for models that are real-world size in small units,
  such as a 1 m barrel modeled as 100 cm. The setting is process-wide.
- **Deterministic.** Box3D is cross-platform deterministic. `B3World.forecast()` copies a world from
  the snapshot that opens a Box3D recording, contacts and warm starts included, and steps
  identically: the tests check equality bit for bit.

## Known gaps

- The character controller ignores `stepLimit`: Box3D's mover has no step-up, so a capsule rolls up
  steps lower than its radius, slowly. Upstream's sample climbs stairs with a sprung ray under the
  capsule instead, which the Geometric Mover scene ports as game code.
- Height fields have no holes, so the Geometric Mover's wave terrain is solid where upstream's has some.
- A body has one density, so shapes of one body cannot differ in density.
- `convexCast` translates the shape without rotating it, as Box3D's shape casts do.
- Multithreading uses Box3D's internal scheduler, not Dispatch.
- `staticFriction` is ignored: Box3D has one friction coefficient.
- Triangle meshes collide only on static bodies, a Box3D rule.
- Box3D hulls have at most 128 faces and 128 edges, so rounded hulls are simplified to fit.
- Box3D's length unit is process-wide, not per world.
- One-way surfaces judge each contact by its normal alone: a body pushed sideways into a
  platform's edge from below can be lifted onto it. On meshes and height fields Box3D
  filters only fast bodies' continuous-collision hits; their regular contacts always collide.
- Forecasts do not break joints, and explosions push spheres, capsules and hulls, not meshes.
- A broken joint reports its load at the end of the step it broke in, not the peak that broke it.
- `sequentialImpacts` covers spheres striking spheres, capsules and hulls, and runs Box3D's substeps
  as steps of their own. Impacts slower than the restitution threshold, 1 m/s by default, are left
  to Box3D: lower it for small, slow balls modeled in meters. So are one-way shapes, bodies with
  locked axes, shapes turning fast, and bodies held by joints, except free pendulums (one revolute
  or spherical joint to a static body, without limits, motors, springs or a break limit), which
  the pass turns about their pivot.
- The apps run on macOS. The library builds for iOS and visionOS but has not run on a device.

## License

MIT for this package. Box3D is MIT, by Erin Catto (`Sources/CBox3D/LICENSE.box3d`). The dice
models are Apple's, under `Samples/Box3DRealityKitSampleScenes/Dice/Models/LICENSE.txt`. The
Geometric Mover's level meshes are upstream's (`box3d/data/meshes`), under Box3D's license.

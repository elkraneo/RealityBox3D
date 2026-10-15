# Migrating from RealityKit Physics

Move code from RealityKit's physics components to Box3D bodies.

## Rename the types

| RealityKit | RealityBox3D |
| --- | --- |
| `PhysicsSimulationComponent` | ``B3PhysicsSimulationComponent`` |
| `PhysicsBodyComponent` | ``B3PhysicsBodyComponent`` |
| `CollisionComponent` | ``B3CollisionComponent`` |
| `ShapeResource` | ``B3ShapeResource`` |
| `PhysicsMotionComponent` | ``B3PhysicsMotionComponent`` |
| `PhysicsMaterialResource` | ``B3PhysicsMaterial`` |
| `entity.addForce(_:relativeTo:)` | `entity.b3.addForce(_:relativeTo:)` |
| `CollisionEvents.Began` | ``B3CollisionEvents/Began`` |
| `PhysicsSimulationEvents.WillSimulate` | ``B3PhysicsSimulationEvents/WillSimulate`` |
| `scene.raycast(origin:direction:length:query:mask:relativeTo:)` | ``B3World/raycast(origin:direction:length:query:mask:)`` |

| `PhysicsRevoluteJoint` | ``B3PhysicsRevoluteJoint`` |
| `PhysicsPrismaticJoint` | ``B3PhysicsPrismaticJoint`` |
| `PhysicsSphericalJoint` | ``B3PhysicsSphericalJoint`` |
| `PhysicsFixedJoint` | ``B3PhysicsFixedJoint`` |
| `PhysicsDistanceJoint` | ``B3PhysicsDistanceJoint`` |
| `PhysicsJointsComponent` | ``B3PhysicsJointsComponent`` |

`PhysicsBodyMode`, `CollisionFilter`, `CollisionGroup` and `GeometricPin` stay RealityKit's own.

Joints keep RealityKit's axes: a revolute joint turns, and a prismatic joint slides, along the pins'
x-axis, and a spherical joint's limit is a cone around pin0's x-axis. Box3D's cone is circular, so
an elliptical `angularLimitInYZ` opens to its larger angle. A fixed joint is Box3D's weld, a stiff
but soft constraint that can bend a few centimeters under hard impacts and springs back.

## Differences

- Give an entity RealityKit's physics components or Box3D's, never both.
- Box3D has one friction coefficient: ``B3PhysicsMaterial/dynamicFriction`` is used.
- Box3D steps at a fixed rate with substeps rather than position and velocity iterations.
- ``B3ShapeResource`` is a plain value: its geometry is readable and comparable, and Box3D builds it
  when the body is created. `generateConvex(from:)` accepts points, a `MeshResource` or an entity.
- Triangle meshes collide only on static bodies.
- Box3D adds rolling resistance, motion locks, bullets, sleep control, hit events, joint motors and
  springs, and ``B3World/forecast()``.

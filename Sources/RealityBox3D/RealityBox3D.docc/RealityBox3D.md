# ``RealityBox3D``

Box3D, Erin Catto's 3D physics engine, inside RealityKit, with an API that mirrors RealityKit's
physics components.

## Overview

RealityBox3D runs Box3D's deterministic soft-step solver on RealityKit entities. Box3D is vendored
unmodified as C; the Swift layer mirrors RealityKit's physics types with a `B3` prefix, so moving
code over is mostly a rename.

```swift
RealityBox3D.register()

let root = Entity()
root.components.set(B3PhysicsSimulationComponent())

let box = ModelEntity(mesh: .generateBox(size: 0.2))
box.components.set(B3PhysicsBodyComponent(density: 1000))
box.components.set(B3CollisionComponent(shapes: [.generateBox(size: [0.2, 0.2, 0.2])]))
root.addChild(box)
```

## Topics

### Essentials

- <doc:GettingStarted>
- <doc:MigratingFromRealityKitPhysics>
- <doc:Performance>

### Components

- ``B3PhysicsSimulationComponent``
- ``B3PhysicsBodyComponent``
- ``B3CollisionComponent``
- ``B3PhysicsMotionComponent``

### Joints

- ``B3PhysicsJointsComponent``
- ``B3PhysicsRevoluteJoint``
- ``B3PhysicsPrismaticJoint``
- ``B3PhysicsSphericalJoint``
- ``B3PhysicsFixedJoint``
- ``B3PhysicsDistanceJoint``
- ``B3JointMotor``
- ``B3JointSpring``
- ``B3JointBreakLimit``
- ``B3JointReading``

- ``B3PhysicsWheelJoint``
- ``B3PhysicsParallelJoint``
- ``B3PhysicsMotorJoint``
- ``B3PhysicsFilterJoint``

### Characters

- ``B3CharacterControllerComponent``
- ``B3CharacterControllerStateComponent``
- ``B3CharacterCollision``
- ``B3CharacterObstacleComponent``

### Shapes and materials

- ``B3ShapeResource``
- ``B3PhysicsMaterial``
- ``B3PhysicsMassProperties``
- ``B3TriangleMaterials``
- ``B3MaterialCombine``
- ``B3Contact``

### Simulation

- ``B3World``
- ``B3Forecast``
- ``B3PhysicsSystem``
- ``B3BodyProxy``

### Events

- ``B3CollisionEvents``
- ``B3PhysicsSimulationEvents``
- ``B3JointEvents``
- ``B3Subscription``

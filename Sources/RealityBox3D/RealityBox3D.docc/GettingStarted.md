# Getting Started

Simulate entities with Box3D in a RealityKit scene.

## Register once

Call ``RealityBox3D/register()`` before creating entities. It registers the components and
``B3PhysicsSystem``, which steps every Box3D world each frame.

## Add a simulation root

Entities below an entity that carries ``B3PhysicsSimulationComponent`` simulate in that entity's
space. Without one, they simulate in scene space with default settings.

```swift
let root = Entity()
var settings = B3PhysicsSimulationComponent()
settings.gravity = [0, -9.81, 0]
settings.subStepCount = 4
root.components.set(settings)
content.add(root)
```

## Add bodies

A body needs both ``B3PhysicsBodyComponent`` and ``B3CollisionComponent``, like RealityKit's
`PhysicsBodyComponent` and `CollisionComponent`.

```swift
let ground = Entity()
ground.components.set(B3PhysicsBodyComponent(mode: .static))
ground.components.set(B3CollisionComponent(shapes: [.generateBox(size: [10, 1, 10])]))
ground.position = [0, -0.5, 0]
root.addChild(ground)

let ball = ModelEntity(mesh: .generateSphere(radius: 0.1))
ball.components.set(B3PhysicsBodyComponent(density: 1000, material: .generate(restitution: 0.6)))
ball.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 0.1)]))
ball.position = [0, 2, 0]
root.addChild(ball)
```

## Apply forces and listen for events

Forces go through `entity.b3`, so they never collide with RealityKit's own methods on
`ModelEntity`. Events come from the ``B3World`` that simulates the entity, available after the
system's first frame.

```swift
ball.b3.applyLinearImpulse([0, 1, 0], relativeTo: nil)
if let world = RealityBox3D.world(for: ball) {
    subscription = world.subscribe(to: B3CollisionEvents.Began.self, on: ball) { event in
        print("touched \(event.entityB.name)")
    }
}
```

Keep the returned ``B3Subscription``: releasing it cancels it.

## Small objects

Box3D's tolerances suit bodies from about 0.1 to 10 units across, with a contact slop of 0.005
units. For objects of a few centimeters, simulate in centimeters: scale the root by 0.01 and
express sizes, gravity and densities in centimeters. Leave ``RealityBox3D/lengthUnitsPerMeter`` at
1, so a 2 cm die gets a slop of 0.05 mm. Setting it to 100 would scale the slop back to 5 mm, a
quarter of the die, and dice would sink into the table and rock.

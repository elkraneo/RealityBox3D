import Testing
import RealityKit
@testable import RealityBox3D

/// The README's and the catalog's usage code, compiled and run, so the docs follow the API.
@MainActor @Test func readmeUsageCompilesAndRuns() {
    let root = Entity()
    root.components.set(B3PhysicsSimulationComponent())
    let ground = Entity()
    ground.position = [0, -0.5, 0]
    ground.components.set(B3PhysicsBodyComponent(mode: .static))
    ground.components.set(B3CollisionComponent(shapes: [.generateBox(size: [10, 1, 10])]))
    root.addChild(ground)
    let die = ModelEntity(mesh: .generateBox(size: 0.02))
    die.components.set(B3PhysicsBodyComponent(density: 1200, material: .generate(dynamicFriction: 0.5, restitution: 0.35)))
    die.components.set(B3CollisionComponent(shapes: [.generateConvex(from: die)]))
    die.position = [0, 1, 0]
    root.addChild(die)
    let world = B3World(root: root)
    world.update(entities: [ground, die], deltaTime: 1.0 / 60)
    die.b3.applyLinearImpulse([0, 0.02, 0], relativeTo: nil)
    var began = false
    let subscription = world.subscribe(to: B3CollisionEvents.Began.self, on: die) { _ in began = true }
    let forecast = world.forecast()
    forecast?.step(120)
    #expect(forecast?.transform(of: die) != nil)
    for _ in 0..<120 { world.update(entities: [ground, die], deltaTime: 1.0 / 60) }
    #expect(began)
    subscription.cancel()
}

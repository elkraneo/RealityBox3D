import Testing
import RealityKit
import simd
@testable import RealityBox3D

/// Breakable joints: Box3D's force and torque thresholds as a `breakLimit`.
@MainActor
@Suite struct JointBreakTests {
    /// A 1 m pendulum: a 0.1 m ball of 1000 kg/m³ (about 4.19 kg, 41 N) under a plain anchor.
    func pendulum(_ limit: B3JointBreakLimit?) throws -> (root: Entity, bob: Entity, world: B3World) {
        let root = Entity()
        root.components.set(B3PhysicsSimulationComponent())
        let anchor = Entity()
        anchor.position = [0, 3, 0]
        root.addChild(anchor)
        let bob = Entity()
        bob.position = [0, 2, 0]
        bob.components.set(B3PhysicsBodyComponent(density: 1000))
        bob.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 0.1)]))
        root.addChild(bob)
        var joint = B3PhysicsSphericalJoint(pin0: anchor.pins.set(named: "a"), pin1: bob.pins.set(named: "b", position: [0, 1, 0]))
        joint.breakLimit = limit
        try joint.addToSimulation()
        return (root, bob, B3World(root: root, settings: root.components[B3PhysicsSimulationComponent.self]!))
    }

    func frames(_ w: B3World, _ bob: Entity, _ n: Int) { for _ in 0..<n { w.update(entities: [bob], deltaTime: 1.0 / 60) } }

    @Test func readsTheConstraintForce() throws {
        let (root, bob, w) = try pendulum(nil)
        frames(w, bob, 60)
        let joint = try #require(root.components[B3PhysicsJointsComponent.self]?.joints.first)
        let force = try #require(w.constraintForce(of: joint))
        let weight = try #require(bob.b3.mass) * 9.81
        // Hanging at rest, the joint holds the ball's weight.
        #expect(abs(simd_length(force) - weight) < 0.05 * weight, "force \(force), weight \(weight)")
    }

    @Test func holdsUnderItsLimit() throws {
        let (root, bob, w) = try pendulum(.init(force: 100))
        var broke = 0
        let sub = w.subscribe(to: B3JointEvents.Broke.self) { _ in broke += 1 }
        frames(w, bob, 120)
        #expect(broke == 0)
        #expect(root.components[B3PhysicsJointsComponent.self]?.joints.count == 1)
        #expect(bob.position.y > 1.9, "bob at \(bob.position)")
        _ = sub
    }

    @Test func breaksOverItsLimit() throws {
        let (root, bob, w) = try pendulum(.init(force: 20))
        var events: [B3JointEvents.Broke] = []
        let sub = w.subscribe(to: B3JointEvents.Broke.self) { events.append($0) }
        frames(w, bob, 120)
        #expect(events.count == 1)
        #expect(simd_length(events.first?.force ?? .zero) > 20, "broke at \(String(describing: events.first?.force))")
        #expect(events.first?.joint is B3PhysicsSphericalJoint)
        #expect(root.components[B3PhysicsJointsComponent.self]?.joints.isEmpty == true, "the broken joint stayed in the component")
        #expect(w.jointIds.isEmpty)
        #expect(bob.position.y < 0, "the ball did not fall: \(bob.position)")
        _ = sub
    }

    // A limit lowered in place, through Box3D's setters, breaks the joint at the next step.
    @Test func lowersItsLimitInPlace() throws {
        let (root, bob, w) = try pendulum(.init(force: 100))
        frames(w, bob, 30)
        let built = w.jointIds
        var component = try #require(root.components[B3PhysicsJointsComponent.self])
        var joint = try #require(component.joints.first as? B3PhysicsSphericalJoint)
        joint.breakLimit = .init(force: 20)
        component.joints[0] = joint
        root.components.set(component)
        var broke = 0
        let sub = w.subscribe(to: B3JointEvents.Broke.self) { _ in broke += 1 }
        frames(w, bob, 1)
        #expect(built.count == 1)
        frames(w, bob, 10)
        #expect(broke == 1)
        _ = sub
    }

    // A breaking joint between two bodies: the other joints keep working, by their own index.
    @Test func otherJointsSurviveABreak() throws {
        let (root, bob, w) = try pendulum(.init(force: 20))
        let second = Entity()
        second.position = [1, 2, 0]
        second.components.set(B3PhysicsBodyComponent(density: 1000))
        second.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 0.1)]))
        root.addChild(second)
        let anchor = Entity()
        anchor.position = [1, 3, 0]
        root.addChild(anchor)
        try B3PhysicsSphericalJoint(pin0: anchor.pins.set(named: "a2"), pin1: second.pins.set(named: "b2", position: [0, 1, 0]))
            .addToSimulation()
        for _ in 0..<120 { w.update(entities: [bob, second], deltaTime: 1.0 / 60) }
        #expect(root.components[B3PhysicsJointsComponent.self]?.joints.count == 1)
        #expect(bob.position.y < 0)
        #expect(abs(simd_distance(second.position, [1, 3, 0]) - 1) < 0.02, "second at \(second.position)")
    }
}

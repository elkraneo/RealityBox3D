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

    /// Infinite and NaN limits never break, a negative one breaks at once; none reaches Box3D,
    /// whose debug build asserts on them.
    @Test func oddLimitsAreSafe() throws {
        for (limit, breaks) in [(Float.infinity, false), (.nan, false), (-1, true)] {
            let (root, bob, w) = try pendulum(.init(force: limit, torque: limit))
            frames(w, bob, 10)
            #expect((root.components[B3PhysicsJointsComponent.self]?.joints.isEmpty ?? true) == breaks, "limit \(limit)")
            // And through the in-place update
            let (root2, bob2, w2) = try pendulum(.init(force: 100))
            frames(w2, bob2, 2)
            var j = root2.components[B3PhysicsJointsComponent.self]!.joints[0] as! B3PhysicsSphericalJoint
            j.breakLimit = .init(force: limit)
            root2.components.set(B3PhysicsJointsComponent(joints: [j]))
            frames(w2, bob2, 10)
            #expect((root2.components[B3PhysicsJointsComponent.self]?.joints.isEmpty ?? true) == breaks, "updated to \(limit)")
        }
    }

    /// A forecast breaks joints as the world does: both drop the ball.
    @Test func forecastsBreakJoints() throws {
        // Too weak for the 41 N ball: it breaks on the first step
        let (root, bob, w) = try pendulum(.init(force: 20))
        defer { withExtendedLifetime(root) {} } // the world holds its root weakly
        w.update(entities: [bob], deltaTime: 0)
        // Before any step, so the joint can only break inside the forecast
        #expect(w.jointIds.count == 1)
        let forecast = try #require(w.forecast())
        forecast.step(120)
        // Step for step: frames of 1/60 s don't add up to whole fixed steps exactly
        var events: [any B3CollisionEvent] = []
        for _ in 0..<120 { w.step(events: &events) }
        let ahead = try #require(forecast.transform(of: bob)).translation
        #expect(w.jointIds.isEmpty, "the joint held in the world")
        #expect(ahead.y < 0, "the forecast kept the ball hanging at \(ahead)")
        #expect(simd_distance(ahead, bob.position) < 0.05, "forecast \(ahead), world \(bob.position)")
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
        // Box3D's hanging load is steady here, so the lowered limit is checked against it first
        // by the same joint, updated in place.
        frames(w, bob, 1)
        #expect(built.count == 1)
        #expect(broke == 1)
        #expect(w.jointIds.isEmpty)
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

    // A handler that edits the breaking joint during the frame does not bring it back.
    @Test func editedWhileBreakingStaysBroken() throws {
        let (root, bob, w) = try pendulum(.init(force: 20))
        let sub = w.subscribe(to: B3PhysicsSimulationEvents.DidSimulate.self) { _ in
            guard var c = root.components[B3PhysicsJointsComponent.self], var j = c.joints.first as? B3PhysicsSphericalJoint else { return }
            j.friction = (j.friction ?? 0) + 0.01
            c.joints[0] = j
            root.components.set(c)
        }
        frames(w, bob, 60)
        #expect(root.components[B3PhysicsJointsComponent.self]?.joints.isEmpty == true)
        #expect(w.jointIds.isEmpty)
        #expect(bob.position.y < 0)
        _ = sub
    }

    // Two joints breaking in the same step both go, and a third, stronger one stays.
    @Test func severalBreakAtOnce() throws {
        let root = Entity()
        root.components.set(B3PhysicsSimulationComponent())
        var bobs: [Entity] = []
        for (i, limit) in [Float(20), 20, 1000].enumerated() {
            let anchor = Entity()
            anchor.position = [Float(i) * 2, 3, 0]
            root.addChild(anchor)
            let bob = Entity()
            bob.position = [Float(i) * 2, 2, 0]
            bob.components.set(B3PhysicsBodyComponent(density: 1000))
            bob.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 0.1)]))
            root.addChild(bob)
            bobs.append(bob)
            var j = B3PhysicsSphericalJoint(pin0: anchor.pins.set(named: "a\(i)"), pin1: bob.pins.set(named: "b\(i)", position: [0, 1, 0]))
            j.breakLimit = .init(force: limit)
            try j.addToSimulation()
        }
        let w = B3World(root: root, settings: root.components[B3PhysicsSimulationComponent.self]!)
        var broke = 0
        let sub = w.subscribe(to: B3JointEvents.Broke.self) { _ in broke += 1 }
        for _ in 0..<60 { w.update(entities: bobs, deltaTime: 1.0 / 60) }
        #expect(broke == 2)
        let left = root.components[B3PhysicsJointsComponent.self]?.joints ?? []
        #expect(left.count == 1 && left.first?.breakLimit?.force == 1000)
        #expect(abs(simd_distance(bobs[2].position, [4, 3, 0]) - 1) < 0.02)
        _ = sub
    }

    // Handlers that reorder the joints every step never see the broken one, so it cannot take
    // another joint's place.
    @Test func reorderingDuringBreaksKeepsTheRightJoints() throws {
        let root = Entity()
        root.components.set(B3PhysicsSimulationComponent())
        var bobs: [Entity] = []
        for (i, limit) in [Float(20), 1000, 1000].enumerated() {
            let anchor = Entity()
            anchor.position = [Float(i) * 2, 3, 0]
            root.addChild(anchor)
            let bob = Entity()
            bob.position = [Float(i) * 2, 2, 0]
            bob.components.set(B3PhysicsBodyComponent(density: 1000))
            bob.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 0.1)]))
            root.addChild(bob)
            bobs.append(bob)
            var j = B3PhysicsSphericalJoint(pin0: anchor.pins.set(named: "a\(i)"), pin1: bob.pins.set(named: "b\(i)", position: [0, 1, 0]))
            j.breakLimit = .init(force: limit)
            try j.addToSimulation()
        }
        let w = B3World(root: root, settings: root.components[B3PhysicsSimulationComponent.self]!)
        let sub = w.subscribe(to: B3PhysicsSimulationEvents.DidSimulate.self) { _ in
            guard var c = root.components[B3PhysicsJointsComponent.self] else { return }
            c.joints.reverse()
            root.components.set(c)
        }
        for _ in 0..<60 { w.update(entities: bobs, deltaTime: 1.0 / 60) }
        let left = root.components[B3PhysicsJointsComponent.self]?.joints ?? []
        #expect(left.count == 2 && left.allSatisfy { $0.breakLimit?.force == 1000 })
        #expect(bobs[0].position.y < 0)
        for i in 1...2 {
            #expect(abs(simd_distance(bobs[i].position, [Float(i) * 2, 3, 0]) - 1) < 0.02, "bob \(i) at \(bobs[i].position)")
        }
        _ = sub
    }

    // A joint edited by a WillSimulate handler, then broken in that step, stays broken.
    @Test func editedJustBeforeBreakingStaysBroken() throws {
        let (root, bob, w) = try pendulum(.init(force: 20))
        let sub = w.subscribe(to: B3PhysicsSimulationEvents.WillSimulate.self) { _ in
            guard var c = root.components[B3PhysicsJointsComponent.self], var j = c.joints.first as? B3PhysicsSphericalJoint else { return }
            j.friction = (j.friction ?? 0) + 0.01
            c.joints[0] = j
            root.components.set(c)
        }
        frames(w, bob, 60)
        #expect(root.components[B3PhysicsJointsComponent.self]?.joints.isEmpty == true)
        #expect(w.jointIds.isEmpty)
        #expect(bob.position.y < 0)
        _ = sub
    }

    // Two joints on the same pins, one weak, reordered by a handler: the weak one goes, the
    // strong one stays.
    @Test func duplicatePinsKeepTheRightJoint() throws {
        let (root, bob, w) = try pendulum(.init(force: 20))
        var c = try #require(root.components[B3PhysicsJointsComponent.self])
        var strong = try #require(c.joints.first as? B3PhysicsSphericalJoint)
        strong.breakLimit = .init(force: 1000)
        c.joints.append(strong)
        root.components.set(c)
        let sub = w.subscribe(to: B3PhysicsSimulationEvents.WillSimulate.self) { _ in
            guard var c = root.components[B3PhysicsJointsComponent.self] else { return }
            c.joints.reverse()
            root.components.set(c)
        }
        frames(w, bob, 60)
        let left = root.components[B3PhysicsJointsComponent.self]?.joints ?? []
        #expect(left.count == 1 && left.first?.breakLimit?.force == 1000, "left \(left.map { $0.breakLimit as Any })")
        #expect(w.jointIds.count == 1)
        #expect(bob.position.y > 1.9, "bob at \(bob.position)")
        _ = sub
    }

    // A handler removes the weak duplicate; its constraint, still built this step and sharing the
    // load, breaks: the strong one stays.
    @Test func removedDuplicateBreakingKeepsTheOther() throws {
        let (root, bob, w) = try pendulum(.init(force: 5))
        var c = try #require(root.components[B3PhysicsJointsComponent.self])
        var strong = try #require(c.joints.first as? B3PhysicsSphericalJoint)
        strong.breakLimit = .init(force: 1000)
        c.joints.append(strong)
        root.components.set(c)
        // Both are built at the frame's start; the handler then removes the weak one before the
        // step in which it breaks.
        let sub = w.subscribe(to: B3PhysicsSimulationEvents.WillSimulate.self) { _ in
            guard var c = root.components[B3PhysicsJointsComponent.self] else { return }
            c.joints.removeAll { $0.breakLimit?.force == 5 }
            root.components.set(c)
        }
        frames(w, bob, 60)
        let left = root.components[B3PhysicsJointsComponent.self]?.joints ?? []
        #expect(left.count == 1 && left.first?.breakLimit?.force == 1000, "left \(left.map { $0.breakLimit as Any })")
        #expect(bob.position.y > 1.9, "bob at \(bob.position)")
        _ = sub
    }
}

import Testing
import RealityKit
import simd
@testable import RealityBox3D

/// Contacts, trigger overlaps, wind, joint readings and disabled entities.
@MainActor
@Suite struct BodyReadoutTests {
    func root(gravity: SIMD3<Float> = [0, -9.81, 0]) -> Entity {
        let r = Entity()
        var s = B3PhysicsSimulationComponent()
        s.gravity = gravity
        s.interpolatesPoses = false
        r.components.set(s)
        return r
    }

    @discardableResult
    func body(_ r: Entity, _ shape: B3ShapeResource, at p: SIMD3<Float>, mode: PhysicsBodyMode = .dynamic,
              collisionMode: B3CollisionComponent.Mode = .default) -> Entity {
        let e = Entity()
        e.position = p
        e.components.set(B3PhysicsBodyComponent(density: 1000, mode: mode))
        e.components.set(B3CollisionComponent(shapes: [shape], mode: collisionMode))
        r.addChild(e)
        return e
    }

    func world(_ r: Entity) -> B3World { B3World(root: r, settings: r.components[B3PhysicsSimulationComponent.self]!) }

    @Test func contactsSeenFromEitherBody() throws {
        let r = root()
        let ground = body(r, .generateBox(size: [4, 1, 4]), at: [0, -0.5, 0], mode: .static)
        let box = body(r, .generateBox(size: [0.5, 0.5, 0.5]), at: [0, 0.25, 0])
        let w = world(r)
        for _ in 0..<30 { w.update(entities: [ground, box], deltaTime: 1.0 / 60) }
        let contact = try #require(box.b3.contacts.first)
        #expect(contact.entity === ground)
        // From the box, the normal points down toward the ground; the points lie on the floor.
        #expect(contact.normal.y < -0.99, "normal \(contact.normal)")
        #expect(contact.points.count >= 3 && contact.points.allSatisfy { abs($0.y) < 0.01 }, "points \(contact.points)")
        // The ground holds the box's weight.
        let weight = try #require(box.b3.mass) * 9.81
        #expect(abs(contact.normalForce - weight) < 0.05 * weight, "force \(contact.normalForce), weight \(weight)")
        #expect(ground.b3.contacts.first.map { $0.entity === box && $0.normal.y > 0.99 } == true)
    }

    @Test func triggerListsWhatIsInside() {
        let r = root(gravity: .zero)
        let zone = body(r, .generateBox(size: [2, 2, 2]), at: .zero, mode: .static, collisionMode: .trigger)
        let inside = body(r, .generateSphere(radius: 0.2), at: [0.5, 0, 0])
        let outside = body(r, .generateSphere(radius: 0.2), at: [5, 0, 0])
        let w = world(r)
        for _ in 0..<3 { w.update(entities: [zone, inside, outside], deltaTime: 1.0 / 60) }
        #expect(zone.b3.overlappingEntities.map(\.id) == [inside.id])
        #expect(inside.b3.overlappingEntities.isEmpty)
    }

    @Test func windPushesDownwind() {
        let r = root(gravity: .zero)
        // A 1 kg ball: 8 m/s of air pushes it with about 30 N.
        let ball = body(r, .generateSphere(radius: 0.5), at: .zero)
        ball.components.set(B3PhysicsBodyComponent(mass: 1))
        let w = world(r)
        let sub = w.subscribe(to: B3PhysicsSimulationEvents.WillSimulate.self) { _ in ball.b3.applyWind([0, 0, 8]) }
        for _ in 0..<60 { w.update(entities: [ball], deltaTime: 1.0 / 60) }
        let v = ball.b3.linearVelocity ?? .zero
        #expect(v.z > 3 && v.z < 8 && abs(v.x) < 1e-4 && abs(v.y) < 1e-4, "velocity \(v)")
        _ = sub
    }

    @Test func revoluteReadingFollowsTheMotor() throws {
        let r = root(gravity: .zero)
        let base = Entity()
        r.addChild(base)
        let wheel = body(r, .generateSphere(radius: 0.3), at: .zero)
        let joint = B3PhysicsRevoluteJoint(pin0: base.pins.set(named: "a"), pin1: wheel.pins.set(named: "b"),
                                           motor: B3JointMotor(speed: 2, maxForce: 1000))
        try joint.addToSimulation()
        let w = world(r)
        for _ in 0..<30 { w.update(entities: [wheel], deltaTime: 1.0 / 60) }
        let reading = try #require(w.reading(of: joint))
        // Half a second at 2 rad/s.
        #expect(abs(abs(reading.angle ?? 0) - 1) < 0.05, "angle \(String(describing: reading.angle))")
        #expect(reading.translation == nil)
    }

    @Test func disabledEntityLeavesTheSimulation() {
        let r = root()
        let ground = body(r, .generateBox(size: [4, 1, 4]), at: [0, -0.5, 0], mode: .static)
        let box = body(r, .generateBox(size: [0.5, 0.5, 0.5]), at: [0, 3, 0])
        let w = world(r)
        for _ in 0..<10 { w.update(entities: [ground, box], deltaTime: 1.0 / 60) }
        box.isEnabled = false
        let frozen = box.position
        for _ in 0..<30 { w.update(entities: [ground, box], deltaTime: 1.0 / 60) }
        #expect(box.position == frozen, "a disabled body moved")
        #expect(w.record(for: box)?.isEnabled == false)
        // Enabled again, it falls on from where it was.
        box.isEnabled = true
        for _ in 0..<120 { w.update(entities: [ground, box], deltaTime: 1.0 / 60) }
        #expect(abs(box.position.y - 0.25) < 0.02, "box at \(box.position)")
    }
}

extension BodyReadoutTests {
    // A body disabled while moving resumes with its velocity.
    @Test func disabledBodyKeepsItsMomentum() {
        let r = root(gravity: .zero)
        let ball = body(r, .generateSphere(radius: 0.2), at: .zero)
        ball.components.set(B3PhysicsMotionComponent(linearVelocity: [2, 0, 0], angularVelocity: [0, 1, 0]))
        let w = world(r)
        for _ in 0..<5 { w.update(entities: [ball], deltaTime: 1.0 / 60) }
        let before = (ball.b3.linearVelocity ?? .zero, ball.b3.angularVelocity ?? .zero)
        ball.isEnabled = false
        for _ in 0..<5 { w.update(entities: [ball], deltaTime: 1.0 / 60) }
        ball.isEnabled = true
        w.update(entities: [ball], deltaTime: 0)
        let v = ball.b3.linearVelocity ?? .zero, spin = ball.b3.angularVelocity ?? .zero
        #expect(simd_length(before.0) > 1.9 && simd_length(before.1) > 0.5, "moving before: \(before)")
        #expect(simd_distance(v, before.0) < 1e-4, "velocity \(v), before \(before.0)")
        #expect(simd_distance(spin, before.1) < 1e-4, "spin \(spin), before \(before.1)")
    }
}

extension BodyReadoutTests {
    // Velocities given while disabled, through the motion component or `setVelocity`, are the
    // ones it resumes with.
    @Test(arguments: [false, true]) func editsWhileDisabledApply(throughComponent: Bool) {
        let r = root(gravity: .zero)
        let ball = body(r, .generateSphere(radius: 0.2), at: .zero)
        ball.components.set(B3PhysicsMotionComponent(linearVelocity: [2, 0, 0]))
        let w = world(r)
        for _ in 0..<5 { w.update(entities: [ball], deltaTime: 1.0 / 60) }
        ball.isEnabled = false
        w.update(entities: [ball], deltaTime: 1.0 / 60)
        if throughComponent {
            ball.components.set(B3PhysicsMotionComponent(linearVelocity: [0, 0, -3]))
        } else {
            ball.b3.setVelocity(linear: [0, 0, -3], angular: .zero)
        }
        for _ in 0..<3 { w.update(entities: [ball], deltaTime: 1.0 / 60) }
        ball.isEnabled = true
        w.update(entities: [ball], deltaTime: 0)
        let v = ball.b3.linearVelocity ?? .zero
        #expect(simd_distance(v, [0, 0, -3]) < 1e-4, "velocity \(v)")
    }
}

extension BodyReadoutTests {
    // A disabled kinematic body moved, then given a velocity: it resumes moving at that velocity.
    @Test func disabledKinematicTakesTheLatestVelocity() {
        let r = root(gravity: .zero)
        let k = body(r, .generateBox(size: [0.5, 0.5, 0.5]), at: .zero, mode: .kinematic)
        let w = world(r)
        w.update(entities: [k], deltaTime: 1.0 / 60)
        // Disabled and moved in the same frame.
        k.isEnabled = false
        k.position = [1, 0, 0]
        w.update(entities: [k], deltaTime: 1.0 / 60)
        k.components.set(B3PhysicsMotionComponent(linearVelocity: [0, 2, 0]))
        w.update(entities: [k], deltaTime: 1.0 / 60)
        k.isEnabled = true
        for _ in 0..<30 { w.update(entities: [k], deltaTime: 1.0 / 60) }
        let v = k.b3.linearVelocity ?? .zero
        #expect(simd_distance(v, [0, 2, 0]) < 1e-3, "velocity \(v)")
        #expect(abs(k.position.y - 1) < 0.05 && abs(k.position.x - 1) < 1e-3, "at \(k.position)")
    }
}

/// Disabled entities in the system's incremental path.
@MainActor
@Suite(.serialized) struct DisabledEntitySystemTests {
    @Test func disablingAnAncestorStopsItsBodies() async throws {
        let h = try SystemHarness()
        h.ground()
        let holder = Entity()
        h.root.addChild(holder)
        let box = Entity()
        box.position = [0, 3, 0]
        box.components.set(B3PhysicsBodyComponent(density: 1000))
        box.components.set(B3CollisionComponent(shapes: [.generateBox(size: [0.5, 0.5, 0.5])]))
        holder.addChild(box)
        try await h.frames(5)
        holder.isEnabled = false
        try await h.frames(1)
        let frozen = box.position(relativeTo: h.root)
        try await h.frames(30)
        #expect(box.position(relativeTo: h.root) == frozen, "a body under a disabled ancestor moved")
        holder.isEnabled = true
        try await h.frames(120)
        #expect(box.position(relativeTo: h.root).y < 0.5, "it did not fall on after re-enabling")
    }
}

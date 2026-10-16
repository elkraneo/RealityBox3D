import Testing
import RealityKit
import simd
import CBox3D
@testable import RealityBox3D

/// The sequential impact pass against the rest of the engine: forces, filters, joints, events.
@MainActor
@Suite struct SequentialImpactsTests {
    func root(_ configure: (inout B3PhysicsSimulationComponent) -> Void = { _ in }) -> Entity {
        let r = Entity()
        var s = B3PhysicsSimulationComponent()
        s.gravity = .zero
        s.sequentialImpacts = true
        s.interpolatesPoses = false
        configure(&s)
        r.components.set(s)
        return r
    }

    @discardableResult
    func body(_ r: Entity, _ shape: B3ShapeResource, at p: SIMD3<Float>, mass: Float = 1, mode: PhysicsBodyMode = .dynamic,
              velocity: SIMD3<Float> = .zero, configure: (inout B3CollisionComponent) -> Void = { _ in }) -> Entity {
        let e = Entity()
        e.position = p
        var b = B3PhysicsBodyComponent(mass: mass, material: .generate(staticFriction: 0, dynamicFriction: 0, restitution: 1), mode: mode)
        b.linearDamping = 0
        b.angularDamping = 0
        e.components.set(b)
        var c = B3CollisionComponent(shapes: [shape])
        configure(&c)
        e.components.set(c)
        if velocity != .zero { e.components.set(B3PhysicsMotionComponent(linearVelocity: velocity)) }
        r.addChild(e)
        return e
    }

    func world(_ r: Entity) -> B3World { B3World(root: r, settings: r.components[B3PhysicsSimulationComponent.self]!) }

    // Box3D clears forces after each of its steps; the substeps the pass takes one by one must
    // each get the step's force, or 60 N on 1 kg for 1/60 s adds a quarter of 1 m/s.
    @Test func forcesActOnTheWholeStep() {
        let r = root()
        let ball = body(r, .generateSphere(radius: 0.1), at: .zero)
        let w = world(r)
        w.update(entities: [ball], deltaTime: 1.0 / 60)
        let sub = w.subscribe(to: B3PhysicsSimulationEvents.WillSimulate.self) { _ in
            ball.b3.addForce([60, 0, 0], relativeTo: nil)
            ball.b3.addTorque([0, 0, 0.6], relativeTo: nil)
        }
        w.update(entities: [ball], deltaTime: 1.0 / 60)
        sub.cancel()
        #expect(abs((ball.b3.linearVelocity?.x ?? 0) - 1) < 0.01, "\(ball.b3.linearVelocity ?? .zero)")
        // Solid sphere, I = 2/5 m r² = 0.004: 0.6 N m for 1/60 s gives 2.5 rad/s.
        #expect(abs((ball.b3.angularVelocity?.z ?? 0) - 2.5) < 0.05, "\(ball.b3.angularVelocity ?? .zero)")
    }

    // A path that misses by a centimeter isn't an impact, however fast.
    @Test func aNearMissIsNotAnImpact() {
        let r = root()
        let still = body(r, .generateSphere(radius: 0.1), at: .zero, mode: .static)
        let ball = body(r, .generateSphere(radius: 0.1), at: [-0.2, 0.21, 0], velocity: [100, 0, 0])
        let w = world(r)
        w.update(entities: [still, ball], deltaTime: 1.0 / 60)
        let v = ball.b3.linearVelocity ?? .zero
        #expect(simd_distance(v, [100, 0, 0]) < 0.01, "deflected to \(v)")
    }

    // Two balls a filter joint keeps apart pass through each other.
    @Test func filterJointsKeepBallsApart() {
        let r = root()
        let a = body(r, .generateSphere(radius: 0.1), at: [-0.25, 0, 0], velocity: [2, 0, 0])
        let b = body(r, .generateSphere(radius: 0.1), at: [0, 0, 0])
        try? B3PhysicsFilterJoint(pin0: a.pins.set(named: "a", position: .zero), pin1: b.pins.set(named: "b", position: .zero)).addToSimulation()
        let w = world(r)
        for _ in 0..<30 { w.update(entities: [a, b], deltaTime: 1.0 / 60) }
        #expect(a.position.x > 0.5, "a stopped at \(a.position)")
        #expect(abs(b.position.x) < 1e-3, "b was struck to \(b.position)")
    }

    // A ball thrown up passes through a one-way platform's underside.
    @Test func oneWayPlatformsLetBallsThrough() {
        let r = root()
        let platform = body(r, .generateBox(size: [4, 0.2, 4]), at: [0, 1, 0], mode: .static) { $0.oneWayDirection = [0, 1, 0] }
        let ball = body(r, .generateSphere(radius: 0.2), at: [0, 0.5, 0], velocity: [0, 5, 0])
        let w = world(r)
        for _ in 0..<30 { w.update(entities: [platform, ball], deltaTime: 1.0 / 60) }
        #expect(ball.position.y > 2, "the ball stopped at \(ball.position)")
    }

    // A ball welded to a static anchor is part of it: a ball striking it bounces back, not
    // stopped by an impulse the weld then takes away.
    @Test func weldedTargetsReturnTheBall() {
        let r = root()
        let anchor = body(r, .generateBox(size: [0.1, 0.1, 0.1]), at: [0, 1, 0], mode: .static)
        let target = body(r, .generateSphere(radius: 0.1), at: .zero)
        let ball = body(r, .generateSphere(radius: 0.1), at: [-0.3, 0, 0], velocity: [2, 0, 0])
        try? B3PhysicsFixedJoint(pin0: anchor.pins.set(named: "a", position: [0, -1, 0]),
                                 pin1: target.pins.set(named: "b", position: .zero)).addToSimulation()
        let w = world(r)
        for _ in 0..<30 { w.update(entities: [anchor, target, ball], deltaTime: 1.0 / 60) }
        // The pass leaves it to Box3D, whose weld gives back part of the speed (1.1 of 2 m/s alone).
        #expect((ball.b3.linearVelocity?.x ?? 0) < -0.5, "the ball left at \(ball.b3.linearVelocity ?? .zero)")
    }

    // A pendulum struck along its string doesn't swing; struck across it, it takes the ball's speed.
    @Test func pendulumsTurnAboutTheirPivot() {
        let r = root()
        let anchor = body(r, .generateBox(size: [0.1, 0.1, 0.1]), at: [0, 1, 0], mode: .static)
        let bob = body(r, .generateSphere(radius: 0.1), at: .zero)
        let ball = body(r, .generateSphere(radius: 0.1), at: [0, -0.3, 0], velocity: [0, 2, 0])
        let hinge = simd_quatf(from: [1, 0, 0], to: [0, 0, 1])
        try? B3PhysicsRevoluteJoint(pin0: anchor.pins.set(named: "a", position: .zero, orientation: hinge),
                                    pin1: bob.pins.set(named: "b", position: [0, 1, 0], orientation: hinge)).addToSimulation()
        let w = world(r)
        for _ in 0..<20 { w.update(entities: [anchor, bob, ball], deltaTime: 1.0 / 60) }
        #expect((ball.b3.linearVelocity?.y ?? 0) < -1.9, "the ball left at \(ball.b3.linearVelocity ?? .zero)")
        #expect(simd_length(bob.position) < 1e-2, "the bob moved to \(bob.position)")
    }

    // Impacts the pass resolves still reach collision subscribers as hits.
    @Test func resolvedImpactsAreHits() {
        let r = root()
        let a = body(r, .generateSphere(radius: 0.1), at: [-0.2, 0, 0], velocity: [3, 0, 0]) { $0.reportsContacts = true }
        let b = body(r, .generateSphere(radius: 0.1), at: .zero) { $0.reportsContacts = true }
        let w = world(r)
        var hits: [Float] = []
        let sub = w.subscribe(to: B3CollisionEvents.Hit.self) { hits.append($0.approachSpeed) }
        for _ in 0..<5 { w.update(entities: [a, b], deltaTime: 1.0 / 60) }
        sub.cancel()
        #expect(hits.count == 1 && abs((hits.first ?? 0) - 3) < 0.01, "hits \(hits)")
        #expect(abs((b.b3.linearVelocity?.x ?? 0) - 3) < 0.01)
    }

    // A small cube struck off center spins: its inertia is tiny but real.
    @Test func smallBodiesSpinWhenStruckOffCenter() {
        let r = root()
        let cube = body(r, .generateBox(size: [0.02, 0.02, 0.02]), at: .zero)
        let ball = body(r, .generateSphere(radius: 0.01), at: [-0.03, 0.008, 0], velocity: [2, 0, 0])
        let w = world(r)
        for _ in 0..<3 { w.update(entities: [cube, ball], deltaTime: 1.0 / 60) }
        #expect(abs(cube.b3.angularVelocity?.z ?? 0) > 1, "spin \(cube.b3.angularVelocity ?? .zero)")
    }
}

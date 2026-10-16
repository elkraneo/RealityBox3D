import Testing
import RealityKit
import simd
import Box3DRealityKitSampleScenes
@testable import RealityBox3D

/// The Newton's cradle sample: Apple's values, driven headlessly.
@MainActor
@Suite(.serialized) struct NewtonsCradleTests {
    /// Releases the first `count` balls from 30° and returns each ball's farthest swing toward +x
    /// and toward -x from the first impact until the struck balls turn back.
    func swing(lift count: Int, sequential: Bool = true) -> (peak: [Float], trough: [Float]) {
        let scene = NewtonsCradleScene()
        scene.root.components[B3PhysicsSimulationComponent.self]!.sequentialImpacts = sequential
        let world = B3World(root: scene.root, settings: scene.root.components[B3PhysicsSimulationComponent.self]!)
        scene.lift(count)
        var peak = [Float](repeating: -90, count: 5), trough = [Float](repeating: 90, count: 5), impact: Int?
        for frame in 0..<150 {
            world.update(entities: scene.bodies, deltaTime: 1.0 / 60)
            if impact == nil, scene.angle(of: 0) > -2 { impact = frame }
            // Slow motion: a quarter period of the 60 cm pendulum is about 0.4 s.
            guard let impact, frame <= impact + 36 else { continue }
            for i in 0..<5 {
                peak[i] = max(peak[i], scene.angle(of: i))
                trough[i] = min(trough[i], scene.angle(of: i))
            }
        }
        return (peak, trough)
    }

    @Test(arguments: 1...3) func liftedBallsSendTheSameNumberOut(count: Int) {
        let (peak, trough) = swing(lift: count)
        for i in 0..<5 {
            if i >= 5 - count {
                #expect(peak[i] > 28, "ball \(i) of \(peak) should swing out")
            } else {
                // A lifted ball swings in from -x; one at rest must stay, either way.
                let farthest = i < count ? peak[i] : max(peak[i], -trough[i])
                #expect(farthest < 1.5, "ball \(i) of \(peak), \(trough) should stay")
            }
        }
    }

    // Box3D's solver alone shares the momentum: the case the sequential pass exists for.
    @Test func withoutSequentialImpactsTheRowShares() {
        let peak = swing(lift: 1, sequential: false).peak
        #expect(peak[1] > 3 || peak[2] > 3 || peak[3] > 3, "\(peak)")
    }

    // Apple's push swings the first ball to about 48°.
    @Test func pushSwingsLikeApples() {
        let scene = NewtonsCradleScene()
        let world = B3World(root: scene.root, settings: scene.root.components[B3PhysicsSimulationComponent.self]!)
        world.update(entities: scene.bodies, deltaTime: 1.0 / 60)
        scene.push()
        var farthest: Float = 0
        for _ in 0..<40 {
            world.update(entities: scene.bodies, deltaTime: 1.0 / 60)
            farthest = min(farthest, scene.angle(of: 0))
        }
        #expect(abs(-farthest - 48.6) < 1.5, "swung to \(-farthest)°")
    }

    // The row falls asleep while the pushed ball swings out; struck asleep, it passes the impact on.
    @Test func sleepingRowPassesThePush() {
        let scene = NewtonsCradleScene()
        let world = B3World(root: scene.root, settings: scene.root.components[B3PhysicsSimulationComponent.self]!)
        for _ in 0..<90 { world.update(entities: scene.bodies, deltaTime: 1.0 / 60) }
        #expect(!scene.balls[2].b3.isAwake, "the row should be asleep before the push")
        scene.push()
        var peak = [Float](repeating: -90, count: 5), back = false
        for _ in 0..<150 {
            world.update(entities: scene.bodies, deltaTime: 1.0 / 60)
            if scene.angle(of: 0) > -2 { back = true }
            guard back else { continue }
            for i in 1..<5 { peak[i] = max(peak[i], abs(scene.angle(of: i))) }
        }
        #expect(peak[4] > 40, "the last ball should swing out: \(peak)")
        #expect(peak[1..<4].allSatisfy { $0 < 1.5 }, "the middle should stay, either way: \(peak)")
    }

    // A ball dragged to 30° by hand and let go swings into the row like a lifted one.
    @Test func draggedBallStrikesLikeALiftedOne() {
        let scene = NewtonsCradleScene()
        let world = B3World(root: scene.root, settings: scene.root.components[B3PhysicsSimulationComponent.self]!)
        world.update(entities: scene.bodies, deltaTime: 1.0 / 60)
        let ball = scene.balls[0]
        #expect(scene.grab(ball, at: ball.position(relativeTo: scene.root)))
        // The pivot in world space, and the point 30° out toward -x, moved there over a second.
        let pivot = scene.root.children[0].convert(position: [ball.position.x, 30, 0], to: nil)
        let length = simd_distance(pivot, ball.position(relativeTo: nil))
        let a: Float = -30 * .pi / 180
        let start = ball.position(relativeTo: nil), end = pivot + length * SIMD3(sin(a), -cos(a), 0)
        for i in 1...60 {
            scene.drag(toWorld: start + (end - start) * Float(i) / 60)
            world.update(entities: scene.bodies, deltaTime: 1.0 / 60)
        }
        for _ in 0..<60 { world.update(entities: scene.bodies, deltaTime: 1.0 / 60) }
        #expect(abs(scene.angle(of: 0) + 30) < 2, "held at \(scene.angle(of: 0))°")
        scene.endDrag()
        #expect(scene.root.components[B3PhysicsJointsComponent.self]!.joints.count == 5)
        var peak = [Float](repeating: -90, count: 5), struck = false
        for frame in 0..<150 {
            world.update(entities: scene.bodies, deltaTime: 1.0 / 60)
            if !struck, scene.angle(of: 0) > -2 { struck = true; _ = frame }
            guard struck else { continue }
            for i in 1..<5 { peak[i] = max(peak[i], i < 4 ? abs(scene.angle(of: i)) : scene.angle(of: i)) }
            if scene.angle(of: 4) < peak[4] - 0.5 { break }
        }
        #expect(peak[4] > 26, "the last ball should swing out: \(peak)")
        #expect(peak[1..<4].allSatisfy { $0 < 1.5 }, "the middle should stay, either way: \(peak)")
    }

    // Reset during a drag lets go: the ball isn't pulled back to the hand.
    @Test func resetLetsGoOfTheBall() {
        let scene = NewtonsCradleScene()
        let world = B3World(root: scene.root, settings: scene.root.components[B3PhysicsSimulationComponent.self]!)
        world.update(entities: scene.bodies, deltaTime: 1.0 / 60)
        let ball = scene.balls[0]
        #expect(scene.grab(ball, at: ball.position(relativeTo: scene.root)))
        scene.drag(toWorld: ball.position(relativeTo: nil) + [-0.1, 0.05, 0])
        for _ in 0..<30 { world.update(entities: scene.bodies, deltaTime: 1.0 / 60) }
        scene.lift(0)
        for _ in 0..<30 { world.update(entities: scene.bodies, deltaTime: 1.0 / 60) }
        #expect(abs(scene.angle(of: 0)) < 0.5, "pulled to \(scene.angle(of: 0))°")
        #expect(scene.root.components[B3PhysicsJointsComponent.self]!.joints.count == 5)
    }

    // A free row of five touching balls in zero gravity: the first one's velocity ends in the last.
    @Test(arguments: [false, true]) func freeRowPassesVelocity(sequential: Bool) {
        let root = Entity()
        var s = B3PhysicsSimulationComponent()
        s.gravity = .zero
        s.sequentialImpacts = sequential
        // Box3D bounces only above 1 m/s by default, the very speed of this row.
        s.restitutionThreshold = 0.1
        s.interpolatesPoses = false
        root.components.set(s)
        let balls = (0..<5).map { i -> Entity in
            let e = Entity()
            e.position = [Float(i) * 2 + (i == 0 ? -1 : 0), 0, 0]
            var b = B3PhysicsBodyComponent(mass: 1, material: .generate(staticFriction: 0, dynamicFriction: 0, restitution: 1))
            b.linearDamping = 0
            e.components.set(b)
            e.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 1)]))
            root.addChild(e)
            return e
        }
        balls[0].components.set(B3PhysicsMotionComponent(linearVelocity: [1, 0, 0]))
        let w = B3World(root: root, settings: s)
        for _ in 0..<120 { w.update(entities: balls, deltaTime: 1.0 / 60) }
        let v = balls.map { $0.b3.linearVelocity?.x ?? 0 }
        if sequential {
            #expect(abs(v[4] - 1) < 1e-3 && v[0..<4].allSatisfy { abs($0) < 1e-3 }, "velocities \(v)")
        } else {
            #expect(abs(v[4] - 1) > 0.05, "Box3D alone passed it on: \(v)")
        }
    }
}

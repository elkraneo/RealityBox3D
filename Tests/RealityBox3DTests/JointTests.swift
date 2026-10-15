import Testing
import RealityKit
import simd
@testable import RealityBox3D

@MainActor
@Suite struct JointTests {
    func simRoot() -> Entity {
        let root = Entity()
        var s = B3PhysicsSimulationComponent()
        s.interpolatesPoses = false
        root.components.set(s)
        return root
    }

    func world(_ root: Entity) -> B3World {
        B3World(root: root, settings: root.components[B3PhysicsSimulationComponent.self]!)
    }

    func box(_ root: Entity, at p: SIMD3<Float>, size: SIMD3<Float> = [0.2, 0.2, 0.2]) -> Entity {
        let e = Entity()
        e.position = p
        e.components.set(B3PhysicsBodyComponent(density: 1000))
        e.components.set(B3CollisionComponent(shapes: [.generateBox(size: size)]))
        root.addChild(e)
        return e
    }

    func run(_ w: B3World, _ es: [Entity], frames: Int, each: (Int) -> Void = { _ in }) {
        for i in 0..<frames { w.update(entities: es, deltaTime: 1.0 / 60); each(i) }
    }

    @Test func revolutePendulumSwingsAroundPinX() throws {
        let root = simRoot()
        let bob = box(root, at: [0, 3, 1]) // 1 m from the pivot, along z
        let pivot = root.pins.set(named: "pivot", position: [0, 3, 0])
        let hinge = bob.pins.set(named: "hinge", position: [0, 0, -1])
        try B3PhysicsRevoluteJoint(pin0: pivot, pin1: hinge).addToSimulation()
        let w = world(root)
        var maxX: Float = 0, lengthError: Float = 0, lowest: Float = 3
        run(w, [bob], frames: 120) { _ in
            maxX = max(maxX, abs(bob.position.x))
            lengthError = max(lengthError, abs(simd_distance(bob.position, [0, 3, 0]) - 1))
            lowest = min(lowest, bob.position.y)
        }
        #expect(maxX < 1e-3, "left the hinge plane: \(maxX)")
        #expect(lengthError < 0.01, "pendulum stretched by \(lengthError)")
        #expect(lowest < 2.05, "never swung down: lowest y \(lowest)")
    }

    @Test func revoluteMotorTurnsAtItsSpeed() throws {
        let root = simRoot()
        let wheel = box(root, at: [0, 2, 0], size: [0.1, 1, 1])
        let joint = B3PhysicsRevoluteJoint(pin0: root.pins.set(named: "axle", position: [0, 2, 0]),
                                           pin1: wheel.pins.set(named: "hub"),
                                           motor: B3JointMotor(speed: 2, maxForce: 1000))
        try joint.addToSimulation()
        let w = world(root)
        run(w, [wheel], frames: 30)
        let a0 = try #require(w.angle(of: joint))
        run(w, [wheel], frames: 30)
        let a1 = try #require(w.angle(of: joint))
        #expect(abs((a1 - a0) - 1) < 0.05, "turned \(a1 - a0) rad in 0.5 s at 2 rad/s")
        w.setMotorSpeed(0, of: joint)
        run(w, [wheel], frames: 10)
        let a2 = try #require(w.angle(of: joint))
        run(w, [wheel], frames: 30)
        #expect(abs(try #require(w.angle(of: joint)) - a2) < 0.01)
    }

    @Test func prismaticSlidesWithinItsLimit() throws {
        let root = simRoot()
        let slider = box(root, at: [0, 3, 0])
        // Pin x-axis pointing down, so gravity slides the box along the joint.
        let down = simd_quatf(angle: -.pi / 2, axis: [0, 0, 1])
        let joint = B3PhysicsPrismaticJoint(pin0: root.pins.set(named: "rail", position: [0, 3, 0], orientation: down),
                                            pin1: slider.pins.set(named: "car", orientation: down), linearLimit: 0...1)
        try joint.addToSimulation()
        let w = world(root)
        run(w, [slider], frames: 120)
        #expect(abs(slider.position.y - 2) < 0.02, "y \(slider.position.y)")
        #expect(abs(slider.position.x) < 1e-3 && abs(slider.position.z) < 1e-3)
        #expect(abs((w.translation(of: joint) ?? -1) - 1) < 0.02)
    }

    @Test func sphericalConeLimitHolds() throws {
        let root = simRoot()
        let link = box(root, at: [0, 2, 0], size: [0.1, 1, 0.1])
        // Cone around pin x, which points down along the link.
        let down = simd_quatf(angle: -.pi / 2, axis: [0, 0, 1])
        try B3PhysicsSphericalJoint(pin0: root.pins.set(named: "top", position: [0, 2.5, 0], orientation: down),
                                    pin1: link.pins.set(named: "end", position: [0, 0.5, 0], orientation: down),
                                    angularLimitInYZ: (0.3, 0.3)).addToSimulation()
        link.components.set(B3PhysicsMotionComponent(linearVelocity: [3, 0, 2]))
        let w = world(root)
        var widest: Float = 0
        run(w, [link], frames: 180) { _ in
            let axis = link.orientation.act([0, 1, 0])
            widest = max(widest, acos(min(1, simd_dot(axis, [0, 1, 0]))))
        }
        #expect(widest < 0.33, "opened to \(widest) rad against a 0.3 rad cone")
        #expect(widest > 0.2, "barely moved: \(widest)")
    }

    @Test func fixedJointKeepsBodiesTogether() throws {
        let root = simRoot()
        let ground = Entity()
        ground.position = [0, -0.5, 0]
        ground.components.set(B3PhysicsBodyComponent(mode: .static))
        ground.components.set(B3CollisionComponent(shapes: [.generateBox(size: [10, 1, 10])]))
        root.addChild(ground)
        let a = box(root, at: [0, 2, 0]), b = box(root, at: [0.3, 2.1, 0])
        try B3PhysicsFixedJoint(pin0: a.pins.set(named: "a", position: [0.3, 0.1, 0]), pin1: b.pins.set(named: "b")).addToSimulation()
        a.components.set(B3PhysicsMotionComponent(angularVelocity: [0, 0, 5]))
        let w = world(root)
        let rest = a.transformMatrix(relativeTo: nil).inverse * b.transformMatrix(relativeTo: nil)
        var drift: Float = 0
        run(w, [ground, a, b], frames: 120) { _ in
            let rel = a.transformMatrix(relativeTo: nil).inverse * b.transformMatrix(relativeTo: nil)
            drift = max(drift, simd_distance(rel.columns.3, rest.columns.3))
        }
        // Box3D's weld is a soft constraint ("cannot hold many bodies together rigidly"): a hard
        // landing while spinning bends it a few centimeters, and it springs back.
        #expect(drift < 0.08, "weld drifted \(drift)")
        let settled = a.transformMatrix(relativeTo: nil).inverse * b.transformMatrix(relativeTo: nil)
        #expect(simd_distance(settled.columns.3, rest.columns.3) < 0.005, "weld did not spring back")
    }

    @Test func fixedJointHoldsInFreeFlight() throws {
        let root = simRoot()
        let a = box(root, at: [0, 2, 0]), b = box(root, at: [0.3, 2.1, 0])
        try B3PhysicsFixedJoint(pin0: a.pins.set(named: "a", position: [0.3, 0.1, 0]), pin1: b.pins.set(named: "b")).addToSimulation()
        a.components.set(B3PhysicsMotionComponent(angularVelocity: [0, 0, 5]))
        let w = world(root)
        let rest = a.transformMatrix(relativeTo: nil).inverse * b.transformMatrix(relativeTo: nil)
        var drift: Float = 0
        run(w, [a, b], frames: 60) { _ in
            let rel = a.transformMatrix(relativeTo: nil).inverse * b.transformMatrix(relativeTo: nil)
            drift = max(drift, simd_distance(rel.columns.3, rest.columns.3))
        }
        #expect(drift < 0.005, "weld drifted \(drift) while spinning freely")
    }

    @Test func distanceJointIsARope() throws {
        let root = simRoot()
        let ball = box(root, at: [0.5, 3, 0])
        try B3PhysicsDistanceJoint(pin0: root.pins.set(named: "hook", position: [0, 3, 0]), pin1: ball.pins.set(named: "knot"),
                                   distanceLimit: 0...2).addToSimulation()
        let w = world(root)
        var longest: Float = 0
        run(w, [ball], frames: 180) { _ in longest = max(longest, simd_distance(ball.position, [0, 3, 0])) }
        #expect(longest < 2.02, "rope stretched to \(longest)")
        #expect(longest > 1.9, "rope never went taut: \(longest)")
    }

    @Test func removingAJointedBodyIsSafe() throws {
        let root = simRoot()
        let a = box(root, at: [0, 3, 0]), b = box(root, at: [0, 2.5, 0])
        try B3PhysicsSphericalJoint(pin0: root.pins.set(named: "p", position: [0, 3.5, 0]), pin1: a.pins.set(named: "a", position: [0, 0.5, 0])).addToSimulation()
        try B3PhysicsSphericalJoint(pin0: a.pins.set(named: "a2", position: [0, -0.25, 0]), pin1: b.pins.set(named: "b", position: [0, 0.25, 0])).addToSimulation()
        let w = world(root)
        run(w, [a, b], frames: 30)
        #expect(w.jointIds.count == 2)
        a.removeFromParent()
        run(w, [b], frames: 30)
        #expect(w.jointIds.count == 0) // the second joint's pin0 body is gone
        #expect(b.position.y < 2.5) // b falls free
    }

    @Test func jointEditsApplyInPlace() throws {
        let root = simRoot()
        let wheel = box(root, at: [0, 2, 0], size: [0.1, 1, 1])
        let other = box(root, at: [2, 2, 0])
        var hinge = B3PhysicsRevoluteJoint(pin0: root.pins.set(named: "axle", position: [0, 2, 0]), pin1: wheel.pins.set(named: "hub"),
                                           motor: B3JointMotor(speed: 2, maxForce: 1000))
        let rope = B3PhysicsDistanceJoint(pin0: root.pins.set(named: "hook", position: [2, 3, 0]), pin1: other.pins.set(named: "knot"), distanceLimit: 0...1)
        root.components.set(B3PhysicsJointsComponent(joints: [hinge, rope]))
        let w = world(root)
        run(w, [wheel, other], frames: 30)
        let ids0 = w.jointIds
        #expect(ids0.count == 2)

        // A limit and a stopped motor: same Box3D joint, new behavior.
        hinge.angularLimit = -0.1...0.1
        hinge.motor = B3JointMotor(speed: 0, maxForce: 1000)
        root.components.set(B3PhysicsJointsComponent(joints: [hinge, rope]))
        run(w, [wheel, other], frames: 60)
        #expect(w.jointIds == ids0, "edit rebuilt the joints")
        #expect(abs((wheel.b3.angularVelocity ?? .zero).x) < 0.05, "motor did not stop the wheel")

        // A new joint leaves the existing ones alone.
        let third = B3PhysicsFilterJoint(pin0: wheel.pins.set(named: "f"), pin1: other.pins.set(named: "f"))
        root.components.set(B3PhysicsJointsComponent(joints: [hinge, rope, third]))
        run(w, [wheel, other], frames: 1)
        #expect(Array(w.jointIds.prefix(2)) == ids0)
        #expect(w.jointIds.count == 3)

        // A moved pin rebuilds only that joint.
        let moved = B3PhysicsDistanceJoint(pin0: root.pins.set(named: "hook2", position: [2.5, 3, 0]), pin1: other.pins.set(named: "knot"), distanceLimit: 0...1)
        root.components.set(B3PhysicsJointsComponent(joints: [hinge, moved, third]))
        run(w, [wheel, other], frames: 1)
        #expect(w.jointIds[0] == ids0[0])
        #expect(w.jointIds[1] != ids0[1])
    }
}

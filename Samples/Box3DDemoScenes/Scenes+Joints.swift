import RealityBox3D
import RealityKit
import simd

/// Upstream's joints use identity frames, so their hinge axis is z. RealityKit's is the pin's x:
/// this turn makes a pin's x-axis point along z.
private let xAlongZ = simd_quatf(angle: -.pi / 2, axis: [0, 1, 0])

extension DemoScene {
    static let joints: [DemoScene] = [
        // The four single-joint samples share upstream's layout: a 1 × 3 × 0.5 m bar hanging from
        // a pivot 5.5 m up. Push swings it, as dragging it in upstream's app.
        DemoScene("Joints", "Revolute", upstream: "samples/sample_joint.cpp RevoluteJoint",
                  summary: "A bar on a hinge around the z-axis. Push swings it; the motor button spins it at 2 rad/s.",
                  camera: .init(yaw: 45, pitch: 30, distance: 15, target: [0, 2, 0]), actions: ["Push", "Motor"]) { b in
            b.ground(extent: 20)
            let bar = b.body(.generateBox(size: [1, 3, 0.5]), at: [0, 4, 0])
            let joint = B3PhysicsRevoluteJoint(pin0: b.anchor("pivot", at: [0, 5.5, 0], orientation: xAlongZ),
                                               pin1: bar.pins.set(named: "hinge", position: [0, 1.5, 0], orientation: xAlongZ),
                                               motor: B3JointMotor(speed: 0, maxForce: 0))
            b.joints.append(joint)
            b.pushAction(bar, impulse: [8000, 0, 0])
            var spinning = false
            b.action("Motor") { ctx in
                spinning.toggle()
                // Upstream's motor torque, 5,000 N·m, toggled between 2 rad/s and off.
                var j = joint
                j.motor = B3JointMotor(speed: spinning ? 2 : 0, maxForce: spinning ? 5000 : 0)
                var c = ctx.root.components[B3PhysicsJointsComponent.self]!
                c.joints = [j]
                ctx.root.components.set(c)
            }
        },

        DemoScene("Joints", "Prismatic", upstream: "samples/sample_joint.cpp PrismaticJoint",
                  summary: "A floating bar on a slider along x, held by a 2 Hz spring. Push slides it and it springs back.",
                  camera: .init(yaw: 45, pitch: 30, distance: 15, target: [0, 2, 0]), actions: ["Push"]) { b in
            b.ground(extent: 20)
            let bar = b.body(.generateBox(size: [1, 3, 0.5]), at: [0, 4, 0]) { $0.isAffectedByGravity = false }
            b.joints.append(B3PhysicsPrismaticJoint(pin0: b.anchor("rail", at: [0, 5.5, 0]),
                                                    pin1: bar.pins.set(named: "car", position: [0, 1.5, 0]),
                                                    spring: B3JointSpring(hertz: 2, dampingRatio: 0.7), constraintHertz: 120))
            b.pushAction(bar, impulse: [8000, 3000, 0])
        },

        DemoScene("Joints", "Spherical", upstream: "samples/sample_joint.cpp SphericalJoint",
                  summary: "A floating bar on a ball joint with a 2 Hz spring toward its rest pose. Push sets it wobbling.",
                  camera: .init(yaw: 45, pitch: 30, distance: 15, target: [0, 2, 0]), actions: ["Push"]) { b in
            b.ground(extent: 20)
            let bar = b.body(.generateBox(size: [1, 3, 0.5]), at: [0, 4, 0], density: 100) { $0.isAffectedByGravity = false }
            b.joints.append(B3PhysicsSphericalJoint(pin0: b.anchor("socket", at: [0, 5.5, 0]),
                                                    pin1: bar.pins.set(named: "ball", position: [0, 1.5, 0]),
                                                    spring: B3JointSpring(hertz: 2, dampingRatio: 0.7)))
            b.pushAction(bar, impulse: [800, 0, 600])
        },

        DemoScene("Joints", "Weld", upstream: "samples/sample_joint.cpp WeldJoint",
                  summary: "A floating bar welded to the world, rigid in translation, a 2 Hz spring in rotation.",
                  camera: .init(yaw: 45, pitch: 30, distance: 15, target: [0, 2, 0]), actions: ["Push"]) { b in
            b.ground(extent: 20)
            let bar = b.body(.generateBox(size: [1, 3, 0.5]), at: [0, 4, 0]) { $0.isAffectedByGravity = false }
            b.joints.append(B3PhysicsFixedJoint(pin0: b.anchor("wall", at: [0, 5.5, 0]),
                                                pin1: bar.pins.set(named: "plate", position: [0, 1.5, 0]),
                                                angularSpring: B3JointSpring(hertz: 2, dampingRatio: 0.7), constraintHertz: 240))
            b.pushAction(bar, impulse: [8000, 0, 0])
        },

        DemoScene("Joints", "Bridge", upstream: "samples/sample_joint.cpp Bridge",
                  summary: "150 planks on pairs of sprung ball joints, a 37 m rope bridge sagging under its weight.",
                  camera: .init(yaw: 0, pitch: 20, distance: 35, target: [0, 10, 0])) { b in
            b.ground(extent: 60)
            let a: Float = 0.125, count = 150
            let xbase = -160 * a
            let plank = B3ShapeResource.generateBox(size: [2 * a, 0.25, 1])
            let spring = B3JointSpring(hertz: 2, dampingRatio: 1)
            var previous: Entity?
            @MainActor func pin(_ e: Entity?, _ name: String, _ world: SIMD3<Float>, builder: DemoBuilder) -> GeometricPin {
                guard let e else { return builder.anchor(name, at: world) }
                return e.pins.set(named: name, position: world - e.position)
            }
            for i in 0..<count {
                let p = b.body(plank, at: [xbase + a * (1 + 2 * Float(i)), 20, 0], density: 20) {
                    $0.linearDamping = 0.1; $0.angularDamping = 0.1
                }
                for z: Float in [-0.5, 0.5] {
                    let pivot = SIMD3<Float>(xbase + 2 * a * Float(i), 20, z)
                    b.joints.append(B3PhysicsSphericalJoint(pin0: pin(previous, "r\(i)\(z)", pivot, builder: b),
                                                            pin1: p.pins.set(named: "l\(i)\(z)", position: pivot - p.position),
                                                            spring: spring, constraintHertz: 1000))
                }
                previous = p
            }
            for z: Float in [-0.5, 0.5] {
                let pivot = SIMD3<Float>(xbase + 2 * a * Float(count), 20, z)
                b.joints.append(B3PhysicsSphericalJoint(pin0: previous!.pins.set(named: "end\(z)", position: pivot - previous!.position),
                                                        pin1: b.anchor("anchor\(z)", at: pivot),
                                                        spring: spring, constraintHertz: 1000))
            }
        },

        DemoScene("Joints", "Ball and Chain", upstream: "samples/sample_joint.cpp BallAndChain",
                  summary: "A 4 m ball on a chain of 32 capsule links, released horizontally. The joints have friction.",
                  camera: .init(yaw: 180, pitch: 15, distance: 50, target: [0, -20, 0])) { b in
            let r: Float = 0.125, e: Float = 0.5
            let link = B3ShapeResource.generateCapsule(height: 2 * e + 2 * r, radius: r)
                .offsetBy(rotation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1]))
            var parentPin = b.anchor("hook", at: .zero)
            for i in 0..<32 {
                let l = b.body(link, at: [(1 + 2 * Float(i)) * e, 0, 0])
                b.joints.append(B3PhysicsSphericalJoint(pin0: parentPin, pin1: l.pins.set(named: "in", position: [-e, 0, 0]), friction: 10))
                parentPin = l.pins.set(named: "out", position: [e, 0, 0])
            }
            let radius: Float = 2
            let ball = b.body(.generateSphere(radius: radius), at: [(1 + 2 * 32) * e + radius - e, 0, 0])
            b.joints.append(B3PhysicsSphericalJoint(pin0: parentPin, pin1: ball.pins.set(named: "eye", position: [-radius, 0, 0]), friction: 10))
        },

        DemoScene("Joints", "Distance Joint", upstream: "samples/sample_joint.cpp DistanceJoint",
                  summary: "Ten balls on rigid 1 m distance joints, a chain hanging from 20 m. Upstream starts with one.",
                  camera: .init(yaw: 0, pitch: 0, distance: 40, target: [0, 10, 0])) { b in
            b.ground(extent: 20)
            let length: Float = 1, y: Float = 20
            var parentPin = b.anchor("top", at: [0, y, 0])
            for i in 0..<10 {
                let ball = b.body(.generateSphere(radius: 0.25), at: [length * Float(i + 1), y, 0], density: 20) { $0.angularDamping = 1 }
                b.joints.append(B3PhysicsDistanceJoint(pin0: parentPin, pin1: ball.pins.set(named: "a"), distanceLimit: length...length))
                parentPin = ball.pins.set(named: "b")
            }
        },
    ]
}

extension DemoBuilder {
    /// A Push action: an impulse at the body's center.
    mutating func pushAction(_ body: Entity, impulse: SIMD3<Float>) {
        action("Push") { _ in body.b3.applyLinearImpulse(impulse, relativeTo: nil) }
    }
}

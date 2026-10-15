import RealityBox3D
import RealityKit
import simd

/// Upstream's revolute, spherical and parallel joint frames turn around z, and so does a wheel's
/// spin frame; RealityKit's pins turn around x. A pin with this orientation is upstream's frame.
func pinOrientation(box3DFrame q: simd_quatf) -> simd_quatf {
    simd_normalize(q * simd_quatf(angle: -.pi / 2, axis: [0, 1, 0]))
}

/// A capsule between two points, as Box3D's `b3Capsule`.
func capsule(_ a: SIMD3<Float>, _ b: SIMD3<Float>, radius: Float) -> B3ShapeResource {
    let axis = b - a
    let length = simd_length(axis)
    let rotation = length > 1e-6 ? simd_quatf(from: [0, 1, 0], to: axis / length) : simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    return B3ShapeResource.generateCapsule(height: length + 2 * radius, radius: radius)
        .offsetBy(rotation: rotation, translation: (a + b) / 2)
}

extension DemoScene {
    static let vehicles: [DemoScene] = [
        DemoScene("Joints", "Driving", upstream: "samples/sample_joint.cpp Driving",
                  summary: "A car on four wheel joints over a wavy height field: rear-wheel drive, front steering, sprung suspension, held upright by a parallel joint.",
                  camera: .init(yaw: 25, pitch: 20, distance: 7, target: [0, 2, 0]),
                  actions: ["Forward", "Reverse", "Stop", "Left", "Straight", "Right"]) { b in
            // Upstream's b3CreateWave(50, 50, {4, 2, 4}, 0.02, 0.04): heights sin(2π·0.02·row)·sin(2π·0.04·column)
            let rows = 50, columns = 50
            var heights: [Float] = []
            for i in 0..<rows {
                for j in 0..<columns {
                    heights.append(sin(2 * .pi * 0.02 * Float(i)) * sin(2 * .pi * 0.04 * Float(j)))
                }
            }
            b.body(.generateHeightField(heights: heights, countX: columns, countZ: rows, scale: [4, 2, 4]), at: [-20, 0, -20], mode: .static)

            let chassis = b.body(.generateBox(size: [4, 1, 2]), at: [0, 2.5, 0], density: 0.5)
            let up = simd_quatf(from: [0, 0, 1], to: [0, 1, 0])
            b.joints.append(B3PhysicsParallelJoint(pin0: b.anchor("level", at: [-20, 0, -20], orientation: pinOrientation(box3DFrame: up)),
                                                   pin1: chassis.pins.set(named: "level", orientation: pinOrientation(box3DFrame: up)),
                                                   spring: B3JointSpring(hertz: 0.5, dampingRatio: 1), checksForInternalCollisions: true))

            let suspensionAxis = simd_quatf(from: [1, 0, 0], to: [0, 1, 0])
            let spinFrame = simd_quatf(from: [0, 0, 1], to: [0, 1, 0])
            let wheelRotation = simd_quatf(from: [0, 1, 0], to: [0, 0, 1])
            let steering = B3PhysicsWheelJoint.Steering(spring: B3JointSpring(hertz: 10, dampingRatio: 0.7), maxTorque: 5,
                                                        limit: -(.pi / 4)...(.pi / 4))
            var rear: [B3PhysicsWheelJoint] = [], front: [B3PhysicsWheelJoint] = []
            for (x, z) in [(Float(1.5), Float(0.8)), (1.5, -0.8), (-1.5, 0.8), (-1.5, -0.8)] {
                let wheel = b.body(.generateSphere(radius: 0.4), at: [x, 2, z], rotation: wheelRotation, density: 2, friction: 3) {
                    $0.allowsFastRotation = true
                }
                let isFront = x > 0
                let joint = B3PhysicsWheelJoint(
                    pin0: chassis.pins.set(named: "wheel\(x)\(z)", position: [x, -0.5, z], orientation: suspensionAxis),
                    pin1: wheel.pins.set(named: "hub", orientation: pinOrientation(box3DFrame: spinFrame)),
                    suspension: B3JointSpring(hertz: 4, dampingRatio: 0.7), suspensionLimit: -0.2...0.2,
                    spinMotor: isFront ? nil : B3JointMotor(speed: 0, maxForce: 5),
                    steering: isFront ? steering : nil)
                b.joints.append(joint)
                if isFront { front.append(joint) } else { rear.append(joint) }
            }
            // Upstream drives with W and S at 30 rad/s and steers with A and D.
            for (name, speed) in [("Forward", Float(-30)), ("Reverse", 30), ("Stop", 0)] {
                b.action(name) { ctx in for j in rear { ctx.world?.setMotorSpeed(speed, maxForce: speed == 0 ? 50 : 5, of: j) } }
            }
            for (name, angle) in [("Left", Float.pi / 6), ("Straight", 0), ("Right", -.pi / 6)] {
                b.action(name) { ctx in for j in front { ctx.world?.setSteeringAngle(angle, of: j) } }
            }
        },

        DemoScene("Ragdoll", "Ragdoll", upstream: "samples/sample_ragdoll.cpp RagdollOnBox, shared/human.c",
                  summary: "Upstream's 12-bone humanoid: capsule limbs on limited ball and hinge joints with friction and soft springs. Respawn drops another.",
                  camera: .init(yaw: 45, pitch: 30, distance: 6, target: .zero), actions: ["Respawn"]) { b in
            b.ground(extent: 20)
            let holder = b.content
            var count = 0
            @MainActor func spawn(into builder: inout DemoBuilder, at position: SIMD3<Float>, group: Int) {
                Human.build(into: &builder, at: position, group: group, frictionTorque: 5, hertz: 1, dampingRatio: 0.7)
            }
            spawn(into: &b, at: [0, 2, 0], group: 1)
            b.action("Respawn") { [weak holder] ctx in
                guard let holder else { return }
                count += 1
                var fresh = DemoBuilder()
                spawn(into: &fresh, at: [Float(count % 3) - 1, 2.5, 0], group: count + 1)
                for e in Array(fresh.content.children) { holder.addChild(e) }
                var c = ctx.root.components[B3PhysicsJointsComponent.self] ?? B3PhysicsJointsComponent()
                c.joints += fresh.joints
                ctx.root.components.set(c)
            }
        },
    ]
}

/// Port of upstream's `CreateHuman` (shared/human.c): bones, shapes, frames and limits verbatim.
enum Human {
    struct Bone {
        var name: String
        var parent: Int
        var position: SIMD3<Float>
        var rotation: simd_quatf
        var capsules: [(SIMD3<Float>, SIMD3<Float>, Float)]
        var friction: Float, rolling: Float
        var negativeGroup: Bool
        var joint: Joint?
        var jointFriction: Float = 1
    }
    enum Joint {
        case spherical(frameA: (SIMD3<Float>, simd_quatf), frameB: simd_quatf, swing: Float, twist: ClosedRange<Float>)
        case revolute(frameA: (SIMD3<Float>, simd_quatf), frameB: simd_quatf, limit: ClosedRange<Float>)
    }

    static func q(_ x: Float, _ y: Float, _ z: Float, _ w: Float) -> simd_quatf { simd_normalize(simd_quatf(ix: x, iy: y, iz: z, r: w)) }
    static let deg = Float.pi / 180

    static let bones: [Bone] = [
        Bone(name: "pelvis", parent: -1, position: [0, 0.996219, -0.023868], rotation: q(1, 0, 0, 0),
             capsules: [([0.04, 0.000001, 0], [-0.04, -0.000001, 0], 0.15)], friction: 0.6, rolling: 0.1, negativeGroup: false),
        Bone(name: "spine_01", parent: 0, position: [0, 1.017288, -0.024882], rotation: q(1, 0, 0, 0),
             capsules: [([0.029876, -0.146581, 0.00626], [-0.029876, -0.146574, 0.00626], 0.145663)], friction: 0.5, rolling: 0.1, negativeGroup: true,
             joint: .spherical(frameA: ([0, -0.021069, 0.001014], q(-0.642737, 0, 0, -0.766087)), frameB: q(-0.707107, 0, 0, -0.707107), swing: 35 * deg, twist: -17.5 * deg...17.5 * deg)),
        Bone(name: "spine_03", parent: 1, position: [0, 1.267766, -0.02232], rotation: q(1, 0, 0, 0),
             capsules: [([0.063996, -0.117434, -0.040199], [-0.063996, -0.117432, -0.040199], 0.165004)], friction: 0.5, rolling: 0.1, negativeGroup: false,
             joint: .spherical(frameA: ([0, -0.250478, -0.002562], q(-0.642766, 0, 0, -0.766063)), frameB: q(-0.707107, 0, 0, -0.707107), swing: 35 * deg, twist: -17.5 * deg...17.5 * deg)),
        Bone(name: "neck_01", parent: 2, position: [0, 1.498783, 0.024462], rotation: q(0.987879, 0, 0, 0.155228),
             capsules: [([0, -0.21234, 0], [0, -0.08734, 0], 0.11)], friction: 0.2, rolling: 0.05, negativeGroup: false,
             joint: .spherical(frameA: ([0, -0.231017, -0.046781], q(-0.516157, -0.000002, 0.000003, -0.856494)), frameB: q(-0.707108, -0.000002, 0.000002, -0.707106), swing: 30 * deg, twist: -17.5 * deg...17.5 * deg)),
        Bone(name: "thigh_l", parent: 0, position: [0.092175, 0.971562, -0.011177], rotation: q(0.471412, 0.52951, -0.499888, -0.497495),
             capsules: [([-0.047269, 0.000001, 0], [-0.379769, 0.000005, 0], 0.091539)], friction: 0.6, rolling: 0.1, negativeGroup: true,
             joint: .spherical(frameA: ([0.092175, 0.024656, -0.012691], q(-0.460186, -0.365368, -0.637662, 0.498118)), frameB: q(-0.499997, -0.500003, 0.500002, 0.499998), swing: 30 * deg, twist: -10 * deg...10 * deg)),
        Bone(name: "calf_l", parent: 4, position: [0.11872, 0.534541, -0.035362], rotation: q(0.521188, 0.480597, -0.54638, -0.445935),
             capsules: [([-0.445, 0.000001, 0], [0.005, -0.000001, 0], 0.08), ([-0.456371, 0.129321, 0.000013], [-0.359977, 0.014394, 0.000013], 0.07)],
             friction: 0.1, rolling: 0, negativeGroup: false,
             joint: .revolute(frameA: ([-0.438494, -0.000175, 0], q(0.344866, -0.938652, 0.000002, -0.000008)), frameB: q(-0.008182, -0.999967, 0.000001, -0.000007), limit: -30 * deg...30 * deg)),
        Bone(name: "thigh_r", parent: 0, position: [-0.092175, 0.971562, -0.011177], rotation: q(-0.497495, 0.499888, 0.52951, -0.471412),
             capsules: [([0.047269, -0.000001, 0], [0.379769, -0.000005, 0], 0.092587)], friction: 0.6, rolling: 0.1, negativeGroup: true,
             joint: .spherical(frameA: ([-0.092175, 0.024656, -0.012691], q(0.380666, 0.491846, 0.488644, -0.611889)), frameB: q(-0.500002, 0.499998, -0.499998, 0.500002), swing: 30 * deg, twist: -10 * deg...10 * deg)),
        Bone(name: "calf_r", parent: 6, position: [-0.11872, 0.534541, -0.035362], rotation: q(-0.445935, 0.54638, 0.480597, -0.521188),
             capsules: [([0.445, -0.000002, 0.000003], [-0.005, 0, 0.000003], 0.08), ([0.369113, -0.00683, 0.000017], [0.465507, -0.121757, 0.000017], 0.07)],
             friction: 0.1, rolling: 0, negativeGroup: false,
             joint: .revolute(frameA: ([0.438494, 0.000175, 0], q(0.000007, 0.000005, -0.34486, -0.938654)), frameB: q(0.000006, 0.000005, 0.008188, -0.999966), limit: -30 * deg...30 * deg)),
        Bone(name: "upperarm_l", parent: 2, position: [0.185817, 1.44363, 0.031306], rotation: q(0.56022, -0.307179, 0.366233, -0.676512),
             capsules: [([0.296038, 0.004386, -0.000794], [-0.016086, 0.004384, -0.016127], 0.065)], friction: 0.6, rolling: 0.1, negativeGroup: false,
             joint: .spherical(frameA: ([0.185817, -0.175865, -0.053625], q(-0.559466, -0.213784, -0.670199, 0.438323)), frameB: q(-0.500953, -0.502617, -0.509318, -0.486844), swing: 45 * deg, twist: -20 * deg...20 * deg),
             jointFriction: 0.8),
        Bone(name: "lowerarm_l", parent: 8, position: [0.347683, 1.193455, 0.029378], rotation: q(0.406736, -0.493012, 0.089135, -0.763911),
             capsules: [([0.312329, -0.008256, -0.016454], [0.033153, 0.008259, -0.002741], 0.06)], friction: 0.3, rolling: 0.1, negativeGroup: false,
             joint: .revolute(frameA: ([0.297979, 0.000361, 0], q(0.916726, 0.399352, 0.000839, -0.011456)), frameB: q(0.999922, -0.003857, 0.004659, -0.010908), limit: -35 * deg...35 * deg),
             jointFriction: 0.06),
        Bone(name: "upperarm_r", parent: 2, position: [-0.185817, 1.44363, 0.031306], rotation: q(0.676512, 0.366233, 0.307179, 0.56022),
             capsules: [([-0.300237, -0.000001, -0.000319], [0.011887, 0.000001, 0.015013], 0.065)], friction: 0.6, rolling: 0.1, negativeGroup: false,
             joint: .spherical(frameA: ([-0.185817, -0.175865, -0.053625], q(0.213793, 0.559449, 0.438334, -0.670204)), frameB: q(-0.509319, 0.486843, 0.500954, -0.502617), swing: 45 * deg, twist: -20 * deg...20 * deg),
             jointFriction: 0.8),
        Bone(name: "lowerarm_r", parent: 10, position: [-0.347683, 1.193455, 0.029378], rotation: q(0.763911, 0.089135, 0.493012, 0.406736),
             capsules: [([-0.312649, -0.000003, 0.016471], [-0.032986, 0, 0.002732], 0.06)], friction: 0.3, rolling: 0.1, negativeGroup: false,
             joint: .revolute(frameA: ([-0.297979, -0.000361, 0], q(-0.405444, 0.914059, 0.010541, 0.000852)), frameB: q(-0.002794, 0.999936, 0.010042, 0.004364), limit: -35 * deg...35 * deg),
             jointFriction: 0.06),
    ]

    /// Builds one humanoid. Upstream's feet have half density; here a body has one density, so
    /// they have the shin's.
    @MainActor
    static func build(into b: inout DemoBuilder, at position: SIMD3<Float>, group: Int, frictionTorque: Float, hertz: Float, dampingRatio: Float) {
        var entities: [Entity] = []
        for bone in bones {
            let shapes = bone.capsules.map { capsule($0.0, $0.1, radius: $0.2) }
            let e = b.body(shapes, at: position + bone.position, rotation: bone.rotation, friction: bone.friction,
                           rollingResistance: bone.rolling, name: bone.name)
            if bone.negativeGroup { e.components[B3CollisionComponent.self]!.groupIndex = -group }
            entities.append(e)
        }
        let spring = hertz > 0 ? B3JointSpring(hertz: hertz, dampingRatio: dampingRatio) : nil
        for (i, bone) in bones.enumerated() {
            guard let joint = bone.joint else { continue }
            let parent = entities[bone.parent], child = entities[i]
            let friction = bone.jointFriction * frictionTorque
            switch joint {
            case .spherical(let a, let qB, let swing, let twist):
                b.joints.append(B3PhysicsSphericalJoint(
                    pin0: parent.pins.set(named: "\(bone.name)_a", position: a.0, orientation: pinOrientation(box3DFrame: a.1)),
                    pin1: child.pins.set(named: "\(bone.name)_b", orientation: pinOrientation(box3DFrame: qB)),
                    angularLimitInYZ: (swing, swing), twistLimit: twist, spring: spring, friction: friction))
            case .revolute(let a, let qB, let limit):
                b.joints.append(B3PhysicsRevoluteJoint(
                    pin0: parent.pins.set(named: "\(bone.name)_a", position: a.0, orientation: pinOrientation(box3DFrame: a.1)),
                    pin1: child.pins.set(named: "\(bone.name)_b", orientation: pinOrientation(box3DFrame: qB)),
                    angularLimit: limit, motor: B3JointMotor(speed: 0, maxForce: friction), spring: spring))
            }
        }
        // Upstream's filter joints: legs pass each other, and the head passes the upper arms.
        for (a, c) in [(4, 6), (3, 8), (3, 10)] {
            b.joints.append(B3PhysicsFilterJoint(pin0: entities[a].pins.set(named: "filter\(c)"), pin1: entities[c].pins.set(named: "filter\(a)")))
        }
    }
}

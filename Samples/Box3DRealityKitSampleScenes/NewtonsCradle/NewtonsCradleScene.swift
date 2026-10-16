// Port of Apple's sample "Simulating physics joints in your RealityKit app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-joints-in-your-realitykit-app
//
// Apple's five pendulums hang from boxes on revolute joints: balls 4 cm in radius and 2 kg on
// 30 cm strings, touching, with no friction, no damping and full restitution, under a simulation
// root scaled by 0.5 with the pendulums scaled by 2, for slow motion. "Push ball" gives the first
// ball an impulse. Differences from Apple's sample:
//
// - The simulation root's units are centimeters, the size Box3D's tolerances suit for 4 cm balls.
// - `sequentialImpacts`: Box3D, like RealityKit, solves a step's contacts together and bounces
//   only by the approach speed it saw before solving, so a ball striking the row shares its
//   momentum among it: one ball in, three out at 12°. RealityKit sends four out together. With
//   impacts resolved pair by pair before each step, one ball in sends one ball out, two send two,
//   and the middle balls stay still, as in a real cradle.
// - "Lift" raises the first one, two or three balls to 30° and lets them go.
// - A ball can be dragged by hand: a motor joint with a force-limited spring pulls it toward the
//   pointer, Box3D's way of grabbing a body, so a dragged ball still strikes the others.

import CoreGraphics
import Foundation
import RealityBox3D
import RealityKit
import simd

@MainActor
package final class NewtonsCradleScene {
    /// Apple's `PendulumSettings`, in centimeters
    package enum Settings {
        static let ballRadius: Float = 4
        static let ballMass: Float = 2
        static let stringLength: Float = 30
        static let stringRadius: Float = 0.1
        static let attachmentSize: SIMD3<Float> = [2, 0.5, 2] * ballRadius
        package static let pendulumCount = 5
        /// Apple's slow motion: the simulation root's scale, which the pendulums undo
        static let pendulumSpeed: Float = 0.5
    }

    package let content = Entity()
    package let camera = SampleCamera(eye: [0, 0.18, 0.7], target: [0, 0.14, 0], fieldOfView: 40)
    /// The simulation root, in centimeters slowed by Apple's pendulum speed
    package let root = Entity()
    private let pendulums = Entity()
    package private(set) var balls: [Entity] = []
    package private(set) var attachments: [Entity] = []

    package init() {
        root.scale = SIMD3(repeating: 0.01 * Settings.pendulumSpeed)
        var settings = B3PhysicsSimulationComponent()
        settings.gravity = [0, -981, 0]
        settings.sequentialImpacts = true
        root.components.set(settings)
        pendulums.scale = SIMD3(repeating: 1 / Settings.pendulumSpeed)
        root.addChild(pendulums)
        content.addChild(root)

        let first = -Float(Settings.pendulumCount - 1) / 2
        for index in 0..<Settings.pendulumCount {
            let x = (first + Float(index)) * Settings.attachmentSize.x
            let (attachment, ball) = makePendulum(at: x, index: index)
            attachments.append(attachment)
            balls.append(ball)
        }

        let light = DirectionalLight()
        light.light.intensity = 3000
        light.look(at: .zero, from: [0.5, 1, 1], relativeTo: nil)
        content.addChild(light)
        let fill = DirectionalLight()
        fill.light.intensity = 1000
        fill.look(at: .zero, from: [-1, 0.3, -0.5], relativeTo: nil)
        content.addChild(fill)
        content.addChild(camera.makeEntity())
    }

    /// Every body of the cradle, for driving its world directly
    package var bodies: [Entity] { attachments + balls }

    /// The pivot of a pendulum, in the pendulums' space
    private func pivot(_ index: Int) -> SIMD3<Float> { attachments[index].position }

    /// A ball's angle from hanging straight down, in degrees; positive toward +x.
    package func angle(of index: Int) -> Float {
        let d = balls[index].position - pivot(index)
        return atan2(d.x, -d.y) * 180 / .pi
    }

    /// All balls at rest, then the first `count` raised to `degrees` toward -x and released.
    package func lift(_ count: Int, degrees: Float = 30) {
        // A ball still held would be pulled back to the hand.
        endDrag()
        for (index, ball) in balls.enumerated() {
            let a = index < count ? -degrees * .pi / 180 : 0
            ball.position = pivot(index) + Settings.stringLength * SIMD3(sin(a), -cos(a), 0)
            ball.orientation = simd_quatf(angle: a, axis: [0, 0, 1])
            ball.components.set(B3PhysicsMotionComponent())
        }
    }

    /// Apple's "Push ball": the first ball leaves at Apple's measured speed, 2 m/s in the
    /// simulation root's frame, which is 200 of its centimeters.
    package func push() {
        balls[0].b3.applyLinearImpulse([-Settings.ballMass * 200, 0, 0], relativeTo: root)
    }

    private func makePendulum(at x: Float, index: Int) -> (attachment: Entity, ball: Entity) {
        let material = SimpleMaterial(color: .init(red: 0.75, green: 0.88, blue: 0.75, alpha: 1), isMetallic: false)
        let attachment = ModelEntity(mesh: .generateBox(size: Settings.attachmentSize), materials: [material])
        attachment.name = "attachment_\(index)"
        attachment.position = [x, Settings.stringLength, 0]
        attachment.components.set(B3PhysicsBodyComponent(mass: 1, material: Self.surface, mode: .static))
        attachment.components.set(B3CollisionComponent(shapes: [.generateBox(size: Settings.attachmentSize)]))
        pendulums.addChild(attachment)

        let ball = ModelEntity(mesh: .generateSphere(radius: Settings.ballRadius),
                               materials: [SimpleMaterial(color: .init(white: 0.8, alpha: 1), roughness: 0.15, isMetallic: true)])
        ball.name = "ball_\(index)"
        ball.position = [x, 0, 0]
        var body = B3PhysicsBodyComponent(mass: Settings.ballMass, material: Self.surface)
        body.linearDamping = 0
        body.angularDamping = 0
        ball.components.set(body)
        ball.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: Settings.ballRadius)]))
        // The string, from the ball up to the pivot; it turns with the ball.
        let stringHeight = Settings.stringLength - Settings.ballRadius
        let string = ModelEntity(mesh: .generateCylinder(height: stringHeight, radius: Settings.stringRadius),
                                 materials: [SimpleMaterial(color: .init(white: 0.2, alpha: 1), isMetallic: false)])
        string.position = [0, Settings.ballRadius + stringHeight / 2, 0]
        ball.addChild(string)
        pendulums.addChild(ball)

        // Apple's pins: the hinge turned from x to z, so the balls swing in the xy plane.
        let hinge = simd_quatf(from: [1, 0, 0], to: [0, 0, 1])
        let attachmentPin = attachment.pins.set(named: "attachment_hinge", position: .zero, orientation: hinge)
        let ballPin = ball.pins.set(named: "ball_hinge", position: attachment.position(relativeTo: ball), orientation: hinge)
        _ = try? B3PhysicsRevoluteJoint(pin0: attachmentPin, pin1: ballPin).addToSimulation()
        return (attachment, ball)
    }

    // MARK: Dragging

    /// The invisible anchor the pointer moves, and the joint pulling the grabbed ball to it
    private var grab: (anchor: Entity, joint: B3PhysicsMotorJoint)?
    /// The anchor of a grab, for tests
    package var grabAnchor: Entity? { grab?.anchor }

    /// Grabs the ball under a point of the view. True if there is one.
    package func beginDrag(at point: CGPoint, in size: CGSize) -> Bool {
        guard grab == nil, let world = RealityBox3D.world(for: root) else { return false }
        let ray = camera.ray(through: point, in: size)
        let origin = root.convert(position: ray.origin, from: nil)
        let direction = root.convert(direction: ray.direction, from: nil)
        guard let hit = world.raycast(origin: origin, direction: direction, length: 10_000, query: .nearest).first,
              balls.contains(where: { $0 === hit.entity }) else { return false }
        return grab(hit.entity, at: hit.position)
    }

    /// Grabs a ball at a point of it, in the simulation root's space.
    package func grab(_ ball: Entity, at point: SIMD3<Float>) -> Bool {
        guard grab == nil else { return false }
        let anchor = Entity()
        pendulums.addChild(anchor)
        anchor.setPosition(root.convert(position: point, to: nil), relativeTo: nil)
        let hit = (position: point, entity: ball)
        // A critically damped 5 Hz spring, capped at 50 times the ball's weight.
        let joint = B3PhysicsMotorJoint(
            pin0: anchor.pins.set(named: "grab", position: .zero),
            pin1: ball.pins.set(named: "grab", position: ball.convert(position: hit.position, from: root)),
            linearSpring: B3JointSpring(hertz: 5, dampingRatio: 1), maxSpringForce: 50 * Settings.ballMass * 981,
            checksForInternalCollisions: false)
        guard (try? joint.addToSimulation()) != nil else { anchor.removeFromParent(); return false }
        grab = (anchor, joint)
        return true
    }

    /// Moves the grab to the point of the cradle's plane under a point of the view.
    package func drag(to point: CGPoint, in size: CGSize) {
        guard let grab else { return }
        let ray = camera.ray(through: point, in: size)
        // The pendulums swing in the plane z = 0 of the content.
        guard abs(ray.direction.z) > 1e-4 else { return }
        let t = -ray.origin.z / ray.direction.z
        guard t > 0 else { return }
        grab.anchor.setPosition(ray.origin + ray.direction * t, relativeTo: nil)
    }

    /// Moves the grab to a world space point.
    package func drag(toWorld point: SIMD3<Float>) { grab?.anchor.setPosition(point, relativeTo: nil) }

    package func endDrag() {
        guard let grab else { return }
        if var joints = root.components[B3PhysicsJointsComponent.self] {
            joints.joints.removeAll { $0 is B3PhysicsMotorJoint } // the grab is the only motor joint
            root.components.set(joints)
        }
        grab.anchor.removeFromParent()
        self.grab = nil
    }

    /// Apple's material: no friction, full restitution
    private static let surface = B3PhysicsMaterial.generate(staticFriction: 0, dynamicFriction: 0, restitution: 1)
}

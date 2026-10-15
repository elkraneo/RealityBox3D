// Apple's Newton's cradle ("Simulating physics joints in your RealityKit app") on RealityKit and
// RealityBox3D, with RealitySDFPhysics's metrics:
//
// 1. Lift: the first k balls raised to 30° and released; each ball's farthest swing toward +x
//    over the first swing out, 0.85 s with Apple's slow motion. A real cradle sends the far k
//    balls out past 28° and leaves the others under 1.5°.
// 2. Push: Apple's impulse on the first ball; farthest swings over 2 s. Ball 5 out past 40°,
//    balls 2–4 under 2°.
// 3. Free row: five of Apple's balls in zero gravity, the first at 1 m/s from 5 cm away;
//    velocities after 20 steps, with the row awake and asleep. The last at 1.000, the rest 0.
//
// RealityKit runs Apple's scene with its values verbatim. RealityBox3D runs the same scene as its
// Newton's Cradle sample: Apple's values in centimeters, with `sequentialImpacts`.

import CoreMedia
import Foundation
import Metal
import RealityBox3D
import RealityKit

enum CradleCase {
    case lift(Int), push
    var frames: Int { if case .push = self { 120 } else { 51 } }
    var label: String { if case .lift(let k) = self { "lift \(k)" } else { "push" } }
}

/// RealityBox3D's Newton's Cradle sample: Apple's pendulums in centimeters, slowed as Apple's are.
@MainActor
struct Box3DCradle {
    let root = Entity()
    var balls: [Entity] = [], attachments: [Entity] = []
    let length: Float = 30

    init() {
        let r: Float = 4, speed: Float = 0.5
        root.scale = SIMD3(repeating: 0.01 * speed)
        var s = B3PhysicsSimulationComponent()
        s.gravity = [0, -981, 0]
        s.sequentialImpacts = true
        root.components.set(s)
        let pendulums = Entity()
        pendulums.scale = SIMD3(repeating: 1 / speed)
        root.addChild(pendulums)
        let surface = B3PhysicsMaterial.generate(staticFriction: 0, dynamicFriction: 0, restitution: 1)
        let hinge = simd_quatf(from: [1, 0, 0], to: [0, 0, 1])
        for i in 0..<5 {
            let x = (Float(i) - 2) * 2 * r
            let top = Entity()
            top.position = [x, length, 0]
            top.components.set(B3PhysicsBodyComponent(mass: 1, material: surface, mode: .static))
            top.components.set(B3CollisionComponent(shapes: [.generateBox(size: SIMD3(2, 0.5, 2) * r)]))
            pendulums.addChild(top)
            let ball = Entity()
            ball.position = [x, 0, 0]
            var body = B3PhysicsBodyComponent(mass: 2, material: surface)
            body.linearDamping = 0
            body.angularDamping = 0
            ball.components.set(body)
            ball.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: r)]))
            pendulums.addChild(ball)
            try? B3PhysicsRevoluteJoint(pin0: top.pins.set(named: "a", position: .zero, orientation: hinge),
                                        pin1: ball.pins.set(named: "b", position: top.position(relativeTo: ball), orientation: hinge)).addToSimulation()
            attachments.append(top)
            balls.append(ball)
        }
    }

    func angle(_ i: Int) -> Float {
        let d = balls[i].position - attachments[i].position
        return atan2(d.x, -d.y) * 180 / .pi
    }

    func lift(_ k: Int) {
        for i in 0..<k {
            let a: Float = -30 * .pi / 180
            balls[i].position = attachments[i].position + length * SIMD3(sin(a), -cos(a), 0)
            balls[i].orientation = simd_quatf(angle: a, axis: [0, 0, 1])
        }
    }
}

/// Farthest swing of each ball toward +x, in degrees.
@MainActor
func cradleBox3D(_ c: CradleCase) -> [Float] {
    let cradle = Box3DCradle()
    let world = B3World(root: cradle.root, settings: cradle.root.components[B3PhysicsSimulationComponent.self]!)
    let bodies = cradle.attachments + cradle.balls
    if case .lift(let k) = c { cradle.lift(k) } else {
        world.update(entities: bodies, deltaTime: 1.0 / 60)
        // Apple's push leaves at 2 m/s in its simulation root's frame: 200 of this root's centimeters.
        cradle.balls[0].b3.applyLinearImpulse([-2 * 200, 0, 0], relativeTo: cradle.root)
    }
    var peak = [Float](repeating: -90, count: 5)
    for _ in 0..<c.frames {
        world.update(entities: bodies, deltaTime: 1.0 / 60)
        for i in 0..<5 { peak[i] = max(peak[i], cradle.angle(i)) }
    }
    return peak
}

/// A RealityKit scene stepped by hand through an offscreen renderer.
@MainActor
final class RealityKitStepper {
    let timebase: CMTimebase
    private let renderer: RealityRenderer
    private let output: RealityRenderer.CameraOutput
    private var step = 0

    init(_ root: Entity) throws {
        var tb: CMTimebase?
        CMTimebaseCreateWithSourceClock(allocator: nil, sourceClock: CMClockGetHostTimeClock(), timebaseOut: &tb)
        timebase = tb!
        CMTimebaseSetRate(timebase, rate: 0)
        CMTimebaseSetTime(timebase, time: .zero)
        root.components[PhysicsSimulationComponent.self]?.clock = timebase
        renderer = try RealityRenderer()
        let camera = PerspectiveCamera()
        renderer.entities.append(contentsOf: [root, camera])
        renderer.activeCamera = camera
        let d = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: 8, height: 8, mipmapped: false)
        d.usage = [.renderTarget]
        d.storageMode = .private
        output = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: MTLCreateSystemDefaultDevice()!.makeTexture(descriptor: d)!))
    }

    func frame() async throws {
        step += 1
        CMTimebaseSetTime(timebase, time: CMTime(value: CMTimeValue(step), timescale: 60))
        try await withCheckedThrowingContinuation { (c: CheckedContinuation<Void, any Error>) in
            do { try renderer.updateAndRender(deltaTime: 1.0 / 60, cameraOutput: output, onComplete: { _ in c.resume() }) }
            catch { c.resume(throwing: error) }
        }
    }
}

/// Apple's scene, values verbatim (RealitySDFPhysics's headless program), lifted or pushed.
@MainActor
func cradleRealityKit(_ c: CradleCase) async throws -> [Float] {
    let ballRadius: Float = 0.04, ballMass: Float = 2, stringLength: Float = 0.3
    let attachmentSize: SIMD3<Float> = [2, 0.5, 2], count = 5, speed: Float = 0.5
    let root = Entity(), parent = Entity()
    root.addChild(parent)
    var sim = PhysicsSimulationComponent()
    sim.solverIterations.positionIterations = 25
    sim.solverIterations.velocityIterations = 25
    root.components.set(sim)
    root.components.set(PhysicsJointsComponent())
    var balls: [Entity] = []
    let firstX = -Float(count - 1) / 2
    let lifted: Int = { if case .lift(let k) = c { k } else { 0 } }()
    for i in 0..<count {
        let p = Entity()
        let ball = Entity()
        ball.position.y = -stringLength
        p.addChild(ball)
        let att = Entity()
        p.addChild(att)
        let ballShape = ShapeResource.generateSphere(radius: ballRadius)
        var bb = PhysicsBodyComponent(shapes: [ballShape], mass: ballMass,
                                      material: .generate(staticFriction: 0, dynamicFriction: 0, restitution: 1), mode: .dynamic)
        bb.linearDamping = 0
        ball.components.set([bb, CollisionComponent(shapes: [ballShape])])
        let attShape = ShapeResource.generateBox(size: attachmentSize * ballRadius)
        var ab = PhysicsBodyComponent(shapes: [attShape], mass: 1,
                                      material: .generate(staticFriction: 0, dynamicFriction: 0, restitution: 1), mode: .static)
        ab.linearDamping = 0
        att.components.set([ab, CollisionComponent(shapes: [attShape])])
        parent.addChild(p)
        let hinge = simd_quatf(from: [1, 0, 0], to: [0, 0, 1])
        let pin0 = att.pins.set(named: "a", position: .zero, orientation: hinge)
        let pin1 = ball.pins.set(named: "b", position: att.position(relativeTo: ball), orientation: hinge)
        try PhysicsRevoluteJoint(pin0: pin0, pin1: pin1).addToSimulation()
        p.position.x = (firstX + Float(i)) * attachmentSize.x * ballRadius
        if i < lifted {
            let a: Float = -30 * .pi / 180
            ball.position = stringLength * SIMD3(sin(a), -cos(a), 0)
            ball.orientation = simd_quatf(angle: a, axis: [0, 0, 1])
        }
        balls.append(ball)
    }
    root.position.y = stringLength / 2
    root.scale = SIMD3(repeating: speed)
    parent.scale = SIMD3(repeating: 1 / speed)
    let stepper = try RealityKitStepper(root)
    func angle(_ i: Int) -> Float {
        let v = balls[i].position - SIMD3(0, 0, 0) // hinge at the pendulum's origin
        return atan2(v.x, -v.y) * 180 / .pi
    }
    if case .push = c {
        for _ in 0..<30 { try await stepper.frame() }
        let impulse: SIMD3<Float> = [-2 / pow(speed, 4), 0, 0]
        balls[0].playAnimation(try AnimationResource.makeActionAnimation(for: ImpulseAction(targetEntity: .sourceEntity, linearImpulse: impulse)))
    }
    var peak = [Float](repeating: -90, count: 5)
    for _ in 0..<c.frames {
        try await stepper.frame()
        for i in 0..<5 { peak[i] = max(peak[i], angle(i)) }
    }
    return peak
}

/// Velocities of a free row of Apple's balls along x, in m/s, and the largest surface spin speed.
@MainActor
func freeRowBox3D(asleep: Bool) -> (velocities: [Float], spin: Float) {
    let root = Entity()
    root.scale = SIMD3(repeating: 0.01) // centimeters
    var s = B3PhysicsSimulationComponent()
    s.gravity = .zero
    s.sequentialImpacts = true
    s.interpolatesPoses = false
    root.components.set(s)
    let balls = (0..<5).map { i -> Entity in
        let e = Entity()
        e.position = [Float(i) * 8 - (i == 0 ? 5 : 0), 0, 0]
        var b = B3PhysicsBodyComponent(mass: 2, material: .generate(staticFriction: 0, dynamicFriction: 0, restitution: 1))
        b.linearDamping = 0
        b.angularDamping = 0
        e.components.set(b)
        e.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 4)]))
        root.addChild(e)
        return e
    }
    let w = B3World(root: root, settings: s)
    // The world builds its bodies on its first update; asleep, the row has rested 1.5 s.
    for _ in 0..<(asleep ? 90 : 1) { w.update(entities: balls, deltaTime: 1.0 / 60) }
    balls[0].b3.setVelocity(linear: [100, 0, 0], angular: .zero)
    for _ in 0..<20 { w.update(entities: balls, deltaTime: 1.0 / 60) }
    let spin = balls.map { simd_length($0.b3.angularVelocity ?? .zero) * 0.04 }.max() ?? 0
    return (balls.map { ($0.b3.linearVelocity?.x ?? 0) / 100 }, spin)
}

@MainActor
func freeRowRealityKit(asleep: Bool) async throws -> (velocities: [Float], spin: Float) {
    let root = Entity()
    var sim = PhysicsSimulationComponent()
    sim.gravity = .zero
    root.components.set(sim)
    let balls = (0..<5).map { i -> Entity in
        let e = Entity()
        e.position = [Float(i) * 0.08 - (i == 0 ? 0.05 : 0), 0, 0]
        let shape = ShapeResource.generateSphere(radius: 0.04)
        var b = PhysicsBodyComponent(shapes: [shape], mass: 2,
                                     material: .generate(staticFriction: 0, dynamicFriction: 0, restitution: 1), mode: .dynamic)
        b.linearDamping = 0
        b.angularDamping = 0
        e.components.set([b, CollisionComponent(shapes: [shape]), PhysicsMotionComponent()])
        root.addChild(e)
        return e
    }
    let stepper = try RealityKitStepper(root)
    for _ in 0..<(asleep ? 90 : 1) { try await stepper.frame() }
    balls[0].components.set(PhysicsMotionComponent(linearVelocity: [1, 0, 0]))
    for _ in 0..<20 { try await stepper.frame() }
    let motion = balls.map { $0.components[PhysicsMotionComponent.self] ?? PhysicsMotionComponent() }
    return (motion.map(\.linearVelocity.x), motion.map { simd_length($0.angularVelocity) * 0.04 }.max() ?? 0)
}

/// Prints the three metrics for both engines.
@MainActor
func reportCradle() async throws {
    print("Newton's cradle, Apple's values · farthest swing toward +x per ball, degrees")
    print("")
    for c in [CradleCase.lift(1), .lift(2), .lift(3), .push] {
        for (name, peak) in [("RealityBox3D", cradleBox3D(c)), ("RealityKit", try await cradleRealityKit(c))] {
            let out: Int = { if case .lift(let k) = c { k } else { 1 } }()
            let far = Array(peak.suffix(out)), rest = Array(peak.prefix(5 - out).dropFirst(c.label == "push" ? 1 : 0))
            let worst = rest.max() ?? 0
            let pass: Bool = {
                if case .push = c { return far.allSatisfy { $0 > 40 } && worst < 2 }
                return far.allSatisfy { $0 > 28 } && worst < 1.5
            }()
            print(String(format: "%-7@ %-13@ ", c.label as NSString, name as NSString)
                  + peak.map { String(format: "%6.1f", $0) }.joined()
                  + String(format: "   far %@, worst other %.1f  %@", far.map { String(format: "%.1f", $0) }.joined(separator: " / ") as NSString,
                           worst, (pass ? "pass" : "FAIL") as NSString))
        }
    }
    print("")
    print("Free row, zero gravity, first ball at 1 m/s · velocities after 20 steps, m/s")
    for asleep in [false, true] {
        for (name, row) in [("RealityBox3D", freeRowBox3D(asleep: asleep)), ("RealityKit", try await freeRowRealityKit(asleep: asleep))] {
            let pass = abs(row.velocities[4] - 1) < 0.005 && row.velocities.prefix(4).allSatisfy { abs($0) < 0.005 } && row.spin < 0.002
            print(String(format: "%-7@ %-13@ ", (asleep ? "asleep" : "awake") as NSString, name as NSString)
                  + row.velocities.map { String(format: "%7.3f", $0) }.joined()
                  + String(format: "   spin %.4f m/s  %@", row.spin, (pass ? "pass" : "FAIL") as NSString))
        }
    }
}

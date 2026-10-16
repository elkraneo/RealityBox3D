import Foundation
import Testing
import RealityKit
import simd
@testable import Box3DDemoScenes
@testable import RealityBox3D

/// Upstream's Geometric Mover scene, driven by the keys the app would send.
@MainActor
@Suite struct GeometricMoverTests {
    func load(at start: SIMD3<Float>? = nil) -> (DemoRunner, Entity) {
        let runner = DemoRunner(DemoScene.named("Geometric Mover")!)
        let character = runner.root.children[0].children.first { $0.name == "character" }!
        if let start { character.position = start }
        return (runner, character)
    }

    func entity(_ runner: DemoRunner, _ name: String) -> Entity {
        runner.root.children[0].children.first { $0.name == name }!
    }

    func drive(_ runner: DemoRunner, forward: SIMD3<Float> = [1, 0, 0], throttle: SIMD2<Float> = [1, 0], jump: Bool = false,
               frames: Int) {
        var input = DemoInput()
        input.throttle = throttle
        input.jump = jump
        DemoScene.setInput(input, cameraForward: forward, root: runner.root)
        runner.advance(frames: frames)
    }

    /// The pogo holds the capsule's lower cap center three radii over the floor: center at 1.4 m.
    @Test func floatsOnThePogo() {
        let (runner, c) = load()
        runner.advance(frames: 60)
        #expect(abs(c.position.y - 1.4) < 0.05, "y \(c.position.y)")
        drive(runner, frames: 60)
        #expect(c.position.x > 7.5 + 4, "walked to x \(c.position.x)")
        #expect(abs(c.position.y - 1.4) < 0.05, "y \(c.position.y)")
    }

    /// Box3D's mover has no step-up; upstream's pogo floats the capsule up the stairs.
    @Test func climbsTheStairs() {
        let (runner, c) = load(at: [-14, 1.4, 0])
        runner.advance(frames: 30)
        var top: Float = 0
        for _ in 0..<60 { drive(runner, frames: 1); top = max(top, c.position.y) }
        // The top step is 1.43 m up, at x −9.3 … −8.9; beyond it the character walks off
        #expect(c.position.x > -8.9, "stuck at x \(c.position.x)")
        #expect(top > 1.4 + 1.3, "did not climb: highest \(top)")
    }

    @Test func jumps() {
        let (runner, c) = load()
        runner.advance(frames: 30)
        drive(runner, throttle: .zero, jump: true, frames: 1)
        var top: Float = 0
        for _ in 0..<60 { drive(runner, throttle: .zero, frames: 1); top = max(top, c.position.y) }
        // v² / 2g = 25 / 30 m above the float height
        #expect(top > 1.4 + 0.6, "top \(top)")
        #expect(abs(c.position.y - 1.4) < 0.05, "landed at \(c.position.y)")
    }

    /// The violet capsule holds the character off; the green one lets it lean in.
    @Test func softAndHardCapsules() {
        func closest(_ name: String, from z: Float, toward dz: Float) -> Float {
            let (runner, c) = load(at: [0, 1.4, z])
            let target = entity(runner, name)
            runner.advance(frames: 10)
            var nearest = Float.infinity
            for _ in 0..<90 {
                drive(runner, forward: [0, 0, dz], frames: 1)
                nearest = min(nearest, simd_distance(c.position, target.position))
            }
            return nearest
        }
        let hard = closest("enemy", from: 8.5, toward: -1)
        let soft = closest("friend", from: 1.5, toward: 1)
        #expect(hard > 0.55, "enemy let the character in to \(hard)")
        #expect(soft < 0.45, "friend held the character at \(soft)")
    }

    /// The character pushes the door on its hinge.
    @Test func pushesTheDoor() {
        let (runner, c) = load(at: [-2.4, 1.4, -2])
        let door = entity(runner, "door")
        runner.advance(frames: 10)
        let before = door.orientation
        drive(runner, forward: [0, 0, 1], frames: 60)
        let turned = 2 * acos(min(abs(simd_dot(before.vector, door.orientation.vector)), 1))
        #expect(turned > 0.3, "door turned \(turned) rad; character at \(c.position)")
    }

    /// Upstream moves at 60 Hz; a 15 fps app takes four moves a frame and walks as far.
    @Test func walksAtFullSpeedAtLowFrameRates() {
        let (fast, a) = load(), (slow, b) = load()
        fast.advance(frames: 30)
        slow.advance(frames: 8, deltaTime: 1.0 / 15)
        var input = DemoInput()
        input.throttle = [1, 0]
        DemoScene.setInput(input, cameraForward: [1, 0, 0], root: fast.root)
        DemoScene.setInput(input, cameraForward: [1, 0, 0], root: slow.root)
        fast.advance(frames: 60)
        slow.advance(frames: 15, deltaTime: 1.0 / 15)
        #expect(abs(a.position.x - b.position.x) < 0.1, "60 fps \(a.position.x), 15 fps \(b.position.x)")
    }

    /// tinyobjloader ear-clips: the stairs' concave sides become triangles covering exactly the
    /// face, each wound as the face.
    @Test func concaveFacesTriangulate() throws {
        let url = try #require(GeometricMover.meshURL("stairs"))
        let text = try String(contentsOf: url, encoding: .utf8)
        var positions: [SIMD3<Float>] = [], concave = 0
        for line in text.split(whereSeparator: \.isNewline) {
            let parts = line.split(separator: " ")
            if parts.first == "v" { positions.append([Float(parts[1])!, Float(parts[2])!, Float(parts[3])!]) }
            guard parts.first == "f" else { continue }
            let face = parts.dropFirst().map { UInt32($0.split(separator: "/")[0])! - 1 }
            var n = SIMD3<Float>.zero
            for i in face.indices {
                let p = positions[Int(face[i])], q = positions[Int(face[(i + 1) % face.count])]
                n += SIMD3((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y))
            }
            let tris = GeometricMover.triangulate(face, positions)
            #expect(tris.count == 3 * (face.count - 2), "\(face.count)-gon gave \(tris.count / 3) triangles")
            var area: Float = 0
            for t in stride(from: 0, to: tris.count, by: 3) {
                let a = positions[Int(tris[t])], b = positions[Int(tris[t + 1])], c = positions[Int(tris[t + 2])]
                let tn = simd_cross(b - a, c - a)
                #expect(simd_dot(tn, n) > 0, "triangle wound against its face")
                area += simd_length(tn) / 2
            }
            #expect(abs(area - simd_length(n) / 2) < 1e-3, "triangles cover \(area), face \(simd_length(n) / 2)")
            if face.count > 4 { concave += 1 }
        }
        #expect(concave > 0)
    }

    @Test func fallingOffTheEdgeStartsOver() {
        let (runner, c) = load(at: [0, 1.4, -19.5])
        drive(runner, forward: [0, 0, -1], frames: 120)
        #expect(c.position.y > 0, "y \(c.position.y)")
    }
}

/// Conveyor Belt, Wind and Door, checked for what each shows.
@MainActor
@Suite struct SurfaceSceneTests {
    func body(_ runner: DemoRunner, _ name: String) -> Entity {
        runner.root.children[0].children.first { $0.name == name }!
    }

    /// The belt's surface runs at 2 m/s along its length, turned 0.2 rad about y.
    @Test func beltCarriesBoxes() {
        let runner = DemoRunner(DemoScene.named("Conveyor Belt")!)
        let boxes = runner.root.children[0].children.filter { $0.components[B3PhysicsBodyComponent.self]?.mode == .dynamic }
        runner.advance(frames: 60)
        let before = boxes.map(\.position)
        runner.advance(frames: 60)
        let along = simd_quatf(angle: 0.2, axis: [0, 1, 0]).act([1, 0, 0])
        let onBelt = zip(before, boxes.map(\.position)).filter { $0.0.y > 5 && $0.1.y > 5 }
        #expect(!onBelt.isEmpty)
        for (a, b) in onBelt { #expect(abs(simd_dot(b - a, along) - 2) < 0.3, "moved \(simd_dot(b - a, along)) m in 1 s") }
    }

    @Test func windLiftsTheChain() {
        let runner = DemoRunner(DemoScene.named("Wind")!)
        runner.advance(frames: 240)
        let tip = body(runner, "link9").position
        // With wind off the chain would hang straight down from the anchor at x = 0
        #expect(tip.x > 1, "tip at \(tip)")
        runner.perform("Reverse")
        runner.advance(frames: 360)
        #expect(body(runner, "link9").position.x < -1, "tip at \(body(runner, "link9").position)")
    }

    @Test func doorSwingsWithinItsLimit() {
        let runner = DemoRunner(DemoScene.named("Door")!)
        let door = body(runner, "door")
        runner.advance(frames: 10)
        runner.perform("Impulse")
        var widest: Float = 0
        for _ in 0..<60 {
            runner.advance()
            widest = max(widest, 2 * acos(min(abs(door.orientation.real), 1)))
        }
        #expect(widest > 0.5, "opened \(widest) rad")
        #expect(widest < .pi / 2 + 0.05, "passed its limit: \(widest) rad")
        // Standing on the ground with double gravity, friction holds it part open, as upstream
        runner.advance(frames: 600)
        // On two stiff hinges the door stays upright
        #expect(abs(door.position.y - 1.5) < 0.05, "sagged to \(door.position.y)")
    }
}

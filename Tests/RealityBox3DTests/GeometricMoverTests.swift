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

    @Test func fallingOffTheEdgeStartsOver() {
        let (runner, c) = load(at: [0, 1.4, -19.5])
        drive(runner, forward: [0, 0, -1], frames: 120)
        #expect(c.position.y > 0, "y \(c.position.y)")
    }
}

import Testing
import RealityKit
import simd
@testable import Box3DDemoScenes
@testable import RealityBox3D

@MainActor
@Suite struct DemoSceneTests {
    // Debug builds run the C engine without optimization, so the big benchmarks get a short run.
    static let heavy: Set<String> = ["Large Pyramid", "Convex Pile", "Falling Boxes", "Dominoes", "Wide Pyramid"]

    @Test(arguments: DemoScene.all.map(\.name))
    func sceneLoadsAndSimulates(_ name: String) throws {
        let scene = try #require(DemoScene.named(name))
        let runner = DemoRunner(scene)
        #expect(runner.bodyCount > 0)
        runner.advance(frames: Self.heavy.contains(name) ? 3 : 120)
        for action in scene.actions { runner.perform(action); runner.advance(frames: 5) }
        #expect(runner.world.bodyCount > 0)
        for e in runner.root.children.flatMap(\.children) where e.components.has(B3PhysicsBodyComponent.self) {
            let p = e.position(relativeTo: nil)
            #expect(p.x.isFinite && p.y.isFinite && p.z.isFinite, "\(name): \(e.name) at \(p)")
        }
    }

    @Test func cardHouseStands() {
        let runner = DemoRunner(DemoScene.named("Card House")!)
        let tops = { runner.root.children[0].children.map { $0.position.y }.max()! }
        let before = tops()
        runner.advance(frames: 180)
        #expect(tops() > before - 0.05, "card house collapsed: \(before) -> \(tops())")
    }

    /// The port and upstream's C sample settle the same way: the keystone sinks about 21 cm and holds.
    @Test func archMatchesUpstream() {
        let runner = DemoRunner(DemoScene.named("Arch")!)
        let keystone = runner.root.children[0].children.filter { $0.components[B3PhysicsBodyComponent.self]?.mode == .dynamic }[16]
        runner.advance(frames: 240)
        let port = keystone.position(relativeTo: nil).y
        let raw = archRawKeystoneY()
        #expect(abs(port - raw) < 0.005, "port \(port), upstream \(raw)")
        #expect(raw > -0.5, "upstream arch collapsed")
    }

    @Test func thinWallStopsFastBodies() {
        let runner = DemoRunner(DemoScene.named("Thin Wall")!)
        runner.advance(frames: 60)
        let movers = runner.root.children[0].children.filter { $0.components[B3PhysicsBodyComponent.self]?.mode == .dynamic }
        #expect(movers.count == 3)
        for m in movers { #expect(m.position.z > -0.5, "\(m.position) tunneled") }
    }

    @Test func sensorDestroysVisitor() {
        let runner = DemoRunner(DemoScene.named("Sensor Visit")!)
        let dynamicCount = { runner.root.children[0].children.filter { $0.components[B3PhysicsBodyComponent.self]?.mode == .dynamic }.count }
        #expect(dynamicCount() == 1)
        runner.advance(frames: 180)
        #expect(dynamicCount() == 0)
    }

    @Test func weebleRightsItself() {
        let runner = DemoRunner(DemoScene.named("Weeble")!)
        runner.advance(frames: 30)
        runner.perform("Push")
        runner.advance(frames: 600)
        let weeble = runner.root.children[0].children.first { $0.components[B3PhysicsBodyComponent.self]?.mode == .dynamic }!
        let up = weeble.orientation.act([0, 1, 0])
        #expect(up.y > 0.95, "weeble up axis \(up) at \(weeble.position) mass \(weeble.b3.mass ?? -1)")
    }

    @Test func dzhanibekovFlips() {
        // The handle's spin axis reverses relative to world space within a few seconds.
        let runner = DemoRunner(DemoScene.named("Gyroscopic Torque")!)
        let handle = runner.root.children[0].children.first { $0.components[B3PhysicsBodyComponent.self]?.mode == .dynamic }!
        let axis0 = handle.orientation.act([0, 0, 1])
        var minDot: Float = 1
        for _ in 0..<600 { runner.advance(); minDot = min(minDot, simd_dot(handle.orientation.act([0, 0, 1]), axis0)) }
        #expect(minDot < -0.5, "never flipped, min dot \(minDot)")
    }

    @Test func frictionSortsInclinedBoxes() {
        let runner = DemoRunner(DemoScene.named("Inclined Plane")!)
        runner.advance(frames: 240)
        let boxes = runner.root.children[0].children.filter { $0.components[B3PhysicsBodyComponent.self]?.mode == .dynamic }
            .sorted { $0.position.x < $1.position.x }
        // Low-friction boxes slide off toward +z, the steepest ones (tan 40° = 0.84) stay.
        #expect(boxes.first!.position.z > boxes.last!.position.z + 2)
    }

    /// Scene scripts use the entity API, which must find worlds the system does not own.
    @Test func dominoesStartFalling() {
        let runner = DemoRunner(DemoScene.named("Dominoes")!)
        runner.advance(frames: 30)
        #expect(runner.world.awakeBodyCount >= 30, "awake \(runner.world.awakeBodyCount)")
    }

    @Test func ragdollFallsAndHoldsTogether() {
        let runner = DemoRunner(DemoScene.named("Ragdoll")!)
        let bones = runner.root.children[0].children.filter { $0.components[B3PhysicsBodyComponent.self]?.mode == .dynamic }
        #expect(bones.count == 12)
        let pelvis = bones.first { $0.name == "pelvis" }!, neck = bones.first { $0.name == "neck_01" }!
        let reach0 = simd_distance(pelvis.position, neck.position)
        runner.advance(frames: 240)
        #expect(pelvis.position.y < 0.5, "pelvis at \(pelvis.position.y): did not fall")
        for bone in bones { #expect(bone.position.y > -0.05, "\(bone.name) under the ground at \(bone.position.y)") }
        #expect(abs(simd_distance(pelvis.position, neck.position) - reach0) < 0.15, "ragdoll came apart")
        #expect(runner.world.jointIds.count == 11 + 3)
    }

    @Test func carDrivesAndSteers() {
        let runner = DemoRunner(DemoScene.named("Driving")!)
        let chassis = runner.root.children[0].children.first { $0.components[B3CollisionComponent.self]?.shapes.first.map { if case .box = $0.geometry { true } else { false } } == true }!
        runner.advance(frames: 120) // settle on the suspension
        let start = chassis.position
        runner.perform("Forward")
        runner.advance(frames: 180)
        let straight = chassis.position - start
        #expect(simd_length(SIMD2(straight.x, straight.z)) > 3, "drove \(straight)")
        let heading0 = chassis.orientation.act([1, 0, 0])
        runner.perform("Left")
        // One second: at full speed, a hard turn held longer rolls this softly sprung car, as it should.
        runner.advance(frames: 60)
        let heading1 = chassis.orientation.act([1, 0, 0])
        #expect(acos(min(1, simd_dot(SIMD2(heading0.x, heading0.z).normalized, SIMD2(heading1.x, heading1.z).normalized))) > 0.2, "did not turn")
        #expect(chassis.orientation.act([0, 1, 0]).y > 0.8, "rolled over")
    }
}

private extension SIMD2<Float> {
    var normalized: SIMD2<Float> { simd_normalize(self) }
}

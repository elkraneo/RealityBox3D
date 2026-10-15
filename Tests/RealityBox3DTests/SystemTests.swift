import Testing
import Metal
import RealityKit
import simd
@testable import RealityBox3D

/// The real B3PhysicsSystem, run by RealityKit inside an offscreen renderer.
@MainActor
final class SystemHarness {
    let renderer: RealityRenderer
    let root = Entity()
    private let output: RealityRenderer.CameraOutput

    init() throws {
        RealityBox3D.register()
        renderer = try RealityRenderer()
        root.components.set(B3PhysicsSimulationComponent())
        let camera = PerspectiveCamera()
        renderer.entities.append(contentsOf: [root, camera])
        renderer.activeCamera = camera
        let d = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: 8, height: 8, mipmapped: false)
        d.usage = [.renderTarget]
        d.storageMode = .private
        output = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: MTLCreateSystemDefaultDevice()!.makeTexture(descriptor: d)!))
    }

    func frames(_ n: Int = 1) async throws {
        for _ in 0..<n {
            try await withCheckedThrowingContinuation { (c: CheckedContinuation<Void, any Error>) in
                do { try renderer.updateAndRender(deltaTime: 1.0 / 60, cameraOutput: output, onComplete: { _ in c.resume() }) }
                catch { c.resume(throwing: error) }
            }
        }
    }

    var world: B3World? { RealityBox3D.world(for: root) }

    @discardableResult
    func ground() -> Entity {
        let g = Entity()
        g.position = [0, -0.5, 0]
        g.components.set(B3PhysicsBodyComponent(mode: .static))
        g.components.set(B3CollisionComponent(shapes: [.generateBox(size: [20, 1, 20])]))
        root.addChild(g)
        return g
    }

    @discardableResult
    func box(at p: SIMD3<Float>, parent: Entity? = nil) -> Entity {
        let b = Entity()
        b.position = p
        b.components.set(B3PhysicsBodyComponent(density: 1000))
        b.components.set(B3CollisionComponent(shapes: [.generateBox(size: [0.2, 0.2, 0.2])]))
        (parent ?? root).addChild(b)
        return b
    }
}

@MainActor
@Suite(.serialized) struct SystemTests {
    @Test func sleepingPileReadsNothing() async throws {
        let h = try SystemHarness()
        h.ground()
        let boxes = (0..<30).map { i in h.box(at: [Float(i % 6) * 0.3, 0.1 + Float(i / 6) * 0.21, 0]) }
        try await h.frames(1)
        #expect(h.world?.statistics.entitiesRead == 31) // first frame: full scan
        try await h.frames(300)
        let s = try #require(h.world?.statistics)
        #expect(s.awakeBodies == 0)
        #expect(s.entitiesRead == 0, "read \(s.entitiesRead) entities in a quiet frame")
        #expect(abs(boxes[0].position.y - 0.1) < 0.01)
    }

    @Test func awakeBodiesAreNotReadBack() async throws {
        let h = try SystemHarness()
        h.ground()
        h.box(at: [0, 3, 0])
        try await h.frames(5)
        // The box is falling and written every frame, but the world's own writes are ignored.
        #expect(h.world?.statistics.awakeBodies == 1)
        #expect(h.world?.statistics.entitiesRead == 0)
    }

    @Test func teleportWakesAndMoves() async throws {
        let h = try SystemHarness()
        h.ground()
        let b = h.box(at: [0, 0.1, 0])
        try await h.frames(120)
        #expect(!b.b3.isAwake)
        b.position = [2, 1, 0]
        try await h.frames(1)
        #expect(h.world?.statistics.entitiesRead == 1)
        try await h.frames(90)
        #expect(abs(b.position.x - 2) < 0.01 && abs(b.position.y - 0.1) < 0.01, "\(b.position)")
    }

    @Test func componentEditsApply() async throws {
        let h = try SystemHarness()
        h.ground()
        let b = h.box(at: [0, 0.1, 0])
        try await h.frames(10)
        b.components[B3PhysicsBodyComponent.self]!.isAffectedByGravity = false
        b.components.set(B3PhysicsMotionComponent(linearVelocity: [0, 1, 0]))
        try await h.frames(60)
        #expect(b.position.y > 0.8, "\(b.position)")
    }

    @Test func addingAndRemovingEntities() async throws {
        let h = try SystemHarness()
        h.ground()
        let a = h.box(at: [0, 0.1, 0])
        try await h.frames(5)
        #expect(h.world?.bodyCount == 2)
        let late = h.box(at: [1, 2, 0])
        try await h.frames(60)
        #expect(h.world?.bodyCount == 3)
        #expect(late.position.y < 0.2)
        a.removeFromParent()
        try await h.frames(1)
        #expect(h.world?.bodyCount == 2)
        late.components.remove(B3CollisionComponent.self)
        try await h.frames(1)
        #expect(h.world?.bodyCount == 1)
    }

    @Test func movingAnAncestorMovesItsBodies() async throws {
        let h = try SystemHarness()
        h.ground()
        let holder = Entity()
        h.root.addChild(holder)
        let b = h.box(at: [0, 0.1, 0], parent: holder)
        try await h.frames(120)
        #expect(!b.b3.isAwake)
        holder.position = [3, 2, 0] // carries the sleeping box up and over
        try await h.frames(120)
        let p = b.position(relativeTo: h.root)
        #expect(abs(p.x - 3) < 0.02 && abs(p.y - 0.1) < 0.02, "\(p)")
    }
}

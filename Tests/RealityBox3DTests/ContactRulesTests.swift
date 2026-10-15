import Testing
import RealityKit
import simd
import CBox3D
@testable import RealityBox3D

/// One-way surfaces and material combine modes, both through Box3D's solver callbacks.
@MainActor
@Suite struct ContactRulesTests {
    func root(_ configure: (inout B3PhysicsSimulationComponent) -> Void = { _ in }) -> Entity {
        let r = Entity()
        var s = B3PhysicsSimulationComponent()
        s.interpolatesPoses = false
        configure(&s)
        r.components.set(s)
        return r
    }

    @discardableResult
    func body(_ r: Entity, _ shape: B3ShapeResource, at p: SIMD3<Float>, mode: PhysicsBodyMode = .dynamic,
              material: B3PhysicsMaterial? = nil, configure: (inout B3CollisionComponent) -> Void = { _ in }) -> Entity {
        let e = Entity()
        e.position = p
        e.components.set(B3PhysicsBodyComponent(density: 1000, material: material, mode: mode))
        var c = B3CollisionComponent(shapes: [shape])
        configure(&c)
        e.components.set(c)
        r.addChild(e)
        return e
    }

    func run(_ r: Entity, _ es: [Entity], frames n: Int) -> B3World {
        let w = B3World(root: r, settings: r.components[B3PhysicsSimulationComponent.self]!)
        for _ in 0..<n { w.update(entities: es, deltaTime: 1.0 / 60) }
        return w
    }

    // A ball thrown up from below passes through a one-way platform and lands on top of it.
    @Test func oneWayPlatformLetsBodiesUpThrough() {
        let r = root()
        let platform = body(r, .generateBox(size: [4, 0.2, 4]), at: [0, 2, 0], mode: .static) { $0.oneWayDirection = [0, 1, 0] }
        let ball = body(r, .generateSphere(radius: 0.2), at: [0, 0.5, 0])
        ball.components.set(B3PhysicsMotionComponent(linearVelocity: [0, 9, 0]))
        let w = run(r, [platform, ball], frames: 180)
        #expect(abs(ball.position.y - 2.3) < 0.05, "the ball should rest on the platform: \(ball.position)")
        _ = w
    }

    // From above, the same platform holds; a platform without a direction stops the throw.
    @Test func oneWayPlatformHoldsFromAbove() {
        let r = root()
        let platform = body(r, .generateBox(size: [4, 0.2, 4]), at: [0, 2, 0], mode: .static) { $0.oneWayDirection = [0, 1, 0] }
        let ball = body(r, .generateSphere(radius: 0.2), at: [0, 4, 0])
        _ = run(r, [platform, ball], frames: 180)
        #expect(abs(ball.position.y - 2.3) < 0.05, "ball at \(ball.position)")

        let r2 = root()
        let solid = body(r2, .generateBox(size: [4, 0.2, 4]), at: [0, 2, 0], mode: .static)
        let thrown = body(r2, .generateSphere(radius: 0.2), at: [0, 0.5, 0])
        thrown.components.set(B3PhysicsMotionComponent(linearVelocity: [0, 9, 0]))
        _ = run(r2, [solid, thrown], frames: 30)
        #expect(thrown.position.y < 1.9, "a solid platform should stop the throw: \(thrown.position)")
    }

    // The direction turns with the platform: rotated upside down, it holds from below.
    @Test func oneWayDirectionFollowsRotation() {
        let r = root()
        let platform = body(r, .generateBox(size: [4, 0.2, 4]), at: [0, 2, 0], mode: .static) { $0.oneWayDirection = [0, 1, 0] }
        platform.orientation = simd_quatf(angle: .pi, axis: [1, 0, 0])
        let ball = body(r, .generateSphere(radius: 0.2), at: [0, 0.5, 0])
        ball.components.set(B3PhysicsMotionComponent(linearVelocity: [0, 9, 0]))
        _ = run(r, [platform, ball], frames: 30)
        #expect(ball.position.y < 1.9, "an upside-down platform should stop the throw: \(ball.position)")
    }

    /// How far a box slides on a 0.2-friction floor at 3 m/s, with its own friction 1.
    func slide(_ combine: B3MaterialCombine) -> Float {
        let r = root { $0.frictionCombine = combine }
        let floor = body(r, .generateBox(size: [40, 1, 4]), at: [0, -0.5, 0], mode: .static,
                         material: .generate(staticFriction: 0.2, dynamicFriction: 0.2))
        let box = body(r, .generateBox(size: [0.5, 0.5, 0.5]), at: [-15, 0.25, 0],
                       material: .generate(staticFriction: 1, dynamicFriction: 1))
        box.components.set(B3PhysicsMotionComponent(linearVelocity: [3, 0, 0]))
        _ = run(r, [floor, box], frames: 240)
        return box.position.x + 15
    }

    @Test func frictionCombines() {
        // Sliding distance is v² / (2 μ g): μ = 0.2 (minimum) slides five times as far as 1.0 (maximum).
        let minimum = slide(.minimum), average = slide(.average), maximum = slide(.maximum)
        #expect(abs(minimum - 9 / (2 * 0.2 * 9.81)) < 0.2, "minimum slid \(minimum)")
        #expect(abs(average - 9 / (2 * 0.6 * 9.81)) < 0.1, "average slid \(average)")
        #expect(abs(maximum - 9 / (2 * 1.0 * 9.81)) < 0.1, "maximum slid \(maximum)")
    }

    /// The highest point a dead ball reaches after falling 2 m onto a fully elastic floor.
    func bounce(_ combine: B3MaterialCombine) -> Float {
        let r = root { $0.restitutionCombine = combine }
        let floor = body(r, .generateBox(size: [4, 1, 4]), at: [0, -0.5, 0], mode: .static,
                         material: .generate(restitution: 1))
        let ball = body(r, .generateSphere(radius: 0.2), at: [0, 2.2, 0], material: .generate(restitution: 0))
        let w = B3World(root: r, settings: r.components[B3PhysicsSimulationComponent.self]!)
        var landed = false, peak: Float = 0
        for _ in 0..<180 {
            w.update(entities: [floor, ball], deltaTime: 1.0 / 60)
            if ball.position.y < 0.3 { landed = true }
            if landed { peak = max(peak, ball.position.y) }
        }
        return peak
    }

    @Test func restitutionCombines() {
        #expect(bounce(.maximum) > 1.5, "maximum should bounce")
        #expect(bounce(.minimum) < 0.3, "minimum should not bounce")
        #expect(bounce(.multiply) < 0.3, "multiply should not bounce")
    }

    // Changing the combine mode on the root applies to new contacts.
    @Test func combineModeChangesApply() {
        let r = root()
        let w = B3World(root: r, settings: r.components[B3PhysicsSimulationComponent.self]!)
        r.components[B3PhysicsSimulationComponent.self]!.frictionCombine = .minimum
        w.update(entities: [], deltaTime: 1.0 / 60)
        #expect(w.settings.frictionCombine == .minimum)
    }
}

/// Explosions through `b3World_Explode`.
@MainActor
@Suite struct ExplosionTests {
    @Test func blastPushesBodiesAway() {
        let r = Entity()
        var s = B3PhysicsSimulationComponent()
        s.gravity = .zero
        s.interpolatesPoses = false
        r.components.set(s)
        let positions: [SIMD3<Float>] = [[1, 0, 0], [-1, 0, 0], [0, 0, 1], [5, 0, 0]]
        let balls = positions.map { p -> Entity in
            let e = Entity()
            e.position = p
            e.components.set(B3PhysicsBodyComponent(density: 1000))
            e.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 0.2)]))
            r.addChild(e)
            return e
        }
        let w = B3World(root: r, settings: s)
        w.update(entities: balls, deltaTime: 1.0 / 60)
        w.explode(at: .zero, radius: 2, impulsePerArea: 50)
        for _ in 0..<30 { w.update(entities: balls, deltaTime: 1.0 / 60) }
        for (ball, start) in zip(balls.prefix(3), positions) {
            // Each moved straight away from the center.
            let moved = ball.position - start
            #expect(simd_dot(simd_normalize(moved), simd_normalize(start)) > 0.99, "ball from \(start) moved \(moved)")
        }
        #expect(simd_distance(balls[3].position, positions[3]) < 1e-4, "a ball out of range moved")
    }

    @Test func malformedBlastIsIgnored() {
        let r = Entity()
        r.components.set(B3PhysicsSimulationComponent())
        let w = B3World(root: r)
        w.explode(at: [.nan, 0, 0], radius: 2, impulsePerArea: 1)
        w.explode(at: .zero, radius: -1, impulsePerArea: 1)
    }
}

/// Surfaces per shape and per triangle.
@MainActor
@Suite struct SurfaceMaterialTests {
    /// How far a box slides at 3 m/s, its own friction 1 and the combine mode the minimum, so the
    /// floor's surface under it decides.
    func slide(on floor: B3ShapeResource, from x: Float = -15) -> Float {
        let r = Entity()
        var s = B3PhysicsSimulationComponent()
        s.frictionCombine = .minimum
        s.interpolatesPoses = false
        r.components.set(s)
        let ground = Entity()
        ground.components.set(B3PhysicsBodyComponent(density: 1000, material: .generate(staticFriction: 1, dynamicFriction: 1), mode: .static))
        ground.components.set(B3CollisionComponent(shapes: [floor]))
        r.addChild(ground)
        let box = Entity()
        box.position = [x, 0.25, 0]
        box.components.set(B3PhysicsBodyComponent(density: 1000, material: .generate(staticFriction: 1, dynamicFriction: 1)))
        box.components.set(B3CollisionComponent(shapes: [.generateBox(size: [0.5, 0.5, 0.5])]))
        box.components.set(B3PhysicsMotionComponent(linearVelocity: [3, 0, 0]))
        r.addChild(box)
        let w = B3World(root: r, settings: s)
        for _ in 0..<240 { w.update(entities: [ground, box], deltaTime: 1.0 / 60) }
        return box.position.x - x
    }

    @Test func shapeMaterialOverridesTheBody() {
        let slippery = B3ShapeResource.generateBox(size: [40, 1, 4]).offsetBy(translation: [0, -0.5, 0])
            .withMaterial(.generate(staticFriction: 0.2, dynamicFriction: 0.2))
        let d = slide(on: slippery)
        #expect(abs(d - 9 / (2 * 0.2 * 9.81)) < 0.2, "slid \(d)")
    }

    @Test func triangleMaterialsApplyPerTriangle() {
        // A 40 × 4 floor of two quads: x < 0 is ice (0.05), x ≥ 0 rubber (1).
        let p: [SIMD3<Float>] = [[-20, 0, -2], [0, 0, -2], [0, 0, 2], [-20, 0, 2], [20, 0, -2], [20, 0, 2]]
        let i: [UInt32] = [0, 2, 1, 0, 3, 2, 1, 5, 4, 1, 2, 5]
        let floor = B3ShapeResource.generateStaticMesh(positions: p, faceIndices: i, triangleMaterials: .init(
            materials: [.generate(staticFriction: 0.05, dynamicFriction: 0.05), .generate(staticFriction: 1, dynamicFriction: 1)],
            indices: [0, 0, 1, 1]))
        let onIce = slide(on: floor, from: -15)
        let onRubber = slide(on: floor, from: 2)
        // Well past what rubber allows; the mesh's edges add a little drag over the ice's own.
        #expect(onIce > 5, "on ice it slid \(onIce)")
        #expect(abs(onRubber - 9 / (2 * 1.0 * 9.81)) < 0.1, "on rubber it slid \(onRubber)")
    }

    @Test func malformedTriangleMaterialsFallBack() {
        let p: [SIMD3<Float>] = [[-20, 0, -2], [20, 0, -2], [20, 0, 2], [-20, 0, 2]]
        let floor = B3ShapeResource.generateStaticMesh(positions: p, faceIndices: [0, 2, 1, 0, 3, 2], triangleMaterials: .init(
            materials: [.generate(staticFriction: 0.05, dynamicFriction: 0.05)], indices: [0, 5]))
        // The bad index is reported and the mesh keeps the body's surface (friction 1).
        let d = slide(on: floor)
        #expect(abs(d - 9 / (2 * 1.0 * 9.81)) < 0.1, "slid \(d)")
    }
}

/// Static bodies baked into one compound shape.
@MainActor
@Suite struct CompoundTests {
    func scene(baked: Bool) -> (Entity, Entity, [Entity], B3World) {
        let r = Entity()
        var s = B3PhysicsSimulationComponent()
        s.interpolatesPoses = false
        r.components.set(s)
        let level = Entity()
        level.components.set(B3PhysicsBodyComponent(density: 1000, mode: .static))
        var c = B3CollisionComponent(shapes: [
            B3ShapeResource.generateBox(size: [2, 1, 2]).offsetBy(translation: [-3, 0.5, 0]),
            B3ShapeResource.generateSphere(radius: 1).offsetBy(translation: [0, 0, 0]),
            B3ShapeResource.generateCapsule(height: 2, radius: 0.5).offsetBy(rotation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1]), translation: [3, 0.5, 0]),
            .generateStaticMesh(positions: [[-10, -1, -10], [10, -1, -10], [10, -1, 10], [-10, -1, 10]], faceIndices: [UInt32(0), 2, 1, 0, 3, 2]),
        ])
        c.bakesCompound = baked
        level.components.set(c)
        r.addChild(level)
        let balls = [Float(-3), 0, 3].map { x -> Entity in
            let e = Entity()
            e.position = [x, 4, 0]
            e.components.set(B3PhysicsBodyComponent(density: 1000))
            e.components.set(B3CollisionComponent(shapes: [.generateBox(size: [0.2, 0.2, 0.2])]))
            r.addChild(e)
            return e
        }
        let w = B3World(root: r, settings: s)
        for _ in 0..<180 { w.update(entities: [level] + balls, deltaTime: 1.0 / 60) }
        return (r, level, balls, w)
    }

    @Test func bakedCompoundCollidesLikeItsShapes() {
        let (_, level, baked, w) = scene(baked: true)
        let (_, _, loose, _) = scene(baked: false)
        #expect(b3Body_GetShapeCount(w.record(for: level)!.bodyId) == 1)
        for (a, b) in zip(baked, loose) {
            #expect(simd_distance(a.position, b.position) < 0.02, "baked \(a.position), loose \(b.position)")
        }
        // Landed on the box, sphere and capsule tops, not the floor.
        #expect(baked.allSatisfy { $0.position.y > 0.9 }, "\(baked.map(\.position))")
    }

    @Test func bakedCompoundRebakesOnMaterialChange() {
        let (_, level, _, w) = scene(baked: true)
        level.components[B3PhysicsBodyComponent.self]!.material = .generate(staticFriction: 0.1, dynamicFriction: 0.1)
        w.update(entities: [level], deltaTime: 1.0 / 60)
        #expect(b3Body_GetShapeCount(w.record(for: level)!.bodyId) == 1)
    }

    @Test func compoundOnADynamicBodyStaysSeparate() {
        let r = Entity()
        r.components.set(B3PhysicsSimulationComponent())
        let e = Entity()
        e.components.set(B3PhysicsBodyComponent(density: 1000))
        var c = B3CollisionComponent(shapes: [.generateSphere(radius: 0.5), B3ShapeResource.generateSphere(radius: 0.5).offsetBy(translation: [1, 0, 0])])
        c.bakesCompound = true
        e.components.set(c)
        r.addChild(e)
        let w = B3World(root: r)
        w.update(entities: [e], deltaTime: 1.0 / 60)
        #expect(b3Body_GetShapeCount(w.record(for: e)!.bodyId) == 2)
    }
}

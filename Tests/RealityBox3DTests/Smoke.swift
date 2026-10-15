import Testing
import RealityKit
import simd
@testable import RealityBox3D
import CBox3D

@MainActor
func makeBox(_ size: Float = 0.2, at p: SIMD3<Float>, mode: PhysicsBodyMode = .dynamic, density: Float = 1000) -> Entity {
    let e = Entity()
    e.position = p
    e.components.set(B3CollisionComponent(shapes: [.generateBox(size: [size, size, size])]))
    e.components.set(B3PhysicsBodyComponent(density: density, mode: mode))
    return e
}

@MainActor
func makeGround(parent: Entity) -> Entity {
    let g = Entity()
    g.position = [0, -0.5, 0]
    g.components.set(B3CollisionComponent(shapes: [.generateBox(size: [20, 1, 20])]))
    g.components.set(B3PhysicsBodyComponent(mode: .static))
    parent.addChild(g)
    return g
}

@MainActor
func run(_ world: B3World, _ entities: [Entity], seconds: Double, dt: Double = 1.0 / 90) {
    for _ in 0..<Int(seconds / dt) { world.update(entities: entities, deltaTime: dt) }
}

@MainActor
@Suite struct WorldTests {
    @Test func boxSettlesOnGroundAndSleeps() {
        let root = Entity()
        root.components.set(B3PhysicsSimulationComponent())
        let ground = makeGround(parent: root)
        let box = makeBox(at: [0, 2, 0]); root.addChild(box)
        let world = B3World(root: root)
        run(world, [ground, box], seconds: 3)
        #expect(abs(box.position.y - 0.1) < 0.01, "y = \(box.position.y)")
        #expect(world.awakeBodyCount == 0)
    }

    @Test func respectsSimulationRootTransform() {
        let scene = Entity()
        let root = Entity()
        root.position = [5, 10, 0]
        root.orientation = simd_quatf(angle: .pi / 2, axis: [0, 0, 1]) // gravity now points along root's -Y
        root.components.set(B3PhysicsSimulationComponent())
        scene.addChild(root)
        let ground = makeGround(parent: root)
        let box = makeBox(at: [0, 2, 0]); root.addChild(box)
        let world = B3World(root: root)
        run(world, [ground, box], seconds: 3)
        #expect(abs(box.position.y - 0.1) < 0.01, "local y = \(box.position.y)")
    }

    @Test func teleportIsHonored() {
        let root = Entity()
        let ground = makeGround(parent: root)
        let box = makeBox(at: [0, 2, 0]); root.addChild(box)
        let world = B3World(root: root)
        run(world, [ground, box], seconds: 0.5)
        box.position = [3, 5, 0]
        run(world, [ground, box], seconds: 0.05)
        #expect(abs(box.position.x - 3) < 0.01)
        #expect(box.position.y > 4.5)
    }

    @Test func motionComponentDrivesVelocity() {
        let root = Entity()
        let box = makeBox(at: [0, 0, 0])
        box.components[B3PhysicsBodyComponent.self]!.isAffectedByGravity = false
        box.components.set(B3PhysicsMotionComponent(linearVelocity: [1, 0, 0]))
        root.addChild(box)
        let world = B3World(root: root)
        run(world, [box], seconds: 1)
        #expect(abs(box.position.x - 1) < 0.03, "x = \(box.position.x)")
        #expect(abs(box.components[B3PhysicsMotionComponent.self]!.linearVelocity.x - 1) < 0.01)
    }

    @Test func kinematicFollowsEntityAndPushesDynamic() {
        let root = Entity()
        let ground = makeGround(parent: root)
        let pusher = makeBox(0.5, at: [-1, 0.25, 0], mode: .kinematic); root.addChild(pusher)
        let box = makeBox(at: [0, 0.1, 0]); root.addChild(box)
        let world = B3World(root: root)
        let dt = 1.0 / 90
        for i in 0..<Int(1.5 / dt) {
            pusher.position.x = -1 + Float(Double(i) * dt) * 1.0 // 1 m/s
            world.update(entities: [ground, pusher, box], deltaTime: dt)
        }
        #expect(box.position.x > 0.3, "box x = \(box.position.x)")
        #expect(abs(pusher.position.x - 0.5) < 0.02) // kinematic pose is owned by the app
    }

    @Test func collisionEventsFire() {
        let root = Entity()
        let ground = makeGround(parent: root)
        let box = makeBox(at: [0, 1, 0]); root.addChild(box)
        let world = B3World(root: root)
        var began = 0
        var hits: [Float] = []
        let s1 = world.subscribe(to: B3CollisionEvents.Began.self, on: box) { _ in began += 1 }
        let s2 = world.subscribe(to: B3CollisionEvents.Hit.self) { hits.append($0.approachSpeed) }
        run(world, [ground, box], seconds: 1.5)
        #expect(began >= 1)
        #expect(hits.contains { $0 > 3 }, "hits \(hits)")
        s1.cancel(); s2.cancel()
    }

    @Test func triggerReportsOverlapWithoutResponse() {
        let root = Entity()
        let sensor = Entity()
        sensor.components.set(B3CollisionComponent(shapes: [.generateBox(size: [2, 0.2, 2])], mode: .trigger))
        sensor.components.set(B3PhysicsBodyComponent(mode: .static))
        root.addChild(sensor)
        let box = makeBox(at: [0, 1, 0]); root.addChild(box)
        let world = B3World(root: root)
        var entered = false
        let subscription = world.subscribe(to: B3CollisionEvents.Began.self, on: sensor) { _ in entered = true }
        run(world, [sensor, box], seconds: 1)
        #expect(entered)
        subscription.cancel()
        #expect(box.position.y < -1, "fell through sensor: y = \(box.position.y)")
    }

    @Test func filterPreventsCollision() {
        let root = Entity()
        let ground = makeGround(parent: root)
        let a = CollisionGroup(rawValue: 1 << 1), b = CollisionGroup(rawValue: 1 << 2)
        ground.components[B3CollisionComponent.self]!.filter = .init(group: a, mask: .all.subtracting(b))
        let box = makeBox(at: [0, 1, 0])
        box.components[B3CollisionComponent.self]!.filter = .init(group: b, mask: .all)
        root.addChild(box)
        let world = B3World(root: root)
        run(world, [ground, box], seconds: 1)
        #expect(box.position.y < -1)
    }

    @Test func removingEntityDestroysBody() {
        let root = Entity()
        let ground = makeGround(parent: root)
        let box = makeBox(at: [0, 1, 0]); root.addChild(box)
        let world = B3World(root: root)
        run(world, [ground, box], seconds: 0.1)
        #expect(world.bodyCount == 2)
        box.components.remove(B3PhysicsBodyComponent.self)
        run(world, [ground, box], seconds: 0.1)
        #expect(world.bodyCount == 1)
    }

    @Test func raycastHitsNearestFirst() {
        let root = Entity()
        let near = makeBox(1, at: [0, 0, -2], mode: .static); root.addChild(near)
        let far = makeBox(1, at: [0, 0, -5], mode: .static); root.addChild(far)
        let world = B3World(root: root)
        world.update(entities: [near, far], deltaTime: 1.0 / 60)
        let all = world.raycast(origin: [0, 0, 0], direction: [0, 0, -1], length: 10)
        #expect(all.map(\.entity) == [near, far])
        #expect(abs(all[0].distance - 1.5) < 1e-3)
        let nearest = world.raycast(origin: [0, 0, 0], direction: [0, 0, -1], length: 10, query: .nearest)
        #expect(nearest.first?.entity == near)
    }

    @Test func explicitMassIsApplied() throws {
        let root = Entity()
        let box = Entity()
        box.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 0.5)]))
        box.components.set(B3PhysicsBodyComponent(massProperties: .init(mass: 7)))
        root.addChild(box)
        let world = B3World(root: root)
        world.update(entities: [box], deltaTime: 0)
        let record = try #require(world.record(for: box))
        #expect(abs(b3Body_GetMass(record.bodyId) - 7) < 1e-4)
    }

    @Test func meshShapesAndConvexHulls() {
        let root = Entity()
        let ground = Entity()
        let mesh = MeshResource.generatePlane(width: 10, depth: 10)
        ground.components.set(B3CollisionComponent(shapes: [.generateStaticMesh(from: mesh)]))
        ground.components.set(B3PhysicsBodyComponent(mode: .static))
        root.addChild(ground)
        let rock = Entity()
        rock.position = [0, 1, 0]
        rock.components.set(B3CollisionComponent(shapes: [.generateConvex(from: .generateSphere(radius: 0.25))]))
        rock.components.set(B3PhysicsBodyComponent(density: 500))
        root.addChild(rock)
        let world = B3World(root: root)
        run(world, [ground, rock], seconds: 3)
        #expect(abs(rock.position.y - 0.25) < 0.02, "y = \(rock.position.y)")
    }

    @Test func editsApplyInPlace() throws {
        let root = Entity()
        let ground = makeGround(parent: root)
        let box = makeBox(at: [0, 1, 0]); root.addChild(box)
        let world = B3World(root: root)
        run(world, [ground, box], seconds: 0.2)
        let id = try #require(world.record(for: box)).bodyId

        box.components[B3PhysicsBodyComponent.self]!.material = .generate(dynamicFriction: 0.1, restitution: 0.5)
        box.components[B3PhysicsBodyComponent.self]!.linearDamping = 2
        box.components[B3CollisionComponent.self]!.filter = .init(group: .init(rawValue: 4), mask: .all)
        run(world, [ground, box], seconds: 0.05)
        #expect(world.record(for: box)!.bodyId == id)
        #expect(b3Body_GetLinearDamping(id) == 2)

        // Geometry edits replace shapes, not the body, and keep it moving.
        let v = b3Body_GetLinearVelocity(id).simd
        box.components[B3CollisionComponent.self]!.shapes = [.generateSphere(radius: 0.1)]
        world.update(entities: [ground, box], deltaTime: 0)
        #expect(world.record(for: box)!.bodyId == id)
        #expect(simd_distance(b3Body_GetLinearVelocity(id).simd, v) < 1e-4)

        box.components[B3PhysicsBodyComponent.self]!.mode = .kinematic
        run(world, [ground, box], seconds: 0.05)
        #expect(b3Body_GetType(id) == b3_kinematicBody)
    }

    @Test func sleepingBodiesLeaveTheActiveSet() {
        let root = Entity()
        let ground = makeGround(parent: root)
        let boxes = (0..<20).map { i in makeBox(at: [Float(i % 5) * 0.3, 0.1 + Float(i / 5) * 0.21, 0]) }
        for b in boxes { root.addChild(b) }
        let world = B3World(root: root)
        run(world, [ground] + boxes, seconds: 0.2)
        #expect(world.statistics.awakeBodies > 0)
        run(world, [ground] + boxes, seconds: 4)
        #expect(world.statistics.awakeBodies == 0)
        #expect(world.activeCount == 0)
    }

    /// Bodies under an intermediate parent, uniformly scaled (plain math path) and non-uniformly
    /// scaled (RealityKit conversion path), settle where they would directly under the root.
    @Test(arguments: [SIMD3<Float>(2, 2, 2), SIMD3<Float>(1, 2, 1)])
    func nestedParentsKeepRootSpace(_ parentScale: SIMD3<Float>) {
        let root = Entity()
        let ground = makeGround(parent: root)
        let holder = Entity()
        holder.position = [3, 0, -1]
        holder.orientation = simd_quatf(angle: 0.7, axis: [0, 1, 0])
        holder.scale = parentScale
        root.addChild(holder)
        let box = makeBox(at: .zero)
        holder.addChild(box)
        box.setPosition([3, 2, -1], relativeTo: root)
        box.scale = 1 / parentScale // keep it a 20 cm cube in root space
        let world = B3World(root: root)
        run(world, [ground, box], seconds: 3)
        let p = box.position(relativeTo: root)
        #expect(abs(p.y - 0.1) < 0.01, "y = \(p.y)")
        #expect(abs(p.x - 3) < 0.01 && abs(p.z + 1) < 0.01, "drifted to \(p)")
    }

    /// A forecast taken mid-pile, with live contacts, matches the real world step for step.
    @Test func forecastIsExact() throws {
        let root = Entity()
        let ground = makeGround(parent: root)
        let boxes = (0..<12).map { i in makeBox(at: [Float(i % 3) * 0.15, 0.3 + Float(i) * 0.25, Float(i % 2) * 0.05]) }
        for b in boxes { root.addChild(b) }
        let world = B3World(root: root, settings: { var s = B3PhysicsSimulationComponent(); s.interpolatesPoses = false; return s }())
        run(world, [ground] + boxes, seconds: 0.5, dt: 1.0 / 60)
        let forecast = try #require(world.forecast())
        forecast.step(90)
        run(world, [ground] + boxes, seconds: 1.5, dt: 1.0 / 60)
        for b in boxes {
            let f = try #require(forecast.transform(of: b))
            #expect(f.translation == b.position, "\(f.translation) vs \(b.position)")
        }
    }

    /// Generated cylinders and cones are centered, like RealityKit's mesh generators.
    @Test func generatedHullsAreCentered() {
        for shape in [B3ShapeResource.generateCylinder(height: 2, radius: 0.5), .generateCone(height: 2, bottomRadius: 0.5)] {
            guard case .convex(let points) = shape.geometry else { Issue.record("not convex"); continue }
            let ys = points.map(\.y)
            #expect(abs(ys.min()! + 1) < 1e-5 && abs(ys.max()! - 1) < 1e-5, "\(ys.min()!)...\(ys.max()!)")
        }
    }

    @Test func overlapAndConvexCast() {
        let root = Entity()
        let a = makeBox(1, at: [0, 0, 0], mode: .static); root.addChild(a)
        let b = makeBox(1, at: [3, 0, 0], mode: .static); root.addChild(b)
        let world = B3World(root: root)
        world.update(entities: [a, b], deltaTime: 0)
        let probe = B3ShapeResource.generateSphere(radius: 0.4)
        #expect(world.overlaps(of: probe, at: Transform(translation: [0.8, 0, 0])) == [a])
        // At x = 1.2 the probe is 0.3 m clear of a and 0.9 m clear of b.
        #expect(world.overlaps(of: probe, at: Transform(translation: [1.2, 0, 0])).isEmpty)
        #expect(world.overlaps(of: probe, at: Transform(translation: [1.2, 0, 0]), margin: 0.4) == [a])
        let ring = B3ShapeResource.generateBox(size: [0.2, 0.2, 0.2])
        let hits = world.convexCast(convexShape: ring, fromPosition: [-3, 0, 0], fromOrientation: .init(ix: 0, iy: 0, iz: 0, r: 1), toPosition: [6, 0, 0])
        #expect(hits.map(\.entity) == [a, b])
        #expect(abs(hits[0].distance - 2.4) < 0.01, "first hit at \(hits[0].distance)")
        let nearest = world.convexCast(convexShape: ring, fromPosition: [6, 0, 0], fromOrientation: .init(ix: 0, iy: 0, iz: 0, r: 1), toPosition: [-3, 0, 0], query: .nearest)
        #expect(nearest.map(\.entity) == [b])
    }
}

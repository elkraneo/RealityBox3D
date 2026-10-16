import Testing
import RealityKit
import simd
import CBox3D
@testable import Box3DDemoScenes
@testable import RealityBox3D

/// One test per finding of the adversarial review, so each fix stays fixed.
@MainActor
@Suite(.serialized) struct ReviewRegressionTests {
    func root() -> Entity {
        let r = Entity()
        var s = B3PhysicsSimulationComponent()
        s.interpolatesPoses = false
        r.components.set(s)
        return r
    }

    func body(_ parent: Entity, _ shapes: [B3ShapeResource], at p: SIMD3<Float> = .zero, mode: PhysicsBodyMode = .dynamic) -> Entity {
        let e = Entity()
        e.position = p
        e.components.set(B3PhysicsBodyComponent(density: 1000, mode: mode))
        e.components.set(B3CollisionComponent(shapes: shapes))
        parent.addChild(e)
        return e
    }

    func world(_ r: Entity) -> B3World { B3World(root: r, settings: r.components[B3PhysicsSimulationComponent.self]!) }

    func frames(_ w: B3World, _ es: [Entity], _ n: Int) { for _ in 0..<n { w.update(entities: es, deltaTime: 1.0 / 60) } }

    func shapeCount(_ w: B3World, _ e: Entity) -> Int { Int(b3Body_GetShapeCount(w.record(for: e)!.bodyId)) }

    // 1. Malformed geometry is rejected before it reaches C.
    @Test func malformedGeometryIsRejected() {
        let r = root()
        let field = body(r, [.generateHeightField(heights: [0], countX: 50, countZ: 50)], mode: .static)
        let mesh = body(r, [.generateStaticMesh(positions: [.zero, [1, 0, 0], [0, 0, 1]], faceIndices: [0, 1, UInt32.max])], mode: .static)
        let w = world(r)
        frames(w, [field, mesh], 2)
        #expect(shapeCount(w, field) == 0)
        #expect(shapeCount(w, mesh) == 0)
    }

    // 10. A rotated box under non-uniform scale is rotated, then scaled.
    @Test func rotatedBoxScalesAfterRotation() {
        let r = root()
        let e = body(r, [B3ShapeResource.generateBox(size: [2, 1, 1]).offsetBy(rotation: simd_quatf(angle: .pi / 2, axis: [0, 1, 0]))], mode: .static)
        e.scale = [2, 1, 1]
        let w = world(r)
        frames(w, [e], 1)
        // Rotated, the 2 m side lies along z and the 1 m side along x; scaling x by 2 makes 2 × 1 × 2.
        let fromX = w.raycast(origin: [5, 0, 0], direction: [-1, 0, 0], length: 10, query: .nearest).first!
        let fromZ = w.raycast(origin: [0, 0, 5], direction: [0, 0, -1], length: 10, query: .nearest).first!
        #expect(abs(fromX.position.x - 1) < 1e-3, "x extent \(fromX.position.x)")
        #expect(abs(fromZ.position.z - 1) < 1e-3, "z extent \(fromZ.position.z)")
    }

    // 11. An offset height field collides where it renders.
    @Test func offsetHeightFieldIsOffset() {
        let r = root()
        let terrain = B3ShapeResource.generateHeightField(heights: [Float](repeating: 0, count: 9), countX: 3, countZ: 3, scale: [1, 1, 1])
            .offsetBy(translation: [0, 10, 0])
        let e = body(r, [terrain], mode: .static)
        let w = world(r)
        frames(w, [e], 1)
        let hit = w.raycast(origin: [1, 20, 1], direction: [0, -1, 0], length: 30, query: .nearest).first
        #expect(abs((hit?.position.y ?? -1) - 10) < 1e-3, "hit at \(String(describing: hit?.position))")
    }

    // 12. Overlap queries use the pose's scale.
    @Test func overlapUsesPoseScale() {
        let r = root()
        let target = body(r, [.generateBox(size: [1, 1, 1])], at: [3, 0, 0], mode: .static)
        let w = world(r)
        frames(w, [target], 1)
        let probe = B3ShapeResource.generateSphere(radius: 0.5)
        #expect(w.overlaps(of: probe, at: Transform(translation: .zero)).isEmpty)
        #expect(w.overlaps(of: probe, at: Transform(scale: SIMD3(repeating: 6), translation: .zero)) == [target])
    }

    // 5. An impulse survives the next full scan when the entity carries a motion component.
    @Test func impulseIsNotUndoneBySync() {
        let r = root()
        let e = body(r, [.generateSphere(radius: 0.5)])
        e.components[B3PhysicsBodyComponent.self]!.isAffectedByGravity = false
        e.components.set(B3PhysicsMotionComponent())
        let w = world(r)
        frames(w, [e], 1)
        e.b3.applyLinearImpulse([0, 0, 523.6], relativeTo: nil) // ~1 m/s for a 523.6 kg ball
        frames(w, [e], 2)
        #expect((e.b3.linearVelocity?.z ?? 0) > 0.9, "velocity \(String(describing: e.b3.linearVelocity))")
        #expect(abs((e.components[B3PhysicsMotionComponent.self]?.linearVelocity.z ?? 0) - 1) < 0.05)
    }

    // An impulse does not add a motion component the app never set, nor one it removed after
    // the world last read the entity.
    @Test(arguments: [false, true]) func impulseAddsNoMotionComponent(removedAfterRead: Bool) {
        let r = root()
        let e = body(r, [.generateSphere(radius: 0.5)])
        if removedAfterRead { e.components.set(B3PhysicsMotionComponent()) }
        let w = world(r)
        frames(w, [e], 1)
        e.components.remove(B3PhysicsMotionComponent.self)
        e.b3.applyLinearImpulse([0, 100, 0], relativeTo: nil)
        // The system's path: nothing changed, so nothing is read back before the write.
        for _ in 0..<2 { w.update(changed: [], removed: [], deltaTime: 1.0 / 60) }
        #expect(!e.components.has(B3PhysicsMotionComponent.self))
    }

    // 13. A kinematic body follows the velocity of its motion component.
    @Test func kinematicFollowsItsVelocity() {
        let r = root()
        let e = body(r, [.generateBox(size: [1, 1, 1])], mode: .kinematic)
        let w = world(r)
        frames(w, [e], 1)
        e.components.set(B3PhysicsMotionComponent(linearVelocity: [1, 0, 0]))
        frames(w, [e], 60)
        #expect(abs(e.position.x - 1) < 0.05, "x \(e.position.x)")
    }

    // 15. Explicit mass properties put the center of mass at zero when they say zero.
    @Test func explicitCenterAtOrigin() {
        let r = root()
        let e = body(r, [B3ShapeResource.generateSphere(radius: 0.5).offsetBy(translation: [1, 0, 0])])
        e.components[B3PhysicsBodyComponent.self]!.massProperties = .init(mass: 2)
        let w = world(r)
        frames(w, [e], 1)
        let center = b3Body_GetLocalCenter(w.record(for: e)!.bodyId).simd
        #expect(simd_length(center) < 1e-5, "center \(center)")
    }

    // 14. Torques convert two lengths: the same body in centimeters spins as in meters.
    @Test func torqueScalesWithLengthSquared() {
        func spin(rootScale: Float, unitsPerMeter: Float) -> Float {
            let r = root()
            r.scale = SIMD3(repeating: rootScale)
            r.components[B3PhysicsSimulationComponent.self]!.gravity = .zero
            let e = Entity()
            e.components.set(B3PhysicsBodyComponent(density: 1000 / pow(unitsPerMeter, 3)))
            e.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 0.1 * unitsPerMeter)]))
            r.addChild(e)
            let w = world(r)
            frames(w, [e], 1)
            e.b3.applyAngularImpulse([0, 1, 0], relativeTo: nil) // newton-meter-seconds, in world space
            frames(w, [e], 1)
            return e.b3.angularVelocity!.y
        }
        let meters = spin(rootScale: 1, unitsPerMeter: 1)
        let centimeters = spin(rootScale: 0.01, unitsPerMeter: 100)
        #expect(abs(centimeters / meters - 1) < 0.01, "meters \(meters) rad/s, centimeters \(centimeters) rad/s")
    }

    // 16. A world without a root is found by the entity API.
    @Test func rootlessWorldIsFound() {
        let holder = Entity()
        let e = Entity()
        e.components.set(B3PhysicsBodyComponent(density: 1000))
        e.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 0.5)]))
        holder.addChild(e)
        let w = B3World(root: nil)
        w.update(entities: [e], deltaTime: 1.0 / 60)
        #expect(e.b3.isSimulated)
        #expect(e.b3.mass != nil)
    }

    // 7. A forecast stays exact with a kinematic body that has just been moved.
    @Test func forecastHandlesKinematicBodies() throws {
        let r = root()
        let ground = body(r, [.generateBox(size: [20, 1, 20])], at: [0, -0.5, 0], mode: .static)
        let paddle = body(r, [.generateBox(size: [0.2, 1, 2])], at: [-1, 0.5, 0], mode: .kinematic)
        let box = body(r, [.generateBox(size: [0.5, 0.5, 0.5])], at: [0, 0.25, 0])
        let w = world(r)
        frames(w, [ground, paddle, box], 30)
        paddle.position.x = -0.6 // sweeps in on the next step, then holds
        w.update(entities: [ground, paddle, box], deltaTime: 0) // read the move, no step yet
        let forecast = try #require(w.forecast())
        forecast.step(60)
        frames(w, [ground, paddle, box], 60)
        #expect(try #require(forecast.transform(of: paddle)).translation == paddle.position)
        #expect(try #require(forecast.transform(of: box)).translation == box.position)
    }

    // 8. A joint anchored to a moving non-body entity follows it.
    @Test func jointFollowsMovingAnchor() throws {
        let r = root()
        let anchor = Entity()
        anchor.position = [0, 3, 0]
        r.addChild(anchor)
        let bob = body(r, [.generateSphere(radius: 0.2)], at: [0, 2, 0])
        try B3PhysicsSphericalJoint(pin0: anchor.pins.set(named: "a"), pin1: bob.pins.set(named: "b", position: [0, 1, 0])).addToSimulation()
        let w = world(r)
        frames(w, [bob], 60)
        // Carried 2 m over one second, as an app moves a handle; a teleport would slingshot the bob.
        for i in 1...60 {
            anchor.position = [2 * Float(i) / 60, 3, 0]
            frames(w, [bob], 1)
        }
        frames(w, [bob], 180)
        // Dragged along, the bob swings on its 1 m joint around the new anchor point.
        #expect(abs(simd_distance(bob.position, anchor.position) - 1) < 0.05, "bob \(bob.position)")
        #expect(abs(bob.position.x - 2) <= 1.05, "bob did not follow: \(bob.position)")
        #expect(simd_length(bob.b3.linearVelocity ?? .zero) < 10, "the move made the pendulum explode")
    }

    // 8. A joint on a body that is rescaled follows the scaled pin.
    @Test func jointFollowsScale() throws {
        let r = root()
        let arm = body(r, [.generateBox(size: [0.2, 1, 0.2])], at: [0, 2, 0], mode: .static)
        let bob = body(r, [.generateSphere(radius: 0.1)], at: [0, 1.5, 0])
        try B3PhysicsSphericalJoint(pin0: arm.pins.set(named: "tip", position: [0, -0.5, 0]), pin1: bob.pins.set(named: "b")).addToSimulation()
        let w = world(r)
        frames(w, [arm, bob], 30)
        arm.scale = [1, 2, 1] // the tip moves to y = 1
        frames(w, [arm, bob], 120)
        #expect(abs(bob.position.y - 1) < 0.05, "bob at \(bob.position.y)")
    }

    // 17. Removing a body inside a sensor still reports the end of the overlap.
    @Test func removalEndsTriggerOverlap() {
        let r = root()
        let sensor = Entity()
        sensor.components.set(B3PhysicsBodyComponent(mode: .static))
        sensor.components.set(B3CollisionComponent(shapes: [.generateBox(size: [4, 4, 4])], mode: .trigger))
        r.addChild(sensor)
        let visitor = body(r, [.generateSphere(radius: 0.2)])
        visitor.components[B3PhysicsBodyComponent.self]!.isAffectedByGravity = false
        let w = world(r)
        var began = 0, ended = 0
        let s1 = w.subscribe(to: B3CollisionEvents.Began.self, on: sensor) { _ in began += 1 }
        let s2 = w.subscribe(to: B3CollisionEvents.Ended.self, on: sensor) { _ in ended += 1 }
        frames(w, [sensor, visitor], 10)
        #expect(began == 1)
        visitor.removeFromParent()
        frames(w, [sensor], 10)
        #expect(ended == 1, "no end event after removal")
        s1.cancel(); s2.cancel()
    }

    // 13 (second pass). A kinematic body with a velocity from the start moves.
    @Test func kinematicInitialVelocity() {
        let r = root()
        let e = body(r, [.generateBox(size: [1, 1, 1])], mode: .kinematic)
        e.components.set(B3PhysicsMotionComponent(linearVelocity: [1, 0, 0]))
        let w = world(r)
        frames(w, [e], 60)
        #expect(abs(e.position.x - 1) < 0.05, "x \(e.position.x)")
    }

    // 1 (second pass). Moving a velocity-driven kinematic body by its entity takes the new pose.
    @Test func kinematicVelocityThenTarget() {
        let r = root()
        let e = body(r, [.generateBox(size: [1, 1, 1])], mode: .kinematic)
        let w = world(r)
        frames(w, [e], 1)
        e.b3.setVelocity(linear: [2, 0, 0], angular: .zero)
        frames(w, [e], 10)
        e.position = [0, 5, 0]
        frames(w, [e], 10)
        #expect(simd_distance(e.position, [0, 5, 0]) < 1e-3, "entity at \(e.position)")
        #expect(simd_distance(e.b3.pose!.translation, [0, 5, 0]) < 1e-3, "body at \(e.b3.pose!.translation)")
    }

    // 3, 4. A dynamic body under a dynamic body is written after it, through its new transform.
    @Test func nestedDynamicBodiesStayConsistent() {
        let r = root()
        let parent = body(r, [.generateBox(size: [1, 1, 1])], at: [0, 5, 0])
        parent.components[B3PhysicsBodyComponent.self]!.isAffectedByGravity = false
        parent.components.set(B3PhysicsMotionComponent(linearVelocity: [2, 0, 0], angularVelocity: [0, 1, 0]))
        let child = body(parent, [.generateSphere(radius: 0.2)], at: [0, 2, 0])
        child.components[B3CollisionComponent.self]!.filter = .init(group: .init(rawValue: 4), mask: .init(rawValue: 4))
        let w = world(r)
        var worst: Float = 0
        for _ in 0..<60 {
            frames(w, [parent, child], 1)
            worst = max(worst, simd_distance(child.b3.pose!.translation, child.position(relativeTo: r)))
        }
        #expect(worst < 1e-3, "child shown \(worst) m from its Box3D pose")
    }

    // 3 (second pass). After a reparent, writes still go parents first.
    @Test func reparentedBodiesStayConsistent() {
        let r = root()
        let a = body(r, [.generateBox(size: [1, 1, 1])], at: [0, 5, 0])
        let b = body(a, [.generateSphere(radius: 0.2)], at: [0, 2, 0])
        for e in [a, b] {
            e.components[B3PhysicsBodyComponent.self]!.isAffectedByGravity = false
            e.components[B3CollisionComponent.self]!.filter = .init(group: .init(rawValue: 4), mask: .init(rawValue: 8))
        }
        let w = world(r)
        frames(w, [a, b], 5)
        // Swap: b to the root, then a under b.
        r.addChild(b, preservingWorldTransform: true)
        b.addChild(a, preservingWorldTransform: true)
        b.components.set(B3PhysicsMotionComponent(linearVelocity: [0, 0, 2], angularVelocity: [1, 0, 0]))
        var worst: Float = 0
        for _ in 0..<60 {
            frames(w, [a, b], 1)
            let d = simd_distance(a.b3.pose!.translation, a.position(relativeTo: r))
            worst = max(worst, d)
        }
        #expect(worst < 1e-3, "after the reparent, a was shown \(worst) m from its Box3D pose")
    }

    // 5 (second pass). Rescaling an anchor moves an offset pin; an anchor that becomes a body
    // attaches the joint to that body.
    @Test func anchorScaleAndPromotion() throws {
        let r = root()
        let anchor = Entity()
        anchor.position = [0, 3, 0]
        r.addChild(anchor)
        let bob = body(r, [.generateSphere(radius: 0.1)], at: [0.5, 2, 0])
        try B3PhysicsSphericalJoint(pin0: anchor.pins.set(named: "a", position: [0.5, 0, 0]),
                                    pin1: bob.pins.set(named: "b", position: [0, 1, 0])).addToSimulation()
        let w = world(r)
        frames(w, [bob], 120)
        anchor.scale = [2, 2, 2] // the pin moves to x = 1
        frames(w, [bob], 240)
        // It swings, but on its 1 m joint around the pin's new place.
        #expect(abs(simd_distance(bob.position, [1, 3, 0]) - 1) < 0.05, "bob at \(bob.position)")
        // The anchor becomes a dynamic body: the bob's weight now pulls on it.
        anchor.components.set(B3PhysicsBodyComponent(density: 1000))
        anchor.components.set(B3CollisionComponent(shapes: [.generateBox(size: [0.2, 0.2, 0.2])],
                                                   filter: .init(group: .init(rawValue: 4), mask: .init(rawValue: 8))))
        frames(w, [bob, anchor], 30)
        #expect(anchor.position.y < 2.9, "the jointed anchor did not fall: \(anchor.position.y)")
        #expect(w.anchors.isEmpty, "the kinematic stand-in was not retired")
    }

    // 9 (second pass). Under a non-uniform root, a torque equals the couple of forces producing it.
    @Test func torqueMatchesItsCoupleUnderNonUniformScale() {
        func spin(couple: Bool) -> SIMD3<Float> {
            let r = root()
            r.scale = [2, 1, 1]
            r.components[B3PhysicsSimulationComponent.self]!.gravity = .zero
            let e = body(r, [.generateSphere(radius: 0.5)])
            let w = world(r)
            frames(w, [e], 1)
            if couple {
                // Forces ±y at ±0.5 m along world x around the center: a torque of (0, 0, 1).
                e.b3.applyImpulse([0, 1, 0], at: [0.5, 0, 0], relativeTo: nil)
                e.b3.applyImpulse([0, -1, 0], at: [-0.5, 0, 0], relativeTo: nil)
            } else {
                e.b3.applyAngularImpulse([0, 0, 1], relativeTo: nil)
            }
            frames(w, [e], 1)
            return e.b3.angularVelocity!
        }
        let a = spin(couple: true), b = spin(couple: false)
        #expect(simd_distance(a, b) < 1e-3 * max(simd_length(a), 1), "couple \(a), torque \(b)")
    }

    // 10, 12 (second pass). Extreme geometry is rejected or handled, never a trap; one case per
    // test, so a trap in one does not hide the others.
    @Test func farMirroredBoxDoesNotTrap() {
        let r = root()
        let far = body(r, [B3ShapeResource.generateBox(size: [2, 1, 1])
            .offsetBy(rotation: simd_quatf(angle: .pi / 4, axis: [0, 0, 1]), translation: [300_000, 0, 0])], mode: .static)
        far.scale = [1, 2, -1]
        let w = world(r)
        frames(w, [far], 1)
        #expect(shapeCount(w, far) == 1)
    }

    @Test func oversizedHeightFieldIsRejected() {
        let r = root()
        let huge = body(r, [.generateHeightField(heights: [0], countX: .max, countZ: 2)], mode: .static)
        let w = world(r)
        frames(w, [huge], 1)
        #expect(shapeCount(w, huge) == 0)
    }

    @Test func nonFiniteMeshIsRejected() {
        let r = root()
        let nan = body(r, [.generateStaticMesh(positions: [.zero, [.nan, 0, 0], [0, 0, 1]], faceIndices: [UInt32(0), 1, 2])], mode: .static)
        let w = world(r)
        frames(w, [nan], 1)
        #expect(shapeCount(w, nan) == 0)
    }

    // 1 (third pass). Moving A, sleeping dynamic B under it, awake dynamic C under B: putting B
    // back must not carry C away from the pose just written for it.
    @Test func sleepingMiddleBodyKeepsChildConsistent() {
        let r = root()
        let a = body(r, [.generateBox(size: [1, 1, 1])], at: [0, 5, 0], mode: .kinematic)
        a.components.set(B3PhysicsMotionComponent(linearVelocity: [2, 0, 0]))
        let b = body(a, [.generateSphere(radius: 0.2)], at: [0, 2, 0])
        let c = body(b, [.generateSphere(radius: 0.2)], at: [0, 2, 0])
        c.components[B3PhysicsBodyComponent.self]!.isSleepEnabled = false
        c.components.set(B3PhysicsMotionComponent(linearVelocity: [0, 0, 1]))
        for (i, e) in [a, b, c].enumerated() {
            e.components[B3PhysicsBodyComponent.self]!.isAffectedByGravity = false
            let bit = UInt32(1) << (i + 2)
            e.components[B3CollisionComponent.self]!.filter = .init(group: .init(rawValue: bit), mask: .init(rawValue: bit))
        }
        let w = world(r)
        frames(w, [a, b, c], 1)
        b3Body_SetAwake(w.record(for: b)!.bodyId, false)
        var worstB: Float = 0, worstC: Float = 0
        for _ in 0..<30 {
            frames(w, [a, b, c], 1)
            #expect(!b.b3.isAwake, "b woke up, so the test no longer covers a sleeping middle body")
            worstB = max(worstB, simd_distance(b.b3.pose!.translation, b.position(relativeTo: r)))
            worstC = max(worstC, simd_distance(c.b3.pose!.translation, c.position(relativeTo: r)))
        }
        #expect(worstB < 1e-3, "sleeping b shown \(worstB) m from its Box3D pose")
        #expect(worstC < 1e-3, "c shown \(worstC) m from its Box3D pose")
    }

    // 3 (third pass). Reparenting a plain holder above a body updates the body's nesting.
    @Test func ancestorReparentRefreshesNesting() {
        let r = root()
        let a = body(r, [.generateBox(size: [1, 1, 1])], at: [0, 5, 0], mode: .kinematic)
        let holder = Entity()
        r.addChild(holder)
        let c = body(holder, [.generateSphere(radius: 0.2)], at: [3, 0, 0], mode: .kinematic)
        let w = world(r)
        frames(w, [a, c], 1)
        #expect(w.nestedBodyCount == 0)
        a.addChild(holder, preservingWorldTransform: true)
        frames(w, [a, c], 1)
        #expect(w.nestedBodyCount == 1, "nesting stale after an ancestor reparent")
        #expect(w.record(for: c)!.depth == 2)
        r.addChild(holder, preservingWorldTransform: true)
        frames(w, [a, c], 1)
        #expect(w.nestedBodyCount == 0)
    }

    // 2 (third pass). A pin on a plain entity with skeletal poses is tracked every frame.
    @Test func skeletalPinOnPlainEntityIsTracked() throws {
        let r = root()
        let rig = Entity()
        rig.components.set(SkeletalPosesComponent(poses: []))
        rig.position = [0, 3, 0]
        r.addChild(rig)
        let bob = body(r, [.generateSphere(radius: 0.1)], at: [0, 2, 0])
        try B3PhysicsSphericalJoint(pin0: rig.pins.set(named: "a"),
                                    pin1: bob.pins.set(named: "b", position: [0, 1, 0])).addToSimulation()
        let w = world(r)
        frames(w, [bob], 2)
        #expect(w.jointSlots.first?.resolved?.tracksPins == true)
        // Without its skeleton, the pin stops being resolved every frame.
        rig.components.remove(SkeletalPosesComponent.self)
        frames(w, [bob], 2)
        #expect(w.jointSlots.first?.resolved?.tracksPins == false)
    }

    /// A rigged box: a skeleton "base" → "tip", the tip 1 m above, every vertex bound to the base.
    func riggedModel() throws -> ModelEntity {
        let skeleton = try #require(MeshResource.Skeleton(
            id: "rig", jointNames: ["base", "base/tip"],
            inverseBindPoseMatrices: [matrix_identity_float4x4, Transform(translation: [0, -1, 0]).matrix],
            restPoseTransforms: [Transform(), Transform(translation: [0, 1, 0])], parentIndices: [nil, 0]))
        var contents = MeshResource.generateBox(size: 0.1).contents
        contents.skeletons = MeshSkeletonCollection([skeleton])
        contents.models = MeshModelCollection(contents.models.map { model in
            var model = model
            model.parts = MeshPartCollection(model.parts.map { part in
                var part = part
                part.skeletonID = "rig"
                part.jointInfluences = .init(influences: MeshBuffer(Array(repeating: MeshJointInfluence(jointIndex: 0, weight: 1),
                                                                         count: part.positions.count)),
                                             influencesPerVertex: 1)
                return part
            })
            return model
        })
        return ModelEntity(mesh: try MeshResource.generate(from: contents), materials: [])
    }

    // 5 (third pass). A torque relative to a degenerate frame stays finite.
    @Test func torqueThroughZeroScaleIsFinite() {
        let r = root()
        r.components[B3PhysicsSimulationComponent.self]!.gravity = .zero
        let e = body(r, [.generateSphere(radius: 0.5)])
        let flat = Entity()
        flat.scale = [0, 1, 1]
        r.addChild(flat)
        let w = world(r)
        frames(w, [e], 1)
        e.b3.applyAngularImpulse([1, 0, 0], relativeTo: flat)
        frames(w, [e], 1)
        let spin = e.b3.angularVelocity!
        #expect(spin.x.isFinite && spin.y.isFinite && spin.z.isFinite, "spin \(spin)")
        #expect(spin.x > 0 && abs(spin.y) < 1e-6 && abs(spin.z) < 1e-6, "spin \(spin)")
    }

    /// Moving a shape's offset or changing the body type resolves the mass again; the center of
    /// mass keeps its velocities instead of gaining ω × Δcenter.
    @Test(arguments: [false, true]) func editsKeepTheCentersVelocity(changesMode: Bool) {
        let r = root()
        let e = body(r, [B3ShapeResource.generateBox(size: [0.5, 0.5, 0.5]).offsetBy(translation: [0, 2, 0])],
                     mode: changesMode ? .kinematic : .dynamic)
        var s = r.components[B3PhysicsSimulationComponent.self]!
        s.gravity = .zero
        r.components.set(s)
        e.components.set(B3PhysicsMotionComponent(linearVelocity: [1, 0, 0], angularVelocity: [0, 0, 3]))
        let w = world(r)
        w.update(entities: [e], deltaTime: 0)
        if changesMode {
            e.components[B3PhysicsBodyComponent.self]?.mode = .dynamic
        } else {
            e.components[B3CollisionComponent.self]?.shapes = [B3ShapeResource.generateBox(size: [0.5, 0.5, 0.5]).offsetBy(translation: [0, 3, 0])]
        }
        w.update(entities: [e], deltaTime: 0)
        let id = w.record(for: e)!.bodyId
        #expect(simd_distance(b3Body_GetLinearVelocity(id).simd, [1, 0, 0]) < 1e-4, "\(b3Body_GetLinearVelocity(id).simd)")
        #expect(simd_distance(b3Body_GetAngularVelocity(id).simd, [0, 0, 3]) < 1e-4)
    }

    /// Restoring velocities after an edit leaves the axes the edit locked at zero.
    @Test func editsKeepNewLocks() {
        let r = root()
        let e = body(r, [.generateSphere(radius: 0.5)], at: [0, 5, 0])
        e.components.set(B3PhysicsMotionComponent(linearVelocity: [3, 0, 0], angularVelocity: .zero))
        let w = world(r)
        w.update(entities: [e], deltaTime: 0)
        e.components[B3PhysicsBodyComponent.self]?.isTranslationLocked.x = true
        w.update(entities: [e], deltaTime: 0)
        #expect(b3Body_GetLinearVelocity(w.record(for: e)!.bodyId).x == 0)
    }

    /// Changing the belt speed keeps the cargo's contact: the old contact's Ended comes before the new one's Began.
    @Test func beltSpeedKeepsContacts() {
        let r = root()
        let belt = body(r, [.generateBox(size: [20, 0.5, 4])], mode: .static)
        let crate = body(r, [.generateBox(size: [1, 1, 1])], at: [0, 0.75, 0])
        let w = world(r)
        // As an app tracks it: a set of touching entities, or here a flag
        var touching = false
        let subs = [w.subscribe(to: B3CollisionEvents.Began.self, on: crate) { _ in touching = true },
                    w.subscribe(to: B3CollisionEvents.Ended.self, on: crate) { _ in touching = false }]
        frames(w, [belt, crate], 30)
        #expect(touching)
        belt.components[B3PhysicsBodyComponent.self]?.material.tangentVelocity = [2, 0, 0]
        frames(w, [belt, crate], 30)
        #expect(touching, "the crate still rests on the belt")
        #expect(crate.position.x > 0.3)
        _ = subs
    }

    /// A belt started under sleeping cargo carries it.
    @Test func startingABeltWakesItsCargo() {
        let r = root()
        let belt = body(r, [.generateBox(size: [20, 0.5, 4])], mode: .static)
        let crate = body(r, [.generateBox(size: [1, 1, 1])], at: [0, 0.75, 0])
        let w = world(r)
        frames(w, [belt, crate], 240)
        #expect(!crate.b3.isAwake, "cargo should have slept")
        belt.components[B3PhysicsBodyComponent.self]?.material.tangentVelocity = [2, 0, 0]
        frames(w, [belt, crate], 60)
        #expect(crate.position.x > 1, "cargo at \(crate.position)")
    }

    /// Wind goes through WillSimulate in the scene, so its push does not depend on the frame rate.
    @Test func windIsFrameRateIndependent() {
        func tip(fps: Double) -> SIMD3<Float> {
            let runner = DemoRunner(DemoScene.named("Wind")!) { $0.maxStepsPerFrame = 8 }
            runner.advance(frames: Int(2 * fps), deltaTime: 1 / fps)
            return runner.root.children[0].children.first { $0.name == "link9" }!.position
        }
        let base = tip(fps: 60)
        for fps in [15.0, 120] { #expect(simd_distance(tip(fps: fps), base) < 0.05, "\(fps) fps tip \(tip(fps: fps)), 60 fps \(base)") }
    }

    /// A motion component's velocities are the center of mass's, at creation as on later edits:
    /// Box3D used to add ω × offset when resolving the mass of a newly created body.
    @Test func creationVelocityIsTheCenterOfMasses() {
        let r = root()
        let e = body(r, [B3ShapeResource.generateBox(size: [0.5, 0.5, 0.5]).offsetBy(translation: [0, 2, 0])])
        e.components.set(B3PhysicsMotionComponent(linearVelocity: [1, 0, 0], angularVelocity: [0, 0, 3]))
        let w = world(r)
        w.update(entities: [e], deltaTime: 0)
        let id = w.record(for: e)!.bodyId
        #expect(simd_distance(b3Body_GetLinearVelocity(id).simd, [1, 0, 0]) < 1e-4, "\(b3Body_GetLinearVelocity(id).simd)")
        #expect(simd_distance(b3Body_GetAngularVelocity(id).simd, [0, 0, 3]) < 1e-4)
    }
}

/// Findings in the RealityKit system path: nested bodies, motion components, scene lifetime.
@MainActor
@Suite(.serialized) struct ReviewSystemRegressionTests {
    // 3, 4. A static body nested under a dynamic one moves with it in Box3D too.
    @Test func nestedBodyFollowsItsParent() async throws {
        let h = try SystemHarness()
        h.ground()
        let cart = h.box(at: [0, 3, 0])
        let flag = Entity()
        flag.position = [0, 0.5, 0]
        flag.components.set(B3PhysicsBodyComponent(mode: .kinematic))
        flag.components.set(B3CollisionComponent(shapes: [.generateBox(size: [0.05, 0.4, 0.05])], filter: .init(group: .init(rawValue: 2), mask: .init(rawValue: 2))))
        cart.addChild(flag)
        try await h.frames(120)
        let world = try #require(h.world)
        let flagPose = try #require(flag.b3.pose)
        let shown = flag.position(relativeTo: h.root)
        #expect(simd_distance(flagPose.translation, shown) < 0.05, "Box3D \(flagPose.translation), shown \(shown)")
        _ = world
    }

    // 6. Adding a motion component to a resting body is an edit.
    @Test func addedMotionComponentApplies() async throws {
        let h = try SystemHarness()
        h.ground()
        let b = h.box(at: [0, 0.1, 0])
        try await h.frames(120)
        #expect(!b.b3.isAwake) // asleep, so only the added component can wake it
        b.components.set(B3PhysicsMotionComponent(linearVelocity: [0, 3, 0]))
        try await h.frames(10)
        #expect(b.position.y > 0.3, "the added velocity was ignored: y \(b.position.y)")
    }

    // 2. Closing a scene releases its worlds.
    @Test func closedScenesReleaseTheirWorlds() async throws {
        var before = 0
        do {
            let h = try SystemHarness()
            h.ground()
            try await h.frames(2)
            before = B3WorldRegistry.shared.worlds.count
        }
        for _ in 0..<3 {
            let h = try SystemHarness()
            h.ground()
            try await h.frames(2)
        }
        #expect(B3WorldRegistry.shared.worlds.count <= before, "worlds grew from \(before) to \(B3WorldRegistry.shared.worlds.count)")
    }

    // 2 (second pass). A root moved to another scene keeps its bodies, whichever scene updates first.
    @Test func rootMovesBetweenScenes() async throws {
        let a = try SystemHarness(), b = try SystemHarness()
        a.ground()
        let ball = a.box(at: [0, 0.1, 0])
        try await a.frames(5)
        ball.b3.setVelocity(linear: [1, 0, 0], angular: .zero)
        let bodyID = try #require(a.world?.record(for: ball)).bodyId
        // Move the whole simulation root from a's scene to b's, then let a update first.
        a.renderer.entities.removeAll { $0 === a.root }
        b.renderer.entities.append(a.root)
        try await a.frames(1)
        try await b.frames(1)
        let record = try #require(RealityBox3D.world(for: ball)?.record(for: ball))
        #expect(record.bodyId == bodyID, "the body was destroyed and recreated")
        #expect((ball.b3.linearVelocity?.x ?? 0) > 0.5, "velocity lost: \(String(describing: ball.b3.linearVelocity))")
    }

    // Skeletal pins: a joint pinned to an animated skeletal joint follows it with no edit.
    // RealityKit resolves skeletal pins only for entities in a running scene.
    @Test func skeletalPinFollowsItsJoint() async throws {
        let h = try SystemHarness()
        let rig = try ReviewRegressionTests().riggedModel()
        h.root.addChild(rig)
        try await h.frames(1)
        let pin = rig.pins.set(named: "tip", skeletalJointName: "base/tip")
        let bob = Entity()
        bob.components.set(B3PhysicsBodyComponent(density: 1000))
        bob.components.set(B3CollisionComponent(shapes: [.generateSphere(radius: 0.1)]))
        h.root.addChild(bob)
        try B3PhysicsSphericalJoint(pin0: pin, pin1: bob.pins.set(named: "b", position: [0, 1, 0])).addToSimulation()
        try await h.frames(30)
        // Animate the tip 2 m along x over 2 s, as a skeletal animation would, touching no component.
        for i in 1...120 {
            rig.jointTransforms[1] = Transform(translation: [2 * Float(i) / 120, 1, 0])
            try await h.frames(1)
        }
        let tip = try #require(pin.position(relativeTo: h.root))
        #expect(simd_distance(tip, [2, 1, 0]) < 1e-3, "RealityKit did not move the pin: \(tip)")
        try await h.frames(30)
        // The bob swings on its 1 m joint around the moved pin, not the pin's first place.
        #expect(abs(simd_distance(bob.position(relativeTo: h.root), [2, 1, 0]) - 1) < 0.05,
                "bob at \(bob.position(relativeTo: h.root))")
    }

    // 4 (third pass). A body removed just before its root moves to a scene that already runs
    // does not linger in the moved world.
    @Test(arguments: [false, true]) func removalBeforeSceneMoveIsKept(destinationFirst: Bool) async throws {
        let a = try SystemHarness(), b = try SystemHarness()
        a.ground(); b.ground()
        let kept = a.box(at: [0, 0.1, 0]), gone = a.box(at: [2, 0.1, 0])
        try await a.frames(3)
        try await b.frames(3)
        let world = try #require(a.world)
        #expect(world.record(for: gone) != nil)
        gone.removeFromParent()
        a.renderer.entities.removeAll { $0 === a.root }
        b.renderer.entities.append(a.root)
        // Whichever scene updates first, the moved world has dropped the body before it steps.
        if destinationFirst {
            try await b.frames(1)
            #expect(world.record(for: gone) == nil, "the removed body lingers in the moved world")
            try await a.frames(1)
        } else {
            try await a.frames(1)
            try await b.frames(1)
            #expect(world.record(for: gone) == nil, "the removed body lingers in the moved world")
        }
        #expect(world.record(for: kept) != nil)
    }
}


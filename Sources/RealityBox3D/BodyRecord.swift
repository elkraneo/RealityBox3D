import CBox3D
import RealityKit
import simd

/// Bridges one entity to one Box3D body.
///
/// Component edits are applied in place through Box3D's setters. Only a change of geometry or of
/// the entity's scale replaces the shapes, and the body itself is never recreated, so contacts,
/// sleep state and velocity survive edits.
@MainActor
final class BodyRecord {
    unowned let world: B3World
    /// Strong: weak references put Entity's reference counting on its slow path. The world drops
    /// the record, and so the entity, on the first update after the entity leaves the query.
    let entity: Entity
    let bodyId: b3BodyId
    private(set) var body: B3PhysicsBodyComponent
    private(set) var collision: B3CollisionComponent
    private(set) var scale: SIMD3<Float>
    private(set) var shapeIds: [b3ShapeId] = []
    var meshes: [UnsafeMutablePointer<b3MeshData>] = []
    /// Baked compounds, referenced by their shapes until those are destroyed.
    var compounds: [UnsafeMutablePointer<b3CompoundData>] = []
    private var heightFields: [UnsafeMutablePointer<b3HeightFieldData>] = []
    private let key: UnsafeMutableRawPointer?

    /// Pose we last wrote to, or accepted from, the entity. A different entity pose means user intent.
    private var lastPose: Pose
    private var lastMotion: B3PhysicsMotionComponent?
    /// Poses bracketing the last fixed step, for interpolation.
    var previous: b3Transform
    var current: b3Transform
    /// Fixed step in which Box3D last reported this body as moved.
    var lastMovedStep = -1
    /// Frame in which the world last found this entity in its input.
    var seenFrame = -1
    /// Depth below the simulation root, and whether a body sits above this one.
    private(set) var depth: Int
    private(set) var isNested: Bool
    /// The parent the hierarchy metadata was computed under.
    /// Kinematic target, consumed during the fixed steps.
    private var kinematicTarget: Pose?
    /// A target the next fixed step will steer a kinematic body to, for forecasts.
    var pendingKinematicTarget: Pose? { kinematicTarget }

    /// A kinematic body moving at a velocity the app set, as RealityKit's kinematic bodies follow
    /// their motion component, rather than toward its entity's pose.
    private(set) var isVelocityDriven = false

    init(world: B3World, entity: Entity, body: B3PhysicsBodyComponent, collision: B3CollisionComponent,
         pose: Pose, motion: B3PhysicsMotionComponent?) {
        self.world = world
        self.entity = entity
        self.body = body
        self.collision = collision
        self.scale = pose.scale
        self.lastPose = pose
        self.lastMotion = motion
        self.key = UnsafeMutableRawPointer(bitPattern: UInt(entity.id))
        (self.depth, self.isNested) = Self.hierarchy(of: entity, root: world.root)

        var def = b3DefaultBodyDef()
        def.type = body.mode.b3
        def.position = b3Vec3(pose.position)
        def.rotation = b3Quat(pose.orientation)
        if let motion {
            def.linearVelocity = b3Vec3(motion.linearVelocity)
            def.angularVelocity = b3Vec3(motion.angularVelocity)
        }
        def.linearDamping = body.linearDamping
        def.angularDamping = body.angularDamping
        def.gravityScale = body.effectiveGravityScale
        def.enableSleep = body.isSleepEnabled
        def.isBullet = body.effectiveBullet
        def.allowFastRotation = body.allowsFastRotation
        def.motionLocks = body.motionLocks
        def.userData = key
        bodyId = b3CreateBody(world.id, &def)
        current = b3Body_GetTransform(bodyId)
        previous = current

        createShapes()
        world.oneWayShapesChanged(self, removing: [])
        updateMass()
        if body.mode == .kinematic, let motion {
            isVelocityDriven = motion.linearVelocity != .zero || motion.angularVelocity != .zero
        }
    }

    /// Depth below the root, and whether a body sits above this one.
    private static func hierarchy(of entity: Entity, root: Entity?) -> (Int, Bool) {
        var depth = 0, nested = false
        var ancestor = entity.parent
        while let a = ancestor, a !== root {
            depth += 1
            if a.components.has(B3PhysicsBodyComponent.self) { nested = true }
            ancestor = a.parent
        }
        return (depth, nested)
    }

    /// Recomputes the hierarchy metadata, which a reparent of this entity or of any ancestor, or
    /// an ancestor gaining or losing a body, can change. Returns the change in nesting.
    func refreshHierarchy(root: Entity?) -> Int {
        // A body straight under the root, as in flat scenes, stays there unless it is reparented.
        // `root` comes from the caller: reading the world's weak reference per body is slow.
        guard depth != 0 || entity.parent !== root else { return 0 }
        let was = isNested
        (depth, isNested) = Self.hierarchy(of: entity, root: root)
        return (isNested ? 1 : 0) - (was ? 1 : 0)
    }

    /// `retiring` keeps the shapes as tombstones for the next step's end events; a world being
    /// torn down has no next step.
    func destroy(retiring: Bool = true) {
        if retiring { world.retire(shapeIds, of: entity) }
        if b3Body_IsValid(bodyId) { b3DestroyBody(bodyId) }
        freeMeshes()
    }

    // MARK: Component changes

    /// Applies edited components in place.
    func apply(body new: B3PhysicsBodyComponent, collision newCollision: B3CollisionComponent, scale newScale: SIMD3<Float>) {
        let old = body
        body = new

        // Compounds, height fields and meshes belong to static bodies: Box3D refuses to change the
        // type of a body holding the first two, locking its world, and cannot push meshes. They go
        // before the type changes and are rebuilt for the new one.
        let rebuildsForMode = new.mode != old.mode && (!compounds.isEmpty || !heightFields.isEmpty || !meshes.isEmpty
            || newCollision.bakesCompound || newCollision.shapes.contains(where: \.isStaticOnly))
        let oldShapeIds = shapeIds
        if rebuildsForMode { destroyShapes() }

        if new.mode != old.mode {
            b3Body_SetType(bodyId, new.mode.b3)
            // A body becoming kinematic keeps a velocity its motion component gives it.
            isVelocityDriven = new.mode == .kinematic
                && (lastMotion.map { $0.linearVelocity != .zero || $0.angularVelocity != .zero } ?? false)
            kinematicTarget = nil
            world.bodyModeChanged(self)
        }
        if new.linearDamping != old.linearDamping { b3Body_SetLinearDamping(bodyId, new.linearDamping) }
        if new.angularDamping != old.angularDamping { b3Body_SetAngularDamping(bodyId, new.angularDamping) }
        if new.effectiveGravityScale != old.effectiveGravityScale { b3Body_SetGravityScale(bodyId, new.effectiveGravityScale) }
        if new.isSleepEnabled != old.isSleepEnabled { b3Body_EnableSleep(bodyId, new.isSleepEnabled) }
        if new.effectiveBullet != old.effectiveBullet { b3Body_SetBullet(bodyId, new.effectiveBullet) }
        if new.allowsFastRotation != old.allowsFastRotation { b3Body_AllowFastRotation(bodyId, new.allowsFastRotation) }
        if new.motionLocks != old.motionLocks { b3Body_SetMotionLocks(bodyId, new.motionLocks) }

        let geometryChanged = newCollision.shapes != collision.shapes || !newScale.isClose(to: scale)
            || newCollision.mode != collision.mode // sensor-ness is fixed at shape creation
            || newCollision.oneWayDirection != collision.oneWayDirection // as is pre-solve
            // Shapes with their own surfaces keep them: rebuild rather than overwrite them all.
            || (new.material != old.material && newCollision.shapes.contains { $0.material != nil || $0.triangleMaterials != nil })
            || newCollision.bakesCompound != collision.bakesCompound
            || rebuildsForMode
            || (new.material != old.material && newCollision.bakesCompound) // a compound's materials are baked
        let oldCollision = collision
        collision = newCollision
        scale = newScale

        if geometryChanged {
            defer { world.oneWayShapesChanged(self, removing: oldShapeIds) }
            destroyShapes()
            createShapes()
            updateMass()
        } else {
            if new.material != old.material {
                for s in shapeIds { b3Shape_SetSurfaceMaterial(s, new.material.b3) }
            }
            if newCollision.filter != oldCollision.filter || newCollision.groupIndex != oldCollision.groupIndex {
                for s in shapeIds { b3Shape_SetFilter(s, newCollision.b3Filter, true) }
            }
            if newCollision.reportsContacts != oldCollision.reportsContacts {
                for s in shapeIds {
                    b3Shape_EnableContactEvents(s, newCollision.reportsContacts)
                    b3Shape_EnableHitEvents(s, newCollision.reportsContacts)
                }
            }
            if new.massSource != old.massSource || new.massProperties != old.massProperties || new.mode != old.mode {
                if case .density(let d) = new.massSource {
                    for s in shapeIds { b3Shape_SetDensity(s, d, false) }
                } else if case .density = old.massSource {
                    for s in shapeIds { b3Shape_SetDensity(s, 1, false) }
                }
                updateMass()
            }
        }
        b3Body_SetAwake(bodyId, true)
    }

    private func createShapes() {
        var def = b3DefaultShapeDef()
        def.userData = key
        def.density = if case .density(let d) = body.massSource { d } else { 1 }
        def.baseMaterial = body.material.b3
        def.filter = collision.b3Filter
        def.isSensor = collision.mode == .trigger
        def.enableSensorEvents = true
        def.enableContactEvents = collision.reportsContacts
        def.enableHitEvents = collision.reportsContacts
        // One-way shapes ask the world's pre-solve callback about each of their contacts.
        def.enablePreSolveEvents = collision.oneWayDirection != nil
        def.updateBodyMass = false
        var shapes = collision.shapes
        if collision.bakesCompound {
            if collision.oneWayDirection != nil {
                // Box3D's compound children report one-way contacts with a pair order that does not
                // match the normal, so one-way bodies keep their shapes separate.
                log.warning("bakesCompound with oneWayDirection: the shapes stay separate so one-way filtering works.")
            } else if collision.mode == .trigger {
                log.warning("bakesCompound on a trigger. Box3D compounds cannot be sensors; the shapes stay separate.")
            } else if body.mode == .static {
                shapes = bakeCompound(def: &def)
            } else {
                log.warning("bakesCompound on a non-static body. Box3D bakes compounds for static bodies only.")
            }
        }
        for shape in shapes {
            def.baseMaterial = (shape.material ?? body.material).b3
            if let id = addShape(shape, def: &def, scale: scale) { shapeIds.append(id) }
        }
    }

    /// Whether the body takes part in the simulation: off while its entity is disabled, as
    /// RealityKit drops disabled entities from physics. It keeps its state while off.
    private(set) var isEnabled = true
    /// Velocities at the moment the body was disabled: Box3D drops them, the body resumes with them.
    private var velocitiesWhileDisabled: (linear: b3Vec3, angular: b3Vec3)?
    func setEnabled(_ enabled: Bool) {
        guard enabled != isEnabled else { return }
        isEnabled = enabled
        if enabled {
            b3Body_Enable(bodyId)
            if let v = velocitiesWhileDisabled, body.mode != .static {
                b3Body_SetLinearVelocity(bodyId, v.linear)
                b3Body_SetAngularVelocity(bodyId, v.angular)
            }
            velocitiesWhileDisabled = nil
        } else {
            velocitiesWhileDisabled = (b3Body_GetLinearVelocity(bodyId), b3Body_GetAngularVelocity(bodyId))
            b3Body_Disable(bodyId)
        }
    }

    /// Sets the body's velocities and wakes it. A disabled body has no velocity in Box3D, so it
    /// takes them when it is enabled again.
    private func writeVelocity(_ linear: b3Vec3, _ angular: b3Vec3) {
        guard isEnabled else {
            velocitiesWhileDisabled = (linear, angular)
            return
        }
        b3Body_SetLinearVelocity(bodyId, linear)
        b3Body_SetAngularVelocity(bodyId, angular)
        b3Body_SetAwake(bodyId, true)
    }

    /// Shapes are added with updateBodyMass = false, so mass is resolved here for every body type.
    /// Box3D asserts on a dirty mass even for kinematic bodies.
    private func updateMass() {
        b3Body_ApplyMassFromShapes(bodyId)
        if body.mode == .dynamic, case .explicit = body.massSource { applyExplicitMass(body.massProperties) }
    }

    private func addShape(_ shape: B3ShapeResource, def: inout b3ShapeDef, scale s: SIMD3<Float>) -> b3ShapeId? {
        let r = shape.rotation
        let t = shape.translation * s
        let isIdentity = abs(r.real) > 1 - 1e-6

        switch shape.geometry {
        case .box(let size):
            // Rotating then scaling equals scaling the box's extents only without rotation, or with a
            // positive uniform scale. Anything else, mirrors included, goes through its corners.
            let positiveUniform = s.x > 0 && abs(s.x - s.y) < 1e-6 * s.x && abs(s.x - s.z) < 1e-6 * s.x
            if !isIdentity && !positiveUniform {
                // A rotated box under non-uniform scale is no longer a box: hull its eight corners,
                // rotated first and scaled after, as the entity's transform applies them.
                let e = size / 2
                var corners: [SIMD3<Float>] = []
                for x: Float in [-1, 1] { for y: Float in [-1, 1] { for z: Float in [-1, 1] { corners.append(e * [x, y, z]) } } }
                return addShape(B3ShapeResource(geometry: .convex(points: corners)).offsetBy(rotation: r, translation: shape.translation),
                                def: &def, scale: s)
            }
            let h = abs(size * 0.5 * s)
            var box = isIdentity
                ? b3MakeOffsetBoxHull(h.x, h.y, h.z, b3Vec3(t))
                : b3MakeTransformedBoxHull(h.x, h.y, h.z, b3Transform(translation: t, rotation: r))
            // `base` is the first field and indexes into the trailing arrays, so pass the whole box's address.
            return withUnsafePointer(to: &box) { p in
                b3CreateHullShape(bodyId, &def, UnsafeRawPointer(p).assumingMemoryBound(to: b3HullData.self))
            }

        case .sphere(let radius):
            var sphere = b3Sphere(center: b3Vec3(t), radius: radius * abs(s).max())
            return b3CreateSphereShape(bodyId, &def, &sphere)

        case .capsule(let height, let radius):
            let half = max(height * 0.5 - radius, 0)
            let a = r.act([0, -half, 0]) * s + t
            let b = r.act([0, half, 0]) * s + t
            var capsule = b3Capsule(center1: b3Vec3(a), center2: b3Vec3(b), radius: radius * max(abs(s.x), abs(s.z)))
            return b3CreateCapsuleShape(bodyId, &def, &capsule)

        case .convex(let points):
            var pts = Self.weld(points.map { r.act($0) * s + t }).map(b3Vec3.init)
            // Box3D caps hulls at 128 vertices but also at 128 faces. A rounded hull has about
            // 2V - 4 faces, so 64 vertices is the practical ceiling. Simplify further on failure.
            var hull: UnsafeMutablePointer<b3HullData>?
            for maxVertices: Int32 in [64, 48, 32, 16] where hull == nil {
                hull = b3CreateHull(&pts, Int32(pts.count), maxVertices)
            }
            guard let hull else {
                log.error("Box3D could not build a hull from \(points.count) points")
                return nil
            }
            defer { b3DestroyHull(hull) }
            return b3CreateHullShape(bodyId, &def, hull)

        case .heightField(let heights, let countX, let countZ, let fieldScale):
            let product = countX.multipliedReportingOverflow(by: countZ)
            guard countX >= 2, countZ >= 2, countX <= Int(Int32.max), countZ <= Int(Int32.max),
                  !product.overflow, heights.count == product.partialValue, heights.allSatisfy(\.isFinite),
                  fieldScale.x > 0, fieldScale.y > 0, fieldScale.z > 0, s.x > 0, s.y > 0, s.z > 0 else {
                log.error("Invalid height field: \(countX)×\(countZ) grid, \(heights.count) heights, scale \(fieldScale), entity scale \(s)")
                return nil
            }
            if !isIdentity || shape.translation != .zero {
                // Box3D's height fields have no local transform: an offset one becomes a static mesh.
                let grid = B3ShapeResource.heightFieldTriangles(heights: heights, countX: countX, countZ: countZ, scale: fieldScale)
                return addShape(B3ShapeResource(geometry: .staticMesh(positions: grid.positions, indices: grid.indices))
                    .offsetBy(rotation: r, translation: shape.translation), def: &def, scale: s)
            }
            if body.mode != .static {
                log.warning("Height field on a non-static body. Box3D only allows height fields on static bodies.")
                return nil
            }
            var h = heights
            let field: UnsafeMutablePointer<b3HeightFieldData>? = h.withUnsafeMutableBufferPointer { hp in
                var def = b3HeightFieldDef()
                def.heights = hp.baseAddress
                def.scale = b3Vec3(fieldScale * s)
                def.countX = Int32(countX)
                def.countZ = Int32(countZ)
                def.globalMinimumHeight = heights.min() ?? 0
                def.globalMaximumHeight = max(heights.max() ?? 0, (heights.min() ?? 0) + 1e-3)
                var materials = [UInt8](repeating: 0, count: max((countX - 1) * (countZ - 1), 0))
                return materials.withUnsafeMutableBufferPointer { mp in
                    def.materialIndices = mp.baseAddress
                    return b3CreateHeightField(&def)
                }
            }
            guard let field else {
                log.error("Box3D could not build a \(countX)×\(countZ) height field")
                return nil
            }
            heightFields.append(field)
            return b3CreateHeightFieldShape(bodyId, &def, field)

        case .staticMesh(let positions, let indices):
            guard positions.count >= 3, indices.count >= 3, indices.count % 3 == 0, indices.count / 3 <= Int(Int32.max),
                  positions.count <= Int(Int32.max), indices.allSatisfy({ Int($0) < positions.count }),
                  positions.allSatisfy({ $0.x.isFinite && $0.y.isFinite && $0.z.isFinite }) else {
                log.error("Invalid static mesh: \(positions.count) positions, \(indices.count) indices, or an index out of range")
                return nil
            }
            if body.mode != .static {
                log.warning("Static mesh shape on a non-static body. Box3D only collides meshes on static bodies.")
            }
            let surfaces = Self.validSurfaces(shape.triangleMaterials, triangles: indices.count / 3)
            guard let mesh = Self.makeMesh(positions.map { r.act($0) * s + t }, indices, surfaces) else {
                log.error("Box3D could not build a mesh from \(indices.count / 3) triangles")
                return nil
            }
            meshes.append(mesh)
            // Explosions push convex shapes only; Box3D asserts on a mesh that is not skipped.
            def.explosionScale = 0
            defer { def.explosionScale = 1 }
            guard let surfaces else { return b3CreateMeshShape(bodyId, &def, mesh, b3Vec3(x: 1, y: 1, z: 1)) }
            var palette = surfaces.materials.map(\.b3)
            return palette.withUnsafeMutableBufferPointer { pp in
                def.materials = pp.baseAddress
                def.materialCount = Int32(pp.count)
                defer { def.materials = nil; def.materialCount = 0 }
                return b3CreateMeshShape(bodyId, &def, mesh, b3Vec3(x: 1, y: 1, z: 1))
            }
        }
    }

    /// Per-triangle surfaces that fit their mesh: one index per triangle, into at most 256 materials.
    static func validSurfaces(_ tm: B3TriangleMaterials?, triangles: Int) -> B3TriangleMaterials? {
        guard let tm else { return nil }
        guard tm.indices.count == triangles, !tm.materials.isEmpty, tm.materials.count <= 256,
              tm.indices.allSatisfy({ Int($0) < tm.materials.count }) else {
            log.error("Invalid triangle materials: \(tm.indices.count) indices for \(triangles) triangles, \(tm.materials.count) materials")
            return nil
        }
        return tm
    }

    /// Box3D mesh data from positions already in body space.
    static func makeMesh(_ positions: [SIMD3<Float>], _ indices: [UInt32], _ surfaces: B3TriangleMaterials?) -> UnsafeMutablePointer<b3MeshData>? {
        var verts = positions.map { b3Vec3($0) }
        var idx = indices.map { Int32(bitPattern: $0) }
        var materialIndices = surfaces?.indices ?? []
        return verts.withUnsafeMutableBufferPointer { vp in
            idx.withUnsafeMutableBufferPointer { ip in
                materialIndices.withUnsafeMutableBufferPointer { mp in
                    var md = b3MeshDef()
                    md.vertices = vp.baseAddress
                    md.indices = ip.baseAddress
                    md.materialIndices = surfaces == nil ? nil : mp.baseAddress
                    md.vertexCount = Int32(vp.count)
                    md.triangleCount = Int32(ip.count / 3)
                    return b3CreateMesh(&md, nil, 0)
                }
            }
        }
    }

    /// Destroys the body's shapes and the geometry they hold. Idempotent.
    private func destroyShapes() {
        world.retire(shapeIds, of: entity)
        for s in shapeIds where b3Shape_IsValid(s) { b3DestroyShape(s, false) }
        shapeIds.removeAll()
        freeMeshes()
    }

    private func freeMeshes() {
        for m in meshes { b3DestroyMesh(m) }
        meshes.removeAll()
        for f in heightFields { b3DestroyHeightField(f) }
        heightFields.removeAll()
        for c in compounds { b3DestroyCompound(c) }
        compounds.removeAll()
    }

    /// Removes duplicate points. Render meshes repeat vertices along seams and poles,
    /// which Box3D's hull builder rejects.
    nonisolated static func weld(_ points: [SIMD3<Float>], tolerance: Float = 1e-4) -> [SIMD3<Float>] {
        var seen = Set<SIMD3<Int64>>()
        var out: [SIMD3<Float>] = []
        out.reserveCapacity(points.count)
        for p in points {
            let q = (p / tolerance).rounded(.toNearestOrEven)
            // Quantize only what fits; anything else is kept as is rather than trapping.
            guard let x = Int64(exactly: Double(q.x)), let y = Int64(exactly: Double(q.y)), let z = Int64(exactly: Double(q.z)) else {
                if p.x.isFinite && p.y.isFinite && p.z.isFinite { out.append(p) }
                continue
            }
            if seen.insert(SIMD3(x, y, z)).inserted { out.append(p) }
        }
        return out
    }

    private func applyExplicitMass(_ m: B3PhysicsMassProperties) {
        var data = b3Body_GetMassData(bodyId)
        guard m.mass > 0 else { return }
        if m.inertia != .zero {
            // Principal moments about the given center-of-mass frame, rotated into body space.
            let R = simd_float3x3(m.centerOfMass.orientation)
            let I = R * simd_float3x3(diagonal: m.inertia) * R.transpose
            data.inertia = b3Matrix3(cx: b3Vec3(I.columns.0), cy: b3Vec3(I.columns.1), cz: b3Vec3(I.columns.2))
        } else if data.mass > 0 {
            // Keep the shape-derived distribution, rescaled to the requested mass. If the center
            // of mass moves away from the shapes' centroid, the same matter now rotates about
            // another point: add the parallel-axis term, as Box3D's samples do with b3Steiner.
            let k = m.mass / data.mass
            var I = simd_float3x3(data.inertia.cx.simd * k, data.inertia.cy.simd * k, data.inertia.cz.simd * k)
            do {
                // RealityKit's explicit mass properties put the center where they say, zero included.
                let d = m.centerOfMass.position - data.center.simd
                I += m.mass * (simd_float3x3(diagonal: SIMD3(repeating: simd_dot(d, d))) - simd_float3x3(rows: [d * d.x, d * d.y, d * d.z]))
            }
            data.inertia = b3Matrix3(cx: b3Vec3(I.columns.0), cy: b3Vec3(I.columns.1), cz: b3Vec3(I.columns.2))
        }
        data.mass = m.mass
        data.center = b3Vec3(m.centerOfMass.position)
        b3Body_SetMassData(bodyId, data)
    }

    func currentMotion() -> B3PhysicsMotionComponent? {
        guard b3Body_IsValid(bodyId) else { return nil }
        // A disabled body's velocities are the ones it keeps for re-enabling; Box3D reads zero.
        if let v = velocitiesWhileDisabled { return .init(linearVelocity: v.linear.simd, angularVelocity: v.angular.simd) }
        return .init(linearVelocity: b3Body_GetLinearVelocity(bodyId).simd,
                     angularVelocity: b3Body_GetAngularVelocity(bodyId).simd)
    }

    // MARK: Sync

    func syncInbound(pose: Pose, motion: B3PhysicsMotionComponent?) {
        let moved = !pose.isClose(to: lastPose)
        switch body.mode {
        case .kinematic where isEnabled:
            if moved {
                kinematicTarget = pose
                isVelocityDriven = false
            }
        default:
            // A disabled kinematic body cannot travel to a target: it is placed, like the others.
            if moved {
                if body.mode == .kinematic { kinematicTarget = nil; isVelocityDriven = false }
                // Teleport: the app moved a static or dynamic entity directly.
                b3Body_SetTransform(bodyId, b3Vec3(pose.position), b3Quat(pose.orientation))
                // Box3D does not wake a body it moves; a teleported dynamic body must fall from there.
                if body.mode == .dynamic { b3Body_SetAwake(bodyId, true) }
                current = b3Body_GetTransform(bodyId)
                previous = current
            }
        }
        if body.mode != .static, let motion, motion != lastMotion {
            writeVelocity(b3Vec3(motion.linearVelocity), b3Vec3(motion.angularVelocity))
            if body.mode == .kinematic, !moved {
                isVelocityDriven = motion.linearVelocity != .zero || motion.angularVelocity != .zero
                // The newer velocity wins over a move still waiting, as with `setVelocity`.
                kinematicTarget = nil
            }
        }
        lastPose = pose
        lastMotion = motion
    }

    /// Kinematic bodies only: aim at the entity's pose over one fixed step, then hold.
    func driveKinematic(fixedDt: Float) {
        guard isEnabled else { return }
        if let target = kinematicTarget {
            b3Body_SetTargetTransform(bodyId, b3Transform(translation: target.position, rotation: target.orientation),
                                      fixedDt, true)
            kinematicTarget = nil
        } else if !isVelocityDriven {
            b3Body_SetLinearVelocity(bodyId, b3Vec3(x: 0, y: 0, z: 0))
            b3Body_SetAngularVelocity(bodyId, b3Vec3(x: 0, y: 0, z: 0))
        }
    }

    /// A kinematic body given a velocity directly keeps it until its entity is moved again.
    func setVelocity(linear: SIMD3<Float>, angular: SIMD3<Float>) {
        writeVelocity(b3Vec3(linear), b3Vec3(angular))
        if body.mode == .kinematic {
            isVelocityDriven = linear != .zero || angular != .zero
            kinematicTarget = nil
        }
        publishMotion()
    }

    /// After a change made through `entity.b3`, writes the new velocities to the entity's motion
    /// component, if it has one, so the next sync does not take the stale component for an edit.
    private func publishMotion() {
        // The entity's component now, not the last one seen: one added or removed since counts.
        guard entity.components.has(B3PhysicsMotionComponent.self), let m = currentMotion() else {
            lastMotion = nil
            return
        }
        B3World.whileWritingEntities { entity.components.set(m) }
        lastMotion = m
    }

    /// Called for each Box3D move event.
    func moved(to transform: b3Transform, step: Int) {
        // Several Box3D steps can make one fixed step: interpolate from where it began.
        if lastMovedStep != step { previous = current }
        current = transform
        lastMovedStep = step
    }

    /// Writes the interpolated pose and the velocity to the entity. Dynamic and velocity-driven
    /// kinematic bodies.
    func syncOutbound(alpha: Float, lastStep: Int, frame: ParentFrame, root: Entity?) {
        // A body that did not move in the latest step is at rest there; stop interpolating.
        if lastMovedStep < lastStep { previous = current }

        let p = simd_mix(previous.p.simd, current.p.simd, SIMD3(repeating: alpha))
        let q = alpha >= 1 ? current.q.simd : simd_slerp(previous.q.simd, current.q.simd, alpha)
        lastPose = frame.write(position: p, orientation: q, scale: lastPose.scale, to: entity, root: root)

        // Velocities only for entities that carry a motion component; syncInbound tracks that.
        if lastMotion != nil, let m = currentMotion() {
            entity.components.set(m)
            lastMotion = m
        }
    }

    // MARK: Forces

    func addForce(_ f: SIMD3<Float>, at point: SIMD3<Float>?) {
        if let point { b3Body_ApplyForce(bodyId, b3Vec3(f), b3Vec3(point), true) }
        else { b3Body_ApplyForceToCenter(bodyId, b3Vec3(f), true) }
    }
    func addTorque(_ t: SIMD3<Float>) { b3Body_ApplyTorque(bodyId, b3Vec3(t), true) }
    func applyLinearImpulse(_ i: SIMD3<Float>, at point: SIMD3<Float>?) {
        if let point { b3Body_ApplyLinearImpulse(bodyId, b3Vec3(i), b3Vec3(point), true) }
        else { b3Body_ApplyLinearImpulseToCenter(bodyId, b3Vec3(i), true) }
        publishMotion()
    }
    func applyAngularImpulse(_ i: SIMD3<Float>) {
        b3Body_ApplyAngularImpulse(bodyId, b3Vec3(i), true)
        publishMotion()
    }
}

// MARK: - Component → Box3D

extension PhysicsBodyMode {
    var b3: b3BodyType {
        switch self {
        case .static: b3_staticBody
        case .kinematic: b3_kinematicBody
        default: b3_dynamicBody
        }
    }
}

extension B3PhysicsBodyComponent {
    var effectiveGravityScale: Float { isAffectedByGravity ? gravityScale : 0 }
    var effectiveBullet: Bool { isBullet || (isContinuousCollisionDetectionEnabled && mode == .dynamic) }
    var motionLocks: b3MotionLocks {
        b3MotionLocks(linearX: isTranslationLocked.x, linearY: isTranslationLocked.y, linearZ: isTranslationLocked.z,
                      angularX: isRotationLocked.x, angularY: isRotationLocked.y, angularZ: isRotationLocked.z)
    }
}

extension B3CollisionComponent {
    var b3Filter: b3Filter {
        var f = filter.b3
        f.groupIndex = Int32(groupIndex)
        return f
    }
}

extension B3PhysicsMaterial {
    var b3: b3SurfaceMaterial {
        var m = b3DefaultSurfaceMaterial()
        m.friction = dynamicFriction
        m.restitution = restitution
        m.rollingResistance = rollingResistance
        return m
    }
}

extension CollisionFilter {
    var b3: b3Filter {
        var f = b3DefaultFilter()
        f.categoryBits = UInt64(group.rawValue)
        f.maskBits = UInt64(mask.rawValue)
        return f
    }
}

extension b3MotionLocks: @retroactive Equatable {
    public static func == (a: Self, b: Self) -> Bool {
        a.linearX == b.linearX && a.linearY == b.linearY && a.linearZ == b.linearZ
            && a.angularX == b.angularX && a.angularY == b.angularY && a.angularZ == b.angularZ
    }
}

// MARK: - Baked compounds

extension BodyRecord {
    /// Bakes the body's shapes into one Box3D compound shape. Returns the shapes a compound cannot
    /// hold, height fields and anything malformed, to add one by one.
    fileprivate func bakeCompound(def: inout b3ShapeDef) -> [B3ShapeResource] {
        var spheres: [b3CompoundSphereDef] = [], capsules: [b3CompoundCapsuleDef] = []
        var hulls: [b3CompoundHullDef] = [], meshParts: [(mesh: UnsafeMutablePointer<b3MeshData>, palette: [b3SurfaceMaterial])] = []
        var hullData: [UnsafeMutablePointer<b3HullData>] = []
        var rest: [B3ShapeResource] = []
        defer {
            for h in hullData { b3DestroyHull(h) }
            for m in meshParts { b3DestroyMesh(m.mesh) }
        }
        let s = scale
        func hull(_ points: [SIMD3<Float>]) -> UnsafeMutablePointer<b3HullData>? {
            var pts = Self.weld(points).map(b3Vec3.init)
            for maxVertices: Int32 in [64, 48, 32, 16] {
                if let h = b3CreateHull(&pts, Int32(pts.count), maxVertices) { return h }
            }
            return nil
        }
        for shape in collision.shapes {
            let r = shape.rotation, t = shape.translation * s
            let material = (shape.material ?? body.material).b3
            switch shape.geometry {
            case .sphere(let radius) where radius.isFinite && radius > 0:
                spheres.append(b3CompoundSphereDef(sphere: b3Sphere(center: b3Vec3(t), radius: radius * abs(s).max()), material: material))
            case .capsule(let height, let radius) where height.isFinite && radius.isFinite && radius > 0:
                let half = max(height * 0.5 - radius, 0)
                capsules.append(b3CompoundCapsuleDef(capsule: b3Capsule(center1: b3Vec3(r.act([0, -half, 0]) * s + t),
                                                                       center2: b3Vec3(r.act([0, half, 0]) * s + t),
                                                                       radius: radius * max(abs(s.x), abs(s.z))),
                                                     material: material))
            case .box(let size) where size.x.isFinite && size.y.isFinite && size.z.isFinite:
                let e = size / 2
                var corners: [SIMD3<Float>] = []
                for x: Float in [-1, 1] { for y: Float in [-1, 1] { for z: Float in [-1, 1] { corners.append(r.act(e * [x, y, z]) * s + t) } } }
                guard let h = hull(corners) else { rest.append(shape); continue }
                hullData.append(h)
                hulls.append(b3CompoundHullDef(hull: h, transform: b3Transform_identity, material: material))
            case .convex(let points):
                guard let h = hull(points.map { r.act($0) * s + t }) else { rest.append(shape); continue }
                hullData.append(h)
                hulls.append(b3CompoundHullDef(hull: h, transform: b3Transform_identity, material: material))
            case .staticMesh(let positions, let indices):
                guard positions.count >= 3, indices.count >= 3, indices.count % 3 == 0,
                      indices.allSatisfy({ Int($0) < positions.count }),
                      positions.allSatisfy({ $0.x.isFinite && $0.y.isFinite && $0.z.isFinite }) else { rest.append(shape); continue }
                let surfaces = Self.validSurfaces(shape.triangleMaterials, triangles: indices.count / 3)
                guard let mesh = Self.makeMesh(positions.map { r.act($0) * s + t }, indices, surfaces) else { rest.append(shape); continue }
                // Box3D counts a mesh's materials from the triangles it kept; the palette must match.
                let count = max(Int(mesh.pointee.materialCount), 1)
                var palette = Array((surfaces?.materials.map(\.b3) ?? [material]).prefix(count))
                while palette.count < count { palette.append(material) }
                meshParts.append((mesh, palette))
            default:
                rest.append(shape)
            }
        }
        guard spheres.count + capsules.count + hulls.count + meshParts.count > 0 else { return rest }
        // The compound clones every part, so the parts are freed when this returns.
        // Palettes live in buffers of their own until the compound has cloned them.
        let palettes = meshParts.map { part -> UnsafeMutableBufferPointer<b3SurfaceMaterial> in
            let buffer = UnsafeMutableBufferPointer<b3SurfaceMaterial>.allocate(capacity: part.palette.count)
            _ = buffer.initialize(from: part.palette)
            return buffer
        }
        defer { for p in palettes { p.deallocate() } }
        var meshDefs = zip(meshParts, palettes).map { part, palette in
            b3CompoundMeshDef(meshData: part.mesh, transform: b3Transform_identity, scale: b3Vec3(x: 1, y: 1, z: 1),
                              materials: UnsafePointer(palette.baseAddress), materialCount: Int32(palette.count))
        }
        let compound: UnsafeMutablePointer<b3CompoundData>? = spheres.withUnsafeMutableBufferPointer { sp in
            capsules.withUnsafeMutableBufferPointer { cp in
                hulls.withUnsafeMutableBufferPointer { hp in
                    meshDefs.withUnsafeMutableBufferPointer { mp in
                        var cd = b3CompoundDef()
                        cd.spheres = sp.baseAddress; cd.sphereCount = Int32(sp.count)
                        cd.capsules = cp.baseAddress; cd.capsuleCount = Int32(cp.count)
                        cd.hulls = hp.baseAddress; cd.hullCount = Int32(hp.count)
                        cd.meshes = mp.baseAddress; cd.meshCount = Int32(mp.count)
                        return b3CreateCompound(&cd)
                    }
                }
            }
        }
        guard let compound else {
            log.error("Box3D could not bake a compound; adding the shapes one by one")
            return collision.shapes
        }
        compounds.append(compound)
        shapeIds.append(b3CreateBakedCompoundShape(bodyId, &def, compound))
        return rest
    }
}

import CBox3D
import RealityKit
import simd
import os
import Foundation
import Combine

let log = Logger(subsystem: "RealityBox3D", category: "world")

/// One Box3D world, bound to a simulation root entity or to the scene itself.
///
/// Normally owned by `B3PhysicsSystem`. Tests and custom loops can drive it directly
/// with `update(entities:deltaTime:)`.
@MainActor
public final class B3World {
    public let id: b3WorldId
    public private(set) weak var root: Entity?
    public private(set) var settings: B3PhysicsSimulationComponent

    var records: [Entity.ID: BodyRecord] = [:]
    // Joints, rebuilt from the root's B3PhysicsJointsComponent when it or its bodies change.
    var jointSlots: [JointSlot] = []
    var appliedJoints: B3PhysicsJointsComponent?
    var anchors: [Entity.ID: JointAnchor] = [:]
    /// Joints that broke this frame, already gone, to report at its end.
    var brokenJoints: [B3JointEvents.Broke] = []
    var jointBreakHandlers: [UUID: (B3JointEvents.Broke) -> Void] = [:]
    /// One-way shapes, read by Box3D's pre-solve callback, and the bodies that own them.
    let oneWayShapes = OneWayShapes()
    var oneWayRecords: [Entity.ID: BodyRecord] = [:]
    /// Kinematic bodies, driven toward their entity's pose every fixed step.
    private var kinematic: [Entity.ID: BodyRecord] = [:]
    /// Dynamic bodies that moved in the latest step, or need one more write to settle.
    private var active: [Entity.ID: BodyRecord] = [:]
    private var accumulator: Double = 0
    private var stepIndex = 0
    private var frameIndex = 0
    private var subscriptions: [UUID: (B3CollisionEventKind, Entity.ID?, (any B3CollisionEvent) -> Void)] = [:]

    /// Timings of the latest `update`, in milliseconds.
    public private(set) var statistics = B3Statistics()

    public convenience init(root: Entity?, settings: B3PhysicsSimulationComponent = .init()) {
        self.init(root: root, settings: settings, registersStandalone: true)
    }

    init(root: Entity?, settings: B3PhysicsSimulationComponent, registersStandalone: Bool) {
        self.root = root
        self.settings = settings
        Self.routeLogOnce
        var def = b3DefaultWorldDef()
        def.gravity = b3Vec3(settings.gravity)
        def.enableSleep = settings.isSleepEnabled
        def.enableContinuous = settings.isContinuousEnabled
        def.workerCount = UInt32(max(1, settings.workerCount))
        if let r = settings.restitutionThreshold { def.restitutionThreshold = r }
        self.id = b3CreateWorld(&def)
        applyContactRules(settings)
        // Lets `entity.b3` find worlds the system does not own.
        if registersStandalone { B3WorldRegistry.shared.registerStandalone(self, root: root) }
    }

    isolated deinit {
        for r in records.values { r.destroy(retiring: false) }
        b3DestroyWorld(id)
    }

    private static let routeLogOnce: Void = routeBox3DLog()

    public var bodyCount: Int { records.count }
    var activeCount: Int { active.count }
    public var awakeBodyCount: Int { Int(b3World_GetAwakeBodyCount(id)) }

    // MARK: Frame update

    /// Syncs `entities` into Box3D, advances the fixed-step simulation by `deltaTime`,
    /// and writes poses and velocities back to the entities.
    public func update(entities: [Entity], deltaTime: Double) {
        let t0 = ContinuousClock.now
        beginFrame()
        let root = self.root
        frameIndex &+= 1
        var seenCount = 0
        for entity in entities {
            guard ingest(entity, root: root) else { continue }
            if let record = records[entity.id], record.seenFrame != frameIndex {
                record.seenFrame = frameIndex
                seenCount += 1
            }
        }
        // Bodies whose entity was removed or lost its components.
        if seenCount != records.count {
            for (key, record) in records where record.seenFrame != frameIndex { remove(key, record) }
        }
        finishFrame(deltaTime: deltaTime, inboundStart: t0)
    }

    /// Like `update(entities:deltaTime:)`, but reads only the entities that changed since the last
    /// frame, and drops the bodies of removed ones. Bodies whose entity did not change keep their
    /// state, so sleeping piles cost nothing to read. `B3PhysicsSystem` feeds it from RealityKit's
    /// change events.
    public func update(changed: some Sequence<Entity>, removed: some Sequence<Entity.ID>, deltaTime: Double) {
        let t0 = ContinuousClock.now
        beginFrame()
        let root = self.root
        for key in removed {
            if let record = records[key] { remove(key, record) }
        }
        for entity in changed where !ingest(entity, root: root) {
            if let record = records[entity.id] { remove(entity.id, record) }
        }
        finishFrame(deltaTime: deltaTime, inboundStart: t0)
    }

    /// Whether the world is writing poses and velocities to entities right now. The system ignores
    /// change events raised by these writes.
    public private(set) static var isWritingEntities = false

    /// Runs a write to entities that the system must not read back as an app edit.
    static func whileWritingEntities(_ body: () -> Void) {
        let was = isWritingEntities
        isWritingEntities = true
        defer { isWritingEntities = was }
        body()
    }

    private var entitiesRead = 0

    private func beginFrame() {
        entitiesRead = 0
        if let root, let s = root.components[B3PhysicsSimulationComponent.self], s != settings { apply(settings: s) }
        frames.removeAll(keepingCapacity: true)
    }

    /// Creates or updates the body of one entity. False if it lacks the body or collision component.
    private func ingest(_ entity: Entity, root: Entity?) -> Bool {
        entitiesRead += 1
        guard let body = entity.components[B3PhysicsBodyComponent.self],
              let collision = entity.components[B3CollisionComponent.self] else { return false }
        let pose = Pose(entity: entity, frame: frame(for: entity.parent, root: root), root: root)
        let motion = entity.components[B3PhysicsMotionComponent.self]
        if let record = records[entity.id] {
            nestedBodyCount += record.refreshHierarchy(root: root)
            if record.body != body || record.collision != collision || !record.scale.isClose(to: pose.scale) {
                record.apply(body: body, collision: collision, scale: pose.scale)
            }
            // Enabled or disabled before the sync: a re-enabled body takes this frame's edits over
            // the velocities it kept, a disabled one is placed and keeps them for later.
            record.setEnabled(entity.isEnabledInHierarchy)
            record.syncInbound(pose: pose, motion: motion)
        } else {
            let record = BodyRecord(world: self, entity: entity, body: body, collision: collision, pose: pose, motion: motion)
            records[entity.id] = record
            if record.isNested { nestedBodyCount += 1 }
            bodyModeChanged(record)
            record.setEnabled(entity.isEnabledInHierarchy)
        }
        return true
    }

    private func remove(_ key: Entity.ID, _ record: BodyRecord) {
        if record.isNested { nestedBodyCount -= 1 }
        if oneWayRecords.removeValue(forKey: key) != nil {
            for s in record.shapeIds { oneWayShapes.directions[.init(s)] = nil }
        }
        record.destroy()
        records[key] = nil
        kinematic[key] = nil
        active[key] = nil
    }

    private func finishFrame(deltaTime: Double, inboundStart t0: ContinuousClock.Instant) {
        readAnchors()
        syncJoints()
        let root = self.root
        let fixedDt = 1.0 / settings.fixedRate

        // Fixed steps.
        let t1 = ContinuousClock.now
        if !settings.isPaused { accumulator += deltaTime }
        var steps = 0
        var events: [any B3CollisionEvent] = []
        while accumulator >= fixedDt && steps < settings.maxStepsPerFrame {
            step(fixedDt: Float(fixedDt), events: &events)
            accumulator -= fixedDt
            steps += 1
        }
        if steps == settings.maxStepsPerFrame { accumulator = min(accumulator, fixedDt) }
        let t2 = ContinuousClock.now

        // Outbound sync, only for bodies Box3D reported as moving.
        let alpha = settings.interpolatesPoses ? Float(accumulator / fixedDt) : 1
        writeActive(alpha: alpha, root: root)

        // Events after poses are final, so handlers see consistent transforms.
        let broken = finishJointBreaks()
        dispatch(events)
        dispatch(broken)

        let t3 = ContinuousClock.now
        statistics = B3Statistics(bodies: records.count, awakeBodies: awakeBodyCount, steps: steps,
                                  syncInMilliseconds: (t1 - t0).milliseconds,
                                  stepMilliseconds: (t2 - t1).milliseconds,
                                  syncOutMilliseconds: (t3 - t2).milliseconds)
        statistics.entitiesRead = entitiesRead
    }

    private func writeActive(alpha: Float, root: Entity?) {
        let was = Self.isWritingEntities
        Self.isWritingEntities = true
        defer { Self.isWritingEntities = was }
        var settled: [Entity.ID] = []
        if hasNestedBodies {
            // A body under another body: write parents first, and drop cached parent frames after
            // each write, so a child is placed through its parent's new transform. A sleeping
            // dynamic body under a body that just moved was carried along on screen; its pose
            // belongs to Box3D, so it is put back in the same parent-first pass, before anything
            // nested under it is written.
            var written = Set<Entity.ID>()
            var candidates = active
            for (key, r) in records where r.isNested && r.body.mode == .dynamic { candidates[key] = r }
            for r in candidates.values.sorted(by: { $0.depth < $1.depth }) {
                let key = r.entity.id
                if active[key] != nil {
                    guard r.body.mode == .dynamic || r.isVelocityDriven else { settled.append(key); continue }
                    r.syncOutbound(alpha: alpha, lastStep: stepIndex, frame: frame(for: r.entity.parent, root: root), root: root)
                    if r.lastMovedStep < stepIndex { settled.append(key) }
                } else {
                    guard hasWrittenAncestor(r.entity, written, root: root) else { continue }
                    r.syncOutbound(alpha: 1, lastStep: stepIndex, frame: frame(for: r.entity.parent, root: root), root: root)
                }
                frames.removeAll(keepingCapacity: true)
                written.insert(key)
            }
        } else {
            for (key, r) in active {
                // A kinematic body switched to following its entity no longer writes its pose.
                guard r.body.mode == .dynamic || r.isVelocityDriven else { settled.append(key); continue }
                r.syncOutbound(alpha: alpha, lastStep: stepIndex, frame: frame(for: r.entity.parent, root: root), root: root)
                if r.lastMovedStep < stepIndex { settled.append(key) }
            }
        }
        for key in settled { active[key] = nil }
    }

    private func hasWrittenAncestor(_ entity: Entity, _ written: Set<Entity.ID>, root: Entity?) -> Bool {
        var ancestor = entity.parent
        while let e = ancestor, e !== root {
            if written.contains(e.id) { return true }
            ancestor = e.parent
        }
        return false
    }

    /// Whether any body sits under another body; then outbound writes go parents first.
    private var hasNestedBodies: Bool { nestedBodyCount > 0 }
    var nestedBodyCount = 0

    /// Advances one fixed step, whatever the accumulator says. For paused stepping and tools.
    public func step(events: inout [any B3CollisionEvent]) {
        step(fixedDt: Float(1.0 / settings.fixedRate), events: &events)
        frames.removeAll(keepingCapacity: true)
        writeActive(alpha: 1, root: root)
        dispatch(finishJointBreaks())
    }

    private func step(fixedDt: Float, events: inout [any B3CollisionEvent]) {
        for h in willSimulate.values { h(.init(deltaTime: fixedDt)) }
        for r in kinematic.values { r.driveKinematic(fixedDt: fixedDt) }
        if !anchors.isEmpty { driveAnchors(fixedDt: fixedDt) }
        stepIndex += 1
        // Sequential impacts run Box3D's substeps as steps of their own, each after an impact
        // pass, so a ball stops within a substep's travel of the one it strikes.
        let parts = settings.sequentialImpacts ? max(settings.subStepCount, 1) : 1
        let partDt = fixedDt / Float(parts)
        for part in 0..<parts {
            // Box3D clears applied forces after each of its steps; they act on the whole fixed step.
            if part > 0 { pendingForces.apply() }
            // Each step judges one-way contacts by where the surfaces point at its start.
            refreshOneWayShapes()
            if settings.sequentialImpacts { resolveImpacts(dt: partDt) }
            b3World_Step(id, partDt, Int32(parts == 1 ? settings.subStepCount : 1))
            let moves = b3World_GetBodyEvents(id)
            for i in 0..<Int(moves.moveCount) {
                let e = moves.moveEvents[i]
                guard let raw = e.userData else { continue }
                let key = Entity.ID(UInt(bitPattern: raw))
                guard let r = records[key], r.body.mode == .dynamic || r.isVelocityDriven else { continue }
                r.moved(to: e.transform, step: stepIndex)
                active[key] = r
            }
            collectEvents(into: &events)
            impactHits = []
            collectJointBreaks()
        }
        pendingForces = PendingForces()
        retiredShapes.removeAll(keepingCapacity: true)
        for h in didSimulate.values { h(.init(deltaTime: fixedDt)) }
    }

    /// The impacts the sequential pass resolved before the current Box3D step.
    var impactHits: [SequentialImpacts.Hit] = []

    /// Forces for the current fixed step, kept so that they act on every substep when sequential
    /// impacts step them one by one.
    var pendingForces = PendingForces()

    /// Whether forces must be kept for `pendingForces`.
    var keepsForces: Bool { settings.sequentialImpacts && settings.subStepCount > 1 }

    private var willSimulate: [UUID: (B3PhysicsSimulationEvents.WillSimulate) -> Void] = [:]
    private var didSimulate: [UUID: (B3PhysicsSimulationEvents.DidSimulate) -> Void] = [:]

    /// Runs before every fixed step. Forces added here act on that step. Keep the subscription.
    public func subscribe(to event: B3PhysicsSimulationEvents.WillSimulate.Type,
                          _ handler: @escaping (B3PhysicsSimulationEvents.WillSimulate) -> Void) -> B3Subscription {
        let token = UUID()
        willSimulate[token] = handler
        return B3Subscription { [weak self] in self?.willSimulate[token] = nil }
    }

    /// Runs after every fixed step, with Box3D's state already advanced. Keep the subscription.
    public func subscribe(to event: B3PhysicsSimulationEvents.DidSimulate.Type,
                          _ handler: @escaping (B3PhysicsSimulationEvents.DidSimulate) -> Void) -> B3Subscription {
        let token = UUID()
        didSimulate[token] = handler
        return B3Subscription { [weak self] in self?.didSimulate[token] = nil }
    }

    private var frames: [ObjectIdentifier: ParentFrame] = [:]
    private func frame(for parent: Entity?, root: Entity?) -> ParentFrame {
        if parent === root { return .identity }
        guard let parent else { return .general }
        let key = ObjectIdentifier(parent)
        if let f = frames[key] { return f }
        let f = ParentFrame(parent: parent, root: root)
        frames[key] = f
        return f
    }

    func bodyModeChanged(_ r: BodyRecord) {
        let key = r.entity.id
        kinematic[key] = r.body.mode == .kinematic ? r : nil
        if r.body.mode != .dynamic { active[key] = nil }
    }

    private func apply(settings s: B3PhysicsSimulationComponent) {
        b3World_SetGravity(id, b3Vec3(s.gravity))
        b3World_EnableSleeping(id, s.isSleepEnabled)
        b3World_EnableContinuous(id, s.isContinuousEnabled)
        b3World_SetWorkerCount(id, Int32(max(1, s.workerCount)))
        b3World_SetRestitutionThreshold(id, s.restitutionThreshold ?? b3DefaultWorldDef().restitutionThreshold)
        if s.frictionCombine != settings.frictionCombine || s.restitutionCombine != settings.restitutionCombine {
            applyContactRules(s)
        }
        settings = s
    }

    // MARK: Lookup

    func entity(for shape: b3ShapeId) -> Entity? {
        // A shape destroyed since the last step can still appear in that step's end events.
        guard b3Shape_IsValid(shape), let raw = b3Shape_GetUserData(shape) else { return retiredShapes[shape] }
        return records[Entity.ID(UInt(bitPattern: raw))]?.entity ?? retiredShapes[shape]
    }

    /// Shapes destroyed since the last step, kept until its events are read so that Box3D's end
    /// events for them still name their entity.
    private var retiredShapes: [b3ShapeId: Entity] = [:]

    func retire(_ shapes: [b3ShapeId], of entity: Entity) {
        for s in shapes { retiredShapes[s] = entity }
    }

    func record(for entity: Entity) -> BodyRecord? { records[entity.id] }

    // MARK: Events

    private func collectEvents(into out: inout [any B3CollisionEvent]) {
        let c = b3World_GetContactEvents(id)
        for i in 0..<Int(c.beginCount) {
            let e = c.beginEvents[i]
            if let a = entity(for: e.shapeIdA), let b = entity(for: e.shapeIdB) {
                out.append(B3CollisionEvents.Began(entityA: a, entityB: b))
            }
        }
        for i in 0..<Int(c.endCount) {
            let e = c.endEvents[i]
            if let a = entity(for: e.shapeIdA), let b = entity(for: e.shapeIdB) {
                out.append(B3CollisionEvents.Ended(entityA: a, entityB: b))
            }
        }
        var hitPairs = Set<SequentialImpacts.Pair>()
        for i in 0..<Int(c.hitCount) {
            let e = c.hitEvents[i]
            if !impactHits.isEmpty {
                hitPairs.insert(.init(sphere: e.shapeIdA, other: e.shapeIdB))
                hitPairs.insert(.init(sphere: e.shapeIdB, other: e.shapeIdA))
            }
            if let a = entity(for: e.shapeIdA), let b = entity(for: e.shapeIdB) {
                out.append(B3CollisionEvents.Hit(entityA: a, entityB: b, position: e.point.simd,
                                                 normal: e.normal.simd, approachSpeed: e.approachSpeed))
            }
        }
        // Impacts the sequential pass resolved, which Box3D then saw separating.
        for h in impactHits where !hitPairs.contains(h.pair) {
            if let a = entity(for: h.pair.sphere), let b = entity(for: h.pair.other) {
                out.append(B3CollisionEvents.Hit(entityA: a, entityB: b, position: h.point, normal: h.normal, approachSpeed: h.speed))
            }
        }
        let s = b3World_GetSensorEvents(id)
        for i in 0..<Int(s.beginCount) {
            let e = s.beginEvents[i]
            if let a = entity(for: e.sensorShapeId), let b = entity(for: e.visitorShapeId) {
                out.append(B3CollisionEvents.Began(entityA: a, entityB: b))
            }
        }
        for i in 0..<Int(s.endCount) {
            let e = s.endEvents[i]
            if let a = entity(for: e.sensorShapeId), let b = entity(for: e.visitorShapeId) {
                out.append(B3CollisionEvents.Ended(entityA: a, entityB: b))
            }
        }
    }

    private func dispatch(_ events: [any B3CollisionEvent]) {
        guard !events.isEmpty, !subscriptions.isEmpty else { return }
        let subs = Array(subscriptions.values)
        for event in events {
            for (kind, filter, handler) in subs where kind == type(of: event).kind {
                if let filter, filter != event.entityA.id && filter != event.entityB.id { continue }
                handler(event)
            }
        }
    }

    /// Mirrors `Scene.subscribe(to:on:_:)` for collision events. Keep the subscription: releasing it cancels.
    public func subscribe<E: B3CollisionEvent>(to event: E.Type, on entity: Entity? = nil,
                                               _ handler: @escaping (E) -> Void) -> B3Subscription {
        let token = UUID()
        subscriptions[token] = (E.kind, entity?.id, { handler($0 as! E) })
        return B3Subscription { [weak self] in self?.subscriptions[token] = nil }
    }

    // MARK: Queries

    /// Mirrors `Scene.raycast(origin:direction:length:query:mask:relativeTo:)`.
    /// Coordinates are in the simulation root's space.
    public func raycast(origin: SIMD3<Float>, direction: SIMD3<Float>, length: Float = 100,
                        query: CollisionCastQueryType = .all, mask: CollisionGroup = .all) -> [B3CastHit] {
        var filter = b3DefaultQueryFilter()
        filter.maskBits = UInt64(mask.rawValue)
        let translation = b3Vec3(simd_normalize(direction) * length)
        let o = b3Vec3(origin)

        if query == .nearest || query == .any {
            let r = b3World_CastRayClosest(id, o, translation, filter)
            guard r.hit, let e = entity(for: r.shapeId) else { return [] }
            return [B3CastHit(entity: e, position: r.point.simd, normal: r.normal.simd, distance: r.fraction * length)]
        }

        final class Box { var hits: [(b3ShapeId, SIMD3<Float>, SIMD3<Float>, Float)] = [] }
        let box = Box()
        _ = withExtendedLifetime(box) {
            b3World_CastRay(id, o, translation, filter, { shape, point, normal, fraction, _, _, _, ctx in
                Unmanaged<Box>.fromOpaque(ctx!).takeUnretainedValue().hits.append((shape, point.simd, normal.simd, fraction))
                return 1 // keep going, collect all
            }, Unmanaged.passUnretained(box).toOpaque())
        }
        return box.hits
            .sorted { $0.3 < $1.3 }
            .compactMap { h in entity(for: h.0).map { B3CastHit(entity: $0, position: h.1, normal: h.2, distance: h.3 * length) } }
    }
}

public struct B3Statistics: Sendable {
    public var bodies = 0
    public var awakeBodies = 0
    /// Fixed steps taken in the latest frame.
    public var steps = 0
    /// Reading entities into Box3D.
    public var syncInMilliseconds = 0.0
    /// Box3D's own stepping, all fixed steps of the frame.
    public var stepMilliseconds = 0.0
    /// Writing poses, velocities and events back.
    public var syncOutMilliseconds = 0.0
    /// Entities read in this frame: all of them with a full scan, only changed ones from the system.
    public var entitiesRead = 0

    public init() {}
    init(bodies: Int, awakeBodies: Int, steps: Int, syncInMilliseconds: Double, stepMilliseconds: Double, syncOutMilliseconds: Double) {
        self.bodies = bodies; self.awakeBodies = awakeBodies; self.steps = steps
        self.syncInMilliseconds = syncInMilliseconds; self.stepMilliseconds = stepMilliseconds; self.syncOutMilliseconds = syncOutMilliseconds
    }
}

extension Duration {
    var milliseconds: Double { Double(components.seconds) * 1e3 + Double(components.attoseconds) * 1e-15 }
}

public struct B3CastHit {
    public let entity: Entity
    public let position: SIMD3<Float>
    public let normal: SIMD3<Float>
    public let distance: Float
}

@MainActor
public final class B3Subscription: @MainActor Cancellable {
    private var onCancel: (@MainActor () -> Void)?
    init(_ onCancel: @escaping @MainActor () -> Void) { self.onCancel = onCancel }
    public func cancel() { onCancel?(); onCancel = nil }
    /// Like Combine's `AnyCancellable`: releasing the subscription cancels it.
    isolated deinit { onCancel?() }
}

// MARK: - Pose

struct Pose {
    var position: SIMD3<Float>
    var orientation: simd_quatf
    var scale: SIMD3<Float>

    @MainActor init(entity: Entity, root: Entity?) {
        self.init(entity: entity, frame: ParentFrame(parent: entity.parent, root: root), root: root)
    }

    @MainActor init(entity: Entity, frame: ParentFrame, root: Entity?) {
        switch frame {
        case .identity:
            let t = entity.transform
            position = t.translation; orientation = t.rotation; scale = t.scale
        case .similarity(let q, let t, let s):
            let local = entity.transform
            position = t + s * q.act(local.translation)
            orientation = q * local.rotation
            scale = s * local.scale
        case .general:
            position = entity.position(relativeTo: root)
            orientation = entity.orientation(relativeTo: root)
            scale = entity.scale(relativeTo: root)
        }
    }

    init(position: SIMD3<Float>, orientation: simd_quatf, scale: SIMD3<Float>) {
        self.position = position; self.orientation = orientation; self.scale = scale
    }

    func isClose(to o: Pose) -> Bool {
        simd_distance_squared(position, o.position) < 1e-8 && abs(simd_dot(orientation.vector, o.orientation.vector)) > 1 - 1e-6
    }
}

/// How an entity's parent space maps to the simulation root's space. Computed once per parent per
/// frame, so bodies sharing a parent skip RealityKit's per-call hierarchy conversions.
enum ParentFrame {
    /// The parent is the root (or both are the scene): local is root space.
    case identity
    /// Rotation, translation and uniform scale: plain math composes it exactly.
    case similarity(simd_quatf, SIMD3<Float>, Float)
    /// Non-uniform scale somewhere in the chain: let RealityKit convert.
    case general

    @MainActor init(parent: Entity?, root: Entity?) {
        if parent === root { self = .identity; return }
        guard let parent else { self = .general; return }
        let t = parent.transformMatrix(relativeTo: root)
        let sx = simd_length(SIMD3(t.columns.0.x, t.columns.0.y, t.columns.0.z))
        let sy = simd_length(SIMD3(t.columns.1.x, t.columns.1.y, t.columns.1.z))
        let sz = simd_length(SIMD3(t.columns.2.x, t.columns.2.y, t.columns.2.z))
        guard sx > 0, abs(sx - sy) < 1e-5 * sx, abs(sx - sz) < 1e-5 * sx else { self = .general; return }
        let r = simd_float3x3(SIMD3(t.columns.0.x, t.columns.0.y, t.columns.0.z) / sx,
                              SIMD3(t.columns.1.x, t.columns.1.y, t.columns.1.z) / sx,
                              SIMD3(t.columns.2.x, t.columns.2.y, t.columns.2.z) / sx)
        self = .similarity(simd_quatf(r), SIMD3(t.columns.3.x, t.columns.3.y, t.columns.3.z), sx)
    }

    /// Writes a root-space pose to the entity as its local transform.
    @MainActor func write(position p: SIMD3<Float>, orientation q: simd_quatf, scale: SIMD3<Float>, to entity: Entity, root: Entity?) -> Pose {
        switch self {
        case .identity:
            entity.transform = Transform(scale: scale, rotation: q, translation: p)
            return Pose(position: p, orientation: q, scale: scale)
        case .similarity(let pq, let pt, let ps):
            let inv = pq.inverse
            let local = Transform(scale: scale / ps, rotation: inv * q, translation: inv.act(p - pt) / ps)
            entity.transform = local
            return Pose(position: p, orientation: q, scale: scale)
        case .general:
            entity.setPosition(p, relativeTo: root)
            entity.setOrientation(q, relativeTo: root)
            return Pose(entity: entity, frame: self, root: root)
        }
    }
}

extension SIMD3<Float> {
    func isClose(to o: Self) -> Bool { simd_distance_squared(self, o) < 1e-10 }
}

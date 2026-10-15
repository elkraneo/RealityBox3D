import CBox3D
import RealityKit
import simd

/// Box3D extension: how the materials of two touching shapes combine into one friction or
/// restitution.
public enum B3MaterialCombine: Equatable, Sendable {
    case geometricMean, average, minimum, maximum, multiply

    // Box3D calls these from its solver threads, so they are plain C functions without context.
    var callback: @convention(c) (Float, UInt64, Float, UInt64) -> Float {
        switch self {
        case .geometricMean: { a, _, b, _ in (a * b).squareRoot() }
        case .average: { a, _, b, _ in (a + b) / 2 }
        case .minimum: { a, _, b, _ in min(a, b) }
        case .maximum: { a, _, b, _ in max(a, b) }
        case .multiply: { a, _, b, _ in a * b }
        }
    }
}

/// The one-way shapes of a world and the direction each pushes along, in the root's space.
/// Written on the main actor between steps, read by Box3D's solver threads during a step.
final class OneWayShapes: @unchecked Sendable {
    /// Keyed by shape index and generation, not world: a forecast's copy has the same shapes.
    struct Key: Hashable { let index: Int32, generation: UInt16 }
    var directions: [Key: SIMD3<Float>] = [:]

    /// A one-way body: where its shapes point in its own space, mirrored by its scale.
    struct Source { var body: b3BodyId; let local: SIMD3<Float>; let shapes: [Key] }

    init(directions: [Key: SIMD3<Float>] = [:]) { self.directions = directions }

    /// Points each source's shapes along its body's current rotation.
    func refresh(_ sources: [Source]) {
        for source in sources where b3Body_IsValid(source.body) {
            let d = b3Body_GetRotation(source.body).simd.act(source.local)
            for key in source.shapes { directions[key] = d }
        }
    }

    /// A contact on a one-way shape counts only when it pushes the other body along the shape's
    /// direction, within about 45°.
    static let preSolve: @convention(c) (b3ShapeId, b3ShapeId, b3Pos, b3Vec3, UnsafeMutableRawPointer?) -> Bool = { a, b, _, n, context in
        guard let context else { return true }
        let shapes = Unmanaged<OneWayShapes>.fromOpaque(context).takeUnretainedValue()
        // The normal points from shape A to shape B: A pushes B along it, B pushes A against it.
        let normal = SIMD3<Float>(Float(n.x), Float(n.y), Float(n.z))
        if let d = shapes.directions[.init(a)], simd_dot(normal, d) < 0.7 { return false }
        if let d = shapes.directions[.init(b)], simd_dot(-normal, d) < 0.7 { return false }
        return true
    }
}

extension B3World {
    func applyContactRules(_ s: B3PhysicsSimulationComponent) {
        Self.installContactRules(on: id, s, oneWay: oneWayShapes)
    }

    /// Installs the material combine modes and the one-way filter on a world, this one or a
    /// forecast's copy. `oneWay` must outlive the world's steps.
    static func installContactRules(on world: b3WorldId, _ s: B3PhysicsSimulationComponent, oneWay: OneWayShapes) {
        b3World_SetFrictionCallback(world, s.frictionCombine.callback)
        b3World_SetRestitutionCallback(world, s.restitutionCombine.callback)
        b3World_SetPreSolveCallback(world, OneWayShapes.preSolve, Unmanaged.passUnretained(oneWay).toOpaque())
    }

    /// Points each one-way shape's direction along its body's current rotation. Runs before
    /// every fixed step.
    func refreshOneWayShapes() {
        guard !oneWayRecords.isEmpty else { return }
        oneWayShapes.refresh(oneWaySources)
    }

    /// The one-way bodies, their directions in their own space and their shapes.
    var oneWaySources: [OneWayShapes.Source] {
        oneWayRecords.values.compactMap { r in
            guard let local = r.collision.oneWayDirection, simd_length_squared(local) > 0 else { return nil }
            // The entity's scale can mirror the direction; its rotation turns it.
            let mirror = SIMD3<Float>(r.scale.x < 0 ? -1 : 1, r.scale.y < 0 ? -1 : 1, r.scale.z < 0 ? -1 : 1)
            return .init(body: r.bodyId, local: simd_normalize(local * mirror), shapes: r.shapeIds.map(OneWayShapes.Key.init))
        }
    }

    /// Called by a body record when it creates or destroys its shapes.
    func oneWayShapesChanged(_ record: BodyRecord, removing old: [b3ShapeId]) {
        for s in old { oneWayShapes.directions[.init(s)] = nil }
        if record.collision.oneWayDirection != nil, b3Body_IsValid(record.bodyId) {
            // Box3D's mesh and height-field contacts never call the pre-solve callback.
            if record.collision.shapes.contains(where: \.isStaticOnly) || record.collision.bakesCompound {
                log.warning("oneWayDirection on meshes, height fields or compounds: Box3D filters their continuous-collision hits but not their regular contacts, which always collide.")
            }
            oneWayRecords[record.entity.id] = record
        } else {
            oneWayRecords[record.entity.id] = nil
        }
    }
}

extension OneWayShapes.Key {
    init(_ id: b3ShapeId) { self.init(index: id.index1, generation: id.generation) }
}

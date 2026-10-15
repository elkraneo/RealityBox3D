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
    var directions: [b3ShapeId: SIMD3<Float>] = [:]

    /// A contact on a one-way shape counts only when it pushes the other body along the shape's
    /// direction, within about 45°.
    static let preSolve: @convention(c) (b3ShapeId, b3ShapeId, b3Pos, b3Vec3, UnsafeMutableRawPointer?) -> Bool = { a, b, _, n, context in
        guard let context else { return true }
        let shapes = Unmanaged<OneWayShapes>.fromOpaque(context).takeUnretainedValue()
        // The normal points from shape A to shape B: A pushes B along it, B pushes A against it.
        let normal = SIMD3<Float>(Float(n.x), Float(n.y), Float(n.z))
        if let d = shapes.directions[a], simd_dot(normal, d) < 0.7 { return false }
        if let d = shapes.directions[b], simd_dot(-normal, d) < 0.7 { return false }
        return true
    }
}

extension B3World {
    func applyContactRules(_ s: B3PhysicsSimulationComponent) {
        b3World_SetFrictionCallback(id, s.frictionCombine.callback)
        b3World_SetRestitutionCallback(id, s.restitutionCombine.callback)
    }

    /// Points each one-way shape's direction along its body's current rotation. Runs before the
    /// steps of a frame.
    func refreshOneWayShapes() {
        guard !oneWayRecords.isEmpty else { return }
        for r in oneWayRecords.values {
            guard let local = r.collision.oneWayDirection, simd_length_squared(local) > 0 else { continue }
            // The entity's scale can mirror the direction; its rotation turns it.
            let mirror = SIMD3<Float>(r.scale.x < 0 ? -1 : 1, r.scale.y < 0 ? -1 : 1, r.scale.z < 0 ? -1 : 1)
            let d = b3Body_GetRotation(r.bodyId).simd.act(simd_normalize(local * mirror))
            for s in r.shapeIds { oneWayShapes.directions[s] = d }
        }
    }

    /// Called by a body record when it creates or destroys its shapes.
    func oneWayShapesChanged(_ record: BodyRecord, removing old: [b3ShapeId]) {
        for s in old { oneWayShapes.directions[s] = nil }
        if record.collision.oneWayDirection != nil, b3Body_IsValid(record.bodyId) {
            oneWayRecords[record.entity.id] = record
        } else {
            oneWayRecords[record.entity.id] = nil
        }
    }
}

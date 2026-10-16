import CBox3D
import RealityKit
import simd

/// Box3D extension: one touching pair, seen from the entity that asked.
public struct B3Contact {
    /// The entity touched.
    public let entity: Entity
    /// The contact points, in the simulation root's space.
    public let points: [SIMD3<Float>]
    /// Points from the asking entity toward the one touched; for a mesh touched in several places,
    /// the average of their normals.
    public let normal: SIMD3<Float>
    /// The force along the normal, summed over the points: Box3D's impulse in its last substep
    /// over that substep's length. Newtons when the simulation root is in meters.
    public let normalForce: Float
}

extension B3BodyProxy {
    private var simulated: (B3World, BodyRecord)? {
        guard let world = RealityBox3D.world(for: entity), let record = world.record(for: entity) else { return nil }
        return (world, record)
    }

    /// Box3D extension: what the body touches now, read live from Box3D.
    public var contacts: [B3Contact] {
        guard let (world, record) = simulated else { return [] }
        let capacity = Int(b3Body_GetContactCapacity(record.bodyId))
        guard capacity > 0 else { return [] }
        var data = [b3ContactData](repeating: b3ContactData(), count: capacity)
        let count = Int(b3Body_GetContactData(record.bodyId, &data, Int32(capacity)))
        let substep = Float(1 / (world.settings.fixedRate * Double(max(world.settings.subStepCount, 1))))
        var out: [B3Contact] = []
        for d in data.prefix(count) {
            guard let manifolds = d.manifolds, d.manifoldCount > 0 else { continue }
            // Box3D orders the pair its own way: flip to look from this body.
            let isA = b3Shape_GetBody(d.shapeIdA) == record.bodyId
            guard let other = world.entity(for: isA ? d.shapeIdB : d.shapeIdA) else { continue }
            let centerA = b3Body_GetWorldCenter(b3Shape_GetBody(d.shapeIdA)).simd
            var points: [SIMD3<Float>] = [], normal = SIMD3<Float>.zero, impulse: Float = 0
            let parts = UnsafeBufferPointer(start: manifolds, count: Int(d.manifoldCount))
            for m in parts {
                normal += m.normal.simd
                withUnsafeBytes(of: m.points) { raw in
                    for p in raw.bindMemory(to: b3ManifoldPoint.self).prefix(Int(m.pointCount)) {
                        points.append(centerA + p.anchorA.simd)
                        impulse += p.normalImpulse
                    }
                }
            }
            // Opposite faces of one mesh can cancel out; then the first manifold speaks for the pair.
            normal = simd_length_squared(normal) > 1e-12 ? simd_normalize(normal) : parts[0].normal.simd
            normal *= isA ? 1 : -1
            out.append(B3Contact(entity: other, points: points, normal: normal, normalForce: impulse / substep))
        }
        return out
    }

    /// Box3D extension: the entities inside this trigger now, read live from Box3D. Empty for a
    /// body that is not a trigger.
    public var overlappingEntities: [Entity] {
        guard let (world, record) = simulated else { return [] }
        var seen = Set<Entity.ID>(), out: [Entity] = []
        for shape in record.shapeIds {
            let capacity = Int(b3Shape_GetSensorCapacity(shape))
            guard capacity > 0 else { continue }
            var visitors = [b3ShapeId](repeating: b3ShapeId(), count: capacity)
            let count = Int(b3Shape_GetSensorData(shape, &visitors, Int32(capacity)))
            for v in visitors.prefix(count) where b3Shape_IsValid(v) {
                if let e = world.entity(for: v), seen.insert(e.id).inserted { out.append(e) }
            }
        }
        return out
    }

    /// Box3D extension: pushes the body with air moving at `wind` (root space unless `ref` is
    /// given), by the area each shape shows it and the body's own motion through it. `drag`
    /// opposes the relative motion, `lift` acts across it; `maxSpeed` caps the relative speed for
    /// stability. A force for one step: call it from `B3PhysicsSimulationEvents.WillSimulate`.
    public func applyWind(_ wind: SIMD3<Float>, drag: Float = 1, lift: Float = 0, maxSpeed: Float = 10, relativeTo ref: Entity? = nil) {
        guard let (_, record) = simulated, record.body.mode == .dynamic else { return }
        let w = ref.map { ref -> SIMD3<Float> in
            let m = ref.transformMatrix(relativeTo: RealityBox3D.world(for: entity)?.root)
            return (m * SIMD4(wind, 0)).xyz
        } ?? wind
        guard w.x.isFinite, w.y.isFinite, w.z.isFinite, drag.isFinite, lift.isFinite, maxSpeed.isFinite, maxSpeed >= 0 else { return }
        for s in record.shapeIds {
            b3Shape_ApplyWind(s, b3Vec3(w), drag, lift, maxSpeed, true)
            if record.world.keepsForces {
                record.world.pendingForces.add(.init(shape: s, wind: w, drag: drag, lift: lift, maxSpeed: maxSpeed))
            }
        }
    }
}

private extension SIMD4 where Scalar == Float {
    var xyz: SIMD3<Float> { SIMD3(x, y, z) }
}

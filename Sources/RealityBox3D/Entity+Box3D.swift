import CBox3D
import RealityKit
import simd

extension Entity {
    /// Box3D counterparts of `HasPhysicsBody` methods. Namespaced so they never collide
    /// with RealityKit's own `addForce` on `ModelEntity`.
    @MainActor public var b3: B3BodyProxy { B3BodyProxy(entity: self) }
}

@MainActor
public struct B3BodyProxy {
    let entity: Entity

    private enum Quantity {
        /// A position.
        case point
        /// Forces, impulses: one length in their units.
        case linear
        /// Torques, angular impulses: two lengths (N·m, N·m·s).
        case angular
    }

    private func resolve(_ v: SIMD3<Float>, relativeTo ref: Entity?, _ quantity: Quantity) -> (B3World, BodyRecord, SIMD3<Float>)? {
        guard let world = RealityBox3D.world(for: entity), let record = world.record(for: entity) else { return nil }
        // Convert from `ref` space into the simulation root's space.
        let m = (ref?.transformMatrix(relativeTo: world.root)) ?? (world.root.map { simd_inverse($0.transformMatrix(relativeTo: nil)) } ?? matrix_identity_float4x4)
        let linear = simd_float3x3(SIMD3(m.columns.0.x, m.columns.0.y, m.columns.0.z),
                                   SIMD3(m.columns.1.x, m.columns.1.y, m.columns.1.z),
                                   SIMD3(m.columns.2.x, m.columns.2.y, m.columns.2.z))
        switch quantity {
        case .point:
            let w = m * SIMD4(v, 1)
            return (world, record, SIMD3(w.x, w.y, w.z))
        case .linear:
            return (world, record, linear * v)
        case .angular:
            // A torque is r × F; with r and F both mapped by A, it maps by A's cofactor matrix,
            // det(A)·A⁻ᵀ, which handles non-uniform scale and mirrors. Built from cross products
            // of A's columns, it stays finite when A is singular, as with a zero scale.
            let (a, b, c) = (linear.columns.0, linear.columns.1, linear.columns.2)
            let cofactor = simd_float3x3(cross(b, c), cross(c, a), cross(a, b))
            return (world, record, cofactor * v)
        }
    }

    public var isSimulated: Bool { RealityBox3D.world(for: entity)?.record(for: entity) != nil }

    private var record: BodyRecord? { RealityBox3D.world(for: entity)?.record(for: entity) }

    /// The body's mass in kilograms, as Box3D resolved it. Nil until the system has seen the entity.
    public var mass: Float? { record.map { b3Body_GetMass($0.bodyId) } }

    /// Velocities in the simulation root's space, read live from Box3D.
    public var linearVelocity: SIMD3<Float>? { record?.currentMotion()?.linearVelocity }
    public var angularVelocity: SIMD3<Float>? { record?.currentMotion()?.angularVelocity }

    /// The body's pose in the simulation root's space, read live from Box3D. Between fixed
    /// steps of one frame it can be ahead of the entity's interpolated transform.
    public var pose: Transform? {
        record.map { let t = b3Body_GetTransform($0.bodyId); return Transform(rotation: t.q.simd, translation: t.p.simd) }
    }

    /// Replaces the body's velocities, waking it. Root space. A kinematic body keeps moving at
    /// them until its entity is moved, as RealityKit's kinematic bodies follow their velocity.
    public func setVelocity(linear: SIMD3<Float>, angular: SIMD3<Float>) {
        record?.setVelocity(linear: linear, angular: angular)
    }

    public var isAwake: Bool { record.map { b3Body_IsAwake($0.bodyId) } ?? false }

    public func addForce(_ force: SIMD3<Float>, relativeTo ref: Entity?) {
        resolve(force, relativeTo: ref, .linear).map { $0.1.addForce($0.2, at: nil) }
    }
    public func addForce(_ force: SIMD3<Float>, at position: SIMD3<Float>, relativeTo ref: Entity?) {
        guard let (_, r, f) = resolve(force, relativeTo: ref, .linear),
              let (_, _, p) = resolve(position, relativeTo: ref, .point) else { return }
        r.addForce(f, at: p)
    }
    public func addTorque(_ torque: SIMD3<Float>, relativeTo ref: Entity?) {
        resolve(torque, relativeTo: ref, .angular).map { $0.1.addTorque($0.2) }
    }
    public func applyLinearImpulse(_ impulse: SIMD3<Float>, relativeTo ref: Entity?) {
        resolve(impulse, relativeTo: ref, .linear).map { $0.1.applyLinearImpulse($0.2, at: nil) }
    }
    public func applyImpulse(_ impulse: SIMD3<Float>, at position: SIMD3<Float>, relativeTo ref: Entity?) {
        guard let (_, r, i) = resolve(impulse, relativeTo: ref, .linear),
              let (_, _, p) = resolve(position, relativeTo: ref, .point) else { return }
        r.applyLinearImpulse(i, at: p)
    }
    public func applyAngularImpulse(_ impulse: SIMD3<Float>, relativeTo ref: Entity?) {
        resolve(impulse, relativeTo: ref, .angular).map { $0.1.applyAngularImpulse($0.2) }
    }
}

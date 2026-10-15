import CBox3D
import simd

/// Box3D extension: impacts resolved pair by pair before each step (Guendelman, Bridson and Fedkiw,
/// "Nonconvex rigid bodies with stacking", 2003). Box3D's solver handles the contacts of a step
/// together and bounces only by the approach speed it saw before solving, so a ball striking a
/// row of touching balls shares its momentum among the row. Here a sphere about to strike a shape
/// within the step bounces off it at once, then the struck body, with its new velocity, strikes
/// the next, until nothing approaches: one ball in, one ball out, as in a Newton's cradle.
///
/// Spheres against spheres, capsules and hulls; other contacts are left to Box3D.
enum SequentialImpacts {
    struct Body {
        var linear: SIMD3<Float>, angular: SIMD3<Float>
        let inverseMass: Float
        let inverseInertia: simd_float3x3
        let center: SIMD3<Float>
        var changed = false
    }

    struct Pair: Hashable { let sphere: b3ShapeId, other: b3ShapeId }

    /// Resolves the impacts the spheres of `spheres` would make within `dt`.
    static func resolve(world: b3WorldId, spheres: [b3ShapeId], dt: Float, threshold: Float,
                        combine: B3MaterialCombine, slop: Float) {
        guard !spheres.isEmpty else { return }
        var bodies: [b3BodyId: Body] = [:]
        func body(_ id: b3BodyId) -> Body {
            if let b = bodies[id] { return b }
            // A sleeping dynamic body is struck like an awake one; setting its velocity wakes it.
            let dynamic = b3Body_GetType(id) == b3_dynamicBody && b3Body_IsEnabled(id)
            let mass = b3Body_GetMass(id)
            var inverseInertia = simd_float3x3()
            if dynamic, mass > 0 {
                let local = b3Body_GetLocalRotationalInertia(id)
                let m = simd_float3x3(local.cx.simd, local.cy.simd, local.cz.simd)
                if m.determinant > 1e-12 {
                    let r = simd_float3x3(b3Body_GetRotation(id).simd)
                    inverseInertia = r * m.inverse * r.transpose
                }
            }
            let b = Body(linear: b3Body_GetLinearVelocity(id).simd, angular: b3Body_GetAngularVelocity(id).simd,
                         inverseMass: dynamic && mass > 0 ? 1 / mass : 0, inverseInertia: inverseInertia,
                         center: b3Body_GetWorldCenter(id).simd)
            bodies[id] = b
            return b
        }

        // Candidate pairs: what each sphere could reach this step, the fastest mover's way included.
        let fastest = spheres.map { simd_length(b3Body_GetLinearVelocity(b3Shape_GetBody($0)).simd) }.max() ?? 0
        var pairs: [Pair] = [], seen = Set<Pair>()
        final class Found { var shapes: [b3ShapeId] = [] }
        // From the moving spheres out to the spheres they reach, sleeping ones included, so a
        // struck row passes the impact along.
        var queue = spheres, queued = Set(spheres), next = 0
        while next < queue.count {
            let s = queue[next]
            next += 1
            let reach = 2 * fastest * dt + slop
            var box = b3Shape_GetAABB(s)
            box.lowerBound = b3Vec3(box.lowerBound.simd - reach)
            box.upperBound = b3Vec3(box.upperBound.simd + reach)
            let found = Found()
            _ = b3World_OverlapAABB(world, box, b3DefaultQueryFilter(), { shape, context in
                Unmanaged<Found>.fromOpaque(context!).takeUnretainedValue().shapes.append(shape)
                return true
            }, Unmanaged.passUnretained(found).toOpaque())
            for o in found.shapes where b3Shape_GetBody(o) != b3Shape_GetBody(s) && collides(s, o) {
                let type = b3Shape_GetType(o)
                guard type == b3_sphereShape || type == b3_capsuleShape || type == b3_hullShape else { continue }
                if type == b3_sphereShape, b3Body_GetType(b3Shape_GetBody(o)) == b3_dynamicBody, queued.insert(o).inserted {
                    queue.append(o)
                }
                // A sphere pair once, from its lower shape.
                if type == b3_sphereShape, seen.contains(Pair(sphere: o, other: s)) { continue }
                let pair = Pair(sphere: s, other: o)
                if seen.insert(pair).inserted { pairs.append(pair) }
            }
        }
        guard !pairs.isEmpty else { return }

        // Pair by pair, from the velocities earlier impacts left, until none approaches.
        for _ in 0..<max(4 * pairs.count, 16) {
            var struck = false
            for pair in pairs {
                let sphere = b3Shape_GetSphere(pair.sphere)
                let idA = b3Shape_GetBody(pair.sphere), idB = b3Shape_GetBody(pair.other)
                var a = body(idA), b = body(idB)
                guard a.inverseMass + b.inverseMass > 0 else { continue }
                let t = b3Body_GetTransform(idA)
                let centerA = t.q.simd.act(sphere.center.simd) + t.p.simd
                let onB = b3Shape_GetClosestPoint(pair.other, b3Vec3(centerA)).simd
                let gap = onB - centerA
                let distance = simd_length(gap)
                guard distance > 1e-6 else { continue }
                let n = gap / distance
                let separation = distance - sphere.radius
                let rA = centerA + n * sphere.radius - a.center, rB = onB - b.center
                let vA = a.linear + simd_cross(a.angular, rA), vB = b.linear + simd_cross(b.angular, rB)
                let vn = simd_dot(vB - vA, n)
                // Approaching faster than the threshold, and touching within this step.
                guard vn < -threshold, separation + vn * dt <= slop else { continue }
                let e = combine.callback(b3Shape_GetRestitution(pair.sphere), 0, b3Shape_GetRestitution(pair.other), 0)
                let kA = simd_dot(n, simd_cross(a.inverseInertia * simd_cross(rA, n), rA))
                let kB = simd_dot(n, simd_cross(b.inverseInertia * simd_cross(rB, n), rB))
                let j = -(1 + e) * vn / (a.inverseMass + b.inverseMass + kA + kB)
                let impulse = j * n
                if a.inverseMass > 0 {
                    a.linear -= impulse * a.inverseMass
                    a.angular -= a.inverseInertia * simd_cross(rA, impulse)
                    a.changed = true
                }
                if b.inverseMass > 0 {
                    b.linear += impulse * b.inverseMass
                    b.angular += b.inverseInertia * simd_cross(rB, impulse)
                    b.changed = true
                }
                bodies[idA] = a
                bodies[idB] = b
                struck = true
            }
            if !struck { break }
        }
        for (id, b) in bodies where b.changed {
            b3Body_SetLinearVelocity(id, b3Vec3(b.linear))
            b3Body_SetAngularVelocity(id, b3Vec3(b.angular))
            b3Body_SetAwake(id, true)
        }
    }

    /// Box3D's filter rule: group index first, then categories and masks both ways. Sensors never.
    private static func collides(_ a: b3ShapeId, _ b: b3ShapeId) -> Bool {
        if b3Shape_IsSensor(a) || b3Shape_IsSensor(b) { return false }
        let fa = b3Shape_GetFilter(a), fb = b3Shape_GetFilter(b)
        if fa.groupIndex == fb.groupIndex, fa.groupIndex != 0 { return fa.groupIndex > 0 }
        return (fa.maskBits & fb.categoryBits) != 0 && (fa.categoryBits & fb.maskBits) != 0
    }
}

extension B3World {
    func resolveImpacts(dt: Float) {
        SequentialImpacts.resolve(world: id, spheres: SequentialImpacts.spheres(of: records.values.filter(\.isEnabled).flatMap(\.shapeIds)),
                                  dt: dt, threshold: b3World_GetRestitutionThreshold(id),
                                  combine: settings.restitutionCombine, slop: 0.005 * b3GetLengthUnitsPerMeter())
    }
}

extension SequentialImpacts {
    /// The sphere shapes among `shapes` whose bodies are dynamic and awake.
    static func spheres(of shapes: [b3ShapeId]) -> [b3ShapeId] {
        shapes.filter { s in
            guard b3Shape_IsValid(s), b3Shape_GetType(s) == b3_sphereShape else { return false }
            let body = b3Shape_GetBody(s)
            return b3Body_GetType(body) == b3_dynamicBody && b3Body_IsEnabled(body) && b3Body_IsAwake(body)
        }
    }
}

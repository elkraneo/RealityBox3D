import CBox3D
import simd

/// Box3D extension: impacts resolved pair by pair before each step (Guendelman, Bridson and Fedkiw,
/// "Nonconvex rigid bodies with stacking", 2003). Box3D's solver handles the contacts of a step
/// together and bounces only by the approach speed it saw before solving, so a ball striking a
/// row of touching balls shares its momentum among the row. Here a sphere about to strike a shape
/// within the step bounces off it at once, then the struck body, with its new velocity, strikes
/// the next, until nothing approaches: one ball in, one ball out, as in a Newton's cradle.
///
/// Spheres against spheres, capsules and hulls; other contacts are left to Box3D, as are bodies
/// held by joints, except a pendulum: one free revolute or spherical joint to a static body, which
/// the pass turns about its pivot. One-way shapes, pairs a joint keeps apart, and pairs turning
/// more than the slop within the step are Box3D's too.
enum SequentialImpacts {
    /// How a body answers an impulse.
    enum Response {
        case free(inverseMass: Float, inverseInertia: simd_float3x3)
        /// Turning about a fixed pivot: `inverseInertia` maps angular impulse about the pivot to
        /// angular velocity, limited to the axis of a hinge.
        case pivoted(pivot: SIMD3<Float>, inverseInertia: simd_float3x3)
        case immovable
        /// Held by joints the pass doesn't model: its impacts are Box3D's.
        case unsupported
    }

    struct Body {
        var linear: SIMD3<Float>, angular: SIMD3<Float>
        let response: Response
        let center: SIMD3<Float>
        var changed = false

        var movable: Bool {
            switch response {
            case .free, .pivoted: true
            case .immovable, .unsupported: false
            }
        }

        /// The change of velocity along `n` at `point` per unit impulse along `n` there.
        func k(at point: SIMD3<Float>, _ n: SIMD3<Float>) -> Float {
            switch response {
            case .free(let m, let i):
                let r = point - center
                return m + simd_dot(n, simd_cross(i * simd_cross(r, n), r))
            case .pivoted(let p, let i):
                let r = point - p
                return simd_dot(n, simd_cross(i * simd_cross(r, n), r))
            case .immovable, .unsupported: return 0
            }
        }

        mutating func apply(_ impulse: SIMD3<Float>, at point: SIMD3<Float>) {
            switch response {
            case .free(let m, let i):
                linear += impulse * m
                angular += i * simd_cross(point - center, impulse)
            case .pivoted(let p, let i):
                let turn = i * simd_cross(point - p, impulse)
                angular += turn
                linear += simd_cross(turn, center - p)
            case .immovable, .unsupported: return
            }
            changed = true
        }
    }

    struct Pair: Hashable { let sphere: b3ShapeId, other: b3ShapeId }

    /// An impact the pass resolved, for the world's hit events. The normal points from the sphere
    /// to the other shape.
    struct Hit { let pair: Pair; let point: SIMD3<Float>, normal: SIMD3<Float>; let speed: Float }

    /// Resolves the impacts the spheres of `spheres` would make within `dt`. Returns those at
    /// `hitThreshold` or faster between shapes that report hits, the fastest of each pair.
    /// `moverSpeed` bounds how fast any non-static shape's points move, kinematic ones included.
    @discardableResult
    static func resolve(world: b3WorldId, spheres: [b3ShapeId], dt: Float, threshold: Float,
                        combine: B3MaterialCombine, slop: Float, oneWay: OneWayShapes?,
                        moverSpeed: Float = 0, hitThreshold: Float = .infinity) -> [Hit] {
        guard !spheres.isEmpty else { return [] }
        var bodies: [b3BodyId: Body] = [:]
        func body(_ id: b3BodyId) -> Body {
            if let b = bodies[id] { return b }
            let b = Body(linear: b3Body_GetLinearVelocity(id).simd, angular: b3Body_GetAngularVelocity(id).simd,
                         response: response(of: id), center: b3Body_GetWorldCenter(id).simd)
            bodies[id] = b
            return b
        }

        // Candidate pairs: what each sphere could reach this step, the fastest mover's way included.
        let fastest = max(moverSpeed, spheres.map { simd_length(b3Body_GetLinearVelocity(b3Shape_GetBody($0)).simd) }.max() ?? 0)
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
            for o in found.shapes where b3Shape_GetBody(o) != b3Shape_GetBody(s) && collides(s, o, oneWay: oneWay) {
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
        guard !pairs.isEmpty else { return [] }

        // Pair by pair, from the velocities earlier impacts left, until none approaches.
        var hits: [Pair: Hit] = [:]
        for _ in 0..<max(4 * pairs.count, 16) {
            var struck = false
            for pair in pairs {
                let sphere = b3Shape_GetSphere(pair.sphere)
                let idA = b3Shape_GetBody(pair.sphere), idB = b3Shape_GetBody(pair.other)
                var a = body(idA), b = body(idB)
                if case .unsupported = a.response { continue }
                if case .unsupported = b.response { continue }
                guard a.movable || b.movable else { continue }
                let t = b3Body_GetTransform(idA)
                let centerA = t.q.simd.act(sphere.center.simd) + t.p.simd
                // The sweep moves the two without turning them: a pair that turns more than the slop
                // within the step is Box3D's. A sphere at its body's center turns in place.
                let turning = simd_length(a.angular) * simd_length(centerA - a.center) + simd_length(b.angular) * reach(of: pair.other, from: b.center)
                guard turning * dt <= slop else { continue }
                guard let (n, onB) = touch(sphere: centerA, radius: sphere.radius, moving: a.linear + simd_cross(a.angular, centerA - a.center),
                                           other: pair.other, moving: b.linear, within: dt, slop: slop) else { continue }
                // Both contact points as they are now, along the normal where the two will touch.
                let point = centerA + n * sphere.radius
                let vA = a.linear + simd_cross(a.angular, point - a.center)
                let vB = b.linear + simd_cross(b.angular, onB - b.center)
                let vn = simd_dot(vB - vA, n)
                // Approaching faster than the threshold.
                guard vn < -threshold else { continue }
                let k = a.k(at: point, n) + b.k(at: onB, n)
                guard k > 0 else { continue }
                let e = combine.callback(b3Shape_GetRestitution(pair.sphere), 0, b3Shape_GetRestitution(pair.other), 0)
                let j = -(1 + e) * vn / k
                a.apply(-j * n, at: point)
                b.apply(j * n, at: onB)
                bodies[idA] = a
                bodies[idB] = b
                struck = true
                if -vn >= hitThreshold, b3Shape_AreHitEventsEnabled(pair.sphere) || b3Shape_AreHitEventsEnabled(pair.other),
                   hits[pair].map({ -vn > $0.speed }) ?? true {
                    hits[pair] = Hit(pair: pair, point: point, normal: n, speed: -vn)
                }
            }
            if !struck { break }
        }
        for (id, b) in bodies where b.changed {
            b3Body_SetLinearVelocity(id, b3Vec3(b.linear))
            b3Body_SetAngularVelocity(id, b3Vec3(b.angular))
            b3Body_SetAwake(id, true)
        }
        return Array(hits.values)
    }

    /// The contact normal, from the sphere towards `other`, and the touched point of `other` at its
    /// present pose, if the sphere comes within `slop` of it within `dt`, the two moving at constant
    /// velocities without turning: conservative advancement, so a near miss isn't an impact.
    static func touch(sphere center: SIMD3<Float>, radius: Float, moving va: SIMD3<Float>,
                      other: b3ShapeId, moving vb: SIMD3<Float>, within dt: Float,
                      slop: Float) -> (normal: SIMD3<Float>, point: SIMD3<Float>)? {
        let motion = va - vb, speed = simd_length(motion)
        var time: Float = 0
        for _ in 0..<32 {
            // In the other shape's frame at its present pose.
            let c = center + motion * time
            let onOther = b3Shape_GetClosestPoint(other, b3Vec3(c)).simd
            let gap = onOther - c
            let distance = simd_length(gap)
            guard distance > 1e-6 else { return nil }
            let separation = distance - radius
            if separation <= slop { return (gap / distance, onOther) }
            guard speed > 0 else { return nil }
            time += separation / speed
            if time > dt { return nil }
        }
        return nil
    }

    /// The farthest a shape reaches from `center`; a sphere's surface turns in place.
    static func reach(of shape: b3ShapeId, from center: SIMD3<Float>) -> Float {
        if b3Shape_GetType(shape) == b3_sphereShape {
            let t = b3Body_GetTransform(b3Shape_GetBody(shape))
            return simd_length(t.q.simd.act(b3Shape_GetSphere(shape).center.simd) + t.p.simd - center)
        }
        let box = b3Shape_GetAABB(shape)
        let mid = (box.lowerBound.simd + box.upperBound.simd) / 2
        return simd_length(mid - center) + simd_length(box.upperBound.simd - box.lowerBound.simd) / 2
    }

    /// The fastest any point of a non-static shape among `shapes` moves, as a bound.
    static func moverSpeed(of shapes: [b3ShapeId]) -> Float {
        var fastest: Float = 0
        for s in shapes where b3Shape_IsValid(s) {
            let body = b3Shape_GetBody(s)
            guard b3Body_GetType(body) != b3_staticBody, b3Body_IsEnabled(body), b3Body_IsAwake(body) else { continue }
            let w = simd_length(b3Body_GetAngularVelocity(body).simd)
            let speed = simd_length(b3Body_GetLinearVelocity(body).simd) + (w > 0 ? w * reach(of: s, from: b3Body_GetWorldCenter(body).simd) : 0)
            fastest = max(fastest, speed)
        }
        return fastest
    }

    /// How a body answers the pass's impulses.
    static func response(of id: b3BodyId) -> Response {
        // A sleeping dynamic body is struck like an awake one; setting its velocity wakes it.
        let mass = b3Body_GetMass(id)
        guard b3Body_GetType(id) == b3_dynamicBody, b3Body_IsEnabled(id), mass > 0 else { return .immovable }
        let w = b3Body_GetWorldInverseRotationalInertia(id)
        let inverseInertia = simd_float3x3(w.cx.simd, w.cy.simd, w.cz.simd)
        // The joints that hold the body; filter joints only keep bodies apart.
        var joints = [b3JointId](repeating: b3JointId(), count: Int(b3Body_GetJointCount(id)))
        let count = Int(b3Body_GetJoints(id, &joints, Int32(joints.count)))
        let holding = joints.prefix(count).filter { b3Joint_IsValid($0) && b3Joint_GetType($0) != b3_filterJoint }
        guard let joint = holding.first else { return .free(inverseMass: 1 / mass, inverseInertia: inverseInertia) }
        // A pendulum: one revolute or spherical joint to a static body.
        let type = b3Joint_GetType(joint)
        let isA = b3Joint_GetBodyA(joint) == id
        guard holding.count == 1, type == b3_revoluteJoint || type == b3_sphericalJoint,
              b3Body_GetType(isA ? b3Joint_GetBodyB(joint) : b3Joint_GetBodyA(joint)) == b3_staticBody
        else { return .unsupported }
        // Free to swing: limits, motors and springs resist what the pass would give, and a joint
        // that can break must feel the impact's load.
        let restrained = type == b3_revoluteJoint
            ? b3RevoluteJoint_IsLimitEnabled(joint) || b3RevoluteJoint_IsMotorEnabled(joint) || b3RevoluteJoint_IsSpringEnabled(joint)
            : b3SphericalJoint_IsConeLimitEnabled(joint) || b3SphericalJoint_IsTwistLimitEnabled(joint)
                || b3SphericalJoint_IsMotorEnabled(joint) || b3SphericalJoint_IsSpringEnabled(joint)
        let breakable = b3Joint_GetForceThreshold(joint) < .greatestFiniteMagnitude
            || b3Joint_GetTorqueThreshold(joint) < .greatestFiniteMagnitude
        guard !restrained, !breakable else { return .unsupported }
        // A pinned body that can't turn can't move.
        guard inverseInertia.determinant != 0 else { return .immovable }
        let frame = isA ? b3Joint_GetLocalFrameA(joint) : b3Joint_GetLocalFrameB(joint)
        let t = b3Body_GetTransform(id)
        let pivot = t.q.simd.act(frame.p.simd) + t.p.simd
        // Inertia about the pivot: about the center of mass, plus the mass at its distance.
        let d = b3Body_GetWorldCenter(id).simd - pivot
        let aboutPivot = inverseInertia.inverse
            + mass * (simd_float3x3(diagonal: SIMD3(repeating: simd_length_squared(d))) - outer(d, d))
        if type == b3_sphericalJoint { return .pivoted(pivot: pivot, inverseInertia: aboutPivot.inverse) }
        // A hinge turns only about its axis, its frame's z.
        let axis = t.q.simd.act(frame.q.simd.act(SIMD3(0, 0, 1)))
        let along = simd_dot(axis, aboutPivot * axis)
        guard along > 0 else { return .immovable }
        return .pivoted(pivot: pivot, inverseInertia: outer(axis, axis) * (1 / along))
    }

    private static func outer(_ a: SIMD3<Float>, _ b: SIMD3<Float>) -> simd_float3x3 {
        simd_float3x3(columns: (a * b.x, a * b.y, a * b.z))
    }

    /// Box3D's filter rule: group index first, then categories and masks both ways. Never sensors,
    /// nor bodies a joint keeps apart, nor one-way shapes, whose rule needs Box3D's contact.
    private static func collides(_ a: b3ShapeId, _ b: b3ShapeId, oneWay: OneWayShapes?) -> Bool {
        if b3Shape_IsSensor(a) || b3Shape_IsSensor(b) { return false }
        if let oneWay, oneWay.directions[.init(a)] != nil || oneWay.directions[.init(b)] != nil { return false }
        let fa = b3Shape_GetFilter(a), fb = b3Shape_GetFilter(b)
        if fa.groupIndex == fb.groupIndex, fa.groupIndex != 0 {
            guard fa.groupIndex > 0 else { return false }
        } else if (fa.maskBits & fb.categoryBits) == 0 || (fa.categoryBits & fb.maskBits) == 0 {
            return false
        }
        let bodyA = b3Shape_GetBody(a), bodyB = b3Shape_GetBody(b)
        var joints = [b3JointId](repeating: b3JointId(), count: Int(b3Body_GetJointCount(bodyA)))
        let count = Int(b3Body_GetJoints(bodyA, &joints, Int32(joints.count)))
        return !joints.prefix(count).contains { j in
            b3Joint_IsValid(j) && !b3Joint_GetCollideConnected(j) && (b3Joint_GetBodyA(j) == bodyB || b3Joint_GetBodyB(j) == bodyB)
        }
    }
}

extension B3World {
    /// Resolves the coming step's impacts and keeps those that are hits, for `collectEvents`.
    func resolveImpacts(dt: Float) {
        let shapes = records.values.filter(\.isEnabled).flatMap(\.shapeIds)
        impactHits = SequentialImpacts.resolve(
            world: id, spheres: SequentialImpacts.spheres(of: shapes),
            dt: dt, threshold: b3World_GetRestitutionThreshold(id), combine: settings.restitutionCombine,
            slop: 0.005 * b3GetLengthUnitsPerMeter(), oneWay: oneWayRecords.isEmpty ? nil : oneWayShapes,
            moverSpeed: SequentialImpacts.moverSpeed(of: shapes), hitThreshold: b3World_GetHitEventThreshold(id))
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

/// The forces applied for the coming fixed step, summed per body and wind per shape. Box3D clears
/// applied forces after each of its steps; when sequential impacts step the substeps one by one,
/// these are applied again before each later one.
struct PendingForces {
    struct Load { var force = SIMD3<Float>.zero, torque = SIMD3<Float>.zero }
    struct Wind: Hashable { let shape: b3ShapeId; let wind: SIMD3<Float>; let drag: Float, lift: Float, maxSpeed: Float }
    var loads: [b3BodyId: Load] = [:]
    var winds: [Wind: Int] = [:]

    var isEmpty: Bool { loads.isEmpty && winds.isEmpty }

    mutating func add(force: SIMD3<Float>, at point: SIMD3<Float>?, to body: b3BodyId) {
        loads[body, default: .init()].force += force
        if let point { loads[body, default: .init()].torque += simd_cross(point - b3Body_GetWorldCenter(body).simd, force) }
    }
    mutating func add(torque: SIMD3<Float>, to body: b3BodyId) { loads[body, default: .init()].torque += torque }
    mutating func add(_ wind: Wind) { winds[wind, default: 0] += 1 }

    func apply() {
        for (body, load) in loads where b3Body_IsValid(body) {
            b3Body_ApplyForceToCenter(body, b3Vec3(load.force), true)
            b3Body_ApplyTorque(body, b3Vec3(load.torque), true)
        }
        for (w, count) in winds where b3Shape_IsValid(w.shape) {
            for _ in 0..<count { b3Shape_ApplyWind(w.shape, b3Vec3(w.wind), w.drag, w.lift, w.maxSpeed, true) }
        }
    }

    /// The same forces on a copy of the world, such as a forecast's.
    func copied(to world: UInt16) -> PendingForces {
        var copy = PendingForces()
        for (body, load) in loads { var b = body; b.world0 = world; copy.loads[b] = load }
        for (w, count) in winds {
            var s = w.shape
            s.world0 = world
            copy.winds[Wind(shape: s, wind: w.wind, drag: w.drag, lift: w.lift, maxSpeed: w.maxSpeed)] = count
        }
        return copy
    }
}

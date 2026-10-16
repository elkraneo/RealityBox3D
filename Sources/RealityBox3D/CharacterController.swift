import CBox3D
import RealityKit
import simd

/// Mirrors RealityKit's `CharacterControllerComponent`: a capsule moved by code, not simulated,
/// that slides along walls and floors. Box3D's geometric mover does the work (`b3World_CastMover`,
/// `b3World_CollideMover`, `b3SolvePlanes`), in the loop of upstream's sample mover.
public struct B3CharacterControllerComponent: Component, Equatable, Sendable {
    public static let defaultSlopeLimit: Float = .pi / 4
    public var radius: Float
    /// Total height, caps included. The capsule is centered on the entity, along `upVector`.
    public var height: Float
    /// Kept for source compatibility; Box3D's mover handles contact without a skin.
    public var skinWidth: Float
    /// Surfaces steeper than this, in radians, do not count as ground.
    public var slopeLimit: Float
    /// Kept for source compatibility, not applied: Box3D's mover has no step-up. The capsule rolls
    /// up steps lower than its radius, slowly, since their edges are too steep to count as ground.
    public var stepLimit: Float
    public var upVector: SIMD3<Float>
    public var collisionFilter: CollisionFilter
    /// Box3D extension: the character pushes the dynamic bodies it walks into, as upstream's mover.
    public var pushesBodies: Bool = true

    public init(radius: Float = 0.3, height: Float = 1.6, skinWidth: Float = 0.01, slopeLimit: Float = defaultSlopeLimit,
                stepLimit: Float = 0.3, upVector: SIMD3<Float> = [0, 1, 0], collisionFilter: CollisionFilter = .default) {
        self.radius = radius; self.height = height; self.skinWidth = skinWidth; self.slopeLimit = slopeLimit
        self.stepLimit = stepLimit; self.upVector = simd_normalize(upVector); self.collisionFilter = collisionFilter
    }
}

/// Mirrors `CharacterControllerStateComponent`: written after every move.
public struct B3CharacterControllerStateComponent: Component, Equatable, Sendable {
    /// Root-space velocity of the last move.
    public var velocity: SIMD3<Float>
    public var isOnGround: Bool
    /// Box3D extension: the requested velocity with its parts into the touched shapes removed
    /// (`b3ClipVector`), skipping obstacles that do not clip. Unlike `velocity` it never picks up
    /// the push out of a soft obstacle, so a game loop can carry it to the next move, as upstream's mover.
    public var clippedVelocity: SIMD3<Float> = .zero
    public init(velocity: SIMD3<Float> = .zero, isOnGround: Bool = false) { self.velocity = velocity; self.isOnGround = isOnGround }
}

/// Box3D extension, upstream's `MoverShapeUserData`: how character controllers treat an entity's
/// shapes. Without it a shape is solid, clips velocity and, on a dynamic body, gets pushed.
public struct B3CharacterObstacleComponent: Component, Equatable, Sendable {
    /// The most a character is pushed out of these shapes in one move, in meters. A small limit
    /// makes a soft obstacle the character leans into and slides off over a few frames.
    public var pushLimit: Float
    /// Whether these shapes clip the character's `clippedVelocity`.
    public var clipsVelocity: Bool
    /// Whether characters push these shapes' dynamic bodies.
    public var isPushable: Bool
    /// Whether these shapes stop a character's sweep. With false only the push limit holds the
    /// character back, so it can step into them, as upstream's friendly capsule.
    public var stopsSweep: Bool

    public init(pushLimit: Float = .greatestFiniteMagnitude, clipsVelocity: Bool = true, isPushable: Bool = true,
                stopsSweep: Bool = true) {
        self.pushLimit = pushLimit; self.clipsVelocity = clipsVelocity; self.isPushable = isPushable; self.stopsSweep = stopsSweep
    }
}

/// Mirrors `CharacterControllerComponent.Collision`.
public struct B3CharacterCollision {
    public let characterEntity: Entity
    public let hitEntity: Entity
    public let hitPosition: SIMD3<Float>
    public let hitNormal: SIMD3<Float>
    public let moveDirection: SIMD3<Float>
    public let moveDistance: Float
}

extension B3BodyProxy {
    /// Mirrors `Entity.moveCharacter(by:deltaTime:relativeTo:collisionHandler:)`: moves the
    /// entity's character capsule as far as the world allows toward `moveDelta`, sliding along what
    /// it touches. Gravity is the caller's, as in RealityKit: include it in `moveDelta`.
    public func moveCharacter(by moveDelta: SIMD3<Float>, deltaTime: Float, relativeTo referenceEntity: Entity?,
                              collisionHandler: ((B3CharacterCollision) -> Void)? = nil) {
        moveCharacter(by: moveDelta, correction: .zero, deltaTime: deltaTime, relativeTo: referenceEntity,
                      collisionHandler: collisionHandler)
    }

    /// Box3D extension: as `moveCharacter(by:deltaTime:relativeTo:)`, plus a positional
    /// `correction` moved in the same sweep that is not motion: it neither pushes bodies nor
    /// enters `clippedVelocity`. Upstream's mover floats its capsule with a spring this way.
    public func moveCharacter(by moveDelta: SIMD3<Float>, correction: SIMD3<Float>, deltaTime: Float,
                              relativeTo referenceEntity: Entity?, collisionHandler: ((B3CharacterCollision) -> Void)? = nil) {
        guard let controller = entity.components[B3CharacterControllerComponent.self],
              let world = RealityBox3D.world(for: entity) else { return }
        let root = world.root
        // The move in the root's space
        let m = referenceEntity.map { $0.transformMatrix(relativeTo: root) }
            ?? (root.map { simd_inverse($0.transformMatrix(relativeTo: nil)) } ?? matrix_identity_float4x4)
        let w = m * SIMD4(moveDelta, 0), k = m * SIMD4(correction, 0)
        let delta = SIMD3(w.x, w.y, w.z)

        let start = entity.position(relativeTo: root)
        let result = world.moveCapsule(from: start, by: delta, correction: SIMD3(k.x, k.y, k.z), controller: controller,
                                       deltaTime: deltaTime)
        entity.setPosition(result.position, relativeTo: root)
        let velocity = deltaTime > 0 ? (result.position - start) / deltaTime : .zero
        var state = B3CharacterControllerStateComponent(velocity: velocity, isOnGround: result.isOnGround)
        state.clippedVelocity = result.clippedVelocity
        entity.components.set(state)

        guard let collisionHandler else { return }
        let moved = result.position - start
        let distance = simd_length(moved)
        for hit in result.hits {
            collisionHandler(B3CharacterCollision(characterEntity: entity, hitEntity: hit.entity, hitPosition: hit.point,
                                                  hitNormal: hit.normal, moveDirection: distance > 0 ? moved / distance : .zero,
                                                  moveDistance: distance))
        }
    }
}

extension B3World {
    struct MoveResult {
        var position: SIMD3<Float>
        var isOnGround: Bool
        var clippedVelocity: SIMD3<Float>
        var hits: [(entity: Entity, point: SIMD3<Float>, normal: SIMD3<Float>)]
    }

    /// Upstream's mover loop: gather contact planes, solve for the closest allowed delta, sweep it,
    /// repeat until the move settles. Then push touched dynamic bodies out of the way.
    func moveCapsule(from start: SIMD3<Float>, by delta: SIMD3<Float>, correction: SIMD3<Float> = .zero,
                     controller c: B3CharacterControllerComponent,
                     deltaTime: Float) -> MoveResult {
        let half = max(c.height / 2 - c.radius, 0)
        var capsule = b3Capsule(center1: b3Vec3(-half * c.upVector), center2: b3Vec3(half * c.upVector), radius: c.radius)
        var filter = b3DefaultQueryFilter()
        filter.categoryBits = UInt64(c.collisionFilter.group.rawValue)
        filter.maskBits = UInt64(c.collisionFilter.mask.rawValue)

        final class Planes {
            var planes: [b3CollisionPlane] = []
            var shapes: [b3ShapeId] = []
            var points: [SIMD3<Float>] = []
            var origin: SIMD3<Float> = .zero
            unowned let world: B3World
            init(world: B3World) { self.world = world }
        }
        let gathered = Planes(world: self)
        let context = Unmanaged.passUnretained(gathered).toOpaque()
        var position = start
        let target = start + delta + correction
        for _ in 0..<5 {
            gathered.planes.removeAll(keepingCapacity: true)
            gathered.shapes.removeAll(keepingCapacity: true)
            gathered.points.removeAll(keepingCapacity: true)
            gathered.origin = position
            b3World_CollideMover(id, b3Vec3(position), &capsule, filter, { shape, results, count, ctx in
                let g = Unmanaged<Planes>.fromOpaque(ctx!).takeUnretainedValue()
                for i in 0..<Int(count) where g.planes.count < 16 {
                    let r = results![i]
                    g.planes.append(b3CollisionPlane(plane: r.plane, pushLimit: .greatestFiniteMagnitude, push: 0, clipVelocity: true))
                    g.shapes.append(shape)
                    g.points.append(r.point.simd + g.origin) // relative to the query origin
                }
                return true
            }, context)
            // Each obstacle's own push limit and clipping, upstream's per-shape user data
            for i in gathered.planes.indices {
                guard let rules = obstacle(of: gathered.shapes[i]) else { continue }
                gathered.planes[i].pushLimit = rules.pushLimit
                gathered.planes[i].clipVelocity = rules.clipsVelocity
            }
            let solved = gathered.planes.withUnsafeMutableBufferPointer { buffer in
                b3SolvePlanes(b3Vec3(target - position), buffer.baseAddress, Int32(buffer.count))
            }
            var step = solved.delta.simd
            // The cast runs on this thread, inside this call
            let fraction = b3World_CastMover(id, b3Vec3(position), &capsule, b3Vec3(step), filter, { shape, ctx in
                let g = Unmanaged<Planes>.fromOpaque(ctx!).takeUnretainedValue()
                return MainActor.assumeIsolated { g.world.obstacle(of: shape)?.stopsSweep ?? true }
            }, context)
            step *= fraction
            position += step
            if simd_length_squared(step) < 1e-4 * c.radius * c.radius { break }
        }

        let cosSlope = cos(c.slopeLimit)
        var onGround = false
        var hits: [(Entity, SIMD3<Float>, SIMD3<Float>)] = []
        let velocity = deltaTime > 0 ? delta / deltaTime : .zero
        for (i, plane) in gathered.planes.enumerated() {
            let normal = plane.plane.normal.simd
            if simd_dot(normal, c.upVector) >= cosSlope { onGround = true }
            let point = gathered.points[i]
            let shape = gathered.shapes[i]
            if let e = entity(for: shape) { hits.append((e, point, normal)) }
            // Upstream's push: an impulse that stops the body moving into the character.
            guard c.pushesBodies, b3Shape_IsValid(shape), obstacle(of: shape)?.isPushable ?? true else { continue }
            let body = b3Shape_GetBody(shape)
            guard b3Body_GetType(body) == b3_dynamicBody else { continue }
            let n = -normal
            let invMass = b3Body_GetInverseMass(body)
            let invI = b3Body_GetWorldInverseRotationalInertia(body)
            let rB = point - b3Body_GetWorldCenter(body).simd
            let rn = simd_cross(rB, n)
            let I = simd_float3x3(invI.cx.simd, invI.cy.simd, invI.cz.simd)
            let k = invMass + simd_dot(rn, I * rn)
            guard k > 0 else { continue }
            let vB = b3Body_GetLinearVelocity(body).simd + simd_cross(b3Body_GetAngularVelocity(body).simd, rB)
            let impulse = max(-(simd_dot(vB - velocity, n)) / k, 0)
            if impulse > 0 { b3Body_ApplyLinearImpulse(body, b3Vec3(impulse * n), b3Vec3(point), true) }
        }
        let clipped = gathered.planes.withUnsafeBufferPointer { b3ClipVector(b3Vec3(velocity), $0.baseAddress, Int32($0.count)) }
        return MoveResult(position: position, isOnGround: onGround, clippedVelocity: clipped.simd, hits: hits)
    }

    func obstacle(of shape: b3ShapeId) -> B3CharacterObstacleComponent? {
        entity(for: shape)?.components[B3CharacterObstacleComponent.self]
    }
}

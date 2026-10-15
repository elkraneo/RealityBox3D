import CBox3D
import RealityKit
import simd

// Joints shaped like RealityKit's: two `GeometricPin`s, an optional limit, and `addToSimulation()`,
// which appends the joint to the `B3PhysicsJointsComponent` of the pins' simulation root.
// Axes follow RealityKit: a revolute joint turns, and a prismatic joint slides, along the pins'
// x-axis; a spherical joint's cone opens around pin0's x-axis. A pin on an entity without a body
// anchors the joint to the simulation root.

/// Mirrors RealityKit's `PhysicsJoint`.
public protocol B3PhysicsJoint: Equatable {
    var pin0: GeometricPin { get set }
    var pin1: GeometricPin { get set }
    var checksForInternalCollisions: Bool { get }
    /// Box3D extension: stiffness of the joint's own constraint in hertz. Nil keeps Box3D's default.
    var constraintHertz: Float? { get }
    /// Box3D extension: the force or torque at which the joint breaks. Nil never breaks.
    var breakLimit: B3JointBreakLimit? { get }
}

extension B3PhysicsJoint {
    public var constraintHertz: Float? { nil }
    public var breakLimit: B3JointBreakLimit? { nil }

    /// Mirrors RealityKit's `addToSimulation()`: appends the joint to the joints component of the
    /// simulation root above `pin0`'s entity, and returns that root.
    @MainActor @discardableResult
    public func addToSimulation() throws -> Entity {
        guard let entity = pin0.entity ?? pin1.entity,
              let root = B3PhysicsSimulationComponent.nearestSimulationEntity(for: entity) else {
            throw B3JointError.noSimulationRoot
        }
        var component = root.components[B3PhysicsJointsComponent.self] ?? B3PhysicsJointsComponent()
        component.joints.append(self)
        root.components.set(component)
        return root
    }

    func isEqual(to other: any B3PhysicsJoint) -> Bool {
        guard let other = other as? Self else { return false }
        return self == other
    }
}

public enum B3JointError: Error {
    /// The pins' entities are not under an entity with a `B3PhysicsSimulationComponent`.
    case noSimulationRoot
}

/// Box3D extension: the load at which a joint breaks. When the force or the torque its constraint
/// applies exceeds a limit, the world removes the joint from the root's `B3PhysicsJointsComponent`
/// and sends `B3JointEvents.Broke`. Newtons and newton-meters when the simulation root is in
/// meters; in general, the root's units, like forces given relative to the root.
public struct B3JointBreakLimit: Equatable, Sendable {
    public var force: Float?
    public var torque: Float?
    public init(force: Float? = nil, torque: Float? = nil) { self.force = force; self.torque = torque }
}

/// Box3D extension: drives a joint at a speed, up to a maximum torque or force. A motor with speed
/// zero acts as joint friction.
public struct B3JointMotor: Equatable, Sendable {
    /// Radians per second for rotation, meters per second for translation.
    public var speed: Float
    /// Newton-meters for rotation, newtons for translation.
    public var maxForce: Float
    public init(speed: Float = 0, maxForce: Float) { self.speed = speed; self.maxForce = maxForce }
}

/// Box3D extension: a soft spring toward a target angle, translation or length.
public struct B3JointSpring: Equatable, Sendable {
    public var hertz: Float
    public var dampingRatio: Float
    /// Radians for a revolute joint, meters for a prismatic or distance joint.
    public var target: Float
    public init(hertz: Float, dampingRatio: Float = 0.7, target: Float = 0) {
        self.hertz = hertz; self.dampingRatio = dampingRatio; self.target = target
    }
}

/// Mirrors `PhysicsRevoluteJoint`: one rotational degree of freedom around the pins' x-axis.
public struct B3PhysicsRevoluteJoint: B3PhysicsJoint {
    public var pin0: GeometricPin
    public var pin1: GeometricPin
    /// Radians, within about ±0.99π.
    public var angularLimit: ClosedRange<Float>?
    public var checksForInternalCollisions: Bool
    public var motor: B3JointMotor?
    public var spring: B3JointSpring?
    public var constraintHertz: Float?
    public var breakLimit: B3JointBreakLimit?

    public init(pin0: GeometricPin, pin1: GeometricPin, angularLimit: ClosedRange<Float>? = nil,
                checksForInternalCollisions: Bool = false, motor: B3JointMotor? = nil, spring: B3JointSpring? = nil,
                constraintHertz: Float? = nil) {
        self.pin0 = pin0; self.pin1 = pin1; self.angularLimit = angularLimit
        self.checksForInternalCollisions = checksForInternalCollisions
        self.motor = motor; self.spring = spring; self.constraintHertz = constraintHertz
    }
}

/// Mirrors `PhysicsPrismaticJoint`: one linear degree of freedom along the pins' x-axis.
public struct B3PhysicsPrismaticJoint: B3PhysicsJoint {
    public var pin0: GeometricPin
    public var pin1: GeometricPin
    public var linearLimit: ClosedRange<Float>?
    public var checksForInternalCollisions: Bool
    public var motor: B3JointMotor?
    public var spring: B3JointSpring?
    public var constraintHertz: Float?
    public var breakLimit: B3JointBreakLimit?

    public init(pin0: GeometricPin, pin1: GeometricPin, linearLimit: ClosedRange<Float>? = nil,
                checksForInternalCollisions: Bool = false, motor: B3JointMotor? = nil, spring: B3JointSpring? = nil,
                constraintHertz: Float? = nil) {
        self.pin0 = pin0; self.pin1 = pin1; self.linearLimit = linearLimit
        self.checksForInternalCollisions = checksForInternalCollisions
        self.motor = motor; self.spring = spring; self.constraintHertz = constraintHertz
    }
}

/// Mirrors `PhysicsSphericalJoint`: free rotation about a shared point.
public struct B3PhysicsSphericalJoint: B3PhysicsJoint {
    public var pin0: GeometricPin
    public var pin1: GeometricPin
    /// RealityKit's elliptical cone around pin0's x-axis, in radians. Box3D's cone is circular,
    /// so it opens to the larger of the two angles.
    public var angularLimitInYZ: (Float, Float)?
    /// Box3D extension: a limit on twist around the x-axis, which RealityKit never limits.
    public var twistLimit: ClosedRange<Float>?
    public var checksForInternalCollisions: Bool
    /// Box3D extension: a spring that pulls the pins' frames into alignment.
    public var spring: B3JointSpring?
    /// Box3D extension: a motor at rest, with this maximum torque, acting as joint friction.
    public var friction: Float?
    public var constraintHertz: Float?
    public var breakLimit: B3JointBreakLimit?

    public init(pin0: GeometricPin, pin1: GeometricPin, angularLimitInYZ: (Float, Float)? = nil,
                checksForInternalCollisions: Bool = false, twistLimit: ClosedRange<Float>? = nil,
                spring: B3JointSpring? = nil, friction: Float? = nil, constraintHertz: Float? = nil) {
        self.pin0 = pin0; self.pin1 = pin1; self.angularLimitInYZ = angularLimitInYZ
        self.checksForInternalCollisions = checksForInternalCollisions; self.twistLimit = twistLimit
        self.spring = spring; self.friction = friction; self.constraintHertz = constraintHertz
    }

    public static func == (a: Self, b: Self) -> Bool {
        a.pin0 == b.pin0 && a.pin1 == b.pin1 && a.angularLimitInYZ?.0 == b.angularLimitInYZ?.0
            && a.angularLimitInYZ?.1 == b.angularLimitInYZ?.1 && a.twistLimit == b.twistLimit
            && a.checksForInternalCollisions == b.checksForInternalCollisions && a.spring == b.spring
            && a.friction == b.friction && a.constraintHertz == b.constraintHertz && a.breakLimit == b.breakLimit
    }
}

/// Mirrors `PhysicsFixedJoint`: no relative motion. Box3D's weld joint, which can soften.
public struct B3PhysicsFixedJoint: B3PhysicsJoint {
    public var pin0: GeometricPin
    public var pin1: GeometricPin
    public let checksForInternalCollisions = false
    /// Box3D extension: zero hertz is rigid; above zero the weld bends like a spring.
    public var linearSpring: B3JointSpring?
    public var angularSpring: B3JointSpring?
    public var constraintHertz: Float?
    public var breakLimit: B3JointBreakLimit?

    public init(pin0: GeometricPin, pin1: GeometricPin, linearSpring: B3JointSpring? = nil,
                angularSpring: B3JointSpring? = nil, constraintHertz: Float? = nil) {
        self.pin0 = pin0; self.pin1 = pin1
        self.linearSpring = linearSpring; self.angularSpring = angularSpring; self.constraintHertz = constraintHertz
    }
}

/// Mirrors `PhysicsDistanceJoint`: keeps the pins within a range of distances, like a rope or rod.
public struct B3PhysicsDistanceJoint: B3PhysicsJoint {
    public var pin0: GeometricPin
    public var pin1: GeometricPin
    /// A single value makes a rigid rod.
    public var distanceLimit: ClosedRange<Float>
    public var checksForInternalCollisions: Bool
    /// Box3D extension: a spring toward `target` meters, within the limit.
    public var spring: B3JointSpring?
    public var constraintHertz: Float?
    public var breakLimit: B3JointBreakLimit?

    public init(pin0: GeometricPin, pin1: GeometricPin, distanceLimit: ClosedRange<Float>,
                checksForInternalCollisions: Bool = false, spring: B3JointSpring? = nil, constraintHertz: Float? = nil) {
        self.pin0 = pin0; self.pin1 = pin1; self.distanceLimit = distanceLimit
        self.checksForInternalCollisions = checksForInternalCollisions
        self.spring = spring; self.constraintHertz = constraintHertz
    }
}

/// Box3D extension: a wheel on a suspension, for vehicles. pin0 is on the chassis, pin1 on the
/// wheel. The suspension slides, and the wheel steers, along pin0's x-axis; the wheel spins around
/// pin1's x-axis.
public struct B3PhysicsWheelJoint: B3PhysicsJoint {
    public struct Steering: Equatable, Sendable {
        public var spring: B3JointSpring
        public var maxTorque: Float
        public var limit: ClosedRange<Float>?
        /// Target steering angle in radians; also settable at runtime with `B3World.setSteeringAngle`.
        public var angle: Float
        public init(spring: B3JointSpring, maxTorque: Float, limit: ClosedRange<Float>? = nil, angle: Float = 0) {
            self.spring = spring; self.maxTorque = maxTorque; self.limit = limit; self.angle = angle
        }
    }
    public var pin0: GeometricPin
    public var pin1: GeometricPin
    public var checksForInternalCollisions: Bool
    public var suspension: B3JointSpring?
    public var suspensionLimit: ClosedRange<Float>?
    public var spinMotor: B3JointMotor?
    public var steering: Steering?
    public var constraintHertz: Float?
    public var breakLimit: B3JointBreakLimit?

    public init(pin0: GeometricPin, pin1: GeometricPin, suspension: B3JointSpring? = nil, suspensionLimit: ClosedRange<Float>? = nil,
                spinMotor: B3JointMotor? = nil, steering: Steering? = nil, checksForInternalCollisions: Bool = false,
                constraintHertz: Float? = nil) {
        self.pin0 = pin0; self.pin1 = pin1; self.suspension = suspension; self.suspensionLimit = suspensionLimit
        self.spinMotor = spinMotor; self.steering = steering
        self.checksForInternalCollisions = checksForInternalCollisions; self.constraintHertz = constraintHertz
    }
}

/// Box3D extension: a spring that keeps pin1's x-axis parallel to pin0's, for example to keep a
/// vehicle upright. Nothing else is constrained.
public struct B3PhysicsParallelJoint: B3PhysicsJoint {
    public var pin0: GeometricPin
    public var pin1: GeometricPin
    public var checksForInternalCollisions: Bool
    public var spring: B3JointSpring
    /// Newton-meters; nil for Box3D's default.
    public var maxTorque: Float?
    public var constraintHertz: Float? { nil }
    public var breakLimit: B3JointBreakLimit?

    public init(pin0: GeometricPin, pin1: GeometricPin, spring: B3JointSpring, maxTorque: Float? = nil,
                checksForInternalCollisions: Bool = false) {
        self.pin0 = pin0; self.pin1 = pin1; self.spring = spring; self.maxTorque = maxTorque
        self.checksForInternalCollisions = checksForInternalCollisions
    }
}

/// Box3D extension: drives pin1's frame toward pin0's with velocity motors and springs, like a
/// soft weld you can steer. Useful for grabbing bodies or powered rigs.
public struct B3PhysicsMotorJoint: B3PhysicsJoint {
    public var pin0: GeometricPin
    public var pin1: GeometricPin
    public var checksForInternalCollisions: Bool
    public var linearVelocity: SIMD3<Float>
    public var maxForce: Float
    public var angularVelocity: SIMD3<Float>
    public var maxTorque: Float
    public var linearSpring: B3JointSpring?
    public var maxSpringForce: Float
    public var angularSpring: B3JointSpring?
    public var maxSpringTorque: Float
    public var constraintHertz: Float? { nil }
    public var breakLimit: B3JointBreakLimit?

    public init(pin0: GeometricPin, pin1: GeometricPin, linearVelocity: SIMD3<Float> = .zero, maxForce: Float = 0,
                angularVelocity: SIMD3<Float> = .zero, maxTorque: Float = 0, linearSpring: B3JointSpring? = nil,
                maxSpringForce: Float = .greatestFiniteMagnitude, angularSpring: B3JointSpring? = nil,
                maxSpringTorque: Float = .greatestFiniteMagnitude, checksForInternalCollisions: Bool = true) {
        self.pin0 = pin0; self.pin1 = pin1; self.linearVelocity = linearVelocity; self.maxForce = maxForce
        self.angularVelocity = angularVelocity; self.maxTorque = maxTorque; self.linearSpring = linearSpring
        self.maxSpringForce = maxSpringForce; self.angularSpring = angularSpring; self.maxSpringTorque = maxSpringTorque
        self.checksForInternalCollisions = checksForInternalCollisions
    }
}

/// Box3D extension: no constraint, only stops two bodies from colliding with each other. The pins'
/// positions do not matter.
public struct B3PhysicsFilterJoint: B3PhysicsJoint {
    public var pin0: GeometricPin
    public var pin1: GeometricPin
    public var checksForInternalCollisions: Bool { false }
    public init(pin0: GeometricPin, pin1: GeometricPin) { self.pin0 = pin0; self.pin1 = pin1 }
}

/// Mirrors `PhysicsJointsComponent`: the joints of a simulation, set on its root entity.
public struct B3PhysicsJointsComponent: Component, Equatable {
    public var joints: [any B3PhysicsJoint]
    public init(joints: [any B3PhysicsJoint] = []) { self.joints = joints }

    public static func == (a: Self, b: Self) -> Bool {
        a.joints.count == b.joints.count && zip(a.joints, b.joints).allSatisfy { $0.isEqual(to: $1) }
    }
}

// MARK: - Building joints in a world

/// A non-body entity that joints pin to, and the kinematic body standing for it.
@MainActor
final class JointAnchor {
    let entity: Entity
    let bodyId: b3BodyId
    var pose: Pose
    var target: Pose?
    var scale: SIMD3<Float> { pose.scale }
    init(entity: Entity, bodyId: b3BodyId, pose: Pose) { self.entity = entity; self.bodyId = bodyId; self.pose = pose }
}

/// One joint of the world: its Box3D id and the pins it was built from.
struct JointSlot {
    var id: b3JointId?
    var resolved: ResolvedJoint?
}

/// A joint's pins resolved to Box3D bodies and local frames.
struct ResolvedJoint {
    var bodyA: b3BodyId, frameA: b3Transform
    var bodyB: b3BodyId, frameB: b3Transform
    /// The body records behind the pins, with the scale each frame was computed at.
    var bodies: [Entity.ID: (record: BodyRecord, scale: SIMD3<Float>)]
    /// The anchors behind the pins on non-body entities, with the scale each frame was computed at.
    var anchors: [Entity.ID: (anchor: JointAnchor, scale: SIMD3<Float>)] = [:]
    /// A pin on a non-body anchor or on a skeletal joint: its frame can move without any edit,
    /// so it is resolved again every frame.
    var tracksPins: Bool

    func isClose(to o: ResolvedJoint) -> Bool {
        func close(_ a: b3Transform, _ b: b3Transform) -> Bool {
            simd_distance_squared(a.p.simd, b.p.simd) < 1e-10 && abs(simd_dot(a.q.simd.vector, b.q.simd.vector)) > 1 - 1e-7
        }
        return bodyA == o.bodyA && bodyB == o.bodyB && close(frameA, o.frameA) && close(frameB, o.frameB)
    }
}

extension B3World {
    /// Brings the world's joints in line with the root's joints component, joint by joint: an
    /// unchanged joint is left alone, a changed parameter goes through Box3D's setters, and only a
    /// joint whose pins, type or bodies changed is rebuilt. Called every frame after the bodies are in.
    func syncJoints() {
        let joints = root?.components[B3PhysicsJointsComponent.self]?.joints ?? []
        let old = appliedJoints?.joints ?? []
        var slots = jointSlots
        slots.reserveCapacity(joints.count)
        for i in joints.count..<max(slots.count, joints.count) {
            if let id = slots[i].id, b3Joint_IsValid(id) { b3DestroyJoint(id, true) }
        }
        if slots.count > joints.count { slots.removeLast(slots.count - joints.count) }
        for (i, joint) in joints.enumerated() {
            let previous = i < old.count ? old[i] : nil
            let slot = i < slots.count ? slots[i] : JointSlot()
            let valid = slot.id.map(b3Joint_IsValid) ?? false
            // The pins' bodies and frames: reused while nothing they depend on changed.
            var resolved = slot.resolved
            let bodiesSame = (slot.resolved?.bodies.allSatisfy { key, value in
                records[key] === value.record && value.record.scale == value.scale
            } ?? false) && (slot.resolved?.anchors.allSatisfy { key, value in
                // Same anchor, same scale, and its entity has not become a simulated body.
                anchors[key] === value.anchor && value.anchor.pose.scale.isClose(to: value.scale) && records[key] == nil
            } ?? false)
            let pinsSame = previous.map { joint.pin0 == $0.pin0 && joint.pin1 == $0.pin1 } ?? false
            if !bodiesSame || !pinsSame || slot.resolved?.tracksPins == true || resolved == nil {
                resolved = resolve(joint)
            }
            if let previous, valid, bodiesSame, let resolved, let was = slot.resolved, resolved.isClose(to: was) {
                // Keep what was learned on resolving, such as a skeleton gone so pins need no tracking.
                if resolved.tracksPins != was.tracksPins { slots[i].resolved?.tracksPins = resolved.tracksPins }
                if joint.isEqual(to: previous) { continue }
                if samePinsAndType(joint, previous), let id = slot.id, update(id, to: joint) {
                    b3Joint_WakeBodies(id)
                    continue
                }
            }
            // New, structurally changed, moved, or its bodies changed: rebuild this one.
            if let id = slot.id, b3Joint_IsValid(id) { b3DestroyJoint(id, true) }
            let id = resolved.flatMap { makeJoint(joint, $0) }.flatMap { b3Joint_IsValid($0) ? $0 : nil }
            let fresh = JointSlot(id: id, resolved: resolved)
            if i < slots.count { slots[i] = fresh } else { slots.append(fresh) }
            if let id { b3Joint_WakeBodies(id) }
        }
        jointSlots = slots
        appliedJoints = B3PhysicsJointsComponent(joints: joints)
        if !anchors.isEmpty { pruneAnchors() }
    }

    /// The live Box3D joints, for tests and diagnostics.
    var jointIds: [b3JointId] { jointSlots.compactMap { $0.id }.filter(b3Joint_IsValid) }

    private func samePinsAndType(_ a: any B3PhysicsJoint, _ b: any B3PhysicsJoint) -> Bool {
        type(of: a) == type(of: b) && a.pin0 == b.pin0 && a.pin1 == b.pin1
            && a.checksForInternalCollisions == b.checksForInternalCollisions && a.constraintHertz == b.constraintHertz
    }

    /// Applies a joint's parameters through Box3D's setters. False if a change needs a rebuild.
    private func update(_ id: b3JointId, to joint: any B3PhysicsJoint) -> Bool {
        b3Joint_SetForceThreshold(id, joint.breakLimit?.force ?? .greatestFiniteMagnitude)
        b3Joint_SetTorqueThreshold(id, joint.breakLimit?.torque ?? .greatestFiniteMagnitude)
        switch joint {
        case let j as B3PhysicsRevoluteJoint:
            b3RevoluteJoint_EnableLimit(id, j.angularLimit != nil)
            if let l = j.angularLimit { b3RevoluteJoint_SetLimits(id, l.lowerBound, l.upperBound) }
            b3RevoluteJoint_EnableMotor(id, j.motor != nil)
            if let m = j.motor { b3RevoluteJoint_SetMotorSpeed(id, m.speed); b3RevoluteJoint_SetMaxMotorTorque(id, m.maxForce) }
            b3RevoluteJoint_EnableSpring(id, j.spring != nil)
            if let sp = j.spring {
                b3RevoluteJoint_SetSpringHertz(id, sp.hertz); b3RevoluteJoint_SetSpringDampingRatio(id, sp.dampingRatio)
                b3RevoluteJoint_SetTargetAngle(id, sp.target)
            }
        case let j as B3PhysicsPrismaticJoint:
            b3PrismaticJoint_EnableLimit(id, j.linearLimit != nil)
            if let l = j.linearLimit { b3PrismaticJoint_SetLimits(id, l.lowerBound, l.upperBound) }
            b3PrismaticJoint_EnableMotor(id, j.motor != nil)
            if let m = j.motor { b3PrismaticJoint_SetMotorSpeed(id, m.speed); b3PrismaticJoint_SetMaxMotorForce(id, m.maxForce) }
            b3PrismaticJoint_EnableSpring(id, j.spring != nil)
            if let sp = j.spring {
                b3PrismaticJoint_SetSpringHertz(id, sp.hertz); b3PrismaticJoint_SetSpringDampingRatio(id, sp.dampingRatio)
                b3PrismaticJoint_SetTargetTranslation(id, sp.target)
            }
        case let j as B3PhysicsSphericalJoint:
            b3SphericalJoint_EnableConeLimit(id, j.angularLimitInYZ != nil)
            if let l = j.angularLimitInYZ { b3SphericalJoint_SetConeLimit(id, max(l.0, l.1)) }
            b3SphericalJoint_EnableTwistLimit(id, j.twistLimit != nil)
            if let t = j.twistLimit { b3SphericalJoint_SetTwistLimits(id, t.lowerBound, t.upperBound) }
            b3SphericalJoint_EnableSpring(id, j.spring != nil)
            if let sp = j.spring { b3SphericalJoint_SetSpringHertz(id, sp.hertz); b3SphericalJoint_SetSpringDampingRatio(id, sp.dampingRatio) }
            b3SphericalJoint_EnableMotor(id, j.friction != nil)
            if let f = j.friction { b3SphericalJoint_SetMaxMotorTorque(id, f) }
        case let j as B3PhysicsFixedJoint:
            b3WeldJoint_SetLinearHertz(id, j.linearSpring?.hertz ?? 0)
            b3WeldJoint_SetLinearDampingRatio(id, j.linearSpring?.dampingRatio ?? 0)
            b3WeldJoint_SetAngularHertz(id, j.angularSpring?.hertz ?? 0)
            b3WeldJoint_SetAngularDampingRatio(id, j.angularSpring?.dampingRatio ?? 0)
        case let j as B3PhysicsDistanceJoint:
            let range = j.distanceLimit
            if range.lowerBound == range.upperBound && j.spring == nil {
                b3DistanceJoint_EnableSpring(id, false)
                b3DistanceJoint_EnableLimit(id, false)
                b3DistanceJoint_SetLength(id, range.lowerBound)
            } else {
                b3DistanceJoint_EnableSpring(id, true)
                b3DistanceJoint_SetSpringHertz(id, j.spring?.hertz ?? 0)
                b3DistanceJoint_SetSpringDampingRatio(id, j.spring?.dampingRatio ?? 0)
                b3DistanceJoint_SetLength(id, j.spring?.target ?? range.upperBound)
                b3DistanceJoint_EnableLimit(id, true)
                b3DistanceJoint_SetLengthRange(id, range.lowerBound, range.upperBound)
            }
        case let j as B3PhysicsWheelJoint:
            b3WheelJoint_EnableSuspension(id, j.suspension != nil)
            if let sp = j.suspension { b3WheelJoint_SetSuspensionHertz(id, sp.hertz); b3WheelJoint_SetSuspensionDampingRatio(id, sp.dampingRatio) }
            b3WheelJoint_EnableSuspensionLimit(id, j.suspensionLimit != nil)
            if let l = j.suspensionLimit { b3WheelJoint_SetSuspensionLimits(id, l.lowerBound, l.upperBound) }
            b3WheelJoint_EnableSpinMotor(id, j.spinMotor != nil)
            if let m = j.spinMotor { b3WheelJoint_SetSpinMotorSpeed(id, m.speed); b3WheelJoint_SetMaxSpinTorque(id, m.maxForce) }
            b3WheelJoint_EnableSteering(id, j.steering != nil)
            if let st = j.steering {
                b3WheelJoint_SetSteeringHertz(id, st.spring.hertz); b3WheelJoint_SetSteeringDampingRatio(id, st.spring.dampingRatio)
                b3WheelJoint_SetMaxSteeringTorque(id, st.maxTorque); b3WheelJoint_SetTargetSteeringAngle(id, st.angle)
                b3WheelJoint_EnableSteeringLimit(id, st.limit != nil)
                if let l = st.limit { b3WheelJoint_SetSteeringLimits(id, l.lowerBound, l.upperBound) }
            }
        case let j as B3PhysicsParallelJoint:
            b3ParallelJoint_SetSpringHertz(id, j.spring.hertz)
            b3ParallelJoint_SetSpringDampingRatio(id, j.spring.dampingRatio)
            b3ParallelJoint_SetMaxTorque(id, j.maxTorque ?? b3DefaultParallelJointDef().maxTorque)
        case let j as B3PhysicsMotorJoint:
            b3MotorJoint_SetLinearVelocity(id, b3Vec3(j.linearVelocity)); b3MotorJoint_SetMaxVelocityForce(id, j.maxForce)
            b3MotorJoint_SetAngularVelocity(id, b3Vec3(j.angularVelocity)); b3MotorJoint_SetMaxVelocityTorque(id, j.maxTorque)
            b3MotorJoint_SetLinearHertz(id, j.linearSpring?.hertz ?? 0); b3MotorJoint_SetLinearDampingRatio(id, j.linearSpring?.dampingRatio ?? 0)
            b3MotorJoint_SetAngularHertz(id, j.angularSpring?.hertz ?? 0); b3MotorJoint_SetAngularDampingRatio(id, j.angularSpring?.dampingRatio ?? 0)
            b3MotorJoint_SetMaxSpringForce(id, j.maxSpringForce); b3MotorJoint_SetMaxSpringTorque(id, j.maxSpringTorque)
        case is B3PhysicsFilterJoint:
            break
        default:
            return false
        }
        return true
    }

    /// Resolves a joint's pins. Nil when a pin cannot attach, for example to a removed body.
    private func resolve(_ joint: any B3PhysicsJoint) -> ResolvedJoint? {
        var bodies: [Entity.ID: (record: BodyRecord, scale: SIMD3<Float>)] = [:]
        var anchorDeps: [Entity.ID: (anchor: JointAnchor, scale: SIMD3<Float>)] = [:]
        var tracks = false
        /// Skeletal pins follow animation, which no event reports: such joints resolve every frame.
        func animates(_ entity: Entity) -> Bool {
            entity.components.has(SkeletalPosesComponent.self) || !((entity as? ModelEntity)?.jointNames.isEmpty ?? true)
        }
        func attach(_ pin: GeometricPin) -> (b3BodyId, b3Transform)? {
            guard let entity = pin.entity else { return nil }
            if let record = records[entity.id] {
                // RealityKit resolves the pin, skeletal joint included, in the entity's space.
                guard let p = pin.position(relativeTo: entity), let q = pin.orientation(relativeTo: entity) else { return nil }
                if animates(entity) { tracks = true }
                bodies[entity.id] = (record, record.scale)
                return (record.bodyId, b3Transform(translation: p * record.scale, rotation: q))
            }
            // A body entity that is not simulated, for example removed: the joint goes with it.
            if entity.components.has(B3PhysicsBodyComponent.self) { return nil }
            // Any other entity anchors the joint through a kinematic body that follows it.
            guard let p = pin.position(relativeTo: entity), let q = pin.orientation(relativeTo: entity) else { return nil }
            if animates(entity) { tracks = true }
            let anchor = anchorBody(for: entity)
            anchorDeps[entity.id] = (anchor, anchor.scale)
            return (anchor.bodyId, b3Transform(translation: p * anchor.scale, rotation: q))
        }
        guard let (bodyA, frameA) = attach(joint.pin0), let (bodyB, frameB) = attach(joint.pin1), bodyA != bodyB else { return nil }
        return ResolvedJoint(bodyA: bodyA, frameA: frameA, bodyB: bodyB, frameB: frameB, bodies: bodies,
                             anchors: anchorDeps, tracksPins: tracks)
    }

    private func makeJoint(_ joint: any B3PhysicsJoint, _ r: ResolvedJoint) -> b3JointId? {
        let (bodyA, frameA, bodyB, frameB) = (r.bodyA, r.frameA, r.bodyB, r.frameB)
        // RealityKit's axis is the pins' x-axis; Box3D's revolute and spherical joints use z.
        let xToZ = simd_quatf(angle: .pi / 2, axis: [0, 1, 0])
        // Fills a definition's base from Box3D's defaults, keeping its validation cookie and stiffness.
        func configure(_ d: inout b3JointDef, turn: Bool) {
            d.bodyIdA = bodyA
            d.bodyIdB = bodyB
            d.localFrameA = frameA
            d.localFrameB = frameB
            if turn {
                d.localFrameA.q = b3Quat(frameA.q.simd * xToZ)
                d.localFrameB.q = b3Quat(frameB.q.simd * xToZ)
            }
            d.collideConnected = joint.checksForInternalCollisions
            if let h = joint.constraintHertz { d.constraintHertz = h }
            // Box3D reports a joint whose load passes these; the world then breaks it.
            d.forceThreshold = joint.breakLimit?.force ?? .greatestFiniteMagnitude
            d.torqueThreshold = joint.breakLimit?.torque ?? .greatestFiniteMagnitude
        }
        switch joint {
        case let j as B3PhysicsRevoluteJoint:
            var d = b3DefaultRevoluteJointDef()
            configure(&d.base, turn: true)
            if let l = j.angularLimit { d.enableLimit = true; d.lowerAngle = l.lowerBound; d.upperAngle = l.upperBound }
            if let m = j.motor { d.enableMotor = true; d.motorSpeed = m.speed; d.maxMotorTorque = m.maxForce }
            if let s = j.spring { d.enableSpring = true; d.hertz = s.hertz; d.dampingRatio = s.dampingRatio; d.targetAngle = s.target }
            return b3CreateRevoluteJoint(id, &d)
        case let j as B3PhysicsPrismaticJoint:
            var d = b3DefaultPrismaticJointDef()
            configure(&d.base, turn: false)
            if let l = j.linearLimit { d.enableLimit = true; d.lowerTranslation = l.lowerBound; d.upperTranslation = l.upperBound }
            if let m = j.motor { d.enableMotor = true; d.motorSpeed = m.speed; d.maxMotorForce = m.maxForce }
            if let s = j.spring { d.enableSpring = true; d.hertz = s.hertz; d.dampingRatio = s.dampingRatio; d.targetTranslation = s.target }
            return b3CreatePrismaticJoint(id, &d)
        case let j as B3PhysicsSphericalJoint:
            var d = b3DefaultSphericalJointDef()
            configure(&d.base, turn: true)
            if let l = j.angularLimitInYZ { d.enableConeLimit = true; d.coneAngle = max(l.0, l.1) }
            if let t = j.twistLimit { d.enableTwistLimit = true; d.lowerTwistAngle = t.lowerBound; d.upperTwistAngle = t.upperBound }
            if let s = j.spring { d.enableSpring = true; d.hertz = s.hertz; d.dampingRatio = s.dampingRatio }
            if let f = j.friction { d.enableMotor = true; d.maxMotorTorque = f }
            return b3CreateSphericalJoint(id, &d)
        case let j as B3PhysicsFixedJoint:
            var d = b3DefaultWeldJointDef()
            configure(&d.base, turn: false)
            if let s = j.linearSpring { d.linearHertz = s.hertz; d.linearDampingRatio = s.dampingRatio }
            if let s = j.angularSpring { d.angularHertz = s.hertz; d.angularDampingRatio = s.dampingRatio }
            return b3CreateWeldJoint(id, &d)
        case let j as B3PhysicsDistanceJoint:
            var d = b3DefaultDistanceJointDef()
            configure(&d.base, turn: false)
            let range = j.distanceLimit
            if range.lowerBound == range.upperBound && j.spring == nil {
                d.length = range.lowerBound // a rigid rod
            } else {
                // A rope: free between the limits, or a spring toward its target within them.
                d.enableSpring = true
                d.hertz = j.spring?.hertz ?? 0
                d.dampingRatio = j.spring?.dampingRatio ?? 0
                d.length = j.spring?.target ?? range.upperBound
                d.enableLimit = true
                d.minLength = range.lowerBound
                d.maxLength = range.upperBound
            }
            return b3CreateDistanceJoint(id, &d)
        case let j as B3PhysicsWheelJoint:
            var d = b3DefaultWheelJointDef()
            configure(&d.base, turn: false)
            d.base.localFrameB.q = b3Quat(frameB.q.simd * xToZ) // spin around pin1's x-axis
            if let s = j.suspension {
                d.enableSuspensionSpring = true; d.suspensionHertz = s.hertz; d.suspensionDampingRatio = s.dampingRatio
            } else {
                d.enableSuspensionSpring = false
            }
            if let l = j.suspensionLimit { d.enableSuspensionLimit = true; d.lowerSuspensionLimit = l.lowerBound; d.upperSuspensionLimit = l.upperBound }
            if let m = j.spinMotor { d.enableSpinMotor = true; d.spinSpeed = m.speed; d.maxSpinTorque = m.maxForce }
            if let st = j.steering {
                d.enableSteering = true
                d.steeringHertz = st.spring.hertz; d.steeringDampingRatio = st.spring.dampingRatio
                d.maxSteeringTorque = st.maxTorque; d.targetSteeringAngle = st.angle
                if let l = st.limit { d.enableSteeringLimit = true; d.lowerSteeringLimit = l.lowerBound; d.upperSteeringLimit = l.upperBound }
            } else {
                d.enableSteering = false
            }
            return b3CreateWheelJoint(id, &d)
        case let j as B3PhysicsParallelJoint:
            var d = b3DefaultParallelJointDef()
            configure(&d.base, turn: true)
            d.hertz = j.spring.hertz
            d.dampingRatio = j.spring.dampingRatio
            if let t = j.maxTorque { d.maxTorque = t }
            return b3CreateParallelJoint(id, &d)
        case let j as B3PhysicsMotorJoint:
            var d = b3DefaultMotorJointDef()
            configure(&d.base, turn: false)
            d.linearVelocity = b3Vec3(j.linearVelocity); d.maxVelocityForce = j.maxForce
            d.angularVelocity = b3Vec3(j.angularVelocity); d.maxVelocityTorque = j.maxTorque
            if let s = j.linearSpring { d.linearHertz = s.hertz; d.linearDampingRatio = s.dampingRatio; d.maxSpringForce = j.maxSpringForce }
            if let s = j.angularSpring { d.angularHertz = s.hertz; d.angularDampingRatio = s.dampingRatio; d.maxSpringTorque = j.maxSpringTorque }
            return b3CreateMotorJoint(id, &d)
        case is B3PhysicsFilterJoint:
            var d = b3DefaultFilterJointDef()
            configure(&d.base, turn: false)
            return b3CreateFilterJoint(id, &d)
        default:
            log.error("Unsupported joint type \(String(describing: type(of: joint)))")
            return nil
        }
    }

    /// The kinematic body that stands for a non-body entity that joints pin to. It follows the
    /// entity like any kinematic body, so moving the entity drags what is jointed to it.
    private func anchorBody(for entity: Entity) -> JointAnchor {
        if let a = anchors[entity.id], b3Body_IsValid(a.bodyId) { return a }
        let pose = Pose(entity: entity, root: root)
        var def = b3DefaultBodyDef()
        def.type = b3_kinematicBody
        def.position = b3Vec3(pose.position)
        def.rotation = b3Quat(pose.orientation)
        let anchor = JointAnchor(entity: entity, bodyId: b3CreateBody(id, &def), pose: pose)
        anchors[entity.id] = anchor
        return anchor
    }

    /// Before the bodies step: anchors moved by the app aim at their entity over the first fixed
    /// step of the frame, then hold.
    func readAnchors() {
        for anchor in anchors.values {
            let pose = Pose(entity: anchor.entity, root: root)
            if !pose.isClose(to: anchor.pose) {
                anchor.target = pose
            }
            anchor.pose = pose // scale included, which joints read to place their frames
        }
    }

    func driveAnchors(fixedDt: Float) {
        for anchor in anchors.values {
            if let target = anchor.target {
                b3Body_SetTargetTransform(anchor.bodyId, b3Transform(translation: target.position, rotation: target.orientation), fixedDt, true)
                anchor.target = nil
            } else {
                b3Body_SetLinearVelocity(anchor.bodyId, b3Vec3(x: 0, y: 0, z: 0))
                b3Body_SetAngularVelocity(anchor.bodyId, b3Vec3(x: 0, y: 0, z: 0))
            }
        }
    }

    /// Destroys anchors no joint uses any more.
    func pruneAnchors() {
        let used = Set(jointSlots.compactMap { $0.resolved }.flatMap { [$0.bodyA, $0.bodyB] })
        for (key, anchor) in anchors where !used.contains(anchor.bodyId) {
            if b3Body_IsValid(anchor.bodyId) { b3DestroyBody(anchor.bodyId) }
            anchors[key] = nil
        }
    }

    // MARK: Joint state

    /// The angle of a revolute joint of the root's joints component, in radians, around its x-axis.
    public func angle(of joint: B3PhysicsRevoluteJoint) -> Float? {
        jointId(of: joint).map { b3RevoluteJoint_GetAngle($0) }
    }

    /// The translation of a prismatic joint along its x-axis, in the root's units.
    public func translation(of joint: B3PhysicsPrismaticJoint) -> Float? {
        jointId(of: joint).map { b3PrismaticJoint_GetTranslation($0) }
    }

    /// Changes a revolute, prismatic or wheel joint's motor in place, without rebuilding joints.
    /// A nil `maxForce` keeps the current one.
    public func setMotorSpeed(_ speed: Float, maxForce: Float? = nil, of joint: any B3PhysicsJoint) {
        guard let id = jointId(of: joint) else { return }
        switch joint {
        case is B3PhysicsRevoluteJoint:
            b3RevoluteJoint_SetMotorSpeed(id, speed)
            if let f = maxForce { b3RevoluteJoint_SetMaxMotorTorque(id, f) }
        case is B3PhysicsPrismaticJoint:
            b3PrismaticJoint_SetMotorSpeed(id, speed)
            if let f = maxForce { b3PrismaticJoint_SetMaxMotorForce(id, f) }
        case is B3PhysicsWheelJoint:
            b3WheelJoint_SetSpinMotorSpeed(id, speed)
            if let f = maxForce { b3WheelJoint_SetMaxSpinTorque(id, f) }
        default: return
        }
        b3Joint_WakeBodies(id)
    }

    /// Steers a wheel joint toward an angle in radians, in place.
    public func setSteeringAngle(_ angle: Float, of joint: B3PhysicsWheelJoint) {
        guard let id = jointId(of: joint) else { return }
        b3WheelJoint_SetTargetSteeringAngle(id, angle)
        b3Joint_WakeBodies(id)
    }

    func jointId(of joint: any B3PhysicsJoint) -> b3JointId? {
        guard let index = appliedJoints?.joints.firstIndex(where: { $0.isEqual(to: joint) }), index < jointSlots.count,
              let id = jointSlots[index].id, b3Joint_IsValid(id) else { return nil }
        return id
    }
}

import CBox3D
import RealityKit
import Foundation

extension B3World {
    /// After a step, before its `DidSimulate` handlers: destroys the joints Box3D reports over
    /// their break limit and removes them from the joints component, so handlers and the next
    /// steps see them gone. Box3D reports only joints with a limit, since the others have none.
    func collectJointBreaks() {
        let events = b3World_GetJointEvents(id)
        guard events.count > 0 else { return }
        var broken: [(slot: Int, force: SIMD3<Float>, torque: SIMD3<Float>)] = []
        for i in 0..<Int(events.count) {
            let jointId = events.jointEvents[i].jointId
            guard let slot = jointSlots.firstIndex(where: { $0.id == jointId }), b3Joint_IsValid(jointId) else { continue }
            broken.append((slot, b3Joint_GetConstraintForce(jointId).simd, b3Joint_GetConstraintTorque(jointId).simd))
            b3DestroyJoint(jointId, true)
        }
        var applied = appliedJoints?.joints ?? []
        var component = root?.components[B3PhysicsJointsComponent.self]
        for b in broken.sorted(by: { $0.slot > $1.slot }) where b.slot < applied.count {
            let joint = applied.remove(at: b.slot)
            jointSlots.remove(at: b.slot)
            // Between frames the component equals the applied joints. A handler may have edited
            // it since, a motor or a limit, or moved entries: a joint is the same while its type
            // and pins are, wherever it now sits.
            if let c = component {
                let index = b.slot < c.joints.count && Self.isSameJoint(c.joints[b.slot], joint)
                    ? b.slot : c.joints.firstIndex { Self.isSameJoint($0, joint) }
                if let index { component?.joints.remove(at: index) }
            }
            brokenJoints.append(B3JointEvents.Broke(joint: joint, force: b.force, torque: b.torque))
        }
        appliedJoints = B3PhysicsJointsComponent(joints: applied)
        if let root, let component { root.components.set(component) }
    }

    /// The same joint, perhaps with other parameters: same type, same pins.
    static func isSameJoint(_ a: any B3PhysicsJoint, _ b: any B3PhysicsJoint) -> Bool {
        type(of: a) == type(of: b) && a.pin0 == b.pin0 && a.pin1 == b.pin1
    }

    /// At the end of the frame: the joints that broke during it, oldest first.
    func finishJointBreaks() -> [B3JointEvents.Broke] {
        guard !brokenJoints.isEmpty else { return [] }
        defer { brokenJoints.removeAll() }
        if !anchors.isEmpty { pruneAnchors() }
        return brokenJoints
    }

    func dispatch(_ events: [B3JointEvents.Broke]) {
        guard !events.isEmpty, !jointBreakHandlers.isEmpty else { return }
        let handlers = Array(jointBreakHandlers.values)
        for event in events { for h in handlers { h(event) } }
    }

    /// Box3D extension: runs when a joint breaks. Keep the subscription: releasing it cancels.
    public func subscribe(to event: B3JointEvents.Broke.Type, _ handler: @escaping (B3JointEvents.Broke) -> Void) -> B3Subscription {
        let token = UUID()
        jointBreakHandlers[token] = handler
        return B3Subscription { [weak self] in self?.jointBreakHandlers[token] = nil }
    }

    /// Box3D extension: the force a joint's constraint applies now, in the simulation root's space.
    /// Nil for a joint the world has not built.
    public func constraintForce(of joint: any B3PhysicsJoint) -> SIMD3<Float>? {
        jointId(of: joint).map { b3Joint_GetConstraintForce($0).simd }
    }

    /// Box3D extension: the torque a joint's constraint applies now, in the simulation root's space.
    public func constraintTorque(of joint: any B3PhysicsJoint) -> SIMD3<Float>? {
        jointId(of: joint).map { b3Joint_GetConstraintTorque($0).simd }
    }
}

/// Box3D extension: a joint's live state, read from Box3D. Only the values its type has are set.
public struct B3JointReading: Equatable, Sendable {
    /// Revolute: the angle in radians. Spherical: the twist around the x-axis.
    public var angle: Float?
    /// Spherical: the cone angle off pin0's x-axis.
    public var coneAngle: Float?
    /// Prismatic: the translation along the x-axis.
    public var translation: Float?
    /// Prismatic: the sliding speed. Wheel: the spin speed in radians per second.
    public var speed: Float?
    /// Distance: the current length.
    public var length: Float?
    /// Wheel: the steering angle.
    public var steeringAngle: Float?
    /// The motor's force (prismatic, distance) or torque (revolute, wheel spin) this step.
    public var motorEffort: Float?
}

extension B3World {
    /// Box3D extension: the live state of a joint the world has built.
    public func reading(of joint: any B3PhysicsJoint) -> B3JointReading? {
        guard let id = jointId(of: joint) else { return nil }
        var r = B3JointReading()
        switch joint {
        case is B3PhysicsRevoluteJoint:
            r.angle = b3RevoluteJoint_GetAngle(id); r.motorEffort = b3RevoluteJoint_GetMotorTorque(id)
        case is B3PhysicsPrismaticJoint:
            r.translation = b3PrismaticJoint_GetTranslation(id); r.speed = b3PrismaticJoint_GetSpeed(id)
            r.motorEffort = b3PrismaticJoint_GetMotorForce(id)
        case is B3PhysicsSphericalJoint:
            r.angle = b3SphericalJoint_GetTwistAngle(id); r.coneAngle = b3SphericalJoint_GetConeAngle(id)
        case is B3PhysicsDistanceJoint:
            r.length = b3DistanceJoint_GetCurrentLength(id); r.motorEffort = b3DistanceJoint_GetMotorForce(id)
        case is B3PhysicsWheelJoint:
            r.speed = b3WheelJoint_GetSpinSpeed(id); r.steeringAngle = b3WheelJoint_GetSteeringAngle(id)
            r.motorEffort = b3WheelJoint_GetSpinTorque(id)
        default: break
        }
        return r
    }
}

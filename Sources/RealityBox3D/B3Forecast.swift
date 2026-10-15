import CBox3D
import RealityKit
import simd

extension B3World {
    /// An exact copy of the world as it is now, to step ahead without touching the real one.
    ///
    /// Box3D is deterministic and its recordings open with a full snapshot of the world, contacts,
    /// warm-start impulses, islands and sleep state included. The forecast restores that snapshot
    /// into a separate world, so stepping it reproduces what the real world will do, provided the
    /// app changes nothing in between. Call it between frames, never during an update.
    ///
    /// Kinematic bodies and joint anchors replay what the world will do on its own: a target set
    /// this frame, then holding still, or a velocity the app gave them. One carried along by a
    /// moving parent, such as a kinematic child of a dynamic body, depends on frames the forecast
    /// does not have, so such scenes forecast only approximately.
    public func forecast() -> B3Forecast? {
        guard let recording = b3CreateRecording(1 << 16) else { return nil }
        defer { b3DestroyRecording(recording) }
        b3World_StartRecording(id, recording)
        b3World_StopRecording(id)
        guard let data = b3Recording_GetData(recording),
              let player = b3CreatePlayer(data, b3Recording_GetSize(recording), 1) else { return nil }
        var bodies: [Entity.ID: b3BodyId] = [:]
        var kinematics: [B3Forecast.Kinematic] = []
        let copy = b3RecPlayer_GetWorldId(player)
        for (key, record) in records {
            // Same id pools, other world: rewrite the world index.
            var id = record.bodyId
            id.world0 = copy.index1 &- 1
            guard b3Body_IsValid(id) else { continue }
            bodies[key] = id
            if record.body.mode == .kinematic {
                // The copy must drive kinematic bodies as the world will: toward a pending target
                // on the first step, then held still unless the app set them a velocity.
                let target = record.pendingKinematicTarget.map { b3Transform(translation: $0.position, rotation: $0.orientation) }
                kinematics.append(.init(id: id, target: target, isVelocityDriven: record.isVelocityDriven))
            }
        }
        for anchor in anchors.values {
            var id = anchor.bodyId
            id.world0 = copy.index1 &- 1
            guard b3Body_IsValid(id) else { continue }
            let target = anchor.target.map { b3Transform(translation: $0.position, rotation: $0.orientation) }
            kinematics.append(.init(id: id, target: target, isVelocityDriven: false))
        }
        // Snapshots keep shapes' pre-solve flags but not the world's callbacks: install the same
        // rules, with a one-way table the forecast owns.
        let oneWay = OneWayShapes()
        // The copy's one-way bodies, turned as they turn in the copy.
        let sources = oneWaySources.map { source -> OneWayShapes.Source in
            var copied = source
            copied.body.world0 = copy.index1 &- 1
            return copied
        }
        Self.installContactRules(on: copy, settings, oneWay: oneWay)
        return B3Forecast(player: player, worldId: copy, bodies: bodies, kinematics: kinematics,
                          timeStep: Float(1 / settings.fixedRate), subSteps: Int32(settings.subStepCount),
                          oneWay: oneWay, oneWaySources: sources)
    }
}

/// A copy of a `B3World` stepped ahead of it. Poses and velocities are in the root's space.
@MainActor
public final class B3Forecast {
    private let player: OpaquePointer
    private let worldId: b3WorldId
    private let bodies: [Entity.ID: b3BodyId]
    struct Kinematic { let id: b3BodyId; let target: b3Transform?; let isVelocityDriven: Bool }
    private let kinematics: [Kinematic]
    private let timeStep: Float
    private let subSteps: Int32
    public private(set) var stepCount = 0
    /// The one-way table the copy's pre-solve callback reads, and the bodies that turn it.
    private let oneWay: OneWayShapes
    private let oneWaySources: [OneWayShapes.Source]

    init(player: OpaquePointer, worldId: b3WorldId, bodies: [Entity.ID: b3BodyId], kinematics: [Kinematic],
         timeStep: Float, subSteps: Int32, oneWay: OneWayShapes, oneWaySources: [OneWayShapes.Source]) {
        self.oneWay = oneWay
        self.oneWaySources = oneWaySources
        self.player = player
        self.worldId = worldId
        self.bodies = bodies
        self.kinematics = kinematics
        self.timeStep = timeStep
        self.subSteps = subSteps
    }

    isolated deinit { b3DestroyPlayer(player) }

    /// Advances the copy by fixed steps of the source world's rate.
    public func step(_ count: Int = 1) {
        for _ in 0..<count {
            for k in kinematics {
                if stepCount == 0, let target = k.target {
                    b3Body_SetTargetTransform(k.id, target, timeStep, true)
                } else if !k.isVelocityDriven {
                    b3Body_SetLinearVelocity(k.id, b3Vec3(x: 0, y: 0, z: 0))
                    b3Body_SetAngularVelocity(k.id, b3Vec3(x: 0, y: 0, z: 0))
                }
            }
            if !oneWaySources.isEmpty { oneWay.refresh(oneWaySources) }
            b3World_Step(worldId, timeStep, subSteps)
            stepCount += 1
        }
    }

    public func transform(of entity: Entity) -> Transform? {
        guard let id = bodies[entity.id] else { return nil }
        let t = b3Body_GetTransform(id)
        return Transform(rotation: t.q.simd, translation: t.p.simd)
    }

    public func motion(of entity: Entity) -> B3PhysicsMotionComponent? {
        guard let id = bodies[entity.id] else { return nil }
        return .init(linearVelocity: b3Body_GetLinearVelocity(id).simd, angularVelocity: b3Body_GetAngularVelocity(id).simd)
    }

    public var awakeBodyCount: Int { Int(b3World_GetAwakeBodyCount(worldId)) }
}

extension RealityBox3D {
    /// Box3D's length unit, process-wide: Box3D's tolerances, such as its contact slop of 0.005
    /// units, are multiplied by it. Raise it for models that are real-world size in small units,
    /// such as a 1 m barrel modeled as 100 cm. For small objects modeled a few units across, such as
    /// dice in centimeters, leave it at 1. Set it before creating any world.
    @MainActor public static var lengthUnitsPerMeter: Float {
        get { b3GetLengthUnitsPerMeter() }
        set { b3SetLengthUnitsPerMeter(newValue) }
    }
}

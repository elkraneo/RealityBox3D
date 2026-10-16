import RealityKit
import simd

/// Mirrors `PhysicsMassProperties`.
public struct B3PhysicsMassProperties: Equatable, Sendable {
    public static let `default` = B3PhysicsMassProperties()
    public var mass: Float
    /// Principal moments of inertia. Zero means "derive from shapes".
    public var inertia: SIMD3<Float>
    public var centerOfMass: (position: SIMD3<Float>, orientation: simd_quatf)

    public init() {
        self.init(mass: 1, inertia: .zero)
    }
    public init(mass: Float, inertia: SIMD3<Float> = .zero,
                centerOfMass: (position: SIMD3<Float>, orientation: simd_quatf) = (.zero, simd_quatf(ix: 0, iy: 0, iz: 0, r: 1))) {
        self.mass = mass
        self.inertia = inertia
        self.centerOfMass = centerOfMass
    }
    public static func == (a: Self, b: Self) -> Bool {
        a.mass == b.mass && a.inertia == b.inertia
            && a.centerOfMass.position == b.centerOfMass.position
            && a.centerOfMass.orientation == b.centerOfMass.orientation
    }
}

/// Mirrors `PhysicsMaterialResource`. Box3D has a single friction coefficient, so
/// `dynamicFriction` is used and `staticFriction` is kept for source compatibility.
public struct B3PhysicsMaterial: Equatable, Sendable {
    public static let `default` = B3PhysicsMaterial(staticFriction: 0.6, dynamicFriction: 0.6, restitution: 0)
    public var staticFriction: Float
    public var dynamicFriction: Float
    public var restitution: Float
    /// Box3D extension: rolling resistance for spheres and capsules.
    public var rollingResistance: Float = 0
    /// Box3D extension: a conveyor belt's surface speed, in the body's frame, projected onto
    /// each contact. Bodies resting on the shape are carried along at up to this speed.
    public var tangentVelocity: SIMD3<Float> = .zero

    public static func generate(staticFriction: Float = 0.6, dynamicFriction: Float = 0.6, restitution: Float = 0) -> B3PhysicsMaterial {
        .init(staticFriction: staticFriction, dynamicFriction: dynamicFriction, restitution: restitution)
    }
    public init(staticFriction: Float, dynamicFriction: Float, restitution: Float) {
        self.staticFriction = staticFriction
        self.dynamicFriction = dynamicFriction
        self.restitution = restitution
    }
}

/// Mirrors `PhysicsBodyComponent`.
public struct B3PhysicsBodyComponent: Component, Equatable, Sendable {
    enum MassSource: Equatable, Sendable {
        case explicit
        case density(Float)
    }

    public var mode: PhysicsBodyMode
    public var massProperties: B3PhysicsMassProperties { didSet { massSource = .explicit } }
    public var material: B3PhysicsMaterial
    public var isTranslationLocked: (x: Bool, y: Bool, z: Bool) = (false, false, false)
    public var isRotationLocked: (x: Bool, y: Bool, z: Bool) = (false, false, false)
    public var isContinuousCollisionDetectionEnabled: Bool = false
    public var isAffectedByGravity: Bool = true
    public var linearDamping: Float = 0
    public var angularDamping: Float = 0.05

    // Box3D extensions, no RealityKit counterpart.
    public var gravityScale: Float = 1
    public var isSleepEnabled: Bool = true
    public var isBullet: Bool = false
    /// Box3D extension: lets the body spin past Box3D's rotation speed limit. For round bodies such as wheels.
    public var allowsFastRotation: Bool = false

    var massSource: MassSource

    public init() {
        self.init(massProperties: .default, material: nil, mode: .dynamic)
    }

    public init(massProperties: B3PhysicsMassProperties = .default, material: B3PhysicsMaterial? = nil, mode: PhysicsBodyMode = .dynamic) {
        self.mode = mode
        self.massProperties = massProperties
        self.material = material ?? .default
        self.massSource = .explicit
    }

    /// Derives mass and inertia from the entity's collision shapes, like RealityKit's
    /// `init(shapes:density:material:mode:)`. The `shapes` argument is accepted for source
    /// compatibility; the shapes actually used are those on the entity's `B3CollisionComponent`.
    public init(shapes: [B3ShapeResource] = [], density: Float, material: B3PhysicsMaterial? = nil, mode: PhysicsBodyMode = .dynamic) {
        self.init(massProperties: .default, material: material, mode: mode)
        self.massSource = .density(density)
    }

    public init(shapes: [B3ShapeResource] = [], mass: Float, material: B3PhysicsMaterial? = nil, mode: PhysicsBodyMode = .dynamic) {
        self.init(massProperties: .init(mass: mass), material: material, mode: mode)
    }

    public static func == (a: Self, b: Self) -> Bool {
        a.mode == b.mode && a.massProperties == b.massProperties && a.material == b.material
            && a.isTranslationLocked == b.isTranslationLocked && a.isRotationLocked == b.isRotationLocked
            && a.isContinuousCollisionDetectionEnabled == b.isContinuousCollisionDetectionEnabled
            && a.isAffectedByGravity == b.isAffectedByGravity && a.linearDamping == b.linearDamping
            && a.angularDamping == b.angularDamping && a.gravityScale == b.gravityScale
            && a.isSleepEnabled == b.isSleepEnabled && a.isBullet == b.isBullet && a.massSource == b.massSource
            && a.allowsFastRotation == b.allowsFastRotation
    }
}

/// Mirrors `CollisionComponent`. Uses RealityKit's own `CollisionFilter`.
public struct B3CollisionComponent: Component, Equatable, Sendable {
    public enum Mode: Equatable, Sendable {
        case `default`
        /// Sensor: reports overlaps, no collision response.
        case trigger
        case colliding
    }
    public var shapes: [B3ShapeResource]
    public var mode: Mode
    public var filter: CollisionFilter
    /// Box3D extension: emit begin and end contact events for this entity.
    public var reportsContacts: Bool = true
    /// Box3D extension: bodies with the same negative group index never collide, with the same
    /// positive one always do, whatever the filter says. Zero leaves it to the filter. A ragdoll
    /// uses one negative index so its limbs pass through each other.
    public var groupIndex: Int = 0
    /// Box3D extension: a one-way surface, such as a platform a character jumps up through. It
    /// pushes bodies only along this direction, in the entity's own space; bodies coming from the
    /// other side pass through. Nil collides from every side.
    public var oneWayDirection: SIMD3<Float>?
    /// Box3D extension: bakes all the shapes of a static body into one compound, which collides
    /// faster when the body has many shapes, as level geometry does. Static bodies only; height
    /// fields stay separate shapes. Changing anything rebakes it.
    public var bakesCompound: Bool = false

    public init(shapes: [B3ShapeResource], mode: Mode = .default, filter: CollisionFilter = .default) {
        self.shapes = shapes
        self.mode = mode
        self.filter = filter
    }

    public init(shapes: [B3ShapeResource], isStatic: Bool, filter: CollisionFilter = .default) {
        self.init(shapes: shapes, mode: .default, filter: filter)
    }
}

extension CollisionFilter: @retroactive @unchecked Sendable {}
extension PhysicsBodyMode: @retroactive @unchecked Sendable {}

/// Mirrors `PhysicsMotionComponent`. Written back every step for dynamic bodies;
/// external changes are pushed into Box3D before the next step.
public struct B3PhysicsMotionComponent: Component, Equatable, Sendable {
    public var linearVelocity: SIMD3<Float>
    public var angularVelocity: SIMD3<Float>
    public init(linearVelocity: SIMD3<Float> = .zero, angularVelocity: SIMD3<Float> = .zero) {
        self.linearVelocity = linearVelocity
        self.angularVelocity = angularVelocity
    }
}

/// Mirrors `PhysicsSimulationComponent`. Bodies below an entity carrying this component
/// are simulated in that entity's space. Without one, bodies live in scene space.
public struct B3PhysicsSimulationComponent: Component, Equatable, Sendable {
    public var gravity: SIMD3<Float> = [0, -9.81, 0]
    /// Box3D uses substeps instead of position and velocity iterations.
    public var subStepCount: Int = 4
    /// Fixed simulation rate in hertz. Poses are interpolated between steps.
    public var fixedRate: Double = 60
    /// Upper bound on fixed steps per frame, to avoid a spiral of death after hitches.
    public var maxStepsPerFrame: Int = 4
    public var isSleepEnabled: Bool = true
    public var isContinuousEnabled: Bool = true
    /// Box3D worker threads. 1 runs single threaded on the calling thread.
    public var workerCount: Int = 1
    public var interpolatesPoses: Bool = true
    public var isPaused: Bool = false
    /// Approach speed above which collisions bounce, in the root's units per second.
    /// Nil keeps Box3D's default of 1 m/s, scaled by `RealityBox3D.lengthUnitsPerMeter`.
    public var restitutionThreshold: Float?
    /// Box3D extension: how two touching materials' frictions combine. Box3D's default is the
    /// geometric mean; RealityKit's PhysX default is the average.
    public var frictionCombine: B3MaterialCombine = .geometricMean
    /// Box3D extension: how two touching materials' restitutions combine. Box3D's default is the
    /// larger one.
    public var restitutionCombine: B3MaterialCombine = .maximum
    /// Box3D extension: resolves sphere impacts pair by pair before each step, so momentum passes
    /// along a row of touching balls, as in a Newton's cradle or a pool break, instead of being
    /// shared among them. Costs a query per awake sphere per step. One-way shapes, and bodies held by
    /// joints other than a pendulum's single revolute or spherical joint to a static body, are left
    /// to Box3D's solver.
    public var sequentialImpacts: Bool = false

    public init() {}

    @MainActor public static func nearestSimulationEntity(for entity: Entity) -> Entity? {
        var e: Entity? = entity
        while let current = e {
            if current.components.has(B3PhysicsSimulationComponent.self) { return current }
            e = current.parent
        }
        return nil
    }
}

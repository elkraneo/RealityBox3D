import RealityBox3D
import RealityKit
import simd

/// What a scene's per-frame code and actions see.
@MainActor
public struct DemoContext {
    /// The simulation root.
    public let root: Entity
    /// Nil until the physics system has seen the scene's bodies.
    public let world: B3World?
    /// Seconds since the scene loaded, advanced only while not paused.
    public let time: Double
    public let deltaTime: Double
    /// Keys held in the app, for scenes with `controls`.
    public let input: DemoInput
    /// The camera's view direction, flattened onto the ground, for camera-relative walking.
    public let cameraForward: SIMD3<Float>
}

/// Upstream's character keys: WASD to walk, space to jump, shift to sprint.
public struct DemoInput: Equatable, Sendable {
    /// x is forward (W minus S), y is right (D minus A).
    public var throttle: SIMD2<Float> = .zero
    public var jump = false
    public var sprint = false
    public init() {}
}

@MainActor
public final class DemoScript {
    var onFrame: [@MainActor (DemoContext) -> Void] = []
    var onFixedStep: [@MainActor (DemoContext) -> Void] = []
    var afterPhysics: [@MainActor (DemoContext) -> Void] = []
    private var stepSubscription: B3Subscription?
    private weak var subscribedWorld: B3World?
    var actions: [String: @MainActor (DemoContext) -> Void] = [:]
    var time: Double = 0
    /// Scratch storage for one-shot work in `onFrame`.
    public var flags: Set<String> = []
    /// Keeps subscriptions and other objects alive as long as the scene.
    public var retained: [AnyObject] = []
    /// Set by the app.
    var input = DemoInput()
    var cameraForward: SIMD3<Float> = [0, 0, -1]
    /// The entity the app's camera follows, as upstream's third-person camera.
    var follow: Entity?

    func frame(root: Entity, world: B3World?, deltaTime: Double) {
        let ctx = DemoContext(root: root, world: world, time: time, deltaTime: deltaTime, input: input, cameraForward: cameraForward)
        // Fixed-step code runs inside the world's steps, before each one, as upstream's Step()
        if !onFixedStep.isEmpty, let world, world !== subscribedWorld {
            subscribedWorld = world
            stepSubscription = world.subscribe(to: B3PhysicsSimulationEvents.WillSimulate.self) { [unowned self, unowned root, unowned world] event in
                let ctx = DemoContext(root: root, world: world, time: self.time, deltaTime: Double(event.deltaTime),
                                      input: self.input, cameraForward: self.cameraForward)
                for f in self.onFixedStep { f(ctx) }
            }
        }
        for f in onFrame { f(ctx) }
        time += deltaTime
    }

    func late(root: Entity, world: B3World?) {
        let ctx = DemoContext(root: root, world: world, time: time, deltaTime: 0, input: input, cameraForward: cameraForward)
        for f in afterPhysics { f(ctx) }
    }

    func perform(_ action: String, root: Entity, world: B3World?) {
        actions[action]?(DemoContext(root: root, world: world, time: time, deltaTime: 0, input: input, cameraForward: cameraForward))
    }
}

public struct DemoScriptComponent: Component {
    let script: DemoScript
}

/// Runs scene scripts before the physics system, as upstream runs a sample's `Step()` before `b3World_Step`.
public struct DemoScriptSystem: System {
    static let query = EntityQuery(where: .has(DemoScriptComponent.self))
    public static var dependencies: [SystemDependency] { [.before(B3PhysicsSystem.self)] }
    public init(scene: RealityKit.Scene) {}

    public func update(context: SceneUpdateContext) {
        for entity in context.entities(matching: Self.query, updatingSystemWhen: .rendering) {
            guard let root = entity.parent, let script = entity.components[DemoScriptComponent.self]?.script else { continue }
            let paused = root.components[B3PhysicsSimulationComponent.self]?.isPaused ?? false
            script.frame(root: root, world: RealityBox3D.world(for: root), deltaTime: paused ? 0 : context.deltaTime)
        }
    }

    @MainActor public static func register() {
        DemoScriptComponent.registerComponent()
        registerSystem()
        DemoLateScriptSystem.registerSystem()
    }
}

/// Runs scene code after the physics system has stepped and written poses, for what is drawn
/// between steps, as the bodies' interpolated poses.
public struct DemoLateScriptSystem: System {
    public static var dependencies: [SystemDependency] { [.after(B3PhysicsSystem.self)] }
    public init(scene: RealityKit.Scene) {}

    public func update(context: SceneUpdateContext) {
        for entity in context.entities(matching: DemoScriptSystem.query, updatingSystemWhen: .rendering) {
            guard let root = entity.parent, let script = entity.components[DemoScriptComponent.self]?.script else { continue }
            script.late(root: root, world: RealityBox3D.world(for: root))
        }
    }
}

extension DemoScene {
    /// Runs a named action of the scene loaded under `root`.
    @MainActor
    public static func perform(_ action: String, root: Entity) {
        for child in root.children {
            child.components[DemoScriptComponent.self]?.script.perform(action, root: root, world: RealityBox3D.world(for: root))
        }
    }

    /// Hands the app's keys and camera direction to the scene loaded under `root`.
    @MainActor
    public static func setInput(_ input: DemoInput, cameraForward: SIMD3<Float>, root: Entity) {
        var forward = cameraForward
        forward.y = 0
        forward = simd_length(forward) > 1e-4 ? simd_normalize(forward) : [0, 0, -1]
        for child in root.children {
            guard let script = child.components[DemoScriptComponent.self]?.script else { continue }
            script.input = input
            script.cameraForward = forward
        }
    }

    /// Where the camera should look in the scene loaded under `root`, if it follows an entity.
    @MainActor
    public static func followTarget(root: Entity) -> SIMD3<Float>? {
        for child in root.children {
            if let e = child.components[DemoScriptComponent.self]?.script.follow { return e.position(relativeTo: nil) }
        }
        return nil
    }
}

import RealityBox3D
import RealityKit

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
}

@MainActor
public final class DemoScript {
    var onFrame: [@MainActor (DemoContext) -> Void] = []
    var actions: [String: @MainActor (DemoContext) -> Void] = [:]
    var time: Double = 0
    /// Scratch storage for one-shot work in `onFrame`.
    public var flags: Set<String> = []
    /// Keeps subscriptions and other objects alive as long as the scene.
    public var retained: [AnyObject] = []

    func frame(root: Entity, world: B3World?, deltaTime: Double) {
        let ctx = DemoContext(root: root, world: world, time: time, deltaTime: deltaTime)
        for f in onFrame { f(ctx) }
        time += deltaTime
    }

    func perform(_ action: String, root: Entity, world: B3World?) {
        actions[action]?(DemoContext(root: root, world: world, time: time, deltaTime: 0))
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
}

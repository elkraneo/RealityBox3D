import RealityBox3D
import RealityKit

/// Runs a scene without a RealityKit scene or renderer, for tests, benchmarks and offscreen tools.
/// It does what `DemoScriptSystem` and `B3PhysicsSystem` do each frame.
@MainActor
public final class DemoRunner {
    public let scene: DemoScene
    public let root = Entity()
    public private(set) var world: B3World
    private(set) var bodies: [Entity] = []

    public init(_ scene: DemoScene, configure: (inout B3PhysicsSimulationComponent) -> Void = { _ in }) {
        self.scene = scene
        var settings = B3PhysicsSimulationComponent()
        scene.configure(&settings)
        configure(&settings)
        root.components.set(settings)
        scene.load(into: root)
        root.components.set(settings) // keep caller overrides over the scene's
        world = B3World(root: root, settings: settings)
        collectBodies()
    }

    public var bodyCount: Int { bodies.count }

    /// Advances `frames` frames of `deltaTime` each.
    public func advance(frames: Int = 1, deltaTime: Double = 1.0 / 60) {
        for _ in 0..<frames {
            for child in root.children {
                child.components[DemoScriptComponent.self]?.script.frame(root: root, world: world, deltaTime: deltaTime)
            }
            if bodiesChanged { collectBodies() }
            world.update(entities: bodies, deltaTime: deltaTime)
        }
    }

    public func perform(_ action: String) {
        for child in root.children {
            child.components[DemoScriptComponent.self]?.script.perform(action, root: root, world: world)
        }
        collectBodies()
    }

    /// Scripts remove bodies by detaching them; actions that add bodies recollect in `perform`.
    private var bodiesChanged: Bool { bodies.contains { $0.parent == nil } }

    private func collectBodies() {
        var out: [Entity] = []
        func visit(_ e: Entity) {
            if e.components.has(B3PhysicsBodyComponent.self) && e.components.has(B3CollisionComponent.self) { out.append(e) }
            for c in e.children { visit(c) }
        }
        visit(root)
        bodies = out
    }
}

import Foundation
import Combine
import CBox3D
import RealityKit

/// Drives every Box3D world in a scene. Register once with `RealityBox3D.register()`.
///
/// After a first full scan, it reads only entities that RealityKit reports as changed: a body or
/// collision component edited, a transform moved by the app, an entity added, reparented or
/// removed. Sleeping and untouched bodies cost nothing per frame.
public struct B3PhysicsSystem: System {
    static let query = EntityQuery(where: .has(B3PhysicsBodyComponent.self) && .has(B3CollisionComponent.self))

    /// Released with the system when RealityKit tears down the scene's systems; then the
    /// scene's Box3D worlds are destroyed, even while entities keep the scene object alive.
    private let lifetime: SceneLifetime

    public init(scene: Scene) {
        lifetime = SceneLifetime(scene: ObjectIdentifier(scene))
    }

    public mutating func update(context: SceneUpdateContext) {
        let tracker = B3WorldRegistry.shared.tracker(for: context.scene, token: lifetime.token)
        if tracker.needsFullScan {
            let all = Array(context.entities(matching: Self.query, updatingSystemWhen: .rendering))
            tracker.fullScan(all, deltaTime: context.deltaTime)
        } else {
            tracker.incremental(deltaTime: context.deltaTime)
        }
    }
}

final class SceneLifetime: @unchecked Sendable {
    let scene: ObjectIdentifier
    /// Tells this scene apart from a later one that reuses its address.
    let token = UUID()
    init(scene: ObjectIdentifier) { self.scene = scene }
    deinit {
        let (scene, token) = (self.scene, self.token)
        Task { @MainActor in B3WorldRegistry.shared.release(scene: scene, token: token) }
    }
}

/// The change tracking and the worlds of one RealityKit scene.
@MainActor
final class SceneTracker {
    typealias Key = B3WorldRegistry.Key
    let sceneKey: ObjectIdentifier
    let token: UUID
    private(set) weak var scene: Scene?
    var needsFullScan = true
    private var members: [Entity.ID: (entity: Entity, key: Key)] = [:]
    private var dirty: [Entity.ID: Entity] = [:]
    private var removed: Set<Entity.ID> = []
    private var subscriptions: [any Cancellable] = []

    init(scene: Scene, token: UUID) {
        sceneKey = ObjectIdentifier(scene)
        self.token = token
        self.scene = scene
        let bodyTypes: [any Component.Type] = [B3PhysicsBodyComponent.self, B3CollisionComponent.self]
        for type in bodyTypes {
            subscriptions.append(scene.subscribe(to: ComponentEvents.DidAdd.self, componentType: type) { [unowned self] e in mark(e.entity) })
            subscriptions.append(scene.subscribe(to: ComponentEvents.DidChange.self, componentType: type) { [unowned self] e in mark(e.entity) })
            subscriptions.append(scene.subscribe(to: ComponentEvents.WillRemove.self, componentType: type) { [unowned self] e in drop(e.entity) })
        }
        // A body added or removed above other bodies changes their nesting.
        subscriptions.append(scene.subscribe(to: ComponentEvents.DidAdd.self, componentType: B3PhysicsBodyComponent.self) { [unowned self] e in
            for c in e.entity.children { markSubtree(c) }
        })
        subscriptions.append(scene.subscribe(to: ComponentEvents.WillRemove.self, componentType: B3PhysicsBodyComponent.self) { [unowned self] e in
            for c in e.entity.children { markSubtree(c) }
        })
        // Motion components added, edited or removed by the app; the world's own writes are not edits.
        subscriptions.append(scene.subscribe(to: ComponentEvents.DidAdd.self, componentType: B3PhysicsMotionComponent.self) { [unowned self] e in
            if !B3World.isWritingEntities { mark(e.entity) }
        })
        subscriptions.append(scene.subscribe(to: ComponentEvents.DidChange.self, componentType: B3PhysicsMotionComponent.self) { [unowned self] e in
            if !B3World.isWritingEntities { mark(e.entity) }
        })
        subscriptions.append(scene.subscribe(to: ComponentEvents.WillRemove.self, componentType: B3PhysicsMotionComponent.self) { [unowned self] e in
            if !B3World.isWritingEntities { mark(e.entity) }
        })
        subscriptions.append(scene.subscribe(to: ComponentEvents.DidChange.self, componentType: Transform.self) { [unowned self] e in
            transformChanged(e.entity, byWorld: B3World.isWritingEntities)
        })
        // A simulation root added or removed regroups bodies between worlds: rescan.
        subscriptions.append(scene.subscribe(to: ComponentEvents.DidAdd.self, componentType: B3PhysicsSimulationComponent.self) { [unowned self] _ in
            needsFullScan = true
        })
        subscriptions.append(scene.subscribe(to: ComponentEvents.WillRemove.self, componentType: B3PhysicsSimulationComponent.self) { [unowned self] _ in
            needsFullScan = true
        })
        subscriptions.append(scene.subscribe(to: SceneEvents.DidAddEntity.self) { [unowned self] e in markSubtree(e.entity) })
        subscriptions.append(scene.subscribe(to: SceneEvents.DidReparentEntity.self) { [unowned self] e in markSubtree(e.child) })
        // Enabling or disabling an entity turns its bodies, and those below it, on or off.
        subscriptions.append(scene.subscribe(to: SceneEvents.DidActivateEntity.self) { [unowned self] e in markSubtree(e.entity) })
        subscriptions.append(scene.subscribe(to: SceneEvents.WillDeactivateEntity.self) { [unowned self] e in markSubtree(e.entity) })
        subscriptions.append(scene.subscribe(to: SceneEvents.WillRemoveEntity.self) { [unowned self] e in dropSubtree(e.entity) })
    }

    private func mark(_ e: Entity) {
        dirty[e.id] = e
        removed.remove(e.id)
    }

    private func drop(_ e: Entity) {
        dirty[e.id] = nil
        if members[e.id] != nil { removed.insert(e.id) }
    }

    private func markSubtree(_ e: Entity) {
        if e.components.has(B3PhysicsBodyComponent.self) { mark(e) }
        for c in e.children { markSubtree(c) }
    }

    private func dropSubtree(_ e: Entity) {
        drop(e)
        for c in e.children { dropSubtree(c) }
    }

    /// A body moved by the app, or an ancestor that carries bodies along. When the world itself
    /// moved the entity, only the bodies nested under it changed.
    private func transformChanged(_ e: Entity, byWorld: Bool) {
        if !byWorld, members[e.id] != nil || e.components.has(B3PhysicsBodyComponent.self) { mark(e) }
        // Moving a simulation root moves its whole world rigidly: nothing changes in root space.
        if e.components.has(B3PhysicsSimulationComponent.self) { return }
        for c in e.children { markSubtree(c) }
    }

    func fullScan(_ entities: [Entity], deltaTime: Double) {
        var groups: [Key: (root: Entity?, entities: [Entity])] = [:]
        members.removeAll(keepingCapacity: true)
        for entity in entities {
            let (key, root) = key(for: entity)
            groups[key, default: (root, [])].entities.append(entity)
            members[entity.id] = (entity, key)
        }
        dirty.removeAll(); removed.removeAll()
        needsFullScan = false
        let registry = B3WorldRegistry.shared
        for (key, group) in groups {
            registry.world(for: key, root: group.root, scene: sceneKey).update(entities: group.entities, deltaTime: deltaTime)
        }
        for (key, world) in registry.worlds(in: sceneKey) where groups[key] == nil {
            if registry.adoptIfMoved(key, from: sceneKey) { continue }
            world.update(entities: [], deltaTime: deltaTime)
            registry.dropIfOrphaned(key)
        }
    }

    func incremental(deltaTime: Double) {
        var changed: [Key: [Entity]] = [:]
        var gone: [Key: [Entity.ID]] = [:]
        var roots: [Key: Entity?] = [:]
        for id in removed {
            if let m = members.removeValue(forKey: id) { gone[m.key, default: []].append(id) }
        }
        for (id, entity) in dirty {
            guard entity.scene != nil, entity.components.has(B3PhysicsBodyComponent.self),
                  entity.components.has(B3CollisionComponent.self) else {
                if let m = members.removeValue(forKey: id) { gone[m.key, default: []].append(id) }
                continue
            }
            let (key, root) = key(for: entity)
            if let old = members[id], old.key != key { gone[old.key, default: []].append(id) }
            members[id] = (entity, key)
            changed[key, default: []].append(entity)
            roots[key] = root
        }
        dirty.removeAll(keepingCapacity: true)
        removed.removeAll(keepingCapacity: true)

        // Every world steps every frame, changed or not.
        let registry = B3WorldRegistry.shared
        var keys = Set(registry.worlds(in: sceneKey).map(\.key))
        keys.formUnion(changed.keys)
        for key in keys {
            // A root moved to another scene: its world and bodies move with it, untouched here.
            if registry.adoptIfMoved(key, from: sceneKey) { continue }
            let world = registry.world(for: key, root: roots[key] ?? nil, scene: sceneKey)
            world.update(changed: changed[key] ?? [], removed: gone[key] ?? [], deltaTime: deltaTime)
            if world.bodyCount == 0 { registry.dropIfOrphaned(key) }
        }
    }

    private func key(for entity: Entity) -> (Key, Entity?) {
        let root = B3PhysicsSimulationComponent.nearestSimulationEntity(for: entity)
        return (root.map { .root($0.id) } ?? .scene(sceneKey), root)
    }
}

@MainActor
final class B3WorldRegistry {
    static let shared = B3WorldRegistry()
    enum Key: Hashable { case root(Entity.ID), scene(ObjectIdentifier) }

    private(set) var worlds: [Key: B3World] = [:]
    private var sceneOf: [Key: ObjectIdentifier] = [:]
    private var trackers: [ObjectIdentifier: SceneTracker] = [:]

    func tracker(for scene: Scene, token: UUID) -> SceneTracker {
        let id = ObjectIdentifier(scene)
        // A tracker from an earlier scene at the same address is stale.
        if let t = trackers[id], t.token != token { release(scene: id) }
        if let t = trackers[id] { return t }
        let t = SceneTracker(scene: scene, token: token)
        trackers[id] = t
        return t
    }

    /// Called when a scene's system is released: destroys that scene's worlds, so closed scenes do
    /// not accumulate Box3D worlds (Box3D allows 128). Runs on the next main-actor turn, so a burst
    /// of scenes opened and closed without yielding can still hold worlds briefly. Ignored if a
    /// newer scene has taken the address since.
    func release(scene id: ObjectIdentifier, token: UUID) {
        guard trackers[id] == nil || trackers[id]?.token == token else { return }
        release(scene: id)
    }

    /// A root's world whose root entity is now in another scene changes owner, so the old scene
    /// neither removes its bodies nor releases it. True if it moved.
    func adoptIfMoved(_ key: Key, from scene: ObjectIdentifier) -> Bool {
        guard case .root = key, let root = worlds[key]?.root, let now = root.scene,
              ObjectIdentifier(now) != scene else { return false }
        moved(key, to: ObjectIdentifier(now))
        return true
    }

    /// Hands a world to the scene its root is in now. That scene rescans, so the world drops
    /// bodies removed before the move, which only the old scene heard about.
    private func moved(_ key: Key, to scene: ObjectIdentifier) {
        guard sceneOf[key] != scene else { return }
        sceneOf[key] = scene
        trackers[scene]?.needsFullScan = true
    }

    private func release(scene id: ObjectIdentifier) {
        trackers[id] = nil
        for (key, owner) in sceneOf where owner == id {
            if adoptIfMoved(key, from: id) { continue }
            worlds[key] = nil
            sceneOf[key] = nil
        }
    }

    func world(for key: Key, root: Entity?, scene: ObjectIdentifier) -> B3World {
        if let w = worlds[key] {
            moved(key, to: scene) // a root moved to another scene takes its world along
            return w
        }
        let w = B3World(root: root, settings: root?.components[B3PhysicsSimulationComponent.self] ?? .init(), registersStandalone: false)
        worlds[key] = w
        sceneOf[key] = scene
        return w
    }

    func worlds(in scene: ObjectIdentifier) -> [(key: Key, world: B3World)] {
        worlds.compactMap { sceneOf[$0.key] == scene ? ($0.key, $0.value) : nil }
    }

    /// Drops a root's world once its root lost the simulation component or left.
    func dropIfOrphaned(_ key: Key) {
        guard case .root = key, let w = worlds[key],
              w.root?.components.has(B3PhysicsSimulationComponent.self) != true || w.root?.scene == nil else { return }
        worlds[key] = nil
        sceneOf[key] = nil
    }

    /// Worlds created outside the system, such as in tests and headless tools, by root entity.
    private var standalone: [ObjectIdentifier: WeakWorld] = [:]
    final class WeakWorld { weak var world: B3World?; init(_ w: B3World) { world = w } }

    /// Standalone worlds without a root, found by the bodies they hold.
    private var rootless: [WeakWorld] = []

    func registerStandalone(_ world: B3World, root: Entity?) {
        standalone = standalone.filter { $0.value.world != nil }
        rootless.removeAll { $0.world == nil }
        if let root {
            standalone[ObjectIdentifier(root)] = WeakWorld(world)
        } else {
            rootless.append(WeakWorld(world))
        }
    }

    func world(containing entity: Entity) -> B3World? {
        if let root = B3PhysicsSimulationComponent.nearestSimulationEntity(for: entity) {
            return worlds[.root(root.id)] ?? standalone[ObjectIdentifier(root)]?.world
        }
        // A standalone world whose root has no simulation component: walk up to find it.
        var e: Entity? = entity
        while let current = e {
            if let w = standalone[ObjectIdentifier(current)]?.world { return w }
            e = current.parent
        }
        if let scene = entity.scene, let w = worlds[.scene(ObjectIdentifier(scene))] { return w }
        if let w = rootless.lazy.compactMap({ $0.world }).first(where: { $0.records[entity.id] != nil }) { return w }
        return worlds.values.first { $0.records[entity.id] != nil }
    }
}

public enum RealityBox3D {
    /// Registers the components and the system. Call once, before creating entities.
    @MainActor public static func register() {
        routeBox3DLog()
        B3PhysicsBodyComponent.registerComponent()
        B3CollisionComponent.registerComponent()
        B3PhysicsMotionComponent.registerComponent()
        B3PhysicsSimulationComponent.registerComponent()
        B3PhysicsJointsComponent.registerComponent()
        B3CharacterControllerComponent.registerComponent()
        B3CharacterControllerStateComponent.registerComponent()
        B3CharacterObstacleComponent.registerComponent()
        B3PhysicsSystem.registerSystem()
    }

    /// The Box3D world simulating `entity`, once the system has seen it.
    @MainActor public static func world(for entity: Entity) -> B3World? {
        B3WorldRegistry.shared.world(containing: entity)
    }
}

/// Sends Box3D's messages, such as a hull simplified to fit its limits, to the unified log
/// instead of standard output.
func routeBox3DLog() {
    b3SetLogFcn { message in
        guard let message else { return }
        log.debug("Box3D: \(String(cString: message), privacy: .public)")
    }
}

import Box3DDemoScenes
import Foundation
import Observation
import RealityBox3D
import RealityKit
import simd

/// State of the demo window: the simulation root, the selected scene and the orbit camera.
@MainActor
@Observable
final class DemoModel {
    /// Parent of the scene, which `B3PhysicsSystem` advances every frame.
    @ObservationIgnored let root = Entity()
    @ObservationIgnored let cameraEntity = DemoStage.makeCamera()
    @ObservationIgnored private var camera = DemoScene.all[0].camera

    /// Scenes grouped by upstream category, computed once so the sidebar never filters inline.
    @ObservationIgnored let categories: [(name: String, scenes: [DemoScene])] = {
        var order: [String] = []
        var groups: [String: [DemoScene]] = [:]
        for s in DemoScene.all {
            if groups[s.category] == nil { order.append(s.category) }
            groups[s.category, default: []].append(s)
        }
        return order.map { ($0, groups[$0]!) }
    }()

    private(set) var scene = DemoScene.all[0]

    /// The root's simulation settings, applied on change.
    var settings = B3PhysicsSimulationComponent() {
        didSet { root.components.set(settings) }
    }

    var isPaused: Bool {
        get { settings.isPaused }
        set { settings.isPaused = newValue }
    }

    static let performanceCores: Int = {
        var n: Int32 = 0
        var size = MemoryLayout<Int32>.size
        sysctlbyname("hw.perflevel0.physicalcpu", &n, &size, nil, 0)
        return max(1, Int(n))
    }()

    init() {
        // One worker: on the engine comparison, more pay off only from about a thousand bodies, and
        // added spikes when the machine was busy. Raise it with the stepper for the big benchmarks.
        settings.workerCount = 1
        root.components.set(settings)
    }

    func load(_ id: DemoScene.ID) {
        guard let next = DemoScene.all.first(where: { $0.id == id }) else { return }
        scene = next
        reset()
        camera = next.camera
        camera.place(cameraEntity)
    }

    /// Rebuilds the scene. The old bodies leave the query, so their Box3D bodies are destroyed
    /// on the next frame and the new ones start from the scene's initial state.
    func reset() {
        let keep = settings
        scene.load(into: root)
        var s = root.components[B3PhysicsSimulationComponent.self] ?? keep
        s.workerCount = keep.workerCount
        s.interpolatesPoses = keep.interpolatesPoses
        s.isContinuousEnabled = keep.isContinuousEnabled
        s.subStepCount = keep.subStepCount
        s.isPaused = keep.isPaused
        settings = s
    }

    func step() {
        var events: [any B3CollisionEvent] = []
        RealityBox3D.world(for: root)?.step(events: &events)
    }

    func perform(_ action: String) {
        DemoScene.perform(action, root: root)
    }

    var statistics: B3Statistics { RealityBox3D.world(for: root)?.statistics ?? .init() }

    // MARK: Camera

    func placeCamera() { camera.place(cameraEntity) }

    func orbit(by delta: CGSize) {
        camera.yaw -= Float(delta.width) * 0.006
        camera.pitch = min(max(camera.pitch + Float(delta.height) * 0.006, -1.4), 1.5)
        camera.place(cameraEntity)
    }

    func zoom(by factor: Float) {
        camera.distance = min(max(camera.distance / factor, 0.5), 1000)
        camera.place(cameraEntity)
    }

    func ray(through point: CGPoint, in size: CGSize) -> (origin: SIMD3<Float>, direction: SIMD3<Float>) {
        camera.ray(through: point, in: size)
    }

    // MARK: Dragging bodies

    /// Starts dragging the dynamic body under a point. Returns its distance from the camera.
    func beginDrag(at point: CGPoint, in size: CGSize) -> Float? {
        let ray = ray(through: point, in: size)
        guard let world = RealityBox3D.world(for: root),
              let hit = world.raycast(origin: ray.origin, direction: ray.direction, length: 2000, query: .nearest).first,
              hit.entity.components[B3PhysicsBodyComponent.self]?.mode == .dynamic else { return nil }
        let local = hit.entity.convert(position: hit.position, from: nil)
        DragSystem.drag = .init(entity: hit.entity, localAnchor: local, target: hit.position)
        return simd_distance(hit.position, ray.origin)
    }

    func drag(to point: CGPoint, in size: CGSize, distance: Float) {
        let ray = ray(through: point, in: size)
        DragSystem.drag?.target = ray.origin + ray.direction * distance
    }

    func endDrag() { DragSystem.drag = nil }
}

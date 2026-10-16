import AppKit
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

    // MARK: Keys, for scenes with controls

    @ObservationIgnored private var keys: Set<UInt16> = []
    @ObservationIgnored private var shift = false
    @ObservationIgnored private var keyMonitor: Any?
    @ObservationIgnored private var focusObservers: [any NSObjectProtocol] = []
    @ObservationIgnored var frameSubscription: EventSubscription?
    /// The window showing this model; key events of other windows are left alone.
    @ObservationIgnored weak var window: NSWindow?

    /// Upstream's WASD, space and shift, read only while the scene has controls.
    func installKeyMonitor() {
        guard keyMonitor == nil else { return }
        keyMonitor = NSEvent.addLocalMonitorForEvents(matching: [.keyDown, .keyUp, .flagsChanged]) { [weak self] event in
            guard let self else { return event }
            return self.handle(event) // nil when it used the key
        }
        // A key released while another window or app has focus never arrives: forget them all.
        let center = NotificationCenter.default
        for name in [NSWindow.didResignKeyNotification, NSApplication.didResignActiveNotification] {
            focusObservers.append(center.addObserver(forName: name, object: nil, queue: .main) { [weak self] note in
                let window = note.object as? NSWindow, isApp = note.object is NSApplication
                MainActor.assumeIsolated {
                    guard let self, isApp || (window != nil && window === self.window) else { return }
                    self.releaseKeys()
                }
            })
        }
    }

    func removeKeyMonitor() {
        if let keyMonitor { NSEvent.removeMonitor(keyMonitor) }
        keyMonitor = nil
        for o in focusObservers { NotificationCenter.default.removeObserver(o) }
        focusObservers = []
        releaseKeys()
    }

    private func handle(_ event: NSEvent) -> NSEvent? {
        guard scene.controls != nil, let window, event.window === window else { return event }
        if event.type == .flagsChanged {
            shift = event.modifierFlags.contains(.shift)
            pushInput()
            return event
        }
        // W, A, S, D and space; other keys keep the window's shortcuts
        let tracked: Set<UInt16> = [13, 0, 1, 2, 49]
        guard tracked.contains(event.keyCode) else { return event }
        if event.type == .keyUp {
            // A release always counts, whatever modifiers are held by then
            keys.remove(event.keyCode)
            pushInput()
            return nil
        }
        guard event.modifierFlags.isDisjoint(with: [.command, .control, .option]) else { return event }
        keys.insert(event.keyCode)
        pushInput()
        return nil
    }

    private func releaseKeys() {
        keys = []
        shift = false
        pushInput()
    }

    private func pushInput() {
        func held(_ code: UInt16) -> Float { keys.contains(code) ? 1 : 0 }
        var input = DemoInput()
        input.throttle = [held(13) - held(1), held(2) - held(0)]
        input.jump = keys.contains(49)
        input.sprint = shift
        DemoScene.setInput(input, cameraForward: camera.target - camera.eye, root: root)
    }

    /// Every frame: keeps the camera on the entity the scene follows.
    func followCamera() {
        guard let target = DemoScene.followTarget(root: root) else { return }
        camera.target = target
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
        keys = []
        pushInput()
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
        pushInput()
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

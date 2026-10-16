import CoreGraphics
import RealityBox3D
import RealityKit
import simd

/// A scene of Box3D's sample app, rebuilt on RealityBox3D's public API with upstream's
/// positions, sizes, densities and materials. `upstream` names the sample it ports.
public struct DemoScene: Identifiable, Sendable {
    public var id: String { "\(category)/\(name)" }
    public let category: String
    public let name: String
    /// Upstream source file in box3d/samples.
    public let upstream: String
    public let summary: String
    /// Upstream camera, as `SetView(yaw, pitch, distance, target)`.
    public let camera: OrbitCamera
    /// Upstream disables sleeping in some benchmarks.
    public var isSleepEnabled = true
    /// Buttons the scene offers, upstream's keyboard actions.
    public var actions: [String] = []
    /// Keys the scene reads through `DemoContext.input`, described for the app's sidebar.
    public var controls: String?
    let build: @MainActor @Sendable (inout DemoBuilder) -> Void

    init(_ category: String, _ name: String, upstream: String, summary: String, camera: OrbitCamera,
         isSleepEnabled: Bool = true, actions: [String] = [], controls: String? = nil, build: @escaping @MainActor @Sendable (inout DemoBuilder) -> Void) {
        self.category = category
        self.name = name
        self.upstream = upstream
        self.summary = summary
        self.camera = camera
        self.isSleepEnabled = isSleepEnabled
        self.actions = actions
        self.controls = controls
        self.build = build
    }

    /// Replaces the bodies under `root` with the scene's, and sets upstream's world settings on
    /// the root's simulation component.
    @MainActor
    public func load(into root: Entity) {
        var builder = DemoBuilder()
        build(&builder)
        for child in Array(root.children) { child.removeFromParent() }
        root.addChild(builder.content)
        var settings = root.components[B3PhysicsSimulationComponent.self] ?? .init()
        configure(&settings)
        root.components.set(settings)
        if builder.joints.isEmpty {
            root.components.remove(B3PhysicsJointsComponent.self)
        } else {
            root.components.set(B3PhysicsJointsComponent(joints: builder.joints))
        }
    }

    /// Box3D's defaults, which its samples use: gravity of 10 m/s², 60 Hz, 4 substeps.
    public func configure(_ settings: inout B3PhysicsSimulationComponent) {
        settings.gravity = [0, -10, 0]
        settings.fixedRate = 60
        settings.subStepCount = 4
        settings.isSleepEnabled = isSleepEnabled
    }

    public static var all: [DemoScene] {
        benchmarks + stacking + shapes + surfaces + restitution + bodies + joints + vehicles + characters + continuous + events + meshes
    }

    public static func named(_ name: String) -> DemoScene? {
        all.first { $0.name.caseInsensitiveCompare(name) == .orderedSame || $0.id.caseInsensitiveCompare(name) == .orderedSame }
    }
}

/// Upstream's orbit camera: `eye = target + distance * (sin yaw cos pitch, sin pitch, cos yaw cos pitch)`.
public struct OrbitCamera: Equatable, Sendable {
    public var target: SIMD3<Float>
    public var distance: Float
    public var yaw: Float
    public var pitch: Float
    public static let fieldOfView: Float = 60

    /// Angles in degrees, as upstream's `SetView`.
    public init(yaw: Float, pitch: Float, distance: Float, target: SIMD3<Float>) {
        self.yaw = yaw * .pi / 180
        self.pitch = pitch * .pi / 180
        self.distance = distance
        self.target = target
    }

    public var eye: SIMD3<Float> {
        target + SIMD3(cos(pitch) * sin(yaw), sin(pitch), cos(pitch) * cos(yaw)) * distance
    }

    @MainActor public func place(_ camera: Entity) {
        camera.look(at: target, from: eye, relativeTo: nil)
    }

    /// Ray through a point of a view of the given size, y pointing down.
    public func ray(through point: CGPoint, in size: CGSize) -> (origin: SIMD3<Float>, direction: SIMD3<Float>) {
        let forward = normalize(target - eye)
        let right = normalize(cross(forward, [0, 1, 0]))
        let up = cross(right, forward)
        let t = tan(Self.fieldOfView * .pi / 360)
        let aspect = Float(size.width / max(size.height, 1))
        let x = Float(2 * point.x / max(size.width, 1) - 1)
        let y = Float(1 - 2 * point.y / max(size.height, 1))
        return (eye, normalize(forward + right * (x * t * aspect) + up * (y * t)))
    }
}

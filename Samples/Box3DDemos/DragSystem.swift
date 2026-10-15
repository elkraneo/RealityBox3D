import RealityBox3D
import RealityKit
import simd

/// Pulls a grabbed body's anchor point toward the pointer with a critically damped spring,
/// as upstream's mouse joint does: stiffness and damping scale with the body's mass.
struct DragSystem: System {
    struct Drag {
        let entity: Entity
        let localAnchor: SIMD3<Float>
        var target: SIMD3<Float>
    }

    @MainActor static var drag: Drag?

    static var dependencies: [SystemDependency] { [.before(B3PhysicsSystem.self)] }

    init(scene: RealityKit.Scene) {}

    func update(context: SceneUpdateContext) {
        guard let drag = Self.drag, let mass = drag.entity.b3.mass, let v = drag.entity.b3.linearVelocity else { return }
        let anchor = drag.entity.convert(position: drag.localAnchor, to: nil)
        let hertz: Float = 5, damping: Float = 0.7
        let omega = 2 * Float.pi * hertz
        let force = mass * (omega * omega * (drag.target - anchor) - 2 * damping * omega * v)
        // Cancel gravity on the grabbed body so it does not sag below the pointer.
        let gravity = mass * SIMD3<Float>(0, 10, 0)
        drag.entity.b3.addForce(force + gravity, at: anchor, relativeTo: nil)
    }
}

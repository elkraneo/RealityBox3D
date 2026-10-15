import RealityKit
import simd

/// Lighting and camera shared by the demo app and the offscreen tools.
@MainActor
public enum DemoStage {
    /// A key light with shadows plus two fills, so faces turned from the key are not black
    /// in renderers without image-based lighting.
    public static func makeLights() -> Entity {
        let rig = Entity()
        let key = DirectionalLight()
        key.light.intensity = 2600
        key.shadow = DirectionalLightComponent.Shadow(maximumDistance: 150, depthBias: 2)
        key.look(at: .zero, from: [20, 40, 30], relativeTo: nil)
        let fill = DirectionalLight()
        fill.light.intensity = 1100
        fill.look(at: .zero, from: [-30, 15, -10], relativeTo: nil)
        let rim = DirectionalLight()
        rim.light.intensity = 600
        rim.look(at: .zero, from: [10, -5, -40], relativeTo: nil)
        for l in [key, fill, rim] { rig.addChild(l) }
        return rig
    }

    public static func makeCamera() -> PerspectiveCamera {
        let c = PerspectiveCamera()
        c.camera.fieldOfViewInDegrees = OrbitCamera.fieldOfView
        c.camera.fieldOfViewOrientation = .vertical
        c.camera.near = 0.02
        c.camera.far = 2000
        return c
    }
}

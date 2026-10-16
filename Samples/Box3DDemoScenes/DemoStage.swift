import RealityKit
import simd

/// Lighting and camera shared by the demo app and the offscreen tools.
@MainActor
public enum DemoStage {
    /// A key light with shadows plus three fills, so faces turned from the key, downward ones
    /// included, are not black in renderers without image-based lighting.
    public static func makeLights() -> Entity {
        let rig = Entity()
        let key = DirectionalLight()
        key.name = "key"
        key.light.intensity = 2600
        key.shadow = DirectionalLightComponent.Shadow(maximumDistance: 40, depthBias: 1)
        key.look(at: .zero, from: [20, 40, 30], relativeTo: nil)
        let fill = DirectionalLight()
        fill.light.intensity = 1100
        fill.look(at: .zero, from: [-30, 15, -10], relativeTo: nil)
        let rim = DirectionalLight()
        rim.light.intensity = 600
        rim.look(at: .zero, from: [10, -5, -40], relativeTo: nil)
        // From below: the undersides of arches and overhangs face away from all the others
        let under = DirectionalLight()
        under.light.intensity = 350
        under.look(at: .zero, from: [5, -40, 10], relativeTo: nil)
        for l in [key, fill, rim, under] { rig.addChild(l) }
        return rig
    }

    /// Fits the key light's shadows to what the camera sees: a shadow map spread over 150 m
    /// leaves a small level blocky and banded, one over a few meters cuts off a big pile.
    public static func fitShadows(_ rig: Entity, to camera: OrbitCamera) {
        guard let key = rig.findEntity(named: "key") as? DirectionalLight, var shadow = key.shadow else { return }
        let distance = min(max(camera.distance * 4, 15), 150)
        guard abs(shadow.maximumDistance - distance) > 0.1 * distance else { return }
        shadow.maximumDistance = distance
        key.shadow = shadow
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

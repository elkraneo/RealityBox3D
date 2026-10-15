import RealityBox3D
import RealityKit
import simd

extension DemoScene {
    static let shapes: [DemoScene] = [
        DemoScene("Shapes", "Inclined Plane", upstream: "samples/sample_shapes.cpp InclinedPlane",
                  summary: "Five boxes on a 40° slope, friction rising from 0.04 to 1. Only the last ones hold.",
                  camera: .init(yaw: -55, pitch: 30, distance: 60, target: [0, 7.5, 0])) { b in
            b.ground(extent: 50)
            b.body(.generateBox(size: [32, 1, 20]), at: [0, 7.5, -5], rotation: simd_quatf(angle: 40 * .pi / 180, axis: [1, 0, 0]),
                   mode: .static, friction: 1)
            for i in 0..<5 {
                b.body(.generateBox(size: [2, 2, 2]), at: [-10 + 5 * Float(i), 14.25, -10.6],
                       rotation: simd_quatf(angle: 40 * .pi / 180, axis: [1, 0, 0]),
                       friction: Float((i + 1) * (i + 1)) * 0.04)
            }
        },

        DemoScene("Shapes", "Rolling Resistance", upstream: "samples/sample_shapes.cpp RollingResistance",
                  summary: "Spheres and capsules roll down a 10° ramp, rolling resistance rising from 0 to 0.2.",
                  camera: .init(yaw: -140, pitch: 17, distance: 60, target: [0, 7.5, 0])) { b in
            b.ground(extent: 80)
            b.body(.generateBox(size: [64, 1, 30]), at: [0, 2, -20], rotation: simd_quatf(angle: 10 * .pi / 180, axis: [1, 0, 0]),
                   mode: .static)
            let capsule = B3ShapeResource.generateCapsule(height: 3, radius: 0.5).offsetBy(rotation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1]))
            for i in 0..<5 {
                b.body(.generateSphere(radius: 1), at: [-25 + 5 * Float(i), 8, -24], rollingResistance: 0.05 * Float(i))
                b.body(capsule, at: [2 + 5 * Float(i), 8, -24], rollingResistance: 0.05 * Float(i))
            }
        },
    ]

    static let restitution: [DemoScene] = [
        DemoScene("Restitution", "Varying", upstream: "samples/sample_restitution.cpp VaryingRestitution",
                  summary: "40 frictionless spheres dropped from 40 m, restitution rising from 0 to 1.",
                  camera: .init(yaw: 0, pitch: 25, distance: 85, target: [0, 20, 0]), actions: ["Spheres", "Boxes"]) { b in
            b.ground(extent: 50)
            let count = 40
            let holder = b.content
            @MainActor func drop(_ shape: B3ShapeResource, into builder: inout DemoBuilder) {
                for i in 0..<count {
                    let r = Float(i) / Float(count - 1)
                    builder.body(shape, at: [-Float(count - 1) + 2 * Float(i), 40, 0], density: 1, friction: 0, restitution: r, name: "drop")
                }
            }
            drop(.generateSphere(radius: 0.5), into: &b)
            for (name, shape) in [("Spheres", B3ShapeResource.generateSphere(radius: 0.5)), ("Boxes", .generateBox(size: [1, 1, 1]))] {
                b.action(name) { [weak holder] _ in
                    guard let holder else { return }
                    for e in holder.children where e.name == "drop" { e.removeFromParent() }
                    var fresh = DemoBuilder()
                    drop(shape, into: &fresh)
                    for e in Array(fresh.content.children) { holder.addChild(e) }
                }
            }
        },
    ]
}

import RealityBox3D
import RealityKit
import simd

extension DemoScene {
    static let meshes: [DemoScene] = [
        DemoScene("Mesh", "Grid", upstream: "samples/sample_mesh.cpp GridMesh",
                  summary: "A cylinder hull drops onto a 40 m triangle-mesh grid, scaled by two as upstream.",
                  camera: .init(yaw: 45, pitch: 30, distance: 6, target: .zero), actions: ["Sphere", "Capsule", "Box", "Cylinder"]) { b in
            let n = 20, cell: Float = 1, scale: Float = 2
            var positions: [SIMD3<Float>] = []
            for ix in 0...n {
                for iz in 0...n {
                    positions.append(SIMD3(-0.5 * Float(n) * cell + Float(ix) * cell, 0, -0.5 * Float(n) * cell + Float(iz) * cell) * scale)
                }
            }
            var indices: [UInt32] = []
            for ix in 0..<n {
                for iz in 0..<n {
                    let a = UInt32(ix * (n + 1) + iz), c = a + UInt32(n + 1)
                    indices += [a, a + 1, c, c, a + 1, c + 1]
                }
            }
            b.body(.generateStaticMesh(positions: positions, faceIndices: indices), at: .zero, mode: .static)

            let holder = b.content
            let shapes: [(String, B3ShapeResource, Float, Float)] = [
                ("Sphere", .generateSphere(radius: 0.5), 0.05, 0),
                ("Capsule", B3ShapeResource.generateCapsule(height: 1.1, radius: 0.15)
                    .offsetBy(rotation: simd_quatf(angle: .pi / 2, axis: [1, 0, 0]), translation: [0, 0, 0.876]), 0.05, 0),
                ("Box", .generateBox(size: [1, 1, 1]), 0, 0),
                ("Cylinder", B3ShapeResource.generateCylinder(height: 1, radius: 0.25, sides: 15).offsetBy(translation: [0, 0.5, 0]), 0.02, 0.1),
            ]
            @MainActor func spawn(_ shape: B3ShapeResource, rolling: Float, damping: Float, into builder: inout DemoBuilder) {
                builder.body(shape, at: [0.1, 1, -0.1], rollingResistance: rolling, name: "dropped") { $0.angularDamping = damping }
            }
            spawn(shapes[3].1, rolling: shapes[3].2, damping: shapes[3].3, into: &b)
            for (name, shape, rolling, damping) in shapes {
                b.action(name) { [weak holder] _ in
                    guard let holder else { return }
                    for e in holder.children where e.name == "dropped" { e.removeFromParent() }
                    var fresh = DemoBuilder()
                    spawn(shape, rolling: rolling, damping: damping, into: &fresh)
                    for e in Array(fresh.content.children) { holder.addChild(e) }
                }
            }
        },
    ]
}

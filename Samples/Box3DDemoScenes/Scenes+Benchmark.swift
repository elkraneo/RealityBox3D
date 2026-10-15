import RealityBox3D
import RealityKit
import simd

extension DemoScene {
    static let benchmarks: [DemoScene] = [
        DemoScene("Benchmark", "Large Pyramid", upstream: "shared/benchmarks.c CreateLargePyramid",
                  summary: "5,050 boxes in a 2D pyramid, sleeping off. Box3D's main stacking benchmark.",
                  camera: .init(yaw: 40, pitch: -10, distance: 110, target: [0, 40, 0]), isSleepEnabled: false) { b in
            b.body(.generateBox(size: [800, 2, 800]), at: [0, -1, 0], mode: .static, name: "ground")
            let baseCount = 100
            let h: Float = 0.5
            let box = B3ShapeResource.generateBox(size: [2 * h, 2 * h, 2 * h])
            for i in 0..<baseCount {
                let y = (2 * Float(i) + 1) * h
                for j in i..<baseCount {
                    let x = (Float(i) + 1) * h + 2 * Float(j - i) * h - h * Float(baseCount)
                    b.body(box, at: [x, y, 0], density: 100)
                }
            }
        },

        DemoScene("Benchmark", "Wide Pyramid", upstream: "shared/benchmarks.c CreateWidePyramid",
                  summary: "A 15-level 3D pyramid of boxes spaced half a box apart, so it falls and settles.",
                  camera: .init(yaw: 0, pitch: 5, distance: 80, target: [0, 18, 0])) { b in
            b.body(.generateBox(size: [200, 2, 200]), at: [0, -1, 0], mode: .static, name: "ground")
            let boxSize: Float = 2, separation: Float = 0.5, half = boxSize / 2, height = 15
            let h = half - 0.025
            let box = B3ShapeResource.generateBox(size: [2 * h, 2 * h, 2 * h])
            for i in 0..<height {
                for j in (i / 2)..<(height - (i + 1) / 2) {
                    for k in (i / 2)..<(height - (i + 1) / 2) {
                        let odd: Float = i & 1 == 1 ? half : 0
                        let x = -Float(height) + boxSize * Float(j) + odd
                        let y = 1 + (boxSize + separation) * Float(i)
                        let z = -Float(height) + boxSize * Float(k) + odd
                        b.body(box, at: [x, y, z])
                    }
                }
            }
        },

        DemoScene("Benchmark", "Convex Pile", upstream: "shared/benchmarks.c CreateConvexPile",
                  summary: "5,120 irregular 32-point hulls dropped in an 8×8 grid, 80 layers tall.",
                  camera: .init(yaw: 45, pitch: 20, distance: 150, target: [0, 15, 0])) { b in
            b.body(.generateBox(size: [500, 2, 500]), at: [0, -1, 0], mode: .static, name: "ground")
            let count = 8, layers = 80
            let amplitude: Float = 2, scatter = 2 * amplitude
            let hull = B3ShapeResource.generateComplexHull(radius: amplitude)
            for layer in 0..<layers {
                for z in 0..<count {
                    for x in 0..<count {
                        let p = SIMD3<Float>((Float(x) - 0.5 * Float(count)) * scatter,
                                             amplitude + 2 * amplitude * Float(layer),
                                             (Float(z) - 0.5 * Float(count)) * scatter)
                        b.body(hull, at: p)
                    }
                }
            }
        },

        DemoScene("Benchmark", "Falling Boxes", upstream: "samples/sample_benchmark.cpp FallingBoxes",
                  summary: "3,200 unit cubes dropped in an 8×8 grid, 50 layers tall.",
                  camera: .init(yaw: 45, pitch: 10, distance: 80, target: [0, 20, 0])) { b in
            b.ground(extent: 100)
            let n = 50, a: Float = 0.5
            let box = B3ShapeResource.generateBox(size: [2 * a, 2 * a, 2 * a])
            for i in 0..<n {
                for j in 0..<8 {
                    for k in 0..<8 {
                        b.body(box, at: [-16 * a + 4 * a * Float(j), 4 * a * Float(i) + 5 * a, -16 * a + 4 * a * Float(k)])
                    }
                }
            }
        },
    ]
}

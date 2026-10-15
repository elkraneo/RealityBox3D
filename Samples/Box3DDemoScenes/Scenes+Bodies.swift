import RealityBox3D
import RealityKit
import simd

extension DemoScene {
    static let bodies: [DemoScene] = [
        DemoScene("Bodies", "Gyroscopic Torque", upstream: "samples/sample_bodies.cpp GyroscopicTorque",
                  summary: "A T-handle spinning about its unstable axis flips over and over: the Dzhanibekov effect.",
                  camera: .init(yaw: 0, pitch: 20, distance: 4, target: [0, 2, 0])) { b in
            b.ground(extent: 20)
            // Upstream's b3CreateCylinder starts at the body origin and rises 0.6 m: the handle's stem.
            b.body([B3ShapeResource.generateCylinder(height: 0.6, radius: 0.15, sides: 32).offsetBy(translation: [0, 0.3, 0]),
                    .generateBox(size: [2, 0.1, 0.2])],
                   at: [0, 2, 0], rotation: simd_quatf(angle: -0.5 * .pi, axis: [1, 0, 0]),
                   angularVelocity: [0.01, 0.01, 10]) { $0.isAffectedByGravity = false }
        },

        DemoScene("Bodies", "Spinning Book", upstream: "samples/sample_bodies.cpp SpinningBooks",
                  summary: "Three books spun about each principal axis. Only the middle axis is unstable.",
                  camera: .init(yaw: 0, pitch: 30, distance: 10, target: [0, 1, 0])) { b in
            b.ground(extent: 10)
            let book = B3ShapeResource.generateBox(size: [0.7, 0.16, 1])
            for (x, w) in [(Float(-2), SIMD3<Float>(5, 0.01, 0.01)), (0, [0.01, 5, 0.01]), (2, [0.01, 0.01, -5])] {
                b.body(book, at: [x, 2, 0], angularVelocity: w) { $0.isAffectedByGravity = false }
            }
        },

        DemoScene("Bodies", "Weeble", upstream: "samples/sample_bodies.cpp Weeble",
                  summary: "A capsule with its center of mass 1.5 m below its middle always rights itself. Push tips it over.",
                  camera: .init(yaw: 45, pitch: 25, distance: 25, target: .zero), actions: ["Push"]) { b in
            b.ground(extent: 30)
            let weeble = b.body(.generateCapsule(height: 4, radius: 1), at: [0, 3, 0], rollingResistance: 0.1) { body in
                // Upstream keeps the shape's mass, moves the center down and adds the parallel-axis term.
                let mass = 1000 * (Float.pi * 1 * 1 * 2 + 4 / 3 * Float.pi) // Box3D's default density times the capsule's volume
                body.massProperties = .init(mass: mass, centerOfMass: ([0, -1.5, 0], simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)))
            }
            b.action("Push") { _ in
                weeble.b3.applyImpulse([20_000, 0, 0], at: weeble.position(relativeTo: nil) + [0, 2, 0], relativeTo: nil)
            }
        },

        DemoScene("Bodies", "Kinematic", upstream: "samples/sample_bodies.cpp Kinematic",
                  summary: "A kinematic paddle follows a figure-eight after two seconds, driven by its entity's pose.",
                  camera: .init(yaw: 0, pitch: 30, distance: 10, target: [0, 1.5, 0])) { b in
            b.ground(extent: 20)
            let amplitude: Float = 2
            let paddle = b.body(.generateBox(size: [0.2, 2, 0.4]), at: [2 * amplitude, amplitude + 1, 0], mode: .kinematic)
            b.onFrame { ctx in
                let delay = 2.0
                guard ctx.time > delay else { return }
                let t = Float(ctx.time - delay)
                paddle.position = [2 * amplitude * cos(t), amplitude * (sin(2 * t) + 1) + 1, 0]
                paddle.orientation = simd_quatf(angle: 2 * t, axis: [0, 0, 1])
            }
        },
    ]
}

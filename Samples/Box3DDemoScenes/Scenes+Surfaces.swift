import RealityBox3D
import RealityKit
import simd

extension DemoScene {
    static let surfaces: [DemoScene] = [
        DemoScene("Shapes", "Conveyor Belt", upstream: "samples/sample_shapes.cpp ConveyorBelt",
                  summary: "A turned platform whose surface moves at 2 m/s along its length carries five dropped boxes off its end.",
                  camera: .init(yaw: 0, pitch: 25, distance: 40, target: [0, 1, 0])) { b in
            b.ground(extent: 20)
            b.body(.generateBox(size: [20, 0.5, 4]), at: [-5, 5, 0], rotation: simd_quatf(angle: 0.2, axis: [0, 1, 0]),
                   mode: .static, friction: 0.8, name: "belt") { $0.material.tangentVelocity = [2, 0, 0] }
            for i in 0..<5 {
                b.body(.generateBox(size: [1, 1, 1]), at: [-10 + 2 * Float(i), 7, 0])
            }
        },

        DemoScene("Shapes", "Wind", upstream: "samples/sample_shapes.cpp Wind",
                  summary: "A chain of ten light plates on ball joints hangs in a gusting 6 m/s wind, with drag and lift on each shape. Change the shape or the wind with the buttons.",
                  camera: .init(yaw: 0, pitch: 0, distance: 5, target: [0, 1, 0]),
                  actions: ["Sphere", "Capsule", "Box", "Stronger", "Weaker", "Reverse"]) { b in
            b.ground(extent: 20)
            let radius: Float = 0.1, height: Float = 2
            let shapes: [String: B3ShapeResource] = [
                "Sphere": .generateSphere(radius: radius),
                "Capsule": capsule([-radius, 0, 0], [radius, 0, 0], radius: 0.5 * radius),
                "Box": .generateBox(size: [2.5 * radius, 1.5 * radius, 0.25 * radius]),
            ]
            var links: [Entity] = []
            var previous = b.anchor("top", at: [0, height, 0])
            for i in 0..<10 {
                let link = b.body(shapes["Box"]!, at: [(2 * Float(i) + 1) * radius, height, 0], density: 20, name: "link\(i)") {
                    $0.gravityScale = 0.5
                    $0.isSleepEnabled = false
                }
                b.joints.append(B3PhysicsSphericalJoint(pin0: previous, pin1: link.pins.set(named: "a", position: [-radius, 0, 0])))
                previous = link.pins.set(named: "b", position: [radius, 0, 0])
                links.append(link)
            }

            // Upstream's gusts: the direction wanders by noise eased toward a random offset
            final class Weather { var wind: SIMD3<Float> = [6, 0, 0]; var noise: SIMD3<Float> = .zero; var random = SeededRandom(seed: 7) }
            let weather = Weather()
            b.onFrame { ctx in
                guard ctx.deltaTime > 0 else { return }
                let speed = simd_length(weather.wind)
                let gust = speed > 0 ? speed * (weather.wind / speed + weather.noise) : .zero
                for link in links { link.b3.applyWind(gust, drag: 1, lift: 0.75, maxSpeed: 10) }
                let target = SIMD3<Float>(Float.random(in: -0.3...0.3, using: &weather.random),
                                          Float.random(in: -0.3...0.3, using: &weather.random),
                                          Float.random(in: -0.3...0.3, using: &weather.random))
                weather.noise += 0.05 * (target - weather.noise)
            }
            for (name, shape) in shapes {
                b.action(name) { _ in
                    for link in links {
                        link.components[B3CollisionComponent.self]?.shapes = [shape]
                        if let mesh = try? MeshResource.generate(from: shape) {
                            for case let model as ModelEntity in link.children { model.model?.mesh = mesh }
                        }
                    }
                }
            }
            // Upstream's slider spans ±50 m/s
            b.action("Stronger") { _ in let x = weather.wind.x; weather.wind.x = x < 0 ? max(x - 6, -50) : min(x + 6, 50) }
            b.action("Weaker") { _ in let x = weather.wind.x; weather.wind.x = x < 0 ? min(x + 6, 0) : max(x - 6, 0) }
            b.action("Reverse") { _ in weather.wind.x = -weather.wind.x }
        },

        DemoScene("Joints", "Door", upstream: "samples/sample_joint.cpp Door",
                  summary: "A heavy door on two stiff hinges, sprung shut and limited to ±90°. Impulse kicks its free edge; One Hinge leaves it hanging on the bottom one.",
                  camera: .init(yaw: 45, pitch: 30, distance: 15, target: [0, 2, 0]),
                  actions: ["Impulse", "One Hinge", "Two Hinges"]) { b in
            b.ground(extent: 20)
            let door = b.body(.generateBox(size: [1.5, 3, 0.2]), at: [0, 1.5, 0], name: "door") { $0.gravityScale = 2 }
            let axis = pinOrientation(box3DFrame: simd_quatf(from: [0, 0, 1], to: [0, 1, 0]))
            var hinges: [B3PhysicsRevoluteJoint] = []
            for (name, y) in [("lower", Float(0)), ("upper", 3)] {
                hinges.append(B3PhysicsRevoluteJoint(pin0: b.anchor(name, at: [-0.75, y, 0], orientation: axis),
                                                     pin1: door.pins.set(named: name, position: [-0.75, y - 1.5, 0], orientation: axis),
                                                     angularLimit: -(.pi / 2)...(.pi / 2), spring: B3JointSpring(hertz: 1, dampingRatio: 0.5),
                                                     constraintHertz: 120))
            }
            b.joints = hinges
            b.action("Impulse") { _ in
                door.b3.applyImpulse([0, 0, -50000], at: door.convert(position: [0.75, 0, 0], to: nil), relativeTo: nil)
            }
            b.action("One Hinge") { ctx in ctx.root.components.set(B3PhysicsJointsComponent(joints: [hinges[0]])) }
            b.action("Two Hinges") { ctx in ctx.root.components.set(B3PhysicsJointsComponent(joints: hinges)) }
        },
    ]
}

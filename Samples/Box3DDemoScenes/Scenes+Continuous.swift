import RealityBox3D
import RealityKit
import simd

extension DemoScene {
    static let continuous: [DemoScene] = [
        DemoScene("Continuous", "Thin Wall", upstream: "samples/sample_continuous.cpp ThinWall",
                  summary: "A sphere, a capsule and a box at 180 m/s hit a 20 cm wall. Continuous collision stops all three.",
                  camera: .init(yaw: 45, pitch: 30, distance: 30, target: .zero)) { b in
            b.ground(extent: 40)
            b.body(.generateBox(size: [20, 0.2, 20]), at: [0, 10, 0], rotation: simd_quatf(angle: .pi / 2, axis: [1, 0, 0]), mode: .static)
            let v: SIMD3<Float> = [0, 0, -180]
            b.body(.generateSphere(radius: 0.1), at: [-5, 10, 20], rollingResistance: 0.1, linearVelocity: v, angularVelocity: [20, 0, 0])
            b.body(B3ShapeResource.generateCapsule(height: 0.8, radius: 0.1).offsetBy(rotation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1])),
                   at: [0, 10, 20], rollingResistance: 0.1, linearVelocity: v, angularVelocity: [20, -5, 0])
            b.body(.generateBox(size: [0.8, 0.2, 0.2]), at: [5, 10, 20], rollingResistance: 0.1, linearVelocity: v, angularVelocity: [20, 5, 0])
        },

        DemoScene("Continuous", "Bullet vs Stack", upstream: "samples/sample_continuous.cpp BulletVersusStack",
                  summary: "Launch fires a dense sphere at 500 m/s into a stack of ten boxes in front of a wall.",
                  camera: .init(yaw: 15, pitch: 20, distance: 30, target: [0, 2, 0]), actions: ["Launch"]) { b in
            b.ground(extent: 50)
            b.body(B3ShapeResource.generateBox(size: [0.2, 10, 20]).offsetBy(translation: [-1, 5, 0]), at: [0, -1, 0], mode: .static)
            for row in 0..<10 {
                b.body(.generateBox(size: [1, 1, 1]), at: [0, 0.5 + 1.1 * Float(row), 0])
            }
            let holder = b.content
            b.action("Launch") { [weak holder] _ in
                guard let holder else { return }
                for e in holder.children where e.name == "bullet" { e.removeFromParent() }
                var fresh = DemoBuilder()
                fresh.body(.generateSphere(radius: 0.25), at: [20.5, 5.5, 0], density: 10 * 1000, linearVelocity: [-500, 0, 0], name: "bullet") {
                    $0.isBullet = true
                }
                for e in Array(fresh.content.children) { holder.addChild(e) }
            }
        },
    ]

    static let events: [DemoScene] = [
        DemoScene("Events", "Sensor Visit", upstream: "samples/sample_events.cpp SensorVisit",
                  summary: "A box falls into a kinematic sensor, whose begin-touch event destroys it.",
                  camera: .init(yaw: 0, pitch: 30, distance: 20, target: [0, 5, 0])) { b in
            b.ground(extent: 10)
            let box = b.body(.generateBox(size: [1, 1, 1]), at: [0, 12.5, 0])
            let sensor = b.body(.generateBox(size: [4, 4, 4]), at: [0, 2, 0], mode: .kinematic, collisionMode: .trigger)
            b.onFrame { ctx in
                guard let world = ctx.world, let script = demoScript(ctx), script.flags.insert("subscribed").inserted else { return }
                script.retained.append(world.subscribe(to: B3CollisionEvents.Began.self, on: sensor) { event in
                    let visitor = event.entityA == sensor ? event.entityB : event.entityA
                    if visitor == box { visitor.removeFromParent() }
                })
            }
        },
    ]
}

/// The script of the scene loaded under the context's root.
@MainActor
func demoScript(_ ctx: DemoContext) -> DemoScript? {
    ctx.root.children.lazy.compactMap({ $0.components[DemoScriptComponent.self]?.script }).first
}

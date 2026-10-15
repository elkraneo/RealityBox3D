import Testing
import RealityKit
import simd
@testable import RealityBox3D
@testable import Box3DRealityKitSampleScenes

@MainActor
@Suite(.serialized) struct DiceTests {
    /// Tosses all dice and checks the forecast predicted exactly the faces that land on top.
    @Test(arguments: [false, true])
    func forecastPredictsTheRoll(maxScore: Bool) async throws {
        let scene = DiceScene()
        try await scene.load()
        let root = scene.simulationRoot
        let bodies = root.children.filter { $0.components.has(B3PhysicsBodyComponent.self) }
        let world = B3World(root: root, settings: root.components[B3PhysicsSimulationComponent.self]!)
        func frames(_ n: Int) { for _ in 0..<n { world.update(entities: bodies, deltaTime: 1.0 / 60) } }

        frames(60) // settle on the table
        scene.pickUp(die: 1, tossAll: true)
        for i in 0..<45 { // lift, then sweep toward +x
            scene.move(heldTo: [-6 + Float(i) * 0.6, 8, -4])
            frames(1)
        }
        scene.toss(maxScore: maxScore)
        var n = 0
        while scene.rolled == nil && n < 1200 { frames(1); n += 1 }

        if scene.rolled == nil {
            for d in scene.dieEntities {
                print("SLOW", d.name, "y", d.position.y, "r", length(SIMD2(d.position.x, d.position.z)), "v", d.b3.linearVelocity.map(length) ?? -1, "w", d.b3.angularVelocity.map(length) ?? -1, "awake", d.b3.isAwake, "predicted", scene.predicted != nil)
            }
        }
        let predicted = try #require(scene.predicted).map(\.value)
        let rolled = try #require(scene.rolled).map(\.value)
        #expect(predicted == rolled, "predicted \(predicted), rolled \(rolled) after \(n) frames")
        #expect(rolled.count == 7)
        if maxScore {
            let maxima = DieKind.allCases.map { $0.faces.map(\.value).max()! }
            #expect(rolled == maxima, "max score toss rolled \(rolled)")
        }
        // Resting on the table top at y = 0: the largest die is under 3 cm from its center to a face.
        for d in scene.dieEntities {
            #expect(d.position.y > 0 && d.position.y < 2.5, "\(d.name) not resting on the table: y = \(d.position.y)")
            #expect(length(SIMD2(d.position.x, d.position.z)) < 50, "\(d.name) left the table: \(d.position)")
        }
    }
}

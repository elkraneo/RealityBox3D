import Testing
import CBox3D
import RealityKit
@testable import Box3DDemoScenes
@testable import RealityBox3D

@MainActor @Test func weebleMassMatchesUpstream() throws {
    var wd = b3DefaultWorldDef(); wd.workerCount = 1
    let w = b3CreateWorld(&wd); defer { b3DestroyWorld(w) }
    var bd = b3DefaultBodyDef(); bd.type = b3_dynamicBody
    let b = b3CreateBody(w, &bd)
    var cap = b3Capsule(center1: b3Vec3(x: 0, y: -1, z: 0), center2: b3Vec3(x: 0, y: 1, z: 0), radius: 1)
    var sd = b3DefaultShapeDef(); _ = b3CreateCapsuleShape(b, &sd, &cap)
    let mass = b3Body_GetMass(b)
    var I = b3Body_GetLocalRotationalInertia(b)
    let off = b3Vec3(x: 0, y: -1.5, z: 0)
    I = b3AddMM(I, b3Steiner(mass, off))
    let up = b3MassData(mass: mass, center: off, inertia: I)

    let runner = DemoRunner(DemoScene.named("Weeble")!)
    runner.advance(frames: 1)
    let weeble = runner.root.children[0].children.first { $0.components[B3PhysicsBodyComponent.self]?.mode == .dynamic }!
    let port = b3Body_GetMassData(try #require(runner.world.record(for: weeble)).bodyId)
    #expect(abs(port.mass - up.mass) < 0.01 * up.mass)
    #expect(port.center.y == up.center.y)
    for (a, b) in [(port.inertia.cx.x, up.inertia.cx.x), (port.inertia.cy.y, up.inertia.cy.y), (port.inertia.cz.z, up.inertia.cz.z)] {
        #expect(abs(a - b) < 0.01 * b, "inertia \(a) vs upstream \(b)")
    }
}

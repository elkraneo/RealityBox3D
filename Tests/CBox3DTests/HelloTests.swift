import Testing
import CBox3D

@Test func boxFallsOntoGround() {
    var worldDef = b3DefaultWorldDef()
    worldDef.workerCount = 1
    let world = b3CreateWorld(&worldDef)
    defer { b3DestroyWorld(world) }

    var groundDef = b3DefaultBodyDef()
    groundDef.position = b3Pos(x: 0, y: -10, z: 0)
    let ground = b3CreateBody(world, &groundDef)
    var groundBox = b3MakeBoxHull(50, 10, 50)
    var groundShape = b3DefaultShapeDef()
    // `base` indexes into the arrays that follow it, so pass the whole box's address.
    _ = withUnsafePointer(to: &groundBox) { b3CreateHullShape(ground, &groundShape, UnsafeRawPointer($0).assumingMemoryBound(to: b3HullData.self)) }

    var bodyDef = b3DefaultBodyDef()
    bodyDef.type = b3_dynamicBody
    bodyDef.position = b3Pos(x: 0, y: 4, z: 0)
    let body = b3CreateBody(world, &bodyDef)
    var cube = b3MakeCubeHull(0.5)
    var shapeDef = b3DefaultShapeDef()
    shapeDef.density = 1
    _ = withUnsafePointer(to: &cube) { b3CreateHullShape(body, &shapeDef, UnsafeRawPointer($0).assumingMemoryBound(to: b3HullData.self)) }

    for _ in 0..<120 { b3World_Step(world, 1.0 / 60.0, 4) }
    let p = b3Body_GetPosition(body)
    #expect(abs(p.y - 0.5) < 0.05, "resting height \(p.y)")
    #expect(abs(p.x) < 1e-2 && abs(p.z) < 1e-2)
}

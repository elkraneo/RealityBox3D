import CBox3D

/// Upstream's Arch sample through the raw C API, as a reference for the RealityBox3D port.
@MainActor func archRawKeystoneY() -> Float {
    var wd = b3DefaultWorldDef(); wd.workerCount = 1
    let world = b3CreateWorld(&wd); defer { b3DestroyWorld(world) }
    var gd = b3DefaultBodyDef(); gd.position = b3Vec3(x: 0, y: -1, z: 0)
    let g = b3CreateBody(world, &gd); var gb = b3MakeBoxHull(40, 1, 40); var gs = b3DefaultShapeDef()
    _ = withUnsafePointer(to: &gb) { b3CreateHullShape(g, &gs, UnsafeRawPointer($0).assumingMemoryBound(to: b3HullData.self)) }
    let s: Float = 0.25
    let ps1: [(Float, Float)] = [(16, 0), (14.93803712795643, 5.133601056842984), (13.79871746027416, 10.24928069555078), (12.56252963284711, 15.34107019122473), (11.20040987372525, 20.39856541571217), (9.66521217819836, 25.40369899225096), (7.87179930638133, 30.3179337000085), (5.635199558196225, 35.03820717801641), (2.405937953536585, 39.09554102558315)].map { ($0.0 * s, $0.1 * s) }
    let ps2: [(Float, Float)] = [(24, 0), (22.33619528222415, 6.02299846205841), (20.54936888969905, 12.00964361211476), (18.60854610798073, 17.9470321677465), (16.46769273811807, 23.81367936585418), (14.05325025774858, 29.57079353071012), (11.23551045834022, 35.13775818285372), (7.752568160730571, 40.30450679009583), (3.016931552701656, 44.28891593799322)].map { ($0.0 * s, $0.1 * s) }
    var sd = b3DefaultShapeDef(); sd.density = 200
    var bd = b3DefaultBodyDef(); bd.type = b3_dynamicBody
    func stone(_ q: [(Float, Float)]) -> b3BodyId {
        let body = b3CreateBody(world, &bd)
        var pts = q.map { b3Vec3(x: $0.0, y: $0.1, z: -0.5) } + q.map { b3Vec3(x: $0.0, y: $0.1, z: 0.5) }
        let hull = b3CreateHull(&pts, 8, 8)!; _ = b3CreateHullShape(body, &sd, hull); b3DestroyHull(hull)
        return body
    }
    for i in 0..<8 { _ = stone([ps1[i], ps2[i], ps2[i + 1], ps1[i + 1]]) }
    for i in 0..<8 { _ = stone([(-ps2[i].0, ps2[i].1), (-ps1[i].0, ps1[i].1), (-ps1[i + 1].0, ps1[i + 1].1), (-ps2[i + 1].0, ps2[i + 1].1)]) }
    let key = stone([ps1[8], ps2[8], (-ps2[8].0, ps2[8].1), (-ps1[8].0, ps1[8].1)])
    var box = b3MakeBoxHull(2, 0.5, 0.5)
    for i in 0..<4 {
        bd.position = b3Vec3(x: 0, y: 0.5 + ps2[8].1 + Float(i), z: 0)
        let b = b3CreateBody(world, &bd)
        _ = withUnsafePointer(to: &box) { b3CreateHullShape(b, &sd, UnsafeRawPointer($0).assumingMemoryBound(to: b3HullData.self)) }
    }
    for _ in 0..<240 { b3World_Step(world, 1.0 / 60, 4) }
    return b3Body_GetPosition(key).y
}

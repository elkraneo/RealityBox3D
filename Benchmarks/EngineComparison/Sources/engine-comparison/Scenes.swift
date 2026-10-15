import CoreMedia
import RealityBox3D
import RealityKit
import RealitySDFPhysics
import simd

/// A pile dropped into RealitySDFPhysics's benchmark box: a floor 4 m across and walls 5 m high
/// around a 2 m square, so every body stays in.
enum Pile: String, CaseIterable {
    /// RealitySDFPhysics's own benchmark: tori 30 cm across. RealityKit and Box3D take each torus as
    /// a compound of 12 capsules, their closest convex approximation.
    case tori
    /// 10 cm boxes, which every engine simulates natively.
    case boxes

    static let walls: [(center: SIMD3<Float>, halfExtents: SIMD3<Float>)] = [
        ([0, -0.5, 0], [2, 0.5, 2]),
        ([-1.05, 2.5, 0], [0.05, 2.5, 1.1]), ([1.05, 2.5, 0], [0.05, 2.5, 1.1]),
        ([0, 2.5, -1.05], [1.1, 2.5, 0.05]), ([0, 2.5, 1.05], [1.1, 2.5, 0.05]),
    ]

    static let major: Float = 0.15, minor: Float = 0.04, segments = 12
    static let box: Float = 0.1

    /// Layers of a grid above the box, tilted, as RealitySDFPhysics's TorusPile places tori.
    func placements(_ count: Int) -> [(position: SIMD3<Float>, orientation: simd_quatf)] {
        let side = self == .tori ? 5 : 10
        let spacing: Float = self == .tori ? 0.36 : 0.18
        let layerHeight: Float = self == .tori ? 0.25 : 0.16
        return (0..<count).map { i in
            let perLayer = side * side
            let layer = i / perLayer, cell = i % perLayer
            let half = Float(side - 1) * spacing / 2
            let x = Float(cell % side) * spacing - half, z = Float(cell / side) * spacing - half
            let angle = Float(i % 7) * 0.4
            return ([x, 0.3 + Float(layer) * layerHeight, z], simd_quatf(angle: angle, axis: normalize([1, 0, Float(i % 3) - 1])))
        }
    }

    /// The 12 capsules of a torus, as offsets: shared by RealityKit and Box3D.
    static var capsuleRing: [(rotation: simd_quatf, translation: SIMD3<Float>, height: Float)] {
        let chord = 2 * major * sin(.pi / Float(segments))
        return (0..<segments).map { k in
            let angle = (Float(k) + 0.5) * 2 * .pi / Float(segments)
            return (simd_quatf(from: [0, 1, 0], to: [-sin(angle), 0, cos(angle)]),
                    [major * cos(angle), 0, major * sin(angle)], chord + 2 * minor)
        }
    }

    // MARK: Builders

    @MainActor
    func makeSDF(count: Int, settings: SDFPhysicsSimulationComponent) -> (root: Entity, bodies: [Entity]) {
        let root = Entity()
        root.components.set(settings)
        for wall in Self.walls {
            let e = Entity(); e.position = wall.center
            e.components.set(SDFPhysicsBodyComponent(shape: .box(halfExtents: wall.halfExtents), mode: .static))
            root.addChild(e)
        }
        let shape: SDFShape = self == .tori ? .torus(majorRadius: Self.major, minorRadius: Self.minor)
                                            : .box(halfExtents: SIMD3(repeating: Self.box / 2))
        let bodies = placements(count).map { p -> Entity in
            let e = Entity()
            e.transform = Transform(rotation: p.orientation, translation: p.position)
            e.components.set(SDFPhysicsBodyComponent(shape: shape))
            root.addChild(e)
            return e
        }
        return (root, bodies)
    }

    @MainActor
    func makeRealityKit(count: Int, clock: CMTimebase) -> (root: Entity, bodies: [Entity]) {
        let root = Entity()
        var physics = PhysicsSimulationComponent()
        physics.clock = clock
        root.components.set(physics)
        for wall in Self.walls {
            let e = Entity(); e.position = wall.center
            let shape = ShapeResource.generateBox(size: 2 * wall.halfExtents)
            e.components.set(CollisionComponent(shapes: [shape]))
            e.components.set(PhysicsBodyComponent(shapes: [shape], mass: 1, mode: .static))
            root.addChild(e)
        }
        let shapes: [ShapeResource] = self == .tori
            ? Self.capsuleRing.map { ShapeResource.generateCapsule(height: $0.height, radius: Self.minor).offsetBy(rotation: $0.rotation, translation: $0.translation) }
            : [.generateBox(size: SIMD3(repeating: Self.box))]
        let bodies = placements(count).map { p -> Entity in
            let e = Entity()
            e.transform = Transform(rotation: p.orientation, translation: p.position)
            e.components.set(CollisionComponent(shapes: shapes))
            e.components.set(PhysicsBodyComponent(shapes: shapes, density: 1000))
            root.addChild(e)
            return e
        }
        return (root, bodies)
    }

    @MainActor
    func makeBox3D(count: Int) -> (root: Entity, bodies: [Entity]) {
        let root = Entity()
        for wall in Self.walls {
            let e = Entity(); e.position = wall.center
            e.components.set(B3PhysicsBodyComponent(mode: .static))
            e.components.set(B3CollisionComponent(shapes: [.generateBox(size: 2 * wall.halfExtents)]))
            root.addChild(e)
        }
        let shapes: [B3ShapeResource] = self == .tori
            ? Self.capsuleRing.map { B3ShapeResource.generateCapsule(height: $0.height, radius: Self.minor).offsetBy(rotation: $0.rotation, translation: $0.translation) }
            : [.generateBox(size: SIMD3(repeating: Self.box))]
        let bodies = placements(count).map { p -> Entity in
            let e = Entity()
            e.transform = Transform(rotation: p.orientation, translation: p.position)
            e.components.set(B3PhysicsBodyComponent(density: 1000))
            e.components.set(B3CollisionComponent(shapes: shapes))
            root.addChild(e)
            return e
        }
        return (root, bodies)
    }

    /// Mean height and how many left the box: the pile fell and settled the same way.
    @MainActor
    static func check(_ bodies: [Entity]) -> String {
        let ps = bodies.map { $0.position(relativeTo: nil) }
        let mean = ps.map(\.y).reduce(0, +) / Float(max(ps.count, 1))
        let out = ps.filter { abs($0.x) > 1 || abs($0.z) > 1 || $0.y < -0.1 }.count
        return String(format: "mean y %.2f m, %d out", mean, out)
    }
}

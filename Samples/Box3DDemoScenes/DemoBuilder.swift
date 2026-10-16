import CoreGraphics
import RealityBox3D
import RealityKit
import simd

/// Creates the bodies of a scene, sharing meshes and materials so piles of thousands of bodies stay cheap to render.
@MainActor
public struct DemoBuilder {
    public let content = Entity()
    var random = SeededRandom(seed: 0x853c49e6748fea9b)
    private var meshes: [(B3ShapeResource, MeshResource)] = []
    private let palette: [RealityKit.Material]
    private let staticMaterial: RealityKit.Material
    private var colorIndex = 0
    private(set) var script: DemoScript?
    /// The scene's joints, set on the simulation root by `DemoScene.load(into:)`.
    public var joints: [any B3PhysicsJoint] = []

    /// A pin on the scene's static frame, for joints anchored to the world, as upstream's ground body.
    public func anchor(_ name: String, at position: SIMD3<Float>, orientation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)) -> GeometricPin {
        content.pins.set(named: name, position: position, orientation: orientation)
    }

    public init() {
        // Box3D's sample palette is pastel on a dark grid; a golden-ratio hue walk gives the same spread.
        palette = (0..<12).map { i in
            var m = PhysicallyBasedMaterial()
            m.baseColor = .init(tint: .init(hue: (CGFloat(i) * 0.618_034).truncatingRemainder(dividingBy: 1), saturation: 0.45, brightness: 0.9, alpha: 1))
            m.roughness = 0.6
            return m
        }
        var s = PhysicallyBasedMaterial()
        s.baseColor = .init(tint: .init(white: 0.45, alpha: 1))
        s.roughness = 0.9
        staticMaterial = s
    }

    /// Upstream's `AddGroundBox(extent)`: a static box whose top face is at y = 0.
    @discardableResult
    public mutating func ground(extent: Float) -> Entity {
        body([.generateBox(size: [2 * extent, 2, 2 * extent])], at: [0, -1, 0], mode: .static, name: "ground")
    }

    /// Defaults are Box3D's `b3DefaultShapeDef`: density 1000 kg/m³ (water), friction 0.6.
    @discardableResult
    public mutating func body(_ shapes: [B3ShapeResource], at position: SIMD3<Float>,
                              rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
                              mode: PhysicsBodyMode = .dynamic, density: Float = 1000,
                              friction: Float = 0.6, restitution: Float = 0, rollingResistance: Float = 0,
                              linearVelocity: SIMD3<Float> = .zero, angularVelocity: SIMD3<Float> = .zero,
                              collisionMode: B3CollisionComponent.Mode = .default,
                              name: String? = nil,
                              configure: (inout B3PhysicsBodyComponent) -> Void = { _ in }) -> Entity {
        let entity = Entity()
        entity.name = name ?? ""
        entity.position = position
        entity.orientation = rotation

        let material: RealityKit.Material = mode == .static ? staticMaterial : nextColor()
        if collisionMode != .trigger {
            for shape in shapes {
                entity.addChild(ModelEntity(mesh: mesh(for: shape), materials: [material]))
            }
        } else {
            var glass = UnlitMaterial(color: .init(red: 0.3, green: 0.8, blue: 0.4, alpha: 0.25))
            glass.blending = .transparent(opacity: 0.25)
            for shape in shapes { entity.addChild(ModelEntity(mesh: mesh(for: shape), materials: [glass])) }
        }

        var m = B3PhysicsMaterial.generate(staticFriction: friction, dynamicFriction: friction, restitution: restitution)
        m.rollingResistance = rollingResistance
        var body = B3PhysicsBodyComponent(density: density, material: m, mode: mode)
        configure(&body)
        entity.components.set(body)
        entity.components.set(B3CollisionComponent(shapes: shapes, mode: collisionMode))
        if linearVelocity != .zero || angularVelocity != .zero {
            entity.components.set(B3PhysicsMotionComponent(linearVelocity: linearVelocity, angularVelocity: angularVelocity))
        }
        content.addChild(entity)
        return entity
    }

    @discardableResult
    public mutating func body(_ shape: B3ShapeResource, at position: SIMD3<Float>,
                              rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
                              mode: PhysicsBodyMode = .dynamic, density: Float = 1000,
                              friction: Float = 0.6, restitution: Float = 0, rollingResistance: Float = 0,
                              linearVelocity: SIMD3<Float> = .zero, angularVelocity: SIMD3<Float> = .zero,
                              collisionMode: B3CollisionComponent.Mode = .default,
                              name: String? = nil,
                              configure: (inout B3PhysicsBodyComponent) -> Void = { _ in }) -> Entity {
        body([shape], at: position, rotation: rotation, mode: mode, density: density, friction: friction,
             restitution: restitution, rollingResistance: rollingResistance, linearVelocity: linearVelocity,
             angularVelocity: angularVelocity, collisionMode: collisionMode, name: name, configure: configure)
    }

    /// Code run every frame before physics, for scenes that animate or react, as upstream's `Step()` overrides.
    public mutating func onFrame(_ body: @escaping @MainActor (DemoContext) -> Void) {
        scriptObject().onFrame.append(body)
    }

    /// Makes the app's camera follow an entity.
    public mutating func follow(_ entity: Entity) {
        scriptObject().follow = entity
    }

    /// A button of the scene, as upstream's keyboard actions.
    public mutating func action(_ name: String, _ body: @escaping @MainActor (DemoContext) -> Void) {
        scriptObject().actions[name] = body
    }

    private mutating func scriptObject() -> DemoScript {
        if let script { return script }
        let s = DemoScript()
        script = s
        content.components.set(DemoScriptComponent(script: s))
        return s
    }

    private mutating func nextColor() -> RealityKit.Material {
        defer { colorIndex += 1 }
        return palette[colorIndex % palette.count]
    }

    private mutating func mesh(for shape: B3ShapeResource) -> MeshResource {
        if let hit = meshes.first(where: { $0.0 == shape }) { return hit.1 }
        let m = (try? MeshResource.generate(from: shape)) ?? .generateSphere(radius: 0.01)
        meshes.append((shape, m))
        return m
    }
}

/// PCG32, so scenes with random placement always load the same.
public struct SeededRandom: RandomNumberGenerator {
    var state: UInt64
    public init(seed: UInt64) { state = seed }
    public mutating func next() -> UInt64 {
        func step() -> UInt32 {
            let old = state
            state = old &* 6364136223846793005 &+ 1442695040888963407
            let xorshifted = UInt32(truncatingIfNeeded: ((old >> 18) ^ old) >> 27)
            let rot = UInt32(truncatingIfNeeded: old >> 59)
            return (xorshifted >> rot) | (xorshifted << ((~rot &+ 1) & 31))
        }
        return UInt64(step()) << 32 | UInt64(step())
    }
}

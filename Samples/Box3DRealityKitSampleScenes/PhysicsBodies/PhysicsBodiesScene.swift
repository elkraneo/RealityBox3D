// Port of the scene of Apple's sample "Simulating physics with collisions in your visionOS app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app
//
// Apple's 16 metallic spheres float without gravity in an invisible containment box, attract each
// other, and follow drags with a spring-like force. Differences from Apple's sample:
//
// - Bodies are `B3PhysicsBodyComponent` and `B3CollisionComponent` in a `B3PhysicsSimulationComponent`
//   root, in place of RealityKit's components: the same names without the prefix.
// - The root is scaled to centimeters, Box3D's length unit for this app (see RealityKitSamplesApp).
//   Masses, forces and the sizes you see are Apple's.
// - The drag force acts on every fixed step, from `B3PhysicsSimulationEvents.WillSimulate`, toward
//   the pointer's last position. On visionOS a hand sends drag events every frame; on macOS the
//   pointer sends them only while it moves.
// - The volume is a fixed 1 m cube viewed through a camera, since a macOS window is not a volume.
//   Its faces are 50 cm thick rather than 1 mm, outside the volume.
// - `Shape` swaps the spheres for tori, as RealitySDFPhysics's port does. Box3D's bodies are convex,
//   so each torus is a ring of 12 capsules, the compound RealityKit would need too; the ring is
//   what you see. RealitySDFPhysics simulates the true concave torus. They appear one at a time
//   where the world's overlap query finds room, so that none start threaded through another.

import Combine
import RealityBox3D
import RealityKit
import simd
import SwiftUI

@MainActor
package final class PhysicsBodiesScene {
    package enum Shape: String, CaseIterable, Identifiable {
        case spheres = "Spheres"
        case tori = "Tori"
        package var id: Self { self }
    }

    /// The edge of the cubic volume, in meters
    static let volumeSize: Float = 1
    /// The walls' collision group, which the drag's raycast leaves out
    static let walls = CollisionGroup(rawValue: 1 << 1)

    package let content = Entity()
    /// The simulation root, scaled so that its units are centimeters
    let root = Entity()
    package let camera = SampleCamera(eye: [0, 0.45, 1.8])
    private let bodies = Entity()
    /// The body being dragged and the world space point it is pulled toward
    private var drag: (entity: Entity, target: SIMD3<Float>)?
    private var dragSubscription: B3Subscription?

    package init() {
        root.scale = SIMD3(repeating: 0.01)
        var settings = B3PhysicsSimulationComponent()
        settings.gravity = .zero
        root.components.set(settings)
        root.addChild(makeContainment())
        root.addChild(bodies)
        content.addChild(root)
        content.addChild(makeOutline())

        let light = DirectionalLight()
        light.light.intensity = 3000
        light.look(at: .zero, from: [1, 2, 1.5], relativeTo: nil)
        content.addChild(light)
        let fill = DirectionalLight()
        fill.light.intensity = 1200
        fill.look(at: .zero, from: [-1.5, 0.5, -1], relativeTo: nil)
        content.addChild(fill)

        content.addChild(camera.makeEntity())
    }

    /// Replaces the bodies with 16 new ones of a shape
    package func reset(shape: Shape) {
        spawning = nil
        waiting?.cancel()
        bodies.children.removeAll()
        guard shape == .tori else {
            // Apple's spheres appear at once in semi-random places, overlapping ones pushing apart.
            for index in 0..<16 {
                let body = Self.makeBody(shape)
                body.name = "\(shape.rawValue)_\(index)"
                bodies.addChild(body)
                body.setPosition(SIMD3<Float>.random(in: -0.2...0.2), relativeTo: nil)
            }
            return
        }
        // Rings placed overlapping can start threaded through each other, links no motion undoes.
        // As in RealitySDFPhysics's port, they appear one at a time, every 12 steps, each where the
        // world's overlap query finds room. The world exists after the system's first frame.
        if let world = RealityBox3D.world(for: root) {
            spawnTori(in: world)
        } else {
            waiting = Task { @MainActor [weak self] in
                while let self, RealityBox3D.world(for: self.root) == nil {
                    try? await Task.sleep(for: .milliseconds(16))
                    if Task.isCancelled { return }
                }
                guard let self, let world = RealityBox3D.world(for: self.root) else { return }
                self.spawnTori(in: world)
            }
        }
    }

    private var spawning: B3Subscription?
    private var waiting: Task<Void, Never>?

    private func spawnTori(in world: B3World) {
        var spawned = 0, steps = 0
        spawning = world.subscribe(to: B3PhysicsSimulationEvents.DidSimulate.self) { [unowned self] _ in
            steps += 1
            guard steps % 12 == 0 else { return }
            let body = Self.makeBody(.tori)
            body.name = "Tori_\(spawned)"
            guard let shapes = body.components[B3CollisionComponent.self]?.shapes else { return }
            for _ in 0..<50 {
                let pose = Transform(rotation: body.orientation, translation: root.convert(position: SIMD3<Float>.random(in: -0.3...0.3), from: nil))
                // Two centimeters of margin, in the root's units
                if shapes.allSatisfy({ world.overlaps(of: $0, at: pose, margin: 2).isEmpty }) {
                    body.transform = pose
                    bodies.addChild(body)
                    break
                }
            }
            spawned += 1
            if spawned == 16 { spawning = nil }
        }
    }

    // MARK: Bodies

    /// Port of Apple's `Entity.metallicSphere(_:)`, for either shape
    private static func makeBody(_ kind: Shape) -> Entity {
        // RealitySDFPhysics's sizes: spheres 6 to 30 cm across, tori 16.5 to 30 cm
        let radius: Float = kind == .spheres ? 0.15 * .random(in: 0.2...1.0) : 0.15 * .random(in: 0.55...1.0)
        let r = radius * 100 // in the root's centimeters
        let shapes: [B3ShapeResource] = kind == .spheres ? [.generateSphere(radius: r)] : Self.torusRing(major: 0.7 * r, minor: 0.3 * r)
        let entity = ModelEntity()
        let material = metallicMaterial()
        for shape in shapes {
            let mesh = (try? MeshResource.generate(from: shape)) ?? .generateSphere(radius: r)
            entity.addChild(ModelEntity(mesh: mesh, materials: [material]))
        }
        entity.orientation = simd_quatf(angle: .random(in: 0...(2 * .pi)), axis: normalize(.random(in: -1...1)))

        // Apple's density of 10,000 kg/m³ is 0.01 kg per cubic centimeter
        var body = B3PhysicsBodyComponent(shapes: shapes, density: 0.01)
        // Make each body float in the air by turning off gravity
        body.isAffectedByGravity = false
        entity.components.set(body)
        entity.components.set(B3CollisionComponent(shapes: shapes))
        // Add an attraction force to the body
        entity.components.set(AttractionComponent())
        return entity
    }

    /// A torus as 12 capsules around the y-axis, as in Benchmarks/EngineComparison
    static func torusRing(major: Float, minor: Float, segments: Int = 12) -> [B3ShapeResource] {
        let chord = 2 * major * sin(.pi / Float(segments))
        return (0..<segments).map { k in
            let angle = (Float(k) + 0.5) * 2 * .pi / Float(segments)
            return B3ShapeResource.generateCapsule(height: chord + 2 * minor, radius: minor)
                .offsetBy(rotation: simd_quatf(from: [0, 1, 0], to: [-sin(angle), 0, cos(angle)]),
                          translation: [major * cos(angle), 0, major * sin(angle)])
        }
    }

    /// Apple's material: a random hue, metallic, with a clear coat
    private static func metallicMaterial() -> PhysicallyBasedMaterial {
        var material = PhysicallyBasedMaterial()
        let color = RealityKit.Material.Color(hue: .random(in: 0...1), saturation: .random(in: 0.5...1), brightness: 0.9, alpha: 1)
        material.baseColor = .init(tint: color)
        material.metallic = 1.0
        material.roughness = 0.5
        material.clearcoat = 1.0
        material.clearcoatRoughness = 0.1
        return material
    }

    // MARK: Containment

    /// Port of Apple's `ContainmentCollisionBox`: six static faces around the volume
    private func makeContainment() -> Entity {
        let box = Entity()
        let half = 100 * Self.volumeSize / 2, thickness: Float = 50
        for axis in 0..<3 {
            for side: Float in [-1, 1] {
                var size = SIMD3<Float>(repeating: 2 * (half + thickness))
                size[axis] = thickness
                var position = SIMD3<Float>.zero
                position[axis] = side * (half + thickness / 2)
                let face = Entity()
                face.position = position
                face.components.set(B3PhysicsBodyComponent(mode: .static))
                face.components.set(B3CollisionComponent(shapes: [.generateBox(size: size)],
                                                         filter: CollisionFilter(group: Self.walls, mask: .all)))
                box.addChild(face)
            }
        }
        return box
    }

    /// Faint edges of the volume, which Apple's sample shows by its window
    private func makeOutline() -> Entity {
        let outline = Entity()
        let half = Self.volumeSize / 2, width: Float = 0.002
        let material = UnlitMaterial(color: .init(white: 0.6, alpha: 1))
        for axis in 0..<3 {
            for a: Float in [-1, 1] {
                for b: Float in [-1, 1] {
                    var size = SIMD3<Float>(repeating: width)
                    size[axis] = Self.volumeSize
                    var position = SIMD3<Float>(repeating: 0)
                    position[(axis + 1) % 3] = a * half
                    position[(axis + 2) % 3] = b * half
                    let edge = ModelEntity(mesh: .generateBox(size: size), materials: [material])
                    edge.position = position
                    outline.addChild(edge)
                }
            }
        }
        return outline
    }

    // MARK: Dragging

    /// Starts dragging the body under a point of the view. Returns its distance from the camera.
    package func beginDrag(at point: CGPoint, in size: CGSize) -> Float? {
        guard let world = RealityBox3D.world(for: root) else { return nil }
        let ray = camera.ray(through: point, in: size)
        // The world's raycast works in the root's space
        let origin = root.convert(position: ray.origin, from: nil)
        let direction = root.convert(direction: ray.direction, from: nil)
        // The camera is outside the volume, so leave out the walls
        guard let hit = world.raycast(origin: origin, direction: direction, length: 1000, query: .nearest,
                                      mask: .all.subtracting(Self.walls)).first else { return nil }
        drag = (hit.entity, hit.entity.position(relativeTo: nil))
        if dragSubscription == nil {
            dragSubscription = world.subscribe(to: B3PhysicsSimulationEvents.WillSimulate.self) { [unowned self] _ in
                if let drag { pull(drag.entity, towards: drag.target) }
            }
        }
        return simd_distance(root.convert(position: hit.position, to: nil), ray.origin)
    }

    /// Moves the drag target to a world space point
    package func drag(to target: SIMD3<Float>) { drag?.target = target }

    package func endDrag() { drag = nil }

    /// Port of Apple's `ForceDragGesture`: a spring-like force towards the drag target
    private func pull(_ entity: Entity, towards target: SIMD3<Float>) {
        guard let pose = entity.b3.pose else { return }
        let direction = target - root.convert(position: pose.translation, to: nil)
        var strength = length(direction)
        guard strength > 1e-6 else { return }
        if strength < 1 { strength *= strength }
        let forceFactor: Float = 3000
        entity.b3.addForce(forceFactor * strength * normalize(direction), relativeTo: nil)
    }
}

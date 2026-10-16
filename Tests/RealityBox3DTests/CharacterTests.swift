import Testing
import RealityKit
import simd
@testable import RealityBox3D

@MainActor
@Suite struct CharacterTests {
    func scene() -> (root: Entity, world: B3World, statics: [Entity]) {
        let root = Entity()
        root.components.set(B3PhysicsSimulationComponent())
        let ground = Entity()
        ground.position = [0, -0.5, 0]
        ground.components.set(B3PhysicsBodyComponent(mode: .static))
        ground.components.set(B3CollisionComponent(shapes: [.generateBox(size: [40, 1, 40])]))
        root.addChild(ground)
        return (root, B3World(root: root), [ground])
    }

    func staticBox(_ root: Entity, at p: SIMD3<Float>, size: SIMD3<Float>, rotation: simd_quatf = .init(ix: 0, iy: 0, iz: 0, r: 1)) -> Entity {
        let e = Entity()
        e.position = p
        e.orientation = rotation
        e.components.set(B3PhysicsBodyComponent(mode: .static))
        e.components.set(B3CollisionComponent(shapes: [.generateBox(size: size)]))
        root.addChild(e)
        return e
    }

    func character(_ root: Entity, at p: SIMD3<Float>) -> Entity {
        let c = Entity()
        c.position = p
        c.components.set(B3CharacterControllerComponent(radius: 0.3, height: 1.6))
        root.addChild(c)
        return c
    }

    /// Walks at a velocity with gravity for some frames, as a game loop would.
    func walk(_ c: Entity, _ world: B3World, bodies: [Entity], velocity: SIMD3<Float>, frames: Int, hits: inout [B3CharacterCollision]) {
        var vy: Float = 0
        let dt: Float = 1.0 / 60
        for _ in 0..<frames {
            let grounded = c.components[B3CharacterControllerStateComponent.self]?.isOnGround ?? false
            vy = grounded ? 0 : vy - 9.81 * dt
            var collected: [B3CharacterCollision] = []
            c.b3.moveCharacter(by: (velocity + [0, vy, 0]) * dt, deltaTime: dt, relativeTo: nil) { collected.append($0) }
            hits += collected
            world.update(entities: bodies, deltaTime: Double(dt))
        }
    }

    @Test func walksOnFlatGround() {
        let (root, world, statics) = scene()
        let c = character(root, at: [0, 0.8, 0])
        world.update(entities: statics, deltaTime: 0)
        var hits: [B3CharacterCollision] = []
        walk(c, world, bodies: statics, velocity: [1, 0, 0], frames: 120, hits: &hits)
        #expect(abs(c.position.x - 2) < 0.05, "x \(c.position.x)")
        #expect(abs(c.position.y - 0.8) < 0.03, "y \(c.position.y)")
        #expect(c.components[B3CharacterControllerStateComponent.self]?.isOnGround == true)
    }

    @Test func wallStopsAndSlides() {
        let (root, world, statics) = scene()
        let wall = staticBox(root, at: [1.5, 1, 0], size: [1, 2, 10])
        let c = character(root, at: [0, 0.8, 0])
        world.update(entities: statics + [wall], deltaTime: 0)
        var hits: [B3CharacterCollision] = []
        // Diagonally into the wall: blocked in x, keeps sliding in z.
        walk(c, world, bodies: statics + [wall], velocity: [2, 0, 1], frames: 120, hits: &hits)
        #expect(abs(c.position.x - 0.7) < 0.03, "x \(c.position.x)")
        #expect(c.position.z > 1.8, "did not slide: z \(c.position.z)")
        #expect(hits.contains { $0.hitEntity == wall })
    }

    @Test func fallsAndLands() {
        let (root, world, statics) = scene()
        let c = character(root, at: [0, 3, 0])
        world.update(entities: statics, deltaTime: 0)
        var hits: [B3CharacterCollision] = []
        walk(c, world, bodies: statics, velocity: .zero, frames: 90, hits: &hits)
        #expect(abs(c.position.y - 0.8) < 0.03, "y \(c.position.y)")
        #expect(c.components[B3CharacterControllerStateComponent.self]?.isOnGround == true)
    }

    @Test func walksUpARamp() {
        let (root, world, statics) = scene()
        let angle: Float = 20 * .pi / 180
        let ramp = staticBox(root, at: [4, 0, 0], size: [6, 0.2, 4], rotation: simd_quatf(angle: angle, axis: [0, 0, 1]))
        let c = character(root, at: [0, 0.8, 0])
        world.update(entities: statics + [ramp], deltaTime: 0)
        var hits: [B3CharacterCollision] = []
        walk(c, world, bodies: statics + [ramp], velocity: [2, 0, 0], frames: 150, hits: &hits)
        #expect(c.position.x > 4.5, "stuck at x \(c.position.x)")
        // The ramp's top surface under the character: 0.1 m above its center line, rising tan 20°.
        let surface = 0.1 / cos(angle) + tan(angle) * (c.position.x - 4)
        #expect(c.position.y > surface + 0.75 && c.position.y < surface + 0.95, "y \(c.position.y) over a surface at \(surface)")
    }

    @Test func pushesABox() {
        let (root, world, statics) = scene()
        let crate = Entity()
        crate.position = [1.5, 0.25, 0]
        crate.components.set(B3PhysicsBodyComponent(density: 100))
        crate.components.set(B3CollisionComponent(shapes: [.generateBox(size: [0.5, 0.5, 0.5])]))
        root.addChild(crate)
        let c = character(root, at: [0, 0.8, 0])
        world.update(entities: statics + [crate], deltaTime: 0)
        var hits: [B3CharacterCollision] = []
        walk(c, world, bodies: statics + [crate], velocity: [1.5, 0, 0], frames: 120, hits: &hits)
        #expect(crate.position.x > 2.2, "crate at \(crate.position.x)")
    }

    /// A correction moves the character but is not motion: it pushes nothing.
    @Test func correctionDoesNotPush() {
        func push(correction: Bool) -> Float {
            let (root, world, statics) = scene()
            let c = character(root, at: [0, 0.8, 0])
            let crate = Entity()
            crate.position = [0.75, 0.8, 0]
            crate.components.set(B3PhysicsBodyComponent(mode: .dynamic))
            crate.components.set(B3CollisionComponent(shapes: [.generateBox(size: [0.6, 0.6, 0.6])]))
            root.addChild(crate)
            world.update(entities: statics + [crate], deltaTime: 0)
            let dt: Float = 1.0 / 60
            for _ in 0..<20 {
                let step: SIMD3<Float> = [0.06, 0, 0]
                if correction { c.b3.moveCharacter(by: .zero, correction: step, deltaTime: dt, relativeTo: nil) }
                else { c.b3.moveCharacter(by: step, deltaTime: dt, relativeTo: nil) }
                world.update(entities: statics + [crate], deltaTime: Double(dt))
            }
            return crate.position.x - 0.75
        }
        #expect(push(correction: false) > 0.1)
        #expect(push(correction: true) < 0.01, "pushed \(push(correction: true))")
    }
}

import CoreGraphics
import Foundation
import RealityBox3D
import RealityKit
import simd

extension DemoScene {
    static let characters: [DemoScene] = [
        DemoScene("Character", "Geometric Mover", upstream: "samples/sample_character.cpp GeometricMover, samples/mover.cpp",
                  summary: "Upstream's capsule mover on its test level: stairs, a ramp, a torus, a wavy height field, a door on a sprung hinge, a falling tree and a falling slab. The violet capsule pushes back hard, the green one is soft and lets you lean into it, the white box is ignored.",
                  camera: .init(yaw: 120, pitch: 30, distance: 5, target: [7.5, 0.75, 9]),
                  actions: ["Reset Character"],
                  controls: "WASD to walk, relative to the camera; space to jump; shift to sprint.") { b in
            GeometricMover.build(&b)
        },
    ]
}

/// Upstream's GeometricMover sample. The level is upstream's, the walking is upstream's
/// `CharacterMover` game code (friction, acceleration, gravity, a pogo spring that floats the
/// capsule over steps, jumping) on RealityBox3D's `moveCharacter`, which runs Box3D's mover solver.
@MainActor
enum GeometricMover {
    // Groups: the friendly capsule, which the pogo ray skips, and the box the character ignores
    static let friendly = CollisionGroup(rawValue: 1 << 1)
    static let ignored = CollisionGroup(rawValue: 1 << 2)
    static let start: SIMD3<Float> = [7.5, 0.75, 9]

    static func build(_ b: inout DemoBuilder) {
        // The level mesh and three boxes on one static body
        let level = loadOBJ("test_map01")
        b.body([.generateStaticMesh(positions: level.positions, faceIndices: level.indices),
                .generateBox(size: [2, 2, 2]).offsetBy(translation: [4, 1, 14]),
                .generateBox(size: [2, 2, 2]).offsetBy(translation: [4, 1, 13.95]),
                .generateBox(size: [2, 2, 2]).offsetBy(rotation: simd_quatf(angle: 0.1 * .pi, axis: [0, 1, 0]), translation: [5.8, 1, 13.7])],
               at: .zero, mode: .static, name: "level")

        let stairs = loadOBJ("stairs").scaled(by: [0.75, 0.75, -1.5])
        b.body(.generateStaticMesh(positions: stairs.positions, faceIndices: stairs.indices), at: [-10, 0, 0], mode: .static, name: "stairs")

        let torus = torusMesh(radialResolution: 10, tubularResolution: 12, radius: 2, thickness: 1).scaled(by: [-0.75, 1.5, 0.5])
        b.body(.generateStaticMesh(positions: torus.positions, faceIndices: torus.indices), at: [-10, 1, -8],
               rotation: simd_quatf(angle: 0.5 * .pi, axis: [0, 1, 0]), mode: .static, name: "torus")

        // b3CreateWave(50, 50, one, 0.02, 0.04, holes): upstream punches a hole in every 16th cell,
        // which RealityBox3D's height fields cannot, so this one is solid.
        var heights: [Float] = []
        for i in 0..<50 {
            for j in 0..<50 { heights.append(sin(2 * .pi * 0.02 * Float(i)) * sin(2 * .pi * 0.04 * Float(j))) }
        }
        b.body(.generateHeightField(heights: heights, countX: 50, countZ: 50), at: [20, 0, 0], mode: .static, name: "waves")

        // The violet capsule pushes the character out hard and stops its velocity; the green one is
        // soft, never stops the sweep and the pogo ray ignores it.
        let enemy = b.body(capsule([0, -0.5, 0], [0, 0.5, 0], radius: 0.3), at: [0, 1.4, 6], mode: .static, name: "enemy")
        enemy.components.set(B3CharacterObstacleComponent(pushLimit: 1, clipsVelocity: true, isPushable: false))
        let friend = b.body(capsule([0, -0.5, 0], [0, 0.5, 0], radius: 0.3), at: [0, 1.4, 5], mode: .static, name: "friend")
        friend.components.set(B3CharacterObstacleComponent(pushLimit: 0.01, clipsVelocity: false, isPushable: false, stopsSweep: false))
        setFilter(friend, CollisionFilter(group: friendly, mask: .all))
        tint(enemy, 0.78, 0.08, 0.52) // b3_colorMediumVioletRed
        tint(friend, 0.2, 0.8, 0.2) // b3_colorLimeGreen

        b.body(.generateSphere(radius: 0.5), at: [7, 5, 0], name: "ball")

        // A slab that falls on a loop through the character's start
        let slabSpawn: SIMD3<Float> = [10.5, 30, 0]
        let slab = b.body(.generateBox(size: [2, 0.2, 2]), at: slabSpawn, name: "slab")

        // Upstream ignores this box with a mover filter callback; here a collision group does it
        let ghost = b.body(.generateBox(size: [1, 0.5, 1]), at: [7, 2, -3], mode: .static, name: "ignored")
        setFilter(ghost, CollisionFilter(group: ignored, mask: .all))
        tint(ghost, 1, 0.98, 0.94) // b3_colorFloralWhite

        // The door: a heavy panel on a sprung hinge about y, limited to ±90°
        let door = b.body(.generateBox(size: [1.5, 3, 0.2]), at: [-2, 1.6, 0], name: "door") { $0.gravityScale = 2 }
        let hinge = pinOrientation(box3DFrame: simd_quatf(from: [0, 0, 1], to: [0, 1, 0]))
        b.joints.append(B3PhysicsRevoluteJoint(pin0: b.anchor("hinge", at: [-2.75, 1.6, 0], orientation: hinge),
                                               pin1: door.pins.set(named: "hinge", position: [-0.75, 0, 0], orientation: hinge),
                                               angularLimit: -(.pi / 2)...(.pi / 2),
                                               spring: B3JointSpring(hertz: 1, dampingRatio: 0.5)))

        // One tree of the falling trees benchmark, toppled along the diagonal, on a loop. The
        // character cannot push it; it shoves the character up to 10 m per move.
        let treeSpawn: SIMD3<Float> = [22, 1.5, 2]
        let tree = Tree.make()
        let trunk = b.body(tree.shapes, at: treeSpawn, friction: 0.9, rollingResistance: 0.05, name: "tree")
        trunk.components.set(B3CharacterObstacleComponent(pushLimit: 10, clipsVelocity: true, isPushable: false))
        let treeSpin = 0.5 * simd_normalize(SIMD3<Float>(1, 0, -1))
        func topple() {
            trunk.position = treeSpawn
            trunk.orientation = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
            trunk.components.set(B3PhysicsMotionComponent(linearVelocity: simd_cross(treeSpin, [0, tree.centerHeight, 0]),
                                                          angularVelocity: treeSpin))
        }
        topple()

        // The character: a controller and a capsule to see, not a body
        let character = Entity()
        character.name = "character"
        character.position = start
        var controller = B3CharacterControllerComponent(radius: 0.3, height: 1.6)
        controller.collisionFilter = CollisionFilter(group: .default, mask: CollisionGroup.all.subtracting(ignored))
        character.components.set(controller)
        var blue = PhysicallyBasedMaterial()
        blue.baseColor = .init(tint: .init(red: 0.2, green: 0.4, blue: 0.95, alpha: 1))
        if let mesh = try? MeshResource.generate(from: .generateCapsule(height: 1.6, radius: 0.3)) {
            character.addChild(ModelEntity(mesh: mesh, materials: [blue]))
        }
        b.content.addChild(character)
        b.follow(character)

        let mover = Mover()
        let slabTimer = SettleTimer(), treeTimer = SettleTimer()
        // Upstream moves once per step, before it: so does this, interleaved with the physics at
        // any frame rate
        b.onFixedStep { ctx in
            guard let world = ctx.world else { return }
            let dt = Float(ctx.deltaTime)
            mover.step(character, world: world, root: ctx.root, input: ctx.input, forward: ctx.cameraForward, dt: dt)
            // Not upstream's: walking off the level's edge starts over
            if character.position.y < -20 { mover.reset(character) }
            if slabTimer.settled(slab, dt: dt) {
                slab.position = slabSpawn
                slab.orientation = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
                slab.components.set(B3PhysicsMotionComponent())
            }
            if treeTimer.settled(trunk, dt: dt) { topple() }
        }
        b.action("Reset Character") { _ in mover.reset(character) }
    }

    /// Upstream's custom colors: static bodies are grey otherwise.
    static func tint(_ e: Entity, _ red: CGFloat, _ green: CGFloat, _ blue: CGFloat) {
        var m = PhysicallyBasedMaterial()
        m.baseColor = .init(tint: .init(red: red, green: green, blue: blue, alpha: 1))
        m.roughness = 0.6
        for case let model as ModelEntity in e.children { model.model?.materials = [m] }
    }

    static func setFilter(_ e: Entity, _ filter: CollisionFilter) {
        guard var c = e.components[B3CollisionComponent.self] else { return }
        c.filter = filter
        e.components.set(c)
    }

    /// Upstream's `SettledForOneSecond`: true once the body has slept for a second. Upstream
    /// lowers the tree's sleep threshold, which RealityBox3D does not expose, so a body also
    /// counts as settled after 12 seconds.
    @MainActor final class SettleTimer {
        var asleep: Float = 0, total: Float = 0
        func settled(_ e: Entity, dt: Float) -> Bool {
            total += dt
            asleep = e.b3.isAwake ? 0 : asleep + dt
            guard asleep >= 1 || total >= 12 else { return false }
            asleep = 0; total = 0
            return true
        }
    }

    /// Upstream's `CharacterMover::SolveMove`, with its constants.
    @MainActor final class Mover {
        let jumpSpeed: Float = 5, maxSpeed: Float = 6, minSpeed: Float = 0.01, stopSpeed: Float = 1
        let accelerate: Float = 30, friction: Float = 4, gravity: Float = 15
        let radius: Float = 0.3, halfSegment: Float = 0.5
        var velocity: SIMD3<Float> = .zero
        var pogoVelocity: Float = 0
        var onGround = false

        func reset(_ character: Entity) {
            character.position = GeometricMover.start
            velocity = .zero
            pogoVelocity = 0
        }

        func step(_ character: Entity, world: B3World, root: Entity, input: DemoInput, forward: SIMD3<Float>, dt: Float) {
            // Friction: linear damping above the stop speed, a fixed reduction below it
            let speed = simd_length(velocity)
            if speed < minSpeed {
                velocity.x = 0; velocity.z = 0
            } else {
                let drop = max(speed, stopSpeed) * friction * dt
                let ratio = max(speed - drop, 0) / speed
                velocity.x *= ratio; velocity.z *= ratio
            }

            let sprint = onGround && input.sprint
            let top = sprint ? 1.5 * maxSpeed : maxSpeed
            let right = simd_normalize(simd_cross(forward, [0, 1, 0]))
            var desired = top * input.throttle.x * forward + top * input.throttle.y * right
            var desiredSpeed = simd_length(desired)
            if desiredSpeed > top { desired *= top / desiredSpeed; desiredSpeed = top }
            let direction = desiredSpeed > 0 ? desired / desiredSpeed : .zero

            if input.jump && onGround {
                velocity.y = jumpSpeed
                onGround = false
            } else if onGround {
                velocity.y = 0
            }

            let add = desiredSpeed - simd_dot(velocity, direction)
            if add > 0 { velocity += min(accelerate * top * dt, add) * direction }
            velocity.y -= gravity * dt

            // The pogo: a ray down from the lower cap's center, sprung to hold it three radii up,
            // which floats the capsule over steps. Off while moving up, so jumps leave the ground.
            let rest = 3 * radius, length = rest + radius
            let origin = character.position(relativeTo: root) - [0, halfSegment, 0]
            let hit = velocity.y > 0 ? nil
                : world.raycast(origin: origin, direction: [0, -1, 0], length: length, query: .nearest,
                                mask: CollisionGroup.all.subtracting(GeometricMover.friendly)).first
            if let hit {
                onGround = true
                let omega = 2 * Float.pi * 4, zeta: Float = 0.7, h = omega * dt
                pogoVelocity = (pogoVelocity - omega * h * (hit.distance - rest)) / (1 + 2 * zeta * h + h * h)
            } else {
                onGround = false
                pogoVelocity = 0
            }

            // The pogo is a correction, not motion: upstream pushes bodies and clips with its
            // velocity alone, so a spring rising under a slab does not launch it
            character.b3.moveCharacter(by: dt * velocity, correction: [0, dt * pogoVelocity, 0], deltaTime: dt, relativeTo: root)
            // Upstream clips its velocity against the planes it touched, so depenetration adds none
            if let state = character.components[B3CharacterControllerStateComponent.self] { velocity = state.clippedVelocity }
        }
    }

    /// The falling trees benchmark's tree: 22 shrinking six-sided cylinders stacked on one body.
    enum Tree {
        static func make() -> (shapes: [B3ShapeResource], centerHeight: Float) {
            var shapes: [B3ShapeResource] = []
            var y: Float = 1, r: Float = 0.75, mass: Float = 0, moment: Float = 0
            let l: Float = 1.5
            for _ in 0..<22 {
                // b3CreateCylinder(l + 2r, r, y - r, 6) spans y - r to y + l + r
                let height = l + 2 * r
                let center = y - r + height / 2
                shapes.append(.generateCylinder(height: height, radius: r, sides: 6).offsetBy(translation: [0, center, 0]))
                let m = r * r * height
                mass += m; moment += m * center
                y += l + 2 * r
                r *= 0.95
            }
            return (shapes, moment / mass)
        }
    }

    struct Mesh {
        var positions: [SIMD3<Float>]
        var indices: [UInt32]

        /// Scales the vertices, flipping the winding when the scale mirrors, as Box3D's mesh scale.
        func scaled(by s: SIMD3<Float>) -> Mesh {
            var m = Mesh(positions: positions.map { $0 * s }, indices: indices)
            if s.x * s.y * s.z < 0 {
                for t in stride(from: 0, to: m.indices.count, by: 3) { m.indices.swapAt(t + 1, t + 2) }
            }
            return m
        }
    }

    static func meshURL(_ name: String) -> URL? { Bundle.module.url(forResource: name, withExtension: "obj", subdirectory: "Meshes") }

    /// Upstream's mesh loader through tinyobjloader, which triangulates polygons by ear clipping:
    /// the stairs' sides are concave.
    static func loadOBJ(_ name: String) -> Mesh {
        guard let url = meshURL(name),
              let text = try? String(contentsOf: url, encoding: .utf8) else { return Mesh(positions: [], indices: []) }
        var mesh = Mesh(positions: [], indices: [])
        for line in text.split(whereSeparator: \.isNewline) {
            let parts = line.split(separator: " ")
            if parts.first == "v", parts.count >= 4, let x = Float(parts[1]), let y = Float(parts[2]), let z = Float(parts[3]) {
                mesh.positions.append([x, y, z])
            } else if parts.first == "f" {
                let face = parts.dropFirst().compactMap { $0.split(separator: "/").first.flatMap { UInt32($0) }.map { $0 - 1 } }
                mesh.indices += triangulate(face, mesh.positions)
            }
        }
        return mesh
    }

    /// Ear clipping of a planar polygon, keeping its winding.
    static func triangulate(_ face: [UInt32], _ positions: [SIMD3<Float>]) -> [UInt32] {
        guard face.count > 3 else { return face.count == 3 ? face : [] }
        let p = face.map { positions[Int($0)] }
        // Newell's normal, then the plane of its largest component
        var n = SIMD3<Float>.zero
        for i in p.indices { let a = p[i], b = p[(i + 1) % p.count]; n += SIMD3((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y)) }
        let drop = abs(n.x) > abs(n.y) ? (abs(n.x) > abs(n.z) ? 0 : 2) : (abs(n.y) > abs(n.z) ? 1 : 2)
        let sign: Float = n[drop] < 0 ? -1 : 1
        let flat = p.map { v -> SIMD2<Float> in
            switch drop { case 0: [v.y, v.z]; case 1: [v.z, v.x]; default: [v.x, v.y] }
        }
        func cross(_ o: SIMD2<Float>, _ a: SIMD2<Float>, _ b: SIMD2<Float>) -> Float { ((a - o).x * (b - o).y - (a - o).y * (b - o).x) * sign }
        var ring = Array(face.indices), out: [UInt32] = []
        var guardCount = 0
        while ring.count > 3, guardCount < 4 * face.count {
            guardCount += 1
            var clipped = false
            for k in ring.indices {
                let i = ring[(k + ring.count - 1) % ring.count], j = ring[k], l = ring[(k + 1) % ring.count]
                guard cross(flat[i], flat[j], flat[l]) > 1e-9 else { continue }
                // No other vertex inside the ear
                let inside = ring.contains { m in
                    m != i && m != j && m != l && cross(flat[i], flat[j], flat[m]) >= 0
                        && cross(flat[j], flat[l], flat[m]) >= 0 && cross(flat[l], flat[i], flat[m]) >= 0
                }
                guard !inside else { continue }
                out += [face[i], face[j], face[l]]
                ring.remove(at: k)
                clipped = true
                break
            }
            if !clipped { break }
        }
        if ring.count == 3 { out += ring.map { face[$0] } }
        return out
    }

    /// `b3CreateTorusMesh`: a ring in the xy plane.
    static func torusMesh(radialResolution: Int, tubularResolution: Int, radius: Float, thickness: Float) -> Mesh {
        var mesh = Mesh(positions: [], indices: [])
        for ri in 0..<radialResolution {
            for ti in 0..<tubularResolution {
                let u = Float(ti) / Float(tubularResolution) * 2 * .pi
                let v = Float(ri) / Float(radialResolution) * 2 * .pi
                mesh.positions.append([(radius + thickness * cos(v)) * cos(u), (radius + thickness * cos(v)) * sin(u), thickness * sin(v)])
            }
        }
        for r1 in 0..<radialResolution {
            let r2 = (r1 + 1) % radialResolution
            for t1 in 0..<tubularResolution {
                let t2 = (t1 + 1) % tubularResolution
                let i1 = UInt32(r1 * tubularResolution + t1), i2 = UInt32(r1 * tubularResolution + t2)
                let i3 = UInt32(r2 * tubularResolution + t2), i4 = UInt32(r2 * tubularResolution + t1)
                mesh.indices += [i1, i2, i3, i3, i4, i1]
            }
        }
        return mesh
    }
}

// Port of Apple's sample "Simulating dice rolls as a component for your game":
// https://developer.apple.com/documentation/tabletopkit/simulating-dice-rolls-as-a-component-for-your-game
//
// Apple's seven dice lie in a row on a round table 1 m across. Pick one up, and throw it, or all of
// them with "Toss all"; the score of the last roll is the sum of the faces on top. Differences from
// Apple's sample, which builds on TabletopKit:
//
// - The dice are Box3D convex hulls of Apple's models, on a Box3D table, in a simulation whose root
//   is scaled to centimeters, Box3D's length unit for this app. TabletopKit's
//   `TossableRepresentation` is visionOS only.
// - Invisible walls around the table keep the dice on it, as TabletopKit's table does.
// - The dice are damped, standing in for the rolling and spinning resistance that stops real
//   dice, which Box3D only models for spheres and capsules.
// - The pointer holds a die by the point it grabbed, with a critically damped spring that carries
//   its weight, and "Toss all" pulls the other dice into Apple's hexagon around it the same way.
//   Released dice keep their velocity, plus a random spin.
// - When the dice are released, `B3World.forecast()` copies the world exactly, contacts and all,
//   from the snapshot that opens a Box3D recording. Box3D is deterministic, so stepping the copy
//   ahead a few milliseconds of each frame finds the roll while the dice still tumble.
// - "Max score toss": TabletopKit decides the outcome when a toss starts and makes the dice land
//   on it. Here, once the forecast knows how each die lands, the die's model is turned by one of
//   its symmetries, so that its highest face is the one that lands on top. A die's shape is the
//   same after the turn, so its flight doesn't change, and the turn is invisible while it spins.
//   The real roll is never adjusted.

import Foundation
import RealityBox3D
import RealityKit
import simd
import SwiftUI

@MainActor
@Observable
package final class DiceScene {
    struct Die {
        let kind: DieKind
        /// The body, in the simulation root's centimeters
        let entity: Entity
        /// Apple's model, in meters, turned by "Max score toss"
        let model: Entity
    }

    @ObservationIgnored package let content = Entity()
    @ObservationIgnored package let camera = SampleCamera(eye: [0.09, 0.3, 0.4], target: [0.09, 0, 0], fieldOfView: 40)
    @ObservationIgnored private let root = Entity()
    @ObservationIgnored private var dice: [Die] = []

    /// A thrown die and the value on top
    package struct Roll: Identifiable {
        package var id: String { kind.rawValue }
        package let kind: DieKind
        package let value: Int
    }

    /// The faces the forecast predicted, as soon as it knows them
    package private(set) var predicted: [Roll]?
    /// The faces on top once the last toss came to rest
    package private(set) var rolled: [Roll]?
    package private(set) var isRolling = false

    // MARK: Setup

    static let tableRadius: Float = 50
    /// The table's collision group, which picking leaves out
    static let tableGroup = CollisionGroup(rawValue: 1 << 2)
    static let tableMaterial = B3PhysicsMaterial.generate(staticFriction: 0.6, dynamicFriction: 0.6, restitution: 0.3)

    package init() {
        let light = DirectionalLight()
        light.light.intensity = 3500
        light.shadow = DirectionalLightComponent.Shadow(maximumDistance: 2, depthBias: 1)
        light.look(at: .zero, from: [0.4, 1, 0.5], relativeTo: nil)
        content.addChild(light)
        let fill = DirectionalLight()
        fill.light.intensity = 1200
        fill.look(at: .zero, from: [-0.5, 0.4, -0.6], relativeTo: nil)
        content.addChild(fill)
        content.addChild(camera.makeEntity())

        root.scale = SIMD3(repeating: 0.01)
        var settings = B3PhysicsSimulationComponent()
        settings.gravity = [0, -981, 0]
        // Dice bounce from slower approaches than Box3D's default 1 m/s
        settings.restitutionThreshold = 10
        root.components.set(settings)
        content.addChild(root)
    }

    /// Loads Apple's dice models and builds their hulls
    package func load() async throws {
        let filter = CollisionFilter(group: Self.tableGroup, mask: .all)
        // A table 1 m across, 10 cm thick, its top at y = 0
        let table = Entity()
        table.position = [0, -5, 0]
        table.components.set(B3PhysicsBodyComponent(material: Self.tableMaterial, mode: .static))
        table.components.set(B3CollisionComponent(shapes: [.generateCylinder(height: 10, radius: Self.tableRadius, sides: 32)], filter: filter))
        let top = ModelEntity(mesh: .generateCylinder(height: 10, radius: Self.tableRadius),
                              materials: [SimpleMaterial(color: .init(red: 0.18, green: 0.32, blue: 0.24, alpha: 1), roughness: 0.9, isMetallic: false)])
        table.addChild(top)
        root.addChild(table)

        // 32 invisible panels, 15 cm high, around the edge
        let panels = 32, wall = B3ShapeResource.generateBox(size: [10.4, 15, 2])
        for i in 0..<panels {
            let angle = Float(i) / Float(panels) * 2 * .pi
            let panel = Entity()
            panel.position = [cos(angle) * (Self.tableRadius + 1), 7.5, sin(angle) * (Self.tableRadius + 1)]
            panel.orientation = simd_quatf(angle: -angle + .pi / 2, axis: [0, 1, 0])
            panel.components.set(B3PhysicsBodyComponent(material: Self.tableMaterial, mode: .static))
            panel.components.set(B3CollisionComponent(shapes: [wall], filter: filter))
            root.addChild(panel)
        }

        for (index, kind) in DieKind.allCases.enumerated() {
            guard let url = Bundle.module.url(forResource: kind.rawValue, withExtension: "usdz", subdirectory: "Models") else {
                throw CocoaError(.fileNoSuchFile)
            }
            let model = try await Entity(contentsOf: url)
            let entity = Entity()
            entity.name = kind.rawValue
            // The model is in meters, the simulation in centimeters
            entity.scale = SIMD3(repeating: 100)
            entity.addChild(model)
            // Plastic: 1.2 g per cm³. The shape is the die's exact polyhedron: Box3D's hull of the
            // rounded model has dozens of nearly coplanar facets, and a die resting on them rocks.
            let shape = Self.polyhedron(kind, enclosing: model)
            var body = B3PhysicsBodyComponent(density: 0.0012,
                                              material: .generate(staticFriction: 0.5, dynamicFriction: 0.5, restitution: 0.35))
            // Real dice stop within a second or two from rolling and spinning resistance, which
            // Box3D only models for spheres and capsules. Without it, a D4 spinning on a vertex
            // keeps going like a top for half a minute. RealitySDFPhysics's port uses the same values.
            body.linearDamping = 0.5
            body.angularDamping = 3
            entity.components.set(body)
            entity.components.set(B3CollisionComponent(shapes: [shape]))
            entity.components.set(B3PhysicsMotionComponent())
            // Apple's layout: a row across the center, 6 cm apart
            entity.position = Self.restingPosition(index)
            root.addChild(entity)
            dice.append(Die(kind: kind, entity: entity, model: model))
        }
    }

    /// The convex polyhedron bounded by the die's face planes, each pushed out to the model's
    /// farthest point along its normal: the die with sharp edges, in the model's frame.
    static func polyhedron(_ kind: DieKind, enclosing model: Entity) -> B3ShapeResource {
        guard case .convex(let points) = B3ShapeResource.generateConvex(from: model).geometry else {
            return .generateConvex(from: model)
        }
        // The hull's points are the model's extremes, enough to find each face plane's distance.
        var all = points
        func visit(_ e: Entity) {
            if let mesh = e.components[ModelComponent.self]?.mesh {
                let m = e.transformMatrix(relativeTo: model)
                for part in mesh.contents.models.flatMap({ $0.parts }) {
                    for p in part.positions { let w = m * SIMD4(p, 1); all.append(SIMD3(w.x, w.y, w.z)) }
                }
            }
            e.children.forEach(visit)
        }
        visit(model)
        let planes = kind.faces.map { f in (n: f.normal, d: all.map { simd_dot($0, f.normal) }.max()!) }
        var vertices: [SIMD3<Float>] = []
        for i in 0..<planes.count {
            for j in (i + 1)..<planes.count {
                for k in (j + 1)..<planes.count {
                    let (a, b, c) = (planes[i], planes[j], planes[k])
                    let det = simd_dot(a.n, simd_cross(b.n, c.n))
                    guard abs(det) > 1e-4 else { continue }
                    let x = (a.d * simd_cross(b.n, c.n) + b.d * simd_cross(c.n, a.n) + c.d * simd_cross(a.n, b.n)) / det
                    let scale = max(1e-4, all.map { simd_length($0) }.max()!)
                    if planes.allSatisfy({ simd_dot($0.n, x) <= $0.d + 1e-3 * scale }) { vertices.append(x) }
                }
            }
        }
        // The listed normals have four decimals, so where more than three faces meet, the plane
        // triples give slightly different points. Merge them, or Box3D folds the faces in two.
        let size = all.map { simd_length($0) }.max()!
        var merged: [(sum: SIMD3<Float>, count: Float)] = []
        for v in vertices {
            if let i = merged.firstIndex(where: { simd_distance($0.sum / $0.count, v) < 0.05 * size }) {
                merged[i].sum += v; merged[i].count += 1
            } else {
                merged.append((v, 1))
            }
        }
        let corners = merged.map { $0.sum / $0.count }
        return corners.count >= 4 ? .generateConvex(from: corners) : .generateConvex(from: model)
    }

    private static func restingPosition(_ index: Int) -> SIMD3<Float> { [-15 + 6 * Float(index + 1), 1.5, 0] }

    // MARK: Picking up and tossing

    private var world: B3World? { RealityBox3D.world(for: root) }
    private var gravity: SIMD3<Float> { root.components[B3PhysicsSimulationComponent.self]!.gravity }
    /// The held die, and the others pulled around it with "Toss all"
    @ObservationIgnored private var held: [(index: Int, offset: SIMD3<Float>)] = []
    /// The grabbed point, in the held die's frame, and where the pointer wants it, in the root's
    @ObservationIgnored private var grab: (anchor: SIMD3<Float>, target: SIMD3<Float>)?
    @ObservationIgnored private var holding: B3Subscription?
    /// The height the dice are held at, in centimeters
    private let holdHeight: Float = 6

    /// Grabs the die under a point of the view, and with `tossAll` pulls the others around it
    package func pickUp(at point: CGPoint, in size: CGSize, tossAll: Bool) -> Bool {
        guard let world else { return false }
        let ray = camera.ray(through: point, in: size)
        guard let hit = world.raycast(origin: root.convert(position: ray.origin, from: nil),
                                      direction: root.convert(direction: ray.direction, from: nil), length: 1000,
                                      query: .nearest, mask: .all.subtracting(Self.tableGroup)).first,
              let index = dice.firstIndex(where: { $0.entity == hit.entity }) else { return false }
        pickUp(die: index, at: hit.position, tossAll: tossAll)
        return true
    }

    /// Grabs a die by its index at a point (by default the top of its center, in the root's frame)
    package func pickUp(die index: Int, at point: SIMD3<Float>? = nil, tossAll: Bool) {
        guard let world, let pose = dice[index].entity.b3.pose else { return }
        let point = point ?? pose.translation + [0, 1, 0]
        held = [(index, .zero)]
        if tossAll {
            // Apple's hexagon of the other dice around the held one, 10 cm across
            for (slot, other) in dice.indices.filter({ $0 != index }).enumerated() {
                let angle = Float(slot) * .pi / 3
                held.append((other, [10 * cos(angle), 0, 10 * sin(angle)]))
            }
        }
        // The anchor in the body's own frame, from the live pose
        grab = (pose.rotation.inverse.act(point - pose.translation), point)
        holding = world.subscribe(to: B3PhysicsSimulationEvents.WillSimulate.self) { [unowned self] _ in hold() }
        stopWatchingRoll()
    }

    /// Moves the grabbed point's target under a point of the view, at the hold height
    package func move(to point: CGPoint, in size: CGSize) {
        let ray = camera.ray(through: point, in: size)
        let origin = root.convert(position: ray.origin, from: nil), direction = root.convert(direction: ray.direction, from: nil)
        guard grab != nil, abs(direction.y) > 1e-6 else { return }
        grab?.target = origin + direction * ((holdHeight - origin.y) / direction.y)
    }

    /// Before every step: pulls the grabbed point towards its target, and the other dice towards
    /// their places around the held one, with critically damped springs that also carry the dice's
    /// weight and calm their spin
    private func hold() {
        guard let grab, let (index, _) = held.first else { return }
        let entity = dice[index].entity
        guard let mass = entity.b3.mass, let pose = entity.b3.pose,
              let v = entity.b3.linearVelocity, let w = entity.b3.angularVelocity else { return }
        let frequency: Float = 15 // radians per second: the die follows within about a tenth of a second
        let anchor = pose.translation + pose.rotation.act(grab.anchor)
        let anchorVelocity = v + cross(w, anchor - pose.translation)
        let pull = frequency * frequency * (grab.target - anchor) - 2 * frequency * anchorVelocity
        entity.b3.addForce(mass * pull, at: anchor, relativeTo: root)
        entity.b3.addForce(-mass * gravity, relativeTo: root)
        entity.b3.addTorque(-mass * 2 * w, relativeTo: root)

        let center = pose.translation
        for (other, offset) in held.dropFirst() {
            let e = dice[other].entity
            guard let mass = e.b3.mass, let p = e.b3.pose, let v = e.b3.linearVelocity else { continue }
            let pull = frequency * frequency * (center + offset - p.translation) - 2 * frequency * v
            e.b3.addForce(mass * (pull - gravity), relativeTo: root)
        }
    }

    /// Lets the held dice go with their velocities and a random spin, and forecasts the roll
    package func toss(maxScore: Bool) {
        toss(maxScore: maxScore, spin: { _ in SIMD3<Float>.random(in: -15...15) })
    }

    /// Tosses with a given extra spin per die, for reproducible tests.
    package func toss(maxScore: Bool, spin: (Int) -> SIMD3<Float>) {
        guard !held.isEmpty else { return }
        holding = nil
        grab = nil
        for (index, _) in held {
            let entity = dice[index].entity
            guard var v = entity.b3.linearVelocity, let w = entity.b3.angularVelocity else { continue }
            // At most 2 m/s
            if length(v) > 200 { v = normalize(v) * 200 }
            entity.b3.setVelocity(linear: v, angular: w + spin(index))
        }
        let tossed = held.map(\.index).sorted()
        held = []
        guard let forecast = world?.forecast() else { return }
        watchRoll(tossed: tossed, forecast: forecast, maxScore: maxScore)
    }

    private static func isAtRest(_ v: SIMD3<Float>?, _ w: SIMD3<Float>?) -> Bool {
        length(v ?? .zero) < 0.5 && length(w ?? .zero) < 0.05
    }

    // MARK: Scoring

    @ObservationIgnored private var rollSubscription: B3Subscription?

    /// After every step of the roll, advances the forecast for a few milliseconds until its dice
    /// rest, then waits for the real dice to rest and reads them
    private func watchRoll(tossed: [Int], forecast: B3Forecast, maxScore: Bool) {
        predicted = nil
        rolled = nil
        isRolling = true
        var forecastResting = 0, resting = 0
        rollSubscription = world?.subscribe(to: B3PhysicsSimulationEvents.DidSimulate.self) { [unowned self] _ in
            if predicted == nil {
                // A few milliseconds of each step, so that the roll stays smooth
                let budget = Date().addingTimeInterval(0.004)
                while Date() < budget && forecastResting < 30 && forecast.stepCount < 3600 {
                    forecast.step()
                    forecastResting = dice.allSatisfy { d in
                        let m = forecast.motion(of: d.entity)
                        return Self.isAtRest(m?.linearVelocity, m?.angularVelocity)
                    } ? forecastResting + 1 : 0
                }
                if forecastResting >= 30 || forecast.stepCount >= 3600 {
                    let landings = tossed.map { forecast.transform(of: dice[$0].entity)!.rotation }
                    for (index, landing) in zip(tossed, landings) where maxScore {
                        let kind = dice[index].kind
                        if let top = kind.faces.max(by: { $0.value < $1.value }),
                           let symmetry = kind.symmetry(mapping: top, to: kind.face(for: landing)) {
                            dice[index].model.orientation = symmetry
                        }
                    }
                    predicted = zip(tossed, landings).map { read(index: $0, orientation: $1) }
                }
            }
            resting = dice.allSatisfy { Self.isAtRest($0.entity.b3.linearVelocity, $0.entity.b3.angularVelocity) } ? resting + 1 : 0
            guard predicted != nil, resting >= 30 else { return }
            rolled = tossed.map { read(index: $0, orientation: dice[$0].entity.b3.pose?.rotation ?? dice[$0].entity.orientation) }
            isRolling = false
            rollSubscription = nil
        }
    }

    /// The face on top of a die with a body orientation
    private func read(index: Int, orientation: simd_quatf) -> Roll {
        Roll(kind: dice[index].kind, value: dice[index].kind.face(for: orientation * dice[index].model.orientation).value)
    }

    private func stopWatchingRoll() {
        rollSubscription = nil
        isRolling = false
    }

    /// Puts the dice back in their row, with their models unturned
    package func reset() {
        held = []
        holding = nil
        grab = nil
        stopWatchingRoll()
        for (index, die) in dice.enumerated() {
            die.entity.transform = Transform(scale: SIMD3(repeating: 100), translation: Self.restingPosition(index))
            die.entity.b3.setVelocity(linear: .zero, angular: .zero)
            die.model.orientation = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        }
        predicted = nil
        rolled = nil
    }

    // MARK: Headless use, for tests and tools

    /// The simulation root, for driving the scene without a RealityKit scene.
    package var simulationRoot: Entity { root }
    package var dieEntities: [Entity] { dice.map(\.entity) }
    package func move(heldTo target: SIMD3<Float>) { grab?.target = target }
}

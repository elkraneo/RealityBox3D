// Simulates demo scenes headlessly and renders each offscreen with RealityKit to a PNG.
//
//   swift run -c release b3-snapshot <output directory> [seconds] [scene name...]
//   swift run -c release b3-snapshot <output directory> --samples   (the RealityKit sample ports)
//
// Seconds default to 4. Scene names match DemoScene names, case-insensitively.

import Box3DDemoScenes
import CoreGraphics
import Foundation
import ImageIO
import Metal
import RealityBox3D
import RealityKit
import Box3DRealityKitSampleScenes
import UniformTypeIdentifiers

let arguments = Array(CommandLine.arguments.dropFirst())
if arguments.contains("--samples"), let output = arguments.first {
    try await renderSamples(to: output)
    exit(0)
}
guard let output = arguments.first else {
    print("usage: b3-snapshot <output directory> [seconds] [scene name...]"); exit(1)
}
let seconds = arguments.count > 1 ? Double(arguments[1]) : nil
let names = Array(arguments.dropFirst(seconds == nil ? 1 : 2))
let scenes = names.isEmpty ? DemoScene.all : names.compactMap { name in
    DemoScene.named(name) ?? { print("unknown scene \(name)"); return nil }()
}

do {
    try FileManager.default.createDirectory(atPath: output, withIntermediateDirectories: true)
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: 1280, height: 800, mipmapped: false)
    descriptor.usage = [.renderTarget, .shaderRead]
    descriptor.storageMode = .shared
    let texture = MTLCreateSystemDefaultDevice()!.makeTexture(descriptor: descriptor)!

    print("scene                       bodies  awake   frames  box3d ms/frame  sync ms/frame")
    for (index, scene) in scenes.enumerated() {
        let runner = DemoRunner(scene) { $0.interpolatesPoses = false }
        let frames = Int((seconds ?? 4) * 60)
        var box3d = 0.0, sync = 0.0
        for _ in 0..<frames {
            runner.advance()
            let s = runner.world.statistics
            box3d += s.stepMilliseconds
            sync += s.syncInMilliseconds + s.syncOutMilliseconds
        }

        let renderer = try RealityRenderer()
        let camera = DemoStage.makeCamera()
        scene.camera.place(camera)
        let light = DemoStage.makeLights()
        renderer.entities.append(contentsOf: [runner.root, light, camera])
        renderer.activeCamera = camera
        renderer.cameraSettings.colorBackground = .color(CGColor(red: 0.16, green: 0.18, blue: 0.22, alpha: 1))
        let cameraOutput = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: texture))
        try await withCheckedThrowingContinuation { (done: CheckedContinuation<Void, any Error>) in
            do {
                try renderer.updateAndRender(deltaTime: 0, cameraOutput: cameraOutput, onComplete: { _ in done.resume() })
            } catch { done.resume(throwing: error) }
        }
        let file = String(format: "%02d-%@-%@.png", index, scene.category, scene.name.replacingOccurrences(of: " ", with: "-"))
        try writePNG(texture, to: URL(fileURLWithPath: output).appendingPathComponent(file))
        let s = runner.world.statistics
        print(scene.name.padding(toLength: 27, withPad: " ", startingAt: 0)
            + String(format: " %6d %6d %8d %15.2f %14.2f", s.bodies, s.awakeBodies, frames, box3d / Double(frames), sync / Double(frames)))
    }
} catch {
    print("error: \(error)"); exit(1)
}

func writePNG(_ texture: MTLTexture, to url: URL) throws {
    let bytesPerRow = texture.width * 4
    let info = CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue)
    guard let context = CGContext(data: nil, width: texture.width, height: texture.height, bitsPerComponent: 8,
                                  bytesPerRow: bytesPerRow, space: CGColorSpace(name: CGColorSpace.sRGB)!, bitmapInfo: info.rawValue),
          let pixels = context.data else { throw CocoaError(.fileWriteUnknown) }
    texture.getBytes(pixels, bytesPerRow: bytesPerRow, from: MTLRegionMake2D(0, 0, texture.width, texture.height), mipmapLevel: 0)
    guard let image = context.makeImage(),
          let destination = CGImageDestinationCreateWithURL(url as CFURL, UTType.png.identifier as CFString, 1, nil)
    else { throw CocoaError(.fileWriteUnknown) }
    CGImageDestinationAddImage(destination, image, nil)
    guard CGImageDestinationFinalize(destination) else { throw CocoaError(.fileWriteUnknown) }
}

/// Renders the RealityKit sample ports: Physics Bodies after 3 s, and the dice after a toss of all.
@MainActor
func renderSamples(to output: String) async throws {
    try FileManager.default.createDirectory(atPath: output, withIntermediateDirectories: true)
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: 1280, height: 800, mipmapped: false)
    descriptor.usage = [.renderTarget, .shaderRead]
    descriptor.storageMode = .shared
    let texture = MTLCreateSystemDefaultDevice()!.makeTexture(descriptor: descriptor)!
    func bodies(_ root: Entity) -> [Entity] {
        var out: [Entity] = []
        func visit(_ e: Entity) { if e.components.has(B3PhysicsBodyComponent.self) { out.append(e) }; e.children.forEach(visit) }
        visit(root); return out
    }
    func render(_ content: Entity, camera: SampleCamera, _ name: String) async throws {
        let renderer = try RealityRenderer()
        let cam = camera.makeEntity()
        renderer.entities.append(content)
        renderer.entities.append(cam)
        renderer.activeCamera = cam
        renderer.cameraSettings.colorBackground = .color(CGColor(red: 0.16, green: 0.18, blue: 0.22, alpha: 1))
        let out = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: texture))
        try await withCheckedThrowingContinuation { (done: CheckedContinuation<Void, any Error>) in
            do { try renderer.updateAndRender(deltaTime: 0, cameraOutput: out, onComplete: { _ in done.resume() }) }
            catch { done.resume(throwing: error) }
        }
        try writePNG(texture, to: URL(fileURLWithPath: output).appendingPathComponent(name))
        renderer.entities.removeAll()
    }

    // Physics Bodies, spheres and tori
    AttractionComponent.registerComponent()
    for shape in PhysicsBodiesScene.Shape.allCases {
        let scene = PhysicsBodiesScene()
        let root = scene.content.children.first { $0.components.has(B3PhysicsSimulationComponent.self) }!
        let world = B3World(root: root, settings: root.components[B3PhysicsSimulationComponent.self]!)
        scene.reset(shape: shape) // the world exists, so tori start spawning one by one
        for _ in 0..<240 {
            // Tori are added while the world steps, so collect the bodies every frame
            let list = bodies(root)
            let attracted = list.filter { $0.components.has(AttractionComponent.self) }
            // AttractionSystem's forces, applied by hand since there is no RealityKit scene
            for b in attracted {
                let p = b.position(relativeTo: nil)
                var f = -0.05 * (p / length(p)) / pow(length(p), 2)
                for n in attracted where n != b {
                    let o = n.position(relativeTo: nil) - p
                    f += 0.1 * (o / length(o)) / pow(length(o), 2)
                }
                b.b3.addForce(f, relativeTo: nil)
            }
            world.update(entities: list, deltaTime: 1.0 / 60)
        }
        try await render(scene.content, camera: scene.camera, "physics-bodies-\(shape.rawValue.lowercased()).png")
        print("Physics Bodies (\(shape.rawValue)): \(world.bodyCount) bodies")
    }

    // Dice
    let dice = DiceScene()
    try await dice.load()
    let root = dice.simulationRoot
    let world = B3World(root: root, settings: root.components[B3PhysicsSimulationComponent.self]!)
    let list = bodies(root)
    func frames(_ n: Int) { for _ in 0..<n { world.update(entities: list, deltaTime: 1.0 / 60) } }
    frames(60)
    try await render(dice.content, camera: dice.camera, "dice-row.png")
    dice.pickUp(die: 1, tossAll: true)
    for i in 0..<45 { dice.move(heldTo: [-6 + Float(i) * 0.6, 8, -4]); frames(1) }
    try await render(dice.content, camera: dice.camera, "dice-held.png")
    dice.toss(maxScore: false)
    var n = 0
    while dice.rolled == nil && n < 1200 { frames(1); n += 1 }
    try await render(dice.content, camera: dice.camera, "dice-rolled.png")
    print("Dice: predicted \(dice.predicted?.map(\.value) ?? []), rolled \(dice.rolled?.map(\.value) ?? []) after \(n) frames")
}

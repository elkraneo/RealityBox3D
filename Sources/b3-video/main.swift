// Renders short videos of the demo scenes and the RealityKit sample ports offscreen, with a HUD
// stating what runs.
//
//   swift run -c release b3-video <output.mp4> [demos | mover | dice | cradle]
//
// Every frame is one 60 Hz step, so the videos play in real time. `demos` is a reel of Box3D's
// sample scenes; `mover` walks the Geometric Mover's character over upstream's test level on a
// scripted route; `dice` throws Apple's dice with the forecast in the HUD; `cradle` runs Apple's
// Newton's cradle with Box3D's solver alone, then with `sequentialImpacts`.

import AppKit
import AVFoundation
import Box3DDemoScenes
import Box3DRealityKitSampleScenes
import CoreMedia
import CoreText
import Metal
import RealityBox3D
import RealityKit

let width = 1280, height = 720, framesPerSecond: Int32 = 60
let boxCredit = "Box3D by Erin Catto (MIT) · scenes from Box3D's samples, rebuilt on RealityBox3D's public API"

let arguments = Array(CommandLine.arguments.dropFirst())
let reels = ["demos", "mover", "dice", "cradle"]
let reel = arguments.count > 1 ? arguments[1] : "demos"
guard let output = arguments.first, reels.contains(reel) else {
    print("usage: b3-video <output.mp4> [\(reels.joined(separator: " | "))]")
    exit(1)
}

/// Writes rendered frames, with a HUD, to an H.264 MP4
@MainActor
final class Recorder {
    let texture: MTLTexture
    let cameraOutput: RealityRenderer.CameraOutput
    private let writer: AVAssetWriter
    private let input: AVAssetWriterInput
    private let adaptor: AVAssetWriterInputPixelBufferAdaptor
    private var frame: Int64 = 0

    init(url: URL) throws {
        try? FileManager.default.removeItem(at: url)
        writer = try AVAssetWriter(outputURL: url, fileType: .mp4)
        input = AVAssetWriterInput(mediaType: .video, outputSettings: [
            AVVideoCodecKey: AVVideoCodecType.h264, AVVideoWidthKey: width, AVVideoHeightKey: height,
            AVVideoCompressionPropertiesKey: [AVVideoAverageBitRateKey: 10_000_000],
        ])
        adaptor = AVAssetWriterInputPixelBufferAdaptor(assetWriterInput: input, sourcePixelBufferAttributes: [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA,
            kCVPixelBufferWidthKey as String: width, kCVPixelBufferHeightKey as String: height,
        ])
        writer.add(input)
        guard writer.startWriting() else { throw writer.error ?? CocoaError(.fileWriteUnknown) }
        writer.startSession(atSourceTime: .zero)

        let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: width, height: height,
                                                                  mipmapped: false)
        descriptor.usage = [.renderTarget, .shaderRead]
        descriptor.storageMode = .shared
        texture = MTLCreateSystemDefaultDevice()!.makeTexture(descriptor: descriptor)!
        cameraOutput = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: texture))
    }

    /// Updates the renderer's scene by a time, which runs the RealityKit systems, and renders it.
    /// Demo scenes pass no time: their DemoRunner steps them.
    func render(_ renderer: RealityRenderer, deltaTime: TimeInterval = 0) async throws {
        try await withCheckedThrowingContinuation { (done: CheckedContinuation<Void, any Error>) in
            do {
                try renderer.updateAndRender(deltaTime: deltaTime, cameraOutput: cameraOutput, onComplete: { _ in done.resume() })
            } catch {
                done.resume(throwing: error)
            }
        }
    }

    /// Appends the rendered frame with HUD lines
    func append(lines: [String], credit: String = boxCredit) async throws {
        while !input.isReadyForMoreMediaData { try await Task.sleep(for: .milliseconds(2)) }
        guard let pool = adaptor.pixelBufferPool else { throw writer.error ?? CocoaError(.fileWriteUnknown) }
        var buffer: CVPixelBuffer?
        CVPixelBufferPoolCreatePixelBuffer(nil, pool, &buffer)
        guard let buffer else { throw CocoaError(.fileWriteUnknown) }

        CVPixelBufferLockBaseAddress(buffer, [])
        let base = CVPixelBufferGetBaseAddress(buffer)!, bytesPerRow = CVPixelBufferGetBytesPerRow(buffer)
        texture.getBytes(base, bytesPerRow: bytesPerRow, from: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0)
        drawHUD(into: base, bytesPerRow: bytesPerRow, lines: lines, credit: credit)
        CVPixelBufferUnlockBaseAddress(buffer, [])

        adaptor.append(buffer, withPresentationTime: CMTime(value: frame, timescale: framesPerSecond))
        frame += 1
    }

    func finish() async throws -> Int64 {
        input.markAsFinished()
        await writer.finishWriting()
        guard writer.status == .completed else { throw writer.error ?? CocoaError(.fileWriteUnknown) }
        return frame
    }
}

// MARK: Reels

/// A renderer for a demo scene: the demos app's lights and background, and its camera
@MainActor
func makeDemoRenderer(_ runner: DemoRunner, camera orbit: OrbitCamera) throws -> (RealityRenderer, PerspectiveCamera, Entity) {
    let renderer = try RealityRenderer()
    let camera = DemoStage.makeCamera()
    orbit.place(camera)
    let lights = DemoStage.makeLights()
    DemoStage.fitShadows(lights, to: orbit)
    renderer.entities.append(contentsOf: [runner.root, lights, camera])
    renderer.activeCamera = camera
    renderer.cameraSettings.colorBackground = .color(CGColor(red: 0.16, green: 0.18, blue: 0.22, alpha: 1))
    return (renderer, camera, lights)
}

/// Box3D's sample scenes, each with a slowly orbiting camera
@MainActor
func renderDemos(_ recorder: Recorder) async throws {
    // Scenes that move on their own, with their length in seconds and a zoom on upstream's camera
    let reel: [(name: String, seconds: Double, zoom: Float)] = [
        ("Convex Pile", 5, 2.2),
        ("Ball and Chain", 4, 1),
        ("Ragdoll", 3.5, 0.5),
        ("Conveyor Belt", 4.5, 0.55),
        ("Wind", 4, 1),
        ("Spinning Book", 4, 1),
    ]
    for item in reel {
        guard let scene = DemoScene.named(item.name) else { print("unknown scene \(item.name)"); continue }
        let runner = DemoRunner(scene) { $0.interpolatesPoses = false }
        var orbit = scene.camera
        orbit.distance *= item.zoom
        let (renderer, camera, _) = try makeDemoRenderer(runner, camera: orbit)
        var milliseconds: [Double] = []
        for _ in 0..<Int(item.seconds * Double(framesPerSecond)) {
            runner.advance()
            milliseconds = Array((milliseconds + [runner.world.statistics.stepMilliseconds]).suffix(30))
            orbit.yaw += 0.002
            orbit.place(camera)
            try await recorder.render(renderer)
            let stats = runner.world.statistics
            let mean = milliseconds.reduce(0, +) / Double(milliseconds.count)
            try await recorder.append(lines: [
                "RealityBox3D · Box3D rigid bodies in RealityKit",
                "\(scene.category.lowercased()) · \(scene.name.lowercased())",
                "\(stats.bodies) bodies · \(stats.awakeBodies) awake · \(String(format: "%.2f", mean)) ms/step",
            ])
        }
        print("\(scene.name): \(item.seconds) s")
    }
}

/// The Geometric Mover's character walks a route of waypoints: past the soft and the hard capsule,
/// past the door, around and up the stairs, and off their top
@MainActor
func renderMover(_ recorder: Recorder) async throws {
    guard let scene = DemoScene.named("Geometric Mover") else { return }
    let runner = DemoRunner(scene) { $0.interpolatesPoses = false }
    guard let character = runner.root.findEntity(named: "character") else { return }
    // Waypoints in the level's xz plane, with whether to sprint and jump on the way
    let route: [(point: SIMD2<Float>, sprint: Bool, jump: Bool, label: String)] = [
        ([1.2, 5.5], false, false, "past the soft green capsule"),
        ([-1, 3], false, false, "and around the hard violet one"),
        ([-6, 3], true, false, "sprinting past the door"),
        ([-13, 2.5], true, false, "sprinting past the door"),
        ([-13.5, 0], false, false, "to the foot of the stairs"),
        ([-9.3, 0], false, false, "up the stairs: a sprung ray floats the capsule"),
        ([-5, 0], false, true, "and a jump off the top"),
        ([-1, -2], false, false, "and a jump off the top"),
    ]
    var orbit = OrbitCamera(yaw: 120, pitch: 22, distance: 6, target: character.position)
    let (renderer, camera, lights) = try makeDemoRenderer(runner, camera: orbit)
    DemoStage.fitShadows(lights, to: orbit)
    var waypoint = 0, label = route[0].label, frames = 0, idle = 0
    var heading = SIMD2<Float>(-1, -0.6), view = heading
    while frames < 30 * Int(framesPerSecond) {
        let p = character.position
        var input = DemoInput()
        if waypoint < route.count {
            let target = route[waypoint]
            let offset = target.point - SIMD2(p.x, p.z)
            if simd_length(offset) < 0.8 {
                waypoint += 1
                if waypoint < route.count { label = route[waypoint].label }
                continue
            }
            heading = simd_normalize(offset)
            input.throttle = [1, 0]
            input.sprint = target.sprint
            input.jump = target.jump && simd_length(offset) < 3
        } else {
            idle += 1
            if idle > 75 { break }
        }
        DemoScene.setInput(input, cameraForward: [heading.x, 0, heading.y], root: runner.root)
        runner.advance()
        frames += 1
        if ProcessInfo.processInfo.environment["B3_VIDEO_TRACE"] != nil, frames % 30 == 0 {
            print(frames, waypoint, character.position)
        }

        // A third-person camera behind the walking direction, easing after it
        view = simd_normalize(simd_mix(view, heading, SIMD2(repeating: 0.04)))
        let yaw = atan2(-view.x, -view.y)
        var turn = yaw - orbit.yaw
        turn = atan2(sin(turn), cos(turn))
        orbit.yaw += 0.05 * turn
        orbit.target += 0.15 * (character.position + [0, 0.4, 0] - orbit.target)
        orbit.place(camera)
        try await recorder.render(renderer)
        let speed = simd_length((character.position - p) * SIMD3(1, 0, 1)) * Float(framesPerSecond)
        try await recorder.append(lines: [
            "RealityBox3D · B3CharacterControllerComponent on Box3D's mover",
            "geometric mover: \(label)",
            "\(String(format: "%.1f", speed)) m/s · scripted input, the same keys the demos app reads",
        ], credit: "Box3D by Erin Catto (MIT) · upstream's test level and mover sample, ported")
    }
    print("Geometric Mover: \(frames) frames, reached \(waypoint) of \(route.count) waypoints")
}

/// A renderer for a sample's scene, with its own lights and camera
@MainActor
func makeSampleRenderer(content: Entity, camera: SampleCamera, background: CGColor = CGColor(red: 0.12, green: 0.13, blue: 0.15, alpha: 1)) throws -> RealityRenderer {
    let renderer = try RealityRenderer()
    let entity = camera.makeEntity()
    renderer.entities.append(contentsOf: [content, entity])
    renderer.activeCamera = entity
    renderer.cameraSettings.colorBackground = .color(background)
    return renderer
}

/// Apple's dice sample: all seven dice gathered, thrown, and forecast to land on their highest faces
@MainActor
func renderDice(_ recorder: Recorder) async throws {
    let scene = DiceScene()
    try await scene.load()
    let renderer = try makeSampleRenderer(content: scene.content, camera: scene.camera)
    func describe(_ rolls: [DiceScene.Roll]) -> String {
        rolls.map { "\($0.kind.label) \($0.value)" }.joined(separator: " · ") + " = \(rolls.map(\.value).reduce(0, +))"
    }
    var tossed: Int?
    for frame in 0..<Int(8 * Double(framesPerSecond)) {
        switch frame {
        case 30: scene.pickUp(die: 1, tossAll: true)
        case 31..<85: scene.move(heldTo: [-6 + Float(frame - 31) * 0.5, 8, -4])
        case 85: scene.toss(maxScore: true); tossed = frame
        default: break
        }
        try await recorder.render(renderer, deltaTime: 1.0 / 60)
        var lines = ["Apple's \"Simulating dice rolls\", ported to RealityBox3D", "toss all, max score"]
        if let rolled = scene.rolled {
            let asForecast = scene.predicted.map { $0.map(\.value) == rolled.map(\.value) } ?? false
            lines.append("rolled    \(describe(rolled))\(asForecast ? ", as forecast" : "")")
        } else if let predicted = scene.predicted {
            lines.append("forecast  \(describe(predicted))")
        } else if tossed != nil {
            lines.append("forecasting…")
        }
        try await recorder.append(lines: lines, credit: "Dice models from Apple's sample · each die its exact polyhedron · forecast with B3World.forecast()")
    }
    print("Dice: predicted \(scene.predicted?.map(\.value) ?? []), rolled \(scene.rolled?.map(\.value) ?? [])")
}

/// Apple's Newton's cradle: two balls let go with Box3D's solver alone, then with sequential impacts
@MainActor
func renderCradle(_ recorder: Recorder) async throws {
    for sequential in [false, true] {
        let scene = NewtonsCradleScene()
        scene.root.components[B3PhysicsSimulationComponent.self]?.sequentialImpacts = sequential
        // Apple's balls are black: a light background keeps them readable at blog size
        let renderer = try makeSampleRenderer(content: scene.content, camera: scene.camera,
                                              background: CGColor(red: 0.62, green: 0.66, blue: 0.72, alpha: 1))
        // A first frame creates the world, then the balls are lifted in it
        try await recorder.render(renderer, deltaTime: 1.0 / 60)
        scene.lift(2)
        for _ in 0..<Int(5 * Double(framesPerSecond)) {
            try await recorder.render(renderer, deltaTime: 1.0 / 60)
            try await recorder.append(lines: [
                "Apple's \"Simulating physics joints\", ported to RealityBox3D",
                "Newton's cradle on revolute joints, two balls lifted",
                sequential ? "sequentialImpacts on: impacts resolved pair by pair, two balls out"
                    : "Box3D's solver alone, as RealityKit's: the momentum spreads along the row",
            ], credit: "Apple's sample settings, sizes and joints · bounces resolved pair by pair (Guendelman, Bridson & Fedkiw 2003)")
        }
    }
}

// MARK: HUD

/// Draws a HUD panel in the top left and a credit in the bottom left, in a monospaced font
func drawHUD(into base: UnsafeMutableRawPointer, bytesPerRow: Int, lines: [String], credit: String) {
    let info = CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue
    guard let context = CGContext(data: base, width: width, height: height, bitsPerComponent: 8, bytesPerRow: bytesPerRow,
                                  space: CGColorSpace(name: CGColorSpace.sRGB)!, bitmapInfo: info) else { return }
    let lineHeight: CGFloat = 25, margin: CGFloat = 24, padding: CGFloat = 14
    func attributed(_ text: String, color: NSColor, size: CGFloat = 17, weight: NSFont.Weight = .medium) -> CTLine {
        let font = NSFont.monospacedSystemFont(ofSize: size, weight: weight)
        return CTLineCreateWithAttributedString(NSAttributedString(string: text, attributes: [.font: font, .foregroundColor: color]))
    }
    let ctLines = lines.enumerated().map { i, text in attributed(text, color: i == 0 ? .white : NSColor(white: 0.85, alpha: 1)) }
    let panelWidth = ctLines.map { CTLineGetTypographicBounds($0, nil, nil, nil) }.max()! + 2 * padding
    let panelHeight = CGFloat(lines.count) * lineHeight + 2 * padding - 6
    // Bitmap context coordinates start at the bottom left
    let panel = CGRect(x: margin, y: CGFloat(height) - margin - panelHeight, width: panelWidth, height: panelHeight)
    context.setFillColor(CGColor(gray: 0.08, alpha: 0.72))
    context.addPath(CGPath(roundedRect: panel, cornerWidth: 10, cornerHeight: 10, transform: nil))
    context.fillPath()
    for (i, line) in ctLines.enumerated() {
        context.textPosition = CGPoint(x: panel.minX + padding, y: panel.maxY - padding - 15 - CGFloat(i) * lineHeight)
        CTLineDraw(line, context)
    }
    let creditLine = attributed(credit, color: NSColor(white: 0.8, alpha: 0.9), size: 13, weight: .regular)
    let creditWidth = CTLineGetTypographicBounds(creditLine, nil, nil, nil)
    let creditPanel = CGRect(x: margin - 8, y: margin - 8, width: creditWidth + 16, height: 26)
    context.setFillColor(CGColor(gray: 0.08, alpha: 0.6))
    context.addPath(CGPath(roundedRect: creditPanel, cornerWidth: 6, cornerHeight: 6, transform: nil))
    context.fillPath()
    context.textPosition = CGPoint(x: margin, y: margin)
    CTLineDraw(creditLine, context)
}

// MARK: Main

RealityBox3D.register()
do {
    let recorder = try Recorder(url: URL(fileURLWithPath: output))
    switch reel {
    case "mover": try await renderMover(recorder)
    case "dice": try await renderDice(recorder)
    case "cradle": try await renderCradle(recorder)
    default: try await renderDemos(recorder)
    }
    let frames = try await recorder.finish()
    print("\(output): \(frames) frames, \(String(format: "%.1f", Double(frames) / Double(framesPerSecond))) s")
} catch {
    print("error: \(error)")
    exit(1)
}

// Renders short videos of the demo scenes and the RealityKit sample ports offscreen, with a HUD
// stating what runs.
//
//   swift run -c release b3-video <output.mp4> [demos | mover | dice | cradle]
//
// Every frame is one 60 Hz step, run by RealityKit's systems as in the apps, so the videos play in
// real time and scene actions work. `demos` is a reel of Box3D's
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

    /// Updates the renderer's scene by a time, which runs the RealityKit systems, and renders it
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

/// A demo scene loaded as the demos app loads it, under a root that RealityKit's systems step:
/// `B3PhysicsSystem` runs the physics and `DemoScriptSystem` the scene's scripts and actions
@MainActor
struct DemoStageRender {
    let root = Entity()
    let renderer: RealityRenderer
    let camera = DemoStage.makeCamera()
    let lights = DemoStage.makeLights()

    init(_ scene: DemoScene, camera orbit: OrbitCamera) throws {
        var settings = B3PhysicsSimulationComponent()
        settings.workerCount = 4
        root.components.set(settings)
        scene.load(into: root)
        renderer = try RealityRenderer()
        orbit.place(camera)
        DemoStage.fitShadows(lights, to: orbit)
        renderer.entities.append(contentsOf: [root, lights, camera])
        renderer.activeCamera = camera
        renderer.cameraSettings.colorBackground = .color(CGColor(red: 0.16, green: 0.18, blue: 0.22, alpha: 1))
    }

    var statistics: B3Statistics { RealityBox3D.world(for: root)?.statistics ?? .init() }
}

/// Box3D's sample scenes, each with a slowly orbiting camera
@MainActor
func renderDemos(_ recorder: Recorder) async throws {
    // Scenes with their length in seconds, a zoom on upstream's camera, and an action with the
    // frame it runs on
    let reel: [(name: String, seconds: Double, zoom: Float, action: (name: String, frame: Int)?)] = [
        ("Dominoes", 7, 0.5, nil),
        ("Bullet vs Stack", 4, 0.55, ("Launch", 40)),
        ("Convex Pile", 5, 2.6, nil),
        ("Ball and Chain", 3.5, 1, nil),
        ("Conveyor Belt", 4, 0.55, nil),
        ("Wind", 3.5, 0.7, nil),
    ]
    for item in reel {
        guard let scene = DemoScene.named(item.name) else { print("unknown scene \(item.name)"); continue }
        var orbit = scene.camera
        orbit.distance *= item.zoom
        let stage = try DemoStageRender(scene, camera: orbit)
        var milliseconds: [Double] = []
        for frame in 0..<Int(item.seconds * Double(framesPerSecond)) {
            if let action = item.action, action.frame == frame { DemoScene.perform(action.name, root: stage.root) }
            orbit.yaw += 0.002
            orbit.place(stage.camera)
            try await recorder.render(stage.renderer, deltaTime: 1.0 / 60)
            let stats = stage.statistics
            milliseconds = Array((milliseconds + [stats.stepMilliseconds]).suffix(30))
            let mean = milliseconds.reduce(0, +) / Double(milliseconds.count)
            try await recorder.append(lines: [
                "RealityBox3D · Box3D rigid bodies in RealityKit",
                "\(scene.category.lowercased()) · \(scene.name.lowercased())" + (item.action.map { frame >= $0.frame ? " · \($0.name.lowercased())" : "" } ?? ""),
                "\(stats.bodies) bodies · \(stats.awakeBodies) awake · \(String(format: "%.2f", mean)) ms/step, 4 workers",
            ])
        }
        print("\(scene.name): \(item.seconds) s, \(stage.statistics.bodies) bodies")
    }
}

/// The Geometric Mover's character walks a route: into the hard violet capsule, which stops it,
/// into the soft green one, which it leans into, past the door, and up the stairs and off their top
@MainActor
func renderMover(_ recorder: Recorder) async throws {
    guard let scene = DemoScene.named("Geometric Mover") else { return }
    let stage = try DemoStageRender(scene, camera: scene.camera)
    guard let character = stage.root.findEntity(named: "character") else { return }
    // Waypoints in the level's xz plane. A waypoint with `hold` frames is walked towards for that
    // long whether or not it is reached, to lean into an obstacle.
    // The capsules are filmed from a fixed camera at their side; the rest from behind the character.
    let side = OrbitCamera(yaw: 90, pitch: 18, distance: 7, target: [0, 1, 5.3])
    let route: [(point: SIMD2<Float>, jump: Bool, hold: Int?, fixed: Bool, label: String)] = [
        ([2.5, 9], false, nil, false, "walking to the violet capsule"),
        ([0.3, 9], false, nil, true, "walking to the violet capsule"),
        ([0, 5.5], false, 80, true, "into the violet capsule: a hard obstacle, it stops the character"),
        ([1.4, 7], false, nil, true, "around to the green one"),
        ([1.4, 2.6], false, nil, true, "around to the green one"),
        ([0, 2.6], false, nil, true, "around to the green one"),
        ([0, 5], false, 70, true, "into the green capsule: a soft obstacle, the character leans in"),
        ([-1.5, 3], false, nil, false, "past the door"),
        ([-6, 3], false, nil, false, "past the door"),
        ([-13, 2.5], false, nil, false, "to the foot of the stairs"),
        ([-13.5, 0], false, nil, false, "to the foot of the stairs"),
        ([-9.3, 0], false, nil, false, "up the stairs: a sprung ray floats the capsule"),
        ([-5, 0], true, nil, false, "and a jump off the top"),
        ([-1, -2], false, nil, false, "and a jump off the top"),
    ]
    var orbit = OrbitCamera(yaw: 120, pitch: 22, distance: 6, target: character.position)
    DemoStage.fitShadows(stage.lights, to: orbit)
    var fixed = true
    var waypoint = 0, held = 0, label = route[0].label, frames = 0, idle = 0
    var heading = SIMD2<Float>(-1, -0.6), view = heading
    while frames < 40 * Int(framesPerSecond) {
        let p = character.position
        var input = DemoInput()
        if waypoint < route.count {
            let target = route[waypoint]
            let offset = target.point - SIMD2(p.x, p.z)
            let done = target.hold.map { held >= $0 } ?? (simd_length(offset) < 0.8)
            if done {
                waypoint += 1
                held = 0
                if waypoint < route.count { label = route[waypoint].label }
                continue
            }
            held += 1
            fixed = target.fixed
            if simd_length(offset) > 0.05 { heading = simd_normalize(offset) }
            // Leaning into an obstacle stops at its center
            input.throttle = target.hold != nil && simd_length(offset) < 0.15 ? .zero : [1, 0]
            input.jump = target.jump && simd_length(offset) < 3
        } else {
            idle += 1
            if idle > 75 { break }
        }
        DemoScene.setInput(input, cameraForward: [heading.x, 0, heading.y], root: stage.root)

        if fixed {
            side.place(stage.camera)
            // The chase camera starts from where the side camera is
            orbit = side
            view = SIMD2(-sin(side.yaw), -cos(side.yaw))
        } else {
            // A third-person camera behind the walking direction, easing after it
            view = simd_normalize(simd_mix(view, heading, SIMD2(repeating: 0.04)))
            let yaw = atan2(-view.x, -view.y)
            var turn = yaw - orbit.yaw
            turn = atan2(sin(turn), cos(turn))
            orbit.yaw += 0.05 * turn
            orbit.pitch += 0.05 * (22 * .pi / 180 - orbit.pitch)
            orbit.distance += 0.05 * (6 - orbit.distance)
            orbit.target += 0.15 * (character.position + [0, 0.4, 0] - orbit.target)
            orbit.place(stage.camera)
        }
        try await recorder.render(stage.renderer, deltaTime: 1.0 / 60)
        frames += 1
        if ProcessInfo.processInfo.environment["B3_VIDEO_TRACE"] != nil, frames % 30 == 0 {
            print(frames, waypoint, character.position)
        }
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
        try await lightWithStudio(renderer)
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

/// Lights a renderer's scene with a studio, as a RealityView's default environment would: offscreen,
/// RealityKit has none, and metallic surfaces such as the cradle's balls render black
@MainActor
func lightWithStudio(_ renderer: RealityRenderer) async throws {
    renderer.lighting.resource = try await EnvironmentResource(equirectangular: makeStudio())
    renderer.lighting.intensityExponent = 0.5
}

/// An equirectangular studio: a light ceiling with a bright strip and a softbox, a dark floor
func makeStudio() -> CGImage {
    let width = 512, height = 256
    let context = CGContext(data: nil, width: width, height: height, bitsPerComponent: 8, bytesPerRow: 0,
                            space: CGColorSpace(name: CGColorSpace.sRGB)!,
                            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)!
    let gradient = CGGradient(colorsSpace: CGColorSpace(name: CGColorSpace.sRGB)!, colors: [
        CGColor(gray: 0.12, alpha: 1), CGColor(gray: 0.3, alpha: 1), CGColor(gray: 0.85, alpha: 1), CGColor(gray: 0.6, alpha: 1),
    ] as CFArray, locations: [0, 0.48, 0.52, 1])!
    // Bitmap coordinates start at the bottom: the floor, then the horizon, then the ceiling
    context.drawLinearGradient(gradient, start: .zero, end: CGPoint(x: 0, y: height), options: [])
    context.setFillColor(CGColor(gray: 1, alpha: 1))
    context.fill(CGRect(x: 0, y: height * 3 / 4, width: width, height: 14))
    context.fill(CGRect(x: width / 3, y: height / 2 + 10, width: width / 6, height: height / 3))
    return context.makeImage()!
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
DemoScriptSystem.register()
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

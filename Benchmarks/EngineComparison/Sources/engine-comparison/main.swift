// The same piles on three engines, headless, on the machine at hand.
//
//   swift run -c release --scratch-path <dir> engine-comparison [tori|boxes] [--counts 25,100,400] [--steps 300]
//
// Every engine runs at 60 Hz for the same number of steps, with its default settings and sleeping:
//   RealityKit:      its physics, stepped by its clock; the time between its WillSimulate and
//                    DidSimulate events (RealitySDFPhysics's method)
//   RealitySDFPhysics: SDFSimulation.step(root:), real-time preset, multithreaded
//   RealityBox3D:    B3World.update, which includes reading entities in and writing poses out;
//                    "engine" is Box3D's own step inside it

import CoreMedia
import Foundation
import Metal
import RealityBox3D
import RealityKit
import RealitySDFPhysics

struct Timing {
    var mean = 0.0, worst = 0.0, engine: Double?
    var check = ""
}

func now() -> UInt64 { DispatchTime.now().uptimeNanoseconds }
func ms(_ a: UInt64, _ b: UInt64) -> Double { Double(b - a) / 1e6 }

@MainActor
func timeSDF(_ pile: Pile, count: Int, steps: Int) -> Timing {
    var settings = SDFPhysicsSimulationComponent(simulation: SDFSimulation())
    settings.isMultithreaded = true
    let (root, bodies) = pile.makeSDF(count: count, settings: settings)
    var times: [Double] = [], engine = 0.0
    for _ in 0..<steps {
        let t0 = now()
        settings.simulation.step(root: root)
        times.append(ms(t0, now()))
        engine += settings.simulation.statistics.stepMilliseconds
    }
    return Timing(mean: times.reduce(0, +) / Double(steps), worst: times.max()!, engine: engine / Double(steps), check: Pile.check(bodies))
}

@MainActor
func timeBox3D(_ pile: Pile, count: Int, steps: Int, workers: Int) -> Timing {
    let (root, bodies) = pile.makeBox3D(count: count)
    var settings = B3PhysicsSimulationComponent()
    settings.workerCount = workers
    settings.interpolatesPoses = false
    root.components.set(settings)
    let world = B3World(root: root, settings: settings)
    let all = root.children.map { $0 }
    var times: [Double] = [], engine = 0.0
    for _ in 0..<steps {
        let t0 = now()
        world.update(entities: all, deltaTime: 1.0 / 60)
        times.append(ms(t0, now()))
        engine += world.statistics.stepMilliseconds
    }
    return Timing(mean: times.reduce(0, +) / Double(steps), worst: times.max()!, engine: engine / Double(steps), check: Pile.check(bodies))
}

@MainActor
func timeRealityKit(_ pile: Pile, count: Int, steps: Int) async throws -> Timing {
    var timebase: CMTimebase?
    CMTimebaseCreateWithSourceClock(allocator: nil, sourceClock: CMClockGetHostTimeClock(), timebaseOut: &timebase)
    guard let timebase, let device = MTLCreateSystemDefaultDevice() else { throw CocoaError(.featureUnsupported) }
    CMTimebaseSetRate(timebase, rate: 0)
    CMTimebaseSetTime(timebase, time: .zero)
    let (root, bodies) = pile.makeRealityKit(count: count, clock: timebase)
    let renderer = try RealityRenderer()
    let camera = PerspectiveCamera()
    renderer.entities.append(contentsOf: [root, camera])
    renderer.activeCamera = camera
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm_srgb, width: 8, height: 8, mipmapped: false)
    descriptor.usage = [.renderTarget]
    descriptor.storageMode = .private
    let output = try RealityRenderer.CameraOutput(.singleProjection(colorTexture: device.makeTexture(descriptor: descriptor)!))
    var will: UInt64 = 0, simulated: [Double] = []
    let subscriptions = [
        renderer.subscribe(to: PhysicsSimulationEvents.WillSimulate.self) { _ in will = now() },
        renderer.subscribe(to: PhysicsSimulationEvents.DidSimulate.self) { _ in simulated.append(ms(will, now())) },
    ]
    defer { _ = subscriptions }
    for step in 1...steps {
        CMTimebaseSetTime(timebase, time: CMTime(value: CMTimeValue(step), timescale: 60))
        try await withCheckedThrowingContinuation { (done: CheckedContinuation<Void, any Error>) in
            do { try renderer.updateAndRender(deltaTime: 1.0 / 60, cameraOutput: output, onComplete: { _ in done.resume() }) }
            catch { done.resume(throwing: error) }
        }
    }
    guard !simulated.isEmpty else { throw CocoaError(.featureUnsupported) }
    return Timing(mean: simulated.reduce(0, +) / Double(simulated.count), worst: simulated.max()!, engine: nil,
                  check: Pile.check(bodies) + (simulated.count == steps ? "" : " (\(simulated.count) steps)"))
}

var args = Array(CommandLine.arguments.dropFirst())
@MainActor func take(_ name: String) -> String? {
    guard let i = args.firstIndex(of: name), i + 1 < args.count else { return nil }
    defer { args.removeSubrange(i...(i + 1)) }
    return args[i + 1]
}
let steps = take("--steps").flatMap(Int.init) ?? 300
let countsArg = take("--counts").map { $0.split(separator: ",").compactMap { Int($0) } }
let piles = args.isEmpty ? Pile.allCases : args.compactMap(Pile.init(rawValue:))
let cores: Int = { var n: Int32 = 0; var s = MemoryLayout<Int32>.size; sysctlbyname("hw.perflevel0.physicalcpu", &n, &s, nil, 0); return Int(n) }()

#if DEBUG
print("warning: debug build; use -c release")
#endif

await MainActor.run {
    print("\(steps) steps at 60 Hz · ms per step, mean / worst · \(ProcessInfo.processInfo.machineModel)")
    print("")
}
for pile in piles {
    let counts = countsArg ?? (pile == .tori ? [25, 100, 400] : [100, 400, 1600])
    for count in counts {
        let rows: [(String, Timing)] = await MainActor.run {
            var rows: [(String, Timing)] = []
            rows.append(("RealityBox3D, 1 worker", timeBox3D(pile, count: count, steps: steps, workers: 1)))
            rows.append(("RealityBox3D, \(cores) workers", timeBox3D(pile, count: count, steps: steps, workers: cores)))
            rows.append(("RealitySDFPhysics, multithreaded", timeSDF(pile, count: count, steps: steps)))
            return rows
        } + [("RealityKit", try await timeRealityKit(pile, count: count, steps: steps))]
        for (name, t) in rows {
            let engine = t.engine.map { String(format: "%7.2f", $0) } ?? "      –"
            print(String(format: "%-6@ %5d  %-31@ %7.2f / %7.2f   engine %@   %@", pile.rawValue as NSString, count, name as NSString,
                         t.mean, t.worst, engine as NSString, t.check as NSString))
        }
        print("")
    }
}

extension ProcessInfo {
    /// The machine's model identifier, such as MacBookPro17,1, rather than its personal host name.
    var machineModel: String {
        var size = 0
        sysctlbyname("hw.model", nil, &size, nil, 0)
        var model = [CChar](repeating: 0, count: size)
        sysctlbyname("hw.model", &model, &size, nil, 0)
        return String(cString: model)
    }
}

// Times Box3D's benchmark scenes two ways, with upstream's method (benchmark/main.c): skip the
// first step, time the rest at 60 Hz with 4 substeps, keep the fastest run.
//
//   raw:    Box3D's own C scene builders (shared/benchmarks.c) stepping a bare b3World
//   bridge: the same scenes rebuilt as RealityKit entities, stepped by RealityBox3D's B3World,
//           including reading entity transforms in and writing poses out every frame
//
//   swift run -c release b3-bench [large_pyramid|wide_pyramid|convex_pile ...] [--threads 1,4] [--runs 2] [--steps N]

import Box3DDemoScenes
import CBox3D
import CBox3DShared
import Foundation
import RealityBox3D
import RealityKit

struct Bench {
    let key: String
    let scene: String
    let steps: Int
    let create: (b3WorldId) -> Void
    let capacity: ((inout b3Capacity) -> Void)?
}

let benches: [Bench] = [
    Bench(key: "large_pyramid", scene: "Large Pyramid", steps: 200, create: { CreateLargePyramid($0) }, capacity: nil),
    Bench(key: "wide_pyramid", scene: "Wide Pyramid", steps: 300, create: { CreateWidePyramid($0) }, capacity: nil),
    Bench(key: "convex_pile", scene: "Convex Pile", steps: 500, create: { CreateConvexPile($0) },
          capacity: { GetConvexPileCapacity(&$0) }),
]

var args = Array(CommandLine.arguments.dropFirst())
@MainActor func option(_ name: String) -> String? {
    guard let i = args.firstIndex(of: name), i + 1 < args.count else { return nil }
    defer { args.removeSubrange(i...(i + 1)) }
    return args[i + 1]
}
let performanceCores = { var n: Int32 = 0; var size = MemoryLayout<Int32>.size; sysctlbyname("hw.perflevel0.physicalcpu", &n, &size, nil, 0); return Int(n) }()
let threads = option("--threads").map { $0.split(separator: ",").compactMap { Int($0) } } ?? [1, performanceCores]
let runs = option("--runs").flatMap(Int.init) ?? 2
let stepOverride = option("--steps").flatMap(Int.init)
args.removeAll { $0 == "--micro" }
let selected = args.isEmpty ? benches : benches.filter { args.contains($0.key) }

#if DEBUG
print("warning: debug build. Box3D's scenes shrink in debug; use swift run -c release b3-bench")
#endif

func ms(_ d: Duration) -> Double { Double(d.components.seconds) * 1e3 + Double(d.components.attoseconds) * 1e-15 }

func raw(_ b: Bench, threads: Int, steps: Int) -> Double {
    var def = b3DefaultWorldDef()
    def.workerCount = UInt32(threads)
    b.capacity?(&def.capacity)
    let world = b3CreateWorld(&def)
    defer { b3DestroyWorld(world) }
    b.create(world)
    b3World_Step(world, 1.0 / 60, 4)
    let clock = ContinuousClock(), t0 = clock.now
    for _ in 1..<steps { b3World_Step(world, 1.0 / 60, 4) }
    return ms(clock.now - t0)
}

struct BridgeTiming { var total = 0.0, step = 0.0, syncIn = 0.0, syncOut = 0.0, bodies = 0 }

@MainActor
func bridge(_ b: Bench, threads: Int, steps: Int) -> BridgeTiming {
    let runner = DemoRunner(DemoScene.named(b.scene)!) { s in
        s.workerCount = threads
        s.interpolatesPoses = false
    }
    runner.advance(frames: 1) // creates the bodies and takes the first step
    var t = BridgeTiming(bodies: runner.world.bodyCount)
    let clock = ContinuousClock(), t0 = clock.now
    for _ in 1..<steps {
        runner.advance(frames: 1, deltaTime: 1.0 / 60)
        let s = runner.world.statistics
        t.step += s.stepMilliseconds
        t.syncIn += s.syncInMilliseconds
        t.syncOut += s.syncOutMilliseconds
    }
    t.total = ms(clock.now - t0)
    return t
}

if !CommandLine.arguments.contains("--micro") { MainActor.assumeIsolated {
    print("RealityBox3D benchmark · \(performanceCores) performance cores · runs \(runs) · fastest kept")
    print("all times in ms for the whole run")
    print("")
    print("scene          thr  bodies   raw C   bridge  of which: box3d  sync-in  sync-out  overhead")
    for b in selected {
        let steps = stepOverride ?? b.steps
        for n in threads {
            let r = (0..<runs).map { _ in raw(b, threads: n, steps: steps) }.min()!
            let br = (0..<runs).map { _ in bridge(b, threads: n, steps: steps) }.min { $0.total < $1.total }!
            let overhead = (br.total - r) / r * 100
            print(String(format: "%-14@ %3d  %6d  %7.1f  %7.1f         %7.1f  %7.1f  %8.1f   %+6.1f%%",
                         b.key as NSString, n, br.bodies, r, br.total, br.step, br.syncIn, br.syncOut, overhead))
        }
    }
    print("")
    print("M2 Air reference (upstream benchmark/m2air_neon): large_pyramid 1 thread 1810.7 ms, 4 threads 563.5 ms;")
    print("convex_pile 1 thread 15032.6 ms, 4 threads 4296.5 ms")
} }

// MARK: Micro: the per-entity cost of RealityKit calls the bridge makes (--micro)
@MainActor func micro() {
    let n = 5000, reps = 50
    let root = Entity()
    let es = (0..<n).map { i -> Entity in
        let e = Entity(); e.position = [Float(i), 0, 0]
        e.components.set(B3PhysicsBodyComponent(density: 1))
        e.components.set(B3CollisionComponent(shapes: [.generateBox(size: [1, 1, 1])]))
        root.addChild(e); return e
    }
    func time(_ label: String, _ body: () -> Void) {
        let c = ContinuousClock(), t0 = c.now
        for _ in 0..<reps { body() }
        print(String(format: "%-34@ %6.3f µs/entity", label as NSString, ms(c.now - t0) * 1000 / Double(n * reps)))
    }
    var sink: Float = 0
    time("transform read") { for e in es { sink += e.transform.translation.x } }
    time("position(relativeTo: root)") { for e in es { sink += e.position(relativeTo: root).x } }
    time("components[Body] read") { for e in es { sink += e.components[B3PhysicsBodyComponent.self]!.linearDamping } }
    time("components[Collision] read") { for e in es { sink += Float(e.components[B3CollisionComponent.self]!.shapes.count) } }
    time("components.has(Motion)") { for e in es { sink += e.components.has(B3PhysicsMotionComponent.self) ? 1 : 0 } }
    time("components[Motion] read (absent)") { for e in es { sink += e.components[B3PhysicsMotionComponent.self] == nil ? 1 : 0 } }
    let b = es[0].components[B3PhysicsBodyComponent.self]!, cc = es[0].components[B3CollisionComponent.self]!
    time("Body == Body") { for e in es { sink += e.components[B3PhysicsBodyComponent.self]! == b ? 1 : 0 } }
    time("Collision == Collision") { for e in es { sink += e.components[B3CollisionComponent.self]! == cc ? 1 : 0 } }
    time("transform write") { for e in es { e.transform = Transform(translation: [sink, 0, 0]) } }
    time("parent === root") { for e in es { sink += e.parent === root ? 1 : 0 } }
    print(sink > 0 ? "" : " ")
}
if CommandLine.arguments.contains("--micro") { MainActor.assumeIsolated { micro() } }

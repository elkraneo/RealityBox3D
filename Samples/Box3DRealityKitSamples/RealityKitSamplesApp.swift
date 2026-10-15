// Apple's RealityKit physics samples, ported from RealityKit's physics to Box3D through RealityBox3D.
// Each sample keeps the structure and values of Apple's, and says where and why it differs.

import RealityBox3D
import RealityKit
import Box3DRealityKitSampleScenes
import SwiftUI

@main
struct RealityKitSamplesApp: App {
    init() {
        // Both samples simulate in centimeters, under roots scaled by 0.01, so Apple's spheres
        // (6 to 30 cm) and dice (2 cm) are a few units across, the size Box3D's tolerances suit.
        RealityBox3D.register()
        AttractionComponent.registerComponent()
        AttractionSystem.registerSystem()
    }

    var body: some SwiftUI.Scene {
        WindowGroup("RealityKit Samples on Box3D") {
            SamplesView()
        }
        .defaultSize(width: 1200, height: 800)
    }
}

private enum Sample: String, CaseIterable, Identifiable {
    case physicsBodies = "Physics Bodies"
    case dice = "Dice"
    case newtonsCradle = "Newton's Cradle"

    var id: Self { self }

    /// How this port differs from Apple's sample
    var differences: [String] {
        switch self {
        case .physicsBodies: [
            "B3PhysicsBodyComponent and B3CollisionComponent in place of RealityKit's; the same sizes, masses and forces.",
            "Shape swaps the spheres for tori, as in RealitySDFPhysics's port. Box3D is convex-only, so each torus is a ring of 12 capsules.",
            "A fixed 1 m volume on macOS. Dragging picks with Box3D's raycast and pulls on every fixed step.",
        ]
        case .dice: [
            "Box3D convex hulls of Apple's dice models, in place of TabletopKit, which is visionOS only.",
            "The roll is forecast from an exact copy of the Box3D world, restored from a recording snapshot; Max score toss turns each die by a symmetry to land on its highest face, without adjusting the roll.",
            "Invisible walls keep the dice on the table; the pointer holds a die with a spring, and releases it with its velocity.",
        ]
        case .newtonsCradle: [
            "B3PhysicsRevoluteJoint on Apple's pins, sizes, masses and slow motion, in a root whose units are centimeters.",
            "sequentialImpacts resolves the impacts pair by pair before each step: one ball in sends one out, as in a real cradle. Box3D's solver alone, like RealityKit's, shares the momentum along the row.",
            "Lift raises one to three balls to 30° and releases them; Push ball is Apple's impulse. Drag a ball by hand: a motor joint with a force-limited spring pulls it to the pointer.",
        ]
        }
    }

    /// Apple's sample this one ports
    var source: (title: String, url: URL) {
        switch self {
        case .physicsBodies:
            ("Simulating physics with collisions in your visionOS app",
             URL(string: "https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app")!)
        case .dice:
            ("Simulating dice rolls as a component for your game",
             URL(string: "https://developer.apple.com/documentation/tabletopkit/simulating-dice-rolls-as-a-component-for-your-game")!)
        case .newtonsCradle:
            ("Simulating physics joints in your RealityKit app",
             URL(string: "https://developer.apple.com/documentation/realitykit/simulating-physics-joints-in-your-realitykit-app")!)
        }
    }
}

private struct SamplesView: View {
    @State private var selection: Sample? = .physicsBodies

    var body: some View {
        NavigationSplitView {
            List(Sample.allCases, selection: $selection) { sample in
                VStack(alignment: .leading) {
                    Text(sample.rawValue)
                    Text(sample.source.title).font(.caption).foregroundStyle(.secondary)
                }
            }
            .navigationSplitViewColumnWidth(min: 220, ideal: 260)
        } detail: {
            if let selection {
                VStack(spacing: 0) {
                    switch selection {
                    case .physicsBodies: PhysicsBodiesView()
                    case .dice: DiceView()
                    case .newtonsCradle: NewtonsCradleView()
                    }
                    Divider()
                    SampleReference(sample: selection)
                }
            } else {
                Text("Select a sample")
            }
        }
    }
}

/// A bar naming the Apple sample a port reimplements, with a link to it and its differences
private struct SampleReference: View {
    let sample: Sample
    @State private var showsDifferences = false

    var body: some View {
        HStack(spacing: 12) {
            Text("Port of Apple's sample").foregroundStyle(.secondary)
            Text(sample.source.title).fontWeight(.semibold)
            Spacer()
            Button("Differences", systemImage: "arrow.triangle.branch") { showsDifferences.toggle() }
                .popover(isPresented: $showsDifferences, arrowEdge: .top) {
                    VStack(alignment: .leading, spacing: 8) {
                        ForEach(sample.differences, id: \.self) { Label($0, systemImage: "arrow.triangle.branch") }
                    }
                    .padding()
                    .frame(width: 420)
                }
            Link(destination: sample.source.url) {
                Label("Open Apple's Sample", systemImage: "arrow.up.right.square")
            }
            .buttonStyle(.borderedProminent)
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 10)
    }
}

// Port of MainView from Apple's sample "Simulating physics joints in your RealityKit app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-joints-in-your-realitykit-app
//
// Apple's "Push ball" button, plus Lift buttons that raise the first one, two or three balls.

import RealityKit
import Box3DRealityKitSampleScenes
import SwiftUI

struct NewtonsCradleView: View {
    @State private var scene = NewtonsCradleScene()

    var body: some View {
        RealityView { content in
            content.add(scene.content)
        }
        .toolbar {
            ForEach(1...3, id: \.self) { count in
                Button("Lift \(count)") { scene.lift(count) }
            }
            Button("Push ball", systemImage: "hand.point.right") { scene.push() }
            Button("Reset", systemImage: "arrow.counterclockwise") { scene.lift(0) }
        }
    }
}

// Port of MainView from Apple's sample "Simulating physics joints in your RealityKit app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-joints-in-your-realitykit-app
//
// Apple's "Push ball" button, plus Lift buttons that raise the first one, two or three balls, and
// a ball drag, picked with Box3D's raycast.

import RealityKit
import Box3DRealityKitSampleScenes
import SwiftUI

struct NewtonsCradleView: View {
    @State private var scene = NewtonsCradleScene()
    @State private var isDragging = false

    var body: some View {
        GeometryReader { geometry in
            RealityView { content in
                content.add(scene.content)
            }
            .gesture(
                DragGesture(minimumDistance: 0)
                    .onChanged { value in
                        if !isDragging { isDragging = scene.beginDrag(at: value.startLocation, in: geometry.size) }
                        if isDragging { scene.drag(to: value.location, in: geometry.size) }
                    }
                    .onEnded { _ in
                        scene.endDrag()
                        isDragging = false
                    }
            )
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

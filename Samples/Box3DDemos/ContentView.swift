import Box3DDemoScenes
import RealityBox3D
import RealityKit
import SwiftUI

struct ContentView: View {
    @State private var model = DemoModel()

    var body: some View {
        NavigationSplitView {
            Sidebar(model: model)
                .navigationSplitViewColumnWidth(min: 280, ideal: 320)
        } detail: {
            ZStack(alignment: .topTrailing) {
                Viewport(model: model)
                StatsOverlay(model: model)
                    .padding(12)
            }
        }
        .background(WindowReader { model.window = $0 })
        .onDisappear { model.removeKeyMonitor() }
        .task {
            model.installKeyMonitor()
            // `Box3DDemos --scene "Geometric Mover"` opens on that scene
            let args = CommandLine.arguments
            let named = args.firstIndex(of: "--scene").flatMap { $0 + 1 < args.count ? DemoScene.named(args[$0 + 1]) : nil }
            model.load((named ?? DemoScene.all[0]).id)
        }
    }
}

// MARK: Sidebar

private struct Sidebar: View {
    @Bindable var model: DemoModel

    var body: some View {
        Form {
            Section("Scene") {
                Picker("Scene", selection: Binding(get: { model.scene.id }, set: { model.load($0) })) {
                    ForEach(model.categories, id: \.name) { category in
                        Section(category.name) {
                            ForEach(category.scenes) { scene in
                                Text(scene.name).tag(scene.id)
                            }
                        }
                    }
                }
                Text(model.scene.summary)
                    .font(.callout)
                    .foregroundStyle(.secondary)
                HStack {
                    Button(model.isPaused ? "Play" : "Pause") { model.isPaused.toggle() }
                        .keyboardShortcut("p", modifiers: [])
                    Button("Step") { model.step() }
                        .disabled(!model.isPaused)
                    Button("Reset") { model.reset() }
                        .keyboardShortcut("r", modifiers: [])
                }
                if !model.scene.actions.isEmpty {
                    HStack {
                        ForEach(model.scene.actions, id: \.self) { action in
                            Button(action) { model.perform(action) }
                        }
                    }
                }
                if let controls = model.scene.controls {
                    Label(controls, systemImage: "keyboard")
                        .font(.callout)
                }
            }

            Section("Simulation") {
                Stepper("Substeps: \(model.settings.subStepCount)", value: $model.settings.subStepCount, in: 1...16)
                Stepper("Workers: \(model.settings.workerCount)", value: $model.settings.workerCount, in: 1...16)
                Toggle("Sleeping", isOn: $model.settings.isSleepEnabled)
                Toggle("Continuous collision", isOn: $model.settings.isContinuousEnabled)
                Toggle("Interpolate poses", isOn: $model.settings.interpolatesPoses)
            }

            Section {
                LabeledContent("Upstream", value: model.scene.upstream)
                    .font(.caption)
                Text("Drag a body to pull it, drag the background to orbit, pinch or scroll to zoom.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
        .formStyle(.grouped)
    }
}

// MARK: Stats

private struct StatsOverlay: View {
    let model: DemoModel

    var body: some View {
        // The world is not observable; sample its statistics a few times per second.
        TimelineView(.periodic(from: .now, by: 0.25)) { _ in
            let s = model.statistics
            VStack(alignment: .leading, spacing: 2) {
                Text("\(model.scene.category) · \(model.scene.name)").font(.headline.monospaced())
                Text("\(s.bodies) bodies · \(s.awakeBodies) awake · \(s.steps) steps this frame")
                Text("Box3D \(s.stepMilliseconds, format: .number.precision(.fractionLength(2))) ms · sync in \(s.syncInMilliseconds, format: .number.precision(.fractionLength(2))) ms · out \(s.syncOutMilliseconds, format: .number.precision(.fractionLength(2))) ms")
            }
            .font(.caption.monospaced())
            .padding(8)
            .background(.regularMaterial, in: .rect(cornerRadius: 8))
        }
    }
}

// MARK: Viewport

private struct Viewport: View {
    let model: DemoModel

    private enum Interaction {
        case body(distance: Float)
        case orbit(last: CGPoint)
    }

    @State private var interaction: Interaction?
    @State private var lastMagnification: CGFloat = 1

    var body: some View {
        GeometryReader { geometry in
            ZStack {
                RealityView { content in
                    content.add(model.root)
                    content.add(model.lights)
                    content.add(model.cameraEntity)
                    model.placeCamera()
                    model.frameSubscription = content.subscribe(to: SceneEvents.Update.self) { _ in model.followCamera() }
                }
                Color.clear
                    .contentShape(.rect)
                    .gesture(dragGesture(in: geometry.size))
                    .simultaneousGesture(magnifyGesture)
                    .onScrollWheel { delta in model.zoom(by: 1 + Float(delta) * 0.01) }
            }
        }
    }

    private func dragGesture(in size: CGSize) -> some Gesture {
        DragGesture(minimumDistance: 0)
            .onChanged { value in
                if interaction == nil {
                    if let distance = model.beginDrag(at: value.startLocation, in: size) {
                        interaction = .body(distance: distance)
                    } else {
                        interaction = .orbit(last: value.startLocation)
                    }
                }
                switch interaction {
                case .body(let distance)?:
                    model.drag(to: value.location, in: size, distance: distance)
                case .orbit(let last)?:
                    model.orbit(by: CGSize(width: value.location.x - last.x, height: value.location.y - last.y))
                    interaction = .orbit(last: value.location)
                case nil:
                    break
                }
            }
            .onEnded { _ in
                if case .body = interaction { model.endDrag() }
                interaction = nil
            }
    }

    private var magnifyGesture: some Gesture {
        MagnifyGesture()
            .onChanged { value in
                model.zoom(by: Float(value.magnification / lastMagnification))
                lastMagnification = value.magnification
            }
            .onEnded { _ in lastMagnification = 1 }
    }
}

private extension View {
    /// Scroll-wheel zoom on macOS.
    func onScrollWheel(_ action: @escaping (CGFloat) -> Void) -> some View {
        background(ScrollWheelCatcher(action: action))
    }
}

/// Hands over the window the view is in, once it is in one.
private struct WindowReader: NSViewRepresentable {
    let found: (NSWindow?) -> Void
    func makeNSView(context: Context) -> NSView {
        let v = ReaderView(); v.found = found; return v
    }
    func updateNSView(_ nsView: NSView, context: Context) {}
    final class ReaderView: NSView {
        var found: ((NSWindow?) -> Void)?
        override func viewDidMoveToWindow() { found?(window) }
    }
}

private struct ScrollWheelCatcher: NSViewRepresentable {
    let action: (CGFloat) -> Void
    func makeNSView(context: Context) -> NSView {
        let v = WheelView(); v.action = action; return v
    }
    func updateNSView(_ nsView: NSView, context: Context) { (nsView as? WheelView)?.action = action }
    final class WheelView: NSView {
        var action: ((CGFloat) -> Void)?
        private var monitor: Any?
        override func viewDidMoveToWindow() {
            if let monitor { NSEvent.removeMonitor(monitor) }
            monitor = NSEvent.addLocalMonitorForEvents(matching: .scrollWheel) { [weak self] event in
                guard let self, event.window === self.window,
                      self.bounds.contains(self.convert(event.locationInWindow, from: nil)) else { return event }
                self.action?(event.scrollingDeltaY)
                return event
            }
        }
    }
}

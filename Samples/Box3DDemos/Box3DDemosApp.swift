import AppKit
import Box3DDemoScenes
import RealityBox3D
import SwiftUI

@main
struct Box3DDemosApp: App {
    @NSApplicationDelegateAdaptor private var appDelegate: AppDelegate

    init() {
        RealityBox3D.register()
        DemoScriptSystem.register()
        DragSystem.registerSystem()
    }

    var body: some Scene {
        WindowGroup("Box3D Demos on RealityKit") {
            ContentView()
                .frame(minWidth: 960, minHeight: 600)
        }
    }
}

/// Run from `swift run`, the executable is not an app bundle, so it asks to be a regular,
/// frontmost app to get a Dock icon and keyboard focus.
final class AppDelegate: NSObject, NSApplicationDelegate {
    func applicationDidFinishLaunching(_ notification: Notification) {
        NSApplication.shared.setActivationPolicy(.regular)
        NSApplication.shared.activate()
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }
}

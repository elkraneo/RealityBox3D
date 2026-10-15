// swift-tools-version: 6.2
import PackageDescription

let package = Package(
    name: "RealityBox3D",
    platforms: [.macOS(.v15), .iOS(.v18), .visionOS(.v2)],
    products: [
        .library(name: "RealityBox3D", targets: ["RealityBox3D"]),
        // The ported scenes, for apps on other platforms such as a visionOS demo
        .library(name: "Box3DDemoScenes", targets: ["Box3DDemoScenes"]),
        .library(name: "Box3DRealityKitSampleScenes", targets: ["Box3DRealityKitSampleScenes"]),
        .executable(name: "Box3DDemos", targets: ["Box3DDemos"]),
        .executable(name: "Box3DRealityKitSamples", targets: ["Box3DRealityKitSamples"]),
        .executable(name: "b3-bench", targets: ["b3-bench"]),
        .executable(name: "b3-snapshot", targets: ["b3-snapshot"]),
    ],
    targets: [
        // Box3D, vendored unmodified (see UPSTREAM.md)
        .target(
            name: "CBox3D",
            path: "Sources/CBox3D",
            exclude: ["LICENSE.box3d", "src/contact_solver_wide.inl", "src/convex_manifold_wide.inl", "src/recording_ops.inl"],
            sources: ["src"],
            publicHeadersPath: "include",
            cSettings: [
                .headerSearchPath("src"),
                // Box3D's asserts and debug-sized benchmark scenes key off NDEBUG; SwiftPM does not set it.
                .define("NDEBUG", .when(configuration: .release)),
            ]
        ),
        // The library: shapes, components and the RealityKit system, shaped like RealityKit's physics
        .target(
            name: "RealityBox3D",
            dependencies: ["CBox3D"]
        ),
        // Box3D's shared benchmark scenes in C, vendored unmodified: the raw baseline for b3-bench
        .target(
            name: "CBox3DShared",
            dependencies: ["CBox3D"],
            path: "Sources/CBox3DShared",
            exclude: ["m2air_neon-reference"],
            cSettings: [.define("NDEBUG", .when(configuration: .release))]
        ),
        // Box3D's sample scenes rebuilt on the public API, shared by the app, tools and tests
        .target(
            name: "Box3DDemoScenes",
            dependencies: ["RealityBox3D", "CBox3D"],
            path: "Samples/Box3DDemoScenes"
        ),
        .executableTarget(
            name: "Box3DDemos",
            dependencies: ["Box3DDemoScenes"],
            path: "Samples/Box3DDemos"
        ),
        // Apple's RealityKit physics samples, ported to Box3D bodies
        .target(
            name: "Box3DRealityKitSampleScenes",
            dependencies: ["RealityBox3D"],
            path: "Samples/Box3DRealityKitSampleScenes",
            exclude: ["Dice/Models/LICENSE.txt"],
            resources: [.copy("Dice/Models")]
        ),
        .executableTarget(
            name: "Box3DRealityKitSamples",
            dependencies: ["Box3DRealityKitSampleScenes"],
            path: "Samples/Box3DRealityKitSamples"
        ),
        // Box3D's benchmark scenes timed in raw C and through RealityBox3D
        .executableTarget(
            name: "b3-bench",
            dependencies: ["CBox3DShared", "Box3DDemoScenes"]
        ),
        // Offscreen RealityKit renders of the demo scenes
        .executableTarget(
            name: "b3-snapshot",
            dependencies: ["Box3DDemoScenes", "Box3DRealityKitSampleScenes"]
        ),
        .testTarget(name: "CBox3DTests", dependencies: ["CBox3D"]),
        .testTarget(name: "RealityBox3DTests", dependencies: ["RealityBox3D", "CBox3D", "Box3DDemoScenes", "Box3DRealityKitSampleScenes"]),
    ],
    cLanguageStandard: .c17
)

// swift-tools-version: 6.2
// Times the same scenes on RealityKit's physics, RealitySDFPhysics and RealityBox3D (Box3D).
// RealitySDFPhysics is pinned to an exact release, so its later changes never move these numbers. Build products are large; consider --scratch-path on a roomy disk.
import PackageDescription

let package = Package(
    name: "EngineComparison",
    platforms: [.macOS(.v15)],
    dependencies: [
        .package(path: "../.."),
        .package(url: "https://github.com/elkraneo/RealitySDFPhysics", exact: "0.2.0"),
    ],
    targets: [
        .executableTarget(
            name: "engine-comparison",
            dependencies: [
                .product(name: "RealityBox3D", package: "RealityBox3D"),
                .product(name: "RealitySDFPhysics", package: "RealitySDFPhysics"),
            ]
        ),
    ]
)

// swift-tools-version: 6.2
// Times the same scenes on RealityKit's physics, SDFCollisionKit and RealityBox3D (Box3D).
// SDFCollisionKit, now RealitySDFPhysics, is pinned to a commit from before its rename, so its
// later changes never move these numbers. Build products are large; consider --scratch-path on a roomy disk.
import PackageDescription

let package = Package(
    name: "EngineComparison",
    platforms: [.macOS(.v15)],
    dependencies: [
        .package(path: "../.."),
        .package(url: "https://github.com/elkraneo/SDFCollisionKit", revision: "30ddbb90b1932a000f17af9cf22c4cb8f9672937"),
    ],
    targets: [
        .executableTarget(
            name: "engine-comparison",
            dependencies: [
                .product(name: "RealityBox3D", package: "RealityBox3D"),
                .product(name: "SDFCollisionKit", package: "SDFCollisionKit"),
            ]
        ),
    ]
)

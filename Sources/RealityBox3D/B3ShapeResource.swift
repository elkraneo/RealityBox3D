import RealityKit
import simd

/// Collision geometry, shaped like RealityKit's `ShapeResource`.
///
/// Unlike `ShapeResource`, this is a plain value: the geometry is readable, comparable
/// and can be built off the main actor. Box3D bodies are created from it lazily.
public struct B3ShapeResource: Equatable, Sendable {
    public enum Geometry: Equatable, Sendable {
        case box(size: SIMD3<Float>)
        case sphere(radius: Float)
        /// Y-aligned capsule; `height` is the total height including both caps, as in RealityKit.
        case capsule(height: Float, radius: Float)
        case convex(points: [SIMD3<Float>])
        /// Static-only triangle mesh. Box3D only generates mesh contacts against non-static bodies
        /// when the mesh itself is on a static body.
        case staticMesh(positions: [SIMD3<Float>], indices: [UInt32])
        /// Static-only terrain: `countX` × `countZ` grid heights, row by row along z, starting at
        /// the origin and spaced by `scale.x` and `scale.z`; heights are multiplied by `scale.y`.
        case heightField(heights: [Float], countX: Int, countZ: Int, scale: SIMD3<Float>)
    }

    public var geometry: Geometry
    public var translation: SIMD3<Float> = .zero
    public var rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    /// Box3D extension: this shape's surface, in place of the body's material.
    public var material: B3PhysicsMaterial?
    /// Box3D extension: a surface per triangle of a static mesh. Ignored by other shapes.
    public var triangleMaterials: B3TriangleMaterials?

    public init(geometry: Geometry) { self.geometry = geometry }

    // MARK: RealityKit-shaped factories

    public static func generateBox(size: SIMD3<Float>) -> B3ShapeResource { .init(geometry: .box(size: size)) }
    public static func generateBox(width: Float, height: Float, depth: Float) -> B3ShapeResource {
        generateBox(size: [width, height, depth])
    }
    public static func generateSphere(radius: Float) -> B3ShapeResource { .init(geometry: .sphere(radius: radius)) }
    public static func generateCapsule(height: Float, radius: Float) -> B3ShapeResource {
        .init(geometry: .capsule(height: height, radius: radius))
    }
    public static func generateConvex(from points: [SIMD3<Float>]) -> B3ShapeResource {
        .init(geometry: .convex(points: points))
    }
    public static func generateStaticMesh(positions: [SIMD3<Float>], faceIndices: [UInt16]) -> B3ShapeResource {
        .init(geometry: .staticMesh(positions: positions, indices: faceIndices.map(UInt32.init)))
    }
    public static func generateStaticMesh(positions: [SIMD3<Float>], faceIndices: [UInt32]) -> B3ShapeResource {
        .init(geometry: .staticMesh(positions: positions, indices: faceIndices))
    }

    /// Box3D extension: terrain from a grid of heights. See `Geometry.heightField`.
    /// Box3D extension: a static mesh with a surface per triangle, such as ice and grass in one
    /// terrain mesh.
    public static func generateStaticMesh(positions: [SIMD3<Float>], faceIndices: [UInt32],
                                          triangleMaterials: B3TriangleMaterials) -> B3ShapeResource {
        var shape = generateStaticMesh(positions: positions, faceIndices: faceIndices)
        shape.triangleMaterials = triangleMaterials
        return shape
    }

    /// Box3D extension: the same shape with its own surface, in place of the body's material.
    public func withMaterial(_ material: B3PhysicsMaterial) -> B3ShapeResource {
        var copy = self
        copy.material = material
        return copy
    }

    public static func generateHeightField(heights: [Float], countX: Int, countZ: Int, scale: SIMD3<Float> = .one) -> B3ShapeResource {
        .init(geometry: .heightField(heights: heights, countX: countX, countZ: countZ, scale: scale))
    }

    @MainActor
    public static func generateConvex(from mesh: MeshResource) -> B3ShapeResource {
        generateConvex(from: mesh.b3Triangles().positions)
    }

    /// The convex hull of every mesh under an entity, in the entity's own space, as Box3D
    /// simplifies it (at most 64 vertices). For convex models such as dice.
    @MainActor
    public static func generateConvex(from entity: Entity) -> B3ShapeResource {
        var points: [SIMD3<Float>] = []
        func visit(_ e: Entity) {
            if let mesh = e.components[ModelComponent.self]?.mesh {
                let m = e.transformMatrix(relativeTo: entity)
                for p in mesh.b3Triangles().positions {
                    let w = m * SIMD4(p, 1)
                    points.append(SIMD3(w.x, w.y, w.z))
                }
            }
            for c in e.children { visit(c) }
        }
        visit(entity)
        return generateConvex(from: points)
    }

    @MainActor
    public static func generateStaticMesh(from mesh: MeshResource) -> B3ShapeResource {
        let t = mesh.b3Triangles()
        return generateStaticMesh(positions: t.positions, faceIndices: t.indices)
    }

    public func offsetBy(rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
                         translation: SIMD3<Float> = .zero) -> B3ShapeResource {
        var copy = self
        copy.rotation = rotation * self.rotation
        copy.translation = rotation.act(self.translation) + translation
        return copy
    }
    public func offsetBy(translation: SIMD3<Float>) -> B3ShapeResource { offsetBy(rotation: .init(ix: 0, iy: 0, iz: 0, r: 1), translation: translation) }
    public func offsetBy(rotation: simd_quatf) -> B3ShapeResource { offsetBy(rotation: rotation, translation: .zero) }
}

extension MeshResource {
    /// Flattens every part of every instance into one triangle soup in mesh space.
    @MainActor
    func b3Triangles() -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        var positions: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        let contents = self.contents
        for instance in contents.instances {
            guard let model = contents.models[instance.model] else { continue }
            let m = instance.transform
            for part in model.parts {
                let base = UInt32(positions.count)
                for p in part.positions {
                    let w = m * SIMD4(p, 1)
                    positions.append(SIMD3(w.x, w.y, w.z))
                }
                if let tri = part.triangleIndices {
                    indices.append(contentsOf: tri.map { $0 + base })
                }
            }
        }
        return (positions, indices)
    }
}

/// Box3D extension: the surfaces of a static mesh's triangles. `indices` holds one entry per
/// triangle, an index into `materials`, which holds at most 256.
public struct B3TriangleMaterials: Equatable, Sendable {
    public var materials: [B3PhysicsMaterial]
    public var indices: [UInt8]
    public init(materials: [B3PhysicsMaterial], indices: [UInt8]) { self.materials = materials; self.indices = indices }
}

import CBox3D
import RealityKit
import simd

// Box3D's own hull generators, and meshes that show exactly what Box3D simulates.
extension B3ShapeResource {
    /// A tessellated cylinder along Y, centered like RealityKit's `generateCylinder`. Box3D's own
    /// `b3CreateCylinder` starts at the origin; use `offsetBy(translation: [0, height / 2, 0])` for
    /// that. Box3D allows 3 to 32 sides.
    public static func generateCylinder(height: Float, radius: Float, sides: Int = 16) -> B3ShapeResource {
        fromBox3DHull(b3CreateCylinder(height, radius, -height / 2, Int32(min(max(sides, 3), 32))))
    }

    /// A tessellated cone along Y, centered like RealityKit's `generateCone`. Box3D allows 3 to 32
    /// slices and needs a top radius above zero, so a pointed cone keeps a tip 0.1% of its base.
    public static func generateCone(height: Float, bottomRadius: Float, topRadius: Float = 0, slices: Int = 16) -> B3ShapeResource {
        fromBox3DHull(b3CreateCone(height, bottomRadius, max(topRadius, bottomRadius * 1e-3), Int32(min(max(slices, 3), 32))))
            .offsetBy(translation: [0, -height / 2, 0]).baked()
    }

    /// Box3D's rock-shaped hull.
    public static func generateRock(radius: Float) -> B3ShapeResource {
        fromBox3DHull(b3CreateRock(radius))
    }

    /// The hull of 32 pseudo-random points on a sphere, used by Box3D's Convex Pile benchmark.
    public static func generateComplexHull(radius: Float) -> B3ShapeResource {
        fromBox3DHull(b3CreateComplexHull(radius))
    }

    /// Folds the offset into the points, so the shape's geometry is self-contained.
    func baked() -> B3ShapeResource {
        guard case .convex(let points) = geometry else { return self }
        return .generateConvex(from: points.map { rotation.act($0) + translation })
    }

    private static func fromBox3DHull(_ hull: UnsafeMutablePointer<b3HullData>?) -> B3ShapeResource {
        guard let hull else { return .generateSphere(radius: 0) }
        defer { b3DestroyHull(hull) }
        let points = b3GetHullPoints(hull)!
        return .generateConvex(from: (0..<Int(hull.pointee.vertexCount)).map { points[$0].simd })
    }
}

extension B3ShapeResource {
    /// A height field's grid as triangles, in the field's local space. Empty for invalid grids.
    static func heightFieldTriangles(heights: [Float], countX: Int, countZ: Int, scale: SIMD3<Float>) -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        let product = countX.multipliedReportingOverflow(by: countZ)
        guard countX >= 2, countZ >= 2, !product.overflow, heights.count == product.partialValue,
              product.partialValue <= Int(UInt32.max) else { return ([], []) }
        var positions: [SIMD3<Float>] = []
        positions.reserveCapacity(heights.count)
        for z in 0..<countZ {
            for x in 0..<countX {
                positions.append(SIMD3(Float(x), heights[z * countX + x], Float(z)) * scale)
            }
        }
        var indices: [UInt32] = []
        for z in 0..<(countZ - 1) {
            for x in 0..<(countX - 1) {
                let a = UInt32(z * countX + x), b = a + 1, c = a + UInt32(countX), d = c + 1
                indices += [a, c, b, b, c, d]
            }
        }
        return (positions, indices)
    }
}

extension MeshResource {
    /// A flat-shaded mesh of the shape as Box3D builds it: boxes and hulls face by face from
    /// Box3D's hull, so simplified hulls render as simulated.
    @MainActor
    public static func generate(from shape: B3ShapeResource) throws -> MeshResource {
        switch shape.geometry {
        case .box(let size):
            return try offset(.generateBox(size: size), by: shape)
        case .sphere(let radius):
            return try offset(.generateSphere(radius: radius), by: shape)
        case .capsule(let height, let radius):
            return try capsuleMesh(height: height, radius: radius, shape: shape)
        case .convex(let points):
            var pts = BodyRecord.weld(points).map(b3Vec3.init)
            var hull: UnsafeMutablePointer<b3HullData>?
            for maxVertices: Int32 in [64, 48, 32, 16] where hull == nil {
                hull = b3CreateHull(&pts, Int32(pts.count), maxVertices)
            }
            guard let hull else { return .generateSphere(radius: 0.01) }
            defer { b3DestroyHull(hull) }
            return try hullMesh(hull, shape: shape)
        case .heightField(let heights, let countX, let countZ, let scale):
            let grid = B3ShapeResource.heightFieldTriangles(heights: heights, countX: countX, countZ: countZ, scale: scale)
            return try .generate(from: [creased(grid.positions.map { shape.rotation.act($0) + shape.translation }, grid.indices)])
        case .staticMesh(let positions, let indices):
            return try .generate(from: [creased(positions.map { shape.rotation.act($0) + shape.translation }, indices)])
        }
    }

    /// Normals that keep hard edges: each corner averages the faces around its vertex that turn
    /// less than `crease` from its own face. Without normals RealityKit smooths across every
    /// shared vertex, smearing the shading of stairs and boxes built as meshes.
    static func creased(_ positions: [SIMD3<Float>], _ indices: [UInt32], crease degrees: Float = 30) -> MeshDescriptor {
        let triangles = indices.count / 3
        var faceNormals: [SIMD3<Float>] = [], unit: [SIMD3<Float>] = []
        var around = [[Int]](repeating: [], count: positions.count)
        faceNormals.reserveCapacity(triangles)
        for t in 0..<triangles {
            let a = positions[Int(indices[3 * t])], b = positions[Int(indices[3 * t + 1])], c = positions[Int(indices[3 * t + 2])]
            let n = simd_cross(b - a, c - a) // area weighted
            faceNormals.append(n)
            let l = simd_length(n)
            unit.append(l > 0 ? n / l : .zero)
            for k in 0..<3 { around[Int(indices[3 * t + k])].append(t) }
        }
        let cosCrease = cos(degrees * .pi / 180)
        var outPositions: [SIMD3<Float>] = [], outNormals: [SIMD3<Float>] = []
        outPositions.reserveCapacity(indices.count)
        outNormals.reserveCapacity(indices.count)
        for t in 0..<triangles {
            for k in 0..<3 {
                let v = Int(indices[3 * t + k])
                var n = SIMD3<Float>.zero
                for f in around[v] where simd_dot(unit[f], unit[t]) >= cosCrease { n += faceNormals[f] }
                let l = simd_length(n)
                outPositions.append(positions[v])
                outNormals.append(l > 0 ? n / l : unit[t])
            }
        }
        var d = MeshDescriptor()
        d.positions = MeshBuffer(outPositions)
        d.normals = MeshBuffer(outNormals)
        d.primitives = .triangles((0..<UInt32(outPositions.count)).map { $0 })
        return d
    }

    @MainActor
    private static func offset(_ mesh: MeshResource, by shape: B3ShapeResource) throws -> MeshResource {
        guard shape.translation != .zero || abs(shape.rotation.real) < 1 - 1e-6 else { return mesh }
        var contents = mesh.contents
        contents.instances = .init(contents.instances.map { instance in
            var i = instance
            i.transform = Transform(rotation: shape.rotation, translation: shape.translation).matrix * i.transform
            return i
        })
        return try .generate(from: contents)
    }

    @MainActor
    private static func capsuleMesh(height: Float, radius: Float, shape: B3ShapeResource) throws -> MeshResource {
        // A capsule is the hull of two spheres: tessellate it as one closed surface.
        let rings = 12, segments = 24
        let half = max(height * 0.5 - radius, 0)
        var positions: [SIMD3<Float>] = []
        var normals: [SIMD3<Float>] = []
        for ring in 0...(2 * rings + 1) {
            // Upper hemisphere rings 0...rings, lower rings+1...2*rings+1
            let upper = ring <= rings
            let phi = Float.pi / 2 * (upper ? 1 - Float(ring) / Float(rings) : -Float(ring - rings - 1) / Float(rings))
            let y = sin(phi), r = cos(phi)
            for s in 0...segments {
                let theta = 2 * Float.pi * Float(s) / Float(segments)
                let n = SIMD3(r * cos(theta), y, r * sin(theta))
                positions.append(n * radius + SIMD3(0, upper ? half : -half, 0))
                normals.append(n)
            }
        }
        var indices: [UInt32] = []
        let stride = UInt32(segments + 1)
        for ring in 0..<UInt32(2 * rings + 1) {
            for s in 0..<UInt32(segments) {
                let a = ring * stride + s, b = a + stride
                indices += [a, a + 1, b, a + 1, b + 1, b]
            }
        }
        var d = MeshDescriptor()
        d.positions = MeshBuffer(positions.map { shape.rotation.act($0) + shape.translation })
        d.normals = MeshBuffer(normals.map { shape.rotation.act($0) })
        d.primitives = .triangles(indices)
        return try .generate(from: [d])
    }

    @MainActor
    private static func hullMesh(_ hull: UnsafeMutablePointer<b3HullData>, shape: B3ShapeResource) throws -> MeshResource {
        let points = b3GetHullPoints(hull)!
        let edges = b3GetHullEdges(hull)!
        let faces = b3GetHullFaces(hull)!
        let planes = b3GetHullPlanes(hull)!
        var positions: [SIMD3<Float>] = []
        var normals: [SIMD3<Float>] = []
        var indices: [UInt32] = []
        for f in 0..<Int(hull.pointee.faceCount) {
            let n = shape.rotation.act(planes[f].normal.simd)
            let base = UInt32(positions.count)
            var e = Int(faces[f].edge)
            let first = e
            repeat {
                positions.append(shape.rotation.act(points[Int(edges[e].origin)].simd) + shape.translation)
                normals.append(n)
                e = Int(edges[e].next)
            } while e != first
            // Fan, counter-clockwise faces with RealityKit's winding.
            for k in 1..<(UInt32(positions.count) - base - 1) {
                indices += [base, base + k, base + k + 1]
            }
        }
        var d = MeshDescriptor()
        d.positions = MeshBuffer(positions)
        d.normals = MeshBuffer(normals)
        d.primitives = .triangles(indices)
        return try .generate(from: [d])
    }
}

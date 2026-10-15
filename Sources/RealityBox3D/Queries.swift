import CBox3D
import RealityKit
import simd

extension B3ShapeResource {
    /// Box3D's query form of a convex shape at a pose: a point cloud and a radius. Nil for static meshes.
    func proxyPoints(at pose: Transform, scale: SIMD3<Float> = .one) -> (points: [b3Vec3], radius: Float)? {
        func place(_ p: SIMD3<Float>) -> b3Vec3 { b3Vec3(pose.rotation.act((rotation.act(p) + translation) * scale) + pose.translation) }
        switch geometry {
        case .sphere(let r):
            return ([place(.zero)], r * scale.max())
        case .capsule(let h, let r):
            let half = max(h / 2 - r, 0)
            return ([place([0, -half, 0]), place([0, half, 0])], r * max(scale.x, scale.z))
        case .box(let size):
            let e = size / 2
            var corners: [b3Vec3] = []
            for x: Float in [-1, 1] { for y: Float in [-1, 1] { for z: Float in [-1, 1] { corners.append(place(e * [x, y, z])) } } }
            return (corners, 0)
        case .convex(let points):
            // Box3D casts at most B3_MAX_SHAPE_CAST_POINTS points: use its simplified hull's.
            var pts = BodyRecord.weld(points).map(b3Vec3.init)
            var hull: UnsafeMutablePointer<b3HullData>?
            for maxVertices: Int32 in [64, 48, 32, 16] where hull == nil {
                hull = b3CreateHull(&pts, Int32(pts.count), maxVertices)
            }
            guard let hull else { return nil }
            defer { b3DestroyHull(hull) }
            let hp = b3GetHullPoints(hull)!
            return ((0..<Int(hull.pointee.vertexCount)).map { place(hp[$0].simd) }, 0)
        case .staticMesh, .heightField:
            return nil
        }
    }
}

extension B3World {
    /// The bodies whose shapes overlap a convex shape placed at a pose in the root's space, as
    /// RealitySDFPhysics's `overlaps(of:at:)`. A margin grows the shape, to keep clear of bodies.
    public func overlaps(of shape: B3ShapeResource, at pose: Transform, margin: Float = 0,
                         mask: CollisionGroup = .all) -> [Entity] {
        guard let proxy = shape.proxyPoints(at: pose, scale: pose.scale) else { return [] }
        var filter = b3DefaultQueryFilter()
        filter.maskBits = UInt64(mask.rawValue)
        final class Box { var shapes: [b3ShapeId] = [] }
        let box = Box()
        proxy.points.withUnsafeBufferPointer { buffer in
            var p = b3ShapeProxy(points: buffer.baseAddress, count: Int32(buffer.count), radius: proxy.radius + margin)
            _ = b3World_OverlapShape(id, b3Vec3(x: 0, y: 0, z: 0), &p, filter, { shape, ctx in
                Unmanaged<Box>.fromOpaque(ctx!).takeUnretainedValue().shapes.append(shape)
                return true
            }, Unmanaged.passUnretained(box).toOpaque())
        }
        var seen = Set<Entity.ID>()
        return box.shapes.compactMap { entity(for: $0) }.filter { seen.insert($0.id).inserted }
    }

    /// Mirrors `Scene.convexCast`: sweeps a convex shape from one position to another without
    /// rotating, and returns what it hits, nearest first. Box3D sweeps translate only, so the shape
    /// keeps `fromOrientation`.
    public func convexCast(convexShape: B3ShapeResource, fromPosition: SIMD3<Float>, fromOrientation: simd_quatf,
                           toPosition: SIMD3<Float>, query: CollisionCastQueryType = .all,
                           mask: CollisionGroup = .all) -> [B3CastHit] {
        guard let proxy = convexShape.proxyPoints(at: Transform(rotation: fromOrientation, translation: fromPosition)) else { return [] }
        var filter = b3DefaultQueryFilter()
        filter.maskBits = UInt64(mask.rawValue)
        let translation = toPosition - fromPosition
        let length = simd_length(translation)
        final class Box { var hits: [(b3ShapeId, SIMD3<Float>, SIMD3<Float>, Float)] = []; var closest = false }
        let box = Box()
        box.closest = query == .nearest || query == .any
        proxy.points.withUnsafeBufferPointer { buffer in
            var p = b3ShapeProxy(points: buffer.baseAddress, count: Int32(buffer.count), radius: proxy.radius)
            _ = b3World_CastShape(id, b3Vec3(x: 0, y: 0, z: 0), &p, b3Vec3(translation), filter, { shape, point, normal, fraction, _, _, _, ctx in
                let b = Unmanaged<Box>.fromOpaque(ctx!).takeUnretainedValue()
                b.hits.append((shape, point.simd, normal.simd, fraction))
                return b.closest ? fraction : 1 // clip to the nearest, or keep collecting
            }, Unmanaged.passUnretained(box).toOpaque())
        }
        var hits = box.hits.sorted { $0.3 < $1.3 }
        if box.closest { hits = Array(hits.prefix(1)) }
        return hits.compactMap { h in entity(for: h.0).map { B3CastHit(entity: $0, position: h.1, normal: h.2, distance: h.3 * length) } }
    }
}

extension B3World {
    /// Box3D extension: a radial blast at `center`, in the simulation root's space. Each sphere,
    /// capsule and hull within `radius` takes an impulse of `impulsePerArea` for each unit of area
    /// facing the blast, fading to zero `falloff` beyond the radius. A negative impulse implodes.
    /// Static meshes and height fields are not pushed.
    public func explode(at center: SIMD3<Float>, radius: Float, falloff: Float = 0, impulsePerArea: Float,
                        mask: CollisionGroup = .all) {
        guard center.x.isFinite, center.y.isFinite, center.z.isFinite, radius.isFinite, radius >= 0,
              falloff.isFinite, falloff >= 0, impulsePerArea.isFinite else { return }
        var def = b3DefaultExplosionDef()
        def.maskBits = UInt64(mask.rawValue)
        def.position = b3Vec3(center)
        def.radius = radius
        def.falloff = falloff
        def.impulsePerArea = impulsePerArea
        b3World_Explode(id, &def)
    }
}

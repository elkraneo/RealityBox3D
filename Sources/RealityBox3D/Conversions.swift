import CBox3D
import simd

extension b3Vec3 {
    @inlinable init(_ v: SIMD3<Float>) { self.init(x: v.x, y: v.y, z: v.z) }
    @inlinable var simd: SIMD3<Float> { SIMD3(x, y, z) }
}

extension b3Quat {
    @inlinable init(_ q: simd_quatf) {
        let n = simd_normalize(q)
        self.init(v: b3Vec3(n.imag), s: n.real)
    }
    @inlinable var simd: simd_quatf { simd_quatf(ix: v.x, iy: v.y, iz: v.z, r: s) }
}

extension b3Transform {
    @inlinable init(translation: SIMD3<Float>, rotation: simd_quatf) {
        self.init(p: b3Vec3(translation), q: b3Quat(rotation))
    }
}

extension b3BodyId: @retroactive Equatable, @retroactive Hashable {
    public static func == (a: b3BodyId, b: b3BodyId) -> Bool {
        a.index1 == b.index1 && a.world0 == b.world0 && a.generation == b.generation
    }
    public func hash(into h: inout Hasher) { h.combine(index1); h.combine(world0); h.combine(generation) }
}

extension b3ShapeId: @retroactive Equatable, @retroactive Hashable {
    public static func == (a: b3ShapeId, b: b3ShapeId) -> Bool {
        a.index1 == b.index1 && a.world0 == b.world0 && a.generation == b.generation
    }
    public func hash(into h: inout Hasher) { h.combine(index1); h.combine(world0); h.combine(generation) }
}

extension b3JointId: @retroactive Equatable, @retroactive Hashable {
    public static func == (a: b3JointId, b: b3JointId) -> Bool {
        a.index1 == b.index1 && a.world0 == b.world0 && a.generation == b.generation
    }
    public func hash(into h: inout Hasher) { h.combine(index1); h.combine(world0); h.combine(generation) }
}

import RealityKit

public enum B3CollisionEventKind: Sendable { case began, ended, hit }

public protocol B3CollisionEvent {
    var entityA: Entity { get }
    var entityB: Entity { get }
    static var kind: B3CollisionEventKind { get }
}

/// Mirrors RealityKit's `CollisionEvents`.
public enum B3CollisionEvents {
    public struct Began: B3CollisionEvent {
        public let entityA: Entity
        public let entityB: Entity
        public static var kind: B3CollisionEventKind { .began }
    }
    public struct Ended: B3CollisionEvent {
        public let entityA: Entity
        public let entityB: Entity
        public static var kind: B3CollisionEventKind { .ended }
    }
    /// Box3D extension: an impact faster than the world's hit threshold.
    public struct Hit: B3CollisionEvent {
        public let entityA: Entity
        public let entityB: Entity
        public let position: SIMD3<Float>
        public let normal: SIMD3<Float>
        public let approachSpeed: Float
        public static var kind: B3CollisionEventKind { .hit }
    }
}

/// Mirrors RealityKit's `PhysicsSimulationEvents`: sent around every fixed step, so code that
/// applies forces each step, such as springs and drags, acts on every step, not once per frame.
public enum B3PhysicsSimulationEvents {
    public struct WillSimulate { public let deltaTime: Float }
    public struct DidSimulate { public let deltaTime: Float }
}

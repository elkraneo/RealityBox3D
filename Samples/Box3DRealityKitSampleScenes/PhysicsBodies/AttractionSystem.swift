// Port of SphereAttractionComponent and SphereAttractionSystem from Apple's sample "Simulating
// physics with collisions in your visionOS app":
// https://developer.apple.com/documentation/realitykit/simulating-physics-with-collisions-in-your-visionos-app
//
// The forces are Apple's, in world space and newtons, through `entity.b3.addForce` in place of
// `ModelEntity.addForce`. RealityBox3D converts them into the root's centimeters.

import RealityBox3D
import RealityKit

/// Marks a body that the other marked bodies and the center attract.
package struct AttractionComponent: Component {}

package struct AttractionSystem: System {
    let entityQuery = EntityQuery(where: .has(AttractionComponent.self))

    package static var dependencies: [SystemDependency] { [.before(B3PhysicsSystem.self)] }

    package init(scene: RealityKit.Scene) {}

    package func update(context: SceneUpdateContext) {
        let bodies = Array(context.entities(matching: entityQuery, updatingSystemWhen: .rendering))
        for body in bodies {
            // Start with a force back to the center, with the inverse-square law.
            let centerForceStrength = Float(0.05)
            let position = body.position(relativeTo: nil)
            let distance = length(position)
            guard distance > 1e-6 else { continue }
            var aggregateForce = -centerForceStrength * (position / distance) / pow(distance, 2)

            for neighbor in bodies where neighbor != body {
                let offset = neighbor.position(relativeTo: nil) - position
                let distance = length(offset)
                guard distance > 1e-6 else { continue }
                let forceFactor = Float(0.1)
                aggregateForce += forceFactor * (offset / distance) / pow(distance, 2)
            }

            body.b3.addForce(aggregateForce, relativeTo: nil)
        }
    }
}

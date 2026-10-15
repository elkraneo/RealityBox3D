import RealityBox3D
import RealityKit
import simd

extension DemoScene {
    static let stacking: [DemoScene] = [
        DemoScene("Stacking", "Card House", upstream: "samples/sample_stacking.cpp CardHouse",
                  summary: "Five levels of 1 mm thick playing cards leaning on each other.",
                  camera: .init(yaw: 30, pitch: 10, distance: 3, target: [0.75, 1, 0.4])) { b in
            b.ground(extent: 10)
            let cardHeight: Float = 0.2, cardThickness: Float = 0.001, cardDepth: Float = 0.1
            let card = B3ShapeResource.generateBox(size: [2 * cardThickness, 2 * cardHeight, 2 * cardDepth])
            let angle0: Float = 25 * .pi / 180, angle1: Float = -25 * .pi / 180, angle2: Float = .pi / 2
            let z: SIMD3<Float> = [0, 0, 1]
            var nb = 5
            var z0: Float = 0
            var y = cardHeight - 0.02
            while nb > 0 {
                var x = z0
                for i in 0..<nb {
                    if i != nb - 1 {
                        b.body(card, at: [x + 0.25, y + cardHeight - 0.015, 0], rotation: simd_quatf(angle: angle2, axis: z),
                               friction: 0.7, rollingResistance: 0.05)
                    }
                    b.body(card, at: [x, y, 0], rotation: simd_quatf(angle: angle1, axis: z), friction: 0.7, rollingResistance: 0.05)
                    x += 0.175
                    b.body(card, at: [x, y, 0], rotation: simd_quatf(angle: angle0, axis: z), friction: 0.7, rollingResistance: 0.05)
                    x += 0.175
                }
                y += cardHeight * 2 - 0.03
                z0 += 0.175
                nb -= 1
            }
        },

        DemoScene("Stacking", "Jenga Stack", upstream: "samples/sample_stacking.cpp JengaStack",
                  summary: "A 30-level tower of 2 m beams, alternating direction each level.",
                  camera: .init(yaw: 35, pitch: 15, distance: 12, target: [0, 2, 0])) { b in
            b.ground(extent: 20)
            let h: Float = 1, r: Float = 0.1
            let beam = B3ShapeResource.generateBox(size: [2 * h, 2 * r, 2 * r])
            for i in 0..<30 {
                let odd = i & 1 == 1
                let alpha: Float = odd ? 0 : 0.5 * .pi
                let x = odd ? 0 : h - 2 * r
                let z = odd ? h - 2 * r : 0
                let y = (2.1 * Float(i) + 0.5) * r
                let q = simd_quatf(angle: alpha, axis: [0, 1, 0])
                b.body(beam, at: [x, y, z], rotation: q, rollingResistance: 0.05)
                b.body(beam, at: [-x, y, -z], rotation: q, rollingResistance: 0.05)
            }
        },

        DemoScene("Stacking", "Dominoes", upstream: "samples/sample_stacking.cpp Dominoes",
                  summary: "30 concentric spiral rings of dominoes, 5,430 in all, each ring tipped at its start.",
                  camera: .init(yaw: 0, pitch: 15, distance: 75, target: .zero)) { b in
            b.ground(extent: 80)
            let domino = B3ShapeResource.generateBox(size: [0.4, 1.6, 0.1])
            var starters: [(Entity, SIMD3<Float>)] = []
            for ring in 0..<30 {
                let radius = 7 + 1.1 * Float(ring)
                var alpha: Float = 0
                while alpha <= 360 {
                    let a = alpha * .pi / 180
                    let normal = SIMD3<Float>(cos(a), 0, sin(a))
                    let p = SIMD3<Float>(radius * cos(a), 0.8, radius * sin(a)) - alpha / 630 * normal
                    let e = b.body(domino, at: p, rotation: simd_quatf(angle: -a, axis: [0, 1, 0]))
                    if alpha == 0 { starters.append((e, p)) }
                    alpha += 2
                }
            }
            // Upstream applies the impulse at creation; here, once the bodies exist.
            b.onFrame { _ in
                for (e, p) in starters where e.b3.isSimulated {
                    e.b3.applyImpulse([0, 0, 25], at: p + [0, 0.8, 0], relativeTo: nil)
                }
                starters.removeAll { $0.0.b3.isSimulated }
            }
        },

        DemoScene("Stacking", "Arch", upstream: "samples/sample_stacking.cpp Arch",
                  summary: "A 17-stone catenary arch with four boxes on top, held by friction alone.",
                  camera: .init(yaw: 25, pitch: 10, distance: 30, target: [0, 5, 0])) { b in
            b.ground(extent: 40)
            let s: Float = 0.25
            let ps1: [SIMD2<Float>] = [[16, 0], [14.93803712795643, 5.133601056842984], [13.79871746027416, 10.24928069555078],
                                       [12.56252963284711, 15.34107019122473], [11.20040987372525, 20.39856541571217],
                                       [9.66521217819836, 25.40369899225096], [7.87179930638133, 30.3179337000085],
                                       [5.635199558196225, 35.03820717801641], [2.405937953536585, 39.09554102558315]].map { $0 * s }
            let ps2: [SIMD2<Float>] = [[24, 0], [22.33619528222415, 6.02299846205841], [20.54936888969905, 12.00964361211476],
                                       [18.60854610798073, 17.9470321677465], [16.46769273811807, 23.81367936585418],
                                       [14.05325025774858, 29.57079353071012], [11.23551045834022, 35.13775818285372],
                                       [7.752568160730571, 40.30450679009583], [3.016931552701656, 44.28891593799322]].map { $0 * s }
            let d: Float = 0.5
            func stone(_ q: [SIMD2<Float>]) -> B3ShapeResource {
                .generateConvex(from: q.flatMap { [SIMD3($0.x, $0.y, -d), SIMD3($0.x, $0.y, d)] })
            }
            for i in 0..<8 {
                b.body(stone([ps1[i], ps2[i], ps2[i + 1], ps1[i + 1]]), at: .zero, density: 200)
            }
            for i in 0..<8 {
                let m = SIMD2<Float>(-1, 1)
                b.body(stone([ps2[i] * m, ps1[i] * m, ps1[i + 1] * m, ps2[i + 1] * m]), at: .zero, density: 200)
            }
            let m = SIMD2<Float>(-1, 1)
            b.body(stone([ps1[8], ps2[8], ps2[8] * m, ps1[8] * m]), at: .zero, density: 200)
            for i in 0..<4 {
                b.body(.generateBox(size: [4, 1, 2 * d]), at: [0, 0.5 + ps2[8].y + Float(i), 0], density: 200)
            }
        },
    ]
}

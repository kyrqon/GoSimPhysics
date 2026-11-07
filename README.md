# GoSimPhysics

GoSimPhysics is a Go library for 2D and 3D rigid-body simulation in game engines.
Embed it in your game or engine; the host application owns the game loop,
rendering, input, and conversion between world units and screen coordinates.

The module path is `github.com/kyrqon/GoSimPhysics`; its root package is named
`physics`. Requires Go 1.27 or later, with no third-party dependencies.

| Simulation | Import path | Shapes and orientation |
| --- | --- | --- |
| 2D | `github.com/kyrqon/GoSimPhysics` | Circles, rotated rectangles, scalar angles |
| 3D | `github.com/kyrqon/GoSimPhysics/physics3d` | Spheres, oriented boxes, quaternions |

## Interactive browser demo

![The GoSimPhysics browser playground](examples/browser/preview.png)

After cloning the repository, double-click
[`examples/browser/demo.html`](examples/browser/demo.html) to open the included
interactive demo. The file contains the Go WebAssembly engine, renderer, and
all assets, and works offline in a current browser. Go is not required to view
this prebuilt file.

Alternatively, with Go installed, run the example from the repository root:

```sh
go run ./examples/browser
```

Open `http://127.0.0.1:8080`. This command rebuilds the WebAssembly from the local
Go source before serving it. Use `-addr 127.0.0.1:8081` to choose another port.
This option also works with managed browsers that restrict local-file access.

The demo includes 3D stacking, mixed-shape falling bodies, a spherical pendulum,
2D billiards, and 2D/3D CCD thin-wall comparisons. Pause or single-step, spawn
shapes, apply impulses, change
gravity and substeps, and drag/scroll to orbit/zoom the 3D camera. Simulation
runs in the actual Go library; JavaScript handles rendering and controls.
The CCD comparisons run the same shot in independent worlds, with CCD enabled
in one and disabled in the other. They automatically pause after one 1/60-second
step so the actual final positions remain visible; reset replays the shot.

To regenerate the standalone HTML after changing the library or frontend:

```sh
go generate ./examples/browser
# Or export to another file:
go run ./examples/browser -export /tmp/gosim-demo.html
```

Frontend sources are in `examples/browser/web`; the Go-to-browser bridge is in
`examples/browser/wasm`. No JavaScript package installation or CDN is used.
The generated file includes the matching Go runtime; its license is reproduced
in [`THIRD_PARTY_LICENSES.txt`](examples/browser/THIRD_PARTY_LICENSES.txt).

## Implemented features

- Static, dynamic, and velocity-driven kinematic bodies.
- One centered circle or rotated box per body, with mass and rotational inertia.
- Gravity, forces, torques, impulses, linear and angular damping, and fixed rotation.
- Circle/circle, circle/box, and box/box contact detection and response, with
  restitution and Coulomb friction.
- Rigid distance joints between two body-local anchors.
- Direct contact queries through `Collide`.
- Opt-in continuous collision detection and pure `TimeOfImpact` sweep queries.

The 3D package provides the same body, force, material, and joint concepts using
`Vec3` and `Quaternion`. It uses the full world-space inertia tensor, including
torque-free precession of anisotropic bodies. Box collision tests all 15 SAT
axes; face clipping produces up to four contacts, while edge/edge contacts use
the closest points on the supporting edges. Tangential impulses are limited to
a Coulomb friction disk, so friction behaves consistently in all directions.

The solver uses semi-implicit Euler integration, sequential contact impulses,
and positional correction. `NewWorld` defaults to four substeps, twelve velocity
iterations, four position iterations, and a restitution speed threshold of
1 world unit per second. These settings are public fields on `World`.

## Embedding the 2D library

Use consistent units: meters, kilograms, seconds, and radians. The examples use
positive Y upward, so gravity points downward. Angles increase counterclockwise.
`Box.HalfExtents` specifies half the width and half the height.

```go
package main

import (
    "fmt"
    "log"

    "github.com/kyrqon/GoSimPhysics"
)

func main() {
    world := physics.NewWorld(physics.Vec2{Y: -9.81})
    ball, err := world.CreateBody(physics.BodyDef{
        Type:     physics.Dynamic,
        Shape:    physics.Circle{Radius: 0.5},
        Position: physics.Vec2{Y: 5},
        Mass:     1,
    })
    if err != nil {
        log.Fatal(err)
    }

    const dt = 1.0 / 60.0
    for range 60 {
        if err := world.Step(dt); err != nil {
            log.Fatal(err)
        }
        // Copy ball.Position and ball.Angle to your engine's transform here.
    }
    fmt.Printf("height after one second: %.3f m\n", ball.Position.Y)
}
```

Advance the simulation with a fixed positive time step, such as `1.0 / 60.0`,
independent of rendering frequency. In a real-time host, accumulate elapsed
time, call `Step(dt)` while at least `dt` remains, and retain the remainder for
the next frame. Bound catch-up work so a stalled renderer cannot create an
unbounded simulation backlog. Smaller substeps help contact stability; enable
`Continuous` on fast bodies when they need swept collision detection.

## Embedding the 3D library

```go
world := physics3d.NewWorld(physics3d.Vec3{Y: -9.81})
body, err := world.CreateBody(physics3d.BodyDef{
    Type:     physics3d.Dynamic,
    Shape:    physics3d.Box{HalfExtents: physics3d.Vec3{X: 0.5, Y: 0.5, Z: 0.5}},
    Position: physics3d.Vec3{Y: 4},
    Rotation: physics3d.QuaternionFromAxisAngle(physics3d.Vec3{Y: 1}, 0.4),
    Mass:     1,
    Friction: 0.6,
})
if err != nil {
    return err
}
if err := world.Step(1.0 / 60); err != nil {
    return err
}
// Copy body.Position and body.Rotation to the host engine's transform.
_ = body
```

Import `github.com/kyrqon/GoSimPhysics/physics3d` in the surrounding Go program.
3D uses a right-handed coordinate system and world-space angular velocities,
torques, forces, and application points. Quaternions have `X, Y, Z, W` fields;
the zero rotation in a body definition defaults to identity. Other nonzero
rotations are normalized. `Inertia` returns local principal moments, and
`AngularMomentum` returns world-space angular momentum. 3D distance joints use
`Vec3` local anchors and otherwise match the 2D API.

## Body and joint behavior

Dynamic bodies require an explicit positive mass. Static bodies stay fixed;
kinematic bodies move at their specified velocity and do not respond to forces
or impulses. Both behave as infinite mass during constraint solving. A body's
shape is copied at creation and cannot be replaced. `FixedRotation` disables
angular motion and response.

Forces and torques accumulate until the next successful `Step`, act throughout
all its substeps, and are then cleared. Apply a continuous force again before
each `Step`. `ApplyForceAtPoint` and `ApplyImpulse` take world-space application
points; off-center applications can change angular motion. Impulses change
velocity immediately. Check the errors returned by body creation, force and
impulse methods, joint creation, and `Step`.

Restitution must be between zero and one; friction and damping must be
nonnegative. Zero values mean no bounce, no friction, and no damping. Contacts
use the larger restitution and the geometric mean of the two friction values.
The restitution threshold suppresses bounce at low relative approach speeds.

A distance joint requires distinct bodies belonging to the same world, at least
one dynamic body, and an explicit positive `Length`. Anchor offsets use each
body's local coordinates; zero offsets attach at the centers. The joint can
both pull and push. Connected bodies do not collide unless `CollideConnected`
is true. `Anchors` returns their current world positions; compare their distance
with `Length` to inspect constraint error. `RemoveBody` also removes its joints.

## Continuous collision detection (CCD)

![CCD enabled versus discrete collision detection](examples/browser/preview-ccd.png)

Set `Continuous: true` on a fast body's definition in either package. The flag
can also be changed on an existing body between steps. A pair uses CCD when
either body has it enabled and at least one body is dynamic; joint collision
filtering still applies. Other pairs retain discrete collision detection.

```go
bullet, err := world.CreateBody(physics.BodyDef{
    Type:       physics.Dynamic,
    Shape:      physics.Circle{Radius: 0.05},
    Position:   physics.Vec2{X: -2},
    Velocity:   physics.Vec2{X: 400},
    Mass:       1,
    Continuous: true,
})
if err != nil {
    return err
}
// Use physics3d.BodyDef, Vec3, and Sphere for the equivalent 3D body.
_ = bullet
```

For each substep the world integrates forces once, searches for the earliest
contact, advances to it, resolves impulses, and then simulates the remaining
time. Impact velocities are used for the next sweep, allowing several bounces
within one frame. Fixed-orientation primitive sweeps use analytic casts;
rotating boxes use bounded conservative advancement with angular motion bounds.
Rounded circle/sphere-to-box corners are tested against the actual shape.

`TimeOfImpact(a, b, dt)` is a pure query returning `(TOIResult, hit, error)`.
`TOIResult.Time` is in **seconds**, not a fraction, and `Contact` contains the
predicted world-space normal and points. The query holds linear and angular
velocities constant, ignores forces and the `Continuous` flags, honors static
and fixed-rotation bodies, and reports initial contact or overlap at time zero.

CCD uses shape-scaled spatial tolerances and piecewise constant velocity and
angular-velocity motion. In 3D, CCD motion segments retain world angular momentum
when recovering angular velocity; their rotational approximation differs from
the default midpoint integration. Initial overlap and constraint position
projection still use the discrete position solver. Direct transform edits are
teleports and are not swept.

`World.MaxTOIEvents` defaults to 32 impacts **per substep**. If that budget is
exhausted, `Step` returns an error wrapping `ErrTOILimit`; check it with
`errors.Is`. An unrepresentable or nonconverging sweep also returns an error.
The complete step is rolled back and pending forces are retained; it never
silently advances the unchecked remainder. Use smaller time steps, more
substeps, or a larger event budget for demanding scenes. Extremely close
rotational grazing contacts may require a smaller step to converge.
Already-active rotating contacts use a query-only continuation skin of
`max(8 * sweepTolerance, min(1e-4, 1e-3 * smallerShapeFeature))` to approximate
persistent contact
without an unbounded number of zero-time impulses. This does not move the real
bodies or change the first-impact query; it permits skin-scale contact error.

Body positions, angles, and velocities may be edited between steps and must
remain finite. Changing a transform directly teleports the body; it does not
sweep through the intervening space. `Step` validates inputs and restores body
state if arithmetic overflow occurs, retaining accumulated forces for a retry.
Do not copy bodies or access a world and its bodies concurrently.

## Examples and development

From the repository root:

```sh
go run ./examples/falling
go run ./examples/pendulum

go build ./...
go test ./...
go test -race ./...
go test -cover ./...
go vet ./...
```

Optional browser interaction tests use Python and Playwright:

```sh
python3 -m pip install playwright
python3 -m playwright install chromium
python3 examples/browser/test_browser.py
```

The test opens the standalone HTML, checks playback, scenes, controls, camera
interaction, and mobile layout, and verifies that no external resources are
requested. If browser policy blocks `file://`, start the local Go demo and set
`GOSIM_DEMO_URL` to `http://127.0.0.1:8080/` when running the test. In that mode,
only the initial HTML loads over HTTP; all interactions are tested offline.

The falling example drops a ball onto a static ground box and prints height and
vertical velocity. The pendulum example connects a moving ball to a static
anchor and prints the measured and target joint lengths. Neither needs a
window or a rendering library.

## Current limitations

This is an initial implementation. Contact and joint constraints are solved
approximately; stability depends on scale, time step, solver settings, and the
scene. Fast bodies without `Continuous` can tunnel through thin shapes. CCD is
tolerance-based with finite iteration and impact budgets, and does not provide
exact accelerated or arbitrary gyroscopic trajectories over an entire frame.
There is no sleeping, compound shape support, or renderer in the core library.
The browser example supplies a small
Canvas renderer for demonstration.

The broad phase scans all body pairs with AABB rejection, taking O(n²) pair
checks rather than using a spatial index. It is intended for small scenes.
There is no thread safety for concurrent access to the same world and no
guarantee of bitwise identical floating-point results across platforms.

## References

The implementation and tests are original. These references explain the
integration and constraint-solving techniques used:

- [Box2D simulation documentation](https://box2d.org/documentation/md_simulation.html)
- [Erin Catto, Sequential Impulses (GDC 2006)](https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf)
- [Erin Catto, Contact Manifolds (GDC 2007)](https://box2d.org/files/ErinCatto_ContactManifolds_GDC2007.pdf)
- [Glenn Fiedler, Integration Basics](https://gafferongames.com/post/integration_basics/)
- [David Eberly, Dynamic Collision Detection using Oriented Bounding Boxes](https://www.geometrictools.com/Documentation/DynamicCollisionDetection.pdf)
- [David Baraff, An Introduction to Physically Based Modeling: Rigid Body Simulation](https://www.cs.cmu.edu/~baraff/pbm/rigid1.pdf)
- [Box2D collision and time-of-impact documentation](https://box2d.org/documentation/md_collision.html)
- [Bullet continuous convex collision / conservative advancement](https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp)

Tests include original analytical impact fixtures, free-fall convergence,
anisotropic angular-momentum conservation, resting bodies, stacking, spherical
pendulums, friction direction invariance, and invalid-step rollback. Fixture
derivations are documented in `testdata/README.md` and
`physics3d/testdata/README.md`.
CCD fixture derivations and thin-wall/ricochet cases are documented in
[`testdata/CCD.md`](testdata/CCD.md).

The project is licensed under GPL-3.0; see [LICENSE](LICENSE).

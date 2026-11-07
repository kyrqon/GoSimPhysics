# GoSimPhysics

GoSimPhysics is a small Go library for 2D rigid-body simulation in game engines.
Embed it in your game or engine; the host application owns the game loop,
rendering, input, and conversion between world units and screen coordinates.

The module path is `github.com/kyrqon/GoSimPhysics`; its root package is named
`physics`. Requires Go 1.27 or later, with no third-party dependencies.

## Implemented features

- Static, dynamic, and velocity-driven kinematic bodies.
- One centered circle or rotated box per body, with mass and rotational inertia.
- Gravity, forces, torques, impulses, linear and angular damping, and fixed rotation.
- Circle/circle, circle/box, and box/box contact detection and response, with
  restitution and Coulomb friction.
- Rigid distance joints between two body-local anchors.
- Direct contact queries through `Collide`.

The solver uses semi-implicit Euler integration, sequential contact impulses,
and positional correction. `NewWorld` defaults to four substeps, twelve velocity
iterations, four position iterations, and a restitution speed threshold of
1 world unit per second. These settings are public fields on `World`.

## Embedding the library

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
unbounded simulation backlog. Smaller substeps help contact stability but do
not provide continuous collision detection.

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

The falling example drops a ball onto a static ground box and prints height and
vertical velocity. The pendulum example connects a moving ball to a static
anchor and prints the measured and target joint lengths. Neither needs a
window or a rendering library.

## Current limitations

This is an initial implementation. Contact and joint constraints are solved
approximately; stability depends on scale, time step, solver settings, and the
scene. Collision detection is discrete, so fast objects can tunnel through thin
shapes. There is no continuous collision detection, sleeping, 3D simulation,
compound shape support, or renderer.

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

The project is licensed under GPL-3.0; see [LICENSE](LICENSE).

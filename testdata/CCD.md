# Analytic continuous-collision fixtures

`ccd_impacts.json` is original test data derived from constant-velocity geometry,
conservation of linear momentum, and Newton's coefficient of restitution. It has
no external data dependency and can be used identically by the 2D and 3D tests:
all trajectories lie on the x axis, through the centers of circles or spheres.
Lengths, masses, and times use internally consistent arbitrary units. The impact
times are seconds, not fractions of a simulation step.

For a round body of radius `r` moving right from `x` toward the left face of a
stationary wall of half-width `h`, first contact occurs at
`t = (-h - r - x) / v`. Thus `r=0.05`, `h=0.01`, `x=-2`, `v=400` gives
`t=0.00485`. With restitution zero the body stops at `x=-0.06`. With restitution
one it reverses velocity and travels during the remaining `0.01-t` seconds,
finishing at `x=-2.12`. Testing final position detects implementations that stop
the clock at the first impact.

For two round bodies the first-contact time is
`t = (xB-xA-rA-rB)/(vA-vB)`. The fixture gives `t=3.8/300`.
Post-impact velocities follow
`vA' = (mA*vA + mB*vB - mB*e*(vA-vB))/(mA+mB)` and
`vB' = (mA*vA + mB*vB + mA*e*(vA-vB))/(mA+mB)`.
They are `-250` and `50`, and advancing both trajectories for the remainder of
the step gives final positions `-1.3` and `1.1`. Momentum is `-100`, and kinetic
energy is `35000` before and after this elastic collision.

The integration tests additionally derive repeated elastic impacts using a
folded line segment. Walls at `-1` and `1`, each of half-width `0.01`, and body
radius `0.05` constrain the center to `[-0.94,0.94]`. Starting from zero at speed
100 for 0.05 seconds covers distance 5, hits three walls, and ends at `0.64`
with velocity `-100`. This oracle uses only modular arithmetic, not engine
collision or time-of-impact calls.

The rounded-corner miss regression uses a unit-half-extent box and radius 0.1
round body, moving from `(-1.11,1.08)` to `(-1.08,1.11)`. Its nearest approach to
the corner `(-1,1)` is `sqrt(2)*0.095`, greater than 0.1. An inflated rectangular
AABB reports an intersection; the actual rounded shape must miss.

The rotational re-entry regression starts a kinematic rod of half-extents
`(2,0.02)` at zero angle, rotating at `-2*pi` radians per second. A round body
of radius `0.05` starts at `(1.4,0.07)`, moving upward at `0.2`. Their initial
touch is separating. Expressing the moving center in the rotating rod's local
coordinates gives
`localY(t) = 1.4*sin(2*pi*t) + (0.07+0.2*t)*cos(2*pi*t)`.
The rod's opposite end returns from above when `localY` next falls to `0.07`,
at approximately `0.47345669262113366` seconds. A `0.49` second step must apply a
downward impulse rather than ignore the pair after its separating initial touch.
The same geometry lies in the xy plane for the 3D version.

These tests use a single original simulation substep, zero friction and zero
restitution threshold. Position and velocity tolerances are `1e-5` or smaller
to accommodate numerical contact construction while rejecting tunneling,
early stopping, repeated force application, and appreciable false collisions.

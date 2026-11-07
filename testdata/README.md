# Analytical fixtures

`impacts.json` is original test data calculated from conservation of linear
momentum and Newton's coefficient of restitution for a central, frictionless,
one-dimensional impact. It is not copied from another engine's output.

For masses `mA`, `mB`, initial velocities `uA`, `uB`, and restitution `e`:

```text
vA = (mA*uA + mB*uB - mB*e*(uA-uB)) / (mA+mB)
vB = (mA*uA + mB*uB + mA*e*(uA-uB)) / (mA+mB)
```

The tests also check total momentum and, for `e = 1`, kinetic energy. These
checks are independent of the engine's impulse implementation.

The free-fall test uses the continuous solution `y = y0 + v0*T + a*T*T/2` and
checks that the positional error decreases when the time step is halved.

Algorithm references:

- [Box2D simulation documentation](https://box2d.org/documentation/md_simulation.html)
- [Glenn Fiedler, Integration Basics](https://gafferongames.com/post/integration_basics/)
- [Erin Catto, Sequential Impulses (GDC 2006)](https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf)

Tests use only these checked-in data and the Go standard library; they never
download files or depend on an external simulation engine.

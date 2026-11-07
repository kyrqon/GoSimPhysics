package physics3d

import (
	"math"
	"testing"
)

func body3Create(t *testing.T, def BodyDef) *Body {
	t.Helper()
	b, err := newBody(def)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func TestBodySphereMassAndOffCenterImpulse(t *testing.T) {
	b := body3Create(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 2})
	if b.Type() != Dynamic || b.Shape() != (Sphere{Radius: 1}) || b.Mass() != 2 || b.Rotation != Identity() {
		t.Fatalf("unexpected body definition: %+v", b)
	}
	bodyQueryMathVecNear(t, "sphere inertia", b.Inertia(), Vec3{X: 0.8, Y: 0.8, Z: 0.8}, 0)
	if err := b.ApplyImpulse(Vec3{X: 4}, Vec3{Y: 1}); err != nil {
		t.Fatal(err)
	}
	bodyQueryMathVecNear(t, "linear velocity", b.Velocity, Vec3{X: 2}, 0)
	bodyQueryMathVecNear(t, "angular velocity", b.AngularVelocity, Vec3{Z: -5}, 0)
	bodyQueryMathVecNear(t, "angular momentum", b.AngularMomentum(), Vec3{Z: -4}, 1e-15)
}

func TestBodyRotatedAnisotropicWorldInertia(t *testing.T) {
	b := body3Create(t, BodyDef{
		Type: Dynamic, Shape: Box{HalfExtents: Vec3{X: 1, Y: 2, Z: 3}}, Mass: 3,
		Rotation: QuaternionFromAxisAngle(Vec3{Z: 1}, math.Pi/2),
	})
	bodyQueryMathVecNear(t, "local inertia", b.Inertia(), Vec3{X: 13, Y: 10, Z: 5}, 0)
	// The local Y principal axis now points along world -X, so a unit world-X
	// angular impulse must be divided by Iy=10, rather than by Ix=13.
	if err := b.ApplyImpulse(Vec3{Z: 1}, Vec3{Y: 1}); err != nil {
		t.Fatal(err)
	}
	bodyQueryMathVecNear(t, "world angular velocity", b.AngularVelocity, Vec3{X: 0.1}, 1e-15)
	bodyQueryMathVecNear(t, "world angular momentum", b.AngularMomentum(), Vec3{X: 1}, 1e-14)
	bodyQueryMathVecNear(t, "world linear velocity", b.Velocity, Vec3{Z: 1.0 / 3}, 0)
}

func TestBodyForceTorqueAccumulationAndTransforms(t *testing.T) {
	b := body3Create(t, BodyDef{
		Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 2,
		Position: Vec3{X: 2, Y: 3, Z: 4}, Rotation: QuaternionFromAxisAngle(Vec3{Z: 1}, math.Pi/2),
	})
	if err := b.ApplyForce(Vec3{X: 1}); err != nil {
		t.Fatal(err)
	}
	if err := b.ApplyForceAtPoint(Vec3{X: 3}, b.Position.Add(Vec3{Y: 2})); err != nil {
		t.Fatal(err)
	}
	if err := b.ApplyTorque(Vec3{Z: 1}); err != nil {
		t.Fatal(err)
	}
	bodyQueryMathVecNear(t, "accumulated force", b.force, Vec3{X: 4}, 0)
	bodyQueryMathVecNear(t, "accumulated torque", b.torque, Vec3{Z: -5}, 0)
	bodyQueryMathVecNear(t, "forces are not impulses", b.Velocity, Vec3{}, 0)
	local := Vec3{X: 1, Y: 2, Z: 3}
	bodyQueryMathVecNear(t, "world point", b.WorldPoint(local), Vec3{X: 0, Y: 4, Z: 7}, 1e-15)
	bodyQueryMathVecNear(t, "local point", b.LocalPoint(b.WorldPoint(local)), local, 2e-15)
}

func TestBodyStaticKinematicAndFixedRotation(t *testing.T) {
	for _, kind := range []BodyType{Static, Kinematic} {
		b := body3Create(t, BodyDef{
			Type: kind, Shape: Sphere{Radius: 1}, Velocity: Vec3{X: 1, Y: 2, Z: 3}, AngularVelocity: Vec3{X: 4, Y: 5, Z: 6},
		})
		before := *b
		if err := b.ApplyForce(Vec3{X: 1, Y: 2, Z: 3}); err != nil {
			t.Fatal(err)
		}
		if err := b.ApplyForceAtPoint(Vec3{X: 1, Y: 2, Z: 3}, Vec3{X: 4, Y: 5, Z: 6}); err != nil {
			t.Fatal(err)
		}
		if err := b.ApplyTorque(Vec3{X: 1, Y: 2, Z: 3}); err != nil {
			t.Fatal(err)
		}
		if err := b.ApplyImpulse(Vec3{X: 1, Y: 2, Z: 3}, Vec3{X: 4, Y: 5, Z: 6}); err != nil {
			t.Fatal(err)
		}
		b.applyImpulse(Vec3{X: 1, Y: 2, Z: 3}, Vec3{X: 4, Y: 5, Z: 6})
		if *b != before || b.Mass() != 0 || b.Inertia() != (Vec3{}) || b.AngularMomentum() != (Vec3{}) {
			t.Fatalf("nondynamic body responded to force or impulse: %+v", b)
		}
		if kind == Static && (b.Velocity != (Vec3{}) || b.AngularVelocity != (Vec3{})) {
			t.Fatal("static creation retained velocity")
		}
	}
	b := body3Create(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 2, FixedRotation: true, AngularVelocity: Vec3{X: 9}})
	if err := b.ApplyForceAtPoint(Vec3{X: 4}, Vec3{Y: 1}); err != nil {
		t.Fatal(err)
	}
	if err := b.ApplyTorque(Vec3{Z: 5}); err != nil {
		t.Fatal(err)
	}
	if err := b.ApplyImpulse(Vec3{X: 4}, Vec3{Y: 1}); err != nil {
		t.Fatal(err)
	}
	if b.torque != (Vec3{}) || b.AngularVelocity != (Vec3{}) || b.invInertia != (Vec3{}) {
		t.Fatal("fixed rotation responded angularly")
	}
	bodyQueryMathVecNear(t, "fixed rotation linear response", b.Velocity, Vec3{X: 2}, 0)
	bodyQueryMathVecNear(t, "fixed rotation physical inertia", b.Inertia(), Vec3{X: 0.8, Y: 0.8, Z: 0.8}, 0)
}

func TestBodyDefinitionValidation(t *testing.T) {
	base := BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1}
	tests := []struct {
		name string
		bad  func(*BodyDef)
	}{
		{"shape", func(d *BodyDef) { d.Shape = nil }},
		{"type", func(d *BodyDef) { d.Type = 99 }},
		{"zero mass", func(d *BodyDef) { d.Mass = 0 }},
		{"negative mass", func(d *BodyDef) { d.Mass = -1 }},
		{"infinite mass", func(d *BodyDef) { d.Mass = math.Inf(1) }},
		{"inverse mass overflow", func(d *BodyDef) { d.Mass = 1e-320 }},
		{"inertia overflow", func(d *BodyDef) { d.Shape = Box{HalfExtents: Vec3{X: 1e300, Y: 1e300, Z: 1e300}} }},
		{"inertia underflow", func(d *BodyDef) { d.Shape = Sphere{Radius: 1e-300} }},
		{"position", func(d *BodyDef) { d.Position.Y = math.Inf(1) }},
		{"velocity", func(d *BodyDef) { d.Velocity.Z = math.NaN() }},
		{"angular velocity", func(d *BodyDef) { d.AngularVelocity.X = math.Inf(1) }},
		{"rotation", func(d *BodyDef) { d.Rotation.W = math.NaN() }},
		{"negative restitution", func(d *BodyDef) { d.Restitution = -0.1 }},
		{"high restitution", func(d *BodyDef) { d.Restitution = 1.1 }},
		{"nan restitution", func(d *BodyDef) { d.Restitution = math.NaN() }},
		{"negative friction", func(d *BodyDef) { d.Friction = -1 }},
		{"infinite friction", func(d *BodyDef) { d.Friction = math.Inf(1) }},
		{"negative damping", func(d *BodyDef) { d.LinearDamping = -1 }},
		{"nan damping", func(d *BodyDef) { d.AngularDamping = math.NaN() }},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			def := base
			tt.bad(&def)
			if b, err := newBody(def); err == nil || b != nil {
				t.Fatalf("accepted invalid definition: %v, %v", b, err)
			}
		})
	}
	for _, kind := range []BodyType{Static, Kinematic} {
		def := base
		def.Type, def.Mass = kind, -1
		if _, err := newBody(def); err == nil {
			t.Fatal("negative ignored mass accepted")
		}
	}
}

func TestBodyCopiesShapeAndNormalizesRotation(t *testing.T) {
	shape := &Sphere{Radius: 1}
	b := body3Create(t, BodyDef{Type: Dynamic, Shape: shape, Mass: 1, Rotation: Quaternion{W: 1e300}})
	shape.Radius = 4
	if b.Shape() != (Sphere{Radius: 1}) || b.Rotation != Identity() {
		t.Fatal("body retained mutable shape or unnormalized rotation")
	}
}

func TestBodyInvalidAndOverflowingOperationsAreAtomic(t *testing.T) {
	for _, operation := range []func(*Body) error{
		func(b *Body) error { return b.ApplyForce(Vec3{X: math.NaN()}) },
		func(b *Body) error { return b.ApplyForceAtPoint(Vec3{X: 1}, Vec3{Z: math.Inf(1)}) },
		func(b *Body) error { return b.ApplyForceAtPoint(Vec3{Y: math.MaxFloat64}, Vec3{X: math.MaxFloat64}) },
		func(b *Body) error { return b.ApplyTorque(Vec3{Z: math.Inf(1)}) },
		func(b *Body) error { return b.ApplyImpulse(Vec3{X: math.Inf(1)}, Vec3{}) },
		func(b *Body) error { return b.ApplyImpulse(Vec3{Y: math.MaxFloat64}, Vec3{X: math.MaxFloat64}) },
	} {
		b := body3Create(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1})
		before := *b
		if err := operation(b); err == nil || *b != before {
			t.Fatalf("invalid operation did not fail atomically: error %v, state %+v", err, b)
		}
	}
	for _, angular := range []bool{false, true} {
		b := body3Create(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1})
		apply := b.ApplyForce
		if angular {
			apply = b.ApplyTorque
		}
		if err := apply(Vec3{X: math.MaxFloat64}); err != nil {
			t.Fatal(err)
		}
		before := *b
		if err := apply(Vec3{X: math.MaxFloat64}); err == nil || *b != before {
			t.Fatalf("accumulation overflow was not atomic: %v", err)
		}
	}
}

func TestBodyFixedRotationSkipsUnrepresentableUnusedTorque(t *testing.T) {
	b := body3Create(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1e308, FixedRotation: true})
	impulse, point := Vec3{Y: 1e308}, Vec3{X: 1e308}
	if err := b.ApplyImpulse(impulse, point); err != nil {
		t.Fatal(err)
	}
	b.applyImpulse(impulse, point)
	bodyQueryMathVecNear(t, "finite locked linear velocity", b.Velocity, Vec3{Y: 2}, 1e-15)
	if b.AngularVelocity != (Vec3{}) || !b.validState() {
		t.Fatal("fixed rotation computed an unused overflowing torque")
	}
}

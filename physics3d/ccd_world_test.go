package physics3d

import (
	"errors"
	"math"
	"testing"
)

func ccd3DBody(t *testing.T, w *World, def BodyDef) *Body {
	t.Helper()
	b, err := w.CreateBody(def)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func ccd3DNear(t *testing.T, value, want, tolerance float64) {
	t.Helper()
	if !finite(value) || math.Abs(value-want) > tolerance {
		t.Fatalf("got %.12g, want %.12g within %g", value, want, tolerance)
	}
}

func TestWorldCCDThinWallAndRemainingTime(t *testing.T) {
	for _, test := range []struct {
		name                string
		continuous          bool
		restitution         float64
		wantX, wantVelocity float64
	}{
		{"discrete_can_tunnel", false, 0, 2, 400},
		{"continuous_stops", true, 0, -0.06, 0},
		{"continuous_bounces_and_uses_remainder", true, 1, -2.12, -400},
	} {
		t.Run(test.name, func(t *testing.T) {
			w := NewWorld(Vec3{})
			w.Substeps = 1
			ccd3DBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 2, 2}}})
			ball := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05}, Mass: 1,
				Position: Vec3{X: -2}, Velocity: Vec3{X: 400}, Restitution: test.restitution, Continuous: test.continuous})
			if err := w.Step(0.01); err != nil {
				t.Fatal(err)
			}
			ccd3DNear(t, ball.Position.X, test.wantX, 1e-7)
			ccd3DNear(t, ball.Velocity.X, test.wantVelocity, 1e-7)
		})
	}
}

func TestWorldCCDDynamicRelativeMotion(t *testing.T) {
	w := NewWorld(Vec3{})
	w.Substeps = 1
	a := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 1,
		Position: Vec3{X: -2}, Velocity: Vec3{X: 200}, Restitution: 1, Continuous: true})
	b := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 3,
		Position: Vec3{X: 2}, Velocity: Vec3{X: -100}, Restitution: 1})
	if err := w.Step(0.02); err != nil {
		t.Fatal(err)
	}
	ccd3DNear(t, a.Position.X, -1.3, 1e-7)
	ccd3DNear(t, b.Position.X, 1.1, 1e-7)
	ccd3DNear(t, a.Velocity.X, -250, 1e-8)
	ccd3DNear(t, b.Velocity.X, 50, 1e-8)
	ccd3DNear(t, a.Mass()*a.Velocity.X+b.Mass()*b.Velocity.X, -100, 1e-8)
	ccd3DNear(t, 0.5*a.Mass()*a.Velocity.LengthSquared()+0.5*b.Mass()*b.Velocity.LengthSquared(), 35000, 1e-6)
}

func TestWorldCCDKinematicMovingWallAndStaticFlag(t *testing.T) {
	w := NewWorld(Vec3{})
	w.Substeps = 1
	wall := ccd3DBody(t, w, BodyDef{Type: Kinematic, Shape: Box{HalfExtents: Vec3{0.01, 2, 2}}, Velocity: Vec3{X: 50}, Continuous: true})
	ball := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05}, Mass: 1,
		Position: Vec3{X: -2}, Velocity: Vec3{X: 400}})
	if err := w.Step(0.01); err != nil {
		t.Fatal(err)
	}
	ccd3DNear(t, wall.Position.X, 0.5, 1e-9)
	ccd3DNear(t, wall.Velocity.X, 50, 1e-9)
	ccd3DNear(t, ball.Position.X, 0.44, 1e-7)
	ccd3DNear(t, ball.Velocity.X, 50, 1e-7)

	w = NewWorld(Vec3{})
	w.Substeps = 1
	ccd3DBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 2, 2}}, Continuous: true})
	ball = ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05}, Mass: 1,
		Position: Vec3{X: -2}, Velocity: Vec3{X: 400}})
	if err := w.Step(0.01); err != nil {
		t.Fatal(err)
	}
	ccd3DNear(t, ball.Position.X, -0.06, 1e-7)
}

func ccd3DCorridor(t *testing.T) (*World, *Body) {
	t.Helper()
	w := NewWorld(Vec3{})
	w.Substeps = 1
	for _, x := range []float64{-1, 1} {
		ccd3DBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 2, 2}}, Position: Vec3{X: x}})
	}
	b := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05}, Mass: 2, Velocity: Vec3{X: 1000}, Restitution: 1, Continuous: true})
	return w, b
}

func TestWorldCCDMultipleRicochetsAndForceLifetime(t *testing.T) {
	w, ball := ccd3DCorridor(t)
	if err := ball.ApplyForce(Vec3{Y: 8}); err != nil {
		t.Fatal(err)
	}
	if err := ball.ApplyTorque(Vec3{Z: 4}); err != nil {
		t.Fatal(err)
	}
	if err := w.Step(0.01); err != nil {
		t.Fatal(err)
	}
	ccd3DNear(t, ball.Position.X, -0.6, 1e-6)
	ccd3DNear(t, ball.Velocity.X, -1000, 1e-7)
	ccd3DNear(t, ball.Velocity.Y, 0.04, 1e-10)
	ccd3DNear(t, ball.Position.Y, 0.0004, 1e-10)
	ccd3DNear(t, ball.AngularVelocity.Z, 20, 1e-7)
	if err := w.Step(0.01); err != nil {
		t.Fatal(err)
	}
	ccd3DNear(t, ball.Velocity.Y, 0.04, 1e-10)
	ccd3DNear(t, ball.AngularVelocity.Z, 20, 1e-7)
}

func TestWorldCCDLimitRollsBackAllBodiesAndPendingForce(t *testing.T) {
	w, ball := ccd3DCorridor(t)
	w.MaxTOIEvents = 1
	if err := ball.ApplyForce(Vec3{Y: 8}); err != nil {
		t.Fatal(err)
	}
	if err := ball.ApplyTorque(Vec3{Z: 4}); err != nil {
		t.Fatal(err)
	}
	before := *ball
	err := w.Step(0.01)
	if !errors.Is(err, ErrTOILimit) {
		t.Fatalf("got %v, want ErrTOILimit", err)
	}
	if *ball != before {
		t.Fatal("failed CCD step changed state or consumed force")
	}
	w.MaxTOIEvents = 32
	if err := w.Step(0.01); err != nil {
		t.Fatal(err)
	}
	ccd3DNear(t, ball.Velocity.Y, 0.04, 1e-10)
	ccd3DNear(t, ball.AngularVelocity.Z, 20, 1e-7)
}

func TestWorldCCDTouchingSeparatingAndMutableFlag(t *testing.T) {
	for _, speed := range []float64{-400, 400} {
		w := NewWorld(Vec3{})
		w.Substeps = 1
		ccd3DBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 2, 2}}})
		ball := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05}, Mass: 1,
			Position: Vec3{X: -0.06}, Velocity: Vec3{X: speed}})
		ball.Continuous = true
		if err := w.Step(0.01); err != nil {
			t.Fatal(err)
		}
		wantX, wantSpeed := -0.06, 0.0
		if speed < 0 {
			wantX, wantSpeed = -4.06, speed
		}
		ccd3DNear(t, ball.Position.X, wantX, 1e-7)
		ccd3DNear(t, ball.Velocity.X, wantSpeed, 1e-7)
	}
}

func TestWorldCCDBoxFaceAndJointFiltering(t *testing.T) {
	w := NewWorld(Vec3{})
	w.Substeps = 1
	ccd3DBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 2, 2}}})
	box := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec3{0.05, 0.04, 0.04}}, Mass: 1,
		Position: Vec3{X: -2}, Velocity: Vec3{X: 400}, Restitution: 1, Continuous: true})
	if err := w.Step(0.01); err != nil {
		t.Fatal(err)
	}
	ccd3DNear(t, box.Position.X, -2.12, 1e-5)
	ccd3DNear(t, box.Velocity.X, -400, 1e-5)

	w = NewWorld(Vec3{})
	w.Substeps = 1
	anchor := ccd3DBody(t, w, BodyDef{Shape: Sphere{Radius: 0.5}})
	ball := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05}, Mass: 1,
		Position: Vec3{X: -2}, Velocity: Vec3{X: 400}, Continuous: true})
	if _, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: anchor, BodyB: ball, Length: 2}); err != nil {
		t.Fatal(err)
	}
	if err := w.Step(0.01); err != nil {
		t.Fatal(err)
	}
	ccd3DNear(t, ball.Position.X, 2, 1e-7)
}

func TestWorldCCDContactApproachIsGalileanInvariant(t *testing.T) {
	for _, drift := range []float64{0, 1e6} {
		w := NewWorld(Vec3{})
		w.Substeps = 1
		a := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1e-5}, Mass: 1,
			Velocity: Vec3{X: drift + 1e-4}, Continuous: true})
		b := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1e-5}, Mass: 1,
			Position: Vec3{X: 2e-5}, Velocity: Vec3{X: drift}})
		if err := w.Step(0.5); err != nil {
			t.Fatal(err)
		}
		ccd3DNear(t, a.Velocity.X, drift+5e-5, 1e-9)
		ccd3DNear(t, b.Velocity.X, drift+5e-5, 1e-9)
		if a.Position.X >= b.Position.X {
			t.Fatalf("common velocity %g changed contact ordering", drift)
		}
		ccd3DNear(t, b.Position.X-a.Position.X, 2e-5, 1e-9)
	}
}

func TestWorldCCDRejectsRelativeSpeedOverflowInsteadOfSkippingContact(t *testing.T) {
	w := NewWorld(Vec3{})
	w.Substeps = 1
	speed := math.MaxFloat64 * 0.75
	a := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1,
		Position: Vec3{X: -1}, Velocity: Vec3{X: speed}, Continuous: true})
	b := ccd3DBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1,
		Position: Vec3{X: 1}, Velocity: Vec3{X: -speed}})
	beforeA, beforeB := *a, *b
	if err := w.Step(0.25); err == nil {
		t.Fatal("overflowing relative contact speed was treated as separating")
	}
	if *a != beforeA || *b != beforeB {
		t.Fatal("invalid relative speed partially advanced the world")
	}
}

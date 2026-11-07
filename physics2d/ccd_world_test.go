package physics2d

import (
	"errors"
	"math"
	"testing"
)

func TestCCDContactApproachIsGalileanInvariant(t *testing.T) {
	for _, drift := range []float64{0, 1e6} {
		w := NewWorld(Vec2{})
		w.Substeps = 1
		a := worldTestBody(t, w, BodyDef{
			Type: Dynamic, Shape: Circle{Radius: 1e-5}, Mass: 1,
			Velocity: Vec2{X: drift + 1e-4}, Continuous: true,
		})
		b := worldTestBody(t, w, BodyDef{
			Type: Dynamic, Shape: Circle{Radius: 1e-5}, Mass: 1,
			Position: Vec2{X: 2e-5}, Velocity: Vec2{X: drift},
		})
		worldTestStep(t, w, 0.5)
		worldTestClose(t, a.Velocity.X, drift+5e-5, 1e-9)
		worldTestClose(t, b.Velocity.X, drift+5e-5, 1e-9)
		if a.Position.X >= b.Position.X {
			t.Fatalf("approaching pair crossed under common drift %g", drift)
		}
		worldTestClose(t, b.Position.X-a.Position.X, 2e-5, 1e-9)
	}
}

func TestCCDEventLimitErrorIsInspectableAndAtomic(t *testing.T) {
	w := NewWorld(Vec2{})
	w.Substeps, w.MaxTOIEvents = 1, 1
	for _, x := range []float64{-1, 1} {
		worldTestBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec2{X: 0.01, Y: 1}}, Position: Vec2{X: x}, Restitution: 1})
	}
	b := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.05}, Mass: 1, Velocity: Vec2{X: 100}, Restitution: 1, Continuous: true})
	if err := b.ApplyForce(Vec2{Y: 1}); err != nil {
		t.Fatal(err)
	}
	before := *b
	err := w.Step(0.05)
	if !errors.Is(err, ErrTOILimit) {
		t.Fatalf("expected inspectable CCD budget error, got %v", err)
	}
	if *b != before {
		t.Fatal("failed CCD step consumed force or changed body state")
	}
	for _, bad := range []int{0, -1} {
		w.MaxTOIEvents = bad
		if w.Step(0.05) == nil || *b != before {
			t.Fatal("invalid CCD budget was accepted or changed state")
		}
	}
}

func TestCCDMovingBoundsFailurePreservesPendingForce(t *testing.T) {
	w := NewWorld(Vec2{})
	w.Substeps = 1
	b := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 1, Velocity: Vec2{X: math.MaxFloat64 / 2}, Continuous: true})
	worldTestBody(t, w, BodyDef{Shape: Circle{Radius: 1}, Position: Vec2{X: 10}})
	if err := b.ApplyForce(Vec2{Y: 1}); err != nil {
		t.Fatal(err)
	}
	before := *b
	if w.Step(8) == nil || *b != before {
		t.Fatal("overflowing continuous motion was accepted or partially applied")
	}
}

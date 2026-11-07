package physics

import (
	"math"
	"testing"
)

func TestCoreShapeInertia(t *testing.T) {
	// Solid disk: I = mr²/2. Solid rectangle: I = m(w²+h²)/12.
	coreClose(t, (Circle{Radius: 2}).inertia(3), 6, 1e-14)
	coreClose(t, (Box{HalfExtents: Vec2{2, 3}}).inertia(3), 13, 1e-14)
}

func TestCoreShapeValidation(t *testing.T) {
	for name, shape := range map[string]Shape{
		"circle zero":        Circle{},
		"circle negative":    Circle{Radius: -1},
		"circle nan":         Circle{Radius: math.NaN()},
		"circle infinity":    Circle{Radius: math.Inf(1)},
		"box zero":           Box{},
		"box missing height": Box{HalfExtents: Vec2{1, 0}},
		"box negative":       Box{HalfExtents: Vec2{-1, 1}},
		"box nan":            Box{HalfExtents: Vec2{math.NaN(), 1}},
		"box infinity":       Box{HalfExtents: Vec2{1, math.Inf(1)}},
	} {
		t.Run(name, func(t *testing.T) {
			if shape.valid() {
				t.Fatal("invalid shape accepted")
			}
		})
	}
	for _, shape := range []Shape{Circle{Radius: 0.5}, Box{HalfExtents: Vec2{0.5, 2}}} {
		if !shape.valid() {
			t.Fatalf("valid shape rejected: %#v", shape)
		}
	}
}

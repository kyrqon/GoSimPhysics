package collision2d

import (
	"math"
	"testing"
)

func TestCoreShapeInertia(t *testing.T) {
	// Solid disk: I = mr²/2. Solid rectangle: I = m(w²+h²)/12.
	collisionClose(t, (Circle{Radius: 2}).Inertia(3), 6, 1e-14)
	collisionClose(t, (Box{HalfExtents: Vec2{X: 2, Y: 3}}).Inertia(3), 13, 1e-14)
}

func TestCoreShapeValidation(t *testing.T) {
	for name, shape := range map[string]Shape{
		"circle zero":        Circle{},
		"circle negative":    Circle{Radius: -1},
		"circle nan":         Circle{Radius: math.NaN()},
		"circle infinity":    Circle{Radius: math.Inf(1)},
		"box zero":           Box{},
		"box missing height": Box{HalfExtents: Vec2{X: 1, Y: 0}},
		"box negative":       Box{HalfExtents: Vec2{X: -1, Y: 1}},
		"box nan":            Box{HalfExtents: Vec2{X: math.NaN(), Y: 1}},
		"box infinity":       Box{HalfExtents: Vec2{X: 1, Y: math.Inf(1)}},
	} {
		t.Run(name, func(t *testing.T) {
			if shape.valid() {
				t.Fatal("invalid shape accepted")
			}
		})
	}
	for _, shape := range []Shape{Circle{Radius: 0.5}, Box{HalfExtents: Vec2{X: 0.5, Y: 2}}} {
		if !shape.valid() {
			t.Fatalf("valid shape rejected: %#v", shape)
		}
	}
}

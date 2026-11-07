package collision3d

import (
	"math"
	"testing"
)

func TestShapePrincipalInertia(t *testing.T) {
	// A sphere has 2*m*r^2/5 about every axis. A cuboid has
	// m*(height^2+depth^2)/12 about its local X axis, and similarly for Y/Z.
	collisionShapeVecNear(t, (Sphere{Radius: 2}).Inertia(5), Vec3{X: 8, Y: 8, Z: 8}, 0)
	collisionShapeVecNear(t, (Box{HalfExtents: Vec3{X: 1, Y: 2, Z: 3}}).Inertia(12), Vec3{X: 52, Y: 40, Z: 20}, 0)
}

func TestShapeValidationAndCanonicalCopies(t *testing.T) {
	var nilSphere *Sphere
	var nilBox *Box
	for _, shape := range []Shape{nil, nilSphere, nilBox, Sphere{}, Sphere{Radius: -1}, Sphere{Radius: math.Inf(1)}, Box{HalfExtents: Vec3{X: 1, Y: 1, Z: 0}}, Box{HalfExtents: Vec3{X: 1, Y: math.NaN(), Z: 1}}} {
		if _, ok := CloneShape(shape); ok {
			t.Fatalf("accepted invalid shape: %#v", shape)
		}
	}
	for _, shape := range []Shape{Sphere{Radius: 1}, &Sphere{Radius: 1}, Box{HalfExtents: Vec3{X: 1, Y: 2, Z: 3}}, &Box{HalfExtents: Vec3{X: 1, Y: 2, Z: 3}}} {
		copy, ok := CloneShape(shape)
		if !ok {
			t.Fatalf("rejected valid shape: %#v", shape)
		}
		switch copy.(type) {
		case Sphere, Box:
		default:
			t.Fatalf("shape was not copied to an immutable value: %T", copy)
		}
	}
}

func collisionShapeVecNear(t *testing.T, got, want Vec3, tolerance float64) {
	t.Helper()
	if !validVec(got) || got.Sub(want).Length() > tolerance {
		t.Fatalf("got %v, want %v", got, want)
	}
}

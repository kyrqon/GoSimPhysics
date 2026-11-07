package physics3d

import (
	"math"
	"testing"
)

func TestShapePrincipalInertia(t *testing.T) {
	// A sphere has 2*m*r^2/5 about every axis. A cuboid has
	// m*(height^2+depth^2)/12 about its local X axis, and similarly for Y/Z.
	math3VecNear(t, "sphere inertia", (Sphere{Radius: 2}).inertia(5), Vec3{8, 8, 8}, 0)
	math3VecNear(t, "box inertia", (Box{HalfExtents: Vec3{1, 2, 3}}).inertia(12), Vec3{52, 40, 20}, 0)
}

func TestShapeValidationAndCanonicalCopies(t *testing.T) {
	var nilSphere *Sphere
	var nilBox *Box
	for _, shape := range []Shape{nil, nilSphere, nilBox, Sphere{}, Sphere{-1}, Sphere{math.Inf(1)}, Box{HalfExtents: Vec3{1, 1, 0}}, Box{HalfExtents: Vec3{1, math.NaN(), 1}}} {
		if _, ok := copyShape(shape); ok {
			t.Fatalf("accepted invalid shape: %#v", shape)
		}
	}
	for _, shape := range []Shape{Sphere{1}, &Sphere{1}, Box{Vec3{1, 2, 3}}, &Box{Vec3{1, 2, 3}}} {
		copy, ok := copyShape(shape)
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

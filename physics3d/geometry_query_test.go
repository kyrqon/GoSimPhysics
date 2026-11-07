package physics3d_test

import (
	"math"
	"reflect"
	"testing"

	"github.com/kyrqon/GoSimPhysics/physics3d"
)

func TestPublicGeometryQueriesWithoutDynamics(t *testing.T) {
	a := physics3d.Collider{Shape: physics3d.Sphere{Radius: 1}}
	b := physics3d.Collider{Shape: physics3d.Box{HalfExtents: physics3d.Vec3{X: 1, Y: 2, Z: 3}}, Position: physics3d.Vec3{X: 1.5}}
	c, hit, err := physics3d.CollideChecked(a, b)
	if err != nil || !hit || c.Penetration != 0.5 || c.Normal.X != 1 {
		t.Fatalf("zero rotations should mean identity: %+v, %v, %v", c, hit, err)
	}
	min, max, err := physics3d.Bounds(b)
	if err != nil || min != (physics3d.Vec3{X: 0.5, Y: -2, Z: -3}) || max != (physics3d.Vec3{X: 2.5, Y: 2, Z: 3}) {
		t.Fatalf("bounds = %v, %v, %v", min, max, err)
	}
	if tolerance := physics3d.DistanceTolerance(a, b); tolerance != 1e-9 {
		t.Fatalf("distance tolerance = %g", tolerance)
	}
	shape := &physics3d.Sphere{Radius: 2}
	cloned, ok := physics3d.CloneShape(shape)
	shape.Radius = -1
	if !ok || cloned != (physics3d.Sphere{Radius: 2}) {
		t.Fatalf("shape was not copied: %#v, %v", cloned, ok)
	}
}

func TestPublicMotionUsesConstantWorldAngularVelocity(t *testing.T) {
	m := physics3d.Motion{
		Collider: physics3d.Collider{
			Shape:    physics3d.Box{HalfExtents: physics3d.Vec3{X: 1, Y: 2, Z: 3}},
			Rotation: physics3d.QuaternionFromAxisAngle(physics3d.Vec3{X: 1}, math.Pi/2),
		},
		Velocity: physics3d.Vec3{X: 2}, AngularVelocity: physics3d.Vec3{Y: math.Pi / 2},
	}
	before := m
	pose, err := m.At(1)
	if err != nil || pose.Position != (physics3d.Vec3{X: 2}) {
		t.Fatalf("pose = %+v, %v", pose, err)
	}
	if got := pose.Rotation.Rotate(physics3d.Vec3{Y: 1}); got.Sub(physics3d.Vec3{X: 1}).Length() > 1e-14 {
		t.Fatalf("angular velocity did not act in world space: %v", got)
	}
	if !reflect.DeepEqual(m, before) {
		t.Fatal("pose query changed its motion")
	}
	zero := physics3d.Motion{Collider: physics3d.Collider{Shape: physics3d.Sphere{Radius: 1}}}
	if at, err := zero.At(0); err != nil || at.Rotation != physics3d.Identity() {
		t.Fatalf("zero pose query = %+v, %v", at, err)
	}
}

func TestPublicCheckedQueriesReportInvalidGeometry(t *testing.T) {
	valid := physics3d.Collider{Shape: physics3d.Sphere{Radius: 1}}
	var nilShape *physics3d.Sphere
	for _, invalid := range []physics3d.Collider{
		{}, {Shape: nilShape}, {Shape: physics3d.Sphere{Radius: -1}},
		{Shape: valid.Shape, Rotation: physics3d.Quaternion{W: math.NaN()}},
		{Shape: valid.Shape, Position: physics3d.Vec3{X: math.Inf(1)}},
		{Shape: physics3d.Sphere{Radius: math.MaxFloat64}, Position: physics3d.Vec3{X: math.MaxFloat64}},
	} {
		if _, _, err := physics3d.CollideChecked(valid, invalid); err == nil {
			t.Fatalf("collision accepted invalid collider: %+v", invalid)
		}
		if _, _, err := physics3d.Bounds(invalid); err == nil {
			t.Fatalf("bounds accepted invalid collider: %+v", invalid)
		}
		if _, hit := physics3d.Collide(valid, invalid); hit {
			t.Fatalf("boolean query accepted invalid collider: %+v", invalid)
		}
	}
	for _, time := range []float64{-1, math.NaN(), math.Inf(1)} {
		if _, err := (physics3d.Motion{Collider: valid}).At(time); err == nil {
			t.Fatalf("accepted invalid pose time: %g", time)
		}
	}
	motion := physics3d.Motion{Collider: valid, Velocity: physics3d.Vec3{X: math.MaxFloat64}}
	if _, _, err := physics3d.TimeOfImpact(motion, physics3d.Motion{Collider: valid}, 2); err == nil {
		t.Fatal("TOI hid endpoint overflow behind initial overlap")
	}
	if _, err := motion.At(2); err == nil {
		t.Fatal("pose query accepted endpoint overflow")
	}
	if physics3d.FeatureSize(nil) != 0 || physics3d.DistanceTolerance(physics3d.Collider{}, valid) != 1e-9 {
		t.Fatal("invalid feature query did not use its documented fallback")
	}
}

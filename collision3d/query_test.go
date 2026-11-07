package collision3d_test

import (
	"math"
	"reflect"
	"testing"

	"github.com/kyrqon/GoSimPhysics/collision3d"
)

func TestPublicGeometryQueriesWithoutDynamics(t *testing.T) {
	a := collision3d.Collider{Shape: collision3d.Sphere{Radius: 1}}
	b := collision3d.Collider{Shape: collision3d.Box{HalfExtents: collision3d.Vec3{X: 1, Y: 2, Z: 3}}, Position: collision3d.Vec3{X: 1.5}}
	c, hit, err := collision3d.CollideChecked(a, b)
	if err != nil || !hit || c.Penetration != 0.5 || c.Normal.X != 1 {
		t.Fatalf("zero rotations should mean identity: %+v, %v, %v", c, hit, err)
	}
	min, max, err := collision3d.Bounds(b)
	if err != nil || min != (collision3d.Vec3{X: 0.5, Y: -2, Z: -3}) || max != (collision3d.Vec3{X: 2.5, Y: 2, Z: 3}) {
		t.Fatalf("bounds = %v, %v, %v", min, max, err)
	}
	if tolerance := collision3d.DistanceTolerance(a, b); tolerance != 1e-9 {
		t.Fatalf("distance tolerance = %g", tolerance)
	}
	shape := &collision3d.Sphere{Radius: 2}
	cloned, ok := collision3d.CloneShape(shape)
	shape.Radius = -1
	if !ok || cloned != (collision3d.Sphere{Radius: 2}) {
		t.Fatalf("shape was not copied: %#v, %v", cloned, ok)
	}
}

func TestPublicMotionUsesConstantWorldAngularVelocity(t *testing.T) {
	m := collision3d.Motion{
		Collider: collision3d.Collider{
			Shape:    collision3d.Box{HalfExtents: collision3d.Vec3{X: 1, Y: 2, Z: 3}},
			Rotation: collision3d.QuaternionFromAxisAngle(collision3d.Vec3{X: 1}, math.Pi/2),
		},
		Velocity: collision3d.Vec3{X: 2}, AngularVelocity: collision3d.Vec3{Y: math.Pi / 2},
	}
	before := m
	pose, err := m.At(1)
	if err != nil || pose.Position != (collision3d.Vec3{X: 2}) {
		t.Fatalf("pose = %+v, %v", pose, err)
	}
	if got := pose.Rotation.Rotate(collision3d.Vec3{Y: 1}); got.Sub(collision3d.Vec3{X: 1}).Length() > 1e-14 {
		t.Fatalf("angular velocity did not act in world space: %v", got)
	}
	if !reflect.DeepEqual(m, before) {
		t.Fatal("pose query changed its motion")
	}
	zero := collision3d.Motion{Collider: collision3d.Collider{Shape: collision3d.Sphere{Radius: 1}}}
	if at, err := zero.At(0); err != nil || at.Rotation != collision3d.Identity() {
		t.Fatalf("zero pose query = %+v, %v", at, err)
	}
}

func TestPublicCheckedQueriesReportInvalidGeometry(t *testing.T) {
	valid := collision3d.Collider{Shape: collision3d.Sphere{Radius: 1}}
	var nilShape *collision3d.Sphere
	for _, invalid := range []collision3d.Collider{
		{}, {Shape: nilShape}, {Shape: collision3d.Sphere{Radius: -1}},
		{Shape: valid.Shape, Rotation: collision3d.Quaternion{W: math.NaN()}},
		{Shape: valid.Shape, Position: collision3d.Vec3{X: math.Inf(1)}},
		{Shape: collision3d.Sphere{Radius: math.MaxFloat64}, Position: collision3d.Vec3{X: math.MaxFloat64}},
	} {
		if _, _, err := collision3d.CollideChecked(valid, invalid); err == nil {
			t.Fatalf("collision accepted invalid collider: %+v", invalid)
		}
		if _, _, err := collision3d.Bounds(invalid); err == nil {
			t.Fatalf("bounds accepted invalid collider: %+v", invalid)
		}
		if _, hit := collision3d.Collide(valid, invalid); hit {
			t.Fatalf("boolean query accepted invalid collider: %+v", invalid)
		}
	}
	for _, time := range []float64{-1, math.NaN(), math.Inf(1)} {
		if _, err := (collision3d.Motion{Collider: valid}).At(time); err == nil {
			t.Fatalf("accepted invalid pose time: %g", time)
		}
	}
	motion := collision3d.Motion{Collider: valid, Velocity: collision3d.Vec3{X: math.MaxFloat64}}
	if _, _, err := collision3d.TimeOfImpact(motion, collision3d.Motion{Collider: valid}, 2); err == nil {
		t.Fatal("TOI hid endpoint overflow behind initial overlap")
	}
	if _, err := motion.At(2); err == nil {
		t.Fatal("pose query accepted endpoint overflow")
	}
	if collision3d.FeatureSize(nil) != 0 || collision3d.DistanceTolerance(collision3d.Collider{}, valid) != 1e-9 {
		t.Fatal("invalid feature query did not use its documented fallback")
	}
}

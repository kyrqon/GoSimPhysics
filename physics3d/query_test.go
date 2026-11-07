package physics3d

import (
	"math"
	"testing"
)

func bodyQueryTestBody(t *testing.T, def BodyDef) *Body {
	t.Helper()
	b, err := newBody(def)
	if err != nil {
		t.Fatal(err)
	}
	return b
}
func bodyQueryTestNear(t *testing.T, got, want, tolerance float64) {
	t.Helper()
	if !finite(got) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.15g, want %.15g within %g", got, want, tolerance)
	}
}
func bodyQueryMathVecNear(t *testing.T, name string, got, want Vec3, tolerance float64) {
	t.Helper()
	if !validVec(got) || got.Sub(want).Length() > tolerance {
		t.Fatalf("%s: got %v, want %v within %g", name, got, want, tolerance)
	}
}
func bodyQueryMathNear(t *testing.T, name string, got, want, tolerance float64) {
	t.Helper()
	if !finite(got) || math.Abs(got-want) > tolerance {
		t.Fatalf("%s: got %.15g, want %.15g within %g", name, got, want, tolerance)
	}
}

func TestTOIInitialTouchStaticMotionAndValidation(t *testing.T) {
	a := bodyQueryTestBody(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1, Position: Vec3{X: -2}, Velocity: Vec3{X: -100}})
	b := bodyQueryTestBody(t, BodyDef{Shape: Sphere{Radius: 1}})
	b.Velocity, b.AngularVelocity = Vec3{X: 1000}, Vec3{Y: 1000}
	result, hit, err := a.TimeOfImpact(b, 1)
	if err != nil || !hit || result.Time != 0 {
		t.Fatalf("separating initial touch = %v, %v, %v", result.Time, hit, err)
	}
	a.Position.X = -3
	if _, hit, err := a.TimeOfImpact(b, 1); err != nil || hit {
		t.Fatalf("static body's public velocity affected sweep: %v, %v", hit, err)
	}
	for _, dt := range []float64{0, -1, math.NaN(), math.Inf(1)} {
		if _, _, err := a.TimeOfImpact(b, dt); err == nil {
			t.Fatalf("accepted duration %v", dt)
		}
	}
	if _, _, err := ((*Body)(nil)).TimeOfImpact(b, 1); err == nil {
		t.Fatal("accepted nil body")
	}
	a.Velocity.X = math.MaxFloat64
	if _, _, err := a.TimeOfImpact(b, 2); err == nil {
		t.Fatal("accepted overflowing finite endpoint")
	}
}

func TestBodyQueriesKeepStaticAndFixedRotationPolicyInDynamics(t *testing.T) {
	obstacle := bodyQueryTestBody(t, BodyDef{Shape: Sphere{Radius: 0.05}, Position: Vec3{X: 1.4, Y: 1.4}})
	for _, def := range []BodyDef{
		{Shape: Box{HalfExtents: Vec3{X: 2, Y: 0.02, Z: 0.02}}},
		{Type: Dynamic, Mass: 1, Shape: Box{HalfExtents: Vec3{X: 2, Y: 0.02, Z: 0.02}}, FixedRotation: true},
	} {
		rod := bodyQueryTestBody(t, def)
		// Public velocity edits must still obey the body's motion policy.
		rod.AngularVelocity = Vec3{Z: math.Pi / 2}
		beforeRod, beforeObstacle := *rod, *obstacle
		if _, hit, err := rod.TimeOfImpact(obstacle, 1); err != nil || hit {
			t.Fatalf("disabled rotation affected sweep: %v, %v", hit, err)
		}
		pose, err := toiPose(rod, 1)
		if err != nil || pose.Rotation != rod.Rotation {
			t.Fatalf("disabled rotation affected pose: %+v, %v", pose, err)
		}
		if *rod != beforeRod || *obstacle != beforeObstacle {
			t.Fatal("body query changed input state")
		}
	}
	rod := bodyQueryTestBody(t, BodyDef{Type: Kinematic, Shape: Box{HalfExtents: Vec3{X: 2, Y: 0.02, Z: 0.02}}, AngularVelocity: Vec3{Z: math.Pi / 2}})
	if result, hit, err := rod.TimeOfImpact(obstacle, 1); err != nil || !hit || result.Time <= 0 || result.Time >= 1 {
		t.Fatalf("kinematic rotation was suppressed: %+v, %v, %v", result, hit, err)
	}
}

func TestBodyQueriesRejectMutableZeroRotation(t *testing.T) {
	a := bodyQueryTestBody(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1})
	b := bodyQueryTestBody(t, BodyDef{Shape: Sphere{Radius: 1}})
	a.Rotation = Quaternion{}
	if _, hit, err := collideChecked(a, b); err == nil || hit {
		t.Fatalf("collision accepted a body with zero rotation: %v, %v", hit, err)
	}
	if _, hit, err := a.TimeOfImpact(b, 1); err == nil || hit {
		t.Fatalf("sweep accepted a body with zero rotation: %v, %v", hit, err)
	}
	if _, err := toiPose(a, 0); err == nil {
		t.Fatal("pose accepted a body with zero rotation")
	}
}

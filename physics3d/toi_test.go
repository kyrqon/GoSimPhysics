package physics3d

import (
	"math"
	"reflect"
	"testing"
)

func toiTestBody(t *testing.T, def BodyDef) *Body {
	t.Helper()
	b, err := newBody(def)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func toiTestNear(t *testing.T, got, want, tolerance float64) {
	t.Helper()
	if !finite(got) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.15g, want %.15g within %g", got, want, tolerance)
	}
}

func TestTOISphereThinWallAnalyticAndPure(t *testing.T) {
	ball := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05}, Mass: 1, Position: Vec3{X: -2}, Velocity: Vec3{X: 400}})
	wall := toiTestBody(t, BodyDef{Shape: Box{HalfExtents: Vec3{X: 0.01, Y: 1, Z: 1}}})
	beforeBall, beforeWall := *ball, *wall
	result, hit, err := TimeOfImpact(ball, wall, 0.01)
	if err != nil || !hit {
		t.Fatalf("thin wall query = %+v, %v, %v", result, hit, err)
	}
	toiTestNear(t, result.Time, (2-0.05-0.01)/400, 1e-13)
	toiTestNear(t, result.Contact.Normal.X, 1, 1e-12)
	if len(result.Contact.Points) != 1 || !contactFinite(result.Contact) {
		t.Fatalf("invalid sphere-wall contact: %+v", result.Contact)
	}
	if boundary, hit, err := TimeOfImpact(ball, wall, result.Time); err != nil || !hit || math.Abs(boundary.Time-result.Time) > 1e-12 {
		t.Fatalf("contact at exact horizon was lost: %+v, %v, %v", boundary, hit, err)
	}
	if !reflect.DeepEqual(*ball, beforeBall) || !reflect.DeepEqual(*wall, beforeWall) {
		t.Fatal("TOI query mutated its bodies")
	}
	if _, hit, err := TimeOfImpact(ball, wall, 0.004); hit || err != nil {
		t.Fatalf("hit after requested horizon: %v, %v", hit, err)
	}
}

func TestTOIMovingSpheresAndTangent(t *testing.T) {
	a := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 1, Position: Vec3{X: -2}, Velocity: Vec3{X: 200}})
	b := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 3, Position: Vec3{X: 2}, Velocity: Vec3{X: -100}})
	result, hit, err := TimeOfImpact(a, b, 0.02)
	if err != nil || !hit {
		t.Fatalf("moving spheres = %v, %v", hit, err)
	}
	toiTestNear(t, result.Time, 3.8/300, 1e-13)
	a.Position, a.Velocity = Vec3{X: -2, Y: 0.2}, Vec3{X: 4}
	b.Position, b.Velocity = Vec3{}, Vec3{}
	result, hit, err = TimeOfImpact(a, b, 1)
	if err != nil || !hit {
		t.Fatalf("exact tangent spheres = %v, %v", hit, err)
	}
	toiTestNear(t, result.Time, 0.5, 1e-12)
	a.Position.Y = 0.20001
	if _, hit, err := TimeOfImpact(a, b, 1); err != nil || hit {
		t.Fatalf("separated tangent spheres hit: %v, %v", hit, err)
	}
}

func TestTOIRoundedBoxCornerDoesNotUseInflatedAABB(t *testing.T) {
	box := toiTestBody(t, BodyDef{Shape: Box{HalfExtents: Vec3{1, 1, 1}}})
	ball := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 1,
		Position: Vec3{-1.11, 1.08, 0}, Velocity: Vec3{0.03, 0.03, 0}})
	// Both path endpoints are outside the rounded corner, while the segment
	// crosses the corner of a naive axis-expanded box.
	if _, hit, err := TimeOfImpact(ball, box, 1); err != nil || hit {
		t.Fatalf("false inflated-box corner hit: %v, %v", hit, err)
	}
	ball.Position, ball.Velocity = Vec3{-2, 1.06, 0}, Vec3{X: 4}
	result, hit, err := TimeOfImpact(ball, box, 1)
	if err != nil || !hit {
		t.Fatalf("actual rounded edge query: %v, %v", hit, err)
	}
	// Corner distance: dx² + .06² = .1² gives dx=.08.
	toiTestNear(t, result.Time, (2-1-0.08)/4, 1e-12)
	toiTestNear(t, result.Contact.Normal.X, 0.8, 1e-10)
	toiTestNear(t, result.Contact.Normal.Y, -0.6, 1e-10)
	ball.Position, ball.Velocity = Vec3{-2, 1.1, 0}, Vec3{X: 4}
	result, hit, err = TimeOfImpact(ball, box, 1)
	if err != nil || !hit {
		t.Fatalf("tangent rounded edge query: %v, %v", hit, err)
	}
	toiTestNear(t, result.Time, 0.25, 1e-8)
	ball.Position, ball.Velocity = Vec3{-2, 1.06, 1.06}, Vec3{X: 4}
	result, hit, err = TimeOfImpact(ball, box, 1)
	if err != nil || !hit {
		t.Fatalf("actual rounded 3D corner query: %v, %v", hit, err)
	}
	toiTestNear(t, result.Time, (1-math.Sqrt(0.01-2*0.06*0.06))/4, 1e-12)
}

func TestTOIOrientedBoxesTranslationAndCovariance(t *testing.T) {
	a := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec3{0.1, 0.2, 0.3}}, Mass: 1, Position: Vec3{X: -2}, Velocity: Vec3{X: 400}})
	b := toiTestBody(t, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 1, 1}}})
	result, hit, err := TimeOfImpact(a, b, 0.01)
	if err != nil || !hit {
		t.Fatalf("box thin-wall sweep = %v, %v", hit, err)
	}
	toiTestNear(t, result.Time, (2-0.1-0.01)/400, 1e-13)
	if len(result.Contact.Points) < 2 {
		t.Fatalf("box face impact needs a distributed manifold: %+v", result.Contact)
	}
	q := QuaternionFromAxisAngle(Vec3{1, 2, 3}, 0.73)
	translation := Vec3{2, 3, -4}
	for _, body := range []*Body{a, b} {
		body.Position = q.Rotate(body.Position).Add(translation)
		body.Velocity = q.Rotate(body.Velocity)
		body.Rotation = q.Mul(body.Rotation)
	}
	moved, hit, err := TimeOfImpact(a, b, 0.01)
	if err != nil || !hit {
		t.Fatalf("rigid transform lost sweep: %v, %v", hit, err)
	}
	toiTestNear(t, moved.Time, result.Time, 1e-12)
	if moved.Contact.Normal.Sub(q.Rotate(result.Contact.Normal)).Length() > 1e-10 {
		t.Fatalf("normal did not rotate: %v", moved.Contact.Normal)
	}
	reverse, hit, err := TimeOfImpact(b, a, 0.01)
	if err != nil || !hit {
		t.Fatalf("swapped sweep lost contact: %v, %v", hit, err)
	}
	toiTestNear(t, reverse.Time, moved.Time, 1e-12)
	if reverse.Contact.Normal.Add(moved.Contact.Normal).Length() > 1e-10 {
		t.Fatal("swapping sweep bodies did not reverse normal")
	}
}

func TestTOIPureRotatingRodStartsAndEndsSeparated(t *testing.T) {
	rod := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec3{2, 0.02, 0.02}}, Mass: 1, AngularVelocity: Vec3{Z: math.Pi / 2}})
	ball := toiTestBody(t, BodyDef{Shape: Sphere{Radius: 0.05}, Position: Vec3{1.4, 1.4, 0}})
	if _, hit := Collide(rod, ball); hit {
		t.Fatal("rod starts in contact")
	}
	end, err := toiPose(rod, 1)
	if err != nil {
		t.Fatal(err)
	}
	if _, hit := Collide(end, ball); hit {
		t.Fatal("rod ends in contact")
	}
	result, hit, err := TimeOfImpact(rod, ball, 1)
	if err != nil || !hit {
		t.Fatalf("rotational sweep = %+v, %v, %v", result, hit, err)
	}
	exact := (math.Pi/4 - math.Asin(0.07/(1.4*math.Sqrt2))) / (math.Pi / 2)
	toiTestNear(t, result.Time, exact, 1e-9)
	if !contactFinite(result.Contact) {
		t.Fatal("rotational sweep returned invalid contact")
	}
}

func TestTOIRotatingBoxAgainstBoxAnalyticalFaceTime(t *testing.T) {
	rod := toiTestBody(t, BodyDef{Type: Kinematic, Shape: Box{HalfExtents: Vec3{2, 0.02, 0.02}}, AngularVelocity: Vec3{Z: math.Pi / 2}})
	obstacle := toiTestBody(t, BodyDef{Shape: Box{HalfExtents: Vec3{0.05, 0.05, 0.05}}, Position: Vec3{1.4, 1.4, 0}})
	result, hit, err := TimeOfImpact(rod, obstacle, 1)
	if err != nil || !hit {
		t.Fatalf("rotating box sweep = %+v, %v, %v", result, hit, err)
	}
	// The target's near corner reaches the rod side when
	// 1.35*cos(theta)-1.45*sin(theta)=0.02; all other SAT axes overlap.
	angle := math.Acos(0.02/math.Hypot(1.35, 1.45)) - math.Atan2(1.45, 1.35)
	toiTestNear(t, result.Time, angle/(math.Pi/2), 1e-9)
}

func TestTOIInitialTouchStaticMotionAndValidation(t *testing.T) {
	a := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1, Position: Vec3{X: -2}, Velocity: Vec3{X: -100}})
	b := toiTestBody(t, BodyDef{Shape: Sphere{Radius: 1}})
	b.Velocity, b.AngularVelocity = Vec3{X: 1000}, Vec3{Y: 1000}
	result, hit, err := TimeOfImpact(a, b, 1)
	if err != nil || !hit || result.Time != 0 {
		t.Fatalf("separating initial touch = %v, %v, %v", result.Time, hit, err)
	}
	a.Position.X = -3
	if _, hit, err := TimeOfImpact(a, b, 1); err != nil || hit {
		t.Fatalf("static body's public velocity affected sweep: %v, %v", hit, err)
	}
	for _, dt := range []float64{0, -1, math.NaN(), math.Inf(1)} {
		if _, _, err := TimeOfImpact(a, b, dt); err == nil {
			t.Fatalf("accepted duration %v", dt)
		}
	}
	if _, _, err := TimeOfImpact(nil, b, 1); err == nil {
		t.Fatal("accepted nil body")
	}
	a.Velocity.X = math.MaxFloat64
	if _, _, err := TimeOfImpact(a, b, 2); err == nil {
		t.Fatal("accepted overflowing finite endpoint")
	}
}

func TestTOISlopCornerUsesActualClosestFeatures(t *testing.T) {
	a := toiTestBody(t, BodyDef{Shape: Box{HalfExtents: Vec3{1, 1, 1}}})
	b := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec3{1, 1, 1}}, Mass: 1,
		Position: Vec3{2 + 5e-10, 2 + 5e-10, 0}, Velocity: Vec3{-1, -1, 0}})
	result, hit, err := TimeOfImpact(a, b, 1)
	if err != nil || !hit || result.Time != 0 || !contactFinite(result.Contact) {
		t.Fatalf("near-touch corner = %+v, %v, %v", result, hit, err)
	}
	toiTestNear(t, result.Contact.Normal.X, math.Sqrt(0.5), 1e-8)
	toiTestNear(t, result.Contact.Normal.Y, math.Sqrt(0.5), 1e-8)
}

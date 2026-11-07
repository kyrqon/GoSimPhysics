package collision2d

import (
	"math"
	"reflect"
	"testing"
)

func TestPublicCheckedQueries(t *testing.T) {
	a := Collider{Shape: Circle{Radius: 1}}
	b := Collider{Shape: Box{HalfExtents: Vec2{X: 1, Y: 1}}, Position: Vec2{X: 4}}
	if _, hit, err := CollideChecked(a, b); err != nil || hit {
		t.Fatalf("ordinary miss returned error or hit: %v %v", hit, err)
	}
	b.Position.X = 1.5
	contact, hit, err := CollideChecked(a, b)
	if err != nil || !hit || !contactFinite(contact) {
		t.Fatalf("valid public contact failed: %+v %v %v", contact, hit, err)
	}
	for _, invalid := range []Collider{{}, {Shape: (*Circle)(nil)}, {Shape: Circle{Radius: -1}}, {Shape: Circle{Radius: 1}, Position: Vec2{X: math.NaN()}}, {Shape: Circle{Radius: 1}, Angle: math.Inf(1)}} {
		if _, hit, err := CollideChecked(a, invalid); err == nil || hit {
			t.Fatalf("invalid collider accepted: %+v", invalid)
		}
		if _, hit := Collide(a, invalid); hit {
			t.Fatal("unchecked query returned invalid contact")
		}
	}
	if _, hit, err := CollideChecked(Collider{Shape: Circle{Radius: 1e308}, Position: Vec2{X: 1e308}}, a); err == nil || hit {
		t.Fatal("individual overflowing bounds became an ordinary miss")
	}
	huge := Collider{Shape: Circle{Radius: 1e308}}
	if _, hit, err := CollideChecked(huge, huge); err == nil || hit {
		t.Fatal("pair overflow became an ordinary miss")
	}
}

func TestPublicBoundsAndFeatureSize(t *testing.T) {
	c := Collider{Shape: &Box{HalfExtents: Vec2{X: 2, Y: 1}}, Position: Vec2{X: 3, Y: 4}, Angle: math.Pi / 2}
	min, max, err := Bounds(c)
	if err != nil {
		t.Fatal(err)
	}
	collisionCloseVec(t, min, Vec2{X: 2, Y: 2}, 1e-12)
	collisionCloseVec(t, max, Vec2{X: 4, Y: 6}, 1e-12)
	collisionClose(t, FeatureSize(c.Shape), 1, 0)
	collisionClose(t, DistanceTolerance(c, Collider{Shape: &Circle{Radius: 0.01}}), 1e-10, 1e-24)
	if DistanceTolerance(c, Collider{}) != toiTolerance {
		t.Fatal("invalid shape pair must use default sweep tolerance")
	}
	if FeatureSize(nil) != 0 {
		t.Fatal("invalid shape feature must be zero")
	}
	for _, bad := range []Collider{{}, {Shape: Circle{Radius: 1}, Position: Vec2{X: math.Inf(1)}}, {Shape: Circle{Radius: math.MaxFloat64}, Position: Vec2{X: math.MaxFloat64}}} {
		if _, _, err := Bounds(bad); err == nil {
			t.Fatalf("invalid bounds accepted: %+v", bad)
		}
	}
}

func TestPublicMotionAtValidationAndCopy(t *testing.T) {
	shape := &Box{HalfExtents: Vec2{X: 2, Y: 1}}
	motion := Motion{Collider: Collider{Shape: shape, Position: Vec2{X: 1, Y: 2}, Angle: 0.1}, Velocity: Vec2{X: 2, Y: -4}, AngularVelocity: 0.5}
	before := motion
	pose, err := motion.At(2)
	if err != nil {
		t.Fatal(err)
	}
	collisionCloseVec(t, pose.Position, Vec2{X: 5, Y: -6}, 0)
	collisionClose(t, pose.Angle, 1.1, 1e-15)
	if !reflect.DeepEqual(motion, before) {
		t.Fatal("At mutated motion")
	}
	shape.HalfExtents.X = 100
	if pose.Shape.(Box).HalfExtents.X != 2 {
		t.Fatal("pose retained caller shape pointer")
	}
	for _, time := range []float64{-1, math.NaN(), math.Inf(1)} {
		if _, err := motion.At(time); err == nil {
			t.Fatalf("invalid time accepted: %g", time)
		}
	}
	bad := motion
	bad.Velocity.X = math.Inf(1)
	if _, err := bad.At(0); err == nil {
		t.Fatal("nonfinite motion accepted at time zero")
	}
	bad = motion
	bad.Position.X = math.MaxFloat64
	bad.Velocity.X = math.MaxFloat64
	if _, err := bad.At(2); err == nil {
		t.Fatal("unrepresentable endpoint accepted")
	}
	// A time-zero overlap cannot hide an overflowing endpoint of a sweep.
	bad = Motion{Collider: Collider{Shape: Circle{Radius: 1}}, Velocity: Vec2{X: math.MaxFloat64}}
	if _, hit, err := TimeOfImpact(bad, Motion{Collider: bad.Collider}, 2); err == nil || hit {
		t.Fatal("initial contact hid overflowing complete motion")
	}
}

func TestCloneShapeCopiesPointers(t *testing.T) {
	circle := &Circle{Radius: 2}
	copied, ok := CloneShape(circle)
	if !ok {
		t.Fatal("valid pointer shape rejected")
	}
	circle.Radius = 9
	if copied.(Circle).Radius != 2 {
		t.Fatal("clone retained pointer shape")
	}
	box := &Box{HalfExtents: Vec2{X: 3, Y: 4}}
	copied, ok = CloneShape(box)
	if !ok {
		t.Fatal("valid box pointer rejected")
	}
	box.HalfExtents.X = 9
	if copied.(Box).HalfExtents.X != 3 {
		t.Fatal("clone retained box pointer")
	}
	for _, shape := range []Shape{nil, (*Circle)(nil), (*Box)(nil), Circle{}, Box{}} {
		if _, ok := CloneShape(shape); ok {
			t.Fatalf("invalid clone accepted: %#v", shape)
		}
	}
}

// Pose bounds use actual oriented geometry, not an overflowing circumradius.
func TestPublicMotionAtUsesActualBounds(t *testing.T) {
	m := Motion{Collider: Collider{Shape: Box{HalfExtents: Vec2{X: 1.4e308, Y: 1.4e308}}}}
	if _, err := m.At(1); err != nil {
		t.Fatalf("finite actual bounds rejected: %v", err)
	}
}

package physics2d

import (
	"math"
	"testing"
)

func jointTestBody(t *testing.T, world *World, def BodyDef) *Body {
	t.Helper()
	body, err := world.CreateBody(def)
	if err != nil {
		t.Fatalf("create joint test body: %v", err)
	}
	return body
}

func jointTestNear(t *testing.T, label string, got, want, tolerance float64) {
	t.Helper()
	if math.IsNaN(got) || math.IsInf(got, 0) || math.Abs(got-want) > tolerance {
		t.Errorf("%s = %.12g, want %.12g within %.3g", label, got, want, tolerance)
	}
}

func TestDistanceJointCenterVelocity(t *testing.T) {
	world := NewWorld(Vec2{})
	a := jointTestBody(t, world, BodyDef{
		Type: Dynamic, Shape: Circle{Radius: 0.25}, Mass: 1,
		Velocity: Vec2{X: -1, Y: 2},
	})
	b := jointTestBody(t, world, BodyDef{
		Type: Dynamic, Shape: Circle{Radius: 0.25}, Mass: 1,
		Position: Vec2{X: 2}, Velocity: Vec2{X: 3, Y: -2},
	})
	joint, err := world.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	joint.prepare(1.0 / 60)
	joint.solveVelocity()

	// Equal masses share their radial momentum; the tangential velocities stay free.
	jointTestNear(t, "a vx", a.Velocity.X, 1, 1e-12)
	jointTestNear(t, "b vx", b.Velocity.X, 1, 1e-12)
	jointTestNear(t, "a vy", a.Velocity.Y, 2, 1e-12)
	jointTestNear(t, "b vy", b.Velocity.Y, -2, 1e-12)
	jointTestNear(t, "a angular velocity", a.AngularVelocity, 0, 1e-12)
	jointTestNear(t, "b angular velocity", b.AngularVelocity, 0, 1e-12)
	if joint.BodyA() != a || joint.BodyB() != b || joint.Length() != 2 {
		t.Error("joint getters do not match its definition")
	}
	anchorA, anchorB := joint.Anchors()
	if anchorA != a.Position || anchorB != b.Position {
		t.Errorf("center anchors = %v, %v; want %v, %v", anchorA, anchorB, a.Position, b.Position)
	}
}

func TestDistanceJointOffsetAnchorEffectiveMass(t *testing.T) {
	world := NewWorld(Vec2{})
	a := jointTestBody(t, world, BodyDef{Type: Static, Shape: Circle{Radius: 0.1}})
	b := jointTestBody(t, world, BodyDef{
		Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 2,
		Position: Vec2{X: 2, Y: -1}, Velocity: Vec2{X: 3},
	})
	joint, err := world.CreateDistanceJoint(DistanceJointDef{
		BodyA: a, BodyB: b, LocalAnchorB: Vec2{Y: 1}, Length: 2,
	})
	if err != nil {
		t.Fatal(err)
	}
	joint.prepare(1.0 / 60)
	joint.solveVelocity()

	// I = mr^2/2 = 1 and effective inverse mass = 1/2 + 1 = 3/2.
	// The impulse is -3/(3/2) = -2, giving dv=-1 and dw=+2.
	jointTestNear(t, "center velocity", b.Velocity.X, 2, 1e-12)
	jointTestNear(t, "angular velocity", b.AngularVelocity, 2, 1e-12)
	jointTestNear(t, "anchor radial velocity", b.Velocity.X-b.AngularVelocity, 0, 1e-12)
	if a.Position != (Vec2{}) || a.Velocity != (Vec2{}) || a.AngularVelocity != 0 {
		t.Error("static anchor moved")
	}
}

func TestDistanceJointPendulumLength(t *testing.T) {
	world := NewWorld(Vec2{Y: -10})
	a := jointTestBody(t, world, BodyDef{Type: Static, Shape: Circle{Radius: 0.05}})
	b := jointTestBody(t, world, BodyDef{
		Type: Dynamic, Shape: Circle{Radius: 0.1}, Mass: 1,
		Position: Vec2{X: 1.2, Y: -1.6},
	})
	joint, err := world.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	maxError := 0.0
	for frame := 0; frame < 1200; frame++ {
		if err := world.Step(1.0 / 60); err != nil {
			t.Fatalf("frame %d: %v", frame, err)
		}
		pa, pb := joint.Anchors()
		distance := pb.Sub(pa).Length()
		if math.IsNaN(distance) || math.IsInf(distance, 0) {
			t.Fatalf("frame %d: non-finite anchor distance %v", frame, distance)
		}
		maxError = math.Max(maxError, math.Abs(distance-2))
	}
	if maxError > 0.01 {
		t.Errorf("20-second pendulum maximum length error = %g, want <= 0.01", maxError)
	}
	if a.Position != (Vec2{}) || a.Angle != 0 {
		t.Errorf("static pendulum anchor changed: position %v, angle %v", a.Position, a.Angle)
	}
}

func TestDistanceJointCoincidentAnchors(t *testing.T) {
	world := NewWorld(Vec2{})
	a := jointTestBody(t, world, BodyDef{Type: Static, Shape: Circle{Radius: 0.1}})
	b := jointTestBody(t, world, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.1}, Mass: 1})
	joint, err := world.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	for frame := 0; frame < 300; frame++ {
		if err := world.Step(1.0 / 60); err != nil {
			t.Fatalf("frame %d: %v", frame, err)
		}
		if !validVec(b.Position) || !validVec(b.Velocity) || !finite(b.AngularVelocity) {
			t.Fatalf("frame %d: coincident anchors produced non-finite state", frame)
		}
	}
	pa, pb := joint.Anchors()
	jointTestNear(t, "settled anchor distance", pb.Sub(pa).Length(), 2, 0.01)
}

func TestDistanceJointValidationAndRemoval(t *testing.T) {
	world := NewWorld(Vec2{})
	a := jointTestBody(t, world, BodyDef{Type: Static, Shape: Circle{Radius: 0.1}})
	b := jointTestBody(t, world, BodyDef{
		Type: Dynamic, Shape: Circle{Radius: 0.1}, Mass: 1, Position: Vec2{X: 2},
	})
	staticB := jointTestBody(t, world, BodyDef{Type: Static, Shape: Circle{Radius: 0.1}})
	foreignWorld := NewWorld(Vec2{})
	foreign := jointTestBody(t, foreignWorld, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.1}, Mass: 1})
	base := DistanceJointDef{BodyA: a, BodyB: b, Length: 2}
	cases := []struct {
		name   string
		change func(*DistanceJointDef)
	}{
		{"nil A", func(def *DistanceJointDef) { def.BodyA = nil }},
		{"nil B", func(def *DistanceJointDef) { def.BodyB = nil }},
		{"same endpoint", func(def *DistanceJointDef) { def.BodyB = a }},
		{"foreign endpoint", func(def *DistanceJointDef) { def.BodyB = foreign }},
		{"both static", func(def *DistanceJointDef) { def.BodyB = staticB }},
		{"zero length", func(def *DistanceJointDef) { def.Length = 0 }},
		{"negative length", func(def *DistanceJointDef) { def.Length = -1 }},
		{"infinite length", func(def *DistanceJointDef) { def.Length = math.Inf(1) }},
		{"NaN length", func(def *DistanceJointDef) { def.Length = math.NaN() }},
		{"invalid anchor A", func(def *DistanceJointDef) { def.LocalAnchorA.X = math.NaN() }},
		{"invalid anchor B", func(def *DistanceJointDef) { def.LocalAnchorB.Y = math.Inf(1) }},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			def := base
			tc.change(&def)
			if joint, err := world.CreateDistanceJoint(def); err == nil || joint != nil {
				t.Errorf("CreateDistanceJoint(%+v) = %v, %v; want nil, error", def, joint, err)
			}
		})
	}

	joint, err := world.CreateDistanceJoint(base)
	if err != nil {
		t.Fatal(err)
	}
	if world.RemoveDistanceJoint(nil) || foreignWorld.RemoveDistanceJoint(joint) {
		t.Error("removed a joint not owned by the world")
	}
	if !world.RemoveDistanceJoint(joint) || world.RemoveDistanceJoint(joint) {
		t.Error("joint removal should succeed once")
	}
	b.Velocity = Vec2{X: 1}
	if err := world.Step(1.0 / 60); err != nil {
		t.Fatal(err)
	}
	jointTestNear(t, "unconstrained velocity after removal", b.Velocity.X, 1, 1e-12)
}

func TestDistanceJointRejectsDerivedGeometryOverflow(t *testing.T) {
	t.Run("finite anchors but overflowing separation", func(t *testing.T) {
		world := NewWorld(Vec2{})
		a := jointTestBody(t, world, BodyDef{
			Type: Static, Shape: Circle{Radius: 1}, Position: Vec2{X: -math.MaxFloat64},
		})
		b := jointTestBody(t, world, BodyDef{
			Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 1, Position: Vec2{X: math.MaxFloat64},
		})
		if joint, err := world.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 1}); err == nil || joint != nil {
			t.Errorf("overflowing anchor separation accepted: joint %v, error %v", joint, err)
		}
		if len(world.joints) != 0 {
			t.Error("rejected joint changed world membership")
		}
	})

	t.Run("finite anchors but overflowing angular effective mass", func(t *testing.T) {
		world := NewWorld(Vec2{})
		a := jointTestBody(t, world, BodyDef{
			Type: Static, Shape: Circle{Radius: 1},
			Position: Vec2{X: math.MaxFloat64 / 2, Y: -math.MaxFloat64 / 2},
		})
		b := jointTestBody(t, world, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 2})
		def := DistanceJointDef{
			BodyA: a, BodyB: b, LocalAnchorB: Vec2{X: math.MaxFloat64 / 2}, Length: 1,
		}
		if joint, err := world.CreateDistanceJoint(def); err == nil || joint != nil {
			t.Errorf("overflowing angular effective mass accepted: joint %v, error %v", joint, err)
		}
	})
}

func TestDistanceJointGeometryOverflowRollsBackStep(t *testing.T) {
	world := NewWorld(Vec2{})
	a := jointTestBody(t, world, BodyDef{Type: Static, Shape: Circle{Radius: 1}})
	b := jointTestBody(t, world, BodyDef{
		Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 1, Position: Vec2{X: 2},
	})
	joint, err := world.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	// Teleporting the public state preserves finite individual coordinates, but
	// the joint's anchor difference no longer fits in a float64.
	a.Position.X, b.Position.X = -math.MaxFloat64, math.MaxFloat64
	if err := b.ApplyForce(Vec2{X: 1}); err != nil {
		t.Fatal(err)
	}
	beforeA, beforeB := *a, *b
	if joint.validGeometry() {
		t.Fatal("overflowing anchor difference reported as valid geometry")
	}
	if err := world.Step(1.0 / 60); err == nil {
		t.Fatal("Step accepted overflowing joint geometry")
	}
	if *a != beforeA || *b != beforeB {
		t.Error("failed Step changed body state or consumed applied force")
	}
}

func TestDistanceJointStaticLeverDoesNotComputeUnusedTorque(t *testing.T) {
	world := NewWorld(Vec2{})
	a := jointTestBody(t, world, BodyDef{
		Type: Static, Shape: Circle{Radius: 1}, Position: Vec2{X: -1e154},
	})
	b := jointTestBody(t, world, BodyDef{
		Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 1e308, Position: Vec2{Y: 2},
	})
	joint, err := world.CreateDistanceJoint(DistanceJointDef{
		BodyA: a, BodyB: b, LocalAnchorA: Vec2{X: 1e154}, Length: 1,
	})
	if err != nil {
		t.Fatal(err)
	}
	joint.solvePosition()
	if a.Angle != 0 || !a.validState() || !b.validState() {
		t.Errorf("position correction computed a non-finite unused static torque: a=%+v b=%+v", a, b)
	}
	if b.Position.Y >= 2 {
		t.Error("dynamic endpoint did not move toward its target distance")
	}
}

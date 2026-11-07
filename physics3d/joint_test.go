package physics3d

import (
	"math"
	"testing"
)

func TestDistanceJointSharesRadialMomentum(t *testing.T) {
	w := NewWorld(Vec3{})
	a := solverBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 1, Velocity: Vec3{X: -1, Y: 2, Z: 3}})
	b := solverBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 1, Position: Vec3{X: 2}, Velocity: Vec3{X: 3, Y: -2, Z: -3}})
	j, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	j.prepare(1.0 / 60)
	j.solveVelocity()
	solverVecNear(t, a.Velocity, Vec3{X: 1, Y: 2, Z: 3}, 1e-12)
	solverVecNear(t, b.Velocity, Vec3{X: 1, Y: -2, Z: -3}, 1e-12)
	if j.BodyA() != a || j.BodyB() != b || j.Length() != 2 {
		t.Fatal("joint getters differ from definition")
	}
	pa, pb := j.Anchors()
	if pa != a.Position || pb != b.Position {
		t.Fatal("center anchors differ from body centers")
	}
}

func TestDistanceJointUsesRotatedAnisotropicInertia(t *testing.T) {
	w := NewWorld(Vec3{})
	a := solverBody(t, w, BodyDef{Shape: Sphere{Radius: 0.1}})
	b := solverBody(t, w, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec3{X: 1, Y: 2, Z: 3}}, Mass: 3,
		Position: Vec3{X: 2, Y: -1, Z: -1}, Velocity: Vec3{X: 3}, Rotation: QuaternionFromAxisAngle(Vec3{Z: 1}, math.Pi/2)})
	j, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, LocalAnchorB: Vec3{X: 1, Y: 0, Z: 1}, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	j.prepare(1.0 / 60)
	j.solveVelocity()
	// World principal inertia after a quarter turn is (10,13,5). The anchor
	// lever is (0,1,1), giving K=1/3+1/13+1/5=119/195 and J=-585/119.
	solverVecNear(t, b.Velocity, Vec3{X: 162.0 / 119}, 1e-12)
	solverVecNear(t, b.AngularVelocity, Vec3{Y: -45.0 / 119, Z: 117.0 / 119}, 1e-12)
	pa, pb := j.Anchors()
	anchorVelocity := b.Velocity.Add(b.AngularVelocity.Cross(pb.Sub(b.Position)))
	solverNear(t, anchorVelocity.Dot(pb.Sub(pa).Normalized()), 0, 1e-12)
}

func TestDistanceJointThreeDimensionalPendulum(t *testing.T) {
	w := NewWorld(Vec3{Y: -9.81})
	a := solverBody(t, w, BodyDef{Shape: Sphere{Radius: 0.05}})
	b := solverBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 1,
		Position: Vec3{X: 1.2, Y: -1.6, Z: 0}, Velocity: Vec3{Z: 0.8}})
	j, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	maxError := 0.0
	for i := 0; i < 1200; i++ {
		solverStep(t, w, 1.0/60)
		pa, pb := j.Anchors()
		maxError = math.Max(maxError, math.Abs(pb.Sub(pa).Length()-2))
	}
	if maxError > 0.01 {
		t.Fatalf("20-second spherical pendulum length error %g", maxError)
	}
	if a.Position != (Vec3{}) || a.Rotation != Identity() {
		t.Fatal("static anchor moved")
	}
}

func TestDistanceJointCoincidentAnchorsRemainFinite(t *testing.T) {
	w := NewWorld(Vec3{})
	a := solverBody(t, w, BodyDef{Shape: Sphere{Radius: 0.1}})
	b := solverBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 1})
	j, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 300; i++ {
		solverStep(t, w, 1.0/60)
	}
	pa, pb := j.Anchors()
	solverNear(t, pb.Sub(pa).Length(), 2, 0.01)
}

func TestDistanceJointValidationAndRemoval(t *testing.T) {
	w := NewWorld(Vec3{})
	a := solverBody(t, w, BodyDef{Shape: Sphere{Radius: 0.1}})
	b := solverBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 1, Position: Vec3{X: 2}})
	c := solverBody(t, w, BodyDef{Shape: Sphere{Radius: 0.1}, Position: Vec3{X: 4}})
	foreignWorld := NewWorld(Vec3{})
	foreign := solverBody(t, foreignWorld, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 1})
	base := DistanceJointDef{BodyA: a, BodyB: b, Length: 2}
	cases := []struct {
		name   string
		change func(*DistanceJointDef)
	}{
		{"nil endpoint", func(d *DistanceJointDef) { d.BodyA = nil }},
		{"same endpoint", func(d *DistanceJointDef) { d.BodyB = a }},
		{"foreign endpoint", func(d *DistanceJointDef) { d.BodyB = foreign }},
		{"static endpoints", func(d *DistanceJointDef) { d.BodyB = c }},
		{"zero length", func(d *DistanceJointDef) { d.Length = 0 }},
		{"negative length", func(d *DistanceJointDef) { d.Length = -1 }},
		{"infinite length", func(d *DistanceJointDef) { d.Length = math.Inf(1) }},
		{"nan anchor", func(d *DistanceJointDef) { d.LocalAnchorB.Z = math.NaN() }},
		{"infinite anchor", func(d *DistanceJointDef) { d.LocalAnchorA.X = math.Inf(1) }},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			def := base
			tc.change(&def)
			if joint, err := w.CreateDistanceJoint(def); err == nil || joint != nil {
				t.Fatal("accepted invalid joint")
			}
		})
	}
	j, err := w.CreateDistanceJoint(base)
	if err != nil {
		t.Fatal(err)
	}
	if w.RemoveDistanceJoint(nil) || foreignWorld.RemoveDistanceJoint(j) {
		t.Fatal("removed a joint not owned by the world")
	}
	if !w.RemoveDistanceJoint(j) || w.RemoveDistanceJoint(j) {
		t.Fatal("joint removal did not succeed exactly once")
	}
	b.Velocity.X = 1
	solverStep(t, w, 0.1)
	solverNear(t, b.Velocity.X, 1, 1e-12)
}

func TestDistanceJointRejectsDerivedOverflowAndRollsBackAfterTeleport(t *testing.T) {
	w := NewWorld(Vec3{})
	a := solverBody(t, w, BodyDef{Shape: Sphere{Radius: 1}})
	b := solverBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1, Position: Vec3{X: 2}})
	j, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	a.Position.X, b.Position.X = -math.MaxFloat64, math.MaxFloat64
	if err := b.ApplyForce(Vec3{Y: 1}); err != nil {
		t.Fatal(err)
	}
	beforeA, beforeB := *a, *b
	if j.validGeometry() {
		t.Fatal("overflowing anchor separation reported as valid")
	}
	if err := w.Step(0.01); err == nil {
		t.Fatal("accepted overflowing joint geometry")
	}
	if *a != beforeA || *b != beforeB {
		t.Fatal("failed Step changed state or consumed force")
	}
	if joint, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 2}); err == nil || joint != nil {
		t.Fatal("created a joint with overflowing derived geometry")
	}
}

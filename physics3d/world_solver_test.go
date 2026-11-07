package physics3d

import (
	"math"
	"testing"
)

func solverBody(t testing.TB, w *World, def BodyDef) *Body {
	t.Helper()
	b, err := w.CreateBody(def)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func solverStep(t testing.TB, w *World, dt float64) {
	t.Helper()
	if err := w.Step(dt); err != nil {
		t.Fatal(err)
	}
}

func solverNear(t testing.TB, got, want, tolerance float64) {
	t.Helper()
	if !finite(got) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.15g, want %.15g within %.3g", got, want, tolerance)
	}
}

func solverVecNear(t testing.TB, got, want Vec3, tolerance float64) {
	t.Helper()
	if !validVec(got) || got.Sub(want).Length() > tolerance {
		t.Fatalf("got %v, want %v within %.3g", got, want, tolerance)
	}
}

func TestSolverTorqueFreeAnisotropicBoxPreservesWorldMomentum(t *testing.T) {
	w := NewWorld(Vec3{})
	b := solverBody(t, w, BodyDef{
		Type: Dynamic, Shape: Box{HalfExtents: Vec3{1, 2, 3}}, Mass: 3,
		Rotation:        QuaternionFromAxisAngle(Vec3{1, 2, -1}, 0.7),
		AngularVelocity: Vec3{1.2, -0.7, 0.9},
	})
	initialMomentum := b.AngularMomentum()
	initialEnergy := 0.5 * b.AngularVelocity.Dot(initialMomentum)
	initialOmega := b.AngularVelocity
	for i := 0; i < 2400; i++ {
		solverStep(t, w, 1.0/240)
		solverVecNear(t, b.AngularMomentum(), initialMomentum, 2e-9)
		energy := 0.5 * b.AngularVelocity.Dot(b.AngularMomentum())
		if math.Abs(energy-initialEnergy) > initialEnergy*0.001 {
			t.Fatalf("free rotation energy drift: %g -> %g", initialEnergy, energy)
		}
		solverNear(t, b.Rotation.X*b.Rotation.X+b.Rotation.Y*b.Rotation.Y+b.Rotation.Z*b.Rotation.Z+b.Rotation.W*b.Rotation.W, 1, 1e-12)
	}
	if b.AngularVelocity.Sub(initialOmega).Length() < 0.05 {
		t.Fatal("anisotropic free body incorrectly kept world angular velocity constant")
	}
}

func TestSolverTorqueIntegratesWorldAngularMomentum(t *testing.T) {
	w := NewWorld(Vec3{})
	b := solverBody(t, w, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec3{1, 2, 3}}, Mass: 3,
		AngularVelocity: Vec3{0.8, 0.4, -0.3}, Rotation: QuaternionFromAxisAngle(Vec3{1, 1, 0}, 0.5)})
	initial := b.AngularMomentum()
	torque := Vec3{0.7, -0.3, 0.5}
	for i := 0; i < 1000; i++ {
		if err := b.ApplyTorque(torque); err != nil {
			t.Fatal(err)
		}
		solverStep(t, w, 0.001)
	}
	solverVecNear(t, b.AngularMomentum(), initial.Add(torque), 1e-9)
}

func TestSolverFrictionClampsToDiskNotSquare(t *testing.T) {
	w := NewWorld(Vec3{})
	a := solverBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{5, 0.5, 5}}, Friction: 0.5})
	b := solverBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1,
		Position: Vec3{Y: 1}, Velocity: Vec3{3, -1, 4}, Friction: 0.5, FixedRotation: true})
	constraint, err := newContactConstraint(a, b, Contact{Normal: Vec3{Y: 1}, Points: []Vec3{{}}}, 1)
	if err != nil {
		t.Fatal(err)
	}
	constraint.solveVelocity()
	// Jn=1 and mu=0.5. Coulomb friction has magnitude 0.5 in any sliding
	// direction, including this 3:4 diagonal in the two-dimensional tangent plane.
	solverVecNear(t, b.Velocity, Vec3{2.7, 0, 3.6}, 1e-12)
	solverNear(t, math.Hypot(3-b.Velocity.X, 4-b.Velocity.Z), 0.5, 1e-12)
	for i := 0; i < 20; i++ {
		constraint.solveVelocity()
	}
	solverVecNear(t, b.Velocity, Vec3{2.7, 0, 3.6}, 1e-12)
}

func TestSolverCoupledTangentMassStopsAnisotropicContactSlip(t *testing.T) {
	w := NewWorld(Vec3{})
	a := solverBody(t, w, BodyDef{Shape: Sphere{Radius: 1}, Friction: 100})
	b := solverBody(t, w, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec3{1, 2, 3}}, Mass: 2,
		Position: Vec3{Y: 2}, Velocity: Vec3{0.8, -2, 0.7}, AngularVelocity: Vec3{-0.2, 0.3, -0.4},
		Rotation: QuaternionFromAxisAngle(Vec3{1, 1, 1}, 0.7), Friction: 100})
	point := Vec3{0.4, 0.5, 0.6}
	constraint, err := newContactConstraint(a, b, Contact{Normal: Vec3{Y: 1}, Points: []Vec3{point}}, 1)
	if err != nil {
		t.Fatal(err)
	}
	constraint.solveVelocity()
	contactVelocity := b.Velocity.Add(b.AngularVelocity.Cross(point.Sub(b.Position)))
	solverNear(t, contactVelocity.X, 0, 1e-12)
	solverNear(t, contactVelocity.Z, 0, 1e-12)
}

func TestSolverStaticAndFixedRotationHonorPublicVelocityEdits(t *testing.T) {
	w := NewWorld(Vec3{})
	a := solverBody(t, w, BodyDef{Shape: Sphere{Radius: 1}, Position: Vec3{X: -10}})
	b := solverBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1, FixedRotation: true})
	a.Velocity, a.AngularVelocity = Vec3{1, 2, 3}, Vec3{3, 2, 1}
	b.AngularVelocity = Vec3{1, 2, 3}
	solverStep(t, w, 0.1)
	if a.Position != (Vec3{X: -10}) || a.Velocity != (Vec3{}) || a.AngularVelocity != (Vec3{}) || a.Rotation != Identity() {
		t.Fatal("static body moved after a public velocity edit")
	}
	if b.AngularVelocity != (Vec3{}) || b.Rotation != Identity() {
		t.Fatal("locked body rotated after a public velocity edit")
	}
}

package physics3d

import (
	"encoding/json"
	"math"
	"os"
	"reflect"
	"strconv"
	"testing"
)

func integrationTestBody(t testing.TB, w *World, def BodyDef) *Body {
	t.Helper()
	body, err := w.CreateBody(def)
	if err != nil {
		t.Fatalf("CreateBody: %v", err)
	}
	return body
}

func integrationTestStep(t testing.TB, w *World, dt float64) {
	t.Helper()
	if err := w.Step(dt); err != nil {
		t.Fatalf("Step(%g): %v", dt, err)
	}
}

func integrationTestNear(t testing.TB, got, want, tolerance float64) {
	t.Helper()
	if math.IsNaN(got) || math.IsInf(got, 0) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.12g, want %.12g within %g", got, want, tolerance)
	}
}

func integrationTestVector(t testing.TB, got, want Vec3, tolerance float64) {
	t.Helper()
	integrationTestNear(t, got.X, want.X, tolerance)
	integrationTestNear(t, got.Y, want.Y, tolerance)
	integrationTestNear(t, got.Z, want.Z, tolerance)
}

func TestIntegrationAnalyticalCentralImpacts(t *testing.T) {
	var fixtures []struct {
		Name        string  `json:"name"`
		MassA       float64 `json:"mass_a"`
		MassB       float64 `json:"mass_b"`
		VelocityA   float64 `json:"velocity_a"`
		VelocityB   float64 `json:"velocity_b"`
		Restitution float64 `json:"restitution"`
		ExpectedA   float64 `json:"expected_a"`
		ExpectedB   float64 `json:"expected_b"`
		Direction   Vec3    `json:"direction"`
	}
	data, err := os.ReadFile("testdata/central_impacts.json")
	if err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(data, &fixtures); err != nil {
		t.Fatal(err)
	}
	if len(fixtures) == 0 {
		t.Fatal("no impact fixtures")
	}
	for _, fixture := range fixtures {
		t.Run(fixture.Name, func(t *testing.T) {
			w := NewWorld(Vec3{})
			w.Substeps, w.RestitutionThreshold = 1, 0
			direction := fixture.Direction.Normalized()
			a := integrationTestBody(t, w, BodyDef{
				Type: Dynamic, Shape: Sphere{Radius: 0.5}, Mass: fixture.MassA,
				Position: direction.Mul(-0.5), Velocity: direction.Mul(fixture.VelocityA),
				Restitution: fixture.Restitution,
			})
			b := integrationTestBody(t, w, BodyDef{
				Type: Dynamic, Shape: Sphere{Radius: 0.5}, Mass: fixture.MassB,
				Position: direction.Mul(0.5), Velocity: direction.Mul(fixture.VelocityB),
				Restitution: fixture.Restitution,
			})
			integrationTestStep(t, w, 0.00001)
			integrationTestVector(t, a.Velocity, direction.Mul(fixture.ExpectedA), 1e-10)
			integrationTestVector(t, b.Velocity, direction.Mul(fixture.ExpectedB), 1e-10)
			initialMomentum := direction.Mul(fixture.MassA*fixture.VelocityA + fixture.MassB*fixture.VelocityB)
			finalMomentum := a.Velocity.Mul(fixture.MassA).Add(b.Velocity.Mul(fixture.MassB))
			integrationTestVector(t, finalMomentum, initialMomentum, 1e-10)
			initialEnergy := 0.5 * (fixture.MassA*fixture.VelocityA*fixture.VelocityA + fixture.MassB*fixture.VelocityB*fixture.VelocityB)
			finalEnergy := 0.5 * (fixture.MassA*a.Velocity.LengthSquared() + fixture.MassB*b.Velocity.LengthSquared())
			if fixture.Restitution == 1 {
				integrationTestNear(t, finalEnergy, initialEnergy, 1e-10)
			} else if finalEnergy > initialEnergy+1e-10 {
				t.Fatalf("impact created energy: before %g, after %g", initialEnergy, finalEnergy)
			}
			integrationTestVector(t, a.AngularVelocity, Vec3{}, 1e-10)
			integrationTestVector(t, b.AngularVelocity, Vec3{}, 1e-10)
		})
	}
}

func TestIntegrationFreeFallConvergence(t *testing.T) {
	gravity, position, velocity := Vec3{-2, -10, 3}, Vec3{8, 20, -4}, Vec3{1, 2, -1}
	exactPosition := position.Add(velocity).Add(gravity.Mul(0.5))
	run := func(steps int) float64 {
		w := NewWorld(gravity)
		w.Substeps = 1
		body := integrationTestBody(t, w, BodyDef{
			Type: Dynamic, Shape: Sphere{Radius: 0.5}, Mass: 2,
			Position: position, Velocity: velocity,
		})
		for range steps {
			integrationTestStep(t, w, 1/float64(steps))
		}
		integrationTestVector(t, body.Velocity, velocity.Add(gravity), 1e-10)
		return body.Position.Sub(exactPosition).Length()
	}
	coarse, fine := run(60), run(120)
	if fine >= coarse || fine > 0.05 {
		t.Fatalf("free fall did not converge: coarse %g, fine %g", coarse, fine)
	}
	integrationTestNear(t, coarse/fine, 2, 1e-9)
}

func TestIntegrationForceAndTorqueLifetime(t *testing.T) {
	for _, substeps := range []int{1, 4, 8} {
		t.Run(strconv.Itoa(substeps), func(t *testing.T) {
			w := NewWorld(Vec3{})
			w.Substeps = substeps
			body := integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 2})
			if err := body.ApplyForce(Vec3{4, 6, -8}); err != nil {
				t.Fatal(err)
			}
			if err := body.ApplyTorque(Vec3{Z: 1.6}); err != nil {
				t.Fatal(err)
			}
			if err := w.Step(0); err == nil {
				t.Fatal("Step(0) accepted")
			}
			integrationTestStep(t, w, 0.25)
			integrationTestVector(t, body.Velocity, Vec3{0.5, 0.75, -1}, 1e-12)
			integrationTestVector(t, body.AngularVelocity, Vec3{Z: 0.5}, 1e-12)
			integrationTestStep(t, w, 0.25)
			integrationTestVector(t, body.Velocity, Vec3{0.5, 0.75, -1}, 1e-12)
			integrationTestVector(t, body.AngularVelocity, Vec3{Z: 0.5}, 1e-12)
		})
	}
}

func TestIntegrationRotatedBoxImpulseUsesWorldInertia(t *testing.T) {
	w := NewWorld(Vec3{})
	body := integrationTestBody(t, w, BodyDef{
		Type: Dynamic, Shape: Box{HalfExtents: Vec3{1, 2, 3}}, Mass: 3,
		Rotation: QuaternionFromAxisAngle(Vec3{Y: 1}, math.Pi/4),
	})
	// Local principal inertias are (13,10,5). A torque around world X is
	// not a principal-axis torque after a 45-degree Y rotation.
	// J=(0,0,2) at r=(0,1,0) gives r×J=(2,0,0).
	if err := body.ApplyImpulse(Vec3{Z: 2}, body.Position.Add(Vec3{Y: 1})); err != nil {
		t.Fatal(err)
	}
	integrationTestVector(t, body.Velocity, Vec3{Z: 2.0 / 3}, 1e-12)
	integrationTestVector(t, body.AngularVelocity, Vec3{18.0 / 65, 0, 8.0 / 65}, 1e-12)
}

// Angular momentum is computed from rotated principal axes, independently of
// the implementation's inverse-inertia helper or integration state.
func integrationTestAngularMomentum(body *Body) Vec3 {
	q := body.Rotation
	ix, iy, iz := q.Rotate(Vec3{X: 1}), q.Rotate(Vec3{Y: 1}), q.Rotate(Vec3{Z: 1})
	inertia := body.Inertia()
	omega := body.AngularVelocity
	return ix.Mul(inertia.X * ix.Dot(omega)).Add(iy.Mul(inertia.Y * iy.Dot(omega))).Add(iz.Mul(inertia.Z * iz.Dot(omega)))
}

func TestIntegrationTorqueFreeBoxConservesWorldAngularMomentum(t *testing.T) {
	w := NewWorld(Vec3{})
	body := integrationTestBody(t, w, BodyDef{
		Type: Dynamic, Shape: Box{HalfExtents: Vec3{0.5, 1, 1.5}}, Mass: 2,
		Rotation: QuaternionFromAxisAngle(Vec3{1, 1, 0}, 0.7), AngularVelocity: Vec3{0.8, 1.1, -0.6},
	})
	initial := integrationTestAngularMomentum(body)
	for frame := range 2400 {
		integrationTestStep(t, w, 1.0/120)
		if frame%100 == 0 {
			error := integrationTestAngularMomentum(body).Sub(initial).Length()
			if error > initial.Length()*0.002 {
				t.Fatalf("torque-free world momentum drift at frame %d: relative error %g", frame, error/initial.Length())
			}
			q := body.Rotation
			integrationTestNear(t, q.X*q.X+q.Y*q.Y+q.Z*q.Z+q.W*q.W, 1, 1e-12)
		}
	}
}

func TestIntegrationSphereMaintainsKnownSpin(t *testing.T) {
	w := NewWorld(Vec3{})
	body := integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 2})
	if err := body.ApplyImpulse(Vec3{X: 4}, Vec3{Y: 1}); err != nil {
		t.Fatal(err)
	}
	// I=2mr²/5=0.8; r×J=(0,0,-4), so omega=(0,0,-5).
	integrationTestVector(t, body.AngularVelocity, Vec3{Z: -5}, 1e-12)
	for range 120 {
		integrationTestStep(t, w, 1.0/120)
	}
	integrationTestVector(t, body.AngularVelocity, Vec3{Z: -5}, 1e-10)
	integrationTestVector(t, body.Rotation.Rotate(Vec3{X: 1}), Vec3{math.Cos(5), -math.Sin(5), 0}, 1e-7)
}

func TestIntegrationRestingSphereAndRotatedBox(t *testing.T) {
	w := NewWorld(Vec3{Y: -10})
	integrationTestBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{20, 0.5, 20}}, Position: Vec3{Y: -0.5}, Friction: 0.6})
	bodies := []*Body{
		integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.5}, Mass: 1, Position: Vec3{-2, 4, 0}, Friction: 0.6}),
		integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec3{0.5, 0.5, 0.5}}, Mass: 1, Position: Vec3{2, 4, 0}, Rotation: QuaternionFromAxisAngle(Vec3{Y: 1}, 0.4), Friction: 0.6}),
	}
	for range 1200 {
		integrationTestStep(t, w, 1.0/120)
	}
	for i, body := range bodies {
		if math.Abs(body.Position.Y-0.5) > 0.02 || body.Velocity.Length() > 0.04 || body.AngularVelocity.Length() > 0.08 {
			t.Fatalf("body %d did not settle: p=%+v v=%+v omega=%+v", i, body.Position, body.Velocity, body.AngularVelocity)
		}
	}
}

func TestIntegrationThreeBoxStack(t *testing.T) {
	w := NewWorld(Vec3{Y: -10})
	integrationTestBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{20, 0.5, 20}}, Position: Vec3{Y: -0.5}, Friction: 0.6})
	var bodies []*Body
	for i := range 3 {
		bodies = append(bodies, integrationTestBody(t, w, BodyDef{
			Type: Dynamic, Shape: Box{HalfExtents: Vec3{0.5, 0.5, 0.5}}, Mass: 1,
			Position: Vec3{Y: 0.5 + 1.01*float64(i)}, Rotation: QuaternionFromAxisAngle(Vec3{Y: 1}, 0.1*float64(i)), Friction: 0.6,
		}))
	}
	for range 600 {
		integrationTestStep(t, w, 1.0/120)
	}
	for i, body := range bodies {
		up := body.Rotation.Rotate(Vec3{Y: 1})
		if math.Abs(body.Position.Y-(float64(i)+0.5)) > 0.15 || math.Hypot(body.Position.X, body.Position.Z) > 0.2 || up.Y < 0.98 || body.Velocity.Length() > 0.1 {
			t.Fatalf("stack body %d drifted: p=%+v v=%+v up=%+v", i, body.Position, body.Velocity, up)
		}
	}
}

func TestIntegrationCoulombFrictionIsIsotropicInContactPlane(t *testing.T) {
	for i, direction := range []Vec3{{X: 1}, {Z: 1}, {0.6, 0, 0.8}} {
		t.Run(strconv.Itoa(i), func(t *testing.T) {
			w := NewWorld(Vec3{Y: -10})
			w.Substeps = 1
			integrationTestBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{50, 0.5, 50}}, Position: Vec3{Y: -0.5}, Friction: 0.25})
			body := integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec3{0.5, 0.5, 0.5}}, Mass: 1, Position: Vec3{Y: 0.5}, Velocity: direction.Mul(5), Friction: 1, FixedRotation: true})
			previous := 5.0
			for frame := range 240 {
				integrationTestStep(t, w, 1.0/120)
				horizontal := Vec3{body.Velocity.X, 0, body.Velocity.Z}
				speed := horizontal.Length()
				if speed > previous+1e-9 {
					t.Fatalf("friction increased speed: %g -> %g", previous, speed)
				}
				previous = speed
				if frame == 59 {
					// mu=sqrt(.25*1)=.5; speed after .5s is 5-mu*g*t=2.5.
					integrationTestVector(t, horizontal, direction.Mul(2.5), 1e-7)
				}
			}
			if previous > 0.01 {
				t.Fatalf("sliding did not stop: speed %g", previous)
			}
		})
	}
}

func TestIntegrationKinematicBodyPushesDynamicBody(t *testing.T) {
	w := NewWorld(Vec3{})
	w.Substeps = 1
	kinematic := integrationTestBody(t, w, BodyDef{Type: Kinematic, Shape: Sphere{Radius: 0.5}, Position: Vec3{X: -1}, Velocity: Vec3{X: 2}})
	dynamic := integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.5}, Mass: 1})
	integrationTestStep(t, w, 0.001)
	integrationTestVector(t, kinematic.Velocity, Vec3{X: 2}, 1e-12)
	integrationTestVector(t, dynamic.Velocity, Vec3{X: 2}, 1e-12)
}

func TestIntegrationInvalidStepDoesNotPartiallyAdvanceBodies(t *testing.T) {
	tests := []struct {
		name string
		bad  func(*World, *Body) float64
	}{
		{"zero dt", func(*World, *Body) float64 { return 0 }},
		{"negative dt", func(*World, *Body) float64 { return -1 }},
		{"NaN dt", func(*World, *Body) float64 { return math.NaN() }},
		{"infinite dt", func(*World, *Body) float64 { return math.Inf(1) }},
		{"substep underflow", func(*World, *Body) float64 { return math.SmallestNonzeroFloat64 }},
		{"zero substeps", func(w *World, _ *Body) float64 { w.Substeps = 0; return 0.1 }},
		{"zero velocity iterations", func(w *World, _ *Body) float64 { w.VelocityIterations = 0; return 0.1 }},
		{"zero position iterations", func(w *World, _ *Body) float64 { w.PositionIterations = 0; return 0.1 }},
		{"invalid gravity", func(w *World, _ *Body) float64 { w.Gravity.X = math.NaN(); return 0.1 }},
		{"invalid restitution threshold", func(w *World, _ *Body) float64 { w.RestitutionThreshold = -1; return 0.1 }},
		{"invalid later position", func(_ *World, b *Body) float64 { b.Position.Z = math.Inf(1); return 0.1 }},
		{"invalid later rotation", func(_ *World, b *Body) float64 { b.Rotation.W = math.NaN(); return 0.1 }},
		{"zero later rotation", func(_ *World, b *Body) float64 { b.Rotation = Quaternion{}; return 0.1 }},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			w := NewWorld(Vec3{Y: -10})
			good := integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.5}, Mass: 1, Position: Vec3{X: -5}, Velocity: Vec3{1, 2, 3}, AngularVelocity: Vec3{0, 1, 2}})
			bad := integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.5}, Mass: 1, Position: Vec3{X: 5}})
			position, velocity, rotation, angular := good.Position, good.Velocity, good.Rotation, good.AngularVelocity
			if err := w.Step(tt.bad(w, bad)); err == nil {
				t.Fatal("invalid step succeeded")
			}
			if good.Position != position || good.Velocity != velocity || good.Rotation != rotation || good.AngularVelocity != angular {
				t.Fatal("invalid later body partially advanced earlier body")
			}
		})
	}
}

func TestIntegrationOverflowRollsBackAndRetainsForces(t *testing.T) {
	w := NewWorld(Vec3{})
	body := integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 2, Position: Vec3{Y: 2}, Velocity: Vec3{Y: 2}, AngularVelocity: Vec3{Z: 1}})
	fast := integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1, Position: Vec3{X: 1e308}, Velocity: Vec3{X: 1e308}})
	if err := body.ApplyForce(Vec3{X: 4}); err != nil {
		t.Fatal(err)
	}
	if err := body.ApplyTorque(Vec3{Z: 0.8}); err != nil {
		t.Fatal(err)
	}
	position, velocity, rotation, angular := body.Position, body.Velocity, body.Rotation, body.AngularVelocity
	fastPosition, fastVelocity := fast.Position, fast.Velocity
	if err := w.Step(8); err == nil {
		t.Fatal("accepted finite inputs with overflowing integrated motion")
	}
	if body.Position != position || body.Velocity != velocity || body.Rotation != rotation || body.AngularVelocity != angular || fast.Position != fastPosition || fast.Velocity != fastVelocity {
		t.Fatal("overflowing step failed to roll back every body")
	}
	fast.Velocity = Vec3{}
	integrationTestStep(t, w, 0.25)
	integrationTestVector(t, body.Velocity, Vec3{0.5, 2, 0}, 1e-12)
	integrationTestVector(t, body.AngularVelocity, Vec3{Z: 1.25}, 1e-12)
	integrationTestStep(t, w, 0.25)
	integrationTestVector(t, body.Velocity, Vec3{0.5, 2, 0}, 1e-12)
	integrationTestVector(t, body.AngularVelocity, Vec3{Z: 1.25}, 1e-12)
}

func TestIntegrationLifecycleAndAttachedJointRemoval(t *testing.T) {
	w := NewWorld(Vec3{})
	a := integrationTestBody(t, w, BodyDef{Shape: Sphere{Radius: 0.1}})
	b := integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 1, Position: Vec3{X: 2}})
	c := integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1}, Mass: 1, Position: Vec3{X: 4}})
	ab, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	bc, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: b, BodyB: c, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	bodies := w.Bodies()
	bodies[0] = nil
	if w.Bodies()[0] != a {
		t.Fatal("Bodies exposes mutable membership slice")
	}
	if !w.RemoveBody(a) || w.RemoveBody(a) || w.RemoveDistanceJoint(ab) {
		t.Fatal("body removal must remove its attached joint exactly once")
	}
	b.Velocity, c.Velocity = Vec3{X: 1}, Vec3{X: 1}
	integrationTestStep(t, w, 0.1)
	integrationTestNear(t, b.Position.X, 2.1, 1e-12)
	integrationTestNear(t, c.Position.X, 4.1, 1e-12)
	if !w.RemoveDistanceJoint(bc) {
		t.Fatal("unrelated joint was removed with body")
	}
}

func TestIntegrationSimulationIsRepeatable(t *testing.T) {
	type state struct {
		Position, Velocity, Angular Vec3
		Rotation                    Quaternion
	}
	run := func() []state {
		w := NewWorld(Vec3{Y: -10})
		integrationTestBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{10, 0.5, 10}}, Position: Vec3{Y: -0.5}, Friction: 0.5})
		integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec3{0.5, 0.5, 0.5}}, Mass: 2, Position: Vec3{Y: 2}, Rotation: QuaternionFromAxisAngle(Vec3{1, 1, 0}, 0.1), Friction: 0.5})
		integrationTestBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.5}, Mass: 1, Position: Vec3{-2, 1, 0.2}, Velocity: Vec3{X: 2}, Restitution: 0.2, Friction: 0.4})
		for range 300 {
			integrationTestStep(t, w, 1.0/120)
		}
		var result []state
		for _, body := range w.Bodies() {
			result = append(result, state{body.Position, body.Velocity, body.AngularVelocity, body.Rotation})
		}
		return result
	}
	if first, second := run(), run(); !reflect.DeepEqual(first, second) {
		t.Fatalf("scene is not repeatable:\n%+v\n%+v", first, second)
	}
}

func BenchmarkIntegrationWorldStep(b *testing.B) {
	w := NewWorld(Vec3{Y: -10})
	integrationTestBody(b, w, BodyDef{Shape: Box{HalfExtents: Vec3{20, 0.5, 20}}, Position: Vec3{Y: -0.5}})
	for i := range 24 {
		integrationTestBody(b, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.4}, Mass: 1, Position: Vec3{float64(i%4)*2 - 3, float64(i/16)*2 + 1, float64((i/4)%4)*2 - 3}})
	}
	b.ResetTimer()
	for range b.N {
		integrationTestStep(b, w, 1.0/60)
	}
}

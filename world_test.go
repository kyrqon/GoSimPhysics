package physics

import (
	"encoding/json"
	"math"
	"os"
	"reflect"
	"strconv"
	"testing"
)

func worldTestBody(t testing.TB, w *World, def BodyDef) *Body {
	t.Helper()
	b, err := w.CreateBody(def)
	if err != nil {
		t.Fatalf("CreateBody: %v", err)
	}
	return b
}

func worldTestStep(t testing.TB, w *World, dt float64) {
	t.Helper()
	if err := w.Step(dt); err != nil {
		t.Fatalf("Step(%g): %v", dt, err)
	}
}

func worldTestClose(t testing.TB, got, want, tolerance float64) {
	t.Helper()
	if math.IsNaN(got) || math.IsInf(got, 0) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.12g, want %.12g (tolerance %g)", got, want, tolerance)
	}
}

func TestWorldAnalyticalImpacts(t *testing.T) {
	var fixtures []struct {
		Name        string  `json:"name"`
		MassA       float64 `json:"mass_a"`
		MassB       float64 `json:"mass_b"`
		VelocityA   float64 `json:"velocity_a"`
		VelocityB   float64 `json:"velocity_b"`
		Restitution float64 `json:"restitution"`
		ExpectedA   float64 `json:"expected_a"`
		ExpectedB   float64 `json:"expected_b"`
	}
	data, err := os.ReadFile("testdata/impacts.json")
	if err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(data, &fixtures); err != nil {
		t.Fatal(err)
	}
	if len(fixtures) == 0 {
		t.Fatal("no impact fixtures")
	}
	for _, f := range fixtures {
		t.Run(f.Name, func(t *testing.T) {
			w := NewWorld(Vec2{})
			w.Substeps = 1
			w.RestitutionThreshold = 0
			a := worldTestBody(t, w, BodyDef{
				Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: f.MassA,
				Position: Vec2{X: -0.5}, Velocity: Vec2{X: f.VelocityA},
				Restitution: f.Restitution,
			})
			b := worldTestBody(t, w, BodyDef{
				Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: f.MassB,
				Position: Vec2{X: 0.5}, Velocity: Vec2{X: f.VelocityB},
				Restitution: f.Restitution,
			})
			worldTestStep(t, w, 0.0001)
			worldTestClose(t, a.Velocity.X, f.ExpectedA, 1e-10)
			worldTestClose(t, b.Velocity.X, f.ExpectedB, 1e-10)
			worldTestClose(t, a.Velocity.Y, 0, 1e-12)
			worldTestClose(t, b.Velocity.Y, 0, 1e-12)
			momentumBefore := f.MassA*f.VelocityA + f.MassB*f.VelocityB
			momentumAfter := f.MassA*a.Velocity.X + f.MassB*b.Velocity.X
			worldTestClose(t, momentumAfter, momentumBefore, 1e-10)
			energyBefore := 0.5 * (f.MassA*f.VelocityA*f.VelocityA + f.MassB*f.VelocityB*f.VelocityB)
			energyAfter := 0.5 * (f.MassA*a.Velocity.LengthSquared() + f.MassB*b.Velocity.LengthSquared())
			if f.Restitution == 1 {
				worldTestClose(t, energyAfter, energyBefore, 1e-10)
			} else if energyAfter > energyBefore+1e-10 {
				t.Fatalf("impact created energy: before %g, after %g", energyBefore, energyAfter)
			}
		})
	}
}

func TestWorldFreeFallConvergesToAnalyticalSolution(t *testing.T) {
	const duration, acceleration, initialY, initialV = 1.0, -10.0, 20.0, 2.0
	exactY := initialY + initialV*duration + 0.5*acceleration*duration*duration
	run := func(steps int) float64 {
		w := NewWorld(Vec2{Y: acceleration})
		w.Substeps = 1
		body := worldTestBody(t, w, BodyDef{
			Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: 2,
			Position: Vec2{Y: initialY}, Velocity: Vec2{Y: initialV},
		})
		for i := 0; i < steps; i++ {
			worldTestStep(t, w, duration/float64(steps))
		}
		worldTestClose(t, body.Velocity.Y, initialV+acceleration*duration, 1e-10)
		return math.Abs(body.Position.Y - exactY)
	}
	coarseError, fineError := run(60), run(120)
	if fineError >= coarseError || fineError > 0.05 {
		t.Fatalf("free-fall does not converge: coarse error %g, fine error %g", coarseError, fineError)
	}
	worldTestClose(t, coarseError/fineError, 2, 1e-9)
}

func TestWorldForcePersistsAcrossSubstepsAndClearsAfterStep(t *testing.T) {
	for _, substeps := range []int{1, 4, 16} {
		t.Run(strconv.Itoa(substeps), func(t *testing.T) {
			w := NewWorld(Vec2{})
			w.Substeps = substeps
			body := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 2})
			if err := body.ApplyForce(Vec2{X: 4, Y: 6}); err != nil {
				t.Fatal(err)
			}
			if err := body.ApplyTorque(3); err != nil {
				t.Fatal(err)
			}
			if err := w.Step(0); err == nil {
				t.Fatal("Step(0) must fail without consuming forces")
			}
			worldTestStep(t, w, 0.25)
			worldTestClose(t, body.Velocity.X, 0.5, 1e-12)
			worldTestClose(t, body.Velocity.Y, 0.75, 1e-12)
			worldTestClose(t, body.AngularVelocity, 0.75, 1e-12)
			worldTestStep(t, w, 0.25)
			worldTestClose(t, body.Velocity.X, 0.5, 1e-12)
			worldTestClose(t, body.Velocity.Y, 0.75, 1e-12)
			worldTestClose(t, body.AngularVelocity, 0.75, 1e-12)
		})
	}
}

func TestWorldStaticAndKinematicBodies(t *testing.T) {
	w := NewWorld(Vec2{Y: -10})
	static := worldTestBody(t, w, BodyDef{
		Type: Static, Shape: Circle{Radius: 0.5}, Position: Vec2{X: -10, Y: 3},
		Velocity: Vec2{X: 20, Y: 20}, AngularVelocity: 20,
	})
	kinematic := worldTestBody(t, w, BodyDef{
		Type: Kinematic, Shape: Circle{Radius: 0.5}, Position: Vec2{X: 10},
		Velocity: Vec2{X: 2, Y: 3}, AngularVelocity: 0.5,
	})
	for _, body := range []*Body{static, kinematic} {
		if err := body.ApplyForce(Vec2{Y: 1000}); err != nil {
			t.Fatal(err)
		}
		if err := body.ApplyTorque(1000); err != nil {
			t.Fatal(err)
		}
	}
	worldTestStep(t, w, 0.5)
	if static.Position != (Vec2{X: -10, Y: 3}) || static.Angle != 0 || static.Velocity != (Vec2{}) || static.AngularVelocity != 0 {
		t.Fatalf("static body moved: %+v", static)
	}
	worldTestClose(t, kinematic.Position.X, 11, 1e-12)
	worldTestClose(t, kinematic.Position.Y, 1.5, 1e-12)
	worldTestClose(t, kinematic.Angle, 0.25, 1e-12)
	if kinematic.Velocity != (Vec2{X: 2, Y: 3}) || kinematic.AngularVelocity != 0.5 {
		t.Fatalf("forces affected kinematic velocity: %+v", kinematic)
	}
}

func TestWorldKinematicBodyPushesDynamicBodyWithoutSlowing(t *testing.T) {
	w := NewWorld(Vec2{})
	w.Substeps = 1
	kinematic := worldTestBody(t, w, BodyDef{Type: Kinematic, Shape: Circle{Radius: 0.5}, Position: Vec2{X: -1}, Velocity: Vec2{X: 2}})
	dynamic := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: 1})
	worldTestStep(t, w, 0.01)
	worldTestClose(t, kinematic.Velocity.X, 2, 1e-12)
	worldTestClose(t, dynamic.Velocity.X, 2, 1e-12)
}

func TestWorldRestingBodiesRemainOnGround(t *testing.T) {
	w := NewWorld(Vec2{Y: -10})
	worldTestBody(t, w, BodyDef{Type: Static, Shape: Box{HalfExtents: Vec2{X: 20, Y: 0.5}}, Position: Vec2{Y: -0.5}, Friction: 0.5})
	bodies := []*Body{
		worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: 1, Position: Vec2{X: -2, Y: 3}, Friction: 0.5}),
		worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec2{X: 0.5, Y: 0.5}}, Mass: 1, Position: Vec2{X: 2, Y: 3}, Friction: 0.5}),
	}
	for i := 0; i < 1200; i++ {
		worldTestStep(t, w, 1.0/120)
	}
	for i, body := range bodies {
		if math.Abs(body.Position.Y-0.5) > 0.015 || body.Velocity.Length() > 0.02 || math.Abs(body.AngularVelocity) > 0.02 {
			t.Fatalf("body %d did not settle on ground: position %+v velocity %+v angular velocity %g", i, body.Position, body.Velocity, body.AngularVelocity)
		}
	}
}

func TestWorldFrictionSlowsSlidingBox(t *testing.T) {
	for _, friction := range []float64{0, 0.5} {
		w := NewWorld(Vec2{Y: -10})
		worldTestBody(t, w, BodyDef{Type: Static, Shape: Box{HalfExtents: Vec2{X: 50, Y: 0.5}}, Position: Vec2{Y: -0.5}, Friction: friction})
		box := worldTestBody(t, w, BodyDef{
			Type: Dynamic, Shape: Box{HalfExtents: Vec2{X: 0.5, Y: 0.5}}, Mass: 1,
			Position: Vec2{Y: 0.5}, Velocity: Vec2{X: 5}, Friction: friction, FixedRotation: true,
		})
		previousSpeed := 5.0
		for i := 0; i < 300; i++ {
			worldTestStep(t, w, 1.0/120)
			if math.Abs(box.Velocity.X) > previousSpeed+1e-9 {
				t.Fatalf("friction created horizontal kinetic energy at frame %d: speed %g -> %g", i, previousSpeed, box.Velocity.X)
			}
			previousSpeed = math.Abs(box.Velocity.X)
		}
		if friction == 0 {
			worldTestClose(t, box.Velocity.X, 5, 1e-9)
		} else if math.Abs(box.Velocity.X) > 0.05 {
			t.Fatalf("friction did not stop sliding box: vx=%g", box.Velocity.X)
		}
	}
}

func TestWorldThreeBoxStackRemainsStable(t *testing.T) {
	w := NewWorld(Vec2{Y: -10})
	worldTestBody(t, w, BodyDef{Type: Static, Shape: Box{HalfExtents: Vec2{X: 20, Y: 0.5}}, Position: Vec2{Y: -0.5}, Friction: 0.6})
	var boxes []*Body
	for i := 0; i < 3; i++ {
		boxes = append(boxes, worldTestBody(t, w, BodyDef{
			Type: Dynamic, Shape: Box{HalfExtents: Vec2{X: 0.5, Y: 0.5}}, Mass: 1,
			Position: Vec2{Y: float64(i) + 0.5}, Friction: 0.6,
		}))
	}
	for i := 0; i < 600; i++ {
		worldTestStep(t, w, 1.0/120)
	}
	for i, body := range boxes {
		if math.Abs(body.Position.Y-(float64(i)+0.5)) > 0.08 || math.Abs(body.Position.X) > 0.1 || math.Abs(body.Angle) > 0.1 || body.Velocity.Length() > 0.1 {
			t.Fatalf("stack body %d drifted: position %+v angle %g velocity %+v", i, body.Position, body.Angle, body.Velocity)
		}
	}
}

func TestWorldOffCenterCollisionTransfersAngularMomentum(t *testing.T) {
	w := NewWorld(Vec2{})
	w.Substeps = 1
	box := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec2{X: 1, Y: 1}}, Mass: 3})
	ball := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: 1, Position: Vec2{X: -1.45, Y: 0.65}, Velocity: Vec2{X: 10}})
	worldTestStep(t, w, 0.0001)
	if box.Velocity.X <= 0 || box.AngularVelocity >= -0.01 || ball.Velocity.X >= 10 {
		t.Fatalf("off-center collision did not rotate target: box vx=%g omega=%g, ball vx=%g", box.Velocity.X, box.AngularVelocity, ball.Velocity.X)
	}
	worldTestClose(t, 3*box.Velocity.X+ball.Velocity.X, 10, 1e-10)
}

func TestWorldSeparatingBodiesDoNotReceiveAttractiveImpulses(t *testing.T) {
	w := NewWorld(Vec2{})
	a := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 1, Position: Vec2{X: -0.9}, Velocity: Vec2{X: -2}})
	b := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 1, Position: Vec2{X: 0.9}, Velocity: Vec2{X: 2}})
	worldTestStep(t, w, 0.001)
	worldTestClose(t, a.Velocity.X, -2, 1e-12)
	worldTestClose(t, b.Velocity.X, 2, 1e-12)
}

func TestWorldRestitutionThresholdSuppressesSmallBounces(t *testing.T) {
	for _, threshold := range []float64{0, 1} {
		w := NewWorld(Vec2{})
		w.Substeps = 1
		w.RestitutionThreshold = threshold
		worldTestBody(t, w, BodyDef{Type: Static, Shape: Circle{Radius: 0.5}, Position: Vec2{Y: -0.5}})
		ball := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: 1, Position: Vec2{Y: 0.5}, Velocity: Vec2{Y: -0.1}, Restitution: 1})
		worldTestStep(t, w, 0.001)
		want := 0.0
		if threshold == 0 {
			want = 0.1
		}
		worldTestClose(t, ball.Velocity.Y, want, 1e-12)
	}
}

func TestWorldDampingDissipatesEnergy(t *testing.T) {
	w := NewWorld(Vec2{})
	body := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 2, Velocity: Vec2{X: 4, Y: -3}, AngularVelocity: 2, LinearDamping: 1, AngularDamping: 1})
	energy := func() float64 {
		return 0.5*body.Mass()*body.Velocity.LengthSquared() + 0.5*body.Inertia()*body.AngularVelocity*body.AngularVelocity
	}
	initial, previous := energy(), energy()
	for i := 0; i < 600; i++ {
		worldTestStep(t, w, 1.0/120)
		current := energy()
		if current >= previous || math.IsNaN(current) {
			t.Fatalf("damping did not dissipate energy at frame %d: %g -> %g", i, previous, current)
		}
		previous = current
	}
	if previous >= initial*0.001 {
		t.Fatalf("damping left excessive energy: initial %g final %g", initial, previous)
	}
}

func TestWorldRejectsInvalidStepsBeforeMutation(t *testing.T) {
	tests := []struct {
		name string
		bad  func(*World) float64
	}{
		{"zero_dt", func(*World) float64 { return 0 }},
		{"negative_dt", func(*World) float64 { return -0.1 }},
		{"nan_dt", func(*World) float64 { return math.NaN() }},
		{"infinite_dt", func(*World) float64 { return math.Inf(1) }},
		{"substep_underflow", func(*World) float64 { return math.SmallestNonzeroFloat64 }},
		{"zero_substeps", func(w *World) float64 { w.Substeps = 0; return 0.1 }},
		{"negative_substeps", func(w *World) float64 { w.Substeps = -1; return 0.1 }},
		{"zero_velocity_iterations", func(w *World) float64 { w.VelocityIterations = 0; return 0.1 }},
		{"zero_position_iterations", func(w *World) float64 { w.PositionIterations = 0; return 0.1 }},
		{"nan_gravity", func(w *World) float64 { w.Gravity.X = math.NaN(); return 0.1 }},
		{"infinite_gravity", func(w *World) float64 { w.Gravity.Y = math.Inf(1); return 0.1 }},
		{"negative_restitution_threshold", func(w *World) float64 { w.RestitutionThreshold = -1; return 0.1 }},
		{"nan_restitution_threshold", func(w *World) float64 { w.RestitutionThreshold = math.NaN(); return 0.1 }},
		{"infinite_restitution_threshold", func(w *World) float64 { w.RestitutionThreshold = math.Inf(1); return 0.1 }},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			w := NewWorld(Vec2{})
			body := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: 1, Position: Vec2{X: 2}, Velocity: Vec2{X: 3}, AngularVelocity: 4})
			position, velocity, angle, angularVelocity := body.Position, body.Velocity, body.Angle, body.AngularVelocity
			if err := w.Step(tt.bad(w)); err == nil {
				t.Fatal("invalid Step succeeded")
			}
			if body.Position != position || body.Velocity != velocity || body.Angle != angle || body.AngularVelocity != angularVelocity {
				t.Fatal("invalid Step mutated body state")
			}
		})
	}
}

func TestWorldRejectsNonfiniteBodyStateBeforeAdvancingAnyBody(t *testing.T) {
	tests := []struct {
		name string
		bad  func(*Body)
	}{
		{"position_nan", func(b *Body) { b.Position.X = math.NaN() }},
		{"position_inf", func(b *Body) { b.Position.Y = math.Inf(1) }},
		{"velocity_nan", func(b *Body) { b.Velocity.X = math.NaN() }},
		{"angle_nan", func(b *Body) { b.Angle = math.NaN() }},
		{"angular_velocity_inf", func(b *Body) { b.AngularVelocity = math.Inf(1) }},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			w := NewWorld(Vec2{Y: -10})
			good := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: 1, Position: Vec2{X: -5}, Velocity: Vec2{X: 3}, AngularVelocity: 4})
			bad := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: 1, Position: Vec2{X: 5}})
			position, velocity, angle, angularVelocity := good.Position, good.Velocity, good.Angle, good.AngularVelocity
			tt.bad(bad)
			if err := w.Step(0.1); err == nil {
				t.Fatal("Step accepted nonfinite body state")
			}
			if good.Position != position || good.Velocity != velocity || good.Angle != angle || good.AngularVelocity != angularVelocity {
				t.Fatal("invalid later body caused partial advancement")
			}
		})
	}
}

func TestWorldOverflowRollsBackAllBodiesAndRetainsForces(t *testing.T) {
	w := NewWorld(Vec2{})
	body := worldTestBody(t, w, BodyDef{
		Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 2,
		Position: Vec2{X: -10}, Velocity: Vec2{X: 1, Y: 2}, Angle: 0.25, AngularVelocity: 0.5,
	})
	fast := worldTestBody(t, w, BodyDef{
		Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 1,
		Position: Vec2{X: 10}, Velocity: Vec2{X: math.MaxFloat64 / 2}, AngularVelocity: 2,
	})
	if err := body.ApplyForce(Vec2{X: 4, Y: 6}); err != nil {
		t.Fatal(err)
	}
	if err := body.ApplyTorque(3); err != nil {
		t.Fatal(err)
	}
	type state struct {
		position, velocity     Vec2
		angle, angularVelocity float64
	}
	before := make([]state, 0, 2)
	for _, b := range []*Body{body, fast} {
		before = append(before, state{b.Position, b.Velocity, b.Angle, b.AngularVelocity})
	}
	// All input values are finite. The fast body's second substep overflows its
	// position, after both bodies have already advanced during the first one.
	if err := w.Step(8); err == nil {
		t.Fatal("Step accepted overflowing arithmetic")
	}
	for i, b := range []*Body{body, fast} {
		after := state{b.Position, b.Velocity, b.Angle, b.AngularVelocity}
		if after != before[i] {
			t.Fatalf("body %d was not rolled back: before %+v, after %+v", i, before[i], after)
		}
	}
	// Reduce the hazardous velocity and retry. Force and torque from the failed
	// step must still act once over the successful interval.
	fast.Velocity = Vec2{}
	worldTestStep(t, w, 0.25)
	worldTestClose(t, body.Velocity.X, 1.5, 1e-12)
	worldTestClose(t, body.Velocity.Y, 2.75, 1e-12)
	worldTestClose(t, body.AngularVelocity, 1.25, 1e-12)
	worldTestStep(t, w, 0.25)
	worldTestClose(t, body.Velocity.X, 1.5, 1e-12)
	worldTestClose(t, body.Velocity.Y, 2.75, 1e-12)
	worldTestClose(t, body.AngularVelocity, 1.25, 1e-12)
}

func TestWorldBodyLifecycleAndBodyListSnapshot(t *testing.T) {
	w := NewWorld(Vec2{})
	a := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 1})
	b := worldTestBody(t, w, BodyDef{Type: Static, Shape: Circle{Radius: 1}, Position: Vec2{X: 5}})
	bodies := w.Bodies()
	if len(bodies) != 2 || bodies[0] != a || bodies[1] != b {
		t.Fatalf("unexpected body insertion order: %v", bodies)
	}
	bodies[0] = nil
	if w.Bodies()[0] != a {
		t.Fatal("Bodies returned mutable internal slice")
	}
	otherWorld := NewWorld(Vec2{})
	foreign := worldTestBody(t, otherWorld, BodyDef{Type: Static, Shape: Circle{Radius: 1}})
	if w.RemoveBody(foreign) || w.RemoveBody(nil) {
		t.Fatal("removed unowned body")
	}
	if !w.RemoveBody(a) || w.RemoveBody(a) {
		t.Fatal("body removal must succeed exactly once")
	}
	if got := w.Bodies(); len(got) != 1 || got[0] != b {
		t.Fatalf("unexpected body list after removal: %v", got)
	}
	oldPosition := a.Position
	a.Velocity.X = 10
	worldTestStep(t, w, 0.1)
	if a.Position != oldPosition {
		t.Fatal("removed body was still simulated")
	}
}

func TestWorldSimulationIsRepeatable(t *testing.T) {
	type state struct {
		Position, Velocity Vec2
		Angle, Angular     float64
	}
	run := func() []state {
		w := NewWorld(Vec2{Y: -10})
		worldTestBody(t, w, BodyDef{Type: Static, Shape: Box{HalfExtents: Vec2{X: 10, Y: 0.5}}, Position: Vec2{Y: -0.5}, Friction: 0.5})
		worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec2{X: 0.5, Y: 0.5}}, Mass: 2, Position: Vec2{Y: 2}, Angle: 0.1, Friction: 0.5})
		worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: 1, Position: Vec2{X: -2, Y: 1}, Velocity: Vec2{X: 2}, Restitution: 0.2, Friction: 0.4})
		for i := 0; i < 300; i++ {
			worldTestStep(t, w, 1.0/120)
		}
		var result []state
		for _, b := range w.Bodies() {
			result = append(result, state{b.Position, b.Velocity, b.Angle, b.AngularVelocity})
		}
		return result
	}
	if first, second := run(), run(); !reflect.DeepEqual(first, second) {
		t.Fatalf("same scene produced different results:\n%+v\n%+v", first, second)
	}
}

func BenchmarkWorldStep(b *testing.B) {
	w := NewWorld(Vec2{Y: -10})
	worldTestBody(b, w, BodyDef{Type: Static, Shape: Box{HalfExtents: Vec2{X: 25, Y: 0.5}}, Position: Vec2{Y: -0.5}})
	for i := 0; i < 30; i++ {
		worldTestBody(b, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.4}, Mass: 1, Position: Vec2{X: float64(i%10)*2 - 9, Y: float64(i/10)*2 + 1}})
	}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		worldTestStep(b, w, 1.0/60)
	}
}

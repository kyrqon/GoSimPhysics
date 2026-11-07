package physics3d

import (
	"encoding/json"
	"math"
	"os"
	"testing"
)

type ccdIntegrationFixture struct {
	Name                  string  `json:"name"`
	Geometry              string  `json:"geometry"`
	RadiusA               float64 `json:"radius_a"`
	RadiusB               float64 `json:"radius_b"`
	PositionA             float64 `json:"position_a"`
	PositionB             float64 `json:"position_b"`
	VelocityA             float64 `json:"velocity_a"`
	VelocityB             float64 `json:"velocity_b"`
	MassA                 float64 `json:"mass_a"`
	MassB                 float64 `json:"mass_b"`
	WallHalfWidth         float64 `json:"wall_half_width"`
	Restitution           float64 `json:"restitution"`
	DT                    float64 `json:"dt"`
	TimeOfImpact          float64 `json:"time_of_impact"`
	ExpectedPositionA     float64 `json:"expected_position_a"`
	ExpectedPositionB     float64 `json:"expected_position_b"`
	ExpectedVelocityA     float64 `json:"expected_velocity_a"`
	ExpectedVelocityB     float64 `json:"expected_velocity_b"`
	ExpectedMomentum      float64 `json:"expected_momentum"`
	ExpectedKineticEnergy float64 `json:"expected_kinetic_energy"`
}

func ccdIntegrationWorld() *World {
	w := NewWorld(Vec3{})
	w.Substeps, w.RestitutionThreshold = 1, 0
	return w
}

func ccdIntegrationBody(t testing.TB, w *World, def BodyDef) *Body {
	t.Helper()
	b, err := w.CreateBody(def)
	if err != nil {
		t.Fatal(err)
	}
	return b
}

func ccdIntegrationStep(t testing.TB, w *World, dt float64) {
	t.Helper()
	if err := w.Step(dt); err != nil {
		t.Fatal(err)
	}
}

func ccdIntegrationNear(t testing.TB, got, want, tolerance float64) {
	t.Helper()
	if math.IsNaN(got) || math.IsInf(got, 0) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.15g, want %.15g (tolerance %g)", got, want, tolerance)
	}
}

func ccdIntegrationVector(t testing.TB, got, want Vec3, tolerance float64) {
	t.Helper()
	ccdIntegrationNear(t, got.X, want.X, tolerance)
	ccdIntegrationNear(t, got.Y, want.Y, tolerance)
	ccdIntegrationNear(t, got.Z, want.Z, tolerance)
}

func TestCCDIntegrationAnalyticFixtures(t *testing.T) {
	data, err := os.ReadFile("../testdata/ccd_impacts.json")
	if err != nil {
		t.Fatal(err)
	}
	var fixtures []ccdIntegrationFixture
	if err := json.Unmarshal(data, &fixtures); err != nil {
		t.Fatal(err)
	}
	if len(fixtures) != 3 {
		t.Fatalf("expected three independently derived fixtures, got %d", len(fixtures))
	}
	for _, f := range fixtures {
		t.Run(f.Name, func(t *testing.T) {
			w := ccdIntegrationWorld()
			a := ccdIntegrationBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: f.RadiusA},
				Position: Vec3{X: f.PositionA}, Velocity: Vec3{X: f.VelocityA}, Mass: f.MassA,
				Restitution: f.Restitution, Continuous: true})
			var b *Body
			switch f.Geometry {
			case "round_wall":
				b = ccdIntegrationBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{f.WallHalfWidth, 5, 5}}, Restitution: f.Restitution})
			case "round_pair":
				b = ccdIntegrationBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: f.RadiusB},
					Position: Vec3{X: f.PositionB}, Velocity: Vec3{X: f.VelocityB}, Mass: f.MassB, Restitution: f.Restitution})
			default:
				t.Fatalf("unknown fixture geometry %q", f.Geometry)
			}
			impact, hit, err := TimeOfImpact(a, b, f.DT)
			if err != nil || !hit {
				t.Fatalf("expected impact: hit=%v err=%v", hit, err)
			}
			ccdIntegrationNear(t, impact.Time, f.TimeOfImpact, 1e-9)
			ccdIntegrationStep(t, w, f.DT)
			ccdIntegrationVector(t, a.Position, Vec3{X: f.ExpectedPositionA}, 1e-5)
			ccdIntegrationVector(t, a.Velocity, Vec3{X: f.ExpectedVelocityA}, 1e-5)
			if f.Geometry == "round_pair" {
				ccdIntegrationVector(t, b.Position, Vec3{X: f.ExpectedPositionB}, 1e-5)
				ccdIntegrationVector(t, b.Velocity, Vec3{X: f.ExpectedVelocityB}, 1e-5)
				ccdIntegrationNear(t, a.Mass()*a.Velocity.X+b.Mass()*b.Velocity.X, f.ExpectedMomentum, 1e-5)
				energy := (a.Mass()*a.Velocity.LengthSquared() + b.Mass()*b.Velocity.LengthSquared()) / 2
				ccdIntegrationNear(t, energy, f.ExpectedKineticEnergy, 1e-5)
			}
		})
	}
}

// This is a geometric oracle for a point reflected inside an interval. It does
// not call the collision detector, solver, or time-of-impact implementation.
func ccdIntegrationFold(position, velocity, dt, lo, hi float64) (float64, float64) {
	width := hi - lo
	phase := math.Mod(position-lo+velocity*dt, 2*width)
	if phase < 0 {
		phase += 2 * width
	}
	if phase <= width {
		return lo + phase, velocity
	}
	return hi - (phase - width), -velocity
}

func ccdIntegrationCorridor(t testing.TB, w *World, def BodyDef) *Body {
	t.Helper()
	for _, x := range []float64{-1, 1} {
		ccdIntegrationBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 5, 5}}, Position: Vec3{X: x}, Restitution: 1})
	}
	return ccdIntegrationBody(t, w, def)
}

func TestCCDIntegrationMultipleImpactsConsumeWholeStep(t *testing.T) {
	w := ccdIntegrationWorld()
	b := ccdIntegrationCorridor(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05},
		Velocity: Vec3{X: 100}, Mass: 1, Restitution: 1, Continuous: true})
	wantX, wantV := ccdIntegrationFold(0, 100, 0.05, -0.94, 0.94)
	ccdIntegrationStep(t, w, 0.05)
	ccdIntegrationVector(t, b.Position, Vec3{X: wantX}, 1e-5)
	ccdIntegrationVector(t, b.Velocity, Vec3{X: wantV}, 1e-5)
}

func TestCCDIntegrationForceAndDampingOncePerOriginalSubstep(t *testing.T) {
	w := ccdIntegrationWorld()
	b := ccdIntegrationCorridor(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05},
		Velocity: Vec3{X: 100}, Mass: 2, Restitution: 1, LinearDamping: 0.2, Continuous: true})
	if err := b.ApplyForce(Vec3{X: 200}); err != nil {
		t.Fatal(err)
	}
	// Semi-implicit force integration and the documented rational damping occur
	// once, before any of the three impacts inside this original substep.
	speed := (100 + 200.0/2*0.05) / (1 + 0.2*0.05)
	wantX, wantV := ccdIntegrationFold(0, speed, 0.05, -0.94, 0.94)
	ccdIntegrationStep(t, w, 0.05)
	ccdIntegrationNear(t, b.Position.X, wantX, 1e-5)
	ccdIntegrationNear(t, b.Velocity.X, wantV, 1e-5)
	// The successful step consumed the force; only damping remains next time.
	ccdIntegrationStep(t, w, 0.0001)
	ccdIntegrationNear(t, b.Velocity.X, wantV/(1+0.2*0.0001), 1e-5)
}

func TestCCDIntegrationOptInAndMutableFlag(t *testing.T) {
	w := ccdIntegrationWorld()
	wall := ccdIntegrationBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 5, 5}}})
	b := ccdIntegrationBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05},
		Position: Vec3{X: -2}, Velocity: Vec3{X: 400}, Mass: 1})
	ccdIntegrationStep(t, w, 0.01)
	ccdIntegrationNear(t, b.Position.X, 2, 1e-12)
	for _, flagOnWall := range []bool{false, true} {
		b.Position, b.Velocity = Vec3{X: -2}, Vec3{X: 400}
		b.Continuous, wall.Continuous = !flagOnWall, flagOnWall
		ccdIntegrationStep(t, w, 0.01)
		ccdIntegrationNear(t, b.Position.X, -0.06, 1e-5)
		ccdIntegrationNear(t, b.Velocity.X, 0, 1e-5)
	}
	b.Position, b.Velocity = Vec3{X: -2}, Vec3{X: 400}
	b.Continuous, wall.Continuous = false, false
	ccdIntegrationStep(t, w, 0.01)
	ccdIntegrationNear(t, b.Position.X, 2, 1e-12)
}

func TestCCDIntegrationInitialTouchApproachingAndSeparating(t *testing.T) {
	for _, velocity := range []float64{10, -10} {
		t.Run(map[bool]string{true: "approaching", false: "separating"}[velocity > 0], func(t *testing.T) {
			w := ccdIntegrationWorld()
			ccdIntegrationBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 5, 5}}})
			b := ccdIntegrationBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05},
				Position: Vec3{X: -(0.05 + 0.01)}, Velocity: Vec3{X: velocity}, Mass: 1, Continuous: true})
			ccdIntegrationStep(t, w, 0.01)
			if velocity > 0 {
				ccdIntegrationNear(t, b.Position.X, -0.06, 1e-5)
				ccdIntegrationNear(t, b.Velocity.X, 0, 1e-5)
			} else {
				ccdIntegrationNear(t, b.Position.X, -0.16, 1e-5)
				ccdIntegrationNear(t, b.Velocity.X, velocity, 1e-5)
			}
		})
	}
}

func TestCCDIntegrationRoundedCornerMiss(t *testing.T) {
	w := ccdIntegrationWorld()
	ccdIntegrationBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{1, 1, 1}}, Restitution: 1})
	b := ccdIntegrationBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1},
		Position: Vec3{-1.11, 1.08, 0}, Velocity: Vec3{3, 3, 0}, Mass: 1, Restitution: 1, Continuous: true})
	ccdIntegrationStep(t, w, 0.01)
	ccdIntegrationVector(t, b.Position, Vec3{-1.08, 1.11, 0}, 1e-9)
	ccdIntegrationVector(t, b.Velocity, Vec3{3, 3, 0}, 1e-9)
}

func TestCCDIntegrationRotatedThinWall(t *testing.T) {
	w := ccdIntegrationWorld()
	rotation := QuaternionFromAxisAngle(Vec3{Z: 1}, 0.7).Mul(QuaternionFromAxisAngle(Vec3{Y: 1}, -0.4))
	normal := rotation.Rotate(Vec3{X: 1})
	ccdIntegrationBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 5, 5}}, Rotation: rotation})
	b := ccdIntegrationBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05},
		Position: normal.Mul(-2), Velocity: normal.Mul(400), Mass: 1, Continuous: true})
	ccdIntegrationStep(t, w, 0.01)
	ccdIntegrationVector(t, b.Position, normal.Mul(-0.06), 1e-5)
	ccdIntegrationVector(t, b.Velocity, Vec3{}, 1e-5)
}

func TestCCDIntegrationRotatingReentryAfterSeparatingInitialTouch(t *testing.T) {
	w := ccdIntegrationWorld()
	ccdIntegrationBody(t, w, BodyDef{Type: Kinematic, Shape: Box{HalfExtents: Vec3{2, 0.02, 0.02}},
		AngularVelocity: Vec3{Z: -2 * math.Pi}, Continuous: true})
	b := ccdIntegrationBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05},
		Position: Vec3{1.4, 0.07, 0}, Velocity: Vec3{Y: 0.2}, Mass: 1, FixedRotation: true})
	// Initially the rod moves down and the touching sphere moves up. The
	// opposite end returns from above just before a half-turn. Ignoring this
	// pair for the entire interval would leave the sphere's velocity unchanged.
	ccdIntegrationStep(t, w, 0.49)
	if b.Velocity.Y >= -1 {
		t.Fatalf("missed rotational re-entry: position=%v velocity=%v", b.Position, b.Velocity)
	}
}

func TestCCDIntegrationMovingKinematicWall(t *testing.T) {
	w := ccdIntegrationWorld()
	wall := ccdIntegrationBody(t, w, BodyDef{Type: Kinematic, Shape: Box{HalfExtents: Vec3{0.01, 5, 5}},
		Velocity: Vec3{X: -20}, Restitution: 1})
	b := ccdIntegrationBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05},
		Position: Vec3{X: -2}, Velocity: Vec3{X: 400}, Mass: 1, Restitution: 1, Continuous: true})
	ccdIntegrationStep(t, w, 0.01)
	ccdIntegrationVector(t, wall.Position, Vec3{X: -0.2}, 1e-9)
	ccdIntegrationVector(t, wall.Velocity, Vec3{X: -20}, 1e-9)
	ccdIntegrationVector(t, b.Position, Vec3{X: -2.52}, 1e-5)
	ccdIntegrationVector(t, b.Velocity, Vec3{X: -440}, 1e-5)
}

func TestCCDIntegrationContinuousBox(t *testing.T) {
	w := ccdIntegrationWorld()
	ccdIntegrationBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 5, 5}}})
	b := ccdIntegrationBody(t, w, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec3{0.05, 0.2, 0.2}},
		Position: Vec3{X: -2}, Velocity: Vec3{X: 400}, Mass: 1, FixedRotation: true, Continuous: true})
	ccdIntegrationStep(t, w, 0.01)
	ccdIntegrationVector(t, b.Position, Vec3{X: -0.06}, 1e-5)
	ccdIntegrationVector(t, b.Velocity, Vec3{}, 1e-5)
}

func TestCCDIntegrationSimultaneousCornerImpacts(t *testing.T) {
	w := ccdIntegrationWorld()
	ccdIntegrationBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{0.01, 5, 5}}, Restitution: 1})
	ccdIntegrationBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{5, 0.01, 5}}, Restitution: 1})
	ccdIntegrationBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec3{5, 5, 0.01}}, Restitution: 1})
	b := ccdIntegrationBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05},
		Position: Vec3{-2, -2, -2}, Velocity: Vec3{400, 400, 400}, Mass: 1, Restitution: 1, Continuous: true})
	ccdIntegrationStep(t, w, 0.01)
	ccdIntegrationVector(t, b.Position, Vec3{-2.12, -2.12, -2.12}, 1e-5)
	ccdIntegrationVector(t, b.Velocity, Vec3{-400, -400, -400}, 1e-5)
}

func TestCCDIntegrationConnectedPairCollisionFiltering(t *testing.T) {
	w := ccdIntegrationWorld()
	a := ccdIntegrationBody(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.1},
		Velocity: Vec3{X: 100}, Mass: 1, Continuous: true})
	b := ccdIntegrationBody(t, w, BodyDef{Type: Kinematic, Shape: Sphere{Radius: 0.1},
		Position: Vec3{X: 0.1}, Velocity: Vec3{X: 100}})
	if _, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 0.1}); err != nil {
		t.Fatal(err)
	}
	ccdIntegrationStep(t, w, 0.01)
	ccdIntegrationVector(t, a.Position, Vec3{X: 1}, 1e-9)
	ccdIntegrationVector(t, b.Position, Vec3{X: 1.1}, 1e-9)
	ccdIntegrationVector(t, a.Velocity, Vec3{X: 100}, 1e-9)
}

func TestCCDIntegrationBudgetFailureRollsBackAndRetainsForce(t *testing.T) {
	w := ccdIntegrationWorld()
	w.MaxTOIEvents = 1
	b := ccdIntegrationCorridor(t, w, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 0.05},
		Velocity: Vec3{X: 100}, Mass: 2, Restitution: 1, Rotation: QuaternionFromAxisAngle(Vec3{Z: 1}, 0.2), AngularVelocity: Vec3{Z: 0.3}, Continuous: true})
	k := ccdIntegrationBody(t, w, BodyDef{Type: Kinematic, Shape: Sphere{Radius: 0.1},
		Position: Vec3{-3, 3, 0}, Velocity: Vec3{X: 0.5}, Rotation: QuaternionFromAxisAngle(Vec3{Z: 1}, -0.4), AngularVelocity: Vec3{Z: 0.2}})
	if err := b.ApplyForce(Vec3{X: 100}); err != nil {
		t.Fatal(err)
	}
	beforeB, beforeK := *b, *k
	if err := w.Step(0.05); err == nil {
		t.Fatal("expected failure rather than silent tunneling after impact budget exhaustion")
	}
	if b.Position != beforeB.Position || b.Velocity != beforeB.Velocity || b.Rotation != beforeB.Rotation || b.AngularVelocity != beforeB.AngularVelocity ||
		k.Position != beforeK.Position || k.Velocity != beforeK.Velocity || k.Rotation != beforeK.Rotation || k.AngularVelocity != beforeK.AngularVelocity {
		t.Fatal("failed step did not restore all dynamic and kinematic state")
	}
	ccdIntegrationStep(t, w, 0.001)
	ccdIntegrationNear(t, b.Velocity.X, 100.05, 1e-9)
	ccdIntegrationNear(t, b.Position.X, 0.10005, 1e-9)
	ccdIntegrationStep(t, w, 0.001)
	ccdIntegrationNear(t, b.Velocity.X, 100.05, 1e-9)
}

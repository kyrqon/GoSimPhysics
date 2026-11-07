package collision2d

import (
	"errors"
	"math"
	"reflect"
	"testing"
)

type motionDef struct {
	Shape                  Shape
	Position, Velocity     Vec2
	Angle, AngularVelocity float64
}

func toiTestBody(t *testing.T, def motionDef) *proxy {
	t.Helper()
	body, err := toiPrepare(&proxy{shape: def.Shape, Position: def.Position, Velocity: def.Velocity, Angle: def.Angle, AngularVelocity: def.AngularVelocity})
	if err != nil {
		t.Fatal(err)
	}
	return body
}

func toiTestNear(t *testing.T, got, want, tolerance float64) {
	t.Helper()
	if !finite(got) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.15g, want %.15g within %g", got, want, tolerance)
	}
}

func TestTimeOfImpactAnalyticalTranslation(t *testing.T) {
	tests := []struct {
		name     string
		a, b     motionDef
		dt, want float64
	}{
		{"thin wall", motionDef{Shape: Circle{Radius: 0.05}, Position: Vec2{X: -2}, Velocity: Vec2{X: 400}}, motionDef{Shape: Box{HalfExtents: Vec2{X: 0.01, Y: 3}}}, 0.01, 1.94 / 400},
		{"moving target", motionDef{Shape: Circle{Radius: 0.05}, Position: Vec2{X: -2}, Velocity: Vec2{X: 400}}, motionDef{Shape: Box{HalfExtents: Vec2{X: 0.01, Y: 3}}, Velocity: Vec2{X: 40}}, 0.01, 1.94 / 360},
		{"opposite circles", motionDef{Shape: Circle{Radius: 0.1}, Position: Vec2{X: -2}, Velocity: Vec2{X: 200}}, motionDef{Shape: Circle{Radius: 0.1}, Position: Vec2{X: 2}, Velocity: Vec2{X: -100}}, 0.02, 3.8 / 300},
		{"tangent circles", motionDef{Shape: Circle{Radius: 1}, Position: Vec2{X: -2, Y: 2}, Velocity: Vec2{X: 4}}, motionDef{Shape: Circle{Radius: 1}}, 1, 0.5},
		{"corner arc", motionDef{Shape: Circle{Radius: 0.1}, Position: Vec2{X: -2, Y: 2}, Velocity: Vec2{X: 2, Y: -2}}, motionDef{Shape: Box{HalfExtents: Vec2{X: 1, Y: 1}}}, 1, (1 - 0.1/math.Sqrt2) / 2},
		{"box face", motionDef{Shape: Box{HalfExtents: Vec2{X: 0.05, Y: 0.1}}, Position: Vec2{X: -2}, Velocity: Vec2{X: 400}}, motionDef{Shape: Box{HalfExtents: Vec2{X: 0.01, Y: 3}}}, 0.01, 1.94 / 400},
		{"rotated translating box", motionDef{Shape: Box{HalfExtents: Vec2{X: 0.08, Y: 0.03}}, Position: Vec2{X: -2}, Velocity: Vec2{X: 400}, Angle: 0.3}, motionDef{Shape: Box{HalfExtents: Vec2{X: 0.01, Y: 3}}}, 0.01, (2 - 0.08*math.Cos(0.3) - 0.03*math.Sin(0.3) - 0.01) / 400},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			a, b := toiTestBody(t, test.a), toiTestBody(t, test.b)
			beforeA, beforeB := *a, *b
			result, hit, err := timeOfImpactProxies(a, b, test.dt)
			if err != nil || !hit {
				t.Fatalf("missing hit: %v", err)
			}
			toiTestNear(t, result.Time, test.want, 1e-11)
			if !contactFinite(result.Contact) {
				t.Fatalf("invalid contact: %+v", result.Contact)
			}
			toiTestNear(t, result.Contact.Normal.Length(), 1, 1e-12)
			if !reflect.DeepEqual(*a, beforeA) || !reflect.DeepEqual(*b, beforeB) {
				t.Fatal("sweep mutated its bodies")
			}
			if test.name == "box face" && len(result.Contact.Points) != 2 {
				t.Fatalf("face impact should retain two contacts, got %v", result.Contact.Points)
			}
			if _, earlier, err := timeOfImpactProxies(a, b, test.want*0.99); err != nil || earlier {
				t.Fatalf("reported impact before the analytical time: hit %v, err %v", earlier, err)
			}
		})
	}
}

func TestTimeOfImpactRoundedCornerMissAndSeparation(t *testing.T) {
	box := toiTestBody(t, motionDef{Shape: Box{HalfExtents: Vec2{X: 1, Y: 1}}})
	circle := toiTestBody(t, motionDef{Shape: Circle{Radius: 0.1}, Position: Vec2{X: -1.11, Y: 1.08}, Velocity: Vec2{X: 0.03, Y: 0.03}})
	if _, hit, err := timeOfImpactProxies(circle, box, 1); err != nil || hit {
		t.Fatalf("inflated-AABB corner is not an actual shape hit: %v, %v", hit, err)
	}
	for _, velocity := range []Vec2{{}, {X: -100}, {Y: 100}} {
		circle.Position, circle.Velocity = Vec2{X: -3}, velocity
		if _, hit, err := timeOfImpactProxies(circle, box, 1); err != nil || hit {
			t.Fatalf("stationary or separating sweep hit: %v, %v", hit, err)
		}
	}
}

func TestTimeOfImpactRotatingRod(t *testing.T) {
	rod := toiTestBody(t, motionDef{Shape: Box{HalfExtents: Vec2{X: 2, Y: 0.02}}, AngularVelocity: math.Pi / 2})
	circle := toiTestBody(t, motionDef{Shape: Circle{Radius: 0.05}, Position: Vec2{X: 1.4, Y: 1.4}})
	if _, hit := collideProxies(rod, circle); hit {
		t.Fatal("fixture starts overlapped")
	}
	end, err := toiPose(rod, 1)
	if err != nil {
		t.Fatal(err)
	}
	if _, hit := collideProxies(end, circle); hit {
		t.Fatal("fixture ends overlapped; it must demonstrate an interior angular impact")
	}
	result, hit, err := timeOfImpactProxies(rod, circle, 1)
	if err != nil || !hit {
		t.Fatalf("rotational sweep missed: %v", err)
	}
	want := (math.Pi/4 - math.Asin(0.07/(1.4*math.Sqrt2))) / (math.Pi / 2)
	toiTestNear(t, result.Time, want, 2e-9)
	if _, hit, err := timeOfImpactProxies(rod, circle, want-0.001); err != nil || hit {
		t.Fatalf("rotational hit too early: %v, %v", hit, err)
	}
}

func TestTimeOfImpactRotatingBoxes(t *testing.T) {
	rod := toiTestBody(t, motionDef{Shape: Box{HalfExtents: Vec2{X: 2, Y: 0.02}}, AngularVelocity: math.Pi / 2})
	target := toiTestBody(t, motionDef{Shape: Box{HalfExtents: Vec2{X: 0.04, Y: 0.04}}, Position: Vec2{X: 1.4, Y: 1.4}})
	result, hit, err := timeOfImpactProxies(rod, target, 1)
	if err != nil || !hit {
		t.Fatalf("box angular sweep missed: %v", err)
	}
	if result.Time <= 0.4 || result.Time >= 0.5 {
		t.Fatalf("unexpected rotating-box hit time %g", result.Time)
	}
	if !contactFinite(result.Contact) {
		t.Fatal("invalid rotating-box contact")
	}
}

func TestTimeOfImpactThinBoxesUseFeatureScaleTolerance(t *testing.T) {
	// The circumradius is ten million times the thickness. A radius-scaled
	// tolerance would incorrectly classify this initial 1e-5 gap as contact.
	rod := toiTestBody(t, motionDef{Shape: Box{HalfExtents: Vec2{X: 1e4, Y: 0.001}}, AngularVelocity: 1e-8})
	target := toiTestBody(t, motionDef{Shape: Box{HalfExtents: Vec2{X: 1e4, Y: 0.001}}, Position: Vec2{Y: 0.00201}})
	result, hit, err := timeOfImpactProxies(rod, target, 0.11)
	if err != nil || !hit || result.Time < 0.099 || result.Time > 0.101 {
		t.Fatalf("thin-box sweep used an overly coarse tolerance: %+v %v %v", result, hit, err)
	}
	if _, hit, err := timeOfImpactProxies(rod, target, 0.05); err != nil || hit {
		t.Fatalf("thin boxes reported contact before angular travel closes the gap: %v %v", hit, err)
	}
}

func TestTimeOfImpactInitialTouch(t *testing.T) {
	a := toiTestBody(t, motionDef{Shape: Circle{Radius: 1}, Position: Vec2{X: -2}, Velocity: Vec2{X: -3}})
	b := toiTestBody(t, motionDef{Shape: Circle{Radius: 1}})
	result, hit, err := timeOfImpactProxies(a, b, 1)
	if err != nil || !hit || result.Time != 0 {
		t.Fatalf("initial touching must be observable even when separating: %+v %v %v", result, hit, err)
	}
}

func TestTimeOfImpactRigidTransformAndSwap(t *testing.T) {
	a := toiTestBody(t, motionDef{Shape: Circle{Radius: 0.1}, Position: Vec2{X: -3, Y: 0.2}, Velocity: Vec2{X: 6, Y: 0.4}})
	b := toiTestBody(t, motionDef{Shape: Box{HalfExtents: Vec2{X: 0.1, Y: 2}}, Velocity: Vec2{X: 0.5}, Angle: 0.2})
	base, hit, err := timeOfImpactProxies(a, b, 1)
	if err != nil || !hit {
		t.Fatal("invalid covariance fixture", err)
	}
	reverse, hit, err := timeOfImpactProxies(b, a, 1)
	if err != nil || !hit {
		t.Fatal("swapped query lost contact", err)
	}
	toiTestNear(t, reverse.Time, base.Time, 1e-12)
	toiTestNear(t, reverse.Contact.Normal.Add(base.Contact.Normal).Length(), 0, 1e-10)
	const angle = 0.71
	translation := Vec2{X: 4, Y: -2}
	for _, body := range []*proxy{a, b} {
		body.Position = body.Position.Rotate(angle).Add(translation)
		body.Velocity = body.Velocity.Rotate(angle)
		body.Angle += angle
	}
	moved, hit, err := timeOfImpactProxies(a, b, 1)
	if err != nil || !hit {
		t.Fatal("transformed query lost contact", err)
	}
	toiTestNear(t, moved.Time, base.Time, 1e-12)
	toiTestNear(t, moved.Contact.Normal.Sub(base.Contact.Normal.Rotate(angle)).Length(), 0, 1e-10)
}

func TestTimeOfImpactInputAndOverflowErrors(t *testing.T) {
	valid := toiTestBody(t, motionDef{Shape: Circle{Radius: 0.1}, Position: Vec2{X: -2}, Velocity: Vec2{X: 1}})
	target := toiTestBody(t, motionDef{Shape: Box{HalfExtents: Vec2{X: 1, Y: 1}}})
	for _, duration := range []float64{0, -1, math.NaN(), math.Inf(1)} {
		if _, hit, err := timeOfImpactProxies(valid, target, duration); err == nil || hit {
			t.Fatalf("accepted invalid duration %g", duration)
		}
	}
	var nilCircle *Circle
	for _, bad := range []*proxy{nil, {}, {shape: nilCircle}, {shape: Circle{Radius: -1}}} {
		if _, hit, err := timeOfImpactProxies(bad, target, 1); err == nil || hit {
			t.Fatal("accepted invalid body")
		}
	}
	for _, change := range []func(*proxy){
		func(b *proxy) { b.Velocity.X = math.NaN() },
		func(b *proxy) { b.AngularVelocity = math.Inf(1) },
		func(b *proxy) { b.Angle = math.NaN() },
		func(b *proxy) { b.Position.X = math.MaxFloat64; b.Velocity.X = math.MaxFloat64 },
	} {
		bad := *valid
		change(&bad)
		if _, hit, err := timeOfImpactProxies(&bad, target, 2); err == nil || hit {
			t.Fatal("accepted nonfinite state or overflowing motion")
		}
	}
}

func TestTimeOfImpactIterationExhaustionIsAnError(t *testing.T) {
	rod := toiTestBody(t, motionDef{Shape: Box{HalfExtents: Vec2{X: 1, Y: 0.01}}, AngularVelocity: 1})
	circle := toiTestBody(t, motionDef{Shape: Circle{Radius: 0.1}, Position: Vec2{X: math.Hypot(1, 0.01) + 0.1}})
	if _, hit, err := toiConservative(rod, circle, 1, 1); hit || !errors.Is(err, ErrTOINoConvergence) {
		t.Fatalf("exhausted search must report failure rather than silently miss: %v %v", hit, err)
	}
}

func TestTimeOfImpactSeparatingTouchReentryQuery(t *testing.T) {
	rod := toiTestBody(t, motionDef{Shape: Box{HalfExtents: Vec2{X: 2, Y: 0.02}}, AngularVelocity: -2 * math.Pi})
	circle := toiTestBody(t, motionDef{Shape: Circle{Radius: 0.05}, Position: Vec2{X: 1.4, Y: 0.07}, Velocity: Vec2{Y: 0.2}})
	circle.Position.Y += 8 * toiDistanceTolerance(rod, circle)
	result, hit, err := timeOfImpactProxies(rod, circle, 0.49)
	if err != nil || !hit || result.Time < 0.4 {
		t.Fatalf("separating initial contact reentry missed: %+v %v %v", result, hit, err)
	}
}

func FuzzTimeOfImpactInvariants(f *testing.F) {
	f.Add(-2.0, 0.0, 100.0, 0.0, 0.0, 0.0)
	f.Add(1.4, 1.4, 0.0, 0.0, 0.0, 2.0)
	f.Add(-1.11, 1.08, 0.03, 0.03, 0.0, 0.0)
	f.Fuzz(func(t *testing.T, x, y, vx, vy, angle, omega float64) {
		for _, value := range []float64{x, y, vx, vy, angle, omega} {
			if !finite(value) || math.Abs(value) > 100 {
				t.Skip()
			}
		}
		a := toiTestBody(t, motionDef{Shape: Circle{Radius: 0.1}, Position: Vec2{X: x, Y: y}, Velocity: Vec2{X: vx, Y: vy}})
		b := toiTestBody(t, motionDef{Shape: Box{HalfExtents: Vec2{X: 0.2, Y: 1}}, Angle: angle, AngularVelocity: omega})
		const dt = 0.1
		beforeA, beforeB := *a, *b
		impact, hit, err := timeOfImpactProxies(a, b, dt)
		if !reflect.DeepEqual(*a, beforeA) || !reflect.DeepEqual(*b, beforeB) {
			t.Fatal("query mutated input bodies")
		}
		if err != nil {
			if !errors.Is(err, ErrTOINoConvergence) {
				t.Fatalf("bounded finite sweep produced numeric error: %v", err)
			}
			return
		}
		if hit {
			if impact.Time < 0 || impact.Time > dt || !contactFinite(impact.Contact) {
				t.Fatalf("invalid sweep result: %+v", impact)
			}
			pa, _ := toiPose(a, impact.Time)
			pb, _ := toiPose(b, impact.Time)
			witness, err := toiSeparation(pa, pb)
			if err != nil || witness.distance > 4*toiDistanceTolerance(a, b) {
				t.Fatalf("reported contact on separated shapes: distance=%g error=%v", witness.distance, err)
			}
		} else {
			// Sampling cannot prove a miss, but any sampled overlap disproves it.
			for sample := 0; sample <= 16; sample++ {
				time := dt * float64(sample) / 16
				pa, _ := toiPose(a, time)
				pb, _ := toiPose(b, time)
				if contact, overlap := collideProxies(pa, pb); overlap && contact.Penetration > toiDistanceTolerance(a, b) {
					t.Fatalf("sweep missed sampled overlap at %g", time)
				}
			}
		}
	})
}

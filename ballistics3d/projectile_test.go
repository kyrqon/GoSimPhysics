package ballistics3d_test

import (
	"errors"
	"math"
	"reflect"
	"testing"

	"github.com/kyrqon/GoSimPhysics/ballistics"
	"github.com/kyrqon/GoSimPhysics/ballistics3d"
	"github.com/kyrqon/GoSimPhysics/collision3d"
	"github.com/kyrqon/GoSimPhysics/math3d"
)

func shot(t *testing.T, position, velocity math3d.Vec3) *ballistics3d.Projectile {
	t.Helper()
	p, err := ballistics3d.NewProjectile(ballistics3d.ProjectileDef{Position: position, Velocity: velocity, Mass: 2, Damage: 100})
	if err != nil {
		t.Fatal(err)
	}
	return p
}

func box(id string, center, half math3d.Vec3, resistance float64) ballistics3d.Target {
	return ballistics3d.Target{ID: id, Collider: collision3d.Collider{Shape: collision3d.Box{HalfExtents: half}, Position: center}, Material: ballistics.Material{Name: id, Resistance: resistance}}
}

func near(t *testing.T, got, want, tolerance float64) {
	t.Helper()
	if math.IsNaN(got) || math.IsInf(got, 0) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.15g, want %.15g within %g", got, want, tolerance)
	}
}

func step(t *testing.T, p *ballistics3d.Projectile, dt float64, targets []ballistics3d.Target) []ballistics3d.Event {
	t.Helper()
	events, err := p.Step(dt, targets)
	if err != nil {
		t.Fatal(err)
	}
	return events
}

func TestThinWallReducesDamageBeforeNextTarget(t *testing.T) {
	p := shot(t, math3d.Vec3{}, math3d.Vec3{X: 10})
	targets := []ballistics3d.Target{
		box("wall", math3d.Vec3{X: 2}, math3d.Vec3{X: 0.5, Y: 2, Z: 2}, 20),
		box("target", math3d.Vec3{X: 5}, math3d.Vec3{X: 0.5, Y: 2, Z: 2}, 1000),
	}
	beforeTargets := append([]ballistics3d.Target(nil), targets...)
	events := step(t, p, 1, targets)
	if len(events) != 4 {
		t.Fatalf("expected wall hit/exit, target hit/stop, got %+v", events)
	}
	if events[0].Kind != ballistics.Hit || events[1].Kind != ballistics.Exit || events[2].Kind != ballistics.Hit || events[3].Kind != ballistics.Stop {
		t.Fatalf("wrong penetration sequence: %+v", events)
	}
	near(t, events[0].Damage, 100, 1e-12)
	near(t, events[1].EnergySpent, 20, 1e-10)
	near(t, events[1].Energy, 80, 1e-10)
	near(t, events[2].Damage, 80, 1e-10)
	near(t, events[2].Energy, 80, 1e-10)
	near(t, p.State().Position.X, 4.58, 1e-10)
	if !p.State().Stopped || p.State().Velocity != (math3d.Vec3{}) || p.State().Energy != 0 || p.State().Damage != 0 {
		t.Fatalf("thick target failed to stop point: %+v", p.State())
	}
	if !reflect.DeepEqual(events[3].ActiveTargets, []string{"target"}) {
		t.Fatalf("stop did not identify its material: %+v", events[3])
	}
	for i, e := range events {
		if e.Time < 0 || e.Time > 1 || i > 0 && e.Time < events[i-1].Time {
			t.Fatal("events are not in chronological order")
		}
		if e.Kind != ballistics.Hit && e.Damage != 0 {
			t.Fatal("exit/stop would apply duplicate game damage")
		}
	}
	if !reflect.DeepEqual(targets, beforeTargets) {
		t.Fatal("step mutated caller targets")
	}
	position := p.State().Position
	if more := step(t, p, 1, targets); len(more) != 0 || p.State().Position != position || p.State().Time != 2 {
		t.Fatal("stopped projectile moved or repeated damage")
	}
}

func TestThickWallStopsAtEnergyLimitedDepth(t *testing.T) {
	p := shot(t, math3d.Vec3{}, math3d.Vec3{X: 10})
	wall := box("thick", math3d.Vec3{X: 11.5}, math3d.Vec3{X: 10, Y: 2, Z: 2}, 20)
	events := step(t, p, 2, []ballistics3d.Target{wall})
	// Initial E=100; stopping depth=E/R=5 after entering at x=1.5.
	near(t, p.State().Position.X, 6.5, 1e-11)
	if len(events) != 2 || events[0].Kind != ballistics.Hit || events[1].Kind != ballistics.Stop {
		t.Fatalf("thick wall should not emit an exit: %+v", events)
	}
	near(t, events[1].Time, 0.15+1, 1e-11)
}

func TestActualSphereChordAndArbitraryBoxRotation(t *testing.T) {
	t.Run("sphere chord", func(t *testing.T) {
		p := shot(t, math3d.Vec3{Y: 0.6}, math3d.Vec3{X: 10})
		target := ballistics3d.Target{ID: "sphere", Collider: collision3d.Collider{Shape: collision3d.Sphere{Radius: 1}, Position: math3d.Vec3{X: 2}}, Material: ballistics.Material{Resistance: 10}}
		events := step(t, p, 0.5, []ballistics3d.Target{target})
		if len(events) != 2 || events[1].Kind != ballistics.Exit {
			t.Fatalf("sphere chord was not traversed: %+v", events)
		}
		near(t, events[0].Position.X, 1.2, 1e-12)
		near(t, events[1].Position.X, 2.8, 1e-12)
		near(t, events[1].EnergySpent, 16, 1e-10)
		near(t, p.State().Energy, 84, 1e-10)
	})
	t.Run("three axis oblique box", func(t *testing.T) {
		p := shot(t, math3d.Vec3{}, math3d.Vec3{X: 10})
		n := (math3d.Vec3{X: 1, Y: 1, Z: 1}).Normalized()
		x := math3d.Vec3{X: 1}
		target := box("oblique", math3d.Vec3{X: 3.5}, math3d.Vec3{X: 0.5, Y: 10, Z: 10}, 10)
		target.Collider.Rotation = math3d.QuaternionFromAxisAngle(x.Cross(n), math.Acos(x.Dot(n)))
		events := step(t, p, 1, []ballistics3d.Target{target})
		if len(events) != 2 {
			t.Fatalf("oblique box did not have entry and exit: %+v", events)
		}
		thickness := math.Sqrt(3)
		near(t, events[1].Distance-events[0].Distance, thickness, 1e-11)
		near(t, events[1].EnergySpent, 10*thickness, 1e-10)
		near(t, p.State().Energy, 100-10*thickness, 1e-10)
		near(t, events[0].Normal.Sub(n.Mul(-1)).Length(), 0, 1e-12)
	})
}

func TestOverlappingMaterialsAndFramePartitionAreConsistent(t *testing.T) {
	targets := []ballistics3d.Target{
		box("first", math3d.Vec3{X: 2}, math3d.Vec3{X: 1, Y: 2, Z: 2}, 10),
		box("second", math3d.Vec3{X: 3}, math3d.Vec3{X: 1, Y: 2, Z: 2}, 20),
	}
	run := func(count int) (ballistics3d.State, []ballistics3d.Event) {
		p := shot(t, math3d.Vec3{}, math3d.Vec3{X: 10})
		var all []ballistics3d.Event
		for i := 0; i < count; i++ {
			all = append(all, step(t, p, 2/float64(count), targets)...)
		}
		return p.State(), all
	}
	one, oneEvents := run(1)
	many, manyEvents := run(20)
	near(t, one.Energy, 40, 1e-10) // 2*10 + 2*20 of work, including overlap.
	near(t, many.Energy, one.Energy, 1e-10)
	near(t, many.Position.Sub(one.Position).Length(), 0, 1e-10)
	near(t, many.Velocity.Sub(one.Velocity).Length(), 0, 1e-10)
	if len(oneEvents) != 4 || len(manyEvents) != 4 {
		t.Fatalf("frame split repeated/lost entries: %+v / %+v", oneEvents, manyEvents)
	}
	for i, event := range oneEvents {
		other := manyEvents[i]
		if event.Kind != other.Kind || event.TargetID != other.TargetID {
			t.Fatal("frame split changed event order")
		}
		near(t, other.Time, event.Time, 1e-11)
		near(t, other.Damage, event.Damage, 1e-10)
		near(t, other.EnergySpent, event.EnergySpent, 1e-10)
	}
	near(t, oneEvents[1].Damage, 90, 1e-10)
	near(t, oneEvents[2].EnergySpent, 20, 1e-10)
	near(t, oneEvents[3].EnergySpent, 40, 1e-10)
}

func TestRigidTransformPreservesPenetrationAndCrossFrameEntries(t *testing.T) {
	rotation := math3d.QuaternionFromAxisAngle(math3d.Vec3{X: 1, Y: 2, Z: 3}, 0.73)
	translation := math3d.Vec3{X: -4, Y: 7, Z: 3}
	targets := []ballistics3d.Target{
		box("wall", math3d.Vec3{X: 2}, math3d.Vec3{X: 0.5, Y: 2, Z: 2}, 20),
	}
	rotatedTargets := append([]ballistics3d.Target(nil), targets...)
	rotatedTargets[0].Collider.Position = rotation.Rotate(targets[0].Collider.Position).Add(translation)
	rotatedTargets[0].Collider.Rotation = rotation
	plain := shot(t, math3d.Vec3{}, math3d.Vec3{X: 10})
	rotated := shot(t, translation, rotation.Rotate(math3d.Vec3{X: 10}))
	var plainEvents, rotatedEvents []ballistics3d.Event
	for i := 0; i < 100; i++ {
		plainEvents = append(plainEvents, step(t, plain, 0.01, targets)...)
		rotatedEvents = append(rotatedEvents, step(t, rotated, 0.01, rotatedTargets)...)
	}
	if len(plainEvents) != 2 || len(rotatedEvents) != 2 {
		t.Fatalf("rotating/translating the scene duplicated entry or exit: %v / %v", plainEvents, rotatedEvents)
	}
	near(t, rotated.State().Energy, plain.State().Energy, 1e-10)
	expectedPosition := rotation.Rotate(plain.State().Position).Add(translation)
	near(t, rotated.State().Position.Sub(expectedPosition).Length(), 0, 1e-10)
	for i, event := range plainEvents {
		transformed := rotatedEvents[i]
		if event.Kind != transformed.Kind || event.TargetID != transformed.TargetID {
			t.Fatal("rigid transform changed material crossing order")
		}
		near(t, transformed.Time, event.Time, 1e-10)
		near(t, transformed.Position.Sub(rotation.Rotate(event.Position).Add(translation)).Length(), 0, 1e-10)
		near(t, transformed.Normal.Sub(rotation.Rotate(event.Normal)).Length(), 0, 1e-10)
	}
}

func TestStartsInsideBoundaryAndGrazingSemantics(t *testing.T) {
	wall := box("wall", math3d.Vec3{}, math3d.Vec3{X: 1, Y: 1, Z: 1}, 10)
	t.Run("starts inside", func(t *testing.T) {
		p := shot(t, math3d.Vec3{}, math3d.Vec3{X: 10})
		all := step(t, p, 0.02, []ballistics3d.Target{wall})
		all = append(all, step(t, p, 0.02, []ballistics3d.Target{wall})...)
		all = append(all, step(t, p, 0.5, []ballistics3d.Target{wall})...)
		if len(all) != 2 || !all[0].StartedInside || all[0].Normal != (math3d.Vec3{}) {
			t.Fatalf("starting inside must hit once with no entry face: %+v", all)
		}
		near(t, all[1].Position.X, 1, 1e-12)
		near(t, p.State().Energy, 90, 1e-10)
	})
	t.Run("starts at exit", func(t *testing.T) {
		p := shot(t, math3d.Vec3{X: 1}, math3d.Vec3{X: 10})
		if events := step(t, p, 0.1, []ballistics3d.Target{wall}); len(events) != 0 {
			t.Fatalf("outward boundary launch damaged material: %+v", events)
		}
		near(t, p.State().Energy, 100, 1e-12)
	})
	for _, target := range []ballistics3d.Target{
		wall,
		{ID: "sphere", Collider: collision3d.Collider{Shape: collision3d.Sphere{Radius: 1}}, Material: ballistics.Material{Resistance: 10}},
	} {
		t.Run("grazing "+target.ID, func(t *testing.T) {
			p := shot(t, math3d.Vec3{X: -2, Y: 1}, math3d.Vec3{X: 10})
			if events := step(t, p, 0.5, []ballistics3d.Target{target}); len(events) != 0 {
				t.Fatalf("boundary-only path generated material events: %+v", events)
			}
			near(t, p.State().Energy, 100, 1e-12)
		})
	}
}

func TestRemovedTargetExitsWithoutRepeatedDamage(t *testing.T) {
	p := shot(t, math3d.Vec3{}, math3d.Vec3{X: 10})
	wall := box("wall", math3d.Vec3{}, math3d.Vec3{X: 5, Y: 1, Z: 1}, 10)
	first := step(t, p, 0.02, []ballistics3d.Target{wall})
	if len(first) != 1 || first[0].Kind != ballistics.Hit {
		t.Fatal("fixture did not start inside wall")
	}
	energy, position, time := p.State().Energy, p.State().Position, p.State().Time
	more := step(t, p, 0.02, nil)
	if len(more) != 1 || more[0].Kind != ballistics.Exit || more[0].Damage != 0 || more[0].Normal != (math3d.Vec3{}) || more[0].Position != position {
		t.Fatalf("target removal must report a zero-time exit: %+v", more)
	}
	near(t, more[0].Time, time, 1e-12)
	near(t, p.State().Energy, energy, 1e-12)
}

func TestValidationBudgetAndOverflowAreAtomic(t *testing.T) {
	p := shot(t, math3d.Vec3{}, math3d.Vec3{X: 10})
	wall := box("wall", math3d.Vec3{X: 2}, math3d.Vec3{X: 0.5, Y: 1, Z: 1}, 10)
	badCases := [][]ballistics3d.Target{
		{wall, wall},
		{{ID: "", Collider: wall.Collider}},
		{{ID: "bad", Collider: collision3d.Collider{Shape: collision3d.Box{}}, Material: wall.Material}},
		{{ID: "bad", Collider: wall.Collider, Material: ballistics.Material{Resistance: math.Inf(1)}}},
		{wall, {ID: "late", Collider: collision3d.Collider{Shape: collision3d.Sphere{Radius: -1}, Position: math3d.Vec3{X: 100}}}},
	}
	before := p.State()
	for _, targets := range badCases {
		events, err := p.Step(1, targets)
		if err == nil || len(events) != 0 || p.State() != before {
			t.Fatalf("invalid targets mutated projectile: %+v, %v", events, err)
		}
	}
	for _, dt := range []float64{0, -1, math.NaN(), math.Inf(1)} {
		if events, err := p.Step(dt, nil); err == nil || len(events) != 0 || p.State() != before {
			t.Fatal("invalid duration changed state")
		}
	}
	p.MaxEvents = 1
	if events, err := p.Step(1, []ballistics3d.Target{wall}); !errors.Is(err, ballistics.ErrEventLimit) || len(events) != 0 || p.State() != before {
		t.Fatalf("event budget must roll back entry history: %+v, %v", events, err)
	}
	p.MaxEvents = ballistics.DefaultMaxEvents
	if events := step(t, p, 1, []ballistics3d.Target{wall}); len(events) != 2 || events[0].Kind != ballistics.Hit {
		t.Fatal("retry lost an entry event")
	}
	copy := p.State()
	copy.Position.X = -1234
	if p.State().Position == copy.Position {
		t.Fatal("state snapshot aliased the projectile")
	}
	fast, err := ballistics3d.NewProjectile(ballistics3d.ProjectileDef{Velocity: math3d.Vec3{X: math.MaxFloat64 / 2}, Mass: 1e-308})
	if err != nil {
		t.Fatal(err)
	}
	fastBefore := fast.State()
	if _, err := fast.Step(8, nil); err == nil || fast.State() != fastBefore {
		t.Fatal("unrepresentable motion was accepted or partially committed")
	}
}

func TestLaunchValidation(t *testing.T) {
	valid := ballistics3d.ProjectileDef{Velocity: math3d.Vec3{X: 10}, Mass: 1, Damage: 100}
	changes := []func(*ballistics3d.ProjectileDef){
		func(d *ballistics3d.ProjectileDef) { d.Position.Z = math.NaN() },
		func(d *ballistics3d.ProjectileDef) { d.Velocity = math3d.Vec3{} },
		func(d *ballistics3d.ProjectileDef) { d.Velocity.Y = math.Inf(1) },
		func(d *ballistics3d.ProjectileDef) { d.Mass = 0 },
		func(d *ballistics3d.ProjectileDef) { d.Damage = -1 },
		func(d *ballistics3d.ProjectileDef) { d.MaxEvents = -1 },
	}
	for _, change := range changes {
		def := valid
		change(&def)
		if p, err := ballistics3d.NewProjectile(def); err == nil || p != nil {
			t.Fatalf("invalid launch accepted: %+v", def)
		}
	}
}

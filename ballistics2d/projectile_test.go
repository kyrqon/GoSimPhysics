package ballistics2d

import (
	"errors"
	"math"
	"reflect"
	"testing"

	"github.com/kyrqon/GoSimPhysics/ballistics"
	"github.com/kyrqon/GoSimPhysics/collision2d"
	"github.com/kyrqon/GoSimPhysics/math2d"
)

func testProjectile(t *testing.T, position, velocity math2d.Vec2) *Projectile {
	t.Helper()
	p, err := NewProjectile(ProjectileDef{Position: position, Velocity: velocity, Mass: 2, Damage: 100})
	if err != nil {
		t.Fatal(err)
	}
	return p
}

func testWall(id string, start, end, resistance float64) Target {
	return Target{
		ID: id,
		Collider: collision2d.Collider{
			Shape:    collision2d.Box{HalfExtents: math2d.Vec2{X: (end - start) / 2, Y: 1}},
			Position: math2d.Vec2{X: (start + end) / 2},
		},
		Material: ballistics.Material{Name: id, Resistance: resistance},
	}
}

func testNear(t *testing.T, got, want, tolerance float64) {
	t.Helper()
	if math.IsNaN(got) || math.IsInf(got, 0) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.15g, want %.15g ±%g", got, want, tolerance)
	}
}

func testStep(t *testing.T, p *Projectile, dt float64, targets []Target) []Event {
	t.Helper()
	events, err := p.Step(dt, targets)
	if err != nil {
		t.Fatal(err)
	}
	return events
}

func TestThinWallReducesFollowingHitDamageAndThickTargetStops(t *testing.T) {
	p := testProjectile(t, math2d.Vec2{X: -1}, math2d.Vec2{X: 10})
	targets := []Target{testWall("thin", 0, 1, 20), testWall("target", 3, 3.5, 1000)}
	events := testStep(t, p, 1, targets)
	if len(events) != 4 {
		t.Fatalf("expected hit, exit, hit, stop; got %+v", events)
	}
	wantKinds := []EventKind{Hit, Exit, Hit, Stop}
	for i, kind := range wantKinds {
		if events[i].Kind != kind {
			t.Fatalf("event %d kind=%v, want %v", i, events[i].Kind, kind)
		}
		if kind != Hit && events[i].Damage != 0 {
			t.Fatal("exit or stop must not charge hit damage again")
		}
	}
	testNear(t, events[0].Damage, 100, 1e-12)
	testNear(t, events[0].Time, 0.1, 1e-12)
	testNear(t, events[1].Energy, 80, 1e-11)
	testNear(t, events[1].EnergySpent, 20, 1e-11)
	testNear(t, events[2].Damage, 80, 1e-11)
	testNear(t, events[2].Position.X, 3, 1e-12)
	testNear(t, events[3].Position.X, 3.08, 1e-11)
	if !reflect.DeepEqual(events[3].ActiveTargets, []string{"target"}) {
		t.Fatalf("stop did not identify its active target: %v", events[3].ActiveTargets)
	}
	if events[0].Normal != (math2d.Vec2{X: -1}) || events[1].Normal != (math2d.Vec2{X: 1}) {
		t.Fatalf("wrong boundary normals: %v, %v", events[0].Normal, events[1].Normal)
	}
	state := p.State()
	if !state.Stopped || state.Energy != 0 || state.Damage != 0 || state.Velocity != (math2d.Vec2{}) {
		t.Fatalf("shot did not stop: %+v", state)
	}
	testNear(t, state.Position.X, 3.08, 1e-11)
	testNear(t, state.Distance, 4.08, 1e-11)
	testNear(t, state.Time, 1, 1e-12)
	if more := testStep(t, p, 2, targets); len(more) != 0 || p.State().Position != state.Position {
		t.Fatal("stopped shot produced more impacts or moved")
	}
	testNear(t, p.State().Time, 3, 1e-12)
}

func TestThickWallStopsInsideAtEnergyOverResistanceDistance(t *testing.T) {
	p := testProjectile(t, math2d.Vec2{X: -1}, math2d.Vec2{X: 10})
	events := testStep(t, p, 2, []Target{testWall("thick", 0, 10, 20)})
	if len(events) != 2 || events[0].Kind != Hit || events[1].Kind != Stop {
		t.Fatalf("thick wall must stop rather than exit: %+v", events)
	}
	testNear(t, p.State().Position.X, 5, 1e-12) // E/F = 100/20.
	testNear(t, events[1].Time, 1.1, 1e-12)     // .1 s flight plus mv/F = 1 s.
}

func TestMaterialEntryPersistsAcrossFramesAndMatchesSingleStep(t *testing.T) {
	targets := []Target{testWall("wall", 0, 2, 10)}
	whole := testProjectile(t, math2d.Vec2{X: -1}, math2d.Vec2{X: 10})
	all := testStep(t, whole, 0.4, targets)
	split := testProjectile(t, math2d.Vec2{X: -1}, math2d.Vec2{X: 10})
	var partitioned []Event
	partitioned = append(partitioned, testStep(t, split, 0.15, targets)...)
	if len(partitioned) != 1 || partitioned[0].Kind != Hit {
		t.Fatal("first partial traversal must report one entry")
	}
	testNear(t, split.State().Position.X, 0.49375, 1e-12)
	testNear(t, split.State().Velocity.X, 9.75, 1e-12)
	if events := testStep(t, split, 0.15, targets); len(events) != 0 {
		t.Fatalf("continuing inside charged another entry: %+v", events)
	}
	partitioned = append(partitioned, testStep(t, split, 0.1, targets)...)
	if len(all) != 2 || len(partitioned) != 2 || partitioned[1].Kind != Exit {
		t.Fatalf("unexpected complete traversal: %v, %v", all, partitioned)
	}
	for _, pair := range [][2]float64{
		{whole.State().Position.X, split.State().Position.X},
		{whole.State().Velocity.X, split.State().Velocity.X},
		{whole.State().Energy, split.State().Energy},
		{all[1].Time, partitioned[1].Time},
		{all[1].EnergySpent, partitioned[1].EnergySpent},
	} {
		testNear(t, pair[0], pair[1], 1e-11)
	}
	testNear(t, partitioned[1].EnergySpent, 20, 1e-11)
}

func TestAngledWallUsesActualPathThickness(t *testing.T) {
	p := testProjectile(t, math2d.Vec2{X: -3}, math2d.Vec2{X: 10})
	wall := Target{ID: "angle", Collider: collision2d.Collider{
		Shape: collision2d.Box{HalfExtents: math2d.Vec2{X: 0.5, Y: 3}}, Angle: math.Pi / 3,
	}, Material: ballistics.Material{Resistance: 10}}
	events := testStep(t, p, 1, []Target{wall})
	if len(events) != 2 || events[1].Kind != Exit {
		t.Fatalf("expected traversal: %+v", events)
	}
	// The one-unit normal thickness becomes two units along this ray.
	testNear(t, events[1].EnergySpent, 20, 1e-10)
	testNear(t, p.State().Energy, 80, 1e-10)
}

func TestOverlappingMaterialsAddWorkWithoutDuplicatingDamage(t *testing.T) {
	p := testProjectile(t, math2d.Vec2{X: -1}, math2d.Vec2{X: 10})
	events := testStep(t, p, 1, []Target{testWall("b", 1, 3, 20), testWall("a", 0, 2, 10)})
	if len(events) != 4 {
		t.Fatalf("expected two entries and exits: %+v", events)
	}
	if events[0].TargetID != "a" || events[1].TargetID != "b" || events[2].TargetID != "a" || events[3].TargetID != "b" {
		t.Fatalf("events were not ordered geometrically: %+v", events)
	}
	testNear(t, events[1].Damage, 90, 1e-10)
	testNear(t, events[2].EnergySpent, 20, 1e-10)
	testNear(t, events[3].EnergySpent, 40, 1e-10)
	testNear(t, p.State().Energy, 40, 1e-10)
}

func TestStartsInsideAndBoundaryDirections(t *testing.T) {
	inside := testProjectile(t, math2d.Vec2{}, math2d.Vec2{X: 10})
	events := testStep(t, inside, 0.2, []Target{testWall("wall", -1, 1, 10)})
	if len(events) != 2 || !events[0].StartedInside || events[0].Position != (math2d.Vec2{}) || events[0].Normal != (math2d.Vec2{}) {
		t.Fatalf("invalid starts-inside events: %+v", events)
	}
	testNear(t, events[1].EnergySpent, 10, 1e-11)
	wall := []Target{testWall("wall", 0, 1, 10)}
	entering := testProjectile(t, math2d.Vec2{}, math2d.Vec2{X: 10})
	if e := testStep(t, entering, 0.01, wall); len(e) != 1 || e[0].Kind != Hit || e[0].StartedInside {
		t.Fatalf("boundary entry missing: %+v", e)
	}
	if e := testStep(t, entering, 0.01, wall); len(e) != 0 {
		t.Fatalf("boundary entry duplicated next frame: %+v", e)
	}
	leaving := testProjectile(t, math2d.Vec2{X: 1}, math2d.Vec2{X: 10})
	if e := testStep(t, leaving, 0.1, wall); len(e) != 0 {
		t.Fatalf("outgoing boundary must not count as an entry: %+v", e)
	}
	testNear(t, leaving.State().Energy, 100, 1e-12)
}

func TestGrazingAndVacuumDoNotConsumeEnergy(t *testing.T) {
	for _, shape := range []collision2d.Shape{
		collision2d.Box{HalfExtents: math2d.Vec2{X: 0.5, Y: 0.5}},
		collision2d.Circle{Radius: 0.5},
	} {
		p := testProjectile(t, math2d.Vec2{X: -2, Y: 0.5}, math2d.Vec2{X: 10})
		events := testStep(t, p, 0.5, []Target{{ID: "tangent", Collider: collision2d.Collider{Shape: shape}, Material: ballistics.Material{Resistance: 1e6}}})
		if len(events) != 0 {
			t.Fatalf("boundary-only grazing created material events: %+v", events)
		}
		testNear(t, p.State().Energy, 100, 1e-12)
		testNear(t, p.State().Position.X, 3, 1e-12)
	}
	p := testProjectile(t, math2d.Vec2{X: -1}, math2d.Vec2{X: 10})
	events := testStep(t, p, 0.5, []Target{testWall("transparent", 0, 1, 0)})
	if len(events) != 2 || events[1].EnergySpent != 0 {
		t.Fatalf("zero resistance must keep boundary events without energy loss: %+v", events)
	}
	testNear(t, p.State().Energy, 100, 1e-12)
}

func TestInvalidInputsAndBudgetErrorsAreAtomic(t *testing.T) {
	p := testProjectile(t, math2d.Vec2{X: -1}, math2d.Vec2{X: 10})
	before := p.State()
	for _, targets := range [][]Target{
		{testWall("good", 0, 1, 10), {ID: "bad", Collider: collision2d.Collider{Shape: collision2d.Circle{Radius: -1}}}},
		{testWall("duplicate", 0, 1, 10), testWall("duplicate", 2, 3, 10)},
		{testWall("", 0, 1, 10)},
		{testWall("bad resistance", 0, 1, -1)},
	} {
		events, err := p.Step(1, targets)
		if err == nil || events != nil || p.State() != before {
			t.Fatalf("invalid targets were committed: events=%v err=%v", events, err)
		}
	}
	for _, dt := range []float64{0, -1, math.NaN(), math.Inf(1)} {
		if e, err := p.Step(dt, nil); err == nil || e != nil || p.State() != before {
			t.Fatal("invalid time step changed state")
		}
	}
	p.MaxEvents = 1
	if e, err := p.Step(1, []Target{testWall("wall", 0, 1, 10)}); !errors.Is(err, ballistics.ErrEventLimit) || e != nil || p.State() != before {
		t.Fatalf("event-budget failure was not atomic: events=%v err=%v", e, err)
	}
	p.MaxEvents = DefaultMaxEvents
	if e := testStep(t, p, 1, []Target{testWall("wall", 0, 1, 10)}); len(e) != 2 {
		t.Fatal("retry after failure did not reproduce the complete event sequence")
	}
}

func TestConstructorValidationAndSnapshotIsolation(t *testing.T) {
	valid := ProjectileDef{Velocity: math2d.Vec2{X: 1}, Mass: 1, Damage: 1}
	for _, change := range []func(*ProjectileDef){
		func(d *ProjectileDef) { d.Mass = 0 },
		func(d *ProjectileDef) { d.Mass = math.Inf(1) },
		func(d *ProjectileDef) { d.Velocity = math2d.Vec2{} },
		func(d *ProjectileDef) { d.Velocity.Y = math.NaN() },
		func(d *ProjectileDef) { d.Position.X = math.Inf(1) },
		func(d *ProjectileDef) { d.Damage = -1 },
		func(d *ProjectileDef) { d.MaxEvents = -1 },
	} {
		def := valid
		change(&def)
		if p, err := NewProjectile(def); p != nil || err == nil {
			t.Fatal("invalid projectile definition accepted")
		}
	}
	p := testProjectile(t, math2d.Vec2{}, math2d.Vec2{X: 10})
	copy := p.State()
	copy.Position.X = 1e6
	copy.Velocity.X = 0
	if p.State().Position != (math2d.Vec2{}) || p.State().Velocity.X != 10 {
		t.Fatal("snapshot mutation changed the projectile")
	}
	var nilProjectile *Projectile
	if events, err := nilProjectile.Step(1, nil); err == nil || events != nil {
		t.Fatal("nil projectile accepted")
	}
}

func TestRotatedBoundaryPartitionsDoNotDuplicateEntry(t *testing.T) {
	targets := []Target{{ID: "rotated", Collider: collision2d.Collider{
		Shape: collision2d.Box{HalfExtents: math2d.Vec2{X: 0.5, Y: 2}}, Angle: math.Pi / 3,
	}, Material: ballistics.Material{Resistance: 10}}}
	whole := testProjectile(t, math2d.Vec2{X: -3}, math2d.Vec2{X: 10})
	wantEvents := testStep(t, whole, 0.8, targets)
	if len(wantEvents) != 2 {
		t.Fatalf("fixture did not cross both boundaries: %+v", wantEvents)
	}
	split := testProjectile(t, math2d.Vec2{X: -3}, math2d.Vec2{X: 10})
	var gotEvents []Event
	// End the first frame exactly at the original ray's entry, then traverse
	// with many small frames. Recasting incrementally from rounded positions
	// used to risk producing another entry at this rotated boundary.
	gotEvents = append(gotEvents, testStep(t, split, wantEvents[0].Time, targets)...)
	remaining := 0.8 - wantEvents[0].Time
	for i := 0; i < 120; i++ {
		gotEvents = append(gotEvents, testStep(t, split, remaining/120, targets)...)
	}
	if len(gotEvents) != 2 || gotEvents[0].Kind != Hit || gotEvents[1].Kind != Exit {
		t.Fatalf("partitioning duplicated a boundary: %+v", gotEvents)
	}
	testNear(t, split.State().Position.X, whole.State().Position.X, 1e-10)
	testNear(t, split.State().Energy, whole.State().Energy, 1e-10)
	testNear(t, gotEvents[1].EnergySpent, wantEvents[1].EnergySpent, 1e-10)
}

func TestTargetsCanBeRemovedOrAddedAroundInFlightProjectile(t *testing.T) {
	p := testProjectile(t, math2d.Vec2{X: -1}, math2d.Vec2{X: 10})
	testStep(t, p, 0.15, []Target{testWall("wall", 0, 2, 10)})
	before := p.State()
	events := testStep(t, p, 0.05, nil)
	if len(events) != 1 || events[0].Kind != Exit || events[0].TargetID != "wall" || events[0].Normal != (math2d.Vec2{}) {
		t.Fatalf("removing a material did not finish its interval: %+v", events)
	}
	testNear(t, events[0].Time, before.Time, 1e-12)
	testNear(t, p.State().Energy, before.Energy, 1e-12)
	testNear(t, p.State().Position.X, before.Position.X+before.Velocity.X*0.05, 1e-12)

	newTarget := testProjectile(t, math2d.Vec2{X: -2}, math2d.Vec2{X: 10})
	testStep(t, newTarget, 0.2, nil)
	events = testStep(t, newTarget, 0.01, []Target{testWall("added", -1, 1, 10)})
	if len(events) != 1 || events[0].Kind != Hit || !events[0].StartedInside || events[0].Normal != (math2d.Vec2{}) {
		t.Fatalf("new enclosing target must be a starts-inside hit: %+v", events)
	}
	testNear(t, events[0].Position.X, 0, 1e-12)
}

func TestCoordinateOverflowReturnsNoPartialEventsOrState(t *testing.T) {
	p, err := NewProjectile(ProjectileDef{
		Position: math2d.Vec2{X: math.MaxFloat64 / 2}, Velocity: math2d.Vec2{X: math.MaxFloat64 / 2},
		Mass: 1e-308, Damage: 10,
	})
	if err != nil {
		t.Fatal(err)
	}
	before := p.State()
	if events, err := p.Step(2, nil); err == nil || events != nil || p.State() != before {
		t.Fatalf("unrepresentable coordinate advance was partially committed: %v, %v", events, err)
	}
}

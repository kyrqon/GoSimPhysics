package penetration

import (
	"errors"
	"math"
	"math/rand"
	"reflect"
	"testing"

	"github.com/kyrqon/GoSimPhysics/ballistics"
)

func testLaunch(t *testing.T, mass, speed float64) State {
	t.Helper()
	state, err := NewState(mass, speed, 100)
	if err != nil {
		t.Fatal(err)
	}
	return state
}

func testAdvance(t *testing.T, state State, intervals []Interval, dt float64) (State, []Event) {
	t.Helper()
	after, _, events, err := Advance(state, intervals, dt, 64)
	if err != nil {
		t.Fatal(err)
	}
	return after, events
}

func near(t *testing.T, got, want, tolerance float64) {
	t.Helper()
	if math.IsNaN(got) || math.IsInf(got, 0) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.15g, want %.15g (tolerance %g)", got, want, tolerance)
	}
}

func TestExactFrameBoundariesDoNotRepeatDamage(t *testing.T) {
	intervals := []Interval{{ID: "glass", Entry: 1, Exit: 2}}
	state := testLaunch(t, 1, 10)
	state, events := testAdvance(t, state, intervals, 0.1)
	if len(events) != 1 || events[0].Kind != ballistics.Hit || events[0].Damage != 100 || state.Distance != 1 {
		t.Fatalf("entry boundary was not handled exactly: %+v, %+v", state, events)
	}
	state, events = testAdvance(t, state, intervals, 0.1)
	if len(events) != 1 || events[0].Kind != ballistics.Exit || events[0].Damage != 0 || state.Distance != 2 || len(state.Inside) != 0 {
		t.Fatalf("cross-frame occupancy repeated or lost an event: %+v, %+v", state, events)
	}
	state, events = testAdvance(t, state, intervals, 0.1)
	if len(events) != 0 || state.Distance != 3 {
		t.Fatalf("exit repeated on a later frame: %+v", events)
	}
}

func TestAdjacentMaterialsExitBeforeNextDamage(t *testing.T) {
	intervals := []Interval{
		{ID: "z-barrier", Entry: 0, Exit: 1, Material: ballistics.Material{Resistance: 10}},
		{ID: "a-target", Entry: 1, Exit: 2, Material: ballistics.Material{Resistance: 20}},
	}
	after, events := testAdvance(t, testLaunch(t, 1, 10), intervals, 1)
	if len(events) != 4 || events[1].Kind != ballistics.Exit || events[1].TargetID != "z-barrier" || events[2].Kind != ballistics.Hit || events[2].TargetID != "a-target" {
		t.Fatalf("shared-face event order: %+v", events)
	}
	near(t, events[1].Time, events[2].Time, 0)
	near(t, events[2].Damage, 80, 1e-12)
	near(t, events[1].EnergySpent, 10, 1e-12)
	near(t, events[3].EnergySpent, 20, 1e-12)
	near(t, after.Energy, 20, 1e-12)
}

func TestZeroEnergyAtFarFaceStopsBeforeExitOrNextHit(t *testing.T) {
	intervals := []Interval{
		{ID: "wall", Entry: 0, Exit: 0.5, Material: ballistics.Material{Resistance: 100}},
		{ID: "target", Entry: 0.5, Exit: 1},
	}
	after, events := testAdvance(t, testLaunch(t, 1, 10), intervals, 0.2)
	if len(events) != 2 || events[0].Kind != ballistics.Hit || events[1].Kind != ballistics.Stop || events[1].TargetID != "wall" || !after.Stopped {
		t.Fatalf("zero-energy penetration or duplicate damage: %+v", events)
	}
	near(t, events[1].Time, 0.1, 1e-15)
	near(t, after.Distance, 0.5, 0)
	near(t, events[1].EnergySpent, 50, 1e-12)
	near(t, events[1].Damage, 0, 0)
	if !reflect.DeepEqual(events[1].ActiveTargets, []string{"wall"}) {
		t.Fatalf("wrong stop ownership: %v", events[1].ActiveTargets)
	}
	later, events := testAdvance(t, after, intervals, 0.1)
	if len(events) != 0 || later.Distance != after.Distance || later.Energy != 0 {
		t.Fatal("stopped projectile emitted more crossings")
	}
	near(t, later.Time, 0.3, 1e-15)
}

func TestFailureDoesNotMutateMaterialHistoryOrConsumeEvents(t *testing.T) {
	intervals := []Interval{
		{ID: "inside", Entry: 0, Exit: 4, Material: ballistics.Material{Resistance: 1}},
		{ID: "later", Entry: 2, Exit: 3, Material: ballistics.Material{Resistance: 2}},
	}
	state, _ := testAdvance(t, testLaunch(t, 1, 10), intervals, 0.1)
	copyInside := make(map[string]Record)
	for id, record := range state.Inside {
		copyInside[id] = record
	}
	_, distance, events, err := Advance(state, intervals, 0.4, 1)
	if !errors.Is(err, ballistics.ErrEventLimit) || len(events) != 0 || distance != 0 || !reflect.DeepEqual(state.Inside, copyInside) {
		t.Fatalf("event failure escaped rollback: %v, %+v", err, events)
	}
	_, events = testAdvance(t, state, intervals, 0.4)
	for _, event := range events {
		if event.Kind == ballistics.Hit && event.TargetID == "inside" {
			t.Fatal("retry applied old entry damage again")
		}
	}
}

func TestOverlappingRandomIntervalsMatchIndependentWorkAndFramePartition(t *testing.T) {
	rng := rand.New(rand.NewSource(20261107))
	for trial := 0; trial < 100; trial++ {
		intervals := make([]Interval, 8)
		workAt := func(distance float64) float64 {
			work := 0.0
			for _, interval := range intervals {
				work += interval.Material.Resistance * math.Max(0, math.Min(distance, interval.Exit)-interval.Entry)
			}
			return work
		}
		for i := range intervals {
			entry := rng.Float64() * 5
			intervals[i] = Interval{ID: string(rune('A' + i)), Entry: entry, Exit: entry + 0.1 + rng.Float64(), Material: ballistics.Material{Resistance: rng.Float64() * 20}}
		}
		start := testLaunch(t, 2, 60)
		whole, expectedEvents := testAdvance(t, start, intervals, 1)
		near(t, whole.Energy, start.Energy-workAt(10), 1e-9)
		for _, event := range expectedEvents {
			if event.Kind == ballistics.Hit {
				near(t, event.Energy, start.Energy-workAt(event.Distance), 1e-9)
			}
			if event.Kind == ballistics.Exit {
				for _, interval := range intervals {
					if interval.ID == event.TargetID {
						near(t, event.EnergySpent, interval.Material.Resistance*(interval.Exit-interval.Entry), 1e-9)
					}
				}
			}
		}
		split := start
		var splitEvents []Event
		for frame := 0; frame < 100; frame++ {
			var batch []Event
			split, batch = testAdvance(t, split, intervals, 0.01)
			splitEvents = append(splitEvents, batch...)
		}
		near(t, split.Distance, whole.Distance, 1e-9)
		near(t, split.Energy, whole.Energy, 1e-9)
		if len(splitEvents) != len(expectedEvents) {
			t.Fatalf("trial %d repeated a frame-boundary crossing", trial)
		}
		for i, event := range splitEvents {
			want := expectedEvents[i]
			if event.Kind != want.Kind || event.TargetID != want.TargetID {
				t.Fatalf("trial %d event order differs", trial)
			}
			near(t, event.Time, want.Time, 1e-9)
			near(t, event.Damage, want.Damage, 1e-9)
		}
	}
}

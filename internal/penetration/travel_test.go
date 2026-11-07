package penetration

import (
	"math"
	"math/rand"
	"testing"
)

func travelNear(t testing.TB, got, want, relative float64) {
	t.Helper()
	tolerance := math.Max(math.Abs(want)*relative, math.SmallestNonzeroFloat64*4)
	if math.IsNaN(got) || math.IsInf(got, 0) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.17g, want %.17g (tolerance %g)", got, want, tolerance)
	}
}

func TestTravelBoundaryStopAndTimeLimits(t *testing.T) {
	for _, test := range []struct {
		name                string
		resistance, dt, max float64
		distance, duration  float64
		energy, speed       float64
		reached, stopped    bool
	}{
		{"thin material", 100, 1, 0.5, 0.5, 1 / (20 + math.Sqrt(300)), 150, math.Sqrt(300), true, false},
		{"thick material", 100, 1, 3, 2, 0.2, 0, 0, false, true},
		{"exact threshold", 100, 1, 2, 2, 0.2, 0, 0, true, true},
		{"partial interior", 100, 0.05, 3, 0.875, 0.05, 112.5, 15, false, false},
		{"boundary at exact time", 100, 0.1, 1.5, 1.5, 0.1, 50, 10, true, false},
		{"vacuum boundary", 0, 1, 5, 5, 0.25, 200, 20, true, false},
		{"vacuum frame", 0, 0.1, 5, 2, 0.1, 200, 20, false, false},
		{"zero boundary", 100, 0.1, 0, 0, 0, 200, 20, true, false},
	} {
		t.Run(test.name, func(t *testing.T) {
			got, err := advanceTravel(1, 200, test.resistance, test.dt, test.max)
			if err != nil {
				t.Fatal(err)
			}
			travelNear(t, got.Distance, test.distance, 2e-14)
			travelNear(t, got.Time, test.duration, 2e-14)
			travelNear(t, got.Energy, test.energy, 2e-14)
			travelNear(t, got.Speed, test.speed, 2e-14)
			if got.Reached != test.reached || got.Stopped != test.stopped {
				t.Fatalf("flags = reached:%v stopped:%v, want %v %v", got.Reached, got.Stopped, test.reached, test.stopped)
			}
			travelNear(t, got.Energy+test.resistance*got.Distance, 200, 2e-14)
		})
	}
	stopped, err := advanceTravel(1, 0, 100, 1, math.Inf(1))
	if err != nil || !stopped.Stopped || stopped.Distance != 0 || stopped.Time != 0 {
		t.Fatalf("zero energy moved: %+v, %v", stopped, err)
	}
}

func TestTravelIsIndependentOfTimePartition(t *testing.T) {
	random := rand.New(rand.NewSource(2048))
	for fixture := 0; fixture < 100; fixture++ {
		mass := 0.25 + random.Float64()*3
		energy := 20 + random.Float64()*500
		resistance := random.Float64() * 200
		duration := 0.02 + random.Float64()*2
		whole, err := advanceTravel(mass, energy, resistance, duration, math.Inf(1))
		if err != nil {
			t.Fatal(err)
		}
		remaining, distance, elapsed := energy, 0.0, 0.0
		for part := 0; part < 19; part++ {
			segment, err := advanceTravel(mass, remaining, resistance, duration/19, math.Inf(1))
			if err != nil {
				t.Fatalf("fixture %d part %d: %v", fixture, part, err)
			}
			distance += segment.Distance
			elapsed += segment.Time
			remaining = segment.Energy
		}
		travelNear(t, distance, whole.Distance, 2e-12)
		travelNear(t, elapsed, whole.Time, 2e-12)
		travelNear(t, remaining, whole.Energy, 2e-12)
	}
}

func TestTravelRetainsRepresentableMotionAtExtremeScales(t *testing.T) {
	// stopTime=1e309 cannot be represented, but a 1e308-second partial interval
	// still loses 19 percent of its energy and travels a finite 9.5e307 units.
	got, err := advanceTravel(1e308, 5e307, 0.1, 1e308, math.Inf(1))
	if err != nil {
		t.Fatal(err)
	}
	travelNear(t, got.Distance, 9.5e307, 2e-15)
	travelNear(t, got.Energy, 4.05e307, 2e-15)
	travelNear(t, got.Speed, 0.9, 2e-15)
	if got.Reached || got.Stopped {
		t.Fatal("finite partial interval incorrectly reached an event")
	}
	if _, err := advanceTravel(1, 5e307, 0, 1e155, math.Inf(1)); err == nil {
		t.Fatal("accepted an overflowing actual travel distance")
	}
}

func TestTravelRejectsInvalidNumbers(t *testing.T) {
	for _, bad := range []float64{-1, math.NaN(), math.Inf(1)} {
		for _, input := range [][5]float64{{bad, 10, 1, 1, 1}, {1, bad, 1, 1, 1}, {1, 10, bad, 1, 1}, {1, 10, 1, bad, 1}} {
			if _, err := advanceTravel(input[0], input[1], input[2], input[3], input[4]); err == nil {
				t.Fatalf("invalid travel accepted: %v", input)
			}
		}
	}
	for _, distance := range []float64{-1, math.NaN(), math.Inf(-1)} {
		if _, err := advanceTravel(1, 10, 1, 1, distance); err == nil {
			t.Fatal("invalid boundary accepted")
		}
	}
	for _, input := range [][5]float64{{0, 10, 1, 1, 1}, {1, 1, 1, 0, 1}} {
		if _, err := advanceTravel(input[0], input[1], input[2], input[3], input[4]); err == nil {
			t.Fatal("invalid zero parameter accepted")
		}
	}
}

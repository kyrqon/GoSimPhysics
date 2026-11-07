package ballistics

import (
	"math"
	"math/big"
	"math/rand"
	"testing"
)

func modelNear(t testing.TB, got, want, relative float64) {
	t.Helper()
	tolerance := math.Max(math.Abs(want)*relative, math.SmallestNonzeroFloat64*4)
	if !numericFinite(got) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.17g, want %.17g (tolerance %g)", got, want, tolerance)
	}
}

func numericFinite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }

func referenceEnergy(mass, speed float64) float64 {
	m := new(big.Float).SetPrec(256).SetFloat64(mass)
	v := new(big.Float).SetPrec(256).SetFloat64(speed)
	energy := new(big.Float).SetPrec(256).Mul(m, v)
	energy.Mul(energy, v)
	energy.Quo(energy, big.NewFloat(2))
	result, _ := energy.Float64()
	return result
}

func referenceSpeed(mass, energy float64) float64 {
	value := new(big.Float).SetPrec(256).SetFloat64(energy)
	value.Mul(value, big.NewFloat(2))
	value.Quo(value, new(big.Float).SetPrec(256).SetFloat64(mass))
	value.Sqrt(value)
	result, _ := value.Float64()
	return result
}

func TestEnergySpeedAnalyticAndScaled(t *testing.T) {
	for _, test := range []struct{ mass, speed float64 }{
		{2, 4}, {0.5, 40}, {10, 0.2},
		// v² overflows here, while the kinetic energy is representable.
		{1e-308, 1e308},
		// Dividing the mass by two first would underflow and lose valid energy.
		{math.SmallestNonzeroFloat64, 1e160},
		// Multiplying mass*speed first overflows, though the final energy fits.
		{math.MaxFloat64, 1.1},
	} {
		energy, err := Energy(test.mass, test.speed)
		if err != nil {
			t.Fatal(err)
		}
		modelNear(t, energy, referenceEnergy(test.mass, test.speed), 2e-15)
		speed, err := Speed(test.mass, energy)
		if err != nil {
			t.Fatal(err)
		}
		modelNear(t, speed, test.speed, 2e-15)
	}
	for _, test := range []struct{ mass, energy float64 }{
		{math.MaxFloat64, math.SmallestNonzeroFloat64},
		{1e-308, 1e308},
		{math.SmallestNonzeroFloat64, math.MaxFloat64},
	} {
		want := referenceSpeed(test.mass, test.energy)
		got, err := Speed(test.mass, test.energy)
		if math.IsInf(want, 1) {
			if err == nil {
				t.Fatal("accepted an unrepresentable speed")
			}
			continue
		}
		if err != nil {
			t.Fatal(err)
		}
		modelNear(t, got, want, 2e-15)
	}
	if energy, err := Energy(1, 0); err != nil || energy != 0 {
		t.Fatal("zero speed must have zero energy")
	}
	if speed, err := Speed(1, 0); err != nil || speed != 0 {
		t.Fatal("zero energy must have zero speed")
	}
}

func TestDamageUsesRemainingEnergyAndIsCapped(t *testing.T) {
	for _, test := range []struct{ base, energy, reference, want float64 }{
		{100, 150, 200, 75},
		{100, 0, 200, 0},
		{0, 100, 200, 0},
		{100, 300, 200, 100},
		{math.MaxFloat64, math.SmallestNonzeroFloat64, math.MaxFloat64, math.SmallestNonzeroFloat64},
	} {
		got, err := Damage(test.base, test.energy, test.reference)
		if err != nil {
			t.Fatal(err)
		}
		modelNear(t, got, test.want, 1e-15)
		if got > test.base || got < 0 {
			t.Fatal("damage exceeded its base cap")
		}
	}
}

func TestAdvanceBoundaryStopAndTimeLimits(t *testing.T) {
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
			got, err := Advance(1, 200, test.resistance, test.dt, test.max)
			if err != nil {
				t.Fatal(err)
			}
			modelNear(t, got.Distance, test.distance, 2e-14)
			modelNear(t, got.Time, test.duration, 2e-14)
			modelNear(t, got.Energy, test.energy, 2e-14)
			modelNear(t, got.Speed, test.speed, 2e-14)
			if got.Reached != test.reached || got.Stopped != test.stopped {
				t.Fatalf("flags = reached:%v stopped:%v, want %v %v", got.Reached, got.Stopped, test.reached, test.stopped)
			}
			modelNear(t, got.Energy+test.resistance*got.Distance, 200, 2e-14)
		})
	}
	stopped, err := Advance(1, 0, 100, 1, math.Inf(1))
	if err != nil || !stopped.Stopped || stopped.Distance != 0 || stopped.Time != 0 {
		t.Fatalf("zero energy moved: %+v, %v", stopped, err)
	}
}

func TestAdvanceIsIndependentOfTimePartition(t *testing.T) {
	random := rand.New(rand.NewSource(2048))
	for fixture := 0; fixture < 100; fixture++ {
		mass := 0.25 + random.Float64()*3
		energy := 20 + random.Float64()*500
		resistance := random.Float64() * 200
		duration := 0.02 + random.Float64()*2
		whole, err := Advance(mass, energy, resistance, duration, math.Inf(1))
		if err != nil {
			t.Fatal(err)
		}
		remaining, distance, elapsed := energy, 0.0, 0.0
		for part := 0; part < 19; part++ {
			segment, err := Advance(mass, remaining, resistance, duration/19, math.Inf(1))
			if err != nil {
				t.Fatalf("fixture %d part %d: %v", fixture, part, err)
			}
			distance += segment.Distance
			elapsed += segment.Time
			remaining = segment.Energy
		}
		modelNear(t, distance, whole.Distance, 2e-12)
		modelNear(t, elapsed, whole.Time, 2e-12)
		modelNear(t, remaining, whole.Energy, 2e-12)
	}
}

func TestAdvanceRetainsRepresentableMotionAtExtremeScales(t *testing.T) {
	// stopTime=1e309 cannot be represented, but a 1e308-second partial interval
	// still loses 19 percent of its energy and travels a finite 9.5e307 units.
	got, err := Advance(1e308, 5e307, 0.1, 1e308, math.Inf(1))
	if err != nil {
		t.Fatal(err)
	}
	modelNear(t, got.Distance, 9.5e307, 2e-15)
	modelNear(t, got.Energy, 4.05e307, 2e-15)
	modelNear(t, got.Speed, 0.9, 2e-15)
	if got.Reached || got.Stopped {
		t.Fatal("finite partial interval incorrectly reached an event")
	}
	if _, err := Advance(1, 5e307, 0, 1e155, math.Inf(1)); err == nil {
		t.Fatal("accepted an overflowing actual travel distance")
	}
}

func TestModelRejectsInvalidNumbers(t *testing.T) {
	for _, resistance := range []float64{-1, math.NaN(), math.Inf(1)} {
		if (Material{Resistance: resistance}).Validate() == nil {
			t.Fatal("invalid resistance accepted")
		}
	}
	if err := (Material{Name: "game material"}).Validate(); err != nil {
		t.Fatal(err)
	}
	for _, bad := range []float64{-1, math.NaN(), math.Inf(1)} {
		if _, err := Energy(bad, 1); err == nil {
			t.Fatal("invalid mass accepted")
		}
		if _, err := Energy(1, bad); err == nil {
			t.Fatal("invalid speed accepted")
		}
		if _, err := Speed(1, bad); err == nil {
			t.Fatal("invalid energy accepted")
		}
		if _, err := Damage(10, bad, 10); err == nil {
			t.Fatal("invalid damage energy accepted")
		}
		if _, err := Advance(1, 10, bad, 1, 1); err == nil {
			t.Fatal("invalid advance resistance accepted")
		}
		if _, err := Advance(1, 10, 1, bad, 1); err == nil {
			t.Fatal("invalid duration accepted")
		}
	}
	for _, distance := range []float64{-1, math.NaN(), math.Inf(-1)} {
		if _, err := Advance(1, 10, 1, 1, distance); err == nil {
			t.Fatal("invalid boundary accepted")
		}
	}
	if _, err := Energy(0, 1); err == nil {
		t.Fatal("zero mass accepted")
	}
	if _, err := Energy(1, math.MaxFloat64); err == nil {
		t.Fatal("overflowing kinetic energy accepted")
	}
	if _, err := Energy(math.SmallestNonzeroFloat64, 1); err == nil {
		t.Fatal("unrepresentable positive kinetic energy accepted")
	}
	if _, err := Damage(1, 1, 0); err == nil {
		t.Fatal("zero reference energy accepted")
	}
	if _, err := Advance(1, 1, 1, 0, 1); err == nil {
		t.Fatal("zero duration accepted")
	}
}

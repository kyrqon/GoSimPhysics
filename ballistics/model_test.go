package ballistics

import (
	"math"
	"math/big"
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
}

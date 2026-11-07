package math2d

import (
	"math"
	"testing"
)

func coreClose(t *testing.T, got, want, tolerance float64) {
	t.Helper()
	if math.IsNaN(got) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.17g, want %.17g (tolerance %g)", got, want, tolerance)
	}
}

func coreVecClose(t *testing.T, got, want Vec2, tolerance float64) {
	t.Helper()
	coreClose(t, got.X, want.X, tolerance)
	coreClose(t, got.Y, want.Y, tolerance)
}

func TestCoreVecArithmetic(t *testing.T) {
	a, b := Vec2{3, 4}, Vec2{-2, 5}
	coreVecClose(t, a.Add(b), Vec2{1, 9}, 0)
	coreVecClose(t, a.Sub(b), Vec2{5, -1}, 0)
	coreVecClose(t, a.Mul(2), Vec2{6, 8}, 0)
	coreClose(t, a.Dot(b), 14, 0)
	coreClose(t, a.Cross(b), 23, 0)
	coreClose(t, a.LengthSquared(), 25, 0)
	coreClose(t, a.Length(), 5, 0)
	coreVecClose(t, a.Normalized(), Vec2{0.6, 0.8}, 1e-15)
	coreVecClose(t, a.Rotate(math.Pi/2), Vec2{-4, 3}, 1e-14)
	coreVecClose(t, a.Rotate(0.37).Rotate(-0.37), a, 1e-14)
}

func TestCoreVecNormalizationAtDifferentScales(t *testing.T) {
	for _, scale := range []float64{1e-300, 1, 1e300} {
		v := Vec2{scale, scale}.Normalized()
		coreVecClose(t, v, Vec2{1 / math.Sqrt2, 1 / math.Sqrt2}, 1e-14)
		coreClose(t, v.Length(), 1, 1e-14)
	}
	coreVecClose(t, (Vec2{}).Normalized(), Vec2{}, 0)
}

func TestCoreAngularVelocityAtPoint(t *testing.T) {
	coreVecClose(t, AngularVelocityAt(3, Vec2{2, -4}), Vec2{12, 6}, 0)
}

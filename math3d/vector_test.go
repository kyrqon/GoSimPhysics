package math3d

import (
	"math"
	"testing"

	"github.com/kyrqon/GoSimPhysics/internal/numeric"
)

func math3Near(t *testing.T, label string, got, want, tolerance float64) {
	t.Helper()
	if !numeric.Finite(got) || math.Abs(got-want) > tolerance {
		t.Fatalf("%s = %.15g, want %.15g within %g", label, got, want, tolerance)
	}
}

func math3VecNear(t *testing.T, label string, got, want Vec3, tolerance float64) {
	t.Helper()
	if !validVec(got) || got.Sub(want).Length() > tolerance {
		t.Fatalf("%s = %v, want %v within %g", label, got, want, tolerance)
	}
}

func TestVec3ArithmeticAndRightHandedCross(t *testing.T) {
	a, b := Vec3{1, 2, 3}, Vec3{-4, 5, -6}
	math3VecNear(t, "sum", a.Add(b), Vec3{-3, 7, -3}, 0)
	math3VecNear(t, "difference", a.Sub(b), Vec3{5, -3, 9}, 0)
	math3VecNear(t, "scale", a.Mul(2), Vec3{2, 4, 6}, 0)
	math3Near(t, "dot", a.Dot(b), -12, 0)
	math3VecNear(t, "cross", a.Cross(b), Vec3{-27, -6, 13}, 0)
	math3Near(t, "cross orthogonal to a", a.Cross(b).Dot(a), 0, 0)
	math3Near(t, "cross orthogonal to b", a.Cross(b).Dot(b), 0, 0)
	math3VecNear(t, "right handed", (Vec3{X: 1}).Cross(Vec3{Y: 1}), Vec3{Z: 1}, 0)
	math3Near(t, "squared length", a.LengthSquared(), 14, 0)
	math3Near(t, "length", a.Length(), math.Sqrt(14), 1e-15)
}

func TestVec3NormalizationAcrossScales(t *testing.T) {
	math3VecNear(t, "zero", (Vec3{}).Normalized(), Vec3{}, 0)
	want := Vec3{1, -2, 2}.Mul(1.0 / 3)
	for _, scale := range []float64{1, 1e300, 1e-300} {
		v := (Vec3{1, -2, 2}).Mul(scale).Normalized()
		math3VecNear(t, "normalized", v, want, 1e-15)
		math3Near(t, "unit length", v.Length(), 1, 1e-15)
	}
	math3VecNear(t, "subnormal", (Vec3{X: math.SmallestNonzeroFloat64}).Normalized(), Vec3{X: 1}, 0)
	if validVec(Vec3{X: math.NaN()}) || validVec(Vec3{Z: math.Inf(1)}) {
		t.Fatal("non-finite vector accepted")
	}
}

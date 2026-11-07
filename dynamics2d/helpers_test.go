package dynamics2d

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

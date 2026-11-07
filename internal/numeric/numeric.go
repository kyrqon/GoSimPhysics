// Package numeric provides floating-point helpers shared by the physics packages.
package numeric

import "math"

// Finite reports whether value is neither NaN nor an infinity.
func Finite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }

// Clamp limits value to the interval from lo to hi.
func Clamp(value, lo, hi float64) float64 { return math.Max(lo, math.Min(value, hi)) }

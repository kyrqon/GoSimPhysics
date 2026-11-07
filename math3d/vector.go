package math3d

import (
	"math"

	"github.com/kyrqon/GoSimPhysics/internal/numeric"
)

// Vec3 is a three-dimensional vector or point, using float64 coordinates.
type Vec3 struct {
	X, Y, Z float64
}

// Add returns v plus other.
func (v Vec3) Add(other Vec3) Vec3 { return Vec3{v.X + other.X, v.Y + other.Y, v.Z + other.Z} }

// Sub returns v minus other.
func (v Vec3) Sub(other Vec3) Vec3 { return Vec3{v.X - other.X, v.Y - other.Y, v.Z - other.Z} }

// Mul scales v by s.
func (v Vec3) Mul(s float64) Vec3 { return Vec3{v.X * s, v.Y * s, v.Z * s} }

// Dot returns the scalar dot product.
func (v Vec3) Dot(other Vec3) float64 { return v.X*other.X + v.Y*other.Y + v.Z*other.Z }

// Cross returns the right-handed vector cross product.
func (v Vec3) Cross(other Vec3) Vec3 {
	return Vec3{v.Y*other.Z - v.Z*other.Y, v.Z*other.X - v.X*other.Z, v.X*other.Y - v.Y*other.X}
}

// LengthSquared returns the squared Euclidean length.
func (v Vec3) LengthSquared() float64 { return v.Dot(v) }

// Length returns the Euclidean length without unnecessary intermediate overflow.
func (v Vec3) Length() float64 { return math.Hypot(math.Hypot(v.X, v.Y), v.Z) }

// Normalized returns a unit vector in v's direction. The zero vector remains zero.
func (v Vec3) Normalized() Vec3 {
	scale := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
	if scale == 0 {
		return Vec3{}
	}
	x, y, z := v.X/scale, v.Y/scale, v.Z/scale
	length := math.Sqrt(x*x + y*y + z*z)
	return Vec3{x / length, y / length, z / length}
}

func validVec(v Vec3) bool {
	return numeric.Finite(v.X) && numeric.Finite(v.Y) && numeric.Finite(v.Z)
}

package math2d

import "math"

// Vec2 is a two-dimensional vector or point. Coordinates use float64 precision.
type Vec2 struct {
	X, Y float64
}

// Add returns the sum of v and other.
func (v Vec2) Add(other Vec2) Vec2 { return Vec2{v.X + other.X, v.Y + other.Y} }

// Sub returns v minus other.
func (v Vec2) Sub(other Vec2) Vec2 { return Vec2{v.X - other.X, v.Y - other.Y} }

// Mul scales v by s.
func (v Vec2) Mul(s float64) Vec2 { return Vec2{v.X * s, v.Y * s} }

// Dot returns the scalar dot product of v and other.
func (v Vec2) Dot(other Vec2) float64 { return v.X*other.X + v.Y*other.Y }

// Cross returns the signed scalar cross product of v and other.
func (v Vec2) Cross(other Vec2) float64 { return v.X*other.Y - v.Y*other.X }

// LengthSquared returns the squared Euclidean length of v.
func (v Vec2) LengthSquared() float64 { return v.Dot(v) }

// Length returns the Euclidean length of v.
func (v Vec2) Length() float64 { return math.Hypot(v.X, v.Y) }

// Normalized returns a unit vector pointing along v. The zero vector remains zero.
func (v Vec2) Normalized() Vec2 {
	// Scaling first avoids overflow in the length or reciprocal of tiny vectors.
	scale := math.Max(math.Abs(v.X), math.Abs(v.Y))
	if scale == 0 {
		return Vec2{}
	}
	x, y := v.X/scale, v.Y/scale
	length := math.Hypot(x, y)
	return Vec2{x / length, y / length}
}

// Rotate rotates v counterclockwise by angle radians.
func (v Vec2) Rotate(angle float64) Vec2 {
	sine, cosine := math.Sincos(angle)
	return Vec2{cosine*v.X - sine*v.Y, sine*v.X + cosine*v.Y}
}

// AngularVelocityAt returns the tangential velocity at radius from the rotation center.
func AngularVelocityAt(angularVelocity float64, radius Vec2) Vec2 {
	return Vec2{-angularVelocity * radius.Y, angularVelocity * radius.X}
}

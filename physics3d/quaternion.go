package physics3d

import (
	"math"

	"github.com/kyrqon/GoSimPhysics/internal/numeric"
)

// Quaternion represents a rotation. Components follow the (X, Y, Z, W) convention,
// where W is the scalar component. Body rotations are stored as unit quaternions.
type Quaternion struct {
	X, Y, Z, W float64
}

// Identity returns the quaternion that leaves vectors unchanged.
func Identity() Quaternion { return Quaternion{W: 1} }

// QuaternionFromAxisAngle returns a right-handed rotation about axis, in radians.
// The axis need not be normalized. A zero axis produces Identity.
func QuaternionFromAxisAngle(axis Vec3, angle float64) Quaternion {
	axis = axis.Normalized()
	if axis == (Vec3{}) {
		return Identity()
	}
	sine, cosine := math.Sincos(angle * 0.5)
	return Quaternion{axis.X * sine, axis.Y * sine, axis.Z * sine, cosine}
}

// Mul returns the Hamilton product q*other. For rotations it applies other first,
// followed by q; quaternion multiplication is not commutative.
func (q Quaternion) Mul(other Quaternion) Quaternion {
	return Quaternion{
		q.W*other.X + q.X*other.W + q.Y*other.Z - q.Z*other.Y,
		q.W*other.Y - q.X*other.Z + q.Y*other.W + q.Z*other.X,
		q.W*other.Z + q.X*other.Y - q.Y*other.X + q.Z*other.W,
		q.W*other.W - q.X*other.X - q.Y*other.Y - q.Z*other.Z,
	}
}

// Conjugate returns the quaternion conjugate, the inverse of a unit quaternion.
func (q Quaternion) Conjugate() Quaternion { return Quaternion{-q.X, -q.Y, -q.Z, q.W} }

// Normalized returns a unit quaternion. The zero quaternion becomes Identity.
// Scaling before normalization handles very large and subnormal components.
func (q Quaternion) Normalized() Quaternion {
	scale := math.Max(math.Abs(q.X), math.Max(math.Abs(q.Y), math.Max(math.Abs(q.Z), math.Abs(q.W))))
	if scale == 0 {
		return Identity()
	}
	x, y, z, w := q.X/scale, q.Y/scale, q.Z/scale, q.W/scale
	length := math.Sqrt(x*x + y*y + z*z + w*w)
	return Quaternion{x / length, y / length, z / length, w / length}
}

// Rotate rotates v using q. q is normalized first, and a zero quaternion acts as
// Identity. Non-finite components propagate to the result.
func (q Quaternion) Rotate(v Vec3) Vec3 {
	q = q.Normalized()
	u := Vec3{q.X, q.Y, q.Z}
	// Rodrigues' quaternion formula avoids constructing temporary quaternions.
	cross := u.Cross(v)
	return v.Add(cross.Mul(2 * q.W)).Add(u.Cross(cross).Mul(2))
}

func validQuat(q Quaternion) bool {
	return numeric.Finite(q.X) && numeric.Finite(q.Y) && numeric.Finite(q.Z) && numeric.Finite(q.W) && q != (Quaternion{})
}

// IntegrateRotation applies a world-space rotation vector by left-multiplying
// the current orientation. This convention matches world-space angular velocity.
func IntegrateRotation(q Quaternion, angularDelta Vec3) Quaternion {
	angle := angularDelta.Length()
	if angle == 0 {
		return q.Normalized()
	}
	return QuaternionFromAxisAngle(angularDelta, angle).Mul(q).Normalized()
}

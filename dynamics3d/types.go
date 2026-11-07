package dynamics3d

import (
	"github.com/kyrqon/GoSimPhysics/collision3d"
	"github.com/kyrqon/GoSimPhysics/internal/numeric"
	"github.com/kyrqon/GoSimPhysics/math3d"
)

// Vec3 is a three-dimensional vector or point.
type Vec3 = math3d.Vec3

// Quaternion represents a three-dimensional rotation.
type Quaternion = math3d.Quaternion

// Shape is a centered sphere or box collision shape.
type Shape = collision3d.Shape

// Sphere is a solid sphere.
type Sphere = collision3d.Sphere

// Box is a solid oriented cuboid.
type Box = collision3d.Box

// Contact describes touching or overlapping shapes.
type Contact = collision3d.Contact

// TOIResult describes the first contact in a constant-velocity motion query.
type TOIResult = collision3d.TOIResult

// Identity returns the identity rotation.
func Identity() Quaternion { return math3d.Identity() }

// QuaternionFromAxisAngle returns a right-handed rotation about axis, in radians.
func QuaternionFromAxisAngle(axis Vec3, angle float64) Quaternion {
	return math3d.QuaternionFromAxisAngle(axis, angle)
}

func copyShape(shape Shape) (Shape, bool) { return collision3d.CloneShape(shape) }
func finite(value float64) bool           { return numeric.Finite(value) }
func validVec(v Vec3) bool                { return finite(v.X) && finite(v.Y) && finite(v.Z) }
func validQuat(q Quaternion) bool {
	return finite(q.X) && finite(q.Y) && finite(q.Z) && finite(q.W) && q != (Quaternion{})
}
func clamp(value, lo, hi float64) float64 { return numeric.Clamp(value, lo, hi) }
func integrateRotation(q Quaternion, delta Vec3) Quaternion {
	return math3d.IntegrateRotation(q, delta)
}

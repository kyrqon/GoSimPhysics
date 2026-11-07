package collision3d

import (
	"fmt"
	"math"

	"github.com/kyrqon/GoSimPhysics/internal/numeric"
	"github.com/kyrqon/GoSimPhysics/math3d"
)

// Vec3 is a three-dimensional vector or point.
type Vec3 = math3d.Vec3

// Quaternion represents a three-dimensional rotation.
type Quaternion = math3d.Quaternion

// Identity returns the identity rotation.
func Identity() Quaternion { return math3d.Identity() }

// QuaternionFromAxisAngle returns a right-handed rotation about axis, in radians.
func QuaternionFromAxisAngle(axis Vec3, angle float64) Quaternion {
	return math3d.QuaternionFromAxisAngle(axis, angle)
}

// Collider is a centered shape and its world-space pose. The zero Rotation
// means Identity. Queries copy pointer shapes and never change their inputs.
type Collider struct {
	Shape    Shape
	Position Vec3
	Rotation Quaternion
}

// Motion adds constant linear and world-space angular velocity to a Collider.
// It has no mass, force, motion type, or connection to a simulation world.
type Motion struct {
	Collider
	Velocity, AngularVelocity Vec3
}

// Collide returns touching or overlapping contacts. Invalid geometry or numeric
// overflow returns no contact; CollideChecked distinguishes those failures.
func Collide(a, b Collider) (Contact, bool) {
	c, hit, err := CollideChecked(a, b)
	return c, hit && err == nil
}

// CollideChecked distinguishes invalid geometry and numeric failure from an
// ordinary separated pair. Successful contacts always contain finite values.
func CollideChecked(a, b Collider) (Contact, bool, error) {
	ga, err := canonicalGeometry(a)
	if err != nil {
		return Contact{}, false, err
	}
	gb, err := canonicalGeometry(b)
	if err != nil {
		return Contact{}, false, err
	}
	return collideChecked(ga, gb)
}

// TimeOfImpact returns the first contact over positive finite dt seconds.
// Velocities remain constant; initial overlap or touching returns Time zero.
// Spheres and fixed-orientation boxes use analytic translational casts, while
// rotating boxes use conservative advancement with certified angular bounds.
// Invalid input, overflow, and convergence exhaustion return an error.
func TimeOfImpact(a, b Motion, dt float64) (TOIResult, bool, error) {
	ga, err := canonicalMotion(a)
	if err != nil {
		return TOIResult{}, false, err
	}
	gb, err := canonicalMotion(b)
	if err != nil {
		return TOIResult{}, false, err
	}
	return timeOfImpact(ga, gb, dt)
}

// At evaluates the constant-velocity pose at a finite, nonnegative time.
// It checks the resulting shape bounds as well as the pose for overflow.
func (m Motion) At(time float64) (Collider, error) {
	g, err := canonicalMotion(m)
	if err != nil {
		return Collider{}, err
	}
	pose, err := toiPose(g, time)
	if err != nil {
		return Collider{}, err
	}
	return Collider{Shape: pose.shape, Position: pose.Position, Rotation: pose.Rotation}, nil
}

// Bounds returns the axis-aligned world-space bounds, reporting invalid shapes,
// invalid transforms, and unrepresentable bounds as errors.
func Bounds(c Collider) (min, max Vec3, err error) {
	g, err := canonicalGeometry(c)
	if err != nil {
		return Vec3{}, Vec3{}, err
	}
	min, max = bodyAABB(g)
	if !validVec(min) || !validVec(max) {
		return Vec3{}, Vec3{}, fmt.Errorf("collision3d: collision bounds overflow")
	}
	return min, max, nil
}

// FeatureSize returns the radius of a sphere or the smallest half extent of a
// box. Invalid shapes return zero.
func FeatureSize(shape Shape) float64 {
	s, ok := CloneShape(shape)
	if !ok {
		return 0
	}
	return toiFeature(&geometry{shape: s})
}

// DistanceTolerance is the TOI contact tolerance: 1e-9 times the smaller shape
// feature, with a representable positive floor. Transforms do not affect it.
func DistanceTolerance(a, b Collider) float64 {
	feature := math.Min(FeatureSize(a.Shape), FeatureSize(b.Shape))
	if feature <= 0 {
		return 1e-9
	}
	return math.Max(1e-9*feature, math.SmallestNonzeroFloat64*64)
}

// geometry contains only the pose and motion needed by the narrow phase.
type geometry struct {
	shape                               Shape
	Position, Velocity, AngularVelocity Vec3
	Rotation                            Quaternion
}

func canonicalGeometry(c Collider) (*geometry, error) {
	shape, ok := CloneShape(c.Shape)
	if !ok {
		return nil, fmt.Errorf("collision3d: invalid collision shape")
	}
	if c.Rotation == (Quaternion{}) {
		c.Rotation = Identity()
	}
	if !validVec(c.Position) || !collisionRotationValid(c.Rotation) {
		return nil, fmt.Errorf("collision3d: invalid collision transform")
	}
	return &geometry{shape: shape, Position: c.Position, Rotation: c.Rotation}, nil
}

func canonicalMotion(m Motion) (*geometry, error) {
	g, err := canonicalGeometry(m.Collider)
	if err != nil {
		return nil, err
	}
	g.Velocity, g.AngularVelocity = m.Velocity, m.AngularVelocity
	if !g.validState() {
		return nil, fmt.Errorf("collision3d: invalid motion")
	}
	return g, nil
}

func (g *geometry) validState() bool {
	return validVec(g.Position) && validVec(g.Velocity) && validVec(g.AngularVelocity) && collisionRotationValid(g.Rotation)
}

func bodyBoundsFinite(g *geometry) bool {
	min, max := bodyAABB(g)
	return validVec(min) && validVec(max)
}

func finite(value float64) bool           { return numeric.Finite(value) }
func validVec(v Vec3) bool                { return finite(v.X) && finite(v.Y) && finite(v.Z) }
func clamp(value, lo, hi float64) float64 { return numeric.Clamp(value, lo, hi) }
func integrateRotation(q Quaternion, delta Vec3) Quaternion {
	return math3d.IntegrateRotation(q, delta)
}

func (g *geometry) worldPoint(local Vec3) Vec3 { return g.Position.Add(g.Rotation.Rotate(local)) }

package physics3d

import (
	"fmt"
	"math"
)

// Collider returns a value snapshot of the body's shape and current pose.
// Editing the snapshot does not move the body. A nil body returns a zero
// Collider, which geometry queries reject as invalid.
func (b *Body) Collider() Collider {
	if b == nil {
		return Collider{}
	}
	return bodyCollider(b)
}

// Motion returns a validated, frozen snapshot of the body's constant motion.
// Static bodies have zero velocity; FixedRotation suppresses angular motion.
// Invalid body state, geometry, or unrepresentable bounds returns an error.
func (b *Body) Motion() (Motion, error) {
	if b == nil || b.kind > Kinematic || !b.validState() {
		return Motion{}, fmt.Errorf("physics3d: motion requires a valid finite body")
	}
	m := bodyMotion(b)
	shape, ok := CloneShape(m.Shape)
	if !ok {
		return Motion{}, fmt.Errorf("physics3d: motion requires a valid shape")
	}
	m.Shape = shape
	if _, err := m.At(0); err != nil {
		return Motion{}, err
	}
	return m, nil
}

// Collide queries the current poses without changing either body or considering
// their motion types. Invalid geometry and numeric failure return no contact.
func (b *Body) Collide(other *Body) (Contact, bool) {
	return collideBodies(b, other)
}

// TimeOfImpact sweeps the bodies at constant velocities over dt seconds without
// changing either body. Static bodies stay fixed and FixedRotation suppresses
// angular motion. Invalid input, overflow, and convergence failures are errors.
func (b *Body) TimeOfImpact(other *Body, dt float64) (TOIResult, bool, error) {
	return timeOfImpactBodies(b, other, dt)
}

func bodyCollider(b *Body) Collider {
	return Collider{Shape: b.shape, Position: b.Position, Rotation: b.Rotation}
}

func bodyMotion(b *Body) Motion {
	m := Motion{Collider: bodyCollider(b)}
	if b.kind != Static {
		m.Velocity = b.Velocity
		if !b.fixedRotation {
			m.AngularVelocity = b.AngularVelocity
		}
	}
	return m
}

// Collide queries the current poses without changing the bodies or considering
// their motion types. Invalid geometry and numeric failure return no contact.
func collideBodies(a, b *Body) (Contact, bool) {
	c, hit, err := collideChecked(a, b)
	return c, hit && err == nil
}

func collideChecked(a, b *Body) (Contact, bool, error) {
	if a == nil || b == nil || !validVec(a.Position) || !validVec(b.Position) || !validQuat(a.Rotation) || !validQuat(b.Rotation) {
		return Contact{}, false, fmt.Errorf("physics3d: invalid collision transform")
	}
	return CollideChecked(bodyCollider(a), bodyCollider(b))
}

// TimeOfImpact sweeps two bodies over positive finite dt seconds without
// changing either body. Static bodies stay fixed; FixedRotation suppresses
// angular motion. Other linear and world-space angular velocities are constant.
// Initial overlap or contact returns Time zero. Invalid input, overflow, and
// convergence exhaustion return an error.
func timeOfImpactBodies(a, b *Body, dt float64) (TOIResult, bool, error) {
	if a == nil || b == nil || !a.validState() || !b.validState() {
		return TOIResult{}, false, fmt.Errorf("physics3d: TOI requires valid bodies and positive finite dt")
	}
	return TimeOfImpact(bodyMotion(a), bodyMotion(b), dt)
}

func toiPose(b *Body, time float64) (*Body, error) {
	if b == nil || !b.validState() {
		return nil, fmt.Errorf("physics3d: invalid TOI pose input")
	}
	c, err := bodyMotion(b).At(time)
	if err != nil {
		return nil, err
	}
	pose := *b
	pose.Position, pose.Rotation = c.Position, c.Rotation
	return &pose, nil
}

func toiFeature(b *Body) float64 { return FeatureSize(b.shape) }
func toiDistanceTolerance(a, b *Body) float64 {
	return DistanceTolerance(bodyCollider(a), bodyCollider(b))
}

func bodyAABB(b *Body) (Vec3, Vec3) {
	if b != nil && validQuat(b.Rotation) {
		min, max, err := Bounds(bodyCollider(b))
		if err == nil {
			return min, max
		}
	}
	invalid := Vec3{X: math.NaN(), Y: math.NaN(), Z: math.NaN()}
	return invalid, invalid
}

func bodyBoundsFinite(b *Body) bool {
	min, max := bodyAABB(b)
	return validVec(min) && validVec(max)
}

func aabbOverlap(a, b *Body) bool {
	minA, maxA := bodyAABB(a)
	minB, maxB := bodyAABB(b)
	// Let the checked narrow phase report failures instead of filtering them out.
	if !validVec(minA) || !validVec(maxA) || !validVec(minB) || !validVec(maxB) {
		return true
	}
	return minA.X <= maxB.X && minB.X <= maxA.X && minA.Y <= maxB.Y && minB.Y <= maxA.Y && minA.Z <= maxB.Z && minB.Z <= maxA.Z
}

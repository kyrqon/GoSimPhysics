package dynamics3d

import (
	"fmt"
	"math"

	"github.com/kyrqon/GoSimPhysics/collision3d"
)

func bodyCollider(b *Body) collision3d.Collider {
	return collision3d.Collider{Shape: b.shape, Position: b.Position, Rotation: b.Rotation}
}

func bodyMotion(b *Body) collision3d.Motion {
	m := collision3d.Motion{Collider: bodyCollider(b)}
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
func Collide(a, b *Body) (Contact, bool) {
	c, hit, err := collideChecked(a, b)
	return c, hit && err == nil
}

func collideChecked(a, b *Body) (Contact, bool, error) {
	if a == nil || b == nil || !validVec(a.Position) || !validVec(b.Position) || !validQuat(a.Rotation) || !validQuat(b.Rotation) {
		return Contact{}, false, fmt.Errorf("dynamics3d: invalid collision transform")
	}
	return collision3d.CollideChecked(bodyCollider(a), bodyCollider(b))
}

// TimeOfImpact sweeps two bodies over positive finite dt seconds without
// changing either body. Static bodies stay fixed; FixedRotation suppresses
// angular motion. Other linear and world-space angular velocities are constant.
// Initial overlap or contact returns Time zero. Invalid input, overflow, and
// convergence exhaustion return an error.
func TimeOfImpact(a, b *Body, dt float64) (TOIResult, bool, error) {
	if a == nil || b == nil || !a.validState() || !b.validState() {
		return TOIResult{}, false, fmt.Errorf("dynamics3d: TOI requires valid bodies and positive finite dt")
	}
	return collision3d.TimeOfImpact(bodyMotion(a), bodyMotion(b), dt)
}

func toiPose(b *Body, time float64) (*Body, error) {
	if b == nil || !b.validState() {
		return nil, fmt.Errorf("dynamics3d: invalid TOI pose input")
	}
	c, err := bodyMotion(b).At(time)
	if err != nil {
		return nil, err
	}
	pose := *b
	pose.Position, pose.Rotation = c.Position, c.Rotation
	return &pose, nil
}

func toiFeature(b *Body) float64 { return collision3d.FeatureSize(b.shape) }
func toiDistanceTolerance(a, b *Body) float64 {
	return collision3d.DistanceTolerance(bodyCollider(a), bodyCollider(b))
}

func bodyAABB(b *Body) (Vec3, Vec3) {
	if b != nil && validQuat(b.Rotation) {
		min, max, err := collision3d.Bounds(bodyCollider(b))
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

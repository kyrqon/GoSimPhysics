package physics2d

import (
	"fmt"
	"math"
)

// Collider returns a value snapshot of the body's shape and current pose.
// Editing the snapshot does not move the body. A nil body returns a zero,
// invalid Collider, which checked geometry queries reject.
func (b *Body) Collider() Collider {
	if b == nil {
		return Collider{}
	}
	return collider(b)
}

// Motion returns a validated constant-motion snapshot. Static bodies have zero
// velocity, and FixedRotation suppresses angular velocity. The shape is copied;
// invalid state, invalid geometry, and unrepresentable bounds return an error.
func (b *Body) Motion() (Motion, error) {
	m, err := motion(b)
	if err != nil {
		return Motion{}, err
	}
	pose, err := m.At(0)
	if err != nil {
		return Motion{}, err
	}
	m.Collider = pose
	return m, nil
}

// Collide queries the current body poses without changing either body. Body
// types are ignored. Nil bodies, invalid input, and unrepresentable geometry
// return no contact. For checked geometry queries, use CollideChecked.
func (b *Body) Collide(other *Body) (Contact, bool) {
	return collideBodies(b, other)
}

// TimeOfImpact sweeps two bodies without changing them or integrating forces.
// Static bodies remain fixed, and FixedRotation suppresses angular motion.
// Nil or invalid bodies and numeric or convergence failures return an error.
func (b *Body) TimeOfImpact(other *Body, dt float64) (TOIResult, bool, error) {
	return timeOfImpactBodies(b, other, dt)
}

func collider(b *Body) Collider {
	return Collider{Shape: b.shape, Position: b.Position, Angle: b.Angle}
}
func motion(b *Body) (Motion, error) {
	if b == nil || !b.validState() || b.kind > Kinematic {
		return Motion{}, fmt.Errorf("physics2d: sweep requires valid finite bodies")
	}
	m := Motion{Collider: collider(b), Velocity: b.Velocity, AngularVelocity: b.AngularVelocity}
	if b.kind == Static {
		m.Velocity, m.AngularVelocity = Vec2{}, 0
	}
	if b.fixedRotation {
		m.AngularVelocity = 0
	}
	return m, nil
}

func collideBodies(a, b *Body) (Contact, bool) {
	if a == nil || b == nil {
		return Contact{}, false
	}
	return Collide(collider(a), collider(b))
}

func timeOfImpactBodies(a, b *Body, dt float64) (TOIResult, bool, error) {
	ma, err := motion(a)
	if err != nil {
		return TOIResult{}, false, err
	}
	mb, err := motion(b)
	if err != nil {
		return TOIResult{}, false, err
	}
	return TimeOfImpact(ma, mb, dt)
}

func toiPose(b *Body, time float64) (*Body, error) {
	m, err := motion(b)
	if err != nil {
		return nil, err
	}
	pose, err := m.At(time)
	if err != nil {
		return nil, err
	}
	result := *b
	result.Position, result.Angle = pose.Position, pose.Angle
	return &result, nil
}
func toiDistanceTolerance(a, b *Body) float64 {
	return DistanceTolerance(collider(a), collider(b))
}
func bodyAABB(b *Body) (min, max Vec2) {
	if b == nil {
		return Vec2{}, Vec2{}
	}
	min, max, err := Bounds(collider(b))
	if err != nil {
		return Vec2{X: math.NaN(), Y: math.NaN()}, Vec2{X: math.NaN(), Y: math.NaN()}
	}
	return min, max
}
func aabbOverlap(a, b *Body) bool {
	amin, amax := bodyAABB(a)
	bmin, bmax := bodyAABB(b)
	return amin.X <= bmax.X && bmin.X <= amax.X && amin.Y <= bmax.Y && bmin.Y <= amax.Y
}
func collisionGeometryFinite(a, b *Body) bool {
	if a == nil || b == nil {
		return false
	}
	_, _, err := CollideChecked(collider(a), collider(b))
	return err == nil
}

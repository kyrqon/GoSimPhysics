package dynamics2d

import (
	"fmt"
	"math"

	"github.com/kyrqon/GoSimPhysics/collision2d"
	"github.com/kyrqon/GoSimPhysics/internal/numeric"
	"github.com/kyrqon/GoSimPhysics/math2d"
)

// Shared geometry and vector types allow common simulations to use one import.
type (
	Vec2      = math2d.Vec2
	Shape     = collision2d.Shape
	Circle    = collision2d.Circle
	Box       = collision2d.Box
	Contact   = collision2d.Contact
	TOIResult = collision2d.TOIResult
)

// ErrTOINoConvergence means a bounded sweep could not establish contact or
// separation. It is the same sentinel used by collision2d.
var ErrTOINoConvergence = collision2d.ErrTOINoConvergence

func collider(b *Body) collision2d.Collider {
	return collision2d.Collider{Shape: b.shape, Position: b.Position, Angle: b.Angle}
}
func motion(b *Body) (collision2d.Motion, error) {
	if b == nil || !b.validState() || b.kind > Kinematic {
		return collision2d.Motion{}, fmt.Errorf("physics: sweep requires valid finite bodies")
	}
	m := collision2d.Motion{Collider: collider(b), Velocity: b.Velocity, AngularVelocity: b.AngularVelocity}
	if b.kind == Static {
		m.Velocity, m.AngularVelocity = Vec2{}, 0
	}
	if b.fixedRotation {
		m.AngularVelocity = 0
	}
	return m, nil
}

// Collide detects contact without changing either body. Body types are ignored.
// Invalid input or unrepresentable geometry returns no contact.
func Collide(a, b *Body) (Contact, bool) {
	if a == nil || b == nil {
		return Contact{}, false
	}
	return collision2d.Collide(collider(a), collider(b))
}

// TimeOfImpact sweeps bodies with constant velocities over dt seconds, without
// integrating forces or modifying either body. Static bodies remain fixed and
// FixedRotation suppresses angular motion. Numeric and convergence failures
// return an error rather than an ordinary miss.
func TimeOfImpact(a, b *Body, dt float64) (TOIResult, bool, error) {
	ma, err := motion(a)
	if err != nil {
		return TOIResult{}, false, err
	}
	mb, err := motion(b)
	if err != nil {
		return TOIResult{}, false, err
	}
	return collision2d.TimeOfImpact(ma, mb, dt)
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
	return collision2d.DistanceTolerance(collider(a), collider(b))
}
func bodyAABB(b *Body) (min, max Vec2) {
	if b == nil {
		return Vec2{}, Vec2{}
	}
	min, max, err := collision2d.Bounds(collider(b))
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
	_, _, err := collision2d.CollideChecked(collider(a), collider(b))
	return err == nil
}
func finite(value float64) bool                    { return numeric.Finite(value) }
func validVec(v Vec2) bool                         { return finite(v.X) && finite(v.Y) }
func clamp(value, lo, hi float64) float64          { return numeric.Clamp(value, lo, hi) }
func angularVelocityAt(omega float64, r Vec2) Vec2 { return math2d.AngularVelocityAt(omega, r) }

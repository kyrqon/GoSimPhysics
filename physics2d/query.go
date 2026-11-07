package physics2d

import (
	"fmt"
	"math"

	"github.com/kyrqon/GoSimPhysics/internal/numeric"
)

// Collider places a centered shape in world coordinates. Angle is in radians.
type Collider struct {
	Shape    Shape
	Position Vec2
	Angle    float64
}

// Motion describes a collider with constant world-space linear velocity and
// angular velocity in radians per second. A zero velocity keeps a collider fixed.
type Motion struct {
	Collider
	Velocity        Vec2
	AngularVelocity float64
}

// proxy is the canonical value geometry used by the narrow-phase algorithms.
// It carries no body ownership, mass properties, forces, or simulation policy.
type proxy struct {
	Position, Velocity     Vec2
	Angle, AngularVelocity float64
	shape                  Shape
}

func (p *proxy) validState() bool {
	return validVec(p.Position) && validVec(p.Velocity) && finite(p.Angle) && finite(p.AngularVelocity)
}
func proxyCollider(c Collider) *proxy {
	return &proxy{Position: c.Position, Angle: c.Angle, shape: c.Shape}
}
func proxyMotion(m Motion) *proxy {
	return &proxy{Position: m.Position, Angle: m.Angle, shape: m.Shape, Velocity: m.Velocity, AngularVelocity: m.AngularVelocity}
}
func (p *proxy) collider() Collider {
	return Collider{Shape: p.shape, Position: p.Position, Angle: p.Angle}
}

// Collide returns a contact when two colliders overlap or touch. Invalid input
// or unrepresentable geometry returns false; use CollideChecked to inspect errors.
func Collide(a, b Collider) (Contact, bool) {
	contact, hit, err := CollideChecked(a, b)
	return contact, hit && err == nil
}

// CollideChecked distinguishes separated shapes from invalid or overflowing
// geometry. A successful contact contains only finite values. Inputs are copied.
func CollideChecked(a, b Collider) (Contact, bool, error) {
	if _, _, err := Bounds(a); err != nil {
		return Contact{}, false, err
	}
	if _, _, err := Bounds(b); err != nil {
		return Contact{}, false, err
	}
	pa, pb := proxyCollider(a), proxyCollider(b)
	if !geometryCollisionFinite(pa, pb) {
		return Contact{}, false, fmt.Errorf("physics2d: invalid or unrepresentable collision geometry")
	}
	contact, hit := collideProxies(pa, pb)
	return contact, hit, nil
}

// TimeOfImpact returns the first contact over dt seconds without changing either
// motion. Initial touching or overlap returns time zero, including when the
// shapes are separating. Linear primitive casts are analytic; rotating boxes
// use bounded conservative advancement. Numeric and convergence failures are
// errors rather than ordinary misses.
func TimeOfImpact(a, b Motion, dt float64) (TOIResult, bool, error) {
	return timeOfImpactProxies(proxyMotion(a), proxyMotion(b), dt)
}

// At evaluates constant-velocity motion at a nonnegative time in seconds.
// It validates the motion, resulting transform, and complete resulting bounds.
func (m Motion) At(time float64) (Collider, error) {
	p := proxyMotion(m)
	shape, valid := CloneShape(m.Shape)
	if !valid || !p.validState() {
		return Collider{}, fmt.Errorf("physics2d: invalid finite motion or shape")
	}
	p.shape = shape
	pose, err := geometryPose(p, time)
	if err != nil {
		return Collider{}, err
	}
	return pose.collider(), nil
}

// Bounds returns the actual world-axis-aligned bounds of a collider. Invalid
// shapes, nonfinite transforms, and unrepresentable bounds return an error.
func Bounds(c Collider) (min, max Vec2, err error) {
	shape, ok := CloneShape(c.Shape)
	if !ok || !validVec(c.Position) || !finite(c.Angle) {
		return Vec2{}, Vec2{}, fmt.Errorf("physics2d: invalid collider")
	}
	p := proxyCollider(c)
	p.shape = shape
	min, max = geometryAABB(p)
	if !validVec(min) || !validVec(max) {
		return Vec2{}, Vec2{}, fmt.Errorf("physics2d: bounds overflow")
	}
	return min, max, nil
}

// FeatureSize returns the smallest intrinsic feature (radius or box half-extent).
// Invalid shapes return zero.
func FeatureSize(shape Shape) float64 {
	copied, valid := CloneShape(shape)
	if !valid {
		return 0
	}
	return toiFeature(&proxy{shape: copied})
}

// DistanceTolerance returns the sweep distance tolerance, scaled by the smaller
// feature of the pair. Invalid input uses the default tolerance of 1e-8.
func DistanceTolerance(a, b Collider) float64 {
	scale := math.Min(FeatureSize(a.Shape), FeatureSize(b.Shape))
	if !finite(scale) || scale <= 0 {
		return toiTolerance
	}
	return math.Max(math.SmallestNonzeroFloat64, scale*toiTolerance)
}

func finite(value float64) bool                    { return numeric.Finite(value) }
func validVec(value Vec2) bool                     { return finite(value.X) && finite(value.Y) }
func clamp(value, lo, hi float64) float64          { return numeric.Clamp(value, lo, hi) }
func angularVelocityAt(omega float64, r Vec2) Vec2 { return AngularVelocityAt(omega, r) }

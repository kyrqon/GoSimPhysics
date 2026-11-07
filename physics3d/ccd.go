package physics3d

import (
	"errors"
	"fmt"
	"math"
)

// ErrTOILimit reports that Step exhausted MaxTOIEvents and rolled back.
var ErrTOILimit = errors.New("physics3d: continuous collision event limit exceeded")

func (w *World) hasContinuousBody() bool {
	for _, b := range w.bodies {
		if b.Continuous {
			return true
		}
	}
	return false
}

// advanceContinuous advances a substep in constant-velocity motion segments.
// Forces have already been integrated for the whole substep. Every impact changes
// the velocities used by the next sweep, including possible ricochets.
func (w *World) advanceContinuous(duration float64) error {
	remaining := duration
	events := 0
	for remaining > 0 {
		a, b, impact, hit, err := w.earliestImpact(remaining)
		if err != nil {
			return err
		}
		if !hit {
			return w.advanceContinuousPoses(remaining)
		}
		if events >= w.MaxTOIEvents {
			return ErrTOILimit
		}
		if impact.Time < 0 || impact.Time > remaining || !finite(impact.Time) {
			return fmt.Errorf("physics3d: invalid continuous impact time")
		}
		if impact.Time > 0 {
			if err := w.advanceContinuousPoses(impact.Time); err != nil {
				return err
			}
			remaining -= impact.Time
		}
		contacts, err := w.contactConstraints()
		if err != nil {
			return err
		}
		found := false
		for _, c := range contacts {
			if (c.a == a && c.b == b) || (c.a == b && c.b == a) {
				found = true
				break
			}
		}
		if !found {
			// Conservative advancement may stop a spatial tolerance before exact
			// overlap. Its contact still needs an impulse at this time of impact.
			c, err := newContactConstraint(a, b, impact.Contact, w.RestitutionThreshold)
			if err != nil {
				return err
			}
			contacts = append(contacts, c)
		}
		for _, joint := range w.joints {
			if !joint.validGeometry() {
				return fmt.Errorf("physics3d: continuous joint geometry overflow")
			}
			joint.prepare(duration)
		}
		for i := 0; i < w.VelocityIterations; i++ {
			for c := range contacts {
				contacts[c].solveVelocity()
			}
			for _, joint := range w.joints {
				joint.solveVelocity()
			}
		}
		for _, body := range w.bodies {
			if !body.validState() || !bodyBoundsFinite(body) {
				return fmt.Errorf("physics3d: continuous contact solver overflow")
			}
		}
		events++
	}
	return nil
}

func (w *World) advanceContinuousPoses(duration float64) error {
	for _, b := range w.bodies {
		if b.kind == Static {
			continue
		}
		momentum := Vec3{}
		if b.kind == Dynamic && !b.fixedRotation {
			momentum = b.inertiaWorld(b.AngularVelocity)
			if !validVec(momentum) {
				return fmt.Errorf("physics3d: continuous angular momentum overflow")
			}
		}
		pose, err := toiPose(b, duration)
		if err != nil {
			return err
		}
		b.Position, b.Rotation = pose.Position, pose.Rotation
		if b.kind == Dynamic && !b.fixedRotation {
			// The sweep's orientation is a constant-omega segment. Recovering
			// omega with the new tensor preserves world angular momentum between
			// impulses even for an anisotropic spinning body.
			b.AngularVelocity = b.inverseInertiaWorld(momentum)
		}
		if !b.validState() || !bodyBoundsFinite(b) {
			return fmt.Errorf("physics3d: continuous motion overflow")
		}
	}
	return nil
}

func (w *World) earliestImpact(duration float64) (*Body, *Body, TOIResult, bool, error) {
	var firstA, firstB *Body
	var first TOIResult
	found := false
	for i, a := range w.bodies {
		for _, b := range w.bodies[i+1:] {
			if (!a.Continuous && !b.Continuous) || (a.invMass == 0 && b.invMass == 0) || w.collisionDisabled(a, b) {
				continue
			}
			possible, err := continuousBoundsOverlap(a, b, duration)
			if err != nil {
				return nil, nil, TOIResult{}, false, err
			}
			if !possible {
				continue
			}
			impact, hit, err := timeOfImpactBodies(a, b, duration)
			if err != nil {
				return nil, nil, TOIResult{}, false, err
			}
			if !hit {
				continue
			}
			approaching := true
			if impact.Time == 0 {
				approaching, err = continuousApproaching(a, b, impact.Contact, duration)
				if err != nil {
					return nil, nil, TOIResult{}, false, err
				}
			}
			if impact.Time == 0 && !approaching {
				if !continuousGeometryRotates(a) && !continuousGeometryRotates(b) {
					// Two convex shapes translating with constant velocities cannot
					// re-enter after separating along a contact normal.
					continue
				}
				impact, hit, err = continuousReentry(a, b, impact.Contact, duration)
				if err != nil {
					return nil, nil, TOIResult{}, false, err
				}
				if !hit {
					continue
				}
			}
			if !found || impact.Time < first.Time {
				firstA, firstB, first, found = a, b, impact, true
			}
		}
	}
	return firstA, firstB, first, found, nil
}

func continuousApproaching(a, b *Body, contact Contact, duration float64) (bool, error) {
	tolerance := toiDistanceTolerance(a, b)
	for _, point := range contact.Points {
		speed := relativeVelocity(a, b, point.Sub(a.Position), point.Sub(b.Position)).Dot(contact.Normal)
		if !finite(speed) {
			return false, fmt.Errorf("physics3d: continuous relative contact speed overflow")
		}
		if speed < 0 && -speed*duration > tolerance {
			return true, nil
		}
	}
	return false, nil
}

func continuousGeometryRotates(b *Body) bool {
	if b.kind == Static || b.fixedRotation || b.AngularVelocity == (Vec3{}) {
		return false
	}
	_, box := b.shape.(Box)
	return box
}

// continuousReentry removes only the initial contact from a query, not from the
// actual world. A query-only separation lets a rotating pair leave and hit again
// instead of being ignored for the remaining interval. For neutral persistent
// contact, a small continuation skin coalesces otherwise infinitesimal impacts;
// its size is at most 0.1% of the smaller feature and 0.0001 world units. Initial
// deep overlaps still receive the ordinary positional projection.
func continuousReentry(a, b *Body, contact Contact, duration float64) (TOIResult, bool, error) {
	copyA, copyB := *a, *b
	tolerance := toiDistanceTolerance(a, b)
	skin := 8 * tolerance
	minimumSpeed := math.Inf(1)
	for _, point := range contact.Points {
		speed := relativeVelocity(a, b, point.Sub(a.Position), point.Sub(b.Position)).Dot(contact.Normal)
		if !finite(speed) {
			return TOIResult{}, false, fmt.Errorf("physics3d: continuous re-entry speed overflow")
		}
		minimumSpeed = math.Min(minimumSpeed, speed)
	}
	if math.Abs(minimumSpeed)*duration <= tolerance {
		feature := math.Min(toiFeature(a), toiFeature(b))
		skin = math.Max(skin, math.Min(1e-4, 1e-3*feature))
	}
	separation := math.Max(contact.Penetration, 0) + skin
	weightA, weightB := 0.0, 0.0
	scale := math.Max(a.invMass, b.invMass)
	if scale > 0 {
		weightA, weightB = a.invMass/scale, b.invMass/scale
		sum := weightA + weightB
		weightA, weightB = weightA/sum, weightB/sum
	}
	copyA.Position = copyA.Position.Sub(contact.Normal.Mul(separation * weightA))
	copyB.Position = copyB.Position.Add(contact.Normal.Mul(separation * weightB))
	if !validVec(copyA.Position) || !validVec(copyB.Position) ||
		copyB.Position.Sub(copyA.Position).Sub(b.Position.Sub(a.Position)).Dot(contact.Normal) < separation*0.5 {
		return TOIResult{}, false, fmt.Errorf("physics3d: continuous contact separation is unrepresentable")
	}
	result, hit, err := timeOfImpactBodies(&copyA, &copyB, duration)
	if err == nil && hit && result.Time == 0 {
		return TOIResult{}, false, fmt.Errorf("physics3d: continuous contact did not separate within query tolerance")
	}
	return result, hit, err
}

func continuousBoundsOverlap(a, b *Body, duration float64) (bool, error) {
	minA, maxA, err := continuousBounds(a, duration)
	if err != nil {
		return false, err
	}
	minB, maxB, err := continuousBounds(b, duration)
	if err != nil {
		return false, err
	}
	return minA.X <= maxB.X && minB.X <= maxA.X && minA.Y <= maxB.Y && minB.Y <= maxA.Y && minA.Z <= maxB.Z && minB.Z <= maxA.Z, nil
}

func continuousBounds(b *Body, duration float64) (Vec3, Vec3, error) {
	radius := 0.0
	switch shape := b.shape.(type) {
	case Sphere:
		radius = shape.Radius
	case Box:
		radius = shape.HalfExtents.Length()
	}
	end := b.Position
	if b.kind != Static {
		end = end.Add(b.Velocity.Mul(duration))
	}
	min := Vec3{X: math.Min(b.Position.X, end.X) - radius, Y: math.Min(b.Position.Y, end.Y) - radius, Z: math.Min(b.Position.Z, end.Z) - radius}
	max := Vec3{X: math.Max(b.Position.X, end.X) + radius, Y: math.Max(b.Position.Y, end.Y) + radius, Z: math.Max(b.Position.Z, end.Z) + radius}
	if !finite(radius) || !validVec(end) || !validVec(min) || !validVec(max) {
		return Vec3{}, Vec3{}, fmt.Errorf("physics3d: continuous motion bounds overflow")
	}
	return min, max, nil
}

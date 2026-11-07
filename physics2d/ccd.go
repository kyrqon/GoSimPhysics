package physics2d

import (
	"errors"
	"fmt"
	"math"
)

// ErrTOILimit reports too many continuous impacts in one substep. Step restores
// all body state and pending forces; use a smaller dt or raise MaxTOIEvents.
var ErrTOILimit = errors.New("physics2d: CCD impact budget exhausted")

func (w *World) hasContinuous() bool {
	for _, body := range w.bodies {
		if body.Continuous {
			return true
		}
	}
	return false
}

// advanceContinuous consumes the entire interval, restarting sweeps whenever an
// impact changes velocity. Force integration remains outside this event loop.
func (w *World) advanceContinuous(dt float64) error {
	remaining := dt
	for events := 0; remaining > 0; {
		var first TOIResult
		var bodyA, bodyB *Body
		for i, a := range w.bodies {
			for _, b := range w.bodies[i+1:] {
				if !(a.Continuous || b.Continuous) || (a.invMass == 0 && b.invMass == 0) || w.collisionDisabled(a, b) {
					continue
				}
				overlap, err := sweptBoundsOverlap(a, b, remaining)
				if err != nil {
					return err
				}
				if !overlap {
					continue
				}
				hit, ok, err := continuousCandidate(a, b, remaining)
				if err != nil {
					return err
				}
				if ok && (bodyA == nil || hit.Time < first.Time) {
					first, bodyA, bodyB = hit, a, b
				}
			}
		}
		if bodyA == nil {
			return w.advanceCCDPoses(remaining)
		}
		if events >= w.MaxTOIEvents {
			return ErrTOILimit
		}
		if !finite(first.Time) || first.Time < 0 || first.Time > remaining {
			return fmt.Errorf("physics2d: invalid continuous impact time")
		}
		if err := w.advanceCCDPoses(first.Time); err != nil {
			return err
		}
		remaining = math.Max(0, remaining-first.Time)
		if err := w.solveCCDImpact(bodyA, bodyB, first.Contact, dt); err != nil {
			return err
		}
		events++
	}
	return nil
}

func continuousCandidate(a, b *Body, dt float64) (TOIResult, bool, error) {
	hit, ok, err := timeOfImpactBodies(a, b, dt)
	if err != nil || !ok || hit.Time > 0 {
		return hit, ok, err
	}
	// A separating contact is not another impact. For rotating shapes, move only
	// query copies apart by a tolerance-scale margin and sweep again, so a later
	// rotational re-entry is still considered. Real body poses never get nudged.
	speed := math.Inf(1)
	for _, point := range hit.Contact.Points {
		normalSpeed := relativeVelocity(a, b, point.Sub(a.Position), point.Sub(b.Position)).Dot(hit.Contact.Normal)
		if !finite(normalSpeed) {
			return TOIResult{}, false, fmt.Errorf("physics2d: CCD relative contact speed overflow")
		}
		speed = math.Min(speed, normalSpeed)
	}
	// Compare relative normal travel with spatial tolerance. A tolerance based
	// on absolute velocities would miss a slow approach atop fast common drift.
	if speed < 0 && -speed*dt > toiDistanceTolerance(a, b) {
		return hit, true, nil
	}
	if !rotatingGeometry(a) && !rotatingGeometry(b) {
		return TOIResult{}, false, nil
	}
	ca, cb := *a, *b
	margin := 8 * toiDistanceTolerance(a, b)
	if math.Abs(speed)*dt <= toiDistanceTolerance(a, b) {
		// A neutral rotating contact can immediately accelerate into the other
		// body again. A small continuation skin regularizes this persistent
		// contact, instead of requiring infinitely many zero-distance impacts.
		feature := math.Min(ccdFeature(a), ccdFeature(b))
		margin = math.Max(margin, math.Min(1e-4, 1e-3*feature))
	}
	slop := hit.Contact.Penetration + margin
	share := 0.5
	scale := math.Max(a.invMass, b.invMass)
	if scale > 0 {
		share = (a.invMass / scale) / (a.invMass/scale + b.invMass/scale)
	}
	ca.Position = ca.Position.Sub(hit.Contact.Normal.Mul(slop * share))
	cb.Position = cb.Position.Add(hit.Contact.Normal.Mul(slop * (1 - share)))
	if !validVec(ca.Position) || !validVec(cb.Position) || cb.Position.Sub(ca.Position).Sub(b.Position.Sub(a.Position)).Dot(hit.Contact.Normal) < slop*0.5 {
		return TOIResult{}, false, fmt.Errorf("physics2d: CCD contact separation is unrepresentable")
	}
	result, found, err := timeOfImpactBodies(&ca, &cb, dt)
	if err == nil && found && result.Time == 0 {
		return TOIResult{}, false, fmt.Errorf("physics2d: CCD contact did not separate within query tolerance")
	}
	return result, found, err
}

func ccdFeature(b *Body) float64 {
	switch shape := b.shape.(type) {
	case Circle:
		return shape.Radius
	case Box:
		return math.Min(shape.HalfExtents.X, shape.HalfExtents.Y)
	}
	return 0
}

func rotatingGeometry(b *Body) bool {
	_, box := b.shape.(Box)
	return box && b.kind != Static && !b.fixedRotation && b.AngularVelocity != 0
}

func (w *World) advanceCCDPoses(dt float64) error {
	for _, b := range w.bodies {
		pose, err := toiPose(b, dt)
		if err != nil {
			return err
		}
		b.Position, b.Angle = pose.Position, pose.Angle
		if !b.validState() || !bodyBoundsFinite(b) {
			return fmt.Errorf("physics2d: CCD motion overflow")
		}
	}
	return nil
}

func (w *World) solveCCDImpact(a, b *Body, impact Contact, dt float64) error {
	contacts, err := w.contactConstraints()
	if err != nil {
		return err
	}
	found := false
	for _, c := range contacts {
		found = found || c.a == a && c.b == b
	}
	if !found {
		c, err := newContactConstraint(a, b, impact, w.RestitutionThreshold)
		if err != nil {
			return err
		}
		contacts = append(contacts, c)
	}
	for _, joint := range w.joints {
		if !joint.validGeometry() {
			return fmt.Errorf("physics2d: CCD joint geometry overflow")
		}
		joint.prepare(dt)
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
		if !body.validState() {
			return fmt.Errorf("physics2d: CCD impulse overflow")
		}
	}
	return nil
}

func sweptBoundsOverlap(a, b *Body, dt float64) (bool, error) {
	amin, amax, err := sweptBounds(a, dt)
	if err != nil {
		return false, err
	}
	bmin, bmax, err := sweptBounds(b, dt)
	if err != nil {
		return false, err
	}
	return amin.X <= bmax.X && bmin.X <= amax.X && amin.Y <= bmax.Y && bmin.Y <= amax.Y, nil
}

func sweptBounds(b *Body, dt float64) (min, max Vec2, err error) {
	min, max = bodyAABB(b)
	if rotatingGeometry(b) {
		radius := b.shape.(Box).HalfExtents.Length()
		min, max = b.Position.Sub(Vec2{X: radius, Y: radius}), b.Position.Add(Vec2{X: radius, Y: radius})
	}
	if b.kind != Static {
		travel := b.Velocity.Mul(dt)
		endMin, endMax := min.Add(travel), max.Add(travel)
		min = Vec2{X: math.Min(min.X, endMin.X), Y: math.Min(min.Y, endMin.Y)}
		max = Vec2{X: math.Max(max.X, endMax.X), Y: math.Max(max.Y, endMax.Y)}
	}
	if !validVec(min) || !validVec(max) {
		return Vec2{}, Vec2{}, fmt.Errorf("physics2d: swept bounds overflow")
	}
	return min, max, nil
}

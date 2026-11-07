package physics

import (
	"fmt"
	"math"
)

// World owns a 2D simulation. A world and its bodies must be accessed from one
// goroutine at a time. Use consistent units, such as meters, kilograms, seconds,
// and radians, and a fixed time step. The zero World needs solver settings;
// NewWorld supplies defaults.
type World struct {
	Gravity Vec2
	// Substeps divides each Step into smaller integration and solver intervals.
	Substeps                               int
	VelocityIterations, PositionIterations int
	// RestitutionThreshold suppresses bounce below this relative approach speed.
	RestitutionThreshold float64
	// MaxTOIEvents limits CCD impacts per substep. Exhaustion rolls back Step.
	MaxTOIEvents int

	bodies []*Body
	joints []*DistanceJoint
}

// NewWorld creates a world with four substeps, twelve velocity iterations, four
// position iterations, and a restitution speed threshold of one unit per second.
// Step validates Gravity and the solver settings before changing any body.
func NewWorld(gravity Vec2) *World {
	return &World{
		Gravity: gravity, Substeps: 4, VelocityIterations: 12,
		PositionIterations: 4, RestitutionThreshold: 1, MaxTOIEvents: 32,
	}
}

// CreateBody validates def and creates a body owned by w.
func (w *World) CreateBody(def BodyDef) (*Body, error) {
	if w == nil {
		return nil, fmt.Errorf("physics: nil world")
	}
	b, err := newBody(def)
	if err != nil {
		return nil, err
	}
	b.world = w
	w.bodies = append(w.bodies, b)
	return b, nil
}

// Bodies returns a copy of the body list in creation order. The bodies themselves
// remain owned by the world; changing this slice does not change membership.
func (w *World) Bodies() []*Body {
	if w == nil {
		return nil
	}
	return append([]*Body(nil), w.bodies...)
}

// RemoveBody removes a body and every joint attached to it. It reports whether
// the body was a member of this world. Other bodies retain their creation order.
func (w *World) RemoveBody(b *Body) bool {
	if w == nil || b == nil || b.world != w {
		return false
	}
	for i, candidate := range w.bodies {
		if candidate != b {
			continue
		}
		copy(w.bodies[i:], w.bodies[i+1:])
		w.bodies[len(w.bodies)-1] = nil
		w.bodies = w.bodies[:len(w.bodies)-1]
		remaining := w.joints[:0]
		for _, joint := range w.joints {
			if joint.bodyA != b && joint.bodyB != b {
				remaining = append(remaining, joint)
			}
		}
		clear(w.joints[len(remaining):])
		w.joints = remaining
		b.world = nil
		return true
	}
	return false
}

type bodyState struct {
	position, velocity     Vec2
	angle, angularVelocity float64
}

// Step advances by dt seconds using semi-implicit Euler, sequential contact
// impulses, and positional constraint correction. dt must be positive and finite.
// Forces and torques act over all substeps and clear only after a successful step.
// Invalid input or arithmetic overflow leaves the body's state and forces intact.
// Bodies with Continuous enabled use swept collision detection between impacts.
func (w *World) Step(dt float64) error {
	if w == nil {
		return fmt.Errorf("physics: nil world")
	}
	if !finite(dt) || dt <= 0 || w.Substeps < 1 || w.VelocityIterations < 1 || w.PositionIterations < 1 || w.MaxTOIEvents < 1 {
		return fmt.Errorf("physics: positive finite dt and positive solver iteration counts required")
	}
	h := dt / float64(w.Substeps)
	if h == 0 || !validVec(w.Gravity) || !finite(w.RestitutionThreshold) || w.RestitutionThreshold < 0 {
		return fmt.Errorf("physics: invalid gravity, restitution threshold, or substep duration")
	}
	for _, b := range w.bodies {
		if !b.validState() || !validVec(b.force) || !finite(b.torque) {
			return fmt.Errorf("physics: body state and forces must be finite")
		}
		if !bodyBoundsFinite(b) {
			return fmt.Errorf("physics: body bounds overflow")
		}
	}
	before := make([]bodyState, len(w.bodies))
	for i, b := range w.bodies {
		before[i] = bodyState{b.Position, b.Velocity, b.Angle, b.AngularVelocity}
	}
	rollback := func(cause error) error {
		for i, b := range w.bodies {
			b.Position, b.Velocity = before[i].position, before[i].velocity
			b.Angle, b.AngularVelocity = before[i].angle, before[i].angularVelocity
		}
		return fmt.Errorf("physics: step failed: %w", cause)
	}
	useCCD := w.hasContinuous()
	for substep := 0; substep < w.Substeps; substep++ {
		for _, b := range w.bodies {
			if b.kind == Static {
				b.Velocity, b.AngularVelocity = Vec2{}, 0
				continue
			}
			if b.kind == Dynamic {
				acceleration := w.Gravity.Add(b.force.Mul(b.invMass))
				b.Velocity = b.Velocity.Add(acceleration.Mul(h)).Mul(1 / (1 + h*b.linearDamping))
				b.AngularVelocity = (b.AngularVelocity + b.torque*b.invInertia*h) / (1 + h*b.angularDamping)
			}
			if b.fixedRotation {
				b.AngularVelocity = 0
			}
			if !useCCD {
				b.Position = b.Position.Add(b.Velocity.Mul(h))
				b.Angle += b.AngularVelocity * h
			}
			if !b.validState() || !bodyBoundsFinite(b) {
				return rollback(fmt.Errorf("integration overflow; reduce dt or input magnitudes"))
			}
		}
		if useCCD {
			if err := w.advanceContinuous(h); err != nil {
				return rollback(err)
			}
		}
		contacts, err := w.contactConstraints()
		if err != nil {
			return rollback(err)
		}
		for _, joint := range w.joints {
			if !joint.validGeometry() {
				return rollback(fmt.Errorf("joint geometry overflow"))
			}
			joint.prepare(h)
		}
		for i := 0; i < w.VelocityIterations; i++ {
			for c := range contacts {
				contacts[c].solveVelocity()
			}
			for _, joint := range w.joints {
				joint.solveVelocity()
			}
		}
		for i := 0; i < w.PositionIterations; i++ {
			for _, contact := range contacts {
				if !collisionGeometryFinite(contact.a, contact.b) {
					return rollback(fmt.Errorf("collision geometry overflow"))
				}
				correctContactPosition(contact.a, contact.b)
			}
			for _, joint := range w.joints {
				if !joint.validGeometry() {
					return rollback(fmt.Errorf("joint geometry overflow"))
				}
				joint.solvePosition()
			}
		}
		for _, b := range w.bodies {
			if !b.validState() || !bodyBoundsFinite(b) {
				return rollback(fmt.Errorf("constraint solver overflow; reduce dt or input magnitudes"))
			}
		}
		for _, joint := range w.joints {
			if !joint.validGeometry() {
				return rollback(fmt.Errorf("joint geometry overflow"))
			}
		}
	}
	for _, b := range w.bodies {
		b.force, b.torque = Vec2{}, 0
	}
	return nil
}

func (w *World) collisionDisabled(a, b *Body) bool {
	for _, joint := range w.joints {
		if !joint.collideConnected && ((joint.bodyA == a && joint.bodyB == b) || (joint.bodyA == b && joint.bodyB == a)) {
			return true
		}
	}
	return false
}

func bodyBoundsFinite(b *Body) bool {
	min, max := bodyAABB(b)
	return validVec(min) && validVec(max)
}

func (w *World) contactConstraints() ([]contactConstraint, error) {
	var contacts []contactConstraint
	for i, a := range w.bodies {
		for _, b := range w.bodies[i+1:] {
			if a.invMass+b.invMass == 0 || w.collisionDisabled(a, b) || !aabbOverlap(a, b) {
				continue
			}
			if !collisionGeometryFinite(a, b) {
				return nil, fmt.Errorf("physics: collision geometry overflow")
			}
			if contact, ok := Collide(a, b); ok {
				constraint, err := newContactConstraint(a, b, contact, w.RestitutionThreshold)
				if err != nil {
					return nil, err
				}
				contacts = append(contacts, constraint)
			}
		}
	}
	return contacts, nil
}

type contactPoint struct {
	point, rA, rB                   Vec2
	normalMass, tangentMass, bounce float64
	normalImpulse, tangentImpulse   float64
}

type contactConstraint struct {
	a, b            *Body
	normal, tangent Vec2
	friction        float64
	points          []contactPoint
}

func relativeVelocity(a, b *Body, rA, rB Vec2) Vec2 {
	return b.Velocity.Add(angularVelocityAt(b.AngularVelocity, rB)).Sub(a.Velocity.Add(angularVelocityAt(a.AngularVelocity, rA)))
}

func effectiveMass(a, b *Body, rA, rB, axis Vec2) float64 {
	ca, cb := rA.Cross(axis), rB.Cross(axis)
	// Multiplying by inverse inertia first keeps long static shapes (invI=0)
	// from evaluating an overflowing cross-product square followed by Inf*0.
	return a.invMass + b.invMass + a.invInertia*ca*ca + b.invInertia*cb*cb
}

func newContactConstraint(a, b *Body, contact Contact, threshold float64) (contactConstraint, error) {
	c := contactConstraint{
		a: a, b: b, normal: contact.Normal,
		tangent:  Vec2{-contact.Normal.Y, contact.Normal.X},
		friction: math.Sqrt(a.friction) * math.Sqrt(b.friction),
	}
	if !finite(c.friction) {
		return contactConstraint{}, fmt.Errorf("physics: contact friction overflow")
	}
	for _, point := range contact.Points {
		rA, rB := point.Sub(a.Position), point.Sub(b.Position)
		p := contactPoint{point: point, rA: rA, rB: rB}
		kn := effectiveMass(a, b, rA, rB, c.normal)
		kt := effectiveMass(a, b, rA, rB, c.tangent)
		if !validVec(rA) || !validVec(rB) || !finite(kn) || !finite(kt) || kn <= 0 || kt <= 0 {
			return contactConstraint{}, fmt.Errorf("physics: contact effective mass overflow")
		}
		p.normalMass, p.tangentMass = 1/kn, 1/kt
		speed := relativeVelocity(a, b, rA, rB).Dot(c.normal)
		if !finite(speed) {
			return contactConstraint{}, fmt.Errorf("physics: contact speed overflow")
		}
		if speed < -threshold {
			p.bounce = -math.Max(a.restitution, b.restitution) * speed
		}
		c.points = append(c.points, p)
	}
	return c, nil
}

func (c *contactConstraint) solveVelocity() {
	for i := range c.points {
		p := &c.points[i]
		speed := relativeVelocity(c.a, c.b, p.rA, p.rB).Dot(c.normal)
		old := p.normalImpulse
		p.normalImpulse = math.Max(old+(p.bounce-speed)*p.normalMass, 0)
		impulse := c.normal.Mul(p.normalImpulse - old)
		c.a.applyImpulse(impulse.Mul(-1), p.point)
		c.b.applyImpulse(impulse, p.point)

		speed = relativeVelocity(c.a, c.b, p.rA, p.rB).Dot(c.tangent)
		old = p.tangentImpulse
		limit := c.friction * p.normalImpulse
		p.tangentImpulse = clamp(old-speed*p.tangentMass, -limit, limit)
		impulse = c.tangent.Mul(p.tangentImpulse - old)
		c.a.applyImpulse(impulse.Mul(-1), p.point)
		c.b.applyImpulse(impulse, p.point)
	}
}

func correctContactPosition(a, b *Body) {
	contact, ok := Collide(a, b)
	if !ok || len(contact.Points) == 0 {
		return
	}
	// Correct only penetration beyond a small slop, without changing velocities.
	// This keeps stabilization from adding rebound energy to resting contacts.
	correction := math.Min(0.2, 0.4*math.Max(contact.Penetration-0.001, 0)) / float64(len(contact.Points))
	for _, point := range contact.Points {
		rA, rB := point.Sub(a.Position), point.Sub(b.Position)
		k := effectiveMass(a, b, rA, rB, contact.Normal)
		if k <= 0 {
			continue
		}
		impulse := contact.Normal.Mul(correction / k)
		a.Position = a.Position.Sub(impulse.Mul(a.invMass))
		if a.invInertia != 0 {
			a.Angle -= rA.Cross(impulse) * a.invInertia
		}
		b.Position = b.Position.Add(impulse.Mul(b.invMass))
		if b.invInertia != 0 {
			b.Angle += rB.Cross(impulse) * b.invInertia
		}
	}
}

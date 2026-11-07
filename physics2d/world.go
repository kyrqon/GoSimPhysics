package physics2d

import (
	"fmt"
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
		return nil, fmt.Errorf("physics2d: nil world")
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
		return fmt.Errorf("physics2d: nil world")
	}
	if !finite(dt) || dt <= 0 || w.Substeps < 1 || w.VelocityIterations < 1 || w.PositionIterations < 1 || w.MaxTOIEvents < 1 {
		return fmt.Errorf("physics2d: positive finite dt and positive solver iteration counts required")
	}
	h := dt / float64(w.Substeps)
	if h == 0 || !validVec(w.Gravity) || !finite(w.RestitutionThreshold) || w.RestitutionThreshold < 0 {
		return fmt.Errorf("physics2d: invalid gravity, restitution threshold, or substep duration")
	}
	for _, b := range w.bodies {
		if !b.validState() || !validVec(b.force) || !finite(b.torque) {
			return fmt.Errorf("physics2d: body state and forces must be finite")
		}
		if !bodyBoundsFinite(b) {
			return fmt.Errorf("physics2d: body bounds overflow")
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
		return fmt.Errorf("physics2d: step failed: %w", cause)
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
				return nil, fmt.Errorf("physics2d: collision geometry overflow")
			}
			if contact, ok := collideBodies(a, b); ok {
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

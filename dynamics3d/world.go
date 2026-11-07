package dynamics3d

import "fmt"

// World owns a three-dimensional rigid-body simulation. Use consistent units
// and a fixed time step. A world and its bodies are not safe for concurrent use.
type World struct {
	Gravity                                          Vec3
	Substeps, VelocityIterations, PositionIterations int
	// RestitutionThreshold suppresses bounce below this approach speed.
	RestitutionThreshold float64
	// MaxTOIEvents bounds impacts in each substep when Continuous bodies exist.
	// Exhausting the budget rolls back the entire Step rather than tunneling.
	MaxTOIEvents int

	bodies []*Body
	joints []*DistanceJoint
}

// NewWorld supplies four substeps, twelve velocity iterations, four position
// iterations, and a restitution speed threshold of one world unit per second.
func NewWorld(gravity Vec3) *World {
	return &World{Gravity: gravity, Substeps: 4, VelocityIterations: 12,
		PositionIterations: 4, RestitutionThreshold: 1, MaxTOIEvents: 32}
}

// CreateBody validates a definition and adds its body to the world.
func (w *World) CreateBody(def BodyDef) (*Body, error) {
	if w == nil {
		return nil, fmt.Errorf("dynamics3d: nil world")
	}
	b, err := newBody(def)
	if err != nil {
		return nil, err
	}
	b.world = w
	w.bodies = append(w.bodies, b)
	return b, nil
}

// Bodies returns a copy of the body list in creation order, not copies of the
// bodies. Changing the returned slice does not change world membership.
func (w *World) Bodies() []*Body {
	if w == nil {
		return nil
	}
	return append([]*Body(nil), w.bodies...)
}

// RemoveBody removes b and every joint attached to it. It succeeds only once
// and only for a body owned by this world.
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
	position, velocity, angularVelocity Vec3
	rotation                            Quaternion
}

// Step advances dt seconds with semi-implicit Euler translation and iterative
// contact/joint impulses. Rotation uses world angular momentum and a midpoint
// quaternion update, including the changing inertia of anisotropic bodies.
// Forces act over all substeps and clear only after success. Invalid input or
// unrepresentable arithmetic preserves all body state and accumulated forces.
// Continuous bodies use swept contact times and consume the time left after each
// impact. Other pairs retain discrete collision detection.
func (w *World) Step(dt float64) error {
	if w == nil {
		return fmt.Errorf("dynamics3d: nil world")
	}
	if !finite(dt) || dt <= 0 || w.Substeps < 1 || w.VelocityIterations < 1 || w.PositionIterations < 1 || w.MaxTOIEvents < 1 {
		return fmt.Errorf("dynamics3d: positive finite dt and positive solver settings required")
	}
	h := dt / float64(w.Substeps)
	if h == 0 || !validVec(w.Gravity) || !finite(w.RestitutionThreshold) || w.RestitutionThreshold < 0 {
		return fmt.Errorf("dynamics3d: invalid gravity, restitution threshold, or substep duration")
	}
	for _, b := range w.bodies {
		if !b.validState() || !validVec(b.force) || !validVec(b.torque) {
			return fmt.Errorf("dynamics3d: body state and forces must be finite, with a nonzero rotation")
		}
	}
	before := make([]bodyState, len(w.bodies))
	for i, b := range w.bodies {
		before[i] = bodyState{b.Position, b.Velocity, b.AngularVelocity, b.Rotation}
		b.Rotation = b.Rotation.Normalized()
	}
	rollback := func(cause error) error {
		for i, b := range w.bodies {
			b.Position, b.Velocity = before[i].position, before[i].velocity
			b.AngularVelocity, b.Rotation = before[i].angularVelocity, before[i].rotation
		}
		return fmt.Errorf("dynamics3d: step failed: %w", cause)
	}
	for _, b := range w.bodies {
		if !bodyBoundsFinite(b) {
			return rollback(fmt.Errorf("body bounds overflow"))
		}
	}
	continuous := w.hasContinuousBody()
	for substep := 0; substep < w.Substeps; substep++ {
		for _, b := range w.bodies {
			if b.kind == Static {
				b.Velocity, b.AngularVelocity = Vec3{}, Vec3{}
				continue
			}
			if b.kind == Dynamic {
				acceleration := w.Gravity.Add(b.force.Mul(b.invMass))
				b.Velocity = b.Velocity.Add(acceleration.Mul(h)).Mul(1 / (1 + h*b.linearDamping))
				if !b.fixedRotation {
					// Evolving world momentum accounts for gyroscopic motion without
					// explicitly integrating the potentially stiff omega x I*omega.
					momentum := b.inertiaWorld(b.AngularVelocity).Add(b.torque.Mul(h)).Mul(1 / (1 + h*b.angularDamping))
					omega := b.inverseInertiaWorld(momentum)
					if continuous {
						b.AngularVelocity = omega
					} else {
						halfRotation := integrateRotation(b.Rotation, omega.Mul(h/2))
						midOmega := inverseInertiaAt(halfRotation, b.invInertia, momentum)
						b.Rotation = integrateRotation(b.Rotation, midOmega.Mul(h))
						b.AngularVelocity = b.inverseInertiaWorld(momentum)
					}
				}
			} else if !b.fixedRotation && !continuous {
				b.Rotation = integrateRotation(b.Rotation, b.AngularVelocity.Mul(h))
			}
			if b.fixedRotation {
				b.AngularVelocity = Vec3{}
			}
			if !continuous {
				b.Position = b.Position.Add(b.Velocity.Mul(h))
			}
			if !b.validState() || !bodyBoundsFinite(b) {
				return rollback(fmt.Errorf("integration overflow; reduce dt or input magnitudes"))
			}
		}
		if continuous {
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
			for _, c := range contacts {
				if err := correctContactPosition(c.a, c.b); err != nil {
					return rollback(err)
				}
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
		b.force, b.torque = Vec3{}, Vec3{}
	}
	return nil
}

func inverseInertiaAt(rotation Quaternion, inverse Vec3, momentum Vec3) Vec3 {
	local := rotation.Conjugate().Rotate(momentum)
	return rotation.Rotate(Vec3{X: local.X * inverse.X, Y: local.Y * inverse.Y, Z: local.Z * inverse.Z})
}

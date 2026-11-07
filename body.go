package physics

import "fmt"

// BodyType determines how a body moves and responds to forces and collisions.
type BodyType uint8

const (
	// Static bodies stay fixed and have infinite effective mass.
	Static BodyType = iota
	// Dynamic bodies respond to gravity, applied forces, impulses, and collisions.
	Dynamic
	// Kinematic bodies move at their specified velocity without force response.
	Kinematic
)

// BodyDef configures a body at creation. Mass is required for Dynamic bodies;
// Static and Kinematic bodies ignore its value. Zero friction means frictionless,
// and zero restitution means inelastic contact. All numeric values must be finite.
type BodyDef struct {
	Type                          BodyType
	Shape                         Shape
	Position, Velocity            Vec2
	Angle, AngularVelocity        float64
	Mass, Restitution, Friction   float64
	LinearDamping, AngularDamping float64
	FixedRotation                 bool
}

// Body is a rigid body with a single centered shape. Position and Velocity use
// world coordinates; Angle and AngularVelocity use radians and radians per second.
// Public state may be changed between steps and must remain finite. Bodies belong
// to one World and must not be copied or mutated concurrently with simulation.
type Body struct {
	Position, Velocity     Vec2
	Angle, AngularVelocity float64

	kind                          BodyType
	shape                         Shape
	mass, invMass                 float64
	inertia, invInertia           float64
	restitution, friction         float64
	linearDamping, angularDamping float64
	force                         Vec2
	torque                        float64
	world                         *World
	fixedRotation                 bool
}

func newBody(def BodyDef) (*Body, error) {
	shape, shapeValid := copyShape(def.Shape)
	if !shapeValid {
		return nil, fmt.Errorf("physics: body requires a valid Circle or Box")
	}
	if def.Type > Kinematic {
		return nil, fmt.Errorf("physics: invalid body type %d", def.Type)
	}
	if !validVec(def.Position) || !validVec(def.Velocity) || !finite(def.Angle) || !finite(def.AngularVelocity) {
		return nil, fmt.Errorf("physics: body transform and velocity must be finite")
	}
	if !finite(def.Mass) || def.Mass < 0 || (def.Type == Dynamic && def.Mass == 0) {
		return nil, fmt.Errorf("physics: dynamic mass must be positive; other mass values must be nonnegative and finite")
	}
	if !finite(def.Restitution) || def.Restitution < 0 || def.Restitution > 1 {
		return nil, fmt.Errorf("physics: restitution must be between zero and one")
	}
	if !finite(def.Friction) || def.Friction < 0 || !finite(def.LinearDamping) || def.LinearDamping < 0 || !finite(def.AngularDamping) || def.AngularDamping < 0 {
		return nil, fmt.Errorf("physics: friction and damping must be nonnegative and finite")
	}
	body := &Body{
		Position: def.Position, Velocity: def.Velocity,
		Angle: def.Angle, AngularVelocity: def.AngularVelocity,
		kind: def.Type, shape: shape,
		restitution: def.Restitution, friction: def.Friction,
		linearDamping: def.LinearDamping, angularDamping: def.AngularDamping,
		fixedRotation: def.FixedRotation,
	}
	if def.Type == Dynamic {
		body.mass, body.invMass = def.Mass, 1/def.Mass
		body.inertia = shape.inertia(def.Mass)
		if !finite(body.invMass) || body.invMass <= 0 || !finite(body.inertia) || body.inertia <= 0 {
			return nil, fmt.Errorf("physics: mass and shape produce unrepresentable mass properties")
		}
		body.invInertia = 1 / body.inertia
		if !finite(body.invInertia) || body.invInertia <= 0 {
			return nil, fmt.Errorf("physics: shape produces unrepresentable inverse inertia")
		}
	}
	if def.Type == Static {
		body.Velocity, body.AngularVelocity = Vec2{}, 0
	}
	if def.FixedRotation {
		body.invInertia, body.AngularVelocity = 0, 0
	}
	return body, nil
}

// Type returns the body's motion type.
func (b *Body) Type() BodyType { return b.kind }

// Shape returns the body's immutable shape value.
func (b *Body) Shape() Shape { return b.shape }

// Mass returns the body's mass. Static and Kinematic bodies report zero, while
// behaving as infinite mass during collision and constraint solving.
func (b *Body) Mass() float64 { return b.mass }

// Inertia returns the moment of inertia about the center of mass. Static and
// Kinematic bodies report zero. FixedRotation keeps a Dynamic body's physical
// inertia but disables angular response.
func (b *Body) Inertia() float64 { return b.inertia }

// ApplyForce accumulates a world-space force at the center of mass for the next
// successful World.Step. Only Dynamic bodies respond to forces.
func (b *Body) ApplyForce(force Vec2) error {
	if !validVec(force) {
		return fmt.Errorf("physics: force must be finite")
	}
	if b.kind != Dynamic {
		return nil
	}
	combined := b.force.Add(force)
	if !validVec(combined) {
		return fmt.Errorf("physics: accumulated force overflow")
	}
	b.force = combined
	return nil
}

// ApplyForceAtPoint accumulates a world-space force at worldPoint, producing
// torque about the body's center as well as a linear force.
func (b *Body) ApplyForceAtPoint(force, worldPoint Vec2) error {
	if !validVec(force) || !validVec(worldPoint) {
		return fmt.Errorf("physics: force and application point must be finite")
	}
	if b.kind != Dynamic {
		return nil
	}
	combinedForce := b.force.Add(force)
	combinedTorque := b.torque
	if !b.fixedRotation {
		combinedTorque += worldPoint.Sub(b.Position).Cross(force)
	}
	if !validVec(combinedForce) || !finite(combinedTorque) {
		return fmt.Errorf("physics: accumulated force or torque overflow")
	}
	b.force, b.torque = combinedForce, combinedTorque
	return nil
}

// ApplyTorque accumulates counterclockwise torque for the next successful step.
// It has no effect on bodies with fixed rotation or nondynamic bodies.
func (b *Body) ApplyTorque(torque float64) error {
	if !finite(torque) {
		return fmt.Errorf("physics: torque must be finite")
	}
	if b.kind != Dynamic || b.fixedRotation {
		return nil
	}
	combined := b.torque + torque
	if !finite(combined) {
		return fmt.Errorf("physics: accumulated torque overflow")
	}
	b.torque = combined
	return nil
}

// ApplyImpulse instantly changes velocity using a world-space impulse at
// worldPoint. An off-center impulse can also change angular velocity.
func (b *Body) ApplyImpulse(impulse, worldPoint Vec2) error {
	if !validVec(impulse) || !validVec(worldPoint) || !b.validState() {
		return fmt.Errorf("physics: impulse, application point, and body state must be finite")
	}
	if b.kind != Dynamic {
		return nil
	}
	velocity := b.Velocity.Add(impulse.Mul(b.invMass))
	angularVelocity := b.AngularVelocity
	if b.invInertia != 0 {
		angularVelocity += worldPoint.Sub(b.Position).Cross(impulse) * b.invInertia
	}
	if !validVec(velocity) || !finite(angularVelocity) {
		return fmt.Errorf("physics: impulse produces unrepresentable velocity")
	}
	b.Velocity, b.AngularVelocity = velocity, angularVelocity
	return nil
}

// WorldPoint transforms a local point to world coordinates.
func (b *Body) WorldPoint(localPoint Vec2) Vec2 {
	return b.Position.Add(localPoint.Rotate(b.Angle))
}

// LocalPoint transforms a world point to body-local coordinates.
func (b *Body) LocalPoint(worldPoint Vec2) Vec2 {
	return worldPoint.Sub(b.Position).Rotate(-b.Angle)
}

func (b *Body) applyImpulse(impulse, point Vec2) {
	if b.kind == Dynamic {
		b.Velocity = b.Velocity.Add(impulse.Mul(b.invMass))
		if b.invInertia != 0 {
			b.AngularVelocity += point.Sub(b.Position).Cross(impulse) * b.invInertia
		}
	}
}

func (b *Body) validState() bool {
	return validVec(b.Position) && validVec(b.Velocity) && finite(b.Angle) && finite(b.AngularVelocity)
}

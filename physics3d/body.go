package physics3d

import "fmt"

// BodyType determines how a body moves and responds to forces and collisions.
type BodyType uint8

const (
	// Static bodies stay fixed and have infinite effective mass.
	Static BodyType = iota
	// Dynamic bodies respond to forces, torques, impulses, and constraints.
	Dynamic
	// Kinematic bodies move at their specified velocities without force response.
	Kinematic
)

// BodyDef configures a body at creation. Dynamic bodies require positive Mass.
// Other bodies ignore finite, nonnegative Mass values. A zero Rotation defaults
// to Identity. All coordinates and material values must be finite.
type BodyDef struct {
	Type                          BodyType
	Shape                         Shape
	Position, Velocity            Vec3
	AngularVelocity               Vec3
	Rotation                      Quaternion
	Mass, Restitution, Friction   float64
	LinearDamping, AngularDamping float64
	FixedRotation                 bool
	// Continuous enables swept collision handling for pairs involving this body.
	Continuous bool
}

// Body is a rigid body with one centered shape. Linear and angular velocities
// are world-space vectors. Rotations use unit quaternions and right-handed axes.
// Public state may be edited between steps but must remain finite, with a
// nonzero rotation. Bodies belong to one World and must not be copied or shared
// concurrently while the world is being stepped.
type Body struct {
	Position, Velocity, AngularVelocity Vec3
	Rotation                            Quaternion
	// Continuous enables CCD and may be changed between steps.
	Continuous bool

	kind                          BodyType
	shape                         Shape
	mass, invMass                 float64
	inertia, invInertia           Vec3
	restitution, friction         float64
	linearDamping, angularDamping float64
	force, torque                 Vec3
	world                         *World
	fixedRotation                 bool
}

func newBody(def BodyDef) (*Body, error) {
	shape, shapeValid := copyShape(def.Shape)
	if !shapeValid {
		return nil, fmt.Errorf("physics3d: body requires a valid Sphere or Box")
	}
	if def.Type > Kinematic {
		return nil, fmt.Errorf("physics3d: invalid body type %d", def.Type)
	}
	if def.Rotation == (Quaternion{}) {
		def.Rotation = Identity()
	}
	if !validVec(def.Position) || !validVec(def.Velocity) || !validVec(def.AngularVelocity) || !validQuat(def.Rotation) {
		return nil, fmt.Errorf("physics3d: body transform and velocities must be finite")
	}
	if !finite(def.Mass) || def.Mass < 0 || (def.Type == Dynamic && def.Mass == 0) {
		return nil, fmt.Errorf("physics3d: dynamic mass must be positive; all mass values must be nonnegative and finite")
	}
	if !finite(def.Restitution) || def.Restitution < 0 || def.Restitution > 1 {
		return nil, fmt.Errorf("physics3d: restitution must be between zero and one")
	}
	if !finite(def.Friction) || def.Friction < 0 || !finite(def.LinearDamping) || def.LinearDamping < 0 || !finite(def.AngularDamping) || def.AngularDamping < 0 {
		return nil, fmt.Errorf("physics3d: friction and damping must be nonnegative and finite")
	}
	b := &Body{
		Position: def.Position, Velocity: def.Velocity, AngularVelocity: def.AngularVelocity,
		Rotation: def.Rotation.Normalized(), kind: def.Type, shape: shape,
		restitution: def.Restitution, friction: def.Friction,
		linearDamping: def.LinearDamping, angularDamping: def.AngularDamping,
		fixedRotation: def.FixedRotation,
		Continuous:    def.Continuous,
	}
	if def.Type == Dynamic {
		b.mass, b.invMass, b.inertia = def.Mass, 1/def.Mass, shape.inertia(def.Mass)
		if !finite(b.invMass) || b.invMass <= 0 || !positiveFiniteVec(b.inertia) {
			return nil, fmt.Errorf("physics3d: mass and shape produce unrepresentable mass properties")
		}
		b.invInertia = Vec3{1 / b.inertia.X, 1 / b.inertia.Y, 1 / b.inertia.Z}
		if !positiveFiniteVec(b.invInertia) {
			return nil, fmt.Errorf("physics3d: shape produces unrepresentable inverse inertia")
		}
	}
	if def.Type == Static {
		b.Velocity, b.AngularVelocity = Vec3{}, Vec3{}
	}
	if def.FixedRotation {
		b.invInertia, b.AngularVelocity = Vec3{}, Vec3{}
	}
	return b, nil
}

func positiveFiniteVec(v Vec3) bool { return validVec(v) && v.X > 0 && v.Y > 0 && v.Z > 0 }

// Type returns the body's motion type.
func (b *Body) Type() BodyType { return b.kind }

// Shape returns the body's immutable shape value.
func (b *Body) Shape() Shape { return b.shape }

// Mass returns the mass, or zero for Static and Kinematic bodies, which behave
// as infinite mass during constraint solving.
func (b *Body) Mass() float64 { return b.mass }

// Inertia returns the principal moments of inertia in body-local coordinates.
// FixedRotation retains physical inertia but disables angular response.
func (b *Body) Inertia() Vec3 { return b.inertia }

// AngularMomentum returns angular momentum about the center of mass in world
// coordinates. Static and Kinematic bodies report zero.
func (b *Body) AngularMomentum() Vec3 { return b.inertiaWorld(b.AngularVelocity) }

// ApplyForce accumulates a world-space force at the center of mass until the next
// successful Step. Only Dynamic bodies respond to forces.
func (b *Body) ApplyForce(force Vec3) error {
	if !validVec(force) {
		return fmt.Errorf("physics3d: force must be finite")
	}
	if b.kind != Dynamic {
		return nil
	}
	combined := b.force.Add(force)
	if !validVec(combined) {
		return fmt.Errorf("physics3d: accumulated force overflow")
	}
	b.force = combined
	return nil
}

// ApplyForceAtPoint accumulates a force at a world-space point, producing torque
// about the center of mass as well as a linear force.
func (b *Body) ApplyForceAtPoint(force, worldPoint Vec3) error {
	if !validVec(force) || !validVec(worldPoint) || !b.validState() {
		return fmt.Errorf("physics3d: force, application point, and body state must be finite")
	}
	if b.kind != Dynamic {
		return nil
	}
	combinedForce, combinedTorque := b.force.Add(force), b.torque
	if !b.fixedRotation {
		combinedTorque = combinedTorque.Add(worldPoint.Sub(b.Position).Cross(force))
	}
	if !validVec(combinedForce) || !validVec(combinedTorque) {
		return fmt.Errorf("physics3d: accumulated force or torque overflow")
	}
	b.force, b.torque = combinedForce, combinedTorque
	return nil
}

// ApplyTorque accumulates a world-space torque until the next successful Step.
// It has no effect on nondynamic bodies or bodies with fixed rotation.
func (b *Body) ApplyTorque(torque Vec3) error {
	if !validVec(torque) {
		return fmt.Errorf("physics3d: torque must be finite")
	}
	if b.kind != Dynamic || b.fixedRotation {
		return nil
	}
	combined := b.torque.Add(torque)
	if !validVec(combined) {
		return fmt.Errorf("physics3d: accumulated torque overflow")
	}
	b.torque = combined
	return nil
}

// ApplyImpulse changes velocity immediately using a world-space impulse at
// worldPoint. An off-center impulse can also change angular velocity.
func (b *Body) ApplyImpulse(impulse, worldPoint Vec3) error {
	if !validVec(impulse) || !validVec(worldPoint) || !b.validState() {
		return fmt.Errorf("physics3d: impulse, application point, and body state must be finite")
	}
	if b.kind != Dynamic {
		return nil
	}
	velocity := b.Velocity.Add(impulse.Mul(b.invMass))
	angularVelocity := b.AngularVelocity
	if !b.fixedRotation {
		angularVelocity = angularVelocity.Add(b.inverseInertiaWorld(worldPoint.Sub(b.Position).Cross(impulse)))
	}
	if !validVec(velocity) || !validVec(angularVelocity) {
		return fmt.Errorf("physics3d: impulse produces unrepresentable velocity")
	}
	b.Velocity, b.AngularVelocity = velocity, angularVelocity
	return nil
}

// WorldPoint transforms a body-local point to world coordinates.
func (b *Body) WorldPoint(localPoint Vec3) Vec3 { return b.Position.Add(b.Rotation.Rotate(localPoint)) }

// LocalPoint transforms a world point to body-local coordinates.
func (b *Body) LocalPoint(worldPoint Vec3) Vec3 {
	return b.Rotation.Normalized().Conjugate().Rotate(worldPoint.Sub(b.Position))
}

func (b *Body) inverseInertiaWorld(v Vec3) Vec3 {
	if b.kind != Dynamic || b.fixedRotation {
		return Vec3{}
	}
	q := b.Rotation.Normalized()
	local := q.Conjugate().Rotate(v)
	return q.Rotate(Vec3{local.X * b.invInertia.X, local.Y * b.invInertia.Y, local.Z * b.invInertia.Z})
}

func (b *Body) inertiaWorld(v Vec3) Vec3 {
	if b.kind != Dynamic {
		return Vec3{}
	}
	q := b.Rotation.Normalized()
	local := q.Conjugate().Rotate(v)
	return q.Rotate(Vec3{local.X * b.inertia.X, local.Y * b.inertia.Y, local.Z * b.inertia.Z})
}

func (b *Body) applyImpulse(impulse, point Vec3) {
	if b.kind == Dynamic {
		b.Velocity = b.Velocity.Add(impulse.Mul(b.invMass))
		if !b.fixedRotation {
			b.AngularVelocity = b.AngularVelocity.Add(b.inverseInertiaWorld(point.Sub(b.Position).Cross(impulse)))
		}
	}
}

func (b *Body) validState() bool {
	return validVec(b.Position) && validVec(b.Velocity) && validVec(b.AngularVelocity) && validQuat(b.Rotation)
}

package physics3d

import (
	"errors"
	"fmt"
	"math"
)

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

// ErrTOILimit reports that a substep required more CCD impacts than allowed.
var ErrTOILimit = errors.New("physics3d: continuous collision event limit exceeded")

// CreateBody validates a definition and adds its body to the world.
func (w *World) CreateBody(def BodyDef) (*Body, error) {
	if w == nil {
		return nil, fmt.Errorf("physics3d: nil world")
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
		return fmt.Errorf("physics3d: nil world")
	}
	if !finite(dt) || dt <= 0 || w.Substeps < 1 || w.VelocityIterations < 1 || w.PositionIterations < 1 || w.MaxTOIEvents < 1 {
		return fmt.Errorf("physics3d: positive finite dt and positive solver settings required")
	}
	h := dt / float64(w.Substeps)
	if h == 0 || !validVec(w.Gravity) || !finite(w.RestitutionThreshold) || w.RestitutionThreshold < 0 {
		return fmt.Errorf("physics3d: invalid gravity, restitution threshold, or substep duration")
	}
	for _, b := range w.bodies {
		if !b.validState() || !validVec(b.force) || !validVec(b.torque) {
			return fmt.Errorf("physics3d: body state and forces must be finite, with a nonzero rotation")
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
		return fmt.Errorf("physics3d: step failed: %w", cause)
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
	return rotation.Rotate(Vec3{local.X * inverse.X, local.Y * inverse.Y, local.Z * inverse.Z})
}

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
			impact, hit, err := TimeOfImpact(a, b, duration)
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
	result, hit, err := TimeOfImpact(&copyA, &copyB, duration)
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
	min := Vec3{math.Min(b.Position.X, end.X) - radius, math.Min(b.Position.Y, end.Y) - radius, math.Min(b.Position.Z, end.Z) - radius}
	max := Vec3{math.Max(b.Position.X, end.X) + radius, math.Max(b.Position.Y, end.Y) + radius, math.Max(b.Position.Z, end.Z) + radius}
	if !finite(radius) || !validVec(end) || !validVec(min) || !validVec(max) {
		return Vec3{}, Vec3{}, fmt.Errorf("physics3d: continuous motion bounds overflow")
	}
	return min, max, nil
}

func bodyBoundsFinite(b *Body) bool {
	min, max := bodyAABB(b)
	return validVec(min) && validVec(max)
}

func (w *World) collisionDisabled(a, b *Body) bool {
	for _, joint := range w.joints {
		if !joint.collideConnected && ((joint.bodyA == a && joint.bodyB == b) || (joint.bodyA == b && joint.bodyB == a)) {
			return true
		}
	}
	return false
}

func (w *World) contactConstraints() ([]contactConstraint, error) {
	var contacts []contactConstraint
	for i, a := range w.bodies {
		for _, b := range w.bodies[i+1:] {
			if (a.invMass == 0 && b.invMass == 0) || w.collisionDisabled(a, b) || !aabbOverlap(a, b) {
				continue
			}
			contact, hit, err := collideChecked(a, b)
			if err != nil {
				return nil, err
			}
			if !hit {
				continue
			}
			constraint, err := newContactConstraint(a, b, contact, w.RestitutionThreshold)
			if err != nil {
				return nil, err
			}
			contacts = append(contacts, constraint)
		}
	}
	return contacts, nil
}

func relativeVelocity(a, b *Body, rA, rB Vec3) Vec3 {
	return b.Velocity.Add(b.AngularVelocity.Cross(rB)).Sub(a.Velocity.Add(a.AngularVelocity.Cross(rA)))
}

func angularMass(b *Body, radius, axisA, axisB Vec3) float64 {
	if b.kind != Dynamic || b.fixedRotation {
		return 0
	}
	return radius.Cross(axisA).Dot(b.inverseInertiaWorld(radius.Cross(axisB)))
}

func effectiveMass(a, b *Body, rA, rB, axis Vec3) float64 {
	return a.invMass + b.invMass + angularMass(a, rA, axis, axis) + angularMass(b, rB, axis, axis)
}

type contactPoint struct {
	point, rA, rB                                   Vec3
	normalMass, bounce                              float64
	tangentMass11, tangentMass12, tangentMass22     float64
	normalImpulse, tangentImpulse1, tangentImpulse2 float64
}

type contactConstraint struct {
	a, b                       *Body
	normal, tangent1, tangent2 Vec3
	friction                   float64
	points                     []contactPoint
}

func contactTangents(normal Vec3) (Vec3, Vec3) {
	var first Vec3
	if math.Abs(normal.X) >= 0.5773502691896258 {
		first = Vec3{normal.Y, -normal.X, 0}.Normalized()
	} else {
		first = Vec3{0, normal.Z, -normal.Y}.Normalized()
	}
	return first, normal.Cross(first)
}

func newContactConstraint(a, b *Body, contact Contact, threshold float64) (contactConstraint, error) {
	t1, t2 := contactTangents(contact.Normal)
	c := contactConstraint{a: a, b: b, normal: contact.Normal, tangent1: t1, tangent2: t2,
		friction: math.Sqrt(a.friction) * math.Sqrt(b.friction)}
	if !finite(c.friction) {
		return contactConstraint{}, fmt.Errorf("physics3d: contact friction overflow")
	}
	for _, point := range contact.Points {
		rA, rB := point.Sub(a.Position), point.Sub(b.Position)
		kn := effectiveMass(a, b, rA, rB, c.normal)
		k11 := effectiveMass(a, b, rA, rB, t1)
		k22 := effectiveMass(a, b, rA, rB, t2)
		k12 := angularMass(a, rA, t1, t2) + angularMass(b, rB, t1, t2)
		if !validVec(rA) || !validVec(rB) || !finite(kn) || !finite(k11) || !finite(k22) || !finite(k12) || kn <= 0 || k11 <= 0 || k22 <= 0 {
			return contactConstraint{}, fmt.Errorf("physics3d: contact effective mass overflow")
		}
		// Scale before inverting the symmetric 2x2 tangent matrix to avoid an
		// otherwise overflowing determinant for representable inverse masses.
		scale := math.Max(k11, math.Max(k22, math.Abs(k12)))
		x, y, z := k11/scale, k12/scale, k22/scale
		det := x*z - y*y
		if !finite(det) || det <= 0 {
			return contactConstraint{}, fmt.Errorf("physics3d: singular contact tangent mass")
		}
		p := contactPoint{point: point, rA: rA, rB: rB, normalMass: 1 / kn,
			tangentMass11: (z / det) / scale, tangentMass12: (-y / det) / scale, tangentMass22: (x / det) / scale}
		if !finite(p.tangentMass11) || !finite(p.tangentMass12) || !finite(p.tangentMass22) {
			return contactConstraint{}, fmt.Errorf("physics3d: inverse tangent mass overflow")
		}
		speed := relativeVelocity(a, b, rA, rB).Dot(c.normal)
		if !finite(speed) {
			return contactConstraint{}, fmt.Errorf("physics3d: contact speed overflow")
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
		oldNormal := p.normalImpulse
		p.normalImpulse = math.Max(oldNormal+(p.bounce-speed)*p.normalMass, 0)
		impulse := c.normal.Mul(p.normalImpulse - oldNormal)
		c.a.applyImpulse(impulse.Mul(-1), p.point)
		c.b.applyImpulse(impulse, p.point)

		velocity := relativeVelocity(c.a, c.b, p.rA, p.rB)
		v1, v2 := velocity.Dot(c.tangent1), velocity.Dot(c.tangent2)
		old1, old2 := p.tangentImpulse1, p.tangentImpulse2
		new1 := old1 - p.tangentMass11*v1 - p.tangentMass12*v2
		new2 := old2 - p.tangentMass12*v1 - p.tangentMass22*v2
		limit := c.friction * p.normalImpulse
		if magnitude := math.Hypot(new1, new2); magnitude > limit {
			new1, new2 = new1*(limit/magnitude), new2*(limit/magnitude)
		}
		p.tangentImpulse1, p.tangentImpulse2 = new1, new2
		impulse = c.tangent1.Mul(new1 - old1).Add(c.tangent2.Mul(new2 - old2))
		c.a.applyImpulse(impulse.Mul(-1), p.point)
		c.b.applyImpulse(impulse, p.point)
	}
}

func moveBodyPosition(b *Body, impulse, radius Vec3) {
	b.Position = b.Position.Add(impulse.Mul(b.invMass))
	if b.kind == Dynamic && !b.fixedRotation {
		b.Rotation = integrateRotation(b.Rotation, b.inverseInertiaWorld(radius.Cross(impulse)))
	}
}

func correctContactPosition(a, b *Body) error {
	contact, hit, err := collideChecked(a, b)
	if err != nil || !hit {
		return err
	}
	if len(contact.Points) == 0 {
		return fmt.Errorf("physics3d: empty contact manifold")
	}
	correction := math.Min(0.2, 0.4*math.Max(contact.Penetration-0.001, 0)) / float64(len(contact.Points))
	for _, point := range contact.Points {
		rA, rB := point.Sub(a.Position), point.Sub(b.Position)
		k := effectiveMass(a, b, rA, rB, contact.Normal)
		if !finite(k) || k <= 0 {
			return fmt.Errorf("physics3d: position effective mass overflow")
		}
		impulse := contact.Normal.Mul(correction / k)
		moveBodyPosition(a, impulse.Mul(-1), rA)
		moveBodyPosition(b, impulse, rB)
	}
	return nil
}

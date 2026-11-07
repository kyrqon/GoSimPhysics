package dynamics3d

import (
	"fmt"
	"math"
)

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
		first = Vec3{X: normal.Y, Y: -normal.X, Z: 0}.Normalized()
	} else {
		first = Vec3{X: 0, Y: normal.Z, Z: -normal.Y}.Normalized()
	}
	return first, normal.Cross(first)
}

func newContactConstraint(a, b *Body, contact Contact, threshold float64) (contactConstraint, error) {
	t1, t2 := contactTangents(contact.Normal)
	c := contactConstraint{a: a, b: b, normal: contact.Normal, tangent1: t1, tangent2: t2,
		friction: math.Sqrt(a.friction) * math.Sqrt(b.friction)}
	if !finite(c.friction) {
		return contactConstraint{}, fmt.Errorf("dynamics3d: contact friction overflow")
	}
	for _, point := range contact.Points {
		rA, rB := point.Sub(a.Position), point.Sub(b.Position)
		kn := effectiveMass(a, b, rA, rB, c.normal)
		k11 := effectiveMass(a, b, rA, rB, t1)
		k22 := effectiveMass(a, b, rA, rB, t2)
		k12 := angularMass(a, rA, t1, t2) + angularMass(b, rB, t1, t2)
		if !validVec(rA) || !validVec(rB) || !finite(kn) || !finite(k11) || !finite(k22) || !finite(k12) || kn <= 0 || k11 <= 0 || k22 <= 0 {
			return contactConstraint{}, fmt.Errorf("dynamics3d: contact effective mass overflow")
		}
		// Scale before inverting the symmetric 2x2 tangent matrix to avoid an
		// otherwise overflowing determinant for representable inverse masses.
		scale := math.Max(k11, math.Max(k22, math.Abs(k12)))
		x, y, z := k11/scale, k12/scale, k22/scale
		det := x*z - y*y
		if !finite(det) || det <= 0 {
			return contactConstraint{}, fmt.Errorf("dynamics3d: singular contact tangent mass")
		}
		p := contactPoint{point: point, rA: rA, rB: rB, normalMass: 1 / kn,
			tangentMass11: (z / det) / scale, tangentMass12: (-y / det) / scale, tangentMass22: (x / det) / scale}
		if !finite(p.tangentMass11) || !finite(p.tangentMass12) || !finite(p.tangentMass22) {
			return contactConstraint{}, fmt.Errorf("dynamics3d: inverse tangent mass overflow")
		}
		speed := relativeVelocity(a, b, rA, rB).Dot(c.normal)
		if !finite(speed) {
			return contactConstraint{}, fmt.Errorf("dynamics3d: contact speed overflow")
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
		return fmt.Errorf("dynamics3d: empty contact manifold")
	}
	correction := math.Min(0.2, 0.4*math.Max(contact.Penetration-0.001, 0)) / float64(len(contact.Points))
	for _, point := range contact.Points {
		rA, rB := point.Sub(a.Position), point.Sub(b.Position)
		k := effectiveMass(a, b, rA, rB, contact.Normal)
		if !finite(k) || k <= 0 {
			return fmt.Errorf("dynamics3d: position effective mass overflow")
		}
		impulse := contact.Normal.Mul(correction / k)
		moveBodyPosition(a, impulse.Mul(-1), rA)
		moveBodyPosition(b, impulse, rB)
	}
	return nil
}

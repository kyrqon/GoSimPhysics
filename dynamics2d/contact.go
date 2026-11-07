package dynamics2d

import (
	"fmt"
	"math"
)

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
		tangent:  Vec2{X: -contact.Normal.Y, Y: contact.Normal.X},
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

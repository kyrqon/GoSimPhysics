package dynamics2d

import (
	"errors"
	"math"
)

// DistanceJointDef describes a rigid distance constraint between two bodies.
// Anchors are measured in each body's local coordinates. Length is expressed
// in world distance units and must be positive. By default, the connected
// bodies do not collide with one another.
type DistanceJointDef struct {
	BodyA, BodyB               *Body
	LocalAnchorA, LocalAnchorB Vec2
	Length                     float64
	CollideConnected           bool
}

// DistanceJoint keeps two body-local anchors a fixed distance apart. It is a
// bilateral constraint: it can both pull and push the connected bodies.
// The world owns the joint and solves it during Step.
type DistanceJoint struct {
	bodyA, bodyB               *Body
	localAnchorA, localAnchorB Vec2
	length                     float64
	collideConnected           bool

	// Cached velocity-constraint geometry, refreshed every substep.
	normal, rA, rB Vec2
	effectiveMass  float64
	bias           float64
}

// CreateDistanceJoint adds a rigid distance constraint to the world. Both
// endpoints must be distinct bodies in this world, and at least one endpoint
// must be dynamic.
func (w *World) CreateDistanceJoint(def DistanceJointDef) (*DistanceJoint, error) {
	if w == nil || def.BodyA == nil || def.BodyB == nil || def.BodyA == def.BodyB {
		return nil, errors.New("physics: distance joint requires two distinct bodies")
	}
	if def.BodyA.world != w || def.BodyB.world != w {
		return nil, errors.New("physics: distance joint bodies must belong to this world")
	}
	foundA, foundB := false, false
	for _, body := range w.bodies {
		foundA = foundA || body == def.BodyA
		foundB = foundB || body == def.BodyB
	}
	if !foundA || !foundB {
		return nil, errors.New("physics: distance joint bodies must be present in this world")
	}
	if def.BodyA.invMass == 0 && def.BodyB.invMass == 0 {
		return nil, errors.New("physics: distance joint requires a dynamic body")
	}
	if !validVec(def.LocalAnchorA) || !validVec(def.LocalAnchorB) ||
		!finite(def.Length) || def.Length <= 0 {
		return nil, errors.New("physics: distance joint requires finite anchors and positive finite length")
	}
	if !validVec(def.BodyA.WorldPoint(def.LocalAnchorA)) ||
		!validVec(def.BodyB.WorldPoint(def.LocalAnchorB)) {
		return nil, errors.New("physics: distance joint anchor transform overflows")
	}
	joint := &DistanceJoint{
		bodyA: def.BodyA, bodyB: def.BodyB,
		localAnchorA: def.LocalAnchorA, localAnchorB: def.LocalAnchorB,
		length: def.Length, collideConnected: def.CollideConnected,
	}
	if !joint.validGeometry() {
		return nil, errors.New("physics: distance joint geometry overflows")
	}
	w.joints = append(w.joints, joint)
	return joint, nil
}

// RemoveDistanceJoint removes j from this world's simulation. It returns false
// when j is nil, belongs to another world, or has already been removed.
func (w *World) RemoveDistanceJoint(j *DistanceJoint) bool {
	if w == nil || j == nil {
		return false
	}
	for i, joint := range w.joints {
		if joint == j {
			copy(w.joints[i:], w.joints[i+1:])
			w.joints[len(w.joints)-1] = nil
			w.joints = w.joints[:len(w.joints)-1]
			return true
		}
	}
	return false
}

// BodyA returns the first endpoint body.
func (j *DistanceJoint) BodyA() *Body { return j.bodyA }

// BodyB returns the second endpoint body.
func (j *DistanceJoint) BodyB() *Body { return j.bodyB }

// Length returns the constraint's target distance.
func (j *DistanceJoint) Length() float64 { return j.length }

// Anchors returns the current world-space endpoint positions.
func (j *DistanceJoint) Anchors() (Vec2, Vec2) {
	return j.bodyA.WorldPoint(j.localAnchorA), j.bodyB.WorldPoint(j.localAnchorB)
}

// geometry computes the current direction and inverse effective mass. A
// deterministic axis lets coincident anchors separate without dividing by zero.
func (j *DistanceJoint) geometry() (normal, rA, rB Vec2, distance, inverseMass float64) {
	rA = j.localAnchorA.Rotate(j.bodyA.Angle)
	rB = j.localAnchorB.Rotate(j.bodyB.Angle)
	delta := j.bodyB.Position.Add(rB).Sub(j.bodyA.Position.Add(rA))
	distance = delta.Length()
	normal = Vec2{X: 1}
	if distance > 0 {
		normal = delta.Normalized()
	}
	crossA, crossB := rA.Cross(normal), rB.Cross(normal)
	inverseMass = j.bodyA.invMass + j.bodyB.invMass +
		j.bodyA.invInertia*crossA*crossA + j.bodyB.invInertia*crossB*crossB
	return
}

// validGeometry also checks derived quantities: finite body coordinates alone
// do not guarantee that anchor subtraction or effective-mass arithmetic fits.
func (j *DistanceJoint) validGeometry() bool {
	if j == nil || j.bodyA == nil || j.bodyB == nil {
		return false
	}
	anchorA, anchorB := j.Anchors()
	normal, rA, rB, distance, inverseMass := j.geometry()
	return validVec(anchorA) && validVec(anchorB) && validVec(normal) &&
		validVec(rA) && validVec(rB) && finite(distance) &&
		finite(inverseMass) && inverseMass >= 0
}

func (j *DistanceJoint) prepare(dt float64) {
	normal, rA, rB, distance, inverseMass := j.geometry()
	j.normal, j.rA, j.rB = normal, rA, rB
	j.effectiveMass = 0
	if inverseMass > 0 && finite(inverseMass) {
		j.effectiveMass = 1 / inverseMass
	}
	// Cap stabilization error for initially stretched joints. Geometric
	// position correction handles the remaining error without injecting energy.
	j.bias = 0.1 * clamp(distance-j.length, -0.2, 0.2) / dt
}

func (j *DistanceJoint) solveVelocity() {
	velocityA := j.bodyA.Velocity.Add(angularVelocityAt(j.bodyA.AngularVelocity, j.rA))
	velocityB := j.bodyB.Velocity.Add(angularVelocityAt(j.bodyB.AngularVelocity, j.rB))
	impulse := j.normal.Mul(-j.effectiveMass * (velocityB.Sub(velocityA).Dot(j.normal) + j.bias))
	j.bodyA.applyImpulse(impulse.Mul(-1), j.bodyA.Position.Add(j.rA))
	j.bodyB.applyImpulse(impulse, j.bodyB.Position.Add(j.rB))
}

func (j *DistanceJoint) solvePosition() {
	normal, rA, rB, distance, inverseMass := j.geometry()
	if inverseMass <= 0 || !finite(inverseMass) {
		return
	}
	correction := 0.4 * math.Min(0.2, math.Abs(distance-j.length))
	if distance < j.length {
		correction = -correction
	}
	impulse := normal.Mul(-correction / inverseMass)
	j.bodyA.Position = j.bodyA.Position.Sub(impulse.Mul(j.bodyA.invMass))
	if j.bodyA.invInertia != 0 {
		j.bodyA.Angle -= j.bodyA.invInertia * rA.Cross(impulse)
	}
	j.bodyB.Position = j.bodyB.Position.Add(impulse.Mul(j.bodyB.invMass))
	if j.bodyB.invInertia != 0 {
		j.bodyB.Angle += j.bodyB.invInertia * rB.Cross(impulse)
	}
}

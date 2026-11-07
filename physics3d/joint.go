package physics3d

import (
	"fmt"
	"math"
)

// DistanceJointDef describes a rigid bilateral constraint between body-local
// anchors. Length must be positive; connected bodies do not collide by default.
type DistanceJointDef struct {
	BodyA, BodyB               *Body
	LocalAnchorA, LocalAnchorB Vec3
	Length                     float64
	CollideConnected           bool
}

// DistanceJoint keeps two anchors approximately a fixed distance apart. It can
// both pull and push. The world owns and solves its joints during Step.
type DistanceJoint struct {
	bodyA, bodyB               *Body
	localAnchorA, localAnchorB Vec3
	length                     float64
	collideConnected           bool
	normal, rA, rB             Vec3
	effectiveMass, bias        float64
}

// CreateDistanceJoint validates ownership and geometry before adding the joint.
// At least one endpoint must be dynamic; both must be distinct bodies in w.
func (w *World) CreateDistanceJoint(def DistanceJointDef) (*DistanceJoint, error) {
	if w == nil || def.BodyA == nil || def.BodyB == nil || def.BodyA == def.BodyB {
		return nil, fmt.Errorf("physics3d: distance joint requires distinct bodies")
	}
	if def.BodyA.world != w || def.BodyB.world != w {
		return nil, fmt.Errorf("physics3d: joint bodies must belong to this world")
	}
	foundA, foundB := false, false
	for _, b := range w.bodies {
		foundA = foundA || b == def.BodyA
		foundB = foundB || b == def.BodyB
	}
	if !foundA || !foundB || (def.BodyA.invMass == 0 && def.BodyB.invMass == 0) {
		return nil, fmt.Errorf("physics3d: joint requires world members and a dynamic endpoint")
	}
	if !validVec(def.LocalAnchorA) || !validVec(def.LocalAnchorB) || !finite(def.Length) || def.Length <= 0 {
		return nil, fmt.Errorf("physics3d: finite anchors and positive finite length required")
	}
	j := &DistanceJoint{bodyA: def.BodyA, bodyB: def.BodyB,
		localAnchorA: def.LocalAnchorA, localAnchorB: def.LocalAnchorB,
		length: def.Length, collideConnected: def.CollideConnected}
	if !j.validGeometry() {
		return nil, fmt.Errorf("physics3d: joint geometry overflow")
	}
	w.joints = append(w.joints, j)
	return j, nil
}

// RemoveDistanceJoint removes a joint owned by this world, succeeding once.
func (w *World) RemoveDistanceJoint(j *DistanceJoint) bool {
	if w == nil || j == nil {
		return false
	}
	for i, candidate := range w.joints {
		if candidate == j {
			copy(w.joints[i:], w.joints[i+1:])
			w.joints[len(w.joints)-1] = nil
			w.joints = w.joints[:len(w.joints)-1]
			return true
		}
	}
	return false
}

// BodyA returns the first endpoint.
func (j *DistanceJoint) BodyA() *Body { return j.bodyA }

// BodyB returns the second endpoint.
func (j *DistanceJoint) BodyB() *Body { return j.bodyB }

// Length returns the target anchor distance.
func (j *DistanceJoint) Length() float64 { return j.length }

// Anchors returns the current world-space anchor positions.
func (j *DistanceJoint) Anchors() (Vec3, Vec3) {
	return j.bodyA.WorldPoint(j.localAnchorA), j.bodyB.WorldPoint(j.localAnchorB)
}

func (j *DistanceJoint) geometry() (normal, rA, rB Vec3, distance, inverseMass float64) {
	rA = j.bodyA.Rotation.Rotate(j.localAnchorA)
	rB = j.bodyB.Rotation.Rotate(j.localAnchorB)
	delta := j.bodyB.Position.Add(rB).Sub(j.bodyA.Position.Add(rA))
	distance = delta.Length()
	normal = Vec3{X: 1}
	if distance > 0 {
		normal = delta.Normalized()
	}
	inverseMass = effectiveMass(j.bodyA, j.bodyB, rA, rB, normal)
	return
}

func (j *DistanceJoint) validGeometry() bool {
	if j == nil || j.bodyA == nil || j.bodyB == nil {
		return false
	}
	a, b := j.Anchors()
	normal, rA, rB, distance, inverseMass := j.geometry()
	return validVec(a) && validVec(b) && validVec(normal) && validVec(rA) && validVec(rB) &&
		finite(distance) && finite(inverseMass) && inverseMass > 0
}

func (j *DistanceJoint) prepare(dt float64) {
	normal, rA, rB, distance, inverseMass := j.geometry()
	j.normal, j.rA, j.rB = normal, rA, rB
	j.effectiveMass = 1 / inverseMass
	j.bias = 0.1 * clamp(distance-j.length, -0.2, 0.2) / dt
}

func (j *DistanceJoint) solveVelocity() {
	speed := relativeVelocity(j.bodyA, j.bodyB, j.rA, j.rB).Dot(j.normal)
	impulse := j.normal.Mul(-(speed + j.bias) * j.effectiveMass)
	j.bodyA.applyImpulse(impulse.Mul(-1), j.bodyA.Position.Add(j.rA))
	j.bodyB.applyImpulse(impulse, j.bodyB.Position.Add(j.rB))
}

func (j *DistanceJoint) solvePosition() {
	normal, rA, rB, distance, inverseMass := j.geometry()
	correction := 0.4 * math.Min(0.2, math.Abs(distance-j.length))
	if distance < j.length {
		correction = -correction
	}
	impulse := normal.Mul(-correction / inverseMass)
	moveBodyPosition(j.bodyA, impulse.Mul(-1), rA)
	moveBodyPosition(j.bodyB, impulse, rB)
}

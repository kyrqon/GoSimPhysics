package physics2d

import (
	"math"
	"testing"
)

func TestWorldLongStaticGroundHasNoRotationalMassOverflow(t *testing.T) {
	w := NewWorld(Vec2{})
	worldTestBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec2{X: 1e200, Y: 1}}})
	ball := worldTestBody(t, w, BodyDef{
		Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 1,
		Position: Vec2{X: 5e199, Y: 2}, Velocity: Vec2{Y: -1},
	})
	worldTestStep(t, w, 1.0/60)
	worldTestClose(t, ball.Velocity.Y, 0, 1e-12)
	worldTestClose(t, ball.Position.Y, 2, 0.01)
}

func TestWorldPairGeometryOverflowRollsBackAndPreservesForce(t *testing.T) {
	w := NewWorld(Vec2{})
	a := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1e308}, Mass: 1e-308})
	b := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1e308}, Mass: 1e-308})
	if err := a.ApplyForce(Vec2{X: 1e-308}); err != nil {
		t.Fatal(err)
	}
	// The single-body bounds and inertia are representable, but the sum of
	// radii is not. A numerical failure must not silently become a missed hit.
	if err := w.Step(0.01); err == nil {
		t.Fatal("accepted overflowing pair geometry")
	}
	if a.Position != (Vec2{}) || a.Velocity != (Vec2{}) || b.Position != (Vec2{}) || a.force.X != 1e-308 {
		t.Fatal("failed collision step changed state or consumed force")
	}
	w.RemoveBody(b)
	worldTestStep(t, w, 0.01)
	worldTestClose(t, a.Velocity.X, 0.01, 1e-12)
}

func TestWorldContactMassOverflowRollsBack(t *testing.T) {
	w := NewWorld(Vec2{})
	a := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 2}, Mass: 1e-308})
	b := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 2}, Mass: 1e-308, Position: Vec2{X: 3}})
	if err := w.Step(0.01); err == nil {
		t.Fatal("accepted overflowing combined inverse mass")
	}
	if a.Position != (Vec2{}) || a.Velocity != (Vec2{}) || b.Position != (Vec2{X: 3}) || b.Velocity != (Vec2{}) {
		t.Fatal("failed contact step changed body state")
	}
}

func TestWorldMaterialMixing(t *testing.T) {
	t.Run("larger restitution", func(t *testing.T) {
		w := NewWorld(Vec2{})
		w.Substeps, w.RestitutionThreshold = 1, 0
		worldTestBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec2{X: 10, Y: 0.5}}, Position: Vec2{Y: -0.5}, Restitution: 0.8})
		ball := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: 1, Position: Vec2{Y: 0.5}, Velocity: Vec2{Y: -1}, Restitution: 0.2})
		worldTestStep(t, w, 0.001)
		worldTestClose(t, ball.Velocity.Y, 0.8, 1e-12)
	})
	t.Run("geometric mean friction", func(t *testing.T) {
		w := NewWorld(Vec2{Y: -10})
		w.Substeps = 1
		worldTestBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec2{X: 10, Y: 0.5}}, Position: Vec2{Y: -0.5}, Friction: 0.25})
		ball := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.5}, Mass: 1, Position: Vec2{Y: 0.5}, Velocity: Vec2{X: 1}, Friction: 1, FixedRotation: true})
		worldTestStep(t, w, 0.01)
		// Normal impulse balances gravity: Jn=m*g*dt=0.1. The sliding friction
		// impulse is sqrt(0.25*1)*Jn=0.05, with rotation locked.
		worldTestClose(t, ball.Velocity.X, 0.95, 1e-12)
	})
}

func TestWorldEnforcesStaticAndFixedRotationAfterPublicStateEdits(t *testing.T) {
	w := NewWorld(Vec2{})
	wall := worldTestBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec2{X: 1, Y: 1}}, Position: Vec2{X: -10}})
	ball := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 1, FixedRotation: true})
	wall.Velocity, wall.AngularVelocity = Vec2{X: 10, Y: 10}, 5
	ball.AngularVelocity = 7
	worldTestStep(t, w, 0.1)
	if wall.Position != (Vec2{X: -10}) || wall.Velocity != (Vec2{}) || wall.AngularVelocity != 0 || wall.Angle != 0 {
		t.Fatal("public velocity edits moved a static body")
	}
	if ball.Angle != 0 || ball.AngularVelocity != 0 {
		t.Fatal("public angular velocity edit defeated fixed rotation")
	}
}

func TestWorldRejectsOverflowingBoundsBeforeMutation(t *testing.T) {
	w := NewWorld(Vec2{})
	ball := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1}, Mass: 1, Velocity: Vec2{X: 1}})
	worldTestBody(t, w, BodyDef{Shape: Box{HalfExtents: Vec2{X: 1.5e308, Y: 1.5e308}}, Angle: math.Pi / 4})
	if err := w.Step(0.1); err == nil {
		t.Fatal("accepted overflowing rotated bounds")
	}
	if ball.Position != (Vec2{}) || ball.Velocity != (Vec2{X: 1}) {
		t.Fatal("invalid later body partially advanced an earlier body")
	}
}

func TestWorldRemoveBodyRemovesOnlyAttachedJoints(t *testing.T) {
	w := NewWorld(Vec2{})
	a := worldTestBody(t, w, BodyDef{Shape: Circle{Radius: 0.1}})
	b := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.1}, Mass: 1, Position: Vec2{X: 2}})
	c := worldTestBody(t, w, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.1}, Mass: 1, Position: Vec2{X: 4}})
	ab, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: a, BodyB: b, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	bc, err := w.CreateDistanceJoint(DistanceJointDef{BodyA: b, BodyB: c, Length: 2})
	if err != nil {
		t.Fatal(err)
	}
	if !w.RemoveBody(a) || w.RemoveDistanceJoint(ab) {
		t.Fatal("removing an endpoint did not remove its joint")
	}
	b.Velocity.X, c.Velocity.X = 1, 1
	worldTestStep(t, w, 0.1)
	worldTestClose(t, b.Position.X, 2.1, 1e-12)
	worldTestClose(t, c.Position.X, 4.1, 1e-12)
	if !w.RemoveDistanceJoint(bc) {
		t.Fatal("removing one body also removed an unrelated joint")
	}
}

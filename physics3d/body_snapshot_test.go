package physics3d

import (
	"math"
	"testing"
)

func TestBodySnapshotsAreIndependentAndRespectMotionPolicy(t *testing.T) {
	body := bodyQueryTestBody(t, BodyDef{Type: Dynamic, Shape: Sphere{Radius: 1}, Mass: 1, Position: Vec3{X: -3}, Velocity: Vec3{X: 2}, AngularVelocity: Vec3{Z: 3}})
	wall := bodyQueryTestBody(t, BodyDef{Shape: Sphere{Radius: 1}})
	before := *body
	collider := body.Collider()
	collider.Position.X = 100
	collider.Shape = Box{HalfExtents: Vec3{X: 2, Y: 2, Z: 2}}
	motion, err := body.Motion()
	if err != nil {
		t.Fatal(err)
	}
	motion.Position.X = 200
	motion.Velocity.X = -20
	if *body != before {
		t.Fatal("editing a collider or motion snapshot changed the body")
	}
	if _, hit, err := body.TimeOfImpact(wall, 1); err != nil || !hit {
		t.Fatalf("body sweep = %v, %v", hit, err)
	}
	if _, hit := body.Collide(wall); hit {
		t.Fatal("separated bodies collided")
	}
	body.Position.X = -2
	if _, hit := body.Collide(wall); !hit {
		t.Fatal("touching bodies did not collide")
	}
	for _, def := range []BodyDef{
		{Shape: Sphere{Radius: 1}},
		{Type: Dynamic, Mass: 1, Shape: Sphere{Radius: 1}, FixedRotation: true},
	} {
		b := bodyQueryTestBody(t, def)
		b.Velocity, b.AngularVelocity = Vec3{X: 7}, Vec3{Y: 9}
		m, err := b.Motion()
		if err != nil || m.AngularVelocity != (Vec3{}) {
			t.Fatalf("disabled angular motion = %+v, %v", m, err)
		}
		if b.kind == Static && m.Velocity != (Vec3{}) {
			t.Fatal("static snapshot retained a public velocity edit")
		}
		if b.kind == Dynamic && m.Velocity != b.Velocity {
			t.Fatal("fixed rotation suppressed linear velocity")
		}
	}
}

func TestBodySnapshotAndQueryValidation(t *testing.T) {
	var absent *Body
	if absent.Collider() != (Collider{}) {
		t.Fatal("nil body collider was not zero")
	}
	if _, err := absent.Motion(); err == nil {
		t.Fatal("nil body motion succeeded")
	}
	valid := bodyQueryTestBody(t, BodyDef{Shape: Sphere{Radius: 1}})
	if _, hit := absent.Collide(valid); hit {
		t.Fatal("nil body collided")
	}
	if _, _, err := absent.TimeOfImpact(valid, 1); err == nil {
		t.Fatal("nil body sweep succeeded")
	}
	for name, mutate := range map[string]func(*Body){
		"nonfinite position": func(b *Body) { b.Position.X = math.Inf(1) },
		"nonfinite velocity": func(b *Body) { b.Velocity.Y = math.NaN() },
		"zero rotation":      func(b *Body) { b.Rotation = Quaternion{} },
		"invalid shape":      func(b *Body) { b.shape = Sphere{Radius: -1} },
		"overflowing bounds": func(b *Body) { b.Position.X = math.MaxFloat64; b.shape = Sphere{Radius: math.MaxFloat64} },
	} {
		t.Run(name, func(t *testing.T) {
			b := *valid
			mutate(&b)
			if _, err := b.Motion(); err == nil {
				t.Fatal("invalid motion snapshot succeeded")
			}
		})
	}
}

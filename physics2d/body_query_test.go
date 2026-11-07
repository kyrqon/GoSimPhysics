package physics2d

import (
	"math"
	"reflect"
	"testing"
)

func toiTestBody(t *testing.T, def BodyDef) *Body {
	t.Helper()
	if def.Type == Dynamic && def.Mass == 0 {
		def.Mass = 1
	}
	body, err := newBody(def)
	if err != nil {
		t.Fatal(err)
	}
	return body
}
func TestTimeOfImpactInitialTouchAndMotionTypes(t *testing.T) {
	a := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1}, Position: Vec2{X: -2}, Velocity: Vec2{X: -3}})
	b := toiTestBody(t, BodyDef{Shape: Circle{Radius: 1}})
	result, hit, err := a.TimeOfImpact(b, 1)
	if err != nil || !hit || result.Time != 0 {
		t.Fatalf("initial touching must be observable even when separating: %+v %v %v", result, hit, err)
	}
	a.Position.X = -3
	// Static public velocity edits do not turn a fixed shape into a moving target.
	b.Velocity, b.AngularVelocity = Vec2{X: -100}, 100
	if _, hit, err := a.TimeOfImpact(b, 1); err != nil || hit {
		t.Fatalf("static velocity was used: %v, %v", hit, err)
	}
	rod := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec2{X: 2, Y: 0.02}}, AngularVelocity: math.Pi / 2, FixedRotation: true})
	rod.AngularVelocity = math.Pi / 2
	stationary := toiTestBody(t, BodyDef{Shape: Circle{Radius: 0.05}, Position: Vec2{X: 1.4, Y: 1.4}})
	if _, hit, err := rod.TimeOfImpact(stationary, 1); err != nil || hit {
		t.Fatalf("FixedRotation was ignored: %v, %v", hit, err)
	}
}

func TestBodyQuerySnapshotsAreIndependent(t *testing.T) {
	bullet := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Circle{Radius: 0.05}, Position: Vec2{X: -2}, Velocity: Vec2{X: 400}})
	wall := toiTestBody(t, BodyDef{Shape: Box{HalfExtents: Vec2{X: 0.01, Y: 1}}})
	beforeBullet, beforeWall := *bullet, *wall
	pose := bullet.Collider()
	pose.Position.X = 100
	pose.Shape = Circle{Radius: 50}
	if !reflect.DeepEqual(*bullet, beforeBullet) {
		t.Fatal("editing a collider snapshot changed its body")
	}
	moving, err := bullet.Motion()
	if err != nil {
		t.Fatal(err)
	}
	fixed, err := wall.Motion()
	if err != nil {
		t.Fatal(err)
	}
	fromSnapshots, hit, err := TimeOfImpact(moving, fixed, 0.01)
	fromBodies, bodyHit, bodyErr := bullet.TimeOfImpact(wall, 0.01)
	if err != nil || bodyErr != nil || !hit || !bodyHit || !reflect.DeepEqual(fromSnapshots, fromBodies) {
		t.Fatalf("body and snapshot sweeps disagree: %+v %+v; %v %v", fromSnapshots, fromBodies, err, bodyErr)
	}
	moving.Position.X = 100
	moving.Velocity.X = -1
	if !reflect.DeepEqual(*bullet, beforeBullet) || !reflect.DeepEqual(*wall, beforeWall) {
		t.Fatal("queries or motion snapshot edits changed the bodies")
	}
	bullet.Position.X = -0.06
	fromBody, bodyHit := bullet.Collide(wall)
	fromGeometry, geometryHit := Collide(bullet.Collider(), wall.Collider())
	if !bodyHit || !geometryHit || !reflect.DeepEqual(fromBody, fromGeometry) {
		t.Fatalf("body and collider contacts disagree: %+v %+v", fromBody, fromGeometry)
	}
}

func TestBodyMotionSnapshotHonorsMotionSettings(t *testing.T) {
	static := toiTestBody(t, BodyDef{Shape: Circle{Radius: 1}})
	static.Velocity, static.AngularVelocity = Vec2{X: 50}, 3
	m, err := static.Motion()
	if err != nil || m.Velocity != (Vec2{}) || m.AngularVelocity != 0 {
		t.Fatalf("static snapshot must ignore edited public velocities: %+v, %v", m, err)
	}
	body := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Circle{Radius: 1}, Velocity: Vec2{X: 2}, FixedRotation: true})
	body.AngularVelocity = 3
	m, err = body.Motion()
	if err != nil || m.Velocity != body.Velocity || m.AngularVelocity != 0 {
		t.Fatalf("fixed-rotation snapshot lost linear motion or retained spin: %+v, %v", m, err)
	}
}

func TestBodyQueryNilAndInvalidSnapshots(t *testing.T) {
	var missing *Body
	valid := toiTestBody(t, BodyDef{Shape: Circle{Radius: 1}})
	if missing.Collider() != (Collider{}) {
		t.Fatal("nil collider snapshot must be zero")
	}
	if _, hit := missing.Collide(valid); hit {
		t.Fatal("nil receiver returned contact")
	}
	if _, hit := valid.Collide(missing); hit {
		t.Fatal("nil other body returned contact")
	}
	for _, bodies := range [][2]*Body{{missing, valid}, {valid, missing}} {
		if _, hit, err := bodies[0].TimeOfImpact(bodies[1], 1); err == nil || hit {
			t.Fatal("nil sweep input was accepted")
		}
	}
	if _, err := missing.Motion(); err == nil {
		t.Fatal("nil body returned a valid motion")
	}
	for name, edit := range map[string]func(*Body){
		"position":         func(b *Body) { b.Position.X = math.NaN() },
		"velocity":         func(b *Body) { b.Velocity.X = math.Inf(1) },
		"angle":            func(b *Body) { b.Angle = math.Inf(1) },
		"angular velocity": func(b *Body) { b.AngularVelocity = math.NaN() },
		"motion type":      func(b *Body) { b.kind = BodyType(255) },
		"shape":            func(b *Body) { b.shape = Circle{Radius: -1} },
		"bounds": func(b *Body) {
			b.Position.X = math.MaxFloat64
			b.shape = Circle{Radius: math.MaxFloat64}
		},
	} {
		t.Run(name, func(t *testing.T) {
			b := *valid
			edit(&b)
			if _, err := b.Motion(); err == nil {
				t.Fatal("invalid body returned a valid motion")
			}
		})
	}
}

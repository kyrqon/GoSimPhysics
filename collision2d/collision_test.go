package collision2d

import (
	"math"
	"math/rand"
	"testing"
)

func collisionCircle(position Vec2, radius float64) *proxy {
	return &proxy{Position: position, shape: Circle{Radius: radius}}
}

func collisionBox(position, half Vec2, angle float64) *proxy {
	return &proxy{Position: position, Angle: angle, shape: Box{HalfExtents: half}}
}

func collisionClose(t *testing.T, got, want, tolerance float64) {
	t.Helper()
	if !finite(got) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.15g, want %.15g (tolerance %g)", got, want, tolerance)
	}
}

func collisionCloseVec(t *testing.T, got, want Vec2, tolerance float64) {
	t.Helper()
	collisionClose(t, got.X, want.X, tolerance)
	collisionClose(t, got.Y, want.Y, tolerance)
}

func TestCollideCircles(t *testing.T) {
	tests := []struct {
		name      string
		position  Vec2
		collision bool
		depth     float64
		normal    Vec2
		point     Vec2
	}{
		{"separated", Vec2{X: 3, Y: 0}, false, 0, Vec2{}, Vec2{}},
		{"touching", Vec2{X: 2, Y: 0}, true, 0, Vec2{X: 1, Y: 0}, Vec2{X: 1, Y: 0}},
		{"overlap", Vec2{X: 1.5, Y: 0}, true, 0.5, Vec2{X: 1, Y: 0}, Vec2{X: 0.75, Y: 0}},
		{"vertical", Vec2{X: 0, Y: -1.5}, true, 0.5, Vec2{X: 0, Y: -1}, Vec2{X: 0, Y: -0.75}},
		{"coincident", Vec2{}, true, 2, Vec2{X: 1, Y: 0}, Vec2{}},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			contact, ok := collideProxies(collisionCircle(Vec2{}, 1), collisionCircle(tt.position, 1))
			if ok != tt.collision {
				t.Fatalf("collision = %v, want %v", ok, tt.collision)
			}
			if !ok {
				return
			}
			collisionClose(t, contact.Penetration, tt.depth, 1e-12)
			collisionCloseVec(t, contact.Normal, tt.normal, 1e-12)
			if len(contact.Points) != 1 {
				t.Fatalf("point count %d, want 1", len(contact.Points))
			}
			collisionCloseVec(t, contact.Points[0], tt.point, 1e-12)
		})
	}
}

func TestCollideCircleBox(t *testing.T) {
	box := collisionBox(Vec2{}, Vec2{X: 1, Y: 1}, 0)
	tests := []struct {
		name      string
		position  Vec2
		radius    float64
		collision bool
		depth     float64
		normal    Vec2
	}{
		{"separated face", Vec2{X: 2, Y: 0}, 0.5, false, 0, Vec2{}},
		{"touching face", Vec2{X: 1.5, Y: 0}, 0.5, true, 0, Vec2{X: -1, Y: 0}},
		{"overlap face", Vec2{X: 1.25, Y: 0}, 0.5, true, 0.25, Vec2{X: -1, Y: 0}},
		{"separated corner", Vec2{X: 1.4, Y: 1.4}, 0.5, false, 0, Vec2{}},
		{"overlap corner", Vec2{X: 1.3, Y: 1.4}, 0.6, true, 0.1, Vec2{X: -0.6, Y: -0.8}},
		{"center inside", Vec2{X: 0.7, Y: 0}, 0.25, true, 0.55, Vec2{X: -1, Y: 0}},
		{"center inside negative", Vec2{X: 0, Y: -0.8}, 0.25, true, 0.45, Vec2{X: 0, Y: 1}},
		{"center on face", Vec2{X: 1, Y: 0}, 0.25, true, 0.25, Vec2{X: -1, Y: 0}},
		{"coincident centers", Vec2{}, 0.25, true, 1.25, Vec2{X: -1, Y: 0}},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			circle := collisionCircle(tt.position, tt.radius)
			contact, ok := collideProxies(circle, box)
			if ok != tt.collision {
				t.Fatalf("collision = %v, want %v", ok, tt.collision)
			}
			if !ok {
				return
			}
			collisionClose(t, contact.Penetration, tt.depth, 1e-12)
			collisionCloseVec(t, contact.Normal, tt.normal, 1e-12)
			if len(contact.Points) != 1 || !validVec(contact.Points[0]) {
				t.Fatalf("invalid points: %v", contact.Points)
			}
			swapped, reverseOK := collideProxies(box, circle)
			if !reverseOK {
				t.Fatal("reversed collision disappeared")
			}
			collisionCloseVec(t, swapped.Normal, contact.Normal.Mul(-1), 1e-12)
			collisionCloseVec(t, swapped.Points[0], contact.Points[0], 1e-12)
			collisionClose(t, swapped.Penetration, contact.Penetration, 1e-12)
		})
	}
}

func TestCollideBoxesFaceAndCornerManifolds(t *testing.T) {
	a := collisionBox(Vec2{}, Vec2{X: 1, Y: 1}, 0)
	b := collisionBox(Vec2{X: 1.5, Y: 0}, Vec2{X: 1, Y: 1}, 0)
	contact, ok := collideProxies(a, b)
	if !ok || len(contact.Points) != 2 {
		t.Fatalf("expected two-point face contact: %+v %v", contact, ok)
	}
	collisionClose(t, contact.Penetration, 0.5, 1e-12)
	collisionCloseVec(t, contact.Normal, Vec2{X: 1, Y: 0}, 1e-12)
	for _, point := range contact.Points {
		collisionClose(t, point.X, 0.75, 1e-12)
		collisionClose(t, math.Abs(point.Y), 1, 1e-12)
	}
	b.Position = Vec2{X: 2, Y: 2}
	contact, ok = collideProxies(a, b)
	if !ok || len(contact.Points) != 1 {
		t.Fatalf("expected single corner contact: %+v %v", contact, ok)
	}
	collisionClose(t, contact.Penetration, 0, 1e-12)
	collisionCloseVec(t, contact.Points[0], Vec2{X: 1, Y: 1}, 1e-12)
	b.Position = Vec2{X: 2.01, Y: 0}
	if _, ok := collideProxies(a, b); ok {
		t.Fatal("separated boxes collided")
	}
}

func TestCollideBoxesRotatedAndContained(t *testing.T) {
	a := collisionBox(Vec2{}, Vec2{X: 1, Y: 1}, 0)
	b := collisionBox(Vec2{X: 2, Y: 0}, Vec2{X: 1, Y: 1}, math.Pi/4)
	contact, ok := collideProxies(a, b)
	if !ok {
		t.Fatal("rotated boxes should collide")
	}
	collisionClose(t, contact.Penetration, math.Sqrt2-1, 1e-12)
	collisionCloseVec(t, contact.Normal, Vec2{X: 1, Y: 0}, 1e-12)
	if len(contact.Points) != 1 {
		t.Fatalf("rotated vertex contact points: %v", contact.Points)
	}
	// Their AABBs overlap, but the separating axis of the rotated box excludes
	// the corner of the stationary box.
	b.Position = Vec2{X: 2, Y: 2}
	if !aabbOverlap(a, b) {
		t.Fatal("fixture must overlap in broad phase")
	}
	if _, ok := collideProxies(a, b); ok {
		t.Fatal("SAT failed to reject an AABB false positive")
	}
	a = collisionBox(Vec2{}, Vec2{X: 5, Y: 5}, 0)
	b = collisionBox(Vec2{X: 1.5, Y: 0}, Vec2{X: 1, Y: 1}, 0)
	contact, ok = collideProxies(a, b)
	if !ok {
		t.Fatal("contained boxes should collide")
	}
	// Interval intersection length is only 2; the true separating translation
	// is 4.5, because the small box must travel past the large box's right face.
	collisionClose(t, contact.Penetration, 4.5, 1e-12)
	collisionCloseVec(t, contact.Normal, Vec2{X: 1, Y: 0}, 1e-12)
	b.Position, b.Angle = Vec2{X: 0, Y: 5.999999}, 1e-12
	contact, ok = collideProxies(a, b)
	if !ok || len(contact.Points) != 2 {
		t.Fatalf("near-parallel face contact failed: %+v %v", contact, ok)
	}
}

func TestCollisionCovarianceAndSwap(t *testing.T) {
	rng := rand.New(rand.NewSource(42))
	for i := 0; i < 300; i++ {
		a := collisionBox(Vec2{X: -0.2, Y: 0.1}, Vec2{X: 1.2, Y: 0.7}, rng.Float64()*2-1)
		var b *proxy
		if i%2 == 0 {
			b = collisionCircle(Vec2{X: 1, Y: 0.3}, 0.6)
		} else {
			b = collisionBox(Vec2{X: 0.9, Y: 0.3}, Vec2{X: 0.6, Y: 0.9}, rng.Float64()*2-1)
		}
		base, ok := collideProxies(a, b)
		if !ok {
			t.Fatalf("overlapping fixture %d lost contact", i)
		}
		collisionClose(t, base.Normal.Length(), 1, 1e-12)
		reverse, ok := collideProxies(b, a)
		if !ok {
			t.Fatalf("swapped fixture %d lost contact", i)
		}
		collisionClose(t, reverse.Penetration, base.Penetration, 1e-10)
		collisionCloseVec(t, reverse.Normal, base.Normal.Mul(-1), 1e-10)
		rotation := rng.Float64()*6 - 3
		translation := Vec2{X: rng.Float64()*20 - 10, Y: rng.Float64()*20 - 10}
		a.Position, b.Position = a.Position.Rotate(rotation).Add(translation), b.Position.Rotate(rotation).Add(translation)
		a.Angle, b.Angle = a.Angle+rotation, b.Angle+rotation
		moved, ok := collideProxies(a, b)
		if !ok {
			t.Fatalf("transformed fixture %d lost contact", i)
		}
		collisionClose(t, moved.Penetration, base.Penetration, 1e-10)
		collisionCloseVec(t, moved.Normal, base.Normal.Rotate(rotation), 1e-10)
		if len(moved.Points) != len(base.Points) {
			t.Fatalf("transformed manifold point count changed")
		}
		for _, expected := range base.Points {
			expected = expected.Rotate(rotation).Add(translation)
			found := false
			for _, point := range moved.Points {
				if point.Sub(expected).Length() < 1e-9 {
					found = true
				}
			}
			if !found {
				t.Fatalf("transformed contact point missing: want %v in %v", expected, moved.Points)
			}
		}
	}
}

func TestCollisionGeometryValidationAndAABB(t *testing.T) {
	valid := collisionCircle(Vec2{}, 1)
	var nilCircle *Circle
	for _, invalid := range []*proxy{nil, {}, {shape: nilCircle}, collisionCircle(Vec2{}, -1), collisionCircle(Vec2{X: math.NaN(), Y: 0}, 1), collisionBox(Vec2{}, Vec2{X: 1, Y: 1}, math.Inf(1))} {
		if _, ok := collideProxies(invalid, valid); ok {
			t.Fatalf("invalid body produced collision: %+v", invalid)
		}
	}
	box := collisionBox(Vec2{X: 3, Y: 4}, Vec2{X: 2, Y: 1}, math.Pi/2)
	min, max := bodyAABB(box)
	collisionCloseVec(t, min, Vec2{X: 2, Y: 2}, 1e-12)
	collisionCloseVec(t, max, Vec2{X: 4, Y: 6}, 1e-12)
	circle := collisionCircle(Vec2{X: 5, Y: 4}, 1)
	if !aabbOverlap(box, circle) {
		t.Fatal("AABB touching must be inclusive")
	}
	circle.Position.X += 0.01
	if aabbOverlap(box, circle) {
		t.Fatal("AABB separated bodies overlap")
	}
}

func FuzzCollisionInvariants(f *testing.F) {
	f.Add(0.0, 0.0, 0.3, 1.2, 0.4, -0.2)
	f.Add(-1.0, 2.0, 1.1, 1.5, 2.2, 0.7)
	f.Add(0.0, 0.0, 0.0, 2.0, 2.0, 0.0)
	f.Fuzz(func(t *testing.T, ax, ay, angleA, bx, by, angleB float64) {
		for _, value := range []float64{ax, ay, angleA, bx, by, angleB} {
			if !finite(value) || math.Abs(value) > 1000 {
				t.Skip()
			}
		}
		a := collisionBox(Vec2{X: ax, Y: ay}, Vec2{X: 1.3, Y: 0.7}, angleA)
		b := collisionBox(Vec2{X: bx, Y: by}, Vec2{X: 0.8, Y: 1.1}, angleB)
		ab, hitAB := collideProxies(a, b)
		ba, hitBA := collideProxies(b, a)
		if hitAB != hitBA {
			t.Fatalf("collision must be symmetric: A=%+v B=%+v", a, b)
		}
		if !hitAB {
			return
		}
		if !finite(ab.Penetration) || ab.Penetration < 0 || len(ab.Points) < 1 || len(ab.Points) > 2 {
			t.Fatalf("invalid contact: %+v", ab)
		}
		collisionClose(t, ab.Normal.Length(), 1, 1e-12)
		collisionClose(t, ab.Penetration, ba.Penetration, 1e-9)
		for _, point := range ab.Points {
			if !validVec(point) {
				t.Fatalf("non-finite contact point: %+v", ab)
			}
		}
		if !aabbOverlap(a, b) {
			t.Fatal("narrow-phase hit must overlap in broad phase")
		}
	})
}

func TestCollisionPairGeometryOverflow(t *testing.T) {
	tests := []struct {
		name string
		a, b *proxy
	}{
		{"circle radius sum", collisionCircle(Vec2{}, 1e308), collisionCircle(Vec2{}, 1e308)},
		{"circle-box extent sum", collisionCircle(Vec2{}, 1e308), collisionBox(Vec2{}, Vec2{X: 1e308, Y: 1}, 0)},
		{"box projection sum", collisionBox(Vec2{}, Vec2{X: 1e308, Y: 1}, 0), collisionBox(Vec2{}, Vec2{X: 1e308, Y: 1}, 0)},
		{"center difference", collisionCircle(Vec2{X: -1e308, Y: 0}, 1), collisionCircle(Vec2{X: 1e308, Y: 0}, 1)},
		{"rotated circle center", collisionCircle(Vec2{X: 1.4e308, Y: 1.4e308}, 1), collisionBox(Vec2{}, Vec2{X: 1, Y: 1}, math.Pi/4)},
		{"box center projection", collisionBox(Vec2{}, Vec2{X: 1, Y: 1}, math.Pi/4), collisionBox(Vec2{X: 1.4e308, Y: 1.4e308}, Vec2{X: 1, Y: 1}, 0)},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if collisionGeometryFinite(tt.a, tt.b) || collisionGeometryFinite(tt.b, tt.a) {
				t.Fatal("unrepresentable pair geometry accepted")
			}
			if contact, hit := collideProxies(tt.a, tt.b); hit {
				t.Fatalf("unrepresentable collision returned contact: %+v", contact)
			}
		})
	}
	// These bodies have representable individual bounds;
	// only their combined radii overflow. This must not be an ordinary miss.
	a, b := collisionCircle(Vec2{}, 1e308), collisionCircle(Vec2{}, 1e308)
	for _, body := range []*proxy{a, b} {
		min, max := bodyAABB(body)
		if !validVec(min) || !validVec(max) {
			t.Fatal("test requires finite individual bounds")
		}
	}
	if !aabbOverlap(a, b) || collisionGeometryFinite(a, b) {
		t.Fatal("combined radius overflow was not detected")
	}
}

func TestCollisionFiniteLongFloor(t *testing.T) {
	floor := collisionBox(Vec2{}, Vec2{X: 1e200, Y: 1}, 0)
	ball := collisionCircle(Vec2{X: 5e199, Y: 1.5}, 1)
	if !aabbOverlap(floor, ball) || !collisionGeometryFinite(floor, ball) || !collisionGeometryFinite(ball, floor) {
		t.Fatal("representable geometry on a long static floor was rejected")
	}
	contact, hit := collideProxies(floor, ball)
	if !hit || !contactFinite(contact) {
		t.Fatalf("finite long-floor collision failed: %+v %v", contact, hit)
	}
	collisionClose(t, contact.Penetration, 0.5, 1e-12)
	collisionCloseVec(t, contact.Normal, Vec2{X: 0, Y: 1}, 1e-12)
	collisionCloseVec(t, contact.Points[0], Vec2{X: 5e199, Y: 0.75}, 1e-12)
}

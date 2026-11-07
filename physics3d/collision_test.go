package physics3d

import (
	"math"
	"math/rand"
	"testing"
)

func collisionTestBody(t testing.TB, shape Shape, position Vec3, rotation Quaternion) *Body {
	t.Helper()
	body, err := newBody(BodyDef{Shape: shape, Position: position, Rotation: rotation})
	if err != nil {
		t.Fatal(err)
	}
	return body
}

func collisionTestNear(t testing.TB, got, want, tolerance float64) {
	t.Helper()
	if !finite(got) || math.Abs(got-want) > tolerance {
		t.Fatalf("got %.12g, want %.12g within %g", got, want, tolerance)
	}
}

func collisionTestVec(t testing.TB, got, want Vec3, tolerance float64) {
	t.Helper()
	if !validVec(got) || got.Sub(want).Length() > tolerance {
		t.Fatalf("got %v, want %v within %g", got, want, tolerance)
	}
}

func TestCollisionSpheres(t *testing.T) {
	tests := []struct {
		name        string
		center      Vec3
		radius      float64
		hit         bool
		penetration float64
		normal      Vec3
		point       Vec3
	}{
		{"separated", Vec3{Z: 3}, 1, false, 0, Vec3{}, Vec3{}},
		{"tangent", Vec3{Z: 2}, 1, true, 0, Vec3{Z: 1}, Vec3{Z: 1}},
		{"overlap", Vec3{Z: 1.5}, 1, true, 0.5, Vec3{Z: 1}, Vec3{Z: 0.75}},
		{"concentric", Vec3{}, 2, true, 3, Vec3{X: 1}, Vec3{X: -0.5}},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			a := collisionTestBody(t, Sphere{Radius: 1}, Vec3{}, Quaternion{})
			b := collisionTestBody(t, Sphere{Radius: test.radius}, test.center, Quaternion{})
			contact, hit, err := collideChecked(a, b)
			if err != nil || hit != test.hit {
				t.Fatalf("hit=%v err=%v, want %v", hit, err, test.hit)
			}
			if !hit {
				return
			}
			collisionTestNear(t, contact.Penetration, test.penetration, 1e-12)
			collisionTestVec(t, contact.Normal, test.normal, 1e-12)
			if len(contact.Points) != 1 {
				t.Fatalf("sphere manifold has %d points", len(contact.Points))
			}
			collisionTestVec(t, contact.Points[0], test.point, 1e-12)
		})
	}
}

func TestCollisionSphereBoxAndInteriorExit(t *testing.T) {
	box := collisionTestBody(t, Box{HalfExtents: Vec3{1, 1, 1}}, Vec3{}, Quaternion{})
	tests := []struct {
		name        string
		center      Vec3
		hit         bool
		penetration float64
		normal      Vec3
	}{
		{"separated", Vec3{X: 1.6}, false, 0, Vec3{}},
		{"face tangent", Vec3{X: 1.5}, true, 0, Vec3{X: -1}},
		{"face overlap", Vec3{X: 1.25}, true, 0.25, Vec3{X: -1}},
		{"edge outside", Vec3{1.4, 1.4, 0}, false, 0, Vec3{}},
		{"edge overlap", Vec3{1.25, 1.25, 0}, true, 0.5 - math.Sqrt(0.125), Vec3{-math.Sqrt(0.5), -math.Sqrt(0.5), 0}},
		{"inside nearest x", Vec3{0.75, 0.2, 0}, true, 0.75, Vec3{X: -1}},
		{"inside nearest negative z", Vec3{0, 0.2, -0.9}, true, 0.6, Vec3{Z: 1}},
		{"on face", Vec3{Z: 1}, true, 0.5, Vec3{Z: -1}},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			sphere := collisionTestBody(t, Sphere{Radius: 0.5}, test.center, Quaternion{})
			contact, hit := Collide(sphere, box)
			if hit != test.hit {
				t.Fatalf("hit=%v, want %v", hit, test.hit)
			}
			if !hit {
				return
			}
			collisionTestNear(t, contact.Penetration, test.penetration, 1e-12)
			collisionTestVec(t, contact.Normal, test.normal, 1e-12)
			reverse, hit := Collide(box, sphere)
			if !hit {
				t.Fatal("swapping shapes lost contact")
			}
			collisionTestVec(t, reverse.Normal, contact.Normal.Mul(-1), 1e-12)
			collisionTestNear(t, reverse.Penetration, contact.Penetration, 1e-12)
			// Moving B along the reported MTV must separate even an interior sphere.
			old := box.Position
			box.Position = box.Position.Add(contact.Normal.Mul(contact.Penetration + 1e-8))
			if _, hit := Collide(sphere, box); hit {
				t.Fatal("reported penetration did not separate the shapes")
			}
			box.Position = old
		})
	}
}

func TestCollisionBoxesFaceClippingAndContainment(t *testing.T) {
	a := collisionTestBody(t, Box{HalfExtents: Vec3{1, 1, 1}}, Vec3{}, Quaternion{})
	b := collisionTestBody(t, Box{HalfExtents: Vec3{1, 1, 1}}, Vec3{Y: 1.5}, Quaternion{})
	contact, hit := Collide(a, b)
	if !hit || len(contact.Points) != 4 {
		t.Fatalf("face contact should have four supporting points: %+v, %v", contact, hit)
	}
	collisionTestNear(t, contact.Penetration, 0.5, 1e-12)
	collisionTestVec(t, contact.Normal, Vec3{Y: 1}, 1e-12)
	for _, point := range contact.Points {
		collisionTestNear(t, point.Y, 0.75, 1e-12)
		collisionTestNear(t, math.Abs(point.X), 1, 1e-12)
		collisionTestNear(t, math.Abs(point.Z), 1, 1e-12)
	}
	b.Position.Y = 2
	contact, hit = Collide(a, b)
	if !hit || len(contact.Points) != 4 || contact.Penetration != 0 {
		t.Fatalf("tangent faces lost their support polygon: %+v", contact)
	}
	b.Position.Y = 2.001
	if _, hit := Collide(a, b); hit {
		t.Fatal("separated faces reported as touching")
	}
	// A rotated incident face clips to an octagon. Reduction should retain four
	// well-spread points rather than four neighboring clipped vertices.
	b.Position.Y = 1.8
	b.Rotation = QuaternionFromAxisAngle(Vec3{Y: 1}, math.Pi/4)
	contact, hit = Collide(a, b)
	if !hit || len(contact.Points) != 4 {
		t.Fatalf("rotated face manifold: %+v, %v", contact, hit)
	}
	area := 0.0
	for i := 1; i < len(contact.Points)-1; i++ {
		area += contact.Points[i].Sub(contact.Points[0]).Cross(contact.Points[i+1].Sub(contact.Points[0])).Length() / 2
	}
	if area < 2 {
		t.Fatalf("reduced support polygon is clustered: area=%g points=%v", area, contact.Points)
	}
	outer := collisionTestBody(t, Box{HalfExtents: Vec3{2, 2, 2}}, Vec3{}, Quaternion{})
	inner := collisionTestBody(t, Box{HalfExtents: Vec3{0.5, 0.5, 0.5}}, Vec3{0.2, 0.1, 0.3}, Quaternion{})
	contact, hit = Collide(outer, inner)
	if !hit {
		t.Fatal("contained box was missed")
	}
	collisionTestNear(t, contact.Penetration, 2.2, 1e-12)
	collisionTestVec(t, contact.Normal, Vec3{Z: 1}, 1e-12)
}

func TestCollisionBoxesRequireEdgeCrossAxes(t *testing.T) {
	half := Vec3{1.5, 0.25, 0.2}
	a := collisionTestBody(t, Box{HalfExtents: half}, Vec3{}, Quaternion{0.7425525612517161, -0.044657783361167186, -0.4334153745828622, 0.5086968539700053})
	b := collisionTestBody(t, Box{HalfExtents: half}, Vec3{0.063922534707475, 0.7694396353834336, -0.6745545741507999}, Quaternion{-0.005693409412246264, -0.0027733981074390033, -0.0018061286974474329, 0.999978315390554})
	// This deterministic fixture overlaps on every face normal. A six-axis
	// approximation would falsely collide the boxes; their long edges separate.
	aa, ab := boxAxes(a.Rotation), boxAxes(b.Rotation)
	for _, axis := range []Vec3{aa[0], aa[1], aa[2], ab[0], ab[1], ab[2]} {
		depth := boxProjection(half, aa, axis) + boxProjection(half, ab, axis) - math.Abs(b.Position.Sub(a.Position).Dot(axis))
		if depth < 0.2 {
			t.Fatalf("fixture no longer demonstrates six-face overlap: depth=%g", depth)
		}
	}
	if _, hit := Collide(a, b); hit {
		t.Fatal("edge cross-axis separation was missed")
	}
}

func TestCollisionBoxEdgeEdgeManifold(t *testing.T) {
	half := Vec3{1.5, 0.25, 0.2}
	a := collisionTestBody(t, Box{HalfExtents: half}, Vec3{}, Quaternion{-0.09504411166496467, -0.3942264483785527, -0.41994525962566404, 0.8119101570707092})
	b := collisionTestBody(t, Box{HalfExtents: half}, Vec3{0.5163676582279262, -0.3749150821464351, 1.1965970639173098}, Quaternion{0.04209639328996008, -0.5820583337738557, -0.5648647700802901, 0.5834070459623545})
	contact, hit := Collide(a, b)
	if !hit || len(contact.Points) != 1 {
		t.Fatalf("skew supporting edges should give a single contact: %+v, %v", contact, hit)
	}
	collisionTestNear(t, contact.Penetration, 0.1433378938921797, 1e-12)
	aa, ab := boxAxes(a.Rotation), boxAxes(b.Rotation)
	collisionTestNear(t, math.Abs(contact.Normal.Dot(aa[0].Cross(ab[0]).Normalized())), 1, 1e-12)
	p0, p1 := supportingBoxEdge(a, Box{half}, aa, 0, contact.Normal)
	q0, q1 := supportingBoxEdge(b, Box{half}, ab, 0, contact.Normal.Mul(-1))
	pa, pb := closestEdgePoints(p0, p1, q0, q1)
	collisionTestVec(t, contact.Points[0], collisionMidpoint(pa, pb), 1e-12)
	collisionTestNear(t, math.Abs(pb.Sub(pa).Dot(contact.Normal)), contact.Penetration, 1e-12)

	pa, pb = closestEdgePoints(Vec3{X: -1}, Vec3{X: 1}, Vec3{Y: -1, Z: 1}, Vec3{Y: 1, Z: 1})
	collisionTestVec(t, pa, Vec3{}, 1e-12)
	collisionTestVec(t, pb, Vec3{Z: 1}, 1e-12)
	pa, pb = closestEdgePoints(Vec3{X: -1}, Vec3{X: 1}, Vec3{X: 2, Y: -1, Z: 1}, Vec3{X: 2, Y: 1, Z: 1})
	collisionTestVec(t, pa, Vec3{X: 1}, 1e-12)
	collisionTestVec(t, pb, Vec3{X: 2, Z: 1}, 1e-12)
}

func TestCollisionCovarianceAndSwap(t *testing.T) {
	rng := rand.New(rand.NewSource(1729))
	for i := 0; i < 150; i++ {
		qa := QuaternionFromAxisAngle(Vec3{1, 2, 3}, rng.Float64()*5-2.5)
		qb := QuaternionFromAxisAngle(Vec3{-2, 1, 1}, rng.Float64()*5-2.5)
		a := collisionTestBody(t, Box{HalfExtents: Vec3{1.3, 0.9, 0.7}}, Vec3{}, qa)
		b := collisionTestBody(t, Box{HalfExtents: Vec3{0.8, 1.1, 1.2}}, Vec3{rng.Float64()*0.6 - 0.3, rng.Float64()*0.6 - 0.3, rng.Float64()*0.6 - 0.3}, qb)
		base, hit := Collide(a, b)
		if !hit {
			t.Fatalf("overlapping fixture %d was missed", i)
		}
		reverse, hit := Collide(b, a)
		if !hit {
			t.Fatal("swapping boxes lost contact")
		}
		collisionTestVec(t, reverse.Normal, base.Normal.Mul(-1), 1e-9)
		collisionTestNear(t, reverse.Penetration, base.Penetration, 1e-9)
		rotation := QuaternionFromAxisAngle(Vec3{2, -3, 1}, rng.Float64()*6-3)
		translation := Vec3{rng.Float64()*20 - 10, rng.Float64()*20 - 10, rng.Float64()*20 - 10}
		a.Position, b.Position = rotation.Rotate(a.Position).Add(translation), rotation.Rotate(b.Position).Add(translation)
		a.Rotation, b.Rotation = rotation.Mul(a.Rotation), rotation.Mul(b.Rotation)
		moved, hit := Collide(a, b)
		if !hit {
			t.Fatal("rigid transform lost contact")
		}
		collisionTestNear(t, moved.Penetration, base.Penetration, 1e-9)
		collisionTestVec(t, moved.Normal, rotation.Rotate(base.Normal), 1e-9)
		if len(moved.Points) != len(base.Points) {
			t.Fatalf("rigid transform changed manifold size %d -> %d", len(base.Points), len(moved.Points))
		}
		for _, point := range base.Points {
			want := rotation.Rotate(point).Add(translation)
			found := false
			for _, candidate := range moved.Points {
				found = found || candidate.Sub(want).Length() < 1e-8
			}
			if !found {
				t.Fatalf("transformed manifold point %v not found in %v", want, moved.Points)
			}
		}
	}
}

func TestCollisionNearParallelAABBAndInvalidGeometry(t *testing.T) {
	a := collisionTestBody(t, Box{HalfExtents: Vec3{1, 1, 1}}, Vec3{}, Quaternion{})
	b := collisionTestBody(t, Box{HalfExtents: Vec3{1, 1, 1}}, Vec3{Y: 1.999}, QuaternionFromAxisAngle(Vec3{1, 2, 3}, 1e-12))
	if contact, hit := Collide(a, b); !hit || !contactFinite(contact) {
		t.Fatalf("near-parallel boxes lost finite contact: %+v, %v", contact, hit)
	}
	box := collisionTestBody(t, Box{HalfExtents: Vec3{2, 1, 0.5}}, Vec3{3, 4, 5}, QuaternionFromAxisAngle(Vec3{Y: 1}, math.Pi/2))
	min, max := bodyAABB(box)
	collisionTestVec(t, min, Vec3{2.5, 3, 3}, 1e-12)
	collisionTestVec(t, max, Vec3{3.5, 5, 7}, 1e-12)
	sphere := collisionTestBody(t, Sphere{Radius: 0.5}, Vec3{4, 4, 5}, Quaternion{})
	if !aabbOverlap(box, sphere) {
		t.Fatal("tangent AABBs must overlap")
	}
	sphere.Position.X += 0.01
	if aabbOverlap(box, sphere) {
		t.Fatal("separated AABBs overlap")
	}
	for _, invalid := range []*Body{nil, {}, {shape: Sphere{Radius: -1}, Rotation: Identity()}, {shape: Sphere{Radius: 1}, Rotation: Quaternion{W: math.NaN()}}, {shape: Sphere{Radius: 1}, Position: Vec3{X: math.Inf(1)}, Rotation: Identity()}} {
		if _, hit := Collide(a, invalid); hit {
			t.Fatal("invalid geometry produced a hit")
		}
		if _, _, err := collideChecked(a, invalid); err == nil {
			t.Fatal("checked collision did not report invalid geometry")
		}
	}
	hugeA := collisionTestBody(t, Sphere{Radius: 1e308}, Vec3{}, Quaternion{})
	hugeB := collisionTestBody(t, Sphere{Radius: 1e308}, Vec3{}, Quaternion{})
	if _, _, err := collideChecked(hugeA, hugeB); err == nil {
		t.Fatal("overflowing radius sum was mistaken for ordinary separation")
	}
}

func FuzzCollisionInvariants(f *testing.F) {
	f.Add(0.0, 0.0, 0.0, 0.2, 1.2, 0.4, 0.7, -0.3)
	f.Add(0.0, 0.0, 0.0, 2.0, 2.0, 0.0, 0.0, 0.0)
	f.Add(-1.0, 2.0, 1.0, 1.5, 2.2, 0.7, 1.1, -0.2)
	f.Fuzz(func(t *testing.T, ax, ay, az, bx, by, bz, angleA, angleB float64) {
		for _, value := range []float64{ax, ay, az, bx, by, bz, angleA, angleB} {
			if !finite(value) || math.Abs(value) > 100 {
				t.Skip()
			}
		}
		a := collisionTestBody(t, Box{HalfExtents: Vec3{1.3, 0.7, 0.9}}, Vec3{ax, ay, az}, QuaternionFromAxisAngle(Vec3{1, 2, 3}, angleA))
		b := collisionTestBody(t, Box{HalfExtents: Vec3{0.8, 1.1, 0.6}}, Vec3{bx, by, bz}, QuaternionFromAxisAngle(Vec3{-2, 1, 1}, angleB))
		ab, hitAB := Collide(a, b)
		ba, hitBA := Collide(b, a)
		if hitAB != hitBA {
			t.Fatal("collision is not symmetric")
		}
		if !hitAB {
			return
		}
		if !contactFinite(ab) || !contactFinite(ba) {
			t.Fatal("successful collision has invalid manifold")
		}
		collisionTestNear(t, ab.Penetration, ba.Penetration, 1e-8)
		if !aabbOverlap(a, b) {
			t.Fatal("narrow-phase hit was culled by AABB broad phase")
		}
		// The MTV is independently checked by using it to separate the shapes.
		b.Position = b.Position.Add(ab.Normal.Mul(ab.Penetration + 1e-7))
		if _, hit := Collide(a, b); hit {
			t.Fatalf("minimum translation did not separate shapes: %+v", ab)
		}
	})
}

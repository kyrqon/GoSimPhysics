package collision3d

import (
	"math"
	"testing"
)

func TestRayIntervalSolidIntervals(t *testing.T) {
	box := Box{HalfExtents: Vec3{X: 1, Y: 1, Z: 1}}
	tests := []struct {
		name            string
		shape           Shape
		origin, dir     Vec3
		entry, exit     float64
		inside, tangent bool
		entryN, exitN   Vec3
	}{
		{name: "wall", shape: box, origin: Vec3{X: -5, Y: .25}, dir: Vec3{X: 7}, entry: 4, exit: 6, entryN: Vec3{X: -1}, exitN: Vec3{X: 1}},
		{name: "angled wall", shape: Box{HalfExtents: Vec3{X: 1, Y: 3, Z: 4}}, origin: Vec3{X: -3, Y: -3}, dir: Vec3{X: 1, Y: 1}, entry: 2 * math.Sqrt2, exit: 4 * math.Sqrt2, entryN: Vec3{X: -1}, exitN: Vec3{X: 1}},
		{name: "spatial angled wall", shape: Box{HalfExtents: Vec3{X: 1, Y: 3, Z: 4}}, origin: Vec3{X: -3, Y: -3, Z: -3}, dir: Vec3{X: 1, Y: 1, Z: 1}, entry: 2 * math.Sqrt(3), exit: 4 * math.Sqrt(3), entryN: Vec3{X: -1}, exitN: Vec3{X: 1}},
		{name: "sphere chord", shape: Sphere{Radius: 1}, origin: Vec3{X: -5, Y: .6}, dir: Vec3{X: 1}, entry: 4.2, exit: 5.8, entryN: Vec3{X: -.8, Y: .6}, exitN: Vec3{X: .8, Y: .6}},
		{name: "sphere inside", shape: Sphere{Radius: 1}, origin: Vec3{X: .25}, dir: Vec3{X: 1}, exit: .75, inside: true, exitN: Vec3{X: 1}},
		{name: "box inside", shape: box, origin: Vec3{X: .25}, dir: Vec3{X: 1}, exit: .75, inside: true, exitN: Vec3{X: 1}},
		{name: "sphere boundary entering", shape: Sphere{Radius: 1}, origin: Vec3{X: -1}, dir: Vec3{X: 1}, exit: 2, entryN: Vec3{X: -1}, exitN: Vec3{X: 1}},
		{name: "box boundary entering", shape: box, origin: Vec3{X: -1}, dir: Vec3{X: 1}, exit: 2, entryN: Vec3{X: -1}, exitN: Vec3{X: 1}},
		{name: "sphere boundary leaving", shape: Sphere{Radius: 1}, origin: Vec3{X: 1}, dir: Vec3{X: 1}, tangent: true, entryN: Vec3{X: 1}, exitN: Vec3{X: 1}},
		{name: "box boundary leaving", shape: box, origin: Vec3{X: 1}, dir: Vec3{X: 1}, tangent: true, entryN: Vec3{X: 1}, exitN: Vec3{X: 1}},
		{name: "sphere grazing", shape: Sphere{Radius: 1}, origin: Vec3{X: -2, Y: 1}, dir: Vec3{X: 1}, entry: 2, exit: 2, tangent: true, entryN: Vec3{Y: 1}, exitN: Vec3{Y: 1}},
		{name: "box face", shape: box, origin: Vec3{X: -2, Y: 1}, dir: Vec3{X: 1}, entry: 1, exit: 3, tangent: true, entryN: Vec3{X: -1}, exitN: Vec3{X: 1}},
		{name: "box edge", shape: box, origin: Vec3{X: -2, Y: 1, Z: 1}, dir: Vec3{X: 1}, entry: 1, exit: 3, tangent: true, entryN: Vec3{X: -1}, exitN: Vec3{X: 1}},
		{name: "box boundary parallel start", shape: box, origin: Vec3{Y: 1}, dir: Vec3{X: 1}, exit: 1, tangent: true, entryN: Vec3{Y: 1}, exitN: Vec3{X: 1}},
		{name: "box corner grazing", shape: box, origin: Vec3{X: -2}, dir: Vec3{X: 1, Y: 1}, entry: math.Sqrt2, exit: math.Sqrt2, tangent: true, entryN: Vec3{X: -1}, exitN: Vec3{Y: 1}},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			hit, ok, err := RayInterval(Collider{Shape: test.shape}, test.origin, test.dir)
			if err != nil || !ok {
				t.Fatalf("RayInterval: hit=%v err=%v", ok, err)
			}
			rayClose3D(t, hit.EntryDistance, test.entry)
			rayClose3D(t, hit.ExitDistance, test.exit)
			rayVecClose3D(t, hit.EntryNormal, test.entryN)
			rayVecClose3D(t, hit.ExitNormal, test.exitN)
			rayVecClose3D(t, hit.EntryPoint, test.origin.Add(test.dir.Normalized().Mul(test.entry)))
			rayVecClose3D(t, hit.ExitPoint, test.origin.Add(test.dir.Normalized().Mul(test.exit)))
			if hit.StartsInside != test.inside || hit.Tangent != test.tangent {
				t.Fatalf("flags: inside=%v tangent=%v, want %v %v", hit.StartsInside, hit.Tangent, test.inside, test.tangent)
			}
		})
	}
}

func TestRayIntervalMisses(t *testing.T) {
	tests := []struct {
		collider          Collider
		origin, direction Vec3
	}{
		{Collider{Shape: Sphere{Radius: 1}}, Vec3{X: 2}, Vec3{X: 1}},
		{Collider{Shape: Box{HalfExtents: Vec3{X: 1, Y: 1, Z: 1}}}, Vec3{X: 2}, Vec3{X: 1}},
		{Collider{Shape: Sphere{Radius: 1}}, Vec3{X: .9, Y: .9}, Vec3{X: 1, Y: 1}},
		{Collider{Shape: Sphere{Radius: 1}}, Vec3{X: -2, Y: 1.01}, Vec3{X: 1}},
		{Collider{Shape: Box{HalfExtents: Vec3{X: 1, Y: 1, Z: 1}}, Rotation: QuaternionFromAxisAngle(Vec3{Z: 1}, math.Pi/4)}, Vec3{X: 1.3, Y: 1.3}, Vec3{X: 1}},
	}
	for i, test := range tests {
		if _, hit, err := RayInterval(test.collider, test.origin, test.direction); hit || err != nil {
			t.Errorf("case %d: hit=%v err=%v", i, hit, err)
		}
	}
}

func TestRayIntervalRotationCovariance(t *testing.T) {
	position := Vec3{X: 4, Y: -3, Z: 2}
	rotation := QuaternionFromAxisAngle(Vec3{X: 1, Y: 2, Z: 3}, .71)
	origin, direction := Vec3{X: -5, Y: .4, Z: -.1}, Vec3{X: 3, Y: .2, Z: -.3}
	for _, shape := range []Shape{Sphere{Radius: 1}, Box{HalfExtents: Vec3{X: 1, Y: 2, Z: 3}}} {
		base, ok, err := RayInterval(Collider{Shape: shape}, origin, direction)
		if err != nil || !ok {
			t.Fatalf("base: %v %v", ok, err)
		}
		rotated, ok, err := RayInterval(Collider{Shape: shape, Position: position, Rotation: rotation}, rotation.Rotate(origin).Add(position), rotation.Rotate(direction))
		if err != nil || !ok {
			t.Fatalf("rotated: %v %v", ok, err)
		}
		rayClose3D(t, rotated.EntryDistance, base.EntryDistance)
		rayClose3D(t, rotated.ExitDistance, base.ExitDistance)
		rayVecClose3D(t, rotated.EntryPoint, rotation.Rotate(base.EntryPoint).Add(position))
		rayVecClose3D(t, rotated.ExitPoint, rotation.Rotate(base.ExitPoint).Add(position))
		rayVecClose3D(t, rotated.EntryNormal, rotation.Rotate(base.EntryNormal))
		rayVecClose3D(t, rotated.ExitNormal, rotation.Rotate(base.ExitNormal))
	}
}

func TestRayIntervalRotatedGrazing(t *testing.T) {
	for _, angle := range []float64{.1, .3, .63, math.Pi / 4, 1.2} {
		rotation := QuaternionFromAxisAngle(Vec3{X: 1, Y: 2, Z: 3}, angle)
		for _, shape := range []Shape{Sphere{Radius: 1}, Box{HalfExtents: Vec3{X: 1, Y: 1, Z: 1}}} {
			c := Collider{Shape: shape, Rotation: rotation}
			for _, gap := range []float64{0, 1e-8, -1e-8} {
				origin := Vec3{X: -2, Y: 1 - gap}
				if _, box := shape.(Box); box {
					origin.Z = 1 - gap // test an edge, not just a face
				}
				hit, ok, err := RayInterval(c, rotation.Rotate(origin), rotation.Rotate(Vec3{X: 1}))
				if err != nil {
					t.Fatal(err)
				}
				if gap < 0 {
					if ok {
						t.Fatalf("rotated outside ray hit: shape=%T angle=%g hit=%+v", shape, angle, hit)
					}
					continue
				}
				if !ok || hit.Tangent != (gap == 0) || hit.StartsInside {
					t.Fatalf("shape=%T angle=%g gap=%g: hit=%v result=%+v", shape, angle, gap, ok, hit)
				}
			}
			for _, sign := range []float64{-1, 1} {
				hit, ok, err := RayInterval(c, rotation.Rotate(Vec3{X: sign}), rotation.Rotate(Vec3{X: 1}))
				if err != nil || !ok || hit.StartsInside || hit.EntryDistance != 0 || (sign > 0 && hit.ExitDistance != 0) {
					t.Fatalf("rotated boundary shape=%T angle=%g sign=%g: hit=%v result=%+v err=%v", shape, angle, sign, ok, hit, err)
				}
			}
		}
	}
}

func TestRayIntervalCopiesAndValidates(t *testing.T) {
	shape := &Box{HalfExtents: Vec3{X: 1, Y: 2, Z: 3}}
	c := Collider{Shape: shape, Position: Vec3{X: 2}, Rotation: QuaternionFromAxisAngle(Vec3{Z: 1}, .2)}
	before, colliderBefore := *shape, c
	if _, hit, err := RayInterval(c, Vec3{X: -3}, Vec3{X: 1}); !hit || err != nil {
		t.Fatalf("pointer shape: %v %v", hit, err)
	}
	if *shape != before || c != colliderBefore {
		t.Fatal("query mutated caller input")
	}
	var nilSphere *Sphere
	invalid := []struct {
		collider          Collider
		origin, direction Vec3
	}{
		{Collider{}, Vec3{}, Vec3{X: 1}},
		{Collider{Shape: nilSphere}, Vec3{}, Vec3{X: 1}},
		{Collider{Shape: Sphere{}}, Vec3{}, Vec3{X: 1}},
		{Collider{Shape: Box{HalfExtents: Vec3{X: 1, Y: -1, Z: 1}}}, Vec3{}, Vec3{X: 1}},
		{Collider{Shape: Sphere{Radius: 1}, Rotation: Quaternion{W: math.Inf(1)}}, Vec3{}, Vec3{X: 1}},
		{Collider{Shape: Sphere{Radius: 1}}, Vec3{X: math.NaN()}, Vec3{X: 1}},
		{Collider{Shape: Sphere{Radius: 1}}, Vec3{}, Vec3{X: math.Inf(1)}},
		{Collider{Shape: Sphere{Radius: 1}}, Vec3{}, Vec3{}},
		{Collider{Shape: Sphere{Radius: math.Inf(1)}}, Vec3{}, Vec3{X: 1}},
		{Collider{Shape: Sphere{Radius: math.MaxFloat64}, Position: Vec3{X: math.MaxFloat64}}, Vec3{}, Vec3{X: 1}},
		{Collider{Shape: Sphere{Radius: 1}, Position: Vec3{X: math.MaxFloat64}}, Vec3{X: -math.MaxFloat64}, Vec3{X: 1}},
		{Collider{Shape: Sphere{Radius: math.MaxFloat64 / 3}, Position: Vec3{X: math.MaxFloat64 / 2}}, Vec3{X: -math.MaxFloat64 / 2}, Vec3{X: 1}},
		{Collider{Shape: Box{HalfExtents: Vec3{X: math.MaxFloat64, Y: math.MaxFloat64, Z: math.MaxFloat64}}, Rotation: QuaternionFromAxisAngle(Vec3{Z: 1}, math.Pi/4)}, Vec3{}, Vec3{X: 1}},
		{Collider{Shape: Sphere{Radius: 1}}, Vec3{X: -1e20}, Vec3{X: 1}},
		{Collider{Shape: Box{HalfExtents: Vec3{X: 1, Y: 1, Z: 1}}}, Vec3{X: -1e20}, Vec3{X: 1}},
	}
	for i, test := range invalid {
		if _, hit, err := RayInterval(test.collider, test.origin, test.direction); hit || err == nil {
			t.Errorf("invalid case %d: hit=%v err=%v", i, hit, err)
		}
	}
}

func TestRayIntervalNumericScales(t *testing.T) {
	for _, radius := range []float64{1e-200, 1, 1e200} {
		for _, speed := range []float64{math.SmallestNonzeroFloat64, 1, math.MaxFloat64} {
			hit, ok, err := RayInterval(Collider{Shape: Sphere{Radius: radius}}, Vec3{X: -3 * radius}, Vec3{X: speed})
			if !ok || err != nil {
				t.Fatalf("radius=%g direction=%g: hit=%v err=%v", radius, speed, ok, err)
			}
			rayClose3D(t, hit.EntryDistance/radius, 2)
			rayClose3D(t, hit.ExitDistance/radius, 4)
			rayVecClose3D(t, hit.EntryNormal, Vec3{X: -1})
			rayVecClose3D(t, hit.ExitNormal, Vec3{X: 1})
		}
	}
	// Slab division may overflow on a nearly parallel axis while the actual
	// interval remains finite and is bounded by the other axis.
	hit, ok, err := RayInterval(Collider{Shape: Box{HalfExtents: Vec3{X: 1, Y: 1, Z: 1}}}, Vec3{Y: -3}, Vec3{X: math.SmallestNonzeroFloat64, Y: 1})
	if !ok || err != nil {
		t.Fatalf("nearly parallel: hit=%v err=%v", ok, err)
	}
	rayClose3D(t, hit.EntryDistance, 2)
	rayClose3D(t, hit.ExitDistance, 4)
	hit, ok, err = RayInterval(Collider{Shape: Sphere{Radius: 1}}, Vec3{X: -1e12, Y: .6}, Vec3{X: 1})
	if !ok || err != nil {
		t.Fatalf("distant chord: hit=%v err=%v", ok, err)
	}
	if math.Abs((hit.ExitDistance-hit.EntryDistance)-1.6) > .001 {
		t.Fatalf("distant chord length: %g", hit.ExitDistance-hit.EntryDistance)
	}
	rayVecClose3D(t, hit.EntryNormal, Vec3{X: -.8, Y: .6})
	rayVecClose3D(t, hit.ExitNormal, Vec3{X: .8, Y: .6})
}

func rayClose3D(t *testing.T, got, want float64) {
	t.Helper()
	if !finite(got) || math.Abs(got-want) > 2e-12*math.Max(1, math.Abs(want)) {
		t.Fatalf("got %.17g, want %.17g", got, want)
	}
}

func rayVecClose3D(t *testing.T, got, want Vec3) {
	t.Helper()
	rayClose3D(t, got.X, want.X)
	rayClose3D(t, got.Y, want.Y)
	rayClose3D(t, got.Z, want.Z)
}

package collision2d

import (
	"math"
	"testing"
)

func TestRayIntervalSolidIntervals(t *testing.T) {
	box := Box{HalfExtents: Vec2{X: 1, Y: 1}}
	tests := []struct {
		name            string
		shape           Shape
		origin, dir     Vec2
		entry, exit     float64
		inside, tangent bool
		entryN, exitN   Vec2
	}{
		{name: "wall", shape: box, origin: Vec2{X: -5, Y: .25}, dir: Vec2{X: 7}, entry: 4, exit: 6, entryN: Vec2{X: -1}, exitN: Vec2{X: 1}},
		{name: "angled wall", shape: Box{HalfExtents: Vec2{X: 1, Y: 3}}, origin: Vec2{X: -3, Y: -3}, dir: Vec2{X: 1, Y: 1}, entry: 2 * math.Sqrt2, exit: 4 * math.Sqrt2, entryN: Vec2{X: -1}, exitN: Vec2{X: 1}},
		{name: "circle chord", shape: Circle{Radius: 1}, origin: Vec2{X: -5, Y: .6}, dir: Vec2{X: 1}, entry: 4.2, exit: 5.8, entryN: Vec2{X: -.8, Y: .6}, exitN: Vec2{X: .8, Y: .6}},
		{name: "circle inside", shape: Circle{Radius: 1}, origin: Vec2{X: .25}, dir: Vec2{X: 1}, exit: .75, inside: true, exitN: Vec2{X: 1}},
		{name: "box inside", shape: box, origin: Vec2{X: .25}, dir: Vec2{X: 1}, exit: .75, inside: true, exitN: Vec2{X: 1}},
		{name: "circle boundary entering", shape: Circle{Radius: 1}, origin: Vec2{X: -1}, dir: Vec2{X: 1}, exit: 2, entryN: Vec2{X: -1}, exitN: Vec2{X: 1}},
		{name: "box boundary entering", shape: box, origin: Vec2{X: -1}, dir: Vec2{X: 1}, exit: 2, entryN: Vec2{X: -1}, exitN: Vec2{X: 1}},
		{name: "circle boundary leaving", shape: Circle{Radius: 1}, origin: Vec2{X: 1}, dir: Vec2{X: 1}, tangent: true, entryN: Vec2{X: 1}, exitN: Vec2{X: 1}},
		{name: "box boundary leaving", shape: box, origin: Vec2{X: 1}, dir: Vec2{X: 1}, tangent: true, entryN: Vec2{X: 1}, exitN: Vec2{X: 1}},
		{name: "circle grazing", shape: Circle{Radius: 1}, origin: Vec2{X: -2, Y: 1}, dir: Vec2{X: 1}, entry: 2, exit: 2, tangent: true, entryN: Vec2{Y: 1}, exitN: Vec2{Y: 1}},
		{name: "box face", shape: box, origin: Vec2{X: -2, Y: 1}, dir: Vec2{X: 1}, entry: 1, exit: 3, tangent: true, entryN: Vec2{X: -1}, exitN: Vec2{X: 1}},
		{name: "box boundary parallel start", shape: box, origin: Vec2{Y: 1}, dir: Vec2{X: 1}, exit: 1, tangent: true, entryN: Vec2{Y: 1}, exitN: Vec2{X: 1}},
		{name: "box corner grazing", shape: box, origin: Vec2{X: -2}, dir: Vec2{X: 1, Y: 1}, entry: math.Sqrt2, exit: math.Sqrt2, tangent: true, entryN: Vec2{X: -1}, exitN: Vec2{Y: 1}},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			hit, ok, err := RayInterval(Collider{Shape: test.shape}, test.origin, test.dir)
			if err != nil || !ok {
				t.Fatalf("RayInterval: hit=%v err=%v", ok, err)
			}
			rayClose2D(t, hit.EntryDistance, test.entry)
			rayClose2D(t, hit.ExitDistance, test.exit)
			rayVecClose2D(t, hit.EntryNormal, test.entryN)
			rayVecClose2D(t, hit.ExitNormal, test.exitN)
			rayVecClose2D(t, hit.EntryPoint, test.origin.Add(test.dir.Normalized().Mul(test.entry)))
			rayVecClose2D(t, hit.ExitPoint, test.origin.Add(test.dir.Normalized().Mul(test.exit)))
			if hit.StartsInside != test.inside || hit.Tangent != test.tangent {
				t.Fatalf("flags: inside=%v tangent=%v, want %v %v", hit.StartsInside, hit.Tangent, test.inside, test.tangent)
			}
		})
	}
}

func TestRayIntervalMisses(t *testing.T) {
	tests := []struct {
		collider          Collider
		origin, direction Vec2
	}{
		{Collider{Shape: Circle{Radius: 1}}, Vec2{X: 2}, Vec2{X: 1}},
		{Collider{Shape: Box{HalfExtents: Vec2{X: 1, Y: 1}}}, Vec2{X: 2}, Vec2{X: 1}},
		{Collider{Shape: Circle{Radius: 1}}, Vec2{X: .9, Y: .9}, Vec2{X: 1, Y: 1}},
		{Collider{Shape: Circle{Radius: 1}}, Vec2{X: -2, Y: 1.01}, Vec2{X: 1}},
		{Collider{Shape: Box{HalfExtents: Vec2{X: 1, Y: 1}}, Angle: math.Pi / 4}, Vec2{X: 1.3, Y: 1.3}, Vec2{X: 1}},
	}
	for i, test := range tests {
		if _, hit, err := RayInterval(test.collider, test.origin, test.direction); hit || err != nil {
			t.Errorf("case %d: hit=%v err=%v", i, hit, err)
		}
	}
}

func TestRayIntervalRotationCovariance(t *testing.T) {
	position, angle := Vec2{X: 4, Y: -3}, .63
	origin, direction := Vec2{X: -5, Y: .4}, Vec2{X: 3, Y: .2}
	for _, shape := range []Shape{Circle{Radius: 1}, Box{HalfExtents: Vec2{X: 1, Y: 2}}} {
		base, ok, err := RayInterval(Collider{Shape: shape}, origin, direction)
		if err != nil || !ok {
			t.Fatalf("base: %v %v", ok, err)
		}
		rotated, ok, err := RayInterval(Collider{Shape: shape, Position: position, Angle: angle}, origin.Rotate(angle).Add(position), direction.Rotate(angle))
		if err != nil || !ok {
			t.Fatalf("rotated: %v %v", ok, err)
		}
		rayClose2D(t, rotated.EntryDistance, base.EntryDistance)
		rayClose2D(t, rotated.ExitDistance, base.ExitDistance)
		rayVecClose2D(t, rotated.EntryPoint, base.EntryPoint.Rotate(angle).Add(position))
		rayVecClose2D(t, rotated.ExitPoint, base.ExitPoint.Rotate(angle).Add(position))
		rayVecClose2D(t, rotated.EntryNormal, base.EntryNormal.Rotate(angle))
		rayVecClose2D(t, rotated.ExitNormal, base.ExitNormal.Rotate(angle))
	}
}

func TestRayIntervalRotatedGrazing(t *testing.T) {
	for _, angle := range []float64{.1, .3, .63, math.Pi / 4, 1.2} {
		for _, shape := range []Shape{Circle{Radius: 1}, Box{HalfExtents: Vec2{X: 1, Y: 1}}} {
			c := Collider{Shape: shape, Angle: angle}
			for _, gap := range []float64{0, 1e-8, -1e-8} {
				origin, direction := (Vec2{X: -2, Y: 1 - gap}).Rotate(angle), (Vec2{X: 1}).Rotate(angle)
				hit, ok, err := RayInterval(c, origin, direction)
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
				hit, ok, err := RayInterval(c, (Vec2{X: sign}).Rotate(angle), (Vec2{X: 1}).Rotate(angle))
				if err != nil || !ok || hit.StartsInside || hit.EntryDistance != 0 || (sign > 0 && hit.ExitDistance != 0) {
					t.Fatalf("rotated boundary shape=%T angle=%g sign=%g: hit=%v result=%+v err=%v", shape, angle, sign, ok, hit, err)
				}
			}
		}
	}
}

func TestRayIntervalCopiesAndValidates(t *testing.T) {
	shape := &Box{HalfExtents: Vec2{X: 1, Y: 2}}
	c := Collider{Shape: shape, Position: Vec2{X: 2}, Angle: .2}
	before, colliderBefore := *shape, c
	if _, hit, err := RayInterval(c, Vec2{X: -3}, Vec2{X: 1}); !hit || err != nil {
		t.Fatalf("pointer shape: %v %v", hit, err)
	}
	if *shape != before || c != colliderBefore {
		t.Fatal("query mutated caller input")
	}
	var nilCircle *Circle
	invalid := []struct {
		collider          Collider
		origin, direction Vec2
	}{
		{Collider{}, Vec2{}, Vec2{X: 1}},
		{Collider{Shape: nilCircle}, Vec2{}, Vec2{X: 1}},
		{Collider{Shape: Circle{}}, Vec2{}, Vec2{X: 1}},
		{Collider{Shape: Box{HalfExtents: Vec2{X: 1, Y: -1}}}, Vec2{}, Vec2{X: 1}},
		{Collider{Shape: Circle{Radius: 1}, Angle: math.Inf(1)}, Vec2{}, Vec2{X: 1}},
		{Collider{Shape: Circle{Radius: 1}}, Vec2{X: math.NaN()}, Vec2{X: 1}},
		{Collider{Shape: Circle{Radius: 1}}, Vec2{}, Vec2{X: math.Inf(1)}},
		{Collider{Shape: Circle{Radius: 1}}, Vec2{}, Vec2{}},
		{Collider{Shape: Circle{Radius: math.Inf(1)}}, Vec2{}, Vec2{X: 1}},
		{Collider{Shape: Circle{Radius: math.MaxFloat64}, Position: Vec2{X: math.MaxFloat64}}, Vec2{}, Vec2{X: 1}},
		{Collider{Shape: Circle{Radius: 1}, Position: Vec2{X: math.MaxFloat64}}, Vec2{X: -math.MaxFloat64}, Vec2{X: 1}},
		{Collider{Shape: Circle{Radius: math.MaxFloat64 / 3}, Position: Vec2{X: math.MaxFloat64 / 2}}, Vec2{X: -math.MaxFloat64 / 2}, Vec2{X: 1}},
		{Collider{Shape: Box{HalfExtents: Vec2{X: math.MaxFloat64, Y: math.MaxFloat64}}, Angle: math.Pi / 4}, Vec2{}, Vec2{X: 1}},
		{Collider{Shape: Circle{Radius: 1}}, Vec2{X: -1e20}, Vec2{X: 1}},
		{Collider{Shape: Box{HalfExtents: Vec2{X: 1, Y: 1}}}, Vec2{X: -1e20}, Vec2{X: 1}},
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
			hit, ok, err := RayInterval(Collider{Shape: Circle{Radius: radius}}, Vec2{X: -3 * radius}, Vec2{X: speed})
			if !ok || err != nil {
				t.Fatalf("radius=%g direction=%g: hit=%v err=%v", radius, speed, ok, err)
			}
			rayClose2D(t, hit.EntryDistance/radius, 2)
			rayClose2D(t, hit.ExitDistance/radius, 4)
			rayVecClose2D(t, hit.EntryNormal, Vec2{X: -1})
			rayVecClose2D(t, hit.ExitNormal, Vec2{X: 1})
		}
	}
	// Slab division may overflow on a nearly parallel axis while the actual
	// interval remains finite and is bounded by the other axis.
	hit, ok, err := RayInterval(Collider{Shape: Box{HalfExtents: Vec2{X: 1, Y: 1}}}, Vec2{Y: -3}, Vec2{X: math.SmallestNonzeroFloat64, Y: 1})
	if !ok || err != nil {
		t.Fatalf("nearly parallel: hit=%v err=%v", ok, err)
	}
	rayClose2D(t, hit.EntryDistance, 2)
	rayClose2D(t, hit.ExitDistance, 4)
	hit, ok, err = RayInterval(Collider{Shape: Circle{Radius: 1}}, Vec2{X: -1e12, Y: .6}, Vec2{X: 1})
	if !ok || err != nil {
		t.Fatalf("distant chord: hit=%v err=%v", ok, err)
	}
	if math.Abs((hit.ExitDistance-hit.EntryDistance)-1.6) > .001 {
		t.Fatalf("distant chord length: %g", hit.ExitDistance-hit.EntryDistance)
	}
	rayVecClose2D(t, hit.EntryNormal, Vec2{X: -.8, Y: .6})
	rayVecClose2D(t, hit.ExitNormal, Vec2{X: .8, Y: .6})
}

func rayClose2D(t *testing.T, got, want float64) {
	t.Helper()
	if !finite(got) || math.Abs(got-want) > 2e-12*math.Max(1, math.Abs(want)) {
		t.Fatalf("got %.17g, want %.17g", got, want)
	}
}

func rayVecClose2D(t *testing.T, got, want Vec2) {
	t.Helper()
	rayClose2D(t, got.X, want.X)
	rayClose2D(t, got.Y, want.Y)
}

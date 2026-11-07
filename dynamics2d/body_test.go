package dynamics2d

import (
	"math"
	"testing"
)

func coreBody(t *testing.T, def BodyDef) *Body {
	t.Helper()
	body, err := newBody(def)
	if err != nil {
		t.Fatal(err)
	}
	return body
}

func coreDynamicDef() BodyDef {
	return BodyDef{Type: Dynamic, Shape: Circle{Radius: 2}, Mass: 4}
}

func TestCoreBodyMassProperties(t *testing.T) {
	body := coreBody(t, coreDynamicDef())
	if body.Type() != Dynamic {
		t.Fatalf("got type %v, want Dynamic", body.Type())
	}
	coreClose(t, body.Mass(), 4, 0)
	coreClose(t, body.Inertia(), 8, 0)
	coreClose(t, body.invMass, 0.25, 0)
	coreClose(t, body.invInertia, 0.125, 0)
	box := coreDynamicDef()
	box.Shape = Box{HalfExtents: Vec2{X: 2, Y: 3}}
	box.Mass = 3
	coreClose(t, coreBody(t, box).Inertia(), 13, 1e-14)
}

func TestCoreBodyForcesAndOffCenterImpulse(t *testing.T) {
	def := coreDynamicDef()
	def.Position = Vec2{X: 3, Y: 5}
	def.Velocity = Vec2{X: 1, Y: -2}
	def.AngularVelocity = 0.5
	body := coreBody(t, def)
	if err := body.ApplyForce(Vec2{X: 3, Y: 1}); err != nil {
		t.Fatal(err)
	}
	if err := body.ApplyForceAtPoint(Vec2{X: 2, Y: 3}, Vec2{X: 4, Y: 7}); err != nil {
		t.Fatal(err)
	}
	coreVecClose(t, body.force, Vec2{X: 5, Y: 4}, 0)
	coreClose(t, body.torque, -1, 0)
	if err := body.ApplyTorque(2); err != nil {
		t.Fatal(err)
	}
	coreClose(t, body.torque, 1, 0)
	if err := body.ApplyImpulse(Vec2{X: 4, Y: 0}, Vec2{X: 3, Y: 7}); err != nil {
		t.Fatal(err)
	}
	coreVecClose(t, body.Velocity, Vec2{X: 2, Y: -2}, 0)
	coreClose(t, body.AngularVelocity, -0.5, 0)
	// Impulses change velocity immediately and do not consume queued forces.
	coreVecClose(t, body.force, Vec2{X: 5, Y: 4}, 0)
	coreClose(t, body.torque, 1, 0)
}

func TestCoreBodyTransforms(t *testing.T) {
	def := coreDynamicDef()
	def.Position = Vec2{X: 3, Y: 5}
	def.Angle = math.Pi / 2
	body := coreBody(t, def)
	coreVecClose(t, body.WorldPoint(Vec2{X: 2, Y: 1}), Vec2{X: 2, Y: 7}, 1e-14)
	coreVecClose(t, body.LocalPoint(Vec2{X: 2, Y: 7}), Vec2{X: 2, Y: 1}, 1e-14)
	point := Vec2{X: -12, Y: 7}
	coreVecClose(t, body.LocalPoint(body.WorldPoint(point)), point, 1e-13)
}

func TestCoreBodyStaticKinematicAndFixedRotation(t *testing.T) {
	for _, kind := range []BodyType{Static, Kinematic} {
		def := coreDynamicDef()
		def.Type = kind
		def.Velocity = Vec2{X: 2, Y: 3}
		def.AngularVelocity = 4
		body := coreBody(t, def)
		wantVelocity, wantAngular := def.Velocity, def.AngularVelocity
		if kind == Static {
			wantVelocity, wantAngular = Vec2{}, 0
		}
		for _, err := range []error{
			body.ApplyForce(Vec2{X: 5, Y: 6}),
			body.ApplyForceAtPoint(Vec2{X: 5, Y: 6}, Vec2{X: 3, Y: 4}),
			body.ApplyTorque(7),
			body.ApplyImpulse(Vec2{X: 5, Y: 6}, Vec2{X: 3, Y: 4}),
		} {
			if err != nil {
				t.Fatal(err)
			}
		}
		coreVecClose(t, body.Velocity, wantVelocity, 0)
		coreClose(t, body.AngularVelocity, wantAngular, 0)
		coreVecClose(t, body.force, Vec2{}, 0)
		coreClose(t, body.torque, 0, 0)
		coreClose(t, body.invMass, 0, 0)
		coreClose(t, body.invInertia, 0, 0)
	}
	def := coreDynamicDef()
	def.FixedRotation = true
	def.AngularVelocity = 9
	body := coreBody(t, def)
	if err := body.ApplyForceAtPoint(Vec2{X: 4, Y: 0}, Vec2{X: 0, Y: 2}); err != nil {
		t.Fatal(err)
	}
	if err := body.ApplyTorque(20); err != nil {
		t.Fatal(err)
	}
	if err := body.ApplyImpulse(Vec2{X: 4, Y: 0}, Vec2{X: 0, Y: 2}); err != nil {
		t.Fatal(err)
	}
	coreClose(t, body.AngularVelocity, 0, 0)
	coreClose(t, body.invInertia, 0, 0)
	coreVecClose(t, body.Velocity, Vec2{X: 1, Y: 0}, 0)
}

func TestCoreBodyFixedRotationExtremeImpulse(t *testing.T) {
	// Both inputs are finite, but their cross product overflows. A locked
	// body's zero inverse inertia must prevent evaluating that angular update.
	impulse, point := Vec2{X: 1e308, Y: 0}, Vec2{X: 0, Y: 1e308}
	for name, apply := range map[string]func(*Body) error{
		"public": func(body *Body) error { return body.ApplyImpulse(impulse, point) },
		"solver": func(body *Body) error {
			body.applyImpulse(impulse, point)
			return nil
		},
	} {
		t.Run(name, func(t *testing.T) {
			def := coreDynamicDef()
			def.FixedRotation = true
			body := coreBody(t, def)
			if err := apply(body); err != nil {
				t.Fatal(err)
			}
			if !body.validState() {
				t.Fatal("fixed-rotation impulse produced nonfinite state")
			}
			coreVecClose(t, body.Velocity, Vec2{X: 2.5e307, Y: 0}, 0)
			coreClose(t, body.AngularVelocity, 0, 0)
		})
	}
}

func TestCoreBodyDefinitionValidation(t *testing.T) {
	invalid := map[string]func(*BodyDef){
		"body type":             func(d *BodyDef) { d.Type = BodyType(255) },
		"nil shape":             func(d *BodyDef) { d.Shape = nil },
		"nil circle pointer":    func(d *BodyDef) { d.Shape = (*Circle)(nil) },
		"nil box pointer":       func(d *BodyDef) { d.Shape = (*Box)(nil) },
		"invalid shape":         func(d *BodyDef) { d.Shape = Circle{} },
		"zero mass":             func(d *BodyDef) { d.Mass = 0 },
		"negative mass":         func(d *BodyDef) { d.Mass = -1 },
		"nan mass":              func(d *BodyDef) { d.Mass = math.NaN() },
		"infinite mass":         func(d *BodyDef) { d.Mass = math.Inf(1) },
		"inverse mass overflow": func(d *BodyDef) { d.Mass = math.SmallestNonzeroFloat64 },
		"inertia overflow":      func(d *BodyDef) { d.Shape = Circle{Radius: math.MaxFloat64} },
		"inertia underflow":     func(d *BodyDef) { d.Shape = Circle{Radius: math.SmallestNonzeroFloat64} },
		"position":              func(d *BodyDef) { d.Position.Y = math.NaN() },
		"velocity":              func(d *BodyDef) { d.Velocity.X = math.Inf(1) },
		"angle":                 func(d *BodyDef) { d.Angle = math.NaN() },
		"angular velocity":      func(d *BodyDef) { d.AngularVelocity = math.Inf(-1) },
		"negative friction":     func(d *BodyDef) { d.Friction = -1 },
		"nan friction":          func(d *BodyDef) { d.Friction = math.NaN() },
		"negative restitution":  func(d *BodyDef) { d.Restitution = -0.1 },
		"excess restitution":    func(d *BodyDef) { d.Restitution = 1.1 },
		"nan restitution":       func(d *BodyDef) { d.Restitution = math.NaN() },
		"linear damping":        func(d *BodyDef) { d.LinearDamping = -1 },
		"angular damping":       func(d *BodyDef) { d.AngularDamping = math.Inf(1) },
	}
	for name, change := range invalid {
		t.Run(name, func(t *testing.T) {
			def := coreDynamicDef()
			change(&def)
			if _, err := newBody(def); err == nil {
				t.Fatal("invalid body definition accepted")
			}
		})
	}
}

func TestCoreBodyCopiesShapeDefinition(t *testing.T) {
	shape := &Circle{Radius: 2}
	def := coreDynamicDef()
	def.Shape = shape
	body := coreBody(t, def)
	shape.Radius = 99
	got, ok := body.Shape().(Circle)
	if !ok || got.Radius != 2 {
		t.Fatalf("body retained mutable shape definition: %#v", body.Shape())
	}
}

func TestCoreBodyNondynamicMassValidation(t *testing.T) {
	for _, kind := range []BodyType{Static, Kinematic} {
		for _, mass := range []float64{-1, math.NaN(), math.Inf(1)} {
			def := BodyDef{Type: kind, Shape: Circle{Radius: 1}, Mass: mass}
			if _, err := newBody(def); err == nil {
				t.Fatalf("body type %v accepted invalid mass %v", kind, mass)
			}
		}
		for _, mass := range []float64{0, 12} {
			body := coreBody(t, BodyDef{Type: kind, Shape: Circle{Radius: 1}, Mass: mass})
			coreClose(t, body.Mass(), 0, 0)
			coreClose(t, body.Inertia(), 0, 0)
		}
	}
}

func TestCoreBodyAccumulationOverflowIsAtomic(t *testing.T) {
	maximum := math.MaxFloat64
	tests := []struct {
		name      string
		operation func(*Body) error
	}{
		{"force", func(b *Body) error { return b.ApplyForce(Vec2{X: maximum, Y: 0}) }},
		{"force at point", func(b *Body) error { return b.ApplyForceAtPoint(Vec2{X: maximum, Y: 0}, Vec2{}) }},
		{"torque", func(b *Body) error { return b.ApplyTorque(maximum) }},
		{"torque at point", func(b *Body) error { return b.ApplyForceAtPoint(Vec2{X: 0, Y: maximum}, Vec2{X: 1, Y: 0}) }},
		{"impulse", func(b *Body) error { return b.ApplyImpulse(Vec2{X: maximum, Y: 0}, Vec2{}) }},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			body := coreBody(t, coreDynamicDef())
			body.force = Vec2{X: maximum, Y: 3}
			body.torque = maximum
			body.Velocity = Vec2{X: maximum, Y: 4}
			body.AngularVelocity = 0.5
			if err := test.operation(body); err == nil {
				t.Fatal("operation accepted overflowing accumulation")
			}
			coreVecClose(t, body.force, Vec2{X: maximum, Y: 3}, 0)
			coreClose(t, body.torque, maximum, 0)
			coreVecClose(t, body.Velocity, Vec2{X: maximum, Y: 4}, 0)
			coreClose(t, body.AngularVelocity, 0.5, 0)
		})
	}
}

func TestCoreBodyInvalidOperationsAreAtomic(t *testing.T) {
	operations := map[string]func(*Body) error{
		"nan force":       func(b *Body) error { return b.ApplyForce(Vec2{X: math.NaN(), Y: 0}) },
		"infinite force":  func(b *Body) error { return b.ApplyForce(Vec2{X: 0, Y: math.Inf(1)}) },
		"nan force point": func(b *Body) error { return b.ApplyForceAtPoint(Vec2{X: 1, Y: 0}, Vec2{X: math.NaN(), Y: 0}) },
		"force torque overflow": func(b *Body) error {
			return b.ApplyForceAtPoint(Vec2{X: math.MaxFloat64, Y: 0}, Vec2{X: 0, Y: math.MaxFloat64})
		},
		"nan torque":             func(b *Body) error { return b.ApplyTorque(math.NaN()) },
		"infinite torque":        func(b *Body) error { return b.ApplyTorque(math.Inf(1)) },
		"nan impulse":            func(b *Body) error { return b.ApplyImpulse(Vec2{X: 0, Y: math.NaN()}, Vec2{}) },
		"infinite impulse point": func(b *Body) error { return b.ApplyImpulse(Vec2{X: 1, Y: 0}, Vec2{X: math.Inf(1), Y: 0}) },
		"impulse torque overflow": func(b *Body) error {
			return b.ApplyImpulse(Vec2{X: math.MaxFloat64, Y: 0}, Vec2{X: 0, Y: math.MaxFloat64})
		},
	}
	for name, operation := range operations {
		t.Run(name, func(t *testing.T) {
			body := coreBody(t, coreDynamicDef())
			body.Velocity = Vec2{X: 3, Y: 4}
			body.AngularVelocity = 0.25
			body.force = Vec2{X: 5, Y: 6}
			body.torque = 7
			if err := operation(body); err == nil {
				t.Fatal("invalid operation succeeded")
			}
			coreVecClose(t, body.Velocity, Vec2{X: 3, Y: 4}, 0)
			coreClose(t, body.AngularVelocity, 0.25, 0)
			coreVecClose(t, body.force, Vec2{X: 5, Y: 6}, 0)
			coreClose(t, body.torque, 7, 0)
		})
	}
}

func TestCoreBodyStateValidation(t *testing.T) {
	body := coreBody(t, coreDynamicDef())
	if !body.validState() {
		t.Fatal("new body has invalid state")
	}
	body.Position.X = math.NaN()
	if body.validState() {
		t.Fatal("NaN position accepted")
	}
	body.Position = Vec2{}
	body.AngularVelocity = math.Inf(1)
	if body.validState() {
		t.Fatal("infinite angular velocity accepted")
	}
}

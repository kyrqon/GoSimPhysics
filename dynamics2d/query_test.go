package dynamics2d

import (
	"math"
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
	result, hit, err := TimeOfImpact(a, b, 1)
	if err != nil || !hit || result.Time != 0 {
		t.Fatalf("initial touching must be observable even when separating: %+v %v %v", result, hit, err)
	}
	a.Position.X = -3
	// Static public velocity edits do not turn a fixed shape into a moving target.
	b.Velocity, b.AngularVelocity = Vec2{X: -100}, 100
	if _, hit, err := TimeOfImpact(a, b, 1); err != nil || hit {
		t.Fatalf("static velocity was used: %v, %v", hit, err)
	}
	rod := toiTestBody(t, BodyDef{Type: Dynamic, Shape: Box{HalfExtents: Vec2{X: 2, Y: 0.02}}, AngularVelocity: math.Pi / 2, FixedRotation: true})
	rod.AngularVelocity = math.Pi / 2
	stationary := toiTestBody(t, BodyDef{Shape: Circle{Radius: 0.05}, Position: Vec2{X: 1.4, Y: 1.4}})
	if _, hit, err := TimeOfImpact(rod, stationary, 1); err != nil || hit {
		t.Fatalf("FixedRotation was ignored: %v, %v", hit, err)
	}
}

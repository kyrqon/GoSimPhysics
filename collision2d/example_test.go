package collision2d_test

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/collision2d"
)

func ExampleCollide() {
	a := collision2d.Collider{Shape: collision2d.Circle{Radius: 1}}
	b := collision2d.Collider{Shape: collision2d.Circle{Radius: 1}, Position: collision2d.Vec2{X: 1.5}}
	contact, hit, err := collision2d.CollideChecked(a, b)
	if err != nil {
		panic(err)
	}
	fmt.Printf("touching: %v, penetration: %.1f\n", hit, contact.Penetration)
	// Output: touching: true, penetration: 0.5
}

func ExampleTimeOfImpact() {
	bullet := collision2d.Motion{
		Collider: collision2d.Collider{Shape: collision2d.Circle{Radius: 0.05}, Position: collision2d.Vec2{X: -2}},
		Velocity: collision2d.Vec2{X: 400},
	}
	wall := collision2d.Motion{Collider: collision2d.Collider{Shape: collision2d.Box{HalfExtents: collision2d.Vec2{X: 0.01, Y: 1}}}}
	impact, hit, err := collision2d.TimeOfImpact(bullet, wall, 0.01)
	if err != nil || !hit {
		panic(fmt.Sprintf("expected contact: %v", err))
	}
	pose, err := bullet.At(impact.Time)
	if err != nil {
		panic(err)
	}
	fmt.Printf("first contact: %.5f s, x=%.2f\n", impact.Time, pose.Position.X)
	// Output: first contact: 0.00485 s, x=-0.06
}

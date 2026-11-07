package physics2d_test

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/physics2d"
)

func ExampleCollide() {
	a := physics2d.Collider{Shape: physics2d.Circle{Radius: 1}}
	b := physics2d.Collider{Shape: physics2d.Circle{Radius: 1}, Position: physics2d.Vec2{X: 1.5}}
	contact, hit, err := physics2d.CollideChecked(a, b)
	if err != nil {
		panic(err)
	}
	fmt.Printf("touching: %v, penetration: %.1f\n", hit, contact.Penetration)
	// Output: touching: true, penetration: 0.5
}

func ExampleTimeOfImpact() {
	bullet := physics2d.Motion{
		Collider: physics2d.Collider{Shape: physics2d.Circle{Radius: 0.05}, Position: physics2d.Vec2{X: -2}},
		Velocity: physics2d.Vec2{X: 400},
	}
	wall := physics2d.Motion{Collider: physics2d.Collider{Shape: physics2d.Box{HalfExtents: physics2d.Vec2{X: 0.01, Y: 1}}}}
	impact, hit, err := physics2d.TimeOfImpact(bullet, wall, 0.01)
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

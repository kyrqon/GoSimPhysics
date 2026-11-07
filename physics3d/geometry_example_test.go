package physics3d_test

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/physics3d"
)

func ExampleCollide() {
	a := physics3d.Collider{Shape: physics3d.Sphere{Radius: 1}}
	b := physics3d.Collider{Shape: physics3d.Sphere{Radius: 1}, Position: physics3d.Vec3{X: 1.5}}
	contact, hit := physics3d.Collide(a, b)
	fmt.Println(hit, contact.Penetration)
	// Output: true 0.5
}

func ExampleTimeOfImpact() {
	ball := physics3d.Motion{
		Collider: physics3d.Collider{Shape: physics3d.Sphere{Radius: 0.05}, Position: physics3d.Vec3{X: -2}},
		Velocity: physics3d.Vec3{X: 400},
	}
	wall := physics3d.Motion{Collider: physics3d.Collider{
		Shape: physics3d.Box{HalfExtents: physics3d.Vec3{X: 0.01, Y: 1, Z: 1}},
	}}
	impact, hit, err := physics3d.TimeOfImpact(ball, wall, 0.01)
	if err != nil {
		panic(err)
	}
	fmt.Printf("hit=%v time=%.5f\n", hit, impact.Time)
	// Output: hit=true time=0.00485
}

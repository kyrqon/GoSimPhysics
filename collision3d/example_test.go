package collision3d_test

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/collision3d"
)

func ExampleCollide() {
	a := collision3d.Collider{Shape: collision3d.Sphere{Radius: 1}}
	b := collision3d.Collider{Shape: collision3d.Sphere{Radius: 1}, Position: collision3d.Vec3{X: 1.5}}
	contact, hit := collision3d.Collide(a, b)
	fmt.Println(hit, contact.Penetration)
	// Output: true 0.5
}

func ExampleTimeOfImpact() {
	ball := collision3d.Motion{
		Collider: collision3d.Collider{Shape: collision3d.Sphere{Radius: 0.05}, Position: collision3d.Vec3{X: -2}},
		Velocity: collision3d.Vec3{X: 400},
	}
	wall := collision3d.Motion{Collider: collision3d.Collider{
		Shape: collision3d.Box{HalfExtents: collision3d.Vec3{X: 0.01, Y: 1, Z: 1}},
	}}
	impact, hit, err := collision3d.TimeOfImpact(ball, wall, 0.01)
	if err != nil {
		panic(err)
	}
	fmt.Printf("hit=%v time=%.5f\n", hit, impact.Time)
	// Output: hit=true time=0.00485
}

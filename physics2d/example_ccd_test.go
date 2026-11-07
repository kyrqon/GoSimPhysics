package physics2d_test

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/physics2d"
)

func ExampleBody_TimeOfImpact() {
	world := physics2d.NewWorld(physics2d.Vec2{})
	world.Substeps = 1
	wall, err := world.CreateBody(physics2d.BodyDef{
		Shape: physics2d.Box{HalfExtents: physics2d.Vec2{X: 0.01, Y: 1}},
	})
	if err != nil {
		panic(err)
	}
	bullet, err := world.CreateBody(physics2d.BodyDef{
		Type: physics2d.Dynamic, Shape: physics2d.Circle{Radius: 0.05}, Mass: 1,
		Position: physics2d.Vec2{X: -2}, Velocity: physics2d.Vec2{X: 400},
		Continuous: true,
	})
	if err != nil {
		panic(err)
	}
	hit, ok, err := bullet.TimeOfImpact(wall, 0.01)
	if err != nil || !ok {
		panic(fmt.Sprintf("expected wall contact: %v", err))
	}
	fmt.Printf("first contact: %.5f s\n", hit.Time)
	if err := world.Step(0.01); err != nil {
		panic(err)
	}
	fmt.Printf("x=%.2f, velocity=%.0f\n", bullet.Position.X, bullet.Velocity.X)
	// Output:
	// first contact: 0.00485 s
	// x=-0.06, velocity=0
}

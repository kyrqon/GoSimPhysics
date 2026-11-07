package physics_test

import (
	"fmt"

	physics "github.com/kyrqon/GoSimPhysics"
)

func ExampleTimeOfImpact() {
	world := physics.NewWorld(physics.Vec2{})
	world.Substeps = 1
	wall, err := world.CreateBody(physics.BodyDef{
		Shape: physics.Box{HalfExtents: physics.Vec2{X: 0.01, Y: 1}},
	})
	if err != nil {
		panic(err)
	}
	bullet, err := world.CreateBody(physics.BodyDef{
		Type: physics.Dynamic, Shape: physics.Circle{Radius: 0.05}, Mass: 1,
		Position: physics.Vec2{X: -2}, Velocity: physics.Vec2{X: 400},
		Continuous: true,
	})
	if err != nil {
		panic(err)
	}
	hit, ok, err := physics.TimeOfImpact(bullet, wall, 0.01)
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

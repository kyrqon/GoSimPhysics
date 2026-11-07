package dynamics2d_test

import (
	"fmt"

	dynamics2d "github.com/kyrqon/GoSimPhysics/dynamics2d"
)

func ExampleTimeOfImpact() {
	world := dynamics2d.NewWorld(dynamics2d.Vec2{})
	world.Substeps = 1
	wall, err := world.CreateBody(dynamics2d.BodyDef{
		Shape: dynamics2d.Box{HalfExtents: dynamics2d.Vec2{X: 0.01, Y: 1}},
	})
	if err != nil {
		panic(err)
	}
	bullet, err := world.CreateBody(dynamics2d.BodyDef{
		Type: dynamics2d.Dynamic, Shape: dynamics2d.Circle{Radius: 0.05}, Mass: 1,
		Position: dynamics2d.Vec2{X: -2}, Velocity: dynamics2d.Vec2{X: 400},
		Continuous: true,
	})
	if err != nil {
		panic(err)
	}
	hit, ok, err := dynamics2d.TimeOfImpact(bullet, wall, 0.01)
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

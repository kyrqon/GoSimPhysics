package dynamics2d_test

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/dynamics2d"
)

func ExampleBody_ApplyForce() {
	world := dynamics2d.NewWorld(dynamics2d.Vec2{})
	world.Substeps = 1
	body, err := world.CreateBody(dynamics2d.BodyDef{
		Type: dynamics2d.Dynamic, Shape: dynamics2d.Circle{Radius: 0.5}, Mass: 2,
	})
	if err != nil {
		panic(err)
	}
	if err := body.ApplyForce(dynamics2d.Vec2{X: 4}); err != nil {
		panic(err)
	}
	if err := world.Step(0.5); err != nil {
		panic(err)
	}
	// a = F/m = 2, v = a*dt = 1; semi-implicit Euler uses the new velocity.
	fmt.Printf("velocity: %.1f, position: %.1f\n", body.Velocity.X, body.Position.X)
	if err := world.Step(0.5); err != nil {
		panic(err)
	}
	// The force cleared after the previous successful step.
	fmt.Printf("velocity: %.1f, position: %.1f\n", body.Velocity.X, body.Position.X)
	// Output:
	// velocity: 1.0, position: 0.5
	// velocity: 1.0, position: 1.0
}

func ExampleBody_ApplyImpulse() {
	world := dynamics2d.NewWorld(dynamics2d.Vec2{})
	body, err := world.CreateBody(dynamics2d.BodyDef{
		Type: dynamics2d.Dynamic, Shape: dynamics2d.Circle{Radius: 0.5}, Mass: 2,
	})
	if err != nil {
		panic(err)
	}
	if err := body.ApplyImpulse(dynamics2d.Vec2{X: 4}, body.Position); err != nil {
		panic(err)
	}
	// A center impulse gives delta-v = impulse/mass immediately, with no torque.
	fmt.Printf("velocity: %.1f, position: %.1f\n", body.Velocity.X, body.Position.X)
	if err := world.Step(0.5); err != nil {
		panic(err)
	}
	fmt.Printf("velocity: %.1f, position: %.1f\n", body.Velocity.X, body.Position.X)
	// Output:
	// velocity: 2.0, position: 0.0
	// velocity: 2.0, position: 1.0
}

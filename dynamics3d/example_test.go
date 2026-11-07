package dynamics3d_test

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/dynamics3d"
)

func ExampleBody_ApplyForce() {
	world := dynamics3d.NewWorld(dynamics3d.Vec3{})
	world.Substeps = 1
	body, err := world.CreateBody(dynamics3d.BodyDef{Type: dynamics3d.Dynamic, Shape: dynamics3d.Sphere{Radius: 0.5}, Mass: 2})
	if err != nil {
		panic(err)
	}
	if err := body.ApplyForce(dynamics3d.Vec3{Y: 4, Z: 2}); err != nil {
		panic(err)
	}
	if err := world.Step(0.5); err != nil {
		panic(err)
	}
	fmt.Printf("velocity: (%.1f, %.1f, %.1f)\n", body.Velocity.X, body.Velocity.Y, body.Velocity.Z)
	fmt.Printf("position: (%.2f, %.2f, %.2f)\n", body.Position.X, body.Position.Y, body.Position.Z)
	// Output:
	// velocity: (0.0, 1.0, 0.5)
	// position: (0.00, 0.50, 0.25)
}

func ExampleBody_ApplyImpulse() {
	world := dynamics3d.NewWorld(dynamics3d.Vec3{})
	body, err := world.CreateBody(dynamics3d.BodyDef{Type: dynamics3d.Dynamic, Shape: dynamics3d.Sphere{Radius: 1}, Mass: 2})
	if err != nil {
		panic(err)
	}
	if err := body.ApplyImpulse(dynamics3d.Vec3{X: 4}, dynamics3d.Vec3{Y: 1}); err != nil {
		panic(err)
	}
	// I=2mr²/5=0.8; this off-center impulse gives delta-v=(2,0,0)
	// and delta-omega=(0,0,-5).
	fmt.Printf("linear velocity: (%.1f, %.1f, %.1f)\n", body.Velocity.X, body.Velocity.Y, body.Velocity.Z)
	fmt.Printf("angular velocity: (%.1f, %.1f, %.1f)\n", body.AngularVelocity.X, body.AngularVelocity.Y, body.AngularVelocity.Z)
	// Output:
	// linear velocity: (2.0, 0.0, 0.0)
	// angular velocity: (0.0, 0.0, -5.0)
}

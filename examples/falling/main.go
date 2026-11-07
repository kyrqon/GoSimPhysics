// The falling example drops a ball onto a static ground box without a renderer.
package main

import (
	"fmt"
	"log"

	"github.com/kyrqon/GoSimPhysics"
)

func main() {
	if err := run(); err != nil {
		log.Fatal(err)
	}
}

func run() error {
	world := physics.NewWorld(physics.Vec2{Y: -9.81})
	if _, err := world.CreateBody(physics.BodyDef{
		Type:     physics.Static,
		Shape:    physics.Box{HalfExtents: physics.Vec2{X: 10, Y: 0.5}},
		Position: physics.Vec2{Y: -0.5}, // The ground's top surface is y = 0.
		Friction: 0.5,
	}); err != nil {
		return fmt.Errorf("create ground: %w", err)
	}
	ball, err := world.CreateBody(physics.BodyDef{
		Type:        physics.Dynamic,
		Shape:       physics.Circle{Radius: 0.5},
		Position:    physics.Vec2{Y: 5},
		Mass:        1,
		Restitution: 0.35,
		Friction:    0.5,
	})
	if err != nil {
		return fmt.Errorf("create ball: %w", err)
	}

	const (
		hz      = 60
		dt      = 1.0 / hz
		seconds = 4
	)
	fmt.Println("A 1 kg ball falls onto the ground at y = 0 (positive Y is up).")
	fmt.Printf("t=%.1f s  center height=%.3f m  vertical velocity=%.3f m/s\n", 0.0, ball.Position.Y, ball.Velocity.Y)
	for frame := 1; frame <= hz*seconds; frame++ {
		if err := world.Step(dt); err != nil {
			return fmt.Errorf("step %d: %w", frame, err)
		}
		if frame%hz == 0 {
			fmt.Printf("t=%.1f s  center height=%.3f m  vertical velocity=%.3f m/s\n", float64(frame)*dt, ball.Position.Y, ball.Velocity.Y)
		}
	}
	return nil
}

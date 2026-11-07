// The pendulum example constrains a ball to a static anchor with a distance joint.
package main

import (
	"fmt"
	"log"

	"github.com/kyrqon/GoSimPhysics/dynamics2d"
)

func main() {
	if err := run(); err != nil {
		log.Fatal(err)
	}
}

func run() error {
	world := dynamics2d.NewWorld(dynamics2d.Vec2{Y: -9.81})
	anchor, err := world.CreateBody(dynamics2d.BodyDef{
		Type: dynamics2d.Static, Shape: dynamics2d.Circle{Radius: 0.05},
	})
	if err != nil {
		return fmt.Errorf("create anchor: %w", err)
	}
	ball, err := world.CreateBody(dynamics2d.BodyDef{
		Type:     dynamics2d.Dynamic,
		Shape:    dynamics2d.Circle{Radius: 0.1},
		Position: dynamics2d.Vec2{X: 1.2, Y: -1.6},
		Mass:     1,
	})
	if err != nil {
		return fmt.Errorf("create ball: %w", err)
	}
	joint, err := world.CreateDistanceJoint(dynamics2d.DistanceJointDef{
		BodyA: anchor, BodyB: ball,
		Length: 2, // Zero local offsets attach the joint at the body centers.
	})
	if err != nil {
		return fmt.Errorf("create distance joint: %w", err)
	}

	const (
		hz      = 60
		dt      = 1.0 / hz
		seconds = 5
	)
	fmt.Println("A ball swings about a fixed anchor; the joint length is approximate.")
	printState(0, joint, ball)
	for frame := 1; frame <= hz*seconds; frame++ {
		if err := world.Step(dt); err != nil {
			return fmt.Errorf("step %d: %w", frame, err)
		}
		if frame%hz == 0 {
			printState(float64(frame)*dt, joint, ball)
		}
	}
	return nil
}

func printState(seconds float64, joint *dynamics2d.DistanceJoint, ball *dynamics2d.Body) {
	a, b := joint.Anchors()
	length := b.Sub(a).Length()
	fmt.Printf("t=%.1f s  ball=(%.3f, %.3f) m  length=%.6f m  target=%.6f m\n",
		seconds, ball.Position.X, ball.Position.Y, length, joint.Length())
}

// The pendulum example constrains a ball to a static anchor with a distance joint.
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
	anchor, err := world.CreateBody(physics.BodyDef{
		Type: physics.Static, Shape: physics.Circle{Radius: 0.05},
	})
	if err != nil {
		return fmt.Errorf("create anchor: %w", err)
	}
	ball, err := world.CreateBody(physics.BodyDef{
		Type:     physics.Dynamic,
		Shape:    physics.Circle{Radius: 0.1},
		Position: physics.Vec2{X: 1.2, Y: -1.6},
		Mass:     1,
	})
	if err != nil {
		return fmt.Errorf("create ball: %w", err)
	}
	joint, err := world.CreateDistanceJoint(physics.DistanceJointDef{
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

func printState(seconds float64, joint *physics.DistanceJoint, ball *physics.Body) {
	a, b := joint.Anchors()
	length := b.Sub(a).Length()
	fmt.Printf("t=%.1f s  ball=(%.3f, %.3f) m  length=%.6f m  target=%.6f m\n",
		seconds, ball.Position.X, ball.Position.Y, length, joint.Length())
}

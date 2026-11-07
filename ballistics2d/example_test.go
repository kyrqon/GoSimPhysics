package ballistics2d_test

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/ballistics"
	"github.com/kyrqon/GoSimPhysics/ballistics2d"
	"github.com/kyrqon/GoSimPhysics/physics2d"
)

func ExampleProjectile_Step() {
	shot, err := ballistics2d.NewProjectile(ballistics2d.ProjectileDef{
		Position: physics2d.Vec2{X: -1}, Velocity: physics2d.Vec2{X: 10},
		Mass: 2, Damage: 100,
	})
	if err != nil {
		panic(err)
	}
	events, err := shot.Step(1, []ballistics2d.Target{{
		ID: "thin barrier",
		Collider: physics2d.Collider{
			Shape:    physics2d.Box{HalfExtents: physics2d.Vec2{X: 0.5, Y: 1}},
			Position: physics2d.Vec2{X: 0.5},
		},
		Material: ballistics.Material{Name: "game barrier", Resistance: 20},
	}})
	if err != nil {
		panic(err)
	}
	for _, event := range events {
		fmt.Printf("%s: energy %.0f J, damage %.0f\n", event.Kind, event.Energy, event.Damage)
	}
	fmt.Printf("next-hit damage: %.0f\n", shot.State().Damage)
	// Output:
	// hit: energy 100 J, damage 100
	// exit: energy 80 J, damage 0
	// next-hit damage: 80
}

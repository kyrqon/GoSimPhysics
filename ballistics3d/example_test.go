package ballistics3d_test

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/ballistics"
	"github.com/kyrqon/GoSimPhysics/ballistics3d"
	"github.com/kyrqon/GoSimPhysics/collision3d"
	"github.com/kyrqon/GoSimPhysics/math3d"
)

func ExampleProjectile_Step() {
	projectile, err := ballistics3d.NewProjectile(ballistics3d.ProjectileDef{
		Velocity: math3d.Vec3{X: 10}, Mass: 2, Damage: 100,
	})
	if err != nil {
		panic(err)
	}
	events, err := projectile.Step(1, []ballistics3d.Target{{
		ID: "barrier",
		Collider: collision3d.Collider{
			Shape:    collision3d.Box{HalfExtents: math3d.Vec3{X: 0.5, Y: 2, Z: 2}},
			Position: math3d.Vec3{X: 2},
		},
		Material: ballistics.Material{Resistance: 20},
	}})
	if err != nil {
		panic(err)
	}
	for _, event := range events {
		fmt.Printf("%s %s: energy=%.0f, damage=%.0f\n", event.Kind, event.TargetID, event.Energy, event.Damage)
	}
	fmt.Printf("remaining potential damage: %.0f\n", projectile.State().Damage)
	// Output:
	// hit barrier: energy=100, damage=100
	// exit barrier: energy=80, damage=0
	// remaining potential damage: 80
}

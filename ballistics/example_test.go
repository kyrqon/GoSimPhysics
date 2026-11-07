package ballistics_test

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/ballistics"
)

func ExampleAdvance() {
	// This game uses a resistance of 100 energy units per distance unit.
	thin, err := ballistics.Advance(1, 200, 100, 1, 0.5)
	if err != nil {
		panic(err)
	}
	damage, err := ballistics.Damage(100, thin.Energy, 200)
	if err != nil {
		panic(err)
	}
	fmt.Printf("thin target: passed=%t, energy=%.0f, next damage=%.0f\n", thin.Reached && !thin.Stopped, thin.Energy, damage)
	thick, err := ballistics.Advance(1, 200, 100, 1, 3)
	if err != nil {
		panic(err)
	}
	fmt.Printf("thick target: stopped=%t, distance=%.1f\n", thick.Stopped, thick.Distance)
	// Output:
	// thin target: passed=true, energy=150, next damage=75
	// thick target: stopped=true, distance=2.0
}

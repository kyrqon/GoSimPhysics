package ballistics_test

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/ballistics"
)

func ExampleDamage() {
	launchEnergy, err := ballistics.Energy(1, 20)
	if err != nil {
		panic(err)
	}
	// The game may use remaining energy reported by a penetration event.
	material := ballistics.Material{Resistance: 100}
	remainingEnergy := launchEnergy - material.Resistance*0.5
	damage, err := ballistics.Damage(100, remainingEnergy, launchEnergy)
	if err != nil {
		panic(err)
	}
	fmt.Printf("energy=%.0f, next damage=%.0f\n", remainingEnergy, damage)
	// Output: energy=150, next damage=75
}

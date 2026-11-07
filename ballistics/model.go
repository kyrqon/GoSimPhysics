// Package ballistics supplies dimension-independent game rules for projectile
// penetration. Resistance is a tunable energy cost per unit distance; these
// parameters are not calibrated measurements of real materials or ammunition.
package ballistics

import (
	"errors"
	"fmt"
	"math"

	"github.com/kyrqon/GoSimPhysics/internal/numeric"
)

// DefaultMaxEvents bounds the number of hit, exit, and stop events per step.
const DefaultMaxEvents = 64

// ErrEventLimit reports an exhausted event budget. Spatial steppers restore the
// complete projectile state and return no partial event stream on this error.
var ErrEventLimit = errors.New("ballistics: event budget exhausted")

// EventKind distinguishes entering a target, leaving it, and exhausting energy.
type EventKind uint8

const (
	Hit EventKind = iota + 1
	Exit
	Stop
)

func (kind EventKind) String() string {
	switch kind {
	case Hit:
		return "hit"
	case Exit:
		return "exit"
	case Stop:
		return "stop"
	default:
		return "unknown"
	}
}

// Material defines a constant energy loss per distance while inside a target.
// For meter/kilogram/second units, Resistance is joules per meter (newtons).
// Zero resistance is transparent to motion; Name is optional game metadata.
type Material struct {
	Name       string
	Resistance float64
}

// Validate checks the material's numerical parameters.
func (material Material) Validate() error {
	if !numeric.Finite(material.Resistance) || material.Resistance < 0 {
		return fmt.Errorf("ballistics: material resistance must be nonnegative and finite")
	}
	return nil
}

// Energy returns kinetic energy, mass*speed²/2. Mass must be positive and finite;
// speed must be nonnegative and finite. Unrepresentable positive energy is an error.
func Energy(mass, speed float64) (float64, error) {
	if !validMass(mass) || !validNonnegative(speed) {
		return 0, fmt.Errorf("ballistics: positive finite mass and nonnegative finite speed required")
	}
	if speed == 0 {
		return 0, nil
	}
	energy := numeric.ProductRatio([]float64{0.5, mass, speed, speed}, nil)
	if !numeric.Finite(energy) || energy == 0 {
		return 0, fmt.Errorf("ballistics: kinetic energy is unrepresentable")
	}
	return energy, nil
}

// Speed recovers speed from mass and kinetic energy without first forming an
// overflowing energy/mass ratio. Zero energy returns zero speed.
func Speed(mass, energy float64) (float64, error) {
	if !validMass(mass) || !validNonnegative(energy) {
		return 0, fmt.Errorf("ballistics: positive finite mass and nonnegative finite energy required")
	}
	if energy == 0 {
		return 0, nil
	}
	energyFraction, energyExponent := math.Frexp(energy)
	massFraction, massExponent := math.Frexp(mass)
	fraction, exponent := 2*energyFraction/massFraction, energyExponent-massExponent
	if exponent%2 != 0 {
		fraction *= 2
		exponent--
	}
	speed := math.Ldexp(math.Sqrt(fraction), exponent/2)
	if !numeric.Finite(speed) || speed == 0 {
		return 0, fmt.Errorf("ballistics: speed is unrepresentable")
	}
	return speed, nil
}

// Damage scales baseDamage by remaining energy/referenceEnergy, clamped to [0,1].
// The launch energy is a convenient fixed reference. This function returns a
// game damage value; it never changes health or applies damage itself.
func Damage(baseDamage, energy, referenceEnergy float64) (float64, error) {
	if !validNonnegative(baseDamage) || !validNonnegative(energy) || !numeric.Finite(referenceEnergy) || referenceEnergy <= 0 {
		return 0, fmt.Errorf("ballistics: damage and energy must be nonnegative and finite, with positive finite reference energy")
	}
	if baseDamage == 0 || energy == 0 {
		return 0, nil
	}
	if energy >= referenceEnergy {
		return baseDamage, nil
	}
	// A clamped result cannot overflow mathematically. Scaling avoids an
	// underflowing intermediate energy/referenceEnergy losing a valid result.
	damage := numeric.ProductRatio([]float64{baseDamage, energy}, []float64{referenceEnergy})
	return math.Min(baseDamage, damage), nil
}

func validMass(mass float64) bool { return numeric.Finite(mass) && mass > 0 }
func validNonnegative(value float64) bool {
	return numeric.Finite(value) && value >= 0
}

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
	energy := productRatio([]float64{0.5, mass, speed, speed}, nil)
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
	damage := productRatio([]float64{baseDamage, energy}, []float64{referenceEnergy})
	return math.Min(baseDamage, damage), nil
}

// Travel is one straight segment ending at a time limit, material boundary, or
// energy exhaustion. Time is the actual duration consumed, at most the requested
// duration. Reached and Stopped may both be true when stopping at a boundary.
type Travel struct {
	Distance, Time, Energy, Speed float64
	Reached, Stopped              bool
}

// Advance integrates a point projectile under constant opposing resistance.
// It stops at the earliest of dt seconds, maxDistance, or energy exhaustion.
// maxDistance may be +Inf to represent no upcoming material boundary.
//
// Resistance has units of energy/distance, so the exact stopping distance is
// energy/resistance. At a boundary, remaining energy is energy-resistance*distance.
// Finite-time movement uses constant deceleration, rather than instantaneous
// movement through the entire target. The function never mutates caller state.
func Advance(mass, energy, resistance, dt, maxDistance float64) (Travel, error) {
	if !validMass(mass) || !validNonnegative(energy) || !validNonnegative(resistance) || !numeric.Finite(dt) || dt <= 0 ||
		math.IsNaN(maxDistance) || maxDistance < 0 || math.IsInf(maxDistance, -1) {
		return Travel{}, fmt.Errorf("ballistics: invalid advance parameters")
	}
	speed, err := Speed(mass, energy)
	if err != nil {
		return Travel{}, err
	}
	if energy == 0 {
		return Travel{Reached: maxDistance == 0, Stopped: true}, nil
	}
	if maxDistance == 0 {
		return Travel{Energy: energy, Speed: speed, Reached: true}, nil
	}
	if resistance == 0 {
		boundaryTime := maxDistance / speed
		if !math.IsInf(maxDistance, 1) && boundaryTime <= dt {
			return Travel{Distance: maxDistance, Time: boundaryTime, Energy: energy, Speed: speed, Reached: true}, nil
		}
		distance := productRatio([]float64{speed, dt}, nil)
		if !numeric.Finite(distance) {
			return Travel{}, fmt.Errorf("ballistics: travel distance overflow")
		}
		return Travel{Distance: distance, Time: dt, Energy: energy, Speed: speed}, nil
	}

	stoppingDistance := productRatio([]float64{energy}, []float64{resistance})
	stoppingTime := productRatio([]float64{mass, speed}, []float64{resistance})
	stopsFirst := maxDistance >= stoppingDistance
	eventDistance, eventTime := maxDistance, math.Inf(1)
	eventEnergy, eventSpeed := 0.0, 0.0
	if stopsFirst {
		eventDistance, eventTime = stoppingDistance, stoppingTime
	} else {
		work := productRatio([]float64{resistance, maxDistance}, nil)
		// At a floating-point equality with the stopping threshold, treating
		// zero remaining energy as a stop prevents a zero-energy penetration.
		eventEnergy = math.Max(0, energy-work)
		eventSpeed, err = Speed(mass, eventEnergy)
		if err != nil {
			return Travel{}, err
		}
		// 2*s/(v0+v1), arranged to avoid overflowing v0+v1 or 2*s.
		eventTime = (maxDistance / speed) / (0.5 * (1 + eventSpeed/speed))
	}
	if eventTime <= dt {
		if !numeric.Finite(eventDistance) || !numeric.Finite(eventTime) {
			return Travel{}, fmt.Errorf("ballistics: event movement is unrepresentable")
		}
		return Travel{
			Distance: eventDistance, Time: eventTime, Energy: eventEnergy, Speed: eventSpeed,
			Reached: eventDistance == maxDistance, Stopped: eventEnergy == 0,
		}, nil
	}

	// Form dt/stopTime directly: stopTime itself can exceed MaxFloat64 while
	// a finite requested interval still consumes a substantial energy fraction.
	fraction := numeric.Clamp(productRatio([]float64{resistance, dt}, []float64{mass, speed}), 0, 1)
	remainingFactor := 1 - fraction
	outEnergy := energy * remainingFactor * remainingFactor
	if outEnergy == 0 && remainingFactor > 0 {
		return Travel{}, fmt.Errorf("ballistics: remaining energy is unrepresentable")
	}
	outSpeed, err := Speed(mass, outEnergy)
	if err != nil {
		return Travel{}, err
	}
	distance := productRatio([]float64{speed, dt, 1 - fraction*0.5}, nil)
	if !numeric.Finite(distance) {
		return Travel{}, fmt.Errorf("ballistics: travel distance overflow")
	}
	return Travel{Distance: distance, Time: dt, Energy: outEnergy, Speed: outSpeed, Stopped: outEnergy == 0}, nil
}

func validMass(mass float64) bool { return numeric.Finite(mass) && mass > 0 }
func validNonnegative(value float64) bool {
	return numeric.Finite(value) && value >= 0
}

// productRatio evaluates products/quotients without avoidable intermediate
// overflow or underflow. Every denominator supplied by this package is positive.
func productRatio(numerators, denominators []float64) float64 {
	fraction, exponent := 1.0, 0
	for _, value := range numerators {
		if value == 0 {
			return 0
		}
		part, power := math.Frexp(value)
		fraction *= part
		exponent += power
	}
	for _, value := range denominators {
		part, power := math.Frexp(value)
		fraction /= part
		exponent -= power
	}
	return math.Ldexp(fraction, exponent)
}

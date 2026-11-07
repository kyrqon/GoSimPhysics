package penetration

import (
	"fmt"
	"math"

	"github.com/kyrqon/GoSimPhysics/ballistics"
	"github.com/kyrqon/GoSimPhysics/internal/numeric"
)

// travelSegment ends at a time limit, material boundary, or energy exhaustion.
// Time is the actual duration consumed, at most the requested duration. Reached
// and Stopped may both be true when stopping at a boundary.
type travelSegment struct {
	Distance, Time, Energy, Speed float64
	Reached, Stopped              bool
}

// advanceTravel integrates a point projectile under constant opposing resistance.
// It stops at the earliest of dt seconds, maxDistance, or energy exhaustion.
// maxDistance may be +Inf to represent no upcoming material boundary.
//
// Resistance has units of energy/distance, so the exact stopping distance is
// energy/resistance. At a boundary, remaining energy is energy-resistance*distance.
// Finite-time movement uses constant deceleration, rather than instantaneous
// movement through the entire target. The function never mutates caller state.
func advanceTravel(mass, energy, resistance, dt, maxDistance float64) (travelSegment, error) {
	if !numeric.Finite(mass) || mass <= 0 || !numeric.Finite(energy) || energy < 0 ||
		!numeric.Finite(resistance) || resistance < 0 || !numeric.Finite(dt) || dt <= 0 ||
		math.IsNaN(maxDistance) || maxDistance < 0 || math.IsInf(maxDistance, -1) {
		return travelSegment{}, fmt.Errorf("ballistics: invalid advance parameters")
	}
	speed, err := ballistics.Speed(mass, energy)
	if err != nil {
		return travelSegment{}, err
	}
	if energy == 0 {
		return travelSegment{Reached: maxDistance == 0, Stopped: true}, nil
	}
	if maxDistance == 0 {
		return travelSegment{Energy: energy, Speed: speed, Reached: true}, nil
	}
	if resistance == 0 {
		boundaryTime := maxDistance / speed
		if !math.IsInf(maxDistance, 1) && boundaryTime <= dt {
			return travelSegment{Distance: maxDistance, Time: boundaryTime, Energy: energy, Speed: speed, Reached: true}, nil
		}
		distance := numeric.ProductRatio([]float64{speed, dt}, nil)
		if !numeric.Finite(distance) {
			return travelSegment{}, fmt.Errorf("ballistics: travel distance overflow")
		}
		return travelSegment{Distance: distance, Time: dt, Energy: energy, Speed: speed}, nil
	}

	stoppingDistance := numeric.ProductRatio([]float64{energy}, []float64{resistance})
	stoppingTime := numeric.ProductRatio([]float64{mass, speed}, []float64{resistance})
	stopsFirst := maxDistance >= stoppingDistance
	eventDistance, eventTime := maxDistance, math.Inf(1)
	eventEnergy, eventSpeed := 0.0, 0.0
	if stopsFirst {
		eventDistance, eventTime = stoppingDistance, stoppingTime
	} else {
		work := numeric.ProductRatio([]float64{resistance, maxDistance}, nil)
		// At a floating-point equality with the stopping threshold, treating
		// zero remaining energy as a stop prevents a zero-energy penetration.
		eventEnergy = math.Max(0, energy-work)
		eventSpeed, err = ballistics.Speed(mass, eventEnergy)
		if err != nil {
			return travelSegment{}, err
		}
		// 2*s/(v0+v1), arranged to avoid overflowing v0+v1 or 2*s.
		eventTime = (maxDistance / speed) / (0.5 * (1 + eventSpeed/speed))
	}
	if eventTime <= dt {
		if !numeric.Finite(eventDistance) || !numeric.Finite(eventTime) {
			return travelSegment{}, fmt.Errorf("ballistics: event movement is unrepresentable")
		}
		return travelSegment{
			Distance: eventDistance, Time: eventTime, Energy: eventEnergy, Speed: eventSpeed,
			Reached: eventDistance == maxDistance, Stopped: eventEnergy == 0,
		}, nil
	}

	// Form dt/stopTime directly: stopTime itself can exceed MaxFloat64 while
	// a finite requested interval still consumes a substantial energy fraction.
	fraction := numeric.Clamp(numeric.ProductRatio([]float64{resistance, dt}, []float64{mass, speed}), 0, 1)
	remainingFactor := 1 - fraction
	outEnergy := energy * remainingFactor * remainingFactor
	if outEnergy == 0 && remainingFactor > 0 {
		return travelSegment{}, fmt.Errorf("ballistics: remaining energy is unrepresentable")
	}
	outSpeed, err := ballistics.Speed(mass, outEnergy)
	if err != nil {
		return travelSegment{}, err
	}
	distance := numeric.ProductRatio([]float64{speed, dt, 1 - fraction*0.5}, nil)
	if !numeric.Finite(distance) {
		return travelSegment{}, fmt.Errorf("ballistics: travel distance overflow")
	}
	return travelSegment{Distance: distance, Time: dt, Energy: outEnergy, Speed: outSpeed, Stopped: outEnergy == 0}, nil
}

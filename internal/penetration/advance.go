// Package penetration owns the scalar boundary/event integration shared by the
// 2D and 3D point-projectile adapters. Distances are measured from launch, so
// crossing a frame boundary cannot change a previously computed material root.
package penetration

import (
	"fmt"
	"math"
	"sort"

	"github.com/kyrqon/GoSimPhysics/ballistics"
	"github.com/kyrqon/GoSimPhysics/internal/numeric"
)

// Interval is a complete material chord along the immutable launch ray.
// Entry and Exit are cumulative distances from launch, not from this frame.
type Interval struct {
	ID                    string
	Material              ballistics.Material
	Entry, Exit           float64
	StartsInside, Tangent bool
}

// Record persists an entry and that target's work across frame boundaries.
type Record struct {
	Material                 ballistics.Material
	EntryEnergy, EnergySpent float64
	StartedInside            bool
}

// State holds the scalar part of one projectile. Callers must not mutate Inside
// concurrently. Advance always makes an independent map before applying changes.
type State struct {
	Mass, BaseDamage, InitialEnergy, Energy, Time, Distance float64
	Stopped                                                 bool
	Inside                                                  map[string]Record
}

// Event carries scalar event data. Offset is measured from the start of this
// call; Distance and Time are cumulative from launch.
type Event struct {
	Kind                                                             ballistics.EventKind
	TargetID                                                         string
	Material                                                         ballistics.Material
	Time, Distance, Offset, Energy, EntryEnergy, EnergySpent, Damage float64
	StartedInside                                                    bool
	ActiveTargets                                                    []string
}

// NewState initializes a positive-energy launch.
func NewState(mass, speed, damage float64) (State, error) {
	energy, err := ballistics.Energy(mass, speed)
	if err != nil {
		return State{}, err
	}
	if energy <= 0 || !numeric.Finite(damage) || damage < 0 {
		return State{}, fmt.Errorf("ballistics: positive launch energy and nonnegative finite damage required")
	}
	return State{Mass: mass, BaseDamage: damage, InitialEnergy: energy, Energy: energy, Inside: make(map[string]Record)}, nil
}

type boundary struct {
	distance float64
	id       string
	enter    bool
	material ballistics.Material
}

// Advance integrates the earliest material boundary, frame end, or stop. It
// returns no partial events or state on error, and never modifies before.Inside.
func Advance(before State, intervals []Interval, dt float64, maxEvents int) (State, float64, []Event, error) {
	if err := validState(before); err != nil {
		return State{}, 0, nil, err
	}
	if !numeric.Finite(dt) || dt <= 0 || maxEvents < 1 || !numeric.Finite(before.Time+dt) {
		return State{}, 0, nil, fmt.Errorf("ballistics: positive finite duration, age, and event budget required")
	}
	ordered := append([]Interval(nil), intervals...)
	sort.Slice(ordered, func(i, j int) bool { return ordered[i].ID < ordered[j].ID })
	for i, interval := range ordered {
		if interval.ID == "" || i > 0 && interval.ID == ordered[i-1].ID ||
			!numeric.Finite(interval.Entry) || !numeric.Finite(interval.Exit) || interval.Entry < 0 || interval.Exit < interval.Entry {
			return State{}, 0, nil, fmt.Errorf("ballistics: invalid or duplicate material interval")
		}
		if err := interval.Material.Validate(); err != nil {
			return State{}, 0, nil, err
		}
	}
	after := before
	after.Inside = make(map[string]Record, len(before.Inside))
	for id, record := range before.Inside {
		after.Inside[id] = record
	}
	after.Time = before.Time + dt
	if before.Stopped {
		return after, 0, nil, nil
	}

	var events []Event
	elapsed := 0.0
	push := func(kind ballistics.EventKind, id string, record Record) error {
		if len(events) >= maxEvents {
			return ballistics.ErrEventLimit
		}
		event := Event{
			Kind: kind, TargetID: id, Material: record.Material,
			Time: before.Time + elapsed, Distance: after.Distance,
			Offset: after.Distance - before.Distance, Energy: after.Energy,
			EntryEnergy: record.EntryEnergy, EnergySpent: record.EnergySpent,
			StartedInside: record.StartedInside,
		}
		if kind == ballistics.Hit {
			var err error
			event.Damage, err = ballistics.Damage(after.BaseDamage, after.Energy, after.InitialEnergy)
			if err != nil {
				return err
			}
		}
		if kind == ballistics.Stop {
			event.ActiveTargets = activeIDs(after.Inside)
		}
		events = append(events, event)
		return nil
	}

	// Geometry may be resnapshotted between calls. Remove departed materials
	// before adding current entries, without repeating damage for retained ones.
	current := make(map[string]Interval)
	var boundaries []boundary
	for _, interval := range ordered {
		if interval.Tangent || interval.Exit <= interval.Entry || interval.Exit <= before.Distance {
			continue
		}
		if interval.Entry <= before.Distance {
			current[interval.ID] = interval
		} else {
			boundaries = append(boundaries, boundary{interval.Entry, interval.ID, true, interval.Material})
		}
		boundaries = append(boundaries, boundary{interval.Exit, interval.ID, false, interval.Material})
	}
	for _, id := range activeIDs(after.Inside) {
		if _, remains := current[id]; !remains {
			if err := push(ballistics.Exit, id, after.Inside[id]); err != nil {
				return State{}, 0, nil, err
			}
			delete(after.Inside, id)
		}
	}
	for _, interval := range ordered {
		if _, active := current[interval.ID]; !active {
			continue
		}
		record, retained := after.Inside[interval.ID]
		if !retained {
			record = Record{EntryEnergy: after.Energy, StartedInside: interval.StartsInside || interval.Entry < before.Distance}
		}
		record.Material = interval.Material
		after.Inside[interval.ID] = record
		if !retained {
			if err := push(ballistics.Hit, interval.ID, record); err != nil {
				return State{}, 0, nil, err
			}
		}
	}
	sort.Slice(boundaries, func(i, j int) bool {
		if boundaries[i].distance != boundaries[j].distance {
			return boundaries[i].distance < boundaries[j].distance
		}
		if boundaries[i].enter != boundaries[j].enter {
			return !boundaries[i].enter // Exit before Hit at a shared face.
		}
		return boundaries[i].id < boundaries[j].id
	})

	index := 0
	for {
		remaining := dt - elapsed
		if remaining <= 0 {
			break
		}
		next := math.Inf(1)
		if index < len(boundaries) {
			next = boundaries[index].distance
		}
		gap := next - after.Distance
		if math.IsNaN(gap) || gap < 0 {
			return State{}, 0, nil, fmt.Errorf("ballistics: invalid boundary order")
		}
		resistance := 0.0
		for _, id := range activeIDs(after.Inside) {
			resistance += after.Inside[id].Material.Resistance
		}
		if !numeric.Finite(resistance) {
			return State{}, 0, nil, fmt.Errorf("ballistics: overlapping resistance overflow")
		}
		travel, err := ballistics.Advance(after.Mass, after.Energy, resistance, remaining, gap)
		if err != nil {
			return State{}, 0, nil, err
		}
		for id, record := range after.Inside {
			record.EnergySpent += record.Material.Resistance * travel.Distance
			if !numeric.Finite(record.EnergySpent) {
				return State{}, 0, nil, fmt.Errorf("ballistics: deposited energy overflow")
			}
			after.Inside[id] = record
		}
		after.Distance += travel.Distance
		if travel.Reached {
			// Reuse the original root exactly, rather than accumulating rounding
			// at a material boundary and re-entering it on the next frame.
			after.Distance = next
		}
		after.Energy = travel.Energy
		elapsed += travel.Time
		if !numeric.Finite(after.Distance) || !numeric.Finite(elapsed) {
			return State{}, 0, nil, fmt.Errorf("ballistics: cumulative travel overflow")
		}
		if travel.Stopped {
			after.Stopped = true
			ids := activeIDs(after.Inside)
			id, record := "", Record{}
			if len(ids) != 0 {
				id, record = ids[0], after.Inside[ids[0]]
			}
			if err := push(ballistics.Stop, id, record); err != nil {
				return State{}, 0, nil, err
			}
			break // A zero-energy boundary stop is not an Exit or a new Hit.
		}
		if !travel.Reached {
			break
		}
		for index < len(boundaries) && boundaries[index].distance == next {
			boundary := boundaries[index]
			if boundary.enter {
				record := Record{Material: boundary.material, EntryEnergy: after.Energy}
				after.Inside[boundary.id] = record
				if err := push(ballistics.Hit, boundary.id, record); err != nil {
					return State{}, 0, nil, err
				}
			} else {
				record, active := after.Inside[boundary.id]
				if active {
					if err := push(ballistics.Exit, boundary.id, record); err != nil {
						return State{}, 0, nil, err
					}
					delete(after.Inside, boundary.id)
				}
			}
			index++
		}
	}
	return after, after.Distance - before.Distance, events, nil
}

func validState(state State) error {
	if !numeric.Finite(state.Mass) || state.Mass <= 0 || !numeric.Finite(state.BaseDamage) || state.BaseDamage < 0 ||
		!numeric.Finite(state.InitialEnergy) || state.InitialEnergy <= 0 || !numeric.Finite(state.Energy) || state.Energy < 0 ||
		!numeric.Finite(state.Time) || state.Time < 0 || !numeric.Finite(state.Distance) || state.Distance < 0 ||
		state.Stopped != (state.Energy == 0) {
		return fmt.Errorf("ballistics: invalid projectile state")
	}
	for id, record := range state.Inside {
		if id == "" || record.Material.Validate() != nil || !numeric.Finite(record.EntryEnergy) || record.EntryEnergy < 0 ||
			!numeric.Finite(record.EnergySpent) || record.EnergySpent < 0 {
			return fmt.Errorf("ballistics: invalid material history")
		}
	}
	return nil
}

func activeIDs(records map[string]Record) []string {
	ids := make([]string, 0, len(records))
	for id := range records {
		ids = append(ids, id)
	}
	sort.Strings(ids)
	return ids
}

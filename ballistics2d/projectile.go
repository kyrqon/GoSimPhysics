// Package ballistics2d models game projectile penetration through frozen 2D
// colliders. Projectiles are point masses following straight paths; material
// resistance removes a configurable amount of kinetic energy per unit distance.
// There is no gravity, finite projectile radius, ricochet, or rigid-body impulse
// response in this layer. Hit damage is returned to the host game, not applied
// to health automatically.
package ballistics2d

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/ballistics"
	"github.com/kyrqon/GoSimPhysics/collision2d"
	"github.com/kyrqon/GoSimPhysics/internal/numeric"
	"github.com/kyrqon/GoSimPhysics/internal/penetration"
	"github.com/kyrqon/GoSimPhysics/math2d"
)

// EventKind identifies a hit, exit, or final stop.
type EventKind = ballistics.EventKind

const (
	Hit  = ballistics.Hit
	Exit = ballistics.Exit
	Stop = ballistics.Stop

	DefaultMaxEvents = ballistics.DefaultMaxEvents
)

// ErrEventLimit means a step exceeded its event budget. No projectile state or
// events are committed on failure. It is shared by both spatial packages.
var ErrEventLimit = ballistics.ErrEventLimit

// Target is a solid collider and its game-tuned material. IDs must be nonempty
// and unique in each Step. Targets are frozen during a step, and their geometry
// is copied before tracing. Reusing an ID preserves entry tracking across steps.
type Target struct {
	ID       string
	Collider collision2d.Collider
	Material ballistics.Material
}

// ProjectileDef configures a new point projectile. Mass and launch speed must be
// positive and finite, and Damage must be nonnegative and finite. MaxEvents zero
// selects DefaultMaxEvents. Coordinates, mass, velocity, and material resistance
// must use consistent units, for example meters, kilograms, seconds, and J/m.
type ProjectileDef struct {
	Position, Velocity math2d.Vec2
	Mass, Damage       float64
	MaxEvents          int
}

// State is an independent snapshot of a projectile. Damage is the damage a new
// hit would receive at the current energy, capped at the launch damage. Time is
// simulated age; Distance is total path length since creation.
type State struct {
	Position, Velocity                                  math2d.Vec2
	Mass, Energy, InitialEnergy, Damage, Time, Distance float64
	Stopped                                             bool
}

// Event reports a material boundary or stopping point. Time and Distance are
// measured from projectile creation. Damage is nonzero only on Hit, once per
// entry; Exit and Stop do not cause a second health deduction. EnergySpent is
// the work done by this target's resistance, including previous steps. A Stop
// lists all active targets when overlapping materials jointly stopped the shot.
// A hit that begins inside has StartedInside set and a zero entry normal.
type Event struct {
	Kind                                                     ballistics.EventKind
	TargetID                                                 string
	Material                                                 ballistics.Material
	Time, Distance, Energy, EntryEnergy, EnergySpent, Damage float64
	Position, Normal                                         math2d.Vec2
	StartedInside                                            bool
	ActiveTargets                                            []string
}

// Projectile owns its flight and material-entry state. Use State to inspect it;
// editing a returned snapshot does not change the projectile. A projectile must
// not be copied or accessed concurrently with Step.
type Projectile struct {
	// MaxEvents bounds hit, exit, and stop events in each Step. It must be positive.
	MaxEvents int

	origin, position, velocity, direction math2d.Vec2
	state                                 penetration.State
}

// NewProjectile validates a definition and creates a straight-flight projectile.
func NewProjectile(def ProjectileDef) (*Projectile, error) {
	if !validVec(def.Position) || !validVec(def.Velocity) {
		return nil, fmt.Errorf("ballistics2d: position and velocity must be finite")
	}
	speed := def.Velocity.Length()
	if !numeric.Finite(speed) || speed <= 0 {
		return nil, fmt.Errorf("ballistics2d: launch speed must be positive and finite")
	}
	if def.MaxEvents < 0 {
		return nil, fmt.Errorf("ballistics2d: event budget must be positive or zero for the default")
	}
	state, err := penetration.NewState(def.Mass, speed, def.Damage)
	if err != nil {
		return nil, err
	}
	budget := def.MaxEvents
	if budget == 0 {
		budget = DefaultMaxEvents
	}
	return &Projectile{
		MaxEvents: budget, origin: def.Position, position: def.Position, velocity: def.Velocity,
		direction: def.Velocity.Normalized(), state: state,
	}, nil
}

// State returns a value snapshot without exposing the projectile's entry map.
func (p *Projectile) State() State {
	if p == nil {
		return State{}
	}
	damage, _ := ballistics.Damage(p.state.BaseDamage, p.state.Energy, p.state.InitialEnergy)
	return State{
		Position: p.position, Velocity: p.velocity, Mass: p.state.Mass,
		Energy: p.state.Energy, InitialEnergy: p.state.InitialEnergy, Damage: damage,
		Time: p.state.Time, Distance: p.state.Distance, Stopped: p.state.Stopped,
	}
}

// Step advances dt seconds against the supplied frozen target snapshots. Material
// resistances add in overlapping solids; only time spent inside consumes energy.
// Entries remain tracked across frames, so splitting a step does not duplicate
// damage. A target removed between steps ends its active material interval.
// Invalid input, numerical failure, or budget exhaustion leaves the complete
// projectile unchanged and returns no events. A stopped projectile remains fixed
// and produces no further events, while successful steps still advance its age.
func (p *Projectile) Step(dt float64, targets []Target) ([]Event, error) {
	if p == nil {
		return nil, fmt.Errorf("ballistics2d: nil projectile")
	}
	if !numeric.Finite(dt) || dt <= 0 || p.MaxEvents < 1 {
		return nil, fmt.Errorf("ballistics2d: positive finite dt and positive event budget required")
	}
	canonical := make([]Target, len(targets))
	seen := make(map[string]bool, len(targets))
	for i, target := range targets {
		if target.ID == "" || seen[target.ID] {
			return nil, fmt.Errorf("ballistics2d: target IDs must be nonempty and unique")
		}
		seen[target.ID] = true
		if err := target.Material.Validate(); err != nil {
			return nil, err
		}
		shape, ok := collision2d.CloneShape(target.Collider.Shape)
		if !ok {
			return nil, fmt.Errorf("ballistics2d: target %q has an invalid shape", target.ID)
		}
		target.Collider.Shape = shape
		if _, _, err := collision2d.Bounds(target.Collider); err != nil {
			return nil, fmt.Errorf("ballistics2d: target %q: %w", target.ID, err)
		}
		canonical[i] = target
	}

	intervals := make([]penetration.Interval, 0, len(targets))
	geometry := make(map[string]collision2d.RayHit, len(targets))
	if !p.state.Stopped {
		for _, target := range canonical {
			// Recast from the immutable launch point so exact boundary distances
			// remain stable across frame partitions, including rotated walls.
			hit, ok, err := collision2d.RayInterval(target.Collider, p.origin, p.direction)
			if err != nil {
				return nil, fmt.Errorf("ballistics2d: target %q: %w", target.ID, err)
			}
			if !ok || hit.ExitDistance < p.state.Distance {
				continue
			}
			geometry[target.ID] = hit
			intervals = append(intervals, penetration.Interval{
				ID: target.ID, Material: target.Material,
				Entry: hit.EntryDistance, Exit: hit.ExitDistance,
				StartsInside: hit.StartsInside || hit.EntryDistance < p.state.Distance && p.state.Distance < hit.ExitDistance,
				Tangent:      hit.Tangent,
			})
		}
	}
	after, _, scalarEvents, err := penetration.Advance(p.state, intervals, dt, p.MaxEvents)
	if err != nil {
		return nil, err
	}
	position := p.origin.Add(p.direction.Mul(after.Distance))
	speed, err := ballistics.Speed(after.Mass, after.Energy)
	if err != nil {
		return nil, err
	}
	velocity := p.direction.Mul(speed)
	if !validVec(position) || !validVec(velocity) {
		return nil, fmt.Errorf("ballistics2d: flight coordinate overflow")
	}
	events := make([]Event, 0, len(scalarEvents))
	for _, scalar := range scalarEvents {
		point := p.origin.Add(p.direction.Mul(scalar.Distance))
		if !validVec(point) {
			return nil, fmt.Errorf("ballistics2d: event coordinate overflow")
		}
		normal := math2d.Vec2{}
		if hit, found := geometry[scalar.TargetID]; found {
			switch scalar.Kind {
			case ballistics.Hit:
				if !scalar.StartedInside {
					normal = hit.EntryNormal
				}
			case ballistics.Exit:
				normal = hit.ExitNormal
			}
		}
		events = append(events, Event{
			Kind: scalar.Kind, TargetID: scalar.TargetID, Material: scalar.Material,
			Time: scalar.Time, Distance: scalar.Distance, Position: point, Normal: normal,
			Energy: scalar.Energy, EntryEnergy: scalar.EntryEnergy, EnergySpent: scalar.EnergySpent,
			Damage: scalar.Damage, StartedInside: scalar.StartedInside,
			ActiveTargets: append([]string(nil), scalar.ActiveTargets...),
		})
	}
	p.state, p.position, p.velocity = after, position, velocity
	return events, nil
}

func validVec(v math2d.Vec2) bool { return numeric.Finite(v.X) && numeric.Finite(v.Y) }

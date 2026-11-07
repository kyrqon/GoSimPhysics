// Package ballistics3d advances point projectiles through frozen 3D colliders.
// Material resistance consumes energy per unit path length. The package reports
// gameplay events; it does not own health, destroy targets, or apply rigid-body
// impulses. This initial model has straight paths, no projectile radius, gravity,
// ricochet, or continuously moving target geometry.
package ballistics3d

import (
	"fmt"

	"github.com/kyrqon/GoSimPhysics/ballistics"
	"github.com/kyrqon/GoSimPhysics/collision3d"
	"github.com/kyrqon/GoSimPhysics/internal/numeric"
	"github.com/kyrqon/GoSimPhysics/internal/penetration"
	"github.com/kyrqon/GoSimPhysics/math3d"
)

// Target places a material in a closed sphere or oriented box. IDs must be
// nonempty and unique within a Step. Colliders are frozen during each call;
// callers may provide a new snapshot of their scene on the following step.
type Target struct {
	ID       string
	Collider collision3d.Collider
	Material ballistics.Material
}

// ProjectileDef describes a launch. Mass, launch speed, and its derived energy
// must be positive and finite. Damage is the nonnegative damage at launch energy.
// MaxEvents zero selects ballistics.DefaultMaxEvents.
type ProjectileDef struct {
	Position, Velocity math3d.Vec3
	Mass, Damage       float64
	MaxEvents          int
}

// State is a value snapshot of a projectile. Damage is its current potential hit
// damage, scaled by remaining energy. Time is its simulated age in seconds, and
// Distance is cumulative travel, both measured from launch.
type State struct {
	Position, Velocity                                  math3d.Vec3
	Mass, Energy, InitialEnergy, Damage, Time, Distance float64
	Stopped                                             bool
}

// Event describes a material crossing or stop. Hit carries damage once on entry;
// Exit and Stop carry zero damage. Position is the actual point on the current
// path, Normal is the outward surface normal for Hit/Exit, and Stop has zero
// normal. A launch inside a solid has a zero entry normal and StartedInside true.
// EnergySpent is work done by the event's target, including across prior steps;
// ActiveTargets identifies every material containing a stopped projectile.
type Event struct {
	Kind                                                     ballistics.EventKind
	TargetID                                                 string
	Material                                                 ballistics.Material
	Time, Distance, Energy, EntryEnergy, EnergySpent, Damage float64
	Position, Normal                                         math3d.Vec3
	StartedInside                                            bool
	ActiveTargets                                            []string
}

// Projectile stores a straight point trajectory and material occupancy across
// steps. Access it from one goroutine at a time. MaxEvents may be adjusted between
// calls; exhausting it returns ballistics.ErrEventLimit and rolls back the call.
type Projectile struct {
	MaxEvents int

	origin, position, velocity, direction math3d.Vec3
	core                                  penetration.State
	damage                                float64
}

// NewProjectile validates a launch and creates independent mutable state.
func NewProjectile(def ProjectileDef) (*Projectile, error) {
	if !validVec(def.Position) || !validVec(def.Velocity) {
		return nil, fmt.Errorf("ballistics3d: launch vectors must be finite")
	}
	speed := def.Velocity.Length()
	if !numeric.Finite(speed) || speed <= 0 {
		return nil, fmt.Errorf("ballistics3d: launch speed must be positive and finite")
	}
	if def.MaxEvents < 0 {
		return nil, fmt.Errorf("ballistics3d: event budget must not be negative")
	}
	core, err := penetration.NewState(def.Mass, speed, def.Damage)
	if err != nil {
		return nil, err
	}
	budget := def.MaxEvents
	if budget == 0 {
		budget = ballistics.DefaultMaxEvents
	}
	return &Projectile{
		MaxEvents: budget, origin: def.Position, position: def.Position, velocity: def.Velocity,
		direction: def.Velocity.Normalized(), core: core, damage: def.Damage,
	}, nil
}

// State returns a value copy, so editing it cannot move the projectile.
func (p *Projectile) State() State {
	if p == nil {
		return State{}
	}
	return State{
		Position: p.position, Velocity: p.velocity, Mass: p.core.Mass,
		Energy: p.core.Energy, InitialEnergy: p.core.InitialEnergy,
		Damage: p.damage, Time: p.core.Time, Distance: p.core.Distance,
		Stopped: p.core.Stopped,
	}
}

// Step advances dt seconds against a frozen target snapshot. It reports entry,
// exit, and stop events in deterministic path order. Overlapping resistances add.
// Material occupancy survives frame boundaries, preventing duplicate entry damage.
// All targets are validated before any state changes. An error returns no events
// and leaves every projectile field and its material history unchanged.
func (p *Projectile) Step(dt float64, targets []Target) ([]Event, error) {
	if p == nil {
		return nil, fmt.Errorf("ballistics3d: nil projectile")
	}
	if !numeric.Finite(dt) || dt <= 0 || p.MaxEvents < 1 {
		return nil, fmt.Errorf("ballistics3d: positive finite duration and event budget required")
	}
	canonical := make([]Target, len(targets))
	seen := make(map[string]bool, len(targets))
	for i, target := range targets {
		if target.ID == "" || seen[target.ID] {
			return nil, fmt.Errorf("ballistics3d: target IDs must be nonempty and unique")
		}
		seen[target.ID] = true
		if err := target.Material.Validate(); err != nil {
			return nil, err
		}
		shape, ok := collision3d.CloneShape(target.Collider.Shape)
		if !ok {
			return nil, fmt.Errorf("ballistics3d: invalid target shape %q", target.ID)
		}
		target.Collider.Shape = shape
		if _, _, err := collision3d.Bounds(target.Collider); err != nil {
			return nil, fmt.Errorf("ballistics3d: target %q: %w", target.ID, err)
		}
		canonical[i] = target
	}

	var intervals []penetration.Interval
	normals := make(map[string]collision3d.RayHit, len(targets))
	if !p.core.Stopped {
		for _, target := range canonical {
			hit, ok, err := collision3d.RayInterval(target.Collider, p.origin, p.direction)
			if err != nil {
				return nil, fmt.Errorf("ballistics3d: target %q: %w", target.ID, err)
			}
			if !ok || hit.ExitDistance < p.core.Distance {
				continue
			}
			normals[target.ID] = hit
			intervals = append(intervals, penetration.Interval{
				ID: target.ID, Material: target.Material, Entry: hit.EntryDistance,
				Exit: hit.ExitDistance, StartsInside: hit.StartsInside || hit.EntryDistance < p.core.Distance && p.core.Distance < hit.ExitDistance, Tangent: hit.Tangent,
			})
		}
	}
	after, _, rawEvents, err := penetration.Advance(p.core, intervals, dt, p.MaxEvents)
	if err != nil {
		return nil, err
	}
	position := p.origin.Add(p.direction.Mul(after.Distance))
	speed, err := ballistics.Speed(after.Mass, after.Energy)
	if err != nil {
		return nil, err
	}
	velocity := p.direction.Mul(speed)
	if after.Stopped {
		velocity = math3d.Vec3{}
	}
	damage, err := ballistics.Damage(after.BaseDamage, after.Energy, after.InitialEnergy)
	if err != nil {
		return nil, err
	}
	if !validVec(position) || !validVec(velocity) {
		return nil, fmt.Errorf("ballistics3d: unrepresentable projectile motion")
	}
	events := make([]Event, 0, len(rawEvents))
	for _, raw := range rawEvents {
		point := p.origin.Add(p.direction.Mul(raw.Distance))
		if !validVec(point) {
			return nil, fmt.Errorf("ballistics3d: unrepresentable event position")
		}
		normal := math3d.Vec3{}
		hit := normals[raw.TargetID]
		switch raw.Kind {
		case ballistics.Hit:
			if !raw.StartedInside {
				normal = hit.EntryNormal
			}
		case ballistics.Exit:
			normal = hit.ExitNormal
		}
		events = append(events, Event{
			Kind: raw.Kind, TargetID: raw.TargetID, Material: raw.Material,
			Time: raw.Time, Distance: raw.Distance, Energy: raw.Energy,
			EntryEnergy: raw.EntryEnergy, EnergySpent: raw.EnergySpent,
			Damage: raw.Damage, Position: point, Normal: normal,
			StartedInside: raw.StartedInside,
			ActiveTargets: append([]string(nil), raw.ActiveTargets...),
		})
	}
	p.core, p.position, p.velocity, p.damage = after, position, velocity, damage
	return events, nil
}

func validVec(v math3d.Vec3) bool {
	return numeric.Finite(v.X) && numeric.Finite(v.Y) && numeric.Finite(v.Z)
}

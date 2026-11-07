// Package demoscene provides the shared Go simulations used by the browser demo.
package demoscene

import (
	"fmt"
	"math"

	"github.com/kyrqon/GoSimPhysics/ballistics"
	"github.com/kyrqon/GoSimPhysics/ballistics2d"
	"github.com/kyrqon/GoSimPhysics/ballistics3d"
	"github.com/kyrqon/GoSimPhysics/collision2d"
	"github.com/kyrqon/GoSimPhysics/collision3d"
	"github.com/kyrqon/GoSimPhysics/dynamics2d"
	"github.com/kyrqon/GoSimPhysics/dynamics3d"
	"github.com/kyrqon/GoSimPhysics/math2d"
	"github.com/kyrqon/GoSimPhysics/math3d"
)

// Vector and Rotation are renderer-facing data, independent of either engine API.
type Vector struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type Rotation struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
	W float64 `json:"w"`
}

type RenderBody struct {
	ID          int      `json:"id"`
	Shape       string   `json:"shape"`
	Position    Vector   `json:"position"`
	Rotation    Rotation `json:"rotation"`
	HalfExtents Vector   `json:"halfExtents"`
	Radius      float64  `json:"radius"`
	Static      bool     `json:"static"`
	Continuous  bool     `json:"continuous"`
	Color       string   `json:"color"`
}

type RenderJoint struct {
	A Vector `json:"a"`
	B Vector `json:"b"`
}

// RenderShotEvent records an actual Go event, used for entry/exit/stop markers.
type RenderShotEvent struct {
	Kind     string  `json:"kind"`
	TargetID string  `json:"targetID"`
	Position Vector  `json:"position"`
	Energy   float64 `json:"energy"`
	Damage   float64 `json:"damage"`
}

// RenderShot includes the example game's health ledger. Only target Hit events
// change HP; the ballistics packages themselves do not manage game health.
type RenderShot struct {
	Lane          string            `json:"lane"`
	Label         string            `json:"label"`
	Color         string            `json:"color"`
	TargetID      string            `json:"targetID"`
	WallID        string            `json:"wallID"`
	Start         Vector            `json:"start"`
	Position      Vector            `json:"position"`
	Target        Vector            `json:"target"`
	Damage        float64           `json:"damage"`
	HP            float64           `json:"hp"`
	Energy        float64           `json:"energy"`
	ExitEnergy    float64           `json:"exitEnergy"`
	Penetrated    bool              `json:"penetrated"`
	Stopped       bool              `json:"stopped"`
	StoppedInWall bool              `json:"stoppedInWall"`
	Events        []RenderShotEvent `json:"events"`
}

type demoShot struct {
	render      RenderShot
	projectile2 *ballistics2d.Projectile
	projectile3 *ballistics3d.Projectile
}

type Snapshot struct {
	Dimension          int           `json:"dimension"`
	Scene              string        `json:"scene"`
	Time               float64       `json:"time"`
	BodyCount          int           `json:"bodyCount"`
	Energy             float64       `json:"energy"`
	JointError         float64       `json:"jointError"`
	Gravity            float64       `json:"gravity"`
	Substeps           int           `json:"substeps"`
	Bodies             []RenderBody  `json:"bodies"`
	Joints             []RenderJoint `json:"joints"`
	Ballistics         []RenderShot  `json:"ballistics,omitempty"`
	BallisticsComplete bool          `json:"ballisticsComplete,omitempty"`
}

// Session is not safe for concurrent use. The browser invokes it on its Go thread.
type Session struct {
	world3 *dynamics3d.World
	world2 *dynamics2d.World
	// The discrete comparison runs independently so CCD event times cannot give
	// its otherwise discrete solver additional collision samples.
	comparison3       *dynamics3d.World
	comparison2       *dynamics2d.World
	joints3           []*dynamics3d.DistanceJoint
	scene             string
	time              float64
	gravity           float64
	spawned           int
	shots             []demoShot
	targets2          []ballistics2d.Target
	targets3          []ballistics3d.Target
	penetrationBodies []RenderBody
}

var colors = [...]string{"#5eead4", "#fbbf24", "#93c5fd", "#fb923c", "#c4b5fd", "#fda4af"}

// New creates one of the named, deterministic demonstration scenes.
func New(scene string) (*Session, error) {
	s := &Session{scene: scene, gravity: 9.81}
	if s.isPenetration() {
		s.gravity = 0
		return s, s.setupPenetration()
	}
	if scene == "billiards2d" || scene == "ccd2d" {
		s.gravity = 0
		s.world2 = dynamics2d.NewWorld(dynamics2d.Vec2{})
		if scene == "ccd2d" {
			s.world2.Substeps = 1
			s.comparison2 = dynamics2d.NewWorld(dynamics2d.Vec2{})
			s.comparison2.Substeps = 1
			return s, s.setupCCD2D()
		}
		return s, s.setup2D()
	}
	if scene != "stack" && scene != "rain" && scene != "pendulum" && scene != "ccd" {
		return nil, fmt.Errorf("unknown scene %q", scene)
	}
	if scene == "ccd" {
		s.gravity = 0
		s.world3 = dynamics3d.NewWorld(dynamics3d.Vec3{})
		s.world3.Substeps = 1
		s.comparison3 = dynamics3d.NewWorld(dynamics3d.Vec3{})
		s.comparison3.Substeps = 1
		return s, s.setupCCD3D()
	}
	s.world3 = dynamics3d.NewWorld(dynamics3d.Vec3{Y: -s.gravity})
	s.world3.VelocityIterations, s.world3.PositionIterations = 16, 6
	if err := s.setup3D(); err != nil {
		return nil, err
	}
	return s, nil
}

func (s *Session) isPenetration() bool { return s.scene == "penetration" || s.scene == "penetration2d" }

// Three independent point shots compare the same game-tuned material. Marker
// radius is a renderer convenience and is never passed to the ballistic solver.
func (s *Session) setupPenetration() error {
	lanes := []struct {
		name, label, color string
		halfX              float64
	}{
		{"clear", "Clear path", "#5eead4", 0},
		{"thin", "Thin barrier · 0.3 m", "#fbbf24", 0.15},
		{"thick", "Thick barrier · 2 m", "#fda4af", 1},
	}
	barrier := ballistics.Material{Name: "Game barrier", Resistance: 1000}
	absorber := ballistics.Material{Name: "Game target", Resistance: 10000}
	for i, lane := range lanes {
		start, target := Vector{X: -3, Y: 0.5, Z: float64(i-1) * 2}, Vector{X: 3, Y: 0.5, Z: float64(i-1) * 2}
		if s.scene == "penetration2d" {
			start.Y, target.Y, start.Z, target.Z = float64(1-i)*2, float64(1-i)*2, 0, 0
		}
		shot := demoShot{render: RenderShot{Lane: lane.name, Label: lane.label, Color: lane.color, TargetID: lane.name + "-target", Start: start, Position: start, Target: target, HP: 100, Energy: 800, Events: []RenderShotEvent{}}}
		addBox := func(id string, pos Vector, halfX float64, material ballistics.Material, shade string) {
			ext := Vector{X: halfX, Y: 0.5, Z: 0.5}
			s.penetrationBodies = append(s.penetrationBodies, RenderBody{ID: len(s.penetrationBodies), Shape: "box", Position: pos, Rotation: Rotation{W: 1}, HalfExtents: ext, Static: true, Color: shade})
			if s.scene == "penetration2d" {
				s.targets2 = append(s.targets2, ballistics2d.Target{ID: id, Collider: collision2d.Collider{Shape: collision2d.Box{HalfExtents: math2d.Vec2{X: halfX, Y: 0.5}}, Position: math2d.Vec2{X: pos.X, Y: pos.Y}}, Material: material})
			} else {
				s.targets3 = append(s.targets3, ballistics3d.Target{ID: id, Collider: collision3d.Collider{Shape: collision3d.Box{HalfExtents: math3d.Vec3{X: halfX, Y: 0.5, Z: 0.5}}, Position: math3d.Vec3{X: pos.X, Y: pos.Y, Z: pos.Z}}, Material: material})
			}
		}
		if lane.halfX > 0 {
			shot.render.WallID = lane.name + "-wall"
			wall := target
			wall.X = 0
			addBox(shot.render.WallID, wall, lane.halfX, barrier, "#69849b")
		}
		addBox(shot.render.TargetID, target, 0.2, absorber, lane.color)
		var err error
		if s.scene == "penetration2d" {
			shot.projectile2, err = ballistics2d.NewProjectile(ballistics2d.ProjectileDef{Position: math2d.Vec2{X: start.X, Y: start.Y}, Velocity: math2d.Vec2{X: 40}, Mass: 1, Damage: 100})
		} else {
			shot.projectile3, err = ballistics3d.NewProjectile(ballistics3d.ProjectileDef{Position: math3d.Vec3{X: start.X, Y: start.Y, Z: start.Z}, Velocity: math3d.Vec3{X: 40}, Mass: 1, Damage: 100})
		}
		if err != nil {
			return err
		}
		s.shots = append(s.shots, shot)
	}
	return nil
}

func (shot *demoShot) record(kind ballistics.EventKind, targetID string, pos Vector, energy, damage float64, active []string) {
	shot.render.Events = append(shot.render.Events, RenderShotEvent{Kind: kind.String(), TargetID: targetID, Position: pos, Energy: energy, Damage: damage})
	if kind == ballistics.Hit && targetID == shot.render.TargetID {
		shot.render.Damage += damage
		shot.render.HP = math.Max(0, shot.render.HP-damage)
	}
	if kind == ballistics.Exit && targetID == shot.render.WallID {
		shot.render.Penetrated, shot.render.ExitEnergy = true, energy
	}
	if kind == ballistics.Stop {
		for _, id := range active {
			if id == shot.render.WallID {
				shot.render.StoppedInWall = true
			}
		}
	}
}

func (s *Session) stepPenetration(dt float64) error {
	for i := range s.shots {
		shot := &s.shots[i]
		if shot.render.Stopped {
			continue
		}
		if shot.projectile2 != nil {
			events, err := shot.projectile2.Step(dt, s.targets2)
			if err != nil {
				return err
			}
			for _, event := range events {
				shot.record(event.Kind, event.TargetID, Vector{X: event.Position.X, Y: event.Position.Y}, event.Energy, event.Damage, event.ActiveTargets)
			}
			state := shot.projectile2.State()
			shot.render.Position, shot.render.Energy, shot.render.Stopped = Vector{X: state.Position.X, Y: state.Position.Y}, state.Energy, state.Stopped
		} else {
			events, err := shot.projectile3.Step(dt, s.targets3)
			if err != nil {
				return err
			}
			for _, event := range events {
				shot.record(event.Kind, event.TargetID, Vector{X: event.Position.X, Y: event.Position.Y, Z: event.Position.Z}, event.Energy, event.Damage, event.ActiveTargets)
			}
			state := shot.projectile3.State()
			shot.render.Position, shot.render.Energy, shot.render.Stopped = Vector{X: state.Position.X, Y: state.Position.Y, Z: state.Position.Z}, state.Energy, state.Stopped
		}
	}
	s.time += dt
	return nil
}

func (s *Session) penetrationSnapshot() Snapshot {
	state := Snapshot{Scene: s.scene, Time: s.time, Dimension: 3, Substeps: 1, Bodies: append([]RenderBody{}, s.penetrationBodies...), Joints: []RenderJoint{}, BallisticsComplete: true}
	if s.scene == "penetration2d" {
		state.Dimension = 2
	}
	for _, shot := range s.shots {
		render := shot.render
		render.Events = append([]RenderShotEvent{}, render.Events...)
		state.Ballistics = append(state.Ballistics, render)
		state.BallisticsComplete = state.BallisticsComplete && render.Stopped
		state.Energy += render.Energy
		shape := "sphere"
		if state.Dimension == 2 {
			shape = "circle"
		}
		state.Bodies = append(state.Bodies, RenderBody{ID: len(state.Bodies), Shape: shape, Position: render.Position, Radius: 0.08, Rotation: Rotation{W: 1}, Color: render.Color})
	}
	state.BodyCount = len(state.Bodies)
	return state
}

// The two lanes differ only in Continuous. At 400 m/s the 0.1 m projectile
// travels 6.67 m in one 1/60 s step, passing a 0.02 m wall between samples.
// The browser pauses after this first step so both actual results remain visible.
func (s *Session) setupCCD3D() error {
	for i, z := range []float64{-1.1, 1.1} {
		world := s.world3
		if i == 1 {
			world = s.comparison3
		}
		if _, err := world.CreateBody(dynamics3d.BodyDef{
			Shape:    dynamics3d.Box{HalfExtents: dynamics3d.Vec3{X: 0.01, Y: 0.5, Z: 0.55}},
			Position: dynamics3d.Vec3{Y: 0.6, Z: z},
		}); err != nil {
			return err
		}
		if _, err := world.CreateBody(dynamics3d.BodyDef{
			Type: dynamics3d.Dynamic, Shape: dynamics3d.Sphere{Radius: 0.05}, Mass: 1,
			Position: dynamics3d.Vec3{X: -2, Y: 0.6, Z: z}, Velocity: dynamics3d.Vec3{X: 400},
			Continuous: i == 0,
		}); err != nil {
			return err
		}
	}
	return nil
}

func (s *Session) setupCCD2D() error {
	for i, y := range []float64{1.1, -1.1} {
		world := s.world2
		if i == 1 {
			world = s.comparison2
		}
		if _, err := world.CreateBody(dynamics2d.BodyDef{
			Shape: dynamics2d.Box{HalfExtents: dynamics2d.Vec2{X: 0.01, Y: 0.5}}, Position: dynamics2d.Vec2{Y: y},
		}); err != nil {
			return err
		}
		if _, err := world.CreateBody(dynamics2d.BodyDef{
			Type: dynamics2d.Dynamic, Shape: dynamics2d.Circle{Radius: 0.05}, Mass: 1,
			Position: dynamics2d.Vec2{X: -2, Y: y}, Velocity: dynamics2d.Vec2{X: 400}, Continuous: i == 0,
		}); err != nil {
			return err
		}
	}
	return nil
}

func (s *Session) setup3D() error {
	add := func(def dynamics3d.BodyDef) error {
		_, err := s.world3.CreateBody(def)
		return err
	}
	if err := add(dynamics3d.BodyDef{Shape: dynamics3d.Box{HalfExtents: dynamics3d.Vec3{X: 6, Y: 0.3, Z: 6}}, Position: dynamics3d.Vec3{Y: -0.3}, Friction: 0.65}); err != nil {
		return err
	}
	if s.scene == "pendulum" {
		anchor, err := s.world3.CreateBody(dynamics3d.BodyDef{Shape: dynamics3d.Sphere{Radius: 0.13}, Position: dynamics3d.Vec3{Y: 4}})
		if err != nil {
			return err
		}
		bob, err := s.world3.CreateBody(dynamics3d.BodyDef{
			Type: dynamics3d.Dynamic, Shape: dynamics3d.Sphere{Radius: 0.4}, Mass: 2,
			Position: dynamics3d.Vec3{X: 2.4, Y: 2.2}, Velocity: dynamics3d.Vec3{Z: 1.8}, Friction: 0.6,
		})
		if err != nil {
			return err
		}
		joint, err := s.world3.CreateDistanceJoint(dynamics3d.DistanceJointDef{BodyA: anchor, BodyB: bob, Length: 3})
		if err != nil {
			return err
		}
		s.joints3 = append(s.joints3, joint)
		return nil
	}
	// Low walls contain the scene after a kick, without hiding the bodies.
	for _, wall := range []dynamics3d.BodyDef{
		{Shape: dynamics3d.Box{HalfExtents: dynamics3d.Vec3{X: 0.15, Y: 0.6, Z: 6}}, Position: dynamics3d.Vec3{X: -6, Y: 0.6}},
		{Shape: dynamics3d.Box{HalfExtents: dynamics3d.Vec3{X: 0.15, Y: 0.6, Z: 6}}, Position: dynamics3d.Vec3{X: 6, Y: 0.6}},
		{Shape: dynamics3d.Box{HalfExtents: dynamics3d.Vec3{X: 6, Y: 0.6, Z: 0.15}}, Position: dynamics3d.Vec3{Y: 0.6, Z: -6}},
		{Shape: dynamics3d.Box{HalfExtents: dynamics3d.Vec3{X: 6, Y: 0.6, Z: 0.15}}, Position: dynamics3d.Vec3{Y: 0.6, Z: 6}},
	} {
		wall.Friction = 0.5
		if err := add(wall); err != nil {
			return err
		}
	}
	if s.scene == "stack" {
		for row := 0; row < 3; row++ {
			for column := 0; column < 2; column++ {
				if err := add(dynamics3d.BodyDef{
					Type: dynamics3d.Dynamic, Shape: dynamics3d.Box{HalfExtents: dynamics3d.Vec3{X: 0.45, Y: 0.45, Z: 0.45}},
					Mass: 1, Position: dynamics3d.Vec3{X: float64(column)*1.1 - 0.55, Y: 0.46 + float64(row)*0.91}, Friction: 0.65,
				}); err != nil {
					return err
				}
			}
		}
		for _, x := range []float64{-2.2, 2.2} {
			if err := add(dynamics3d.BodyDef{Type: dynamics3d.Dynamic, Shape: dynamics3d.Sphere{Radius: 0.5}, Mass: 1, Position: dynamics3d.Vec3{X: x, Y: 3.6, Z: 1}, Restitution: 0.55, Friction: 0.5}); err != nil {
				return err
			}
		}
		return add(dynamics3d.BodyDef{
			Type: dynamics3d.Dynamic, Shape: dynamics3d.Box{HalfExtents: dynamics3d.Vec3{X: 0.5, Y: 0.35, Z: 0.6}}, Mass: 1.5,
			Position: dynamics3d.Vec3{X: 2, Y: 4.5, Z: -1.5}, Rotation: dynamics3d.QuaternionFromAxisAngle(dynamics3d.Vec3{X: 1, Z: 1}, 0.6), Friction: 0.6,
		})
	}
	for i := 0; i < 14; i++ {
		var shape dynamics3d.Shape = dynamics3d.Sphere{Radius: 0.35 + float64(i%3)*0.05}
		if i%2 == 1 {
			shape = dynamics3d.Box{HalfExtents: dynamics3d.Vec3{X: 0.35, Y: 0.35, Z: 0.35}}
		}
		if err := add(dynamics3d.BodyDef{
			Type: dynamics3d.Dynamic, Shape: shape, Mass: 1,
			Position:    dynamics3d.Vec3{X: float64(i%4)*1.2 - 1.8, Y: 2 + float64(i/4)*1.2, Z: float64((i*3)%5)*0.9 - 1.8},
			Rotation:    dynamics3d.QuaternionFromAxisAngle(dynamics3d.Vec3{X: 1, Y: 2, Z: 1}, float64(i)*0.17),
			Restitution: 0.35, Friction: 0.55,
		}); err != nil {
			return err
		}
	}
	return nil
}

func (s *Session) setup2D() error {
	for _, def := range []dynamics2d.BodyDef{
		{Shape: dynamics2d.Box{HalfExtents: dynamics2d.Vec2{X: 0.15, Y: 3.75}}, Position: dynamics2d.Vec2{X: -6}},
		{Shape: dynamics2d.Box{HalfExtents: dynamics2d.Vec2{X: 0.15, Y: 3.75}}, Position: dynamics2d.Vec2{X: 6}},
		{Shape: dynamics2d.Box{HalfExtents: dynamics2d.Vec2{X: 6, Y: 0.15}}, Position: dynamics2d.Vec2{Y: -3.75}},
		{Shape: dynamics2d.Box{HalfExtents: dynamics2d.Vec2{X: 6, Y: 0.15}}, Position: dynamics2d.Vec2{Y: 3.75}},
		{Shape: dynamics2d.Box{HalfExtents: dynamics2d.Vec2{X: 0.35, Y: 0.7}}, Position: dynamics2d.Vec2{X: -1.2, Y: -1.9}, Angle: 0.4},
	} {
		def.Restitution = 0.92
		if _, err := s.world2.CreateBody(def); err != nil {
			return err
		}
	}
	if _, err := s.world2.CreateBody(dynamics2d.BodyDef{
		Type: dynamics2d.Dynamic, Shape: dynamics2d.Circle{Radius: 0.38}, Mass: 1,
		Position: dynamics2d.Vec2{X: -3.6}, Velocity: dynamics2d.Vec2{X: 5}, Restitution: 0.92, Friction: 0.05,
	}); err != nil {
		return err
	}
	for row := 0; row < 3; row++ {
		for i := 0; i <= row; i++ {
			if _, err := s.world2.CreateBody(dynamics2d.BodyDef{
				Type: dynamics2d.Dynamic, Shape: dynamics2d.Circle{Radius: 0.38}, Mass: 1,
				Position: dynamics2d.Vec2{X: 1 + float64(row)*0.7, Y: (float64(i) - float64(row)/2) * 0.8}, Restitution: 0.92, Friction: 0.05,
			}); err != nil {
				return err
			}
		}
	}
	return nil
}

// Step advances the Go engine; the frontend supplies a fixed 1/60-second dt.
func (s *Session) Step(dt float64) error {
	if !finite(dt) || dt <= 0 || dt > 0.1 {
		return fmt.Errorf("demo step must be in (0, 0.1] seconds")
	}
	if s.isPenetration() {
		return s.stepPenetration(dt)
	}
	var err error
	if s.world3 != nil {
		err = s.world3.Step(dt)
		if err == nil && s.comparison3 != nil {
			err = s.comparison3.Step(dt)
		}
	} else {
		err = s.world2.Step(dt)
		if err == nil && s.comparison2 != nil {
			err = s.comparison2.Step(dt)
		}
	}
	if err == nil {
		s.time += dt
	}
	return err
}

// Configure changes the world settings without reconstructing the scene.
func (s *Session) Configure(gravity float64, substeps int) error {
	if s.isPenetration() {
		return fmt.Errorf("world settings are fixed in the penetration comparison")
	}
	if !finite(gravity) || gravity < 0 || gravity > 30 || substeps < 1 || substeps > 8 {
		return fmt.Errorf("gravity must be in [0,30] and substeps in [1,8]")
	}
	s.gravity = gravity
	if s.world3 != nil {
		s.world3.Gravity, s.world3.Substeps = dynamics3d.Vec3{Y: -gravity}, substeps
		if s.comparison3 != nil {
			s.comparison3.Gravity, s.comparison3.Substeps = s.world3.Gravity, substeps
		}
	} else {
		s.world2.Gravity, s.world2.Substeps = dynamics2d.Vec2{Y: -gravity}, substeps
		if s.comparison2 != nil {
			s.comparison2.Gravity, s.comparison2.Substeps = s.world2.Gravity, substeps
		}
	}
	return nil
}

func (s *Session) dynamicCount() int {
	if s.isPenetration() {
		return len(s.shots)
	}
	count := 0
	if s.world3 != nil {
		for _, b := range s.world3.Bodies() {
			if b.Type() == dynamics3d.Dynamic {
				count++
			}
		}
	} else {
		for _, b := range s.world2.Bodies() {
			if b.Type() == dynamics2d.Dynamic {
				count++
			}
		}
	}
	return count
}

// Spawn adds a body, bounded to keep this small-scene demonstration responsive.
func (s *Session) Spawn(kind string) error {
	if s.isPenetration() {
		return fmt.Errorf("spawning is unavailable in the penetration comparison")
	}
	if kind != "sphere" && kind != "box" {
		return fmt.Errorf("unknown shape %q", kind)
	}
	if s.dynamicCount() >= 40 {
		return fmt.Errorf("scene limit reached (40 dynamic bodies); reset to start over")
	}
	i := s.spawned
	if s.world3 != nil {
		var shape dynamics3d.Shape = dynamics3d.Sphere{Radius: 0.4}
		if kind == "box" {
			shape = dynamics3d.Box{HalfExtents: dynamics3d.Vec3{X: 0.4, Y: 0.4, Z: 0.4}}
		}
		_, err := s.world3.CreateBody(dynamics3d.BodyDef{
			Type: dynamics3d.Dynamic, Shape: shape, Mass: 1,
			Position:    dynamics3d.Vec3{X: float64(i%5)*0.9 - 1.8, Y: 5.5 + float64(i%3)*0.6, Z: float64((i*3)%5)*0.8 - 1.6},
			Rotation:    dynamics3d.QuaternionFromAxisAngle(dynamics3d.Vec3{X: 1, Y: 1, Z: 1}, float64(i)*0.3),
			Restitution: 0.35, Friction: 0.6,
		})
		if err != nil {
			return err
		}
	} else {
		var shape dynamics2d.Shape = dynamics2d.Circle{Radius: 0.38}
		if kind == "box" {
			shape = dynamics2d.Box{HalfExtents: dynamics2d.Vec2{X: 0.35, Y: 0.35}}
		}
		_, err := s.world2.CreateBody(dynamics2d.BodyDef{
			Type: dynamics2d.Dynamic, Shape: shape, Mass: 1,
			Position: dynamics2d.Vec2{X: float64(i%6)*0.8 - 2.5, Y: 2},
			Velocity: dynamics2d.Vec2{X: 2, Y: -1}, Restitution: 0.9, Friction: 0.05,
		})
		if err != nil {
			return err
		}
	}
	s.spawned++
	return nil
}

// Kick applies off-center impulses to the dynamic bodies.
func (s *Session) Kick() error {
	if s.isPenetration() {
		return fmt.Errorf("impulses are unavailable in the penetration comparison")
	}
	if s.world3 != nil {
		for i, b := range s.world3.Bodies() {
			if b.Type() != dynamics3d.Dynamic {
				continue
			}
			impulse := dynamics3d.Vec3{X: float64(i%3-1) * 1.3, Y: 3.2, Z: float64((i+1)%3-1) * 1.4}.Mul(b.Mass())
			if err := b.ApplyImpulse(impulse, b.Position.Add(dynamics3d.Vec3{Y: 0.15, Z: 0.15})); err != nil {
				return err
			}
		}
	} else {
		for i, b := range s.world2.Bodies() {
			if b.Type() != dynamics2d.Dynamic {
				continue
			}
			if err := b.ApplyImpulse(dynamics2d.Vec2{X: float64(i%3-1) * 2, Y: 1.5}, b.Position.Add(dynamics2d.Vec2{Y: 0.1})); err != nil {
				return err
			}
		}
	}
	return nil
}

func vector(v dynamics3d.Vec3) Vector { return Vector{v.X, v.Y, v.Z} }

// Snapshot exports only geometry and measurements; all simulation remains in Go.
func (s *Session) Snapshot() Snapshot {
	if s.isPenetration() {
		return s.penetrationSnapshot()
	}
	state := Snapshot{Scene: s.scene, Time: s.time, Gravity: s.gravity, Bodies: []RenderBody{}, Joints: []RenderJoint{}}
	if s.world3 != nil {
		state.Dimension, state.Substeps = 3, s.world3.Substeps
		for _, world := range []*dynamics3d.World{s.world3, s.comparison3} {
			if world == nil {
				continue
			}
			for _, b := range world.Bodies() {
				id := len(state.Bodies)
				body := RenderBody{ID: id, Position: vector(b.Position), Rotation: Rotation{b.Rotation.X, b.Rotation.Y, b.Rotation.Z, b.Rotation.W}, Static: b.Type() == dynamics3d.Static, Continuous: b.Continuous, Color: colors[id%len(colors)]}
				if body.Static {
					body.Color = "#344657"
				} else if s.scene == "ccd" {
					body.Color = ccdColor(body.Continuous)
				}
				switch shape := b.Shape().(type) {
				case dynamics3d.Sphere:
					body.Shape, body.Radius = "sphere", shape.Radius
				case dynamics3d.Box:
					body.Shape, body.HalfExtents = "box", vector(shape.HalfExtents)
				}
				state.Bodies = append(state.Bodies, body)
				if b.Type() == dynamics3d.Dynamic {
					state.Energy += 0.5*b.Mass()*b.Velocity.LengthSquared() + 0.5*b.AngularVelocity.Dot(b.AngularMomentum()) + b.Mass()*s.gravity*b.Position.Y
				}
			}
		}
		for _, j := range s.joints3 {
			a, b := j.Anchors()
			state.Joints = append(state.Joints, RenderJoint{vector(a), vector(b)})
			state.JointError = math.Max(state.JointError, math.Abs(b.Sub(a).Length()-j.Length()))
		}
	} else {
		state.Dimension, state.Substeps = 2, s.world2.Substeps
		for _, world := range []*dynamics2d.World{s.world2, s.comparison2} {
			if world == nil {
				continue
			}
			for _, b := range world.Bodies() {
				id := len(state.Bodies)
				body := RenderBody{ID: id, Position: Vector{X: b.Position.X, Y: b.Position.Y}, Rotation: Rotation{Z: math.Sin(b.Angle / 2), W: math.Cos(b.Angle / 2)}, Static: b.Type() == dynamics2d.Static, Continuous: b.Continuous, Color: colors[id%len(colors)]}
				if body.Static {
					body.Color = "#344657"
				} else if s.scene == "ccd2d" {
					body.Color = ccdColor(body.Continuous)
				}
				switch shape := b.Shape().(type) {
				case dynamics2d.Circle:
					body.Shape, body.Radius = "circle", shape.Radius
				case dynamics2d.Box:
					body.Shape, body.HalfExtents = "box", Vector{X: shape.HalfExtents.X, Y: shape.HalfExtents.Y}
				}
				state.Bodies = append(state.Bodies, body)
				if b.Type() == dynamics2d.Dynamic {
					state.Energy += 0.5*b.Mass()*b.Velocity.LengthSquared() + 0.5*b.Inertia()*b.AngularVelocity*b.AngularVelocity + b.Mass()*s.gravity*b.Position.Y
				}
			}
		}
	}
	state.BodyCount = len(state.Bodies)
	return state
}

func ccdColor(continuous bool) string {
	if continuous {
		return "#5eead4"
	}
	return "#fda4af"
}

func finite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }

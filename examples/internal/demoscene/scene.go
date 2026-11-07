// Package demoscene provides the shared Go simulations used by the browser demo.
package demoscene

import (
	"fmt"
	"math"

	physics "github.com/kyrqon/GoSimPhysics"
	"github.com/kyrqon/GoSimPhysics/physics3d"
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
	Color       string   `json:"color"`
}

type RenderJoint struct {
	A Vector `json:"a"`
	B Vector `json:"b"`
}

type Snapshot struct {
	Dimension  int           `json:"dimension"`
	Scene      string        `json:"scene"`
	Time       float64       `json:"time"`
	BodyCount  int           `json:"bodyCount"`
	Energy     float64       `json:"energy"`
	JointError float64       `json:"jointError"`
	Gravity    float64       `json:"gravity"`
	Substeps   int           `json:"substeps"`
	Bodies     []RenderBody  `json:"bodies"`
	Joints     []RenderJoint `json:"joints"`
}

// Session is not safe for concurrent use. The browser invokes it on its Go thread.
type Session struct {
	world3  *physics3d.World
	world2  *physics.World
	joints3 []*physics3d.DistanceJoint
	scene   string
	time    float64
	gravity float64
	spawned int
}

var colors = [...]string{"#5eead4", "#fbbf24", "#93c5fd", "#fb923c", "#c4b5fd", "#fda4af"}

// New creates one of the named, deterministic demonstration scenes.
func New(scene string) (*Session, error) {
	s := &Session{scene: scene, gravity: 9.81}
	if scene == "billiards2d" {
		s.gravity = 0
		s.world2 = physics.NewWorld(physics.Vec2{})
		return s, s.setup2D()
	}
	if scene != "stack" && scene != "rain" && scene != "pendulum" {
		return nil, fmt.Errorf("unknown scene %q", scene)
	}
	s.world3 = physics3d.NewWorld(physics3d.Vec3{Y: -s.gravity})
	s.world3.VelocityIterations, s.world3.PositionIterations = 16, 6
	if err := s.setup3D(); err != nil {
		return nil, err
	}
	return s, nil
}

func (s *Session) setup3D() error {
	add := func(def physics3d.BodyDef) error {
		_, err := s.world3.CreateBody(def)
		return err
	}
	if err := add(physics3d.BodyDef{Shape: physics3d.Box{HalfExtents: physics3d.Vec3{X: 6, Y: 0.3, Z: 6}}, Position: physics3d.Vec3{Y: -0.3}, Friction: 0.65}); err != nil {
		return err
	}
	if s.scene == "pendulum" {
		anchor, err := s.world3.CreateBody(physics3d.BodyDef{Shape: physics3d.Sphere{Radius: 0.13}, Position: physics3d.Vec3{Y: 4}})
		if err != nil {
			return err
		}
		bob, err := s.world3.CreateBody(physics3d.BodyDef{
			Type: physics3d.Dynamic, Shape: physics3d.Sphere{Radius: 0.4}, Mass: 2,
			Position: physics3d.Vec3{X: 2.4, Y: 2.2}, Velocity: physics3d.Vec3{Z: 1.8}, Friction: 0.6,
		})
		if err != nil {
			return err
		}
		joint, err := s.world3.CreateDistanceJoint(physics3d.DistanceJointDef{BodyA: anchor, BodyB: bob, Length: 3})
		if err != nil {
			return err
		}
		s.joints3 = append(s.joints3, joint)
		return nil
	}
	// Low walls contain the scene after a kick, without hiding the bodies.
	for _, wall := range []physics3d.BodyDef{
		{Shape: physics3d.Box{HalfExtents: physics3d.Vec3{X: 0.15, Y: 0.6, Z: 6}}, Position: physics3d.Vec3{X: -6, Y: 0.6}},
		{Shape: physics3d.Box{HalfExtents: physics3d.Vec3{X: 0.15, Y: 0.6, Z: 6}}, Position: physics3d.Vec3{X: 6, Y: 0.6}},
		{Shape: physics3d.Box{HalfExtents: physics3d.Vec3{X: 6, Y: 0.6, Z: 0.15}}, Position: physics3d.Vec3{Y: 0.6, Z: -6}},
		{Shape: physics3d.Box{HalfExtents: physics3d.Vec3{X: 6, Y: 0.6, Z: 0.15}}, Position: physics3d.Vec3{Y: 0.6, Z: 6}},
	} {
		wall.Friction = 0.5
		if err := add(wall); err != nil {
			return err
		}
	}
	if s.scene == "stack" {
		for row := 0; row < 3; row++ {
			for column := 0; column < 2; column++ {
				if err := add(physics3d.BodyDef{
					Type: physics3d.Dynamic, Shape: physics3d.Box{HalfExtents: physics3d.Vec3{X: 0.45, Y: 0.45, Z: 0.45}},
					Mass: 1, Position: physics3d.Vec3{X: float64(column)*1.1 - 0.55, Y: 0.46 + float64(row)*0.91}, Friction: 0.65,
				}); err != nil {
					return err
				}
			}
		}
		for _, x := range []float64{-2.2, 2.2} {
			if err := add(physics3d.BodyDef{Type: physics3d.Dynamic, Shape: physics3d.Sphere{Radius: 0.5}, Mass: 1, Position: physics3d.Vec3{X: x, Y: 3.6, Z: 1}, Restitution: 0.55, Friction: 0.5}); err != nil {
				return err
			}
		}
		return add(physics3d.BodyDef{
			Type: physics3d.Dynamic, Shape: physics3d.Box{HalfExtents: physics3d.Vec3{X: 0.5, Y: 0.35, Z: 0.6}}, Mass: 1.5,
			Position: physics3d.Vec3{X: 2, Y: 4.5, Z: -1.5}, Rotation: physics3d.QuaternionFromAxisAngle(physics3d.Vec3{X: 1, Z: 1}, 0.6), Friction: 0.6,
		})
	}
	for i := 0; i < 14; i++ {
		var shape physics3d.Shape = physics3d.Sphere{Radius: 0.35 + float64(i%3)*0.05}
		if i%2 == 1 {
			shape = physics3d.Box{HalfExtents: physics3d.Vec3{X: 0.35, Y: 0.35, Z: 0.35}}
		}
		if err := add(physics3d.BodyDef{
			Type: physics3d.Dynamic, Shape: shape, Mass: 1,
			Position:    physics3d.Vec3{X: float64(i%4)*1.2 - 1.8, Y: 2 + float64(i/4)*1.2, Z: float64((i*3)%5)*0.9 - 1.8},
			Rotation:    physics3d.QuaternionFromAxisAngle(physics3d.Vec3{X: 1, Y: 2, Z: 1}, float64(i)*0.17),
			Restitution: 0.35, Friction: 0.55,
		}); err != nil {
			return err
		}
	}
	return nil
}

func (s *Session) setup2D() error {
	for _, def := range []physics.BodyDef{
		{Shape: physics.Box{HalfExtents: physics.Vec2{X: 0.15, Y: 3.75}}, Position: physics.Vec2{X: -6}},
		{Shape: physics.Box{HalfExtents: physics.Vec2{X: 0.15, Y: 3.75}}, Position: physics.Vec2{X: 6}},
		{Shape: physics.Box{HalfExtents: physics.Vec2{X: 6, Y: 0.15}}, Position: physics.Vec2{Y: -3.75}},
		{Shape: physics.Box{HalfExtents: physics.Vec2{X: 6, Y: 0.15}}, Position: physics.Vec2{Y: 3.75}},
		{Shape: physics.Box{HalfExtents: physics.Vec2{X: 0.35, Y: 0.7}}, Position: physics.Vec2{X: -1.2, Y: -1.9}, Angle: 0.4},
	} {
		def.Restitution = 0.92
		if _, err := s.world2.CreateBody(def); err != nil {
			return err
		}
	}
	if _, err := s.world2.CreateBody(physics.BodyDef{
		Type: physics.Dynamic, Shape: physics.Circle{Radius: 0.38}, Mass: 1,
		Position: physics.Vec2{X: -3.6}, Velocity: physics.Vec2{X: 5}, Restitution: 0.92, Friction: 0.05,
	}); err != nil {
		return err
	}
	for row := 0; row < 3; row++ {
		for i := 0; i <= row; i++ {
			if _, err := s.world2.CreateBody(physics.BodyDef{
				Type: physics.Dynamic, Shape: physics.Circle{Radius: 0.38}, Mass: 1,
				Position: physics.Vec2{X: 1 + float64(row)*0.7, Y: (float64(i) - float64(row)/2) * 0.8}, Restitution: 0.92, Friction: 0.05,
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
	var err error
	if s.world3 != nil {
		err = s.world3.Step(dt)
	} else {
		err = s.world2.Step(dt)
	}
	if err == nil {
		s.time += dt
	}
	return err
}

// Configure changes the world settings without reconstructing the scene.
func (s *Session) Configure(gravity float64, substeps int) error {
	if !finite(gravity) || gravity < 0 || gravity > 30 || substeps < 1 || substeps > 8 {
		return fmt.Errorf("gravity must be in [0,30] and substeps in [1,8]")
	}
	s.gravity = gravity
	if s.world3 != nil {
		s.world3.Gravity, s.world3.Substeps = physics3d.Vec3{Y: -gravity}, substeps
	} else {
		s.world2.Gravity, s.world2.Substeps = physics.Vec2{Y: -gravity}, substeps
	}
	return nil
}

func (s *Session) dynamicCount() int {
	count := 0
	if s.world3 != nil {
		for _, b := range s.world3.Bodies() {
			if b.Type() == physics3d.Dynamic {
				count++
			}
		}
	} else {
		for _, b := range s.world2.Bodies() {
			if b.Type() == physics.Dynamic {
				count++
			}
		}
	}
	return count
}

// Spawn adds a body, bounded to keep this small-scene demonstration responsive.
func (s *Session) Spawn(kind string) error {
	if kind != "sphere" && kind != "box" {
		return fmt.Errorf("unknown shape %q", kind)
	}
	if s.dynamicCount() >= 40 {
		return fmt.Errorf("scene limit reached (40 dynamic bodies); reset to start over")
	}
	i := s.spawned
	if s.world3 != nil {
		var shape physics3d.Shape = physics3d.Sphere{Radius: 0.4}
		if kind == "box" {
			shape = physics3d.Box{HalfExtents: physics3d.Vec3{X: 0.4, Y: 0.4, Z: 0.4}}
		}
		_, err := s.world3.CreateBody(physics3d.BodyDef{
			Type: physics3d.Dynamic, Shape: shape, Mass: 1,
			Position:    physics3d.Vec3{X: float64(i%5)*0.9 - 1.8, Y: 5.5 + float64(i%3)*0.6, Z: float64((i*3)%5)*0.8 - 1.6},
			Rotation:    physics3d.QuaternionFromAxisAngle(physics3d.Vec3{X: 1, Y: 1, Z: 1}, float64(i)*0.3),
			Restitution: 0.35, Friction: 0.6,
		})
		if err != nil {
			return err
		}
	} else {
		var shape physics.Shape = physics.Circle{Radius: 0.38}
		if kind == "box" {
			shape = physics.Box{HalfExtents: physics.Vec2{X: 0.35, Y: 0.35}}
		}
		_, err := s.world2.CreateBody(physics.BodyDef{
			Type: physics.Dynamic, Shape: shape, Mass: 1,
			Position: physics.Vec2{X: float64(i%6)*0.8 - 2.5, Y: 2},
			Velocity: physics.Vec2{X: 2, Y: -1}, Restitution: 0.9, Friction: 0.05,
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
	if s.world3 != nil {
		for i, b := range s.world3.Bodies() {
			if b.Type() != physics3d.Dynamic {
				continue
			}
			impulse := physics3d.Vec3{X: float64(i%3-1) * 1.3, Y: 3.2, Z: float64((i+1)%3-1) * 1.4}.Mul(b.Mass())
			if err := b.ApplyImpulse(impulse, b.Position.Add(physics3d.Vec3{Y: 0.15, Z: 0.15})); err != nil {
				return err
			}
		}
	} else {
		for i, b := range s.world2.Bodies() {
			if b.Type() != physics.Dynamic {
				continue
			}
			if err := b.ApplyImpulse(physics.Vec2{X: float64(i%3-1) * 2, Y: 1.5}, b.Position.Add(physics.Vec2{Y: 0.1})); err != nil {
				return err
			}
		}
	}
	return nil
}

func vector(v physics3d.Vec3) Vector { return Vector{v.X, v.Y, v.Z} }

// Snapshot exports only geometry and measurements; all simulation remains in Go.
func (s *Session) Snapshot() Snapshot {
	state := Snapshot{Scene: s.scene, Time: s.time, Gravity: s.gravity, Bodies: []RenderBody{}, Joints: []RenderJoint{}}
	if s.world3 != nil {
		state.Dimension, state.Substeps = 3, s.world3.Substeps
		for i, b := range s.world3.Bodies() {
			body := RenderBody{ID: i, Position: vector(b.Position), Rotation: Rotation{b.Rotation.X, b.Rotation.Y, b.Rotation.Z, b.Rotation.W}, Static: b.Type() == physics3d.Static, Color: colors[i%len(colors)]}
			if body.Static {
				body.Color = "#344657"
			}
			switch shape := b.Shape().(type) {
			case physics3d.Sphere:
				body.Shape, body.Radius = "sphere", shape.Radius
			case physics3d.Box:
				body.Shape, body.HalfExtents = "box", vector(shape.HalfExtents)
			}
			state.Bodies = append(state.Bodies, body)
			if b.Type() == physics3d.Dynamic {
				state.Energy += 0.5*b.Mass()*b.Velocity.LengthSquared() + 0.5*b.AngularVelocity.Dot(b.AngularMomentum()) + b.Mass()*s.gravity*b.Position.Y
			}
		}
		for _, j := range s.joints3 {
			a, b := j.Anchors()
			state.Joints = append(state.Joints, RenderJoint{vector(a), vector(b)})
			state.JointError = math.Max(state.JointError, math.Abs(b.Sub(a).Length()-j.Length()))
		}
	} else {
		state.Dimension, state.Substeps = 2, s.world2.Substeps
		for i, b := range s.world2.Bodies() {
			body := RenderBody{ID: i, Position: Vector{X: b.Position.X, Y: b.Position.Y}, Rotation: Rotation{Z: math.Sin(b.Angle / 2), W: math.Cos(b.Angle / 2)}, Static: b.Type() == physics.Static, Color: colors[i%len(colors)]}
			if body.Static {
				body.Color = "#344657"
			}
			switch shape := b.Shape().(type) {
			case physics.Circle:
				body.Shape, body.Radius = "circle", shape.Radius
			case physics.Box:
				body.Shape, body.HalfExtents = "box", Vector{X: shape.HalfExtents.X, Y: shape.HalfExtents.Y}
			}
			state.Bodies = append(state.Bodies, body)
			if b.Type() == physics.Dynamic {
				state.Energy += 0.5*b.Mass()*b.Velocity.LengthSquared() + 0.5*b.Inertia()*b.AngularVelocity*b.AngularVelocity + b.Mass()*s.gravity*b.Position.Y
			}
		}
	}
	state.BodyCount = len(state.Bodies)
	return state
}

func finite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }

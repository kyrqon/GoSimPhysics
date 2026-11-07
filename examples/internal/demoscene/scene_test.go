package demoscene

import (
	"encoding/json"
	"math"
	"testing"
)

func TestScenesRunRealEnginesAndEncodeFiniteState(t *testing.T) {
	for _, name := range []string{"stack", "rain", "pendulum", "billiards2d"} {
		t.Run(name, func(t *testing.T) {
			session, err := New(name)
			if err != nil {
				t.Fatal(err)
			}
			before := session.Snapshot()
			for frame := 0; frame < 180; frame++ {
				if err := session.Step(1.0 / 60); err != nil {
					t.Fatalf("frame %d: %v", frame, err)
				}
			}
			after := session.Snapshot()
			if after.Time <= before.Time || after.BodyCount != len(after.Bodies) || after.BodyCount == 0 {
				t.Fatal("scene did not advance or has inconsistent body metadata")
			}
			moved := false
			for i, body := range after.Bodies {
				if !body.Static && body.Position != before.Bodies[i].Position {
					moved = true
				}
				q := body.Rotation
				if math.Abs(q.X*q.X+q.Y*q.Y+q.Z*q.Z+q.W*q.W-1) > 1e-10 {
					t.Fatal("renderer received a nonunit rotation")
				}
			}
			if !moved {
				t.Fatal("scene bodies never moved")
			}
			if name == "pendulum" && (len(after.Joints) != 1 || after.JointError > 0.01) {
				t.Fatalf("pendulum constraint failed: %+v", after.Joints)
			}
			if _, err := json.Marshal(after); err != nil {
				t.Fatalf("cannot encode browser snapshot: %v", err)
			}
			if name == "billiards2d" && after.Dimension != 2 || name != "billiards2d" && after.Dimension != 3 {
				t.Fatal("wrong simulation dimension")
			}
		})
	}
}

func TestSceneControlsValidateAndBoundSpawning(t *testing.T) {
	for _, name := range []string{"stack", "billiards2d"} {
		t.Run(name, func(t *testing.T) {
			s, err := New(name)
			if err != nil {
				t.Fatal(err)
			}
			initial := s.Snapshot()
			if s.Configure(math.NaN(), 4) == nil || s.Configure(10, 0) == nil || s.Configure(31, 4) == nil || s.Configure(10, 9) == nil {
				t.Fatal("invalid world settings accepted")
			}
			if s.Snapshot().Gravity != initial.Gravity || s.Snapshot().Substeps != initial.Substeps {
				t.Fatal("invalid settings partially changed the scene")
			}
			if err := s.Configure(5, 2); err != nil {
				t.Fatal(err)
			}
			if s.Snapshot().Gravity != 5 || s.Snapshot().Substeps != 2 {
				t.Fatal("valid controls did not reach the Go world")
			}
			if s.Step(0) == nil || s.Step(0.2) == nil || s.Step(math.NaN()) == nil || s.Snapshot().Time != 0 {
				t.Fatal("invalid browser step advanced the simulation")
			}
			for _, kind := range []string{"sphere", "box"} {
				if err := s.Spawn(kind); err != nil {
					t.Fatal(err)
				}
			}
			if s.Snapshot().BodyCount != initial.BodyCount+2 {
				t.Fatal("spawn did not add physics bodies")
			}
			if err := s.Kick(); err != nil {
				t.Fatal(err)
			}
			if s.Spawn("invalid") == nil {
				t.Fatal("invalid shape accepted")
			}
			for s.dynamicCount() < 40 {
				if err := s.Spawn("sphere"); err != nil {
					t.Fatal(err)
				}
			}
			count := s.Snapshot().BodyCount
			if s.Spawn("box") == nil || s.Snapshot().BodyCount != count {
				t.Fatal("spawn limit did not preserve the scene")
			}
		})
	}
	if session, err := New("unknown"); err == nil || session != nil {
		t.Fatal("unknown scene accepted")
	}
}

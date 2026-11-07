//go:build js && wasm

package main

import (
	"encoding/json"
	"fmt"
	"syscall/js"

	"github.com/kyrqon/GoSimPhysics/examples/internal/demoscene"
)

var callbacks []js.Func

func main() {
	session, err := demoscene.New("stack")
	if err != nil {
		panic(err)
	}
	bridge := js.Global().Get("Object").New()
	register := func(name string, fn func([]js.Value) (any, error)) {
		callback := js.FuncOf(func(_ js.Value, args []js.Value) (result any) {
			defer func() {
				if recovered := recover(); recovered != nil {
					result = response(nil, fmt.Errorf("demo: %v", recovered))
				}
			}()
			value, err := fn(args)
			return response(value, err)
		})
		callbacks = append(callbacks, callback)
		bridge.Set(name, callback)
	}
	register("state", func([]js.Value) (any, error) { return session.Snapshot(), nil })
	register("reset", func(args []js.Value) (any, error) {
		if len(args) != 1 || args[0].Type() != js.TypeString {
			return nil, fmt.Errorf("reset requires a scene name")
		}
		next, err := demoscene.New(args[0].String())
		if err != nil {
			return nil, err
		}
		session = next
		return session.Snapshot(), nil
	})
	register("step", func(args []js.Value) (any, error) {
		if len(args) != 1 || args[0].Type() != js.TypeNumber {
			return nil, fmt.Errorf("step requires a duration in seconds")
		}
		if err := session.Step(args[0].Float()); err != nil {
			return nil, err
		}
		return session.Snapshot(), nil
	})
	register("spawn", func(args []js.Value) (any, error) {
		if len(args) != 1 || args[0].Type() != js.TypeString {
			return nil, fmt.Errorf("spawn requires a shape name")
		}
		if err := session.Spawn(args[0].String()); err != nil {
			return nil, err
		}
		return session.Snapshot(), nil
	})
	register("kick", func([]js.Value) (any, error) {
		if err := session.Kick(); err != nil {
			return nil, err
		}
		return session.Snapshot(), nil
	})
	register("configure", func(args []js.Value) (any, error) {
		if len(args) != 1 || args[0].Type() != js.TypeString {
			return nil, fmt.Errorf("configure requires a JSON object")
		}
		var options struct {
			Gravity  *float64 `json:"gravity"`
			Substeps *int     `json:"substeps"`
		}
		if err := json.Unmarshal([]byte(args[0].String()), &options); err != nil {
			return nil, fmt.Errorf("invalid settings: %w", err)
		}
		current := session.Snapshot()
		gravity, substeps := current.Gravity, current.Substeps
		if options.Gravity != nil {
			gravity = *options.Gravity
		}
		if options.Substeps != nil {
			substeps = *options.Substeps
		}
		if err := session.Configure(gravity, substeps); err != nil {
			return nil, err
		}
		return session.Snapshot(), nil
	})
	js.Global().Set("gosim", bridge)
	js.Global().Call("dispatchEvent", js.Global().Get("Event").New("gosim-ready"))
	select {}
}

func response(value any, err error) string {
	if err != nil {
		value = map[string]string{"error": err.Error()}
	}
	data, marshalErr := json.Marshal(value)
	if marshalErr != nil {
		data, _ = json.Marshal(map[string]string{"error": "cannot encode simulation state: " + marshalErr.Error()})
	}
	return string(data)
}

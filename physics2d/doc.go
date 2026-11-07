// Package physics2d provides two-dimensional game physics: vectors, circles and
// oriented boxes, pure collision and sweep queries, rigid bodies, distance
// constraints, and optional continuous collision detection.
//
// Use Collide, TimeOfImpact, and RayInterval directly with Collider and Motion
// values for queries without a simulation world. Use World to integrate bodies,
// forces, constraints, and collision response; Body query methods respect static
// and fixed-rotation motion settings. Body.Collider and Body.Motion expose value
// snapshots for queries and optional gameplay extensions.
//
// Use consistent units and drive World.Step with a fixed time step, such as
// 1.0/60 seconds. Substeps subdivides each call rather than setting an independent
// tick rate. Worlds and bodies are not safe for concurrent mutation.
package physics2d

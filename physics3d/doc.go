// Package physics3d provides three-dimensional game physics: vectors,
// quaternions, sphere and box geometry, pure collision and sweep queries,
// rigid bodies, forces, distance constraints, and optional continuous collision
// detection. Geometry queries do not require a simulation world.
//
// Use consistent units and drive World.Step with a fixed time step, such as
// 1.0/60 seconds. Substeps subdivides each call rather than setting a tick rate.
// World and Body values are not safe for concurrent mutation.
package physics3d

// Package dynamics3d simulates three-dimensional rigid bodies with forces,
// collision response, distance constraints, and optional continuous collision
// detection. It builds on collision3d and math3d. Use consistent units and drive
// World.Step with a fixed time step, such as 1.0/60 seconds; Substeps subdivides
// each call and does not set an independent tick rate. World and Body values are
// not safe for concurrent mutation.
package dynamics3d

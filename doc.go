// Package physics provides 2D rigid-body simulation for embedding in game engines.
//
// A World owns static, dynamic, and kinematic bodies with one centered Circle or
// Box each. It integrates forces and velocities, resolves contacts with friction
// and restitution, and solves rigid distance joints. The host application owns
// rendering, input, and its game loop.
//
// Use consistent units such as meters, kilograms, seconds, and radians. Angles
// increase counterclockwise; the examples use positive Y upward and negative Y
// gravity. Advance a world with a fixed positive time step. NewWorld supplies
// default substep and solver settings; Dynamic bodies require positive mass.
//
// Forces and torques accumulate over all substeps of the next successful Step
// and are then cleared. Impulses change velocities immediately. Public body state
// may be edited between steps, but must remain finite; changing a transform
// directly teleports the body. Check errors returned by creation, force, impulse,
// and stepping methods. A failed Step preserves body state and accumulated forces.
//
// The simulation uses semi-implicit Euler integration and approximate iterative
// constraints. Collision detection is discrete, and the broad phase scans all
// body pairs with AABB rejection. Fast bodies can tunnel through thin shapes.
// There is no continuous collision detection, sleeping, 3D simulation, or
// cross-platform bitwise determinism guarantee. A world and its bodies must be
// accessed from one goroutine at a time, and bodies must not be copied.
package physics

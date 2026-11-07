// Package collision3d provides pure three-dimensional collision and constant-
// velocity sweep queries for spheres and oriented boxes. It depends only on
// math3d, so queries can be used without rigid bodies or a simulation world.
// Use consistent units; rotations and angular velocities use radians. A zero
// Collider rotation means identity. Checked queries report invalid geometry,
// overflow, and convergence failure instead of silently treating them as misses.
package collision3d

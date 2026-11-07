// Package physics3d provides 3D rigid-body simulation for embedding in game
// engines, alongside the module's independent 2D physics package.
//
// A World contains static, dynamic, or kinematic bodies with one centered Sphere
// or oriented Box each. Shapes have mass and local principal inertia; rotations
// use quaternions. The solver integrates world-space angular momentum and uses
// the full rotated inertia tensor for forces, impulses, contacts, and joints.
//
// Use consistent units, for example meters, kilograms, seconds, and radians, with
// right-handed coordinates. Forces, torques, velocities, and impulse application
// points use world coordinates. Distance-joint anchors use local coordinates.
// A zero Quaternion in BodyDef defaults to identity. Advance with a fixed positive
// step such as 1/60 second; NewWorld supplies substep and solver defaults.
//
// Forces accumulate until the next successful Step and act through every
// substep; impulses change velocity immediately. Check errors from creation,
// applied forces or impulses, and stepping. Failed steps preserve body state and
// pending forces. Public body state may be edited between steps, with finite
// coordinates and nonzero finite quaternions; direct position edits teleport.
//
// Collision detection uses sphere tests and 15-axis oriented-box SAT with face
// clipping or edge contacts. Enable BodyDef.Continuous for swept detection on
// fast bodies, or use TimeOfImpact for a pure constant-velocity motion query.
// CCD consumes remaining time after impacts and has a finite MaxTOIEvents
// budget; failed steps preserve all body state and pending forces. Its motion
// segments use constant world angular velocity with momentum recovery, rather
// than the default midpoint rotational approximation. Initial overlap and
// position correction remain discrete, and CCD has shape-scaled tolerances.
// The broad phase scans pairs with AABB rejection, intended for small scenes.
// Fast bodies without CCD may tunnel through thin geometry. There is no sleeping,
// compound shapes, concurrent-world access, or cross-platform bitwise determinism
// guarantee. The host application supplies rendering and the game loop.
package physics3d

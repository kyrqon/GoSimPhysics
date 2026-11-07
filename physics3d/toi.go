package physics3d

import (
	"fmt"
	"math"
	"sort"
)

// TOIResult describes the first contact in a motion query. Time is measured in
// seconds from the supplied initial poses, rather than as a fraction of dt.
type TOIResult struct {
	Time    float64
	Contact Contact
}

// TimeOfImpact sweeps two bodies over dt seconds without changing either body.
// Linear and world-space angular velocities remain constant during the query;
// Static bodies stay fixed. Initial overlap or contact returns Time zero.
//
// Sphere and fixed-orientation box sweeps use analytic translational casts.
// Rotating boxes use conservative advancement with angular motion bounds.
// Contacts have a spatial tolerance of 1e-9 times the smaller shape feature.
// Invalid inputs, unrepresentable geometry, and convergence exhaustion return an
// error, so a caller can avoid treating numerical failure as a missed collision.
func TimeOfImpact(a, b *Body, dt float64) (TOIResult, bool, error) {
	if !finite(dt) || dt <= 0 || a == nil || b == nil || !a.validState() || !b.validState() {
		return TOIResult{}, false, fmt.Errorf("physics3d: TOI requires valid bodies and positive finite dt")
	}
	if _, valid := copyShape(a.shape); !valid {
		return TOIResult{}, false, fmt.Errorf("physics3d: invalid first TOI shape")
	}
	if _, valid := copyShape(b.shape); !valid {
		return TOIResult{}, false, fmt.Errorf("physics3d: invalid second TOI shape")
	}
	// Validate the entire requested displacement, even when an early hit would
	// otherwise hide an overflowing endpoint from the caller.
	for _, body := range []*Body{a, b} {
		if _, err := toiPose(body, dt); err != nil {
			return TOIResult{}, false, err
		}
	}
	gap, normal, err := toiSeparation(a, b)
	if err != nil {
		return TOIResult{}, false, err
	}
	if gap <= toiDistanceTolerance(a, b) {
		return toiResult(a, b, 0, normal)
	}
	if sa, ok := a.shape.(Sphere); ok {
		if sb, ok := b.shape.(Sphere); ok {
			return toiSpheres(a, sa, b, sb, dt)
		}
		if sb, ok := b.shape.(Box); ok && !toiRotates(b) {
			return toiSphereBox(a, sa, b, sb, dt, false)
		}
	}
	if sa, ok := a.shape.(Box); ok {
		if sb, ok := b.shape.(Sphere); ok && !toiRotates(a) {
			return toiSphereBox(b, sb, a, sa, dt, true)
		}
		if sb, ok := b.shape.(Box); ok && !toiRotates(a) && !toiRotates(b) {
			return toiBoxes(a, sa, b, sb, dt)
		}
	}
	return toiConservative(a, b, dt)
}

func toiVelocity(b *Body) Vec3 {
	if b.kind == Static {
		return Vec3{}
	}
	return b.Velocity
}

func toiRotates(b *Body) bool {
	_, box := b.shape.(Box)
	return box && b.kind != Static && !b.fixedRotation && b.AngularVelocity != (Vec3{})
}

// toiPose shares the query's constant-world-omega motion model with the CCD
// world stepper. The returned body is a private copy, not a world member.
func toiPose(b *Body, t float64) (*Body, error) {
	if b == nil || !finite(t) || t < 0 || !b.validState() {
		return nil, fmt.Errorf("physics3d: invalid TOI pose input")
	}
	pose := *b
	pose.Rotation = b.Rotation.Normalized()
	if b.kind != Static {
		displacement := b.Velocity.Mul(t)
		angular := Vec3{}
		if !b.fixedRotation {
			angular = b.AngularVelocity.Mul(t)
		}
		if !validVec(displacement) || !validVec(angular) || !finite(angular.Length()) {
			return nil, fmt.Errorf("physics3d: TOI motion overflow")
		}
		pose.Position = b.Position.Add(displacement)
		pose.Rotation = integrateRotation(b.Rotation, angular)
	}
	if !pose.validState() || !bodyBoundsFinite(&pose) {
		return nil, fmt.Errorf("physics3d: TOI transform or bounds overflow")
	}
	return &pose, nil
}

func toiFeature(b *Body) float64 {
	switch s := b.shape.(type) {
	case Sphere:
		return s.Radius
	case Box:
		return math.Min(s.HalfExtents.X, math.Min(s.HalfExtents.Y, s.HalfExtents.Z))
	}
	return 0
}

func toiDistanceTolerance(a, b *Body) float64 {
	return math.Max(1e-9*math.Min(toiFeature(a), toiFeature(b)), math.SmallestNonzeroFloat64*64)
}

func toiAngularBound(b *Body) float64 {
	if !toiRotates(b) {
		return 0
	}
	return b.AngularVelocity.Length() * b.shape.(Box).HalfExtents.Length()
}

func toiConservative(a, b *Body, dt float64) (TOIResult, bool, error) {
	relative := toiVelocity(b).Sub(toiVelocity(a))
	angular := toiAngularBound(a) + toiAngularBound(b)
	if !validVec(relative) || !finite(angular) {
		return TOIResult{}, false, fmt.Errorf("physics3d: TOI motion bound overflow")
	}
	t, tolerance := 0.0, toiDistanceTolerance(a, b)
	lastGap, lastPlane := 0.0, 0.0
	for iteration := 0; iteration < 256; iteration++ {
		pa, err := toiPose(a, t)
		if err != nil {
			return TOIResult{}, false, err
		}
		pb, err := toiPose(b, t)
		if err != nil {
			return TOIResult{}, false, err
		}
		gap, normal, err := toiSeparation(pa, pb)
		if err != nil {
			return TOIResult{}, false, err
		}
		if gap <= tolerance {
			return toiResult(a, b, t, normal)
		}
		closing := -relative.Dot(normal) + angular
		if !finite(closing) {
			return TOIResult{}, false, fmt.Errorf("physics3d: TOI closing speed overflow")
		}
		if closing <= 0 {
			return TOIResult{}, false, nil
		}
		// A separating support plane cannot close by gap sooner than this
		// bound, even when both boxes rotate. Recompute it after every advance.
		advance := gap / closing
		// The speed-only bound becomes excessively pessimistic when a contact
		// is tangent and opening: its gap grows quadratically, while |omega|R
		// remains large. A fixed-plane, second-order bound also certifies a
		// safe interval and accounts for every possible support vertex.
		lastGap, lastPlane = gap, math.Max(toiPlaneAdvance(pa, pb, normal), toiMovingFaceAdvance(pa, pb, dt-t))
		advance = math.Max(advance, lastPlane)
		if advance > dt-t {
			return TOIResult{}, false, nil
		}
		next := t + advance*0.95
		if next <= t || !finite(next) {
			return TOIResult{}, false, fmt.Errorf("physics3d: TOI time cannot make representable progress")
		}
		t = next
	}
	return TOIResult{}, false, fmt.Errorf("physics3d: TOI conservative advancement did not converge (time %.12g, gap %.12g, plane advance %.12g)", t, lastGap, lastPlane)
}

func toiMovingFaceAdvance(a, b *Body, horizon float64) float64 {
	return math.Max(toiFacePlaneAdvance(a, b, horizon), toiFacePlaneAdvance(b, a, horizon))
}

func toiFacePlaneAdvance(reference, incident *Body, horizon float64) float64 {
	box, ok := reference.shape.(Box)
	if !ok {
		return 0
	}
	delta := incident.Position.Sub(reference.Position)
	velocity := toiVelocity(incident).Sub(toiVelocity(reference))
	_, _, omega := toiMotionVertices(reference)
	vertices, radius, incidentOmega := toiMotionVertices(incident)
	speed, angular := velocity.Length(), omega.Length()
	difference := incidentOmega.Sub(omega)
	acceleration := angular*angular*(delta.Length()+speed*horizon) + 2*angular*speed
	// The dot product of two co-rotating vectors is constant. Bounding by
	// relative angular velocity preserves this useful cancellation, instead of
	// pretending distant corners close at the full sum of their spin speeds.
	if _, box := incident.shape.(Box); box {
		acceleration += difference.Length() * (angular + incidentOmega.Length()) * radius
	}
	if !finite(acceleration) {
		return 0
	}
	sphereRadius := 0.0
	if sphere, ok := incident.shape.(Sphere); ok {
		sphereRadius = sphere.Radius
	}
	advance := 0.0
	for i, axis := range boxAxes(reference.Rotation) {
		projection := axis.Dot(delta)
		if projection < 0 {
			axis = axis.Mul(-1)
		}
		bound := math.Inf(1)
		for _, vertex := range vertices {
			gap := axis.Dot(delta.Add(vertex)) - collisionComponent(box.HalfExtents, i) - sphereRadius
			if gap <= 0 {
				bound = 0 // This face does not separate the complete incident shape.
				break
			}
			derivative := omega.Cross(axis).Dot(delta) + axis.Dot(velocity) + axis.Dot(difference.Cross(vertex))
			bound = math.Min(bound, toiQuadraticAdvance(gap, derivative, acceleration))
		}
		advance = math.Max(advance, bound)
	}
	return advance
}

func toiQuadraticAdvance(gap, derivative, acceleration float64) float64 {
	if !finite(gap) || !finite(derivative) || gap < 0 || !finite(acceleration) || acceleration < 0 {
		return 0
	}
	if acceleration == 0 {
		if derivative < 0 {
			return gap / -derivative
		}
		return math.Inf(1)
	}
	discriminant := derivative*derivative + 2*acceleration*gap
	if !finite(discriminant) {
		return 0
	}
	square := math.Sqrt(discriminant)
	if derivative < 0 {
		return 2 * gap / (square - derivative)
	}
	return (derivative + square) / acceleration
}

func toiPlaneAdvance(a, b *Body, normal Vec3) float64 {
	verticesA, radiusA, omegaA := toiMotionVertices(a)
	verticesB, radiusB, omegaB := toiMotionVertices(b)
	acceleration := omegaA.LengthSquared()*radiusA + omegaB.LengthSquared()*radiusB
	if !finite(acceleration) {
		return 0 // The first-order bound remains valid when this bound overflows.
	}
	delta := b.Position.Sub(a.Position)
	velocity := toiVelocity(b).Sub(toiVelocity(a))
	surfaceRadiusA, surfaceRadiusB := 0.0, 0.0
	if sphere, ok := a.shape.(Sphere); ok {
		surfaceRadiusA = sphere.Radius
	}
	if sphere, ok := b.shape.(Sphere); ok {
		surfaceRadiusB = sphere.Radius
	}
	advance := math.Inf(1)
	for _, rA := range verticesA {
		for _, rB := range verticesB {
			gap := normal.Dot(delta.Add(rB).Sub(rA)) - surfaceRadiusA - surfaceRadiusB
			derivative := normal.Dot(velocity.Add(omegaB.Cross(rB)).Sub(omegaA.Cross(rA)))
			if !finite(gap) || !finite(derivative) || gap < 0 {
				return 0
			}
			root := toiQuadraticAdvance(gap, derivative, acceleration)
			advance = math.Min(advance, root)
		}
	}
	return advance
}

func toiMotionVertices(b *Body) ([]Vec3, float64, Vec3) {
	if sphere, ok := b.shape.(Sphere); ok {
		return []Vec3{{}}, sphere.Radius, Vec3{}
	}
	box := b.shape.(Box)
	axes := boxAxes(b.Rotation)
	vertices := make([]Vec3, 0, 8)
	for x := -1.0; x <= 1; x += 2 {
		for y := -1.0; y <= 1; y += 2 {
			for z := -1.0; z <= 1; z += 2 {
				vertices = append(vertices, axes[0].Mul(x*box.HalfExtents.X).Add(axes[1].Mul(y*box.HalfExtents.Y)).Add(axes[2].Mul(z*box.HalfExtents.Z)))
			}
		}
	}
	omega := Vec3{}
	if toiRotates(b) {
		omega = b.AngularVelocity
	}
	return vertices, box.HalfExtents.Length(), omega
}

func toiSpheres(a *Body, sa Sphere, b *Body, sb Sphere, dt float64) (TOIResult, bool, error) {
	delta := b.Position.Sub(a.Position)
	displacement := toiVelocity(b).Sub(toiVelocity(a)).Mul(dt)
	radius := sa.Radius + sb.Radius
	if !validVec(delta) || !validVec(displacement) || !finite(radius) {
		return TOIResult{}, false, fmt.Errorf("physics3d: sphere TOI geometry overflow")
	}
	u, hit, err := toiRaySphere(delta, displacement, radius)
	if err != nil || !hit {
		return TOIResult{}, false, err
	}
	return toiResult(a, b, u*dt, Vec3{})
}

// toiRaySphere returns the first fraction in [0,1] where p+u*d reaches a sphere.
// It uses closest approach rather than a subtractive quadratic discriminant.
func toiRaySphere(p, d Vec3, radius float64) (float64, bool, error) {
	scale := math.Max(radius, math.Max(p.Length(), d.Length()))
	if !finite(scale) || scale <= 0 {
		return 0, false, fmt.Errorf("physics3d: swept quadratic overflow")
	}
	p = Vec3{p.X / scale, p.Y / scale, p.Z / scale}
	d = Vec3{d.X / scale, d.Y / scale, d.Z / scale}
	radius /= scale
	if p.Length() <= radius {
		return 0, true, nil
	}
	speed := d.Length()
	if speed == 0 {
		return 0, false, nil
	}
	axis := d.Normalized()
	projection := p.Dot(axis)
	if projection >= 0 {
		return 0, false, nil
	}
	perpendicular := p.Sub(axis.Mul(projection)).Length()
	if perpendicular > radius {
		// A mathematically tangent decimal coordinate (for example 1.1-1)
		// can round just outside its radius. Admit only a few ulps of radial
		// roundoff; toiResult still verifies the actual unscaled geometry.
		if perpendicular-radius > radius*(64*2.220446049250313e-16) {
			return 0, false, nil
		}
		perpendicular = radius
	}
	distance := -projection - math.Sqrt(math.Max(0, (radius-perpendicular)*(radius+perpendicular)))
	u := distance / speed
	const fractionRoundoff = 64 * 2.220446049250313e-16
	return clamp(u, 0, 1), u >= -fractionRoundoff && u <= 1+fractionRoundoff, nil
}

func toiSphereBox(sphere *Body, ss Sphere, box *Body, bs Box, dt float64, reverse bool) (TOIResult, bool, error) {
	rotation := box.Rotation.Normalized().Conjugate()
	p := rotation.Rotate(sphere.Position.Sub(box.Position))
	d := rotation.Rotate(toiVelocity(sphere).Sub(toiVelocity(box)).Mul(dt))
	if !validVec(p) || !validVec(d) {
		return TOIResult{}, false, fmt.Errorf("physics3d: sphere-box TOI transform overflow")
	}
	// Distance from a moving point to a box is a quadratic on every interval
	// between slab crossings. Solving each interval handles rounded edges and
	// corners exactly rather than casting against an oversized expanded AABB.
	intervals := []float64{0, 1}
	for axis := 0; axis < 3; axis++ {
		position, velocity, half := collisionComponent(p, axis), collisionComponent(d, axis), collisionComponent(bs.HalfExtents, axis)
		if velocity == 0 {
			continue
		}
		for _, side := range []float64{-half, half} {
			u := (side - position) / velocity
			if !finite(u) {
				// An infinite crossing is simply outside this finite interval.
				continue
			}
			if u > 0 && u < 1 {
				intervals = append(intervals, u)
			}
		}
	}
	sort.Float64s(intervals)
	for i := 0; i+1 < len(intervals); i++ {
		lo, hi := intervals[i], intervals[i+1]
		if lo == hi {
			continue
		}
		mid := p.Add(d.Mul((lo + hi) * 0.5))
		start, movement := Vec3{}, Vec3{}
		for axis := 0; axis < 3; axis++ {
			value, half := collisionComponent(mid, axis), collisionComponent(bs.HalfExtents, axis)
			if value >= -half && value <= half {
				continue
			}
			side := half
			if value < 0 {
				side = -half
			}
			start = collisionSetComponent(start, axis, collisionComponent(p, axis)+collisionComponent(d, axis)*lo-side)
			movement = collisionSetComponent(movement, axis, collisionComponent(d, axis)*(hi-lo))
		}
		u, hit, err := toiRaySphere(start, movement, ss.Radius)
		if err != nil {
			return TOIResult{}, false, err
		}
		if hit {
			time := (lo + u*(hi-lo)) * dt
			if reverse {
				return toiResult(box, sphere, time, Vec3{})
			}
			return toiResult(sphere, box, time, Vec3{})
		}
	}
	return TOIResult{}, false, nil
}

func toiBoxes(a *Body, sa Box, b *Body, sb Box, dt float64) (TOIResult, bool, error) {
	axesA, axesB := boxAxes(a.Rotation), boxAxes(b.Rotation)
	delta := b.Position.Sub(a.Position)
	displacement := toiVelocity(b).Sub(toiVelocity(a)).Mul(dt)
	if !validVec(delta) || !validVec(displacement) {
		return TOIResult{}, false, fmt.Errorf("physics3d: box TOI displacement overflow")
	}
	enter, exit := 0.0, 1.0
	var normal Vec3
	for _, axis := range toiBoxAxes(axesA, axesB) {
		p, v := delta.Dot(axis), displacement.Dot(axis)
		radius := boxProjection(sa.HalfExtents, axesA, axis) + boxProjection(sb.HalfExtents, axesB, axis)
		if !finite(p) || !finite(v) || !finite(radius) {
			return TOIResult{}, false, fmt.Errorf("physics3d: swept box projection overflow")
		}
		if v == 0 {
			if math.Abs(p) > radius {
				return TOIResult{}, false, nil
			}
			continue
		}
		scale := math.Max(radius, math.Max(math.Abs(p), math.Abs(v)))
		p, v, radius = p/scale, v/scale, radius/scale
		first, last := (-radius-p)/v, (radius-p)/v
		if first > last {
			first, last = last, first
		}
		if first > enter {
			enter, normal = first, axis
			if p+v*first < 0 {
				normal = normal.Mul(-1)
			}
		}
		exit = math.Min(exit, last)
		if enter > exit {
			return TOIResult{}, false, nil
		}
	}
	if enter < 0 || enter > 1 || exit < 0 {
		return TOIResult{}, false, nil
	}
	return toiResult(a, b, enter*dt, normal)
}

func toiBoxAxes(a, b [3]Vec3) []Vec3 {
	axes := []Vec3{a[0], a[1], a[2], b[0], b[1], b[2]}
	for i := range a {
		for j := range b {
			cross := a[i].Cross(b[j])
			if cross.LengthSquared() > 1e-20 {
				axes = append(axes, cross.Normalized())
			}
		}
	}
	return axes
}

func toiSeparation(a, b *Body) (float64, Vec3, error) {
	if contact, hit, err := collideChecked(a, b); err != nil {
		return 0, Vec3{}, err
	} else if hit {
		return -contact.Penetration, contact.Normal, nil
	}
	switch sa := a.shape.(type) {
	case Sphere:
		switch sb := b.shape.(type) {
		case Sphere:
			delta := b.Position.Sub(a.Position)
			gap := delta.Length() - sa.Radius - sb.Radius
			return gap, delta.Normalized(), nil
		case Box:
			return toiSphereBoxSeparation(a, sa, b, sb)
		}
	case Box:
		switch sb := b.shape.(type) {
		case Sphere:
			gap, normal, err := toiSphereBoxSeparation(b, sb, a, sa)
			return gap, normal.Mul(-1), err
		case Box:
			axesA, axesB := boxAxes(a.Rotation), boxAxes(b.Rotation)
			delta := b.Position.Sub(a.Position)
			gap := -math.MaxFloat64
			var normal Vec3
			for _, axis := range toiBoxAxes(axesA, axesB) {
				projection := delta.Dot(axis)
				separation := math.Abs(projection) - boxProjection(sa.HalfExtents, axesA, axis) - boxProjection(sb.HalfExtents, axesB, axis)
				if !finite(separation) {
					return 0, Vec3{}, fmt.Errorf("physics3d: TOI separating axis overflow")
				}
				if separation > gap {
					gap, normal = separation, axis
					if projection < 0 {
						normal = normal.Mul(-1)
					}
				}
			}
			return gap, normal, nil
		}
	}
	return 0, Vec3{}, fmt.Errorf("physics3d: unsupported TOI shapes")
}

func toiSphereBoxSeparation(sphere *Body, ss Sphere, box *Body, bs Box) (float64, Vec3, error) {
	rotation := box.Rotation.Normalized()
	center := rotation.Conjugate().Rotate(sphere.Position.Sub(box.Position))
	half := bs.HalfExtents
	closest := Vec3{clamp(center.X, -half.X, half.X), clamp(center.Y, -half.Y, half.Y), clamp(center.Z, -half.Z, half.Z)}
	delta := closest.Sub(center)
	gap := delta.Length() - ss.Radius
	if !finite(gap) || !validVec(delta) || delta == (Vec3{}) {
		return 0, Vec3{}, fmt.Errorf("physics3d: unrepresentable sphere-box TOI separation")
	}
	return gap, rotation.Rotate(delta.Normalized()), nil
}

func toiResult(a, b *Body, t float64, hint Vec3) (TOIResult, bool, error) {
	pa, err := toiPose(a, t)
	if err != nil {
		return TOIResult{}, false, err
	}
	pb, err := toiPose(b, t)
	if err != nil {
		return TOIResult{}, false, err
	}
	contact, hit, err := collideChecked(pa, pb)
	if err != nil {
		return TOIResult{}, false, err
	}
	if !hit {
		gap, normal, err := toiSeparation(pa, pb)
		if err != nil {
			return TOIResult{}, false, err
		}
		if !validVec(normal) || normal == (Vec3{}) {
			normal = hint
		}
		if !finite(gap) || gap > toiDistanceTolerance(pa, pb)*4 || normal == (Vec3{}) {
			return TOIResult{}, false, fmt.Errorf("physics3d: TOI root did not reach the actual shapes")
		}
		// Close only numerical slop on a private copy to obtain the same face
		// manifold as the discrete collider, then center it on the real surfaces.
		shift := math.Max(0, gap) + toiDistanceTolerance(pa, pb)*2
		originalPosition := pa.Position
		pa.Position = pa.Position.Add(normal.Mul(shift))
		contact, hit, err = collideChecked(pa, pb)
		if err != nil {
			return TOIResult{}, false, err
		}
		if hit {
			for i := range contact.Points {
				contact.Points[i] = contact.Points[i].Sub(normal.Mul(shift * 0.5))
			}
		} else {
			// At a near-touch corner there can be several small separating
			// axes; closing only the winning axis cannot create a manifold.
			// Use actual closest features of the ORIGINAL poses, not a box
			// expanded in every direction (which would introduce false hits).
			pa.Position = originalPosition
			pointA, pointB, ok := toiClosestBoxWitnesses(pa, pb)
			if !ok || pointB.Sub(pointA).Length() > toiDistanceTolerance(pa, pb)*4 {
				return TOIResult{}, false, fmt.Errorf("physics3d: TOI contact manifold could not be represented")
			}
			direction := pointB.Sub(pointA).Normalized()
			if direction == (Vec3{}) {
				direction = normal
			}
			contact = Contact{Normal: direction, Points: []Vec3{collisionMidpoint(pointA, pointB)}}
		}
		contact.Penetration = 0
	}
	if !contactFinite(contact) {
		return TOIResult{}, false, fmt.Errorf("physics3d: invalid TOI contact")
	}
	return TOIResult{Time: t, Contact: contact}, true, nil
}

func toiClosestBoxWitnesses(a, b *Body) (Vec3, Vec3, bool) {
	boxA, okA := a.shape.(Box)
	boxB, okB := b.shape.(Box)
	if !okA || !okB {
		return Vec3{}, Vec3{}, false
	}
	rA, _, _ := toiMotionVertices(a)
	rB, _, _ := toiMotionVertices(b)
	verticesA, verticesB := make([]Vec3, 8), make([]Vec3, 8)
	for i := range verticesA {
		verticesA[i], verticesB[i] = a.Position.Add(rA[i]), b.Position.Add(rB[i])
	}
	closest := func(point Vec3, body *Body, shape Box) Vec3 {
		local := body.Rotation.Conjugate().Rotate(point.Sub(body.Position))
		h := shape.HalfExtents
		return body.WorldPoint(Vec3{clamp(local.X, -h.X, h.X), clamp(local.Y, -h.Y, h.Y), clamp(local.Z, -h.Z, h.Z)})
	}
	best := math.Inf(1)
	var pointA, pointB Vec3
	consider := func(pa, pb Vec3) {
		distance := pb.Sub(pa).Length()
		if validVec(pa) && validVec(pb) && finite(distance) && distance < best {
			best, pointA, pointB = distance, pa, pb
		}
	}
	for _, pa := range verticesA {
		consider(pa, closest(pa, b, boxB))
	}
	for _, pb := range verticesB {
		consider(closest(pb, a, boxA), pb)
	}
	// Vertex indices differ by a single bit along a physical edge.
	for ia, pa := range verticesA {
		for _, bitA := range []int{1, 2, 4} {
			ja := ia ^ bitA
			if ja < ia {
				continue
			}
			for ib, pb := range verticesB {
				for _, bitB := range []int{1, 2, 4} {
					jb := ib ^ bitB
					if jb < ib {
						continue
					}
					p, q := closestEdgePoints(pa, verticesA[ja], pb, verticesB[jb])
					consider(p, q)
				}
			}
		}
	}
	return pointA, pointB, finite(best)
}

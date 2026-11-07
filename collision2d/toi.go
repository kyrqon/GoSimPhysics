package collision2d

import (
	"errors"
	"fmt"
	"math"
)

// ErrTOINoConvergence means a bounded time-of-impact search could not establish
// either contact or separation. Callers must not treat this error as a miss.
var ErrTOINoConvergence = errors.New("physics: time-of-impact search did not converge")

// TOIResult describes the first contact in a constant-velocity sweep. Time is
// measured in seconds from the supplied bodies' current transforms. The contact
// may be separated by a numerical tolerance proportional to the smallest shape
// feature (circle radius or box half-extent), rather than its circumradius.
type TOIResult struct {
	Time    float64
	Contact Contact
}

const toiTolerance = 1e-8

// timeOfImpactProxies sweeps two shapes using constant linear and angular velocities.
// Initial overlap or touching returns Time zero, including separating contacts.
// Translating primitive casts are analytic; rotating shapes use bounded
// conservative advancement. A numeric or convergence failure returns an error,
// rather than being reported as an ordinary miss.
func timeOfImpactProxies(a, b *proxy, dt float64) (TOIResult, bool, error) {
	if !finite(dt) || dt <= 0 {
		return TOIResult{}, false, fmt.Errorf("physics: sweep duration must be positive and finite")
	}
	a, err := toiPrepare(a)
	if err != nil {
		return TOIResult{}, false, err
	}
	b, err = toiPrepare(b)
	if err != nil {
		return TOIResult{}, false, err
	}
	// Validate the complete requested motion, even if an earlier contact exists.
	if _, err := toiPose(a, dt); err != nil {
		return TOIResult{}, false, err
	}
	if _, err := toiPose(b, dt); err != nil {
		return TOIResult{}, false, err
	}
	if !collisionGeometryFinite(a, b) {
		return TOIResult{}, false, fmt.Errorf("physics: sweep geometry overflow")
	}
	if contact, hit := collideProxies(a, b); hit {
		return TOIResult{Contact: contact}, true, nil
	}

	linear := true
	if _, box := a.shape.(Box); box && a.AngularVelocity != 0 {
		linear = false
	}
	if _, box := b.shape.(Box); box && b.AngularVelocity != 0 {
		linear = false
	}
	if linear {
		fraction, hit, err := toiLinearFraction(a, b, dt)
		if err != nil || !hit {
			return TOIResult{}, false, err
		}
		time := clamp(fraction, 0, 1) * dt
		contact, err := toiContactAt(a, b, time)
		if err != nil {
			return TOIResult{}, false, err
		}
		return TOIResult{Time: time, Contact: contact}, true, nil
	}
	return toiConservative(a, b, dt, 256)
}

func toiConservative(a, b *proxy, dt float64, maxIterations int) (TOIResult, bool, error) {
	angularBound := toiAngularBound(a) + toiAngularBound(b)
	relative := b.Velocity.Sub(a.Velocity)
	if !finite(angularBound) || !validVec(relative) {
		return TOIResult{}, false, fmt.Errorf("physics: sweep speed bound overflow")
	}
	tolerance := toiDistanceTolerance(a, b)
	time := 0.0
	for iteration := 0; iteration < maxIterations; iteration++ {
		pa, err := toiPose(a, time)
		if err != nil {
			return TOIResult{}, false, err
		}
		pb, err := toiPose(b, time)
		if err != nil {
			return TOIResult{}, false, err
		}
		witness, err := toiSeparation(pa, pb)
		if err != nil {
			return TOIResult{}, false, err
		}
		if witness.distance <= tolerance {
			contact, err := toiContactAt(a, b, time)
			if err != nil {
				return TOIResult{}, false, err
			}
			return TOIResult{Time: time, Contact: contact}, true, nil
		}
		// The current closest-point plane remains separating until projected
		// translation plus the global rotational support bound closes its gap.
		closing := -relative.Dot(witness.normal) + angularBound
		if !finite(closing) {
			return TOIResult{}, false, fmt.Errorf("physics: sweep closing speed overflow")
		}
		if closing <= 0 {
			return TOIResult{}, false, nil
		}
		advance := witness.distance / closing
		// Near a separating or grazing contact, a global angular speed bound
		// alone can make almost no progress. Every corner pair also gives a
		// certified quadratic lower bound for this fixed separating plane.
		if curved, valid := toiCurvatureAdvance(pa, pb, witness.normal, dt-time); valid {
			advance = math.Max(advance, curved)
		}
		if moving, valid := toiMovingBoxPlaneAdvance(pa, pb, witness.normal, dt-time); valid {
			advance = math.Max(advance, moving)
		}
		if !finite(advance) {
			// A positive finite gap divided by an exceptionally small speed can
			// overflow; it still proves contact is beyond finite dt.
			return TOIResult{}, false, nil
		}
		if advance > dt-time {
			return TOIResult{}, false, nil
		}
		if advance <= 0 || time+advance <= time {
			return TOIResult{}, false, ErrTOINoConvergence
		}
		time += advance
	}
	return TOIResult{}, false, ErrTOINoConvergence
}

// toiCurvatureAdvance bounds every support-feature pair, including features
// that can replace the current closest pair after rotation. For a fixed plane,
// g(t) >= g(0)+g'(0)t-A*t*t/2, where A bounds projected angular acceleration.
func toiCurvatureAdvance(a, b *proxy, normal Vec2, duration float64) (float64, bool) {
	pointsA, radiusA := toiSupportFeatures(a)
	pointsB, radiusB := toiSupportFeatures(b)
	curvature := math.Abs(a.AngularVelocity)*toiAngularBound(a) + math.Abs(b.AngularVelocity)*toiAngularBound(b)
	if !finite(curvature) || curvature <= 0 || duration <= 0 {
		return 0, false
	}
	quadratic := (curvature * (duration * 0.5)) * duration
	if !finite(quadratic) || quadratic <= 0 {
		return 0, false
	}
	delta := b.Position.Sub(a.Position)
	relative := b.Velocity.Sub(a.Velocity)
	first := math.Inf(1)
	for _, ra := range pointsA {
		for _, rb := range pointsB {
			gap := delta.Add(rb).Sub(ra).Dot(normal) - radiusA - radiusB
			speed := relative.Add(angularVelocityAt(b.AngularVelocity, rb)).Sub(angularVelocityAt(a.AngularVelocity, ra)).Dot(normal)
			linear := speed * duration
			if !finite(gap) || gap <= 0 || !finite(linear) {
				return 0, false
			}
			scale := math.Max(gap, math.Max(math.Abs(linear), quadratic))
			g, l, q := gap/scale, linear/scale, quadratic/scale
			if g == 0 || q == 0 {
				return 0, false
			}
			discriminant := math.Hypot(l, 2*math.Sqrt(g)*math.Sqrt(q))
			var fraction float64
			if l < 0 {
				fraction = 2 * g / (discriminant - l)
			} else {
				fraction = (l + discriminant) / (2 * q)
			}
			if math.IsNaN(fraction) || fraction < 0 {
				return 0, false
			}
			first = math.Min(first, fraction*duration)
		}
	}
	return first, true
}

func toiSupportFeatures(body *proxy) ([]Vec2, float64) {
	switch shape := body.shape.(type) {
	case Circle:
		return []Vec2{{}}, shape.Radius
	case Box:
		points := make([]Vec2, 0, 4)
		for _, sx := range []float64{-1, 1} {
			for _, sy := range []float64{-1, 1} {
				points = append(points, (Vec2{X: sx * shape.HalfExtents.X, Y: sy * shape.HalfExtents.Y}).Rotate(body.Angle))
			}
		}
		return points, 0
	}
	return nil, 0
}

// A plane that rotates with a box keeps the box's projected support extent
// constant. Unlike a fixed world plane it does not let a distant face endpoint
// artificially dominate an almost-tangent circle contact.
func toiMovingBoxPlaneAdvance(a, b *proxy, normal Vec2, duration float64) (float64, bool) {
	box, circle := a, b
	shape, boxOK := box.shape.(Box)
	round, circleOK := circle.shape.(Circle)
	if !boxOK || !circleOK {
		box, circle = b, a
		shape, boxOK = box.shape.(Box)
		round, circleOK = circle.shape.(Circle)
		normal = normal.Mul(-1)
	}
	if !boxOK || !circleOK || box.AngularVelocity == 0 {
		return 0, false
	}
	delta := circle.Position.Sub(box.Position)
	velocity := circle.Velocity.Sub(box.Velocity)
	gap := normal.Dot(delta) - boxProjection(shape.HalfExtents, boxAxes(box.Angle), normal) - round.Radius
	omega := box.AngularVelocity
	speed := velocity.Dot(normal) + omega*normal.Cross(delta)
	rangeBound := delta.Length() + velocity.Length()*duration
	curvature := math.Abs(omega) * (math.Abs(omega)*rangeBound + 2*velocity.Length())
	linear, quadratic := speed*duration, (curvature*(duration*0.5))*duration
	if !finite(gap) || gap <= 0 || !finite(linear) || !finite(quadratic) || quadratic <= 0 {
		return 0, false
	}
	scale := math.Max(gap, math.Max(math.Abs(linear), quadratic))
	g, l, q := gap/scale, linear/scale, quadratic/scale
	if g == 0 || q == 0 {
		return 0, false
	}
	discriminant := math.Hypot(l, 2*math.Sqrt(g)*math.Sqrt(q))
	var fraction float64
	if l < 0 {
		fraction = 2 * g / (discriminant - l)
	} else {
		fraction = (l + discriminant) / (2 * q)
	}
	if math.IsNaN(fraction) || fraction < 0 {
		return 0, false
	}
	return fraction * duration, true
}

func toiPrepare(body *proxy) (*proxy, error) {
	if body == nil || !body.validState() {
		return nil, fmt.Errorf("physics: sweep requires valid finite bodies")
	}
	shape, valid := CloneShape(body.shape)
	if !valid {
		return nil, fmt.Errorf("physics: sweep requires a valid Circle or Box")
	}
	copy := *body
	copy.shape = shape
	if !finite(toiRadius(&copy)) {
		return nil, fmt.Errorf("physics: sweep radius overflow")
	}
	min, max := bodyAABB(&copy)
	if !validVec(min) || !validVec(max) {
		return nil, fmt.Errorf("physics: sweep bounds overflow")
	}
	return &copy, nil
}

// toiPose uses the same constant-velocity motion model as TimeOfImpact. The
// returned value is an independent geometry-and-motion copy.
func toiPose(body *proxy, time float64) (*proxy, error) {
	if body == nil || !finite(time) || time < 0 {
		return nil, fmt.Errorf("physics: invalid sweep pose")
	}
	pose := *body
	pose.Position = body.Position.Add(body.Velocity.Mul(time))
	pose.Angle += body.AngularVelocity * time
	min, max := bodyAABB(&pose)
	if !pose.validState() || !validVec(min) || !validVec(max) {
		return nil, fmt.Errorf("physics: sweep motion overflow")
	}
	return &pose, nil
}

func toiRadius(body *proxy) float64 {
	switch shape := body.shape.(type) {
	case Circle:
		return shape.Radius
	case Box:
		return shape.HalfExtents.Length()
	}
	return math.NaN()
}

func toiDistanceTolerance(a, b *proxy) float64 {
	scale := math.Min(toiFeature(a), toiFeature(b))
	if !finite(scale) || scale <= 0 {
		return toiTolerance
	}
	return math.Max(math.SmallestNonzeroFloat64, scale*toiTolerance)
}

func toiFeature(body *proxy) float64 {
	switch shape := body.shape.(type) {
	case Circle:
		return shape.Radius
	case Box:
		return math.Min(shape.HalfExtents.X, shape.HalfExtents.Y)
	}
	return math.NaN()
}

func toiAngularBound(body *proxy) float64 {
	if _, circle := body.shape.(Circle); circle {
		return 0
	}
	return math.Abs(body.AngularVelocity) * toiRadius(body)
}

func toiLinearFraction(a, b *proxy, dt float64) (float64, bool, error) {
	displacement := b.Velocity.Sub(a.Velocity).Mul(dt)
	if !validVec(displacement) {
		return 0, false, fmt.Errorf("physics: sweep relative displacement overflow")
	}
	switch sa := a.shape.(type) {
	case Circle:
		switch sb := b.shape.(type) {
		case Circle:
			return toiRayCircle(b.Position.Sub(a.Position), displacement, sa.Radius+sb.Radius)
		case Box:
			return toiCircleBoxFraction(a, sa, b, sb, dt)
		}
	case Box:
		switch sb := b.shape.(type) {
		case Circle:
			return toiCircleBoxFraction(b, sb, a, sa, dt)
		case Box:
			return toiBoxesFraction(a, sa, b, sb, displacement)
		}
	}
	return 0, false, fmt.Errorf("physics: unsupported sweep shapes")
}

// toiRayCircle intersects p+fraction*d with a circle at the origin. Projecting
// onto the ray avoids subtracting two nearly equal squared distances in the
// usual quadratic discriminant, important for small high-speed projectiles.
func toiRayCircle(p, d Vec2, radius float64) (float64, bool, error) {
	scale := math.Max(radius, math.Max(math.Max(math.Abs(p.X), math.Abs(p.Y)), math.Max(math.Abs(d.X), math.Abs(d.Y))))
	if !finite(scale) || scale <= 0 || !finite(radius) || radius <= 0 {
		return 0, false, fmt.Errorf("physics: sweep circle geometry overflow")
	}
	p, d, radius = (Vec2{X: p.X / scale, Y: p.Y / scale}), (Vec2{X: d.X / scale, Y: d.Y / scale}), radius/scale
	length := d.Length()
	if length == 0 {
		return 0, false, nil
	}
	if radius == 0 {
		return 0, false, fmt.Errorf("physics: sweep circle scale underflow")
	}
	direction := d.Normalized()
	perpendicular := math.Abs(p.Cross(direction))
	if perpendicular > radius {
		return 0, false, nil
	}
	approach := -p.Dot(direction)
	radial := math.Sqrt(math.Max(0, (radius-perpendicular)*(radius+perpendicular)))
	fraction := (approach - radial) / length
	if !finite(fraction) {
		return 0, false, fmt.Errorf("physics: sweep root overflow")
	}
	return fraction, fraction >= 0 && fraction <= 1, nil
}

func toiCircleBoxFraction(circle *proxy, sc Circle, box *proxy, sb Box, dt float64) (float64, bool, error) {
	p := circle.Position.Sub(box.Position).Rotate(-box.Angle)
	d := circle.Velocity.Sub(box.Velocity).Mul(dt).Rotate(-box.Angle)
	if !validVec(p) || !validVec(d) {
		return 0, false, fmt.Errorf("physics: sweep circle-box transform overflow")
	}
	// A rounded rectangle comprises four finite face strips and four circular
	// corner arcs. Testing those features avoids inflated-AABB corner false hits.
	best := math.Inf(1)
	consider := func(fraction float64, inside bool) {
		if inside && fraction >= 0 && fraction <= 1 && fraction < best {
			best = fraction
		}
	}
	half := sb.HalfExtents
	tolerance := toiDistanceTolerance(circle, box)
	if d.X != 0 {
		for _, sign := range []float64{-1, 1} {
			fraction := (sign*(half.X+sc.Radius) - p.X) / d.X
			y := p.Y + fraction*d.Y
			consider(fraction, finite(fraction) && math.Abs(y) <= half.Y+tolerance)
		}
	}
	if d.Y != 0 {
		for _, sign := range []float64{-1, 1} {
			fraction := (sign*(half.Y+sc.Radius) - p.Y) / d.Y
			x := p.X + fraction*d.X
			consider(fraction, finite(fraction) && math.Abs(x) <= half.X+tolerance)
		}
	}
	for _, sx := range []float64{-1, 1} {
		for _, sy := range []float64{-1, 1} {
			corner := Vec2{X: sx * half.X, Y: sy * half.Y}
			fraction, hit, err := toiRayCircle(p.Sub(corner), d, sc.Radius)
			if err != nil {
				return 0, false, err
			}
			point := p.Add(d.Mul(fraction))
			consider(fraction, hit && sx*point.X >= half.X-tolerance && sy*point.Y >= half.Y-tolerance)
		}
	}
	return best, finite(best), nil
}

func toiBoxesFraction(a *proxy, sa Box, b *proxy, sb Box, displacement Vec2) (float64, bool, error) {
	axesA, axesB := boxAxes(a.Angle), boxAxes(b.Angle)
	delta := b.Position.Sub(a.Position)
	enter, exit := 0.0, 1.0
	for _, axis := range [4]Vec2{axesA[0], axesA[1], axesB[0], axesB[1]} {
		projection, speed := delta.Dot(axis), displacement.Dot(axis)
		radius := boxProjection(sa.HalfExtents, axesA, axis) + boxProjection(sb.HalfExtents, axesB, axis)
		if !finite(projection) || !finite(speed) || !finite(radius) {
			return 0, false, fmt.Errorf("physics: swept SAT projection overflow")
		}
		if speed == 0 {
			if math.Abs(projection) > radius {
				return 0, false, nil
			}
			continue
		}
		scale := math.Max(radius, math.Max(math.Abs(projection), math.Abs(speed)))
		projection, speed, radius = projection/scale, speed/scale, radius/scale
		first, last := (-radius-projection)/speed, (radius-projection)/speed
		if first > last {
			first, last = last, first
		}
		enter, exit = math.Max(enter, first), math.Min(exit, last)
		if enter > exit {
			return 0, false, nil
		}
	}
	return enter, enter >= 0 && enter <= 1 && exit >= 0, nil
}

type toiWitness struct {
	distance float64
	normal   Vec2
	a, b     Vec2
}

func toiSeparation(a, b *proxy) (toiWitness, error) {
	if !collisionGeometryFinite(a, b) {
		return toiWitness{}, fmt.Errorf("physics: swept shape geometry overflow")
	}
	if contact, hit := collideProxies(a, b); hit {
		return toiWitness{-contact.Penetration, contact.Normal, contact.Points[0], contact.Points[0]}, nil
	}
	var witness toiWitness
	switch sa := a.shape.(type) {
	case Circle:
		switch sb := b.shape.(type) {
		case Circle:
			delta := b.Position.Sub(a.Position)
			normal := delta.Normalized()
			witness = toiWitness{delta.Length() - sa.Radius - sb.Radius, normal, a.Position.Add(normal.Mul(sa.Radius)), b.Position.Sub(normal.Mul(sb.Radius))}
		case Box:
			witness = toiCircleBoxWitness(a, sa, b, sb)
		}
	case Box:
		switch sb := b.shape.(type) {
		case Circle:
			witness = toiCircleBoxWitness(b, sb, a, sa)
			witness.normal, witness.a, witness.b = witness.normal.Mul(-1), witness.b, witness.a
		case Box:
			witness = toiBoxesWitness(a, sa, b, sb)
		}
	}
	if !finite(witness.distance) || !validVec(witness.normal) || !validVec(witness.a) || !validVec(witness.b) || witness.normal.LengthSquared() == 0 {
		return toiWitness{}, fmt.Errorf("physics: sweep distance overflow")
	}
	return witness, nil
}

func toiCircleBoxWitness(circle *proxy, sc Circle, box *proxy, sb Box) toiWitness {
	local := circle.Position.Sub(box.Position).Rotate(-box.Angle)
	closest := Vec2{X: clamp(local.X, -sb.HalfExtents.X, sb.HalfExtents.X), Y: clamp(local.Y, -sb.HalfExtents.Y, sb.HalfExtents.Y)}
	pointB := box.Position.Add(closest.Rotate(box.Angle))
	delta := pointB.Sub(circle.Position)
	normal := delta.Normalized()
	return toiWitness{delta.Length() - sc.Radius, normal, circle.Position.Add(normal.Mul(sc.Radius)), pointB}
}

func toiClosestBoxPoint(point Vec2, body *proxy, shape Box) Vec2 {
	local := point.Sub(body.Position).Rotate(-body.Angle)
	local = Vec2{X: clamp(local.X, -shape.HalfExtents.X, shape.HalfExtents.X), Y: clamp(local.Y, -shape.HalfExtents.Y, shape.HalfExtents.Y)}
	return body.Position.Add(local.Rotate(body.Angle))
}

func toiBoxesWitness(a *proxy, sa Box, b *proxy, sb Box) toiWitness {
	best := toiWitness{distance: math.Inf(1)}
	consider := func(pa, pb Vec2) {
		delta := pb.Sub(pa)
		distance := delta.Length()
		if distance < best.distance {
			best = toiWitness{distance, delta.Normalized(), pa, pb}
		}
	}
	for _, sx := range []float64{-1, 1} {
		for _, sy := range []float64{-1, 1} {
			pa := a.Position.Add((Vec2{X: sx * sa.HalfExtents.X, Y: sy * sa.HalfExtents.Y}).Rotate(a.Angle))
			consider(pa, toiClosestBoxPoint(pa, b, sb))
			pb := b.Position.Add((Vec2{X: sx * sb.HalfExtents.X, Y: sy * sb.HalfExtents.Y}).Rotate(b.Angle))
			consider(toiClosestBoxPoint(pb, a, sa), pb)
		}
	}
	return best
}

func toiContactAt(a, b *proxy, time float64) (Contact, error) {
	pa, err := toiPose(a, time)
	if err != nil {
		return Contact{}, err
	}
	pb, err := toiPose(b, time)
	if err != nil {
		return Contact{}, err
	}
	if contact, hit := collideProxies(pa, pb); hit {
		return contact, nil
	}
	witness, err := toiSeparation(pa, pb)
	if err != nil {
		return Contact{}, err
	}
	tolerance := toiDistanceTolerance(a, b)
	if witness.distance > tolerance*4 {
		return Contact{}, fmt.Errorf("physics: sweep contact is not representable at computed time")
	}
	// A tiny roundoff gap must not discard an analytically established impact.
	// Move a query-only copy into contact to recover a full face manifold. Real
	// poses remain untouched; remove the shift from midpoint contact locations.
	shift := witness.normal.Mul(math.Max(witness.distance, 0) + tolerance*0.25)
	pa.Position = pa.Position.Add(shift)
	if contact, hit := collideProxies(pa, pb); hit {
		contact.Penetration = 0
		for i := range contact.Points {
			contact.Points[i] = contact.Points[i].Sub(shift.Mul(0.5))
		}
		return contact, nil
	}
	contact := Contact{Normal: witness.normal, Points: []Vec2{witness.a.Mul(0.5).Add(witness.b.Mul(0.5))}}
	if !contactFinite(contact) {
		return Contact{}, fmt.Errorf("physics: sweep contact manifold overflow")
	}
	return contact, nil
}

package physics

import "math"

// Contact describes an overlap between two shapes. Normal is a unit vector from
// the first body toward the second body. Penetration is the minimum translation
// distance needed to separate them along Normal. Points are world-space contact
// locations, midway between the overlapping surfaces (one for circles, up to two
// for boxes). Shapes that just touch produce a zero-penetration contact.
type Contact struct {
	Normal      Vec2
	Penetration float64
	Points      []Vec2
}

// Collide detects contact between two bodies without changing them. Body types
// are ignored, so callers can also query contacts between static bodies. Nil
// bodies, invalid geometry, and unrepresentable collision geometry produce no
// contact. In particular, a successful result contains only finite values.
func Collide(a, b *Body) (Contact, bool) {
	if a == nil || b == nil ||
		!validVec(a.Position) || !validVec(b.Position) || !finite(a.Angle) || !finite(b.Angle) {
		return Contact{}, false
	}
	shapeA, validA := copyShape(a.shape)
	shapeB, validB := copyShape(b.shape)
	if !validA || !validB {
		return Contact{}, false
	}
	if !pairGeometryFinite(a, shapeA, b, shapeB) {
		return Contact{}, false
	}
	contact, ok := collideShapes(a, shapeA, b, shapeB)
	if !ok || !contactFinite(contact) {
		return Contact{}, false
	}
	return contact, true
}

// collisionGeometryFinite distinguishes an ordinary separated pair from a pair
// whose derived geometry cannot be represented. World checks this after broad
// phase; otherwise arithmetic overflow could silently look like no collision.
func collisionGeometryFinite(a, b *Body) bool {
	if a == nil || b == nil || !validVec(a.Position) || !validVec(b.Position) || !finite(a.Angle) || !finite(b.Angle) {
		return false
	}
	shapeA, validA := copyShape(a.shape)
	shapeB, validB := copyShape(b.shape)
	if !validA || !validB || !pairGeometryFinite(a, shapeA, b, shapeB) {
		return false
	}
	// Validate the manifold as well: the individual AABBs and center-relative
	// projections can be finite while an intermediate contact point overflows.
	contact, hit := collideShapes(a, shapeA, b, shapeB)
	return !hit || contactFinite(contact)
}

func contactFinite(contact Contact) bool {
	if !validVec(contact.Normal) || !finite(contact.Penetration) || contact.Penetration < 0 || len(contact.Points) == 0 {
		return false
	}
	for _, point := range contact.Points {
		if !validVec(point) {
			return false
		}
	}
	return true
}

func pairGeometryFinite(a *Body, shapeA Shape, b *Body, shapeB Shape) bool {
	delta := b.Position.Sub(a.Position)
	if !validVec(delta) {
		return false
	}
	switch sa := shapeA.(type) {
	case Circle:
		switch sb := shapeB.(type) {
		case Circle:
			return finite(sa.Radius+sb.Radius) && finite(delta.Length())
		case Box:
			return circleBoxGeometryFinite(a, sa, b, sb)
		}
	case Box:
		switch sb := shapeB.(type) {
		case Circle:
			return circleBoxGeometryFinite(b, sb, a, sa)
		case Box:
			axesA, axesB := boxAxes(a.Angle), boxAxes(b.Angle)
			for _, axis := range [4]Vec2{axesA[0], axesA[1], axesB[0], axesB[1]} {
				if !finite(delta.Dot(axis)) || !finite(boxProjection(sa.HalfExtents, axesA, axis)+boxProjection(sb.HalfExtents, axesB, axis)) {
					return false
				}
			}
			return true
		}
	}
	return false
}

func circleBoxGeometryFinite(circle *Body, sc Circle, box *Body, sb Box) bool {
	center := circle.Position.Sub(box.Position).Rotate(-box.Angle)
	if !validVec(center) || !finite(sc.Radius+sb.HalfExtents.X) || !finite(sc.Radius+sb.HalfExtents.Y) {
		return false
	}
	closest := Vec2{clamp(center.X, -sb.HalfExtents.X, sb.HalfExtents.X), clamp(center.Y, -sb.HalfExtents.Y, sb.HalfExtents.Y)}
	return finite(closest.Sub(center).Length())
}

func collideShapes(a *Body, shapeA Shape, b *Body, shapeB Shape) (Contact, bool) {
	switch sa := shapeA.(type) {
	case Circle:
		switch sb := shapeB.(type) {
		case Circle:
			return collideCircles(a, sa, b, sb)
		case Box:
			return collideCircleBox(a, sa, b, sb)
		}
	case Box:
		switch sb := shapeB.(type) {
		case Circle:
			contact, ok := collideCircleBox(b, sb, a, sa)
			contact.Normal = contact.Normal.Mul(-1)
			return contact, ok
		case Box:
			return collideBoxes(a, sa, b, sb)
		}
	}
	return Contact{}, false
}

func collideCircles(a *Body, sa Circle, b *Body, sb Circle) (Contact, bool) {
	delta := b.Position.Sub(a.Position)
	distance := delta.Length()
	radii := sa.Radius + sb.Radius
	if !finite(distance) || !finite(radii) || distance > radii {
		return Contact{}, false
	}
	normal := Vec2{X: 1}
	if distance > 0 {
		normal = delta.Normalized()
	}
	pointA := a.Position.Add(normal.Mul(sa.Radius))
	pointB := b.Position.Sub(normal.Mul(sb.Radius))
	return Contact{normal, radii - distance, []Vec2{pointA.Add(pointB.Sub(pointA).Mul(0.5))}}, true
}

func collideCircleBox(circle *Body, sc Circle, box *Body, sb Box) (Contact, bool) {
	center := circle.Position.Sub(box.Position).Rotate(-box.Angle)
	half := sb.HalfExtents
	closest := Vec2{clamp(center.X, -half.X, half.X), clamp(center.Y, -half.Y, half.Y)}
	delta := closest.Sub(center)
	distance := delta.Length()
	if !finite(distance) || distance > sc.Radius {
		return Contact{}, false
	}
	var normal Vec2
	penetration := sc.Radius - distance
	if distance > 0 {
		normal = delta.Normalized()
	} else {
		// The circle center is inside the box (or exactly on its boundary).
		// Normal points into the box so moving the circle opposite Normal takes
		// it out through its nearest face, including its full radius.
		dx, dy := half.X-math.Abs(center.X), half.Y-math.Abs(center.Y)
		if dx <= dy {
			sign := 1.0
			if center.X < 0 {
				sign = -1
			}
			normal = Vec2{X: -sign}
			closest.X = sign * half.X
			penetration = sc.Radius + dx
		} else {
			sign := 1.0
			if center.Y < 0 {
				sign = -1
			}
			normal = Vec2{Y: -sign}
			closest.Y = sign * half.Y
			penetration = sc.Radius + dy
		}
	}
	normal = normal.Rotate(box.Angle)
	pointBox := box.Position.Add(closest.Rotate(box.Angle))
	pointCircle := circle.Position.Add(normal.Mul(sc.Radius))
	point := pointCircle.Add(pointBox.Sub(pointCircle).Mul(0.5))
	return Contact{normal, penetration, []Vec2{point}}, true
}

func boxAxes(angle float64) [2]Vec2 {
	s, c := math.Sincos(angle)
	return [2]Vec2{{c, s}, {-s, c}}
}

func boxProjection(half Vec2, axes [2]Vec2, axis Vec2) float64 {
	return half.X*math.Abs(axes[0].Dot(axis)) + half.Y*math.Abs(axes[1].Dot(axis))
}

func collideBoxes(a *Body, sa Box, b *Body, sb Box) (Contact, bool) {
	axesA, axesB := boxAxes(a.Angle), boxAxes(b.Angle)
	axes := [4]Vec2{axesA[0], axesA[1], axesB[0], axesB[1]}
	delta := b.Position.Sub(a.Position)
	penetration, best := math.Inf(1), 0
	var normal Vec2
	for i, axis := range axes {
		projection := delta.Dot(axis)
		depth := boxProjection(sa.HalfExtents, axesA, axis) + boxProjection(sb.HalfExtents, axesB, axis) - math.Abs(projection)
		if !finite(depth) || depth < 0 {
			return Contact{}, false
		}
		// Prefer the first face on ties to avoid flipping the reference face
		// between almost-parallel boxes due to roundoff.
		if i == 0 || depth < penetration-1e-10*math.Max(math.Abs(depth), penetration) {
			penetration, best = depth, i
			normal = axis
			if projection < 0 {
				normal = normal.Mul(-1)
			}
		}
	}

	reference, incident := a, b
	refShape, incShape := sa, sb
	refAxes, incAxes := axesA, axesB
	refNormal := normal
	if best >= 2 {
		reference, incident = b, a
		refShape, incShape = sb, sa
		refAxes, incAxes = axesB, axesA
		refNormal = normal.Mul(-1)
	}
	refIndex := best % 2
	refExtent, tangentExtent := refShape.HalfExtents.X, refShape.HalfExtents.Y
	if refIndex == 1 {
		refExtent, tangentExtent = tangentExtent, refExtent
	}
	faceCenter := reference.Position.Add(refNormal.Mul(refExtent))
	tangent := refAxes[1-refIndex]

	// Pick the incident face whose outward normal most opposes the reference
	// normal, then clip that edge to the reference face's two side planes.
	incIndex := 0
	if math.Abs(incAxes[1].Dot(refNormal)) > math.Abs(incAxes[0].Dot(refNormal)) {
		incIndex = 1
	}
	incNormal := incAxes[incIndex]
	if incNormal.Dot(refNormal) > 0 {
		incNormal = incNormal.Mul(-1)
	}
	incExtent, edgeExtent := incShape.HalfExtents.X, incShape.HalfExtents.Y
	if incIndex == 1 {
		incExtent, edgeExtent = edgeExtent, incExtent
	}
	incCenter := incident.Position.Add(incNormal.Mul(incExtent))
	edge := incAxes[1-incIndex].Mul(edgeExtent)
	points := []Vec2{incCenter.Sub(edge), incCenter.Add(edge)}
	// Clip in coordinates relative to the reference center. This avoids large
	// absolute plane offsets when a scene has been translated far from zero.
	points = clipContactEdge(points, tangent, faceCenter, tangentExtent)
	points = clipContactEdge(points, tangent.Mul(-1), faceCenter, tangentExtent)
	contacts := make([]Vec2, 0, 2)
	tolerance := 1e-10 * math.Max(refExtent, tangentExtent)
	for _, point := range points {
		separation := point.Sub(faceCenter).Dot(refNormal)
		if separation <= tolerance {
			contacts = append(contacts, point.Sub(refNormal.Mul(separation*0.5)))
		}
	}
	if len(contacts) == 0 {
		// SAT still establishes overlap. At a nearly tangent corner, roundoff
		// in clipping can discard the sole endpoint; use the opposing support
		// points as a deterministic one-point manifold in that case.
		pa := boxSupport(a, sa, axesA, normal)
		pb := boxSupport(b, sb, axesB, normal.Mul(-1))
		contacts = append(contacts, pa.Add(pb.Sub(pa).Mul(0.5)))
	}
	return Contact{normal, penetration, contacts}, true
}

// clipContactEdge keeps the portion of an edge in the half-plane
// normal dot (point-origin) <= extent. A corner can reduce the edge to one point.
func clipContactEdge(points []Vec2, normal, origin Vec2, extent float64) []Vec2 {
	if len(points) == 0 {
		return nil
	}
	d0 := points[0].Sub(origin).Dot(normal) - extent
	if len(points) == 1 {
		if d0 <= 0 {
			return points
		}
		return nil
	}
	d1 := points[1].Sub(origin).Dot(normal) - extent
	result := make([]Vec2, 0, 2)
	if d0 <= 0 {
		result = append(result, points[0])
	}
	if d1 <= 0 {
		result = append(result, points[1])
	}
	if (d0 < 0 && d1 > 0) || (d0 > 0 && d1 < 0) {
		fraction := d0 / (d0 - d1)
		result = append(result, points[0].Add(points[1].Sub(points[0]).Mul(fraction)))
	}
	return result
}

func boxSupport(body *Body, shape Box, axes [2]Vec2, direction Vec2) Vec2 {
	x, y := shape.HalfExtents.X, shape.HalfExtents.Y
	if axes[0].Dot(direction) < 0 {
		x = -x
	}
	if axes[1].Dot(direction) < 0 {
		y = -y
	}
	return body.Position.Add(axes[0].Mul(x)).Add(axes[1].Mul(y))
}

func bodyAABB(body *Body) (min, max Vec2) {
	if body == nil {
		return Vec2{}, Vec2{}
	}
	var extent Vec2
	switch shape := body.shape.(type) {
	case Circle:
		extent = Vec2{shape.Radius, shape.Radius}
	case Box:
		s, c := math.Sincos(body.Angle)
		extent = Vec2{
			math.Abs(c)*shape.HalfExtents.X + math.Abs(s)*shape.HalfExtents.Y,
			math.Abs(s)*shape.HalfExtents.X + math.Abs(c)*shape.HalfExtents.Y,
		}
	}
	return body.Position.Sub(extent), body.Position.Add(extent)
}

func aabbOverlap(a, b *Body) bool {
	minA, maxA := bodyAABB(a)
	minB, maxB := bodyAABB(b)
	return minA.X <= maxB.X && minB.X <= maxA.X && minA.Y <= maxB.Y && minB.Y <= maxA.Y
}

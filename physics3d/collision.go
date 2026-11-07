package physics3d

import (
	"errors"
	"math"
)

// Contact describes touching or overlapping shapes. Normal points from the
// first body toward the second and Penetration is the minimum separation
// distance along that axis. Points contains world-space contact locations,
// midway between the surfaces: one for spheres or edge contacts, up to four
// for a box face manifold. Touching shapes have zero penetration.
type Contact struct {
	Normal      Vec3
	Penetration float64
	Points      []Vec3
}

// Collide queries contacts without changing bodies or considering their motion
// types. Nil bodies, invalid geometry, and unrepresentable intermediate geometry
// return no contact. Every successful contact contains finite values.
func Collide(a, b *Body) (Contact, bool) {
	contact, hit, err := collideChecked(a, b)
	return contact, hit && err == nil
}

// collideChecked distinguishes numeric failure from an ordinary separated
// pair, so World can roll back rather than silently miss a collision.
func collideChecked(a, b *Body) (Contact, bool, error) {
	if a == nil || b == nil || !validVec(a.Position) || !validVec(b.Position) || !collisionRotationValid(a.Rotation) || !collisionRotationValid(b.Rotation) {
		return Contact{}, false, errors.New("physics3d: invalid collision transform")
	}
	sa, validA := copyShape(a.shape)
	sb, validB := copyShape(b.shape)
	if !validA || !validB {
		return Contact{}, false, errors.New("physics3d: invalid collision shape")
	}
	for _, body := range []*Body{a, b} {
		min, max := bodyAABB(body)
		if !validVec(min) || !validVec(max) {
			return Contact{}, false, errors.New("physics3d: collision bounds overflow")
		}
	}
	if !validVec(b.Position.Sub(a.Position)) {
		return Contact{}, false, errors.New("physics3d: collision center difference overflow")
	}
	var contact Contact
	var hit bool
	var err error
	switch shapeA := sa.(type) {
	case Sphere:
		switch shapeB := sb.(type) {
		case Sphere:
			contact, hit, err = collideSpheres(a, shapeA, b, shapeB)
		case Box:
			contact, hit, err = collideSphereBox(a, shapeA, b, shapeB)
		}
	case Box:
		switch shapeB := sb.(type) {
		case Sphere:
			contact, hit, err = collideSphereBox(b, shapeB, a, shapeA)
			contact.Normal = contact.Normal.Mul(-1)
		case Box:
			contact, hit, err = collideBoxes(a, shapeA, b, shapeB)
		}
	}
	if err != nil {
		return Contact{}, false, err
	}
	if hit && !contactFinite(contact) {
		return Contact{}, false, errors.New("physics3d: collision manifold overflow")
	}
	return contact, hit, nil
}

func collisionRotationValid(q Quaternion) bool {
	return finite(q.X) && finite(q.Y) && finite(q.Z) && finite(q.W) &&
		(q.X != 0 || q.Y != 0 || q.Z != 0 || q.W != 0)
}

func contactFinite(c Contact) bool {
	if !validVec(c.Normal) || !finite(c.Penetration) || c.Penetration < 0 || len(c.Points) < 1 || len(c.Points) > 4 {
		return false
	}
	length := c.Normal.Length()
	if !finite(length) || math.Abs(length-1) > 1e-8 {
		return false
	}
	for _, point := range c.Points {
		if !validVec(point) {
			return false
		}
	}
	return true
}

func collideSpheres(a *Body, sa Sphere, b *Body, sb Sphere) (Contact, bool, error) {
	delta := b.Position.Sub(a.Position)
	distance, radii := delta.Length(), sa.Radius+sb.Radius
	if !finite(distance) || !finite(radii) {
		return Contact{}, false, errors.New("physics3d: sphere pair geometry overflow")
	}
	if distance > radii {
		return Contact{}, false, nil
	}
	normal := Vec3{X: 1}
	if distance > 0 {
		normal = delta.Normalized()
	}
	pa := a.Position.Add(normal.Mul(sa.Radius))
	pb := b.Position.Sub(normal.Mul(sb.Radius))
	return Contact{normal, radii - distance, []Vec3{collisionMidpoint(pa, pb)}}, true, nil
}

func collideSphereBox(sphere *Body, ss Sphere, box *Body, bs Box) (Contact, bool, error) {
	rotation := box.Rotation.Normalized()
	center := rotation.Conjugate().Rotate(sphere.Position.Sub(box.Position))
	half := bs.HalfExtents
	if !validVec(center) || !finite(ss.Radius+half.X) || !finite(ss.Radius+half.Y) || !finite(ss.Radius+half.Z) {
		return Contact{}, false, errors.New("physics3d: sphere-box geometry overflow")
	}
	closest := Vec3{clamp(center.X, -half.X, half.X), clamp(center.Y, -half.Y, half.Y), clamp(center.Z, -half.Z, half.Z)}
	delta := closest.Sub(center)
	distance := delta.Length()
	if !finite(distance) {
		return Contact{}, false, errors.New("physics3d: sphere-box distance overflow")
	}
	if distance > ss.Radius {
		return Contact{}, false, nil
	}
	normal := Vec3{}
	penetration := ss.Radius - distance
	if distance > 0 {
		normal = delta.Normalized()
	} else {
		// For an interior center, move the sphere opposite Normal through its
		// nearest face. The full radius is part of the exit distance.
		axis, faceDistance := 0, half.X-math.Abs(center.X)
		for i := 1; i < 3; i++ {
			if d := collisionComponent(half, i) - math.Abs(collisionComponent(center, i)); d < faceDistance {
				axis, faceDistance = i, d
			}
		}
		sign := 1.0
		if collisionComponent(center, axis) < 0 {
			sign = -1
		}
		normal = collisionBasis(axis).Mul(-sign)
		closest = collisionSetComponent(closest, axis, sign*collisionComponent(half, axis))
		penetration = ss.Radius + faceDistance
	}
	normal = rotation.Rotate(normal)
	pointBox := box.Position.Add(rotation.Rotate(closest))
	pointSphere := sphere.Position.Add(normal.Mul(ss.Radius))
	return Contact{normal, penetration, []Vec3{collisionMidpoint(pointSphere, pointBox)}}, true, nil
}

func boxAxes(rotation Quaternion) [3]Vec3 {
	q := rotation.Normalized()
	return [3]Vec3{q.Rotate(Vec3{X: 1}), q.Rotate(Vec3{Y: 1}), q.Rotate(Vec3{Z: 1})}
}

func boxProjection(half Vec3, axes [3]Vec3, axis Vec3) float64 {
	return half.X*math.Abs(axes[0].Dot(axis)) + half.Y*math.Abs(axes[1].Dot(axis)) + half.Z*math.Abs(axes[2].Dot(axis))
}

type boxAxis struct {
	normal Vec3
	depth  float64
	// face is 0..2 for A, 3..5 for B; edge-edge contacts use face=-1.
	face         int
	edgeA, edgeB int
}

func collideBoxes(a *Body, sa Box, b *Body, sb Box) (Contact, bool, error) {
	axesA, axesB := boxAxes(a.Rotation), boxAxes(b.Rotation)
	delta := b.Position.Sub(a.Position)
	best := boxAxis{depth: math.Inf(1)}
	testAxis := func(axis Vec3, face, edgeA, edgeB int) (bool, error) {
		projection := delta.Dot(axis)
		radii := boxProjection(sa.HalfExtents, axesA, axis) + boxProjection(sb.HalfExtents, axesB, axis)
		if !finite(projection) || !finite(radii) {
			return false, errors.New("physics3d: box projection overflow")
		}
		depth := radii - math.Abs(projection)
		if depth < 0 {
			return false, nil
		}
		// A relative tie bias keeps nearly coplanar faces coherent. All fifteen
		// axes still take part in separation, even if another axis wins the tie.
		tolerance := 1e-9 * math.Max(math.Abs(depth), math.Abs(best.depth))
		if math.IsInf(best.depth, 1) || depth < best.depth-tolerance {
			if projection < 0 {
				axis = axis.Mul(-1)
			}
			best = boxAxis{axis, depth, face, edgeA, edgeB}
		}
		return true, nil
	}
	for i := 0; i < 3; i++ {
		if overlap, err := testAxis(axesA[i], i, 0, 0); !overlap || err != nil {
			return Contact{}, false, err
		}
	}
	for i := 0; i < 3; i++ {
		if overlap, err := testAxis(axesB[i], i+3, 0, 0); !overlap || err != nil {
			return Contact{}, false, err
		}
	}
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			cross := axesA[i].Cross(axesB[j])
			if cross.LengthSquared() <= 1e-20 {
				continue
			}
			if overlap, err := testAxis(cross.Normalized(), -1, i, j); !overlap || err != nil {
				return Contact{}, false, err
			}
		}
	}
	if best.face == -1 {
		p0, p1 := supportingBoxEdge(a, sa, axesA, best.edgeA, best.normal)
		q0, q1 := supportingBoxEdge(b, sb, axesB, best.edgeB, best.normal.Mul(-1))
		pa, pb := closestEdgePoints(p0, p1, q0, q1)
		return Contact{best.normal, best.depth, []Vec3{collisionMidpoint(pa, pb)}}, true, nil
	}
	points := boxFaceManifold(a, sa, axesA, b, sb, axesB, best)
	if len(points) == 0 {
		// A tangent corner can disappear during clipping through roundoff. The
		// separating-axis result still establishes contact.
		pa := boxSupport(a, sa, axesA, best.normal)
		pb := boxSupport(b, sb, axesB, best.normal.Mul(-1))
		points = []Vec3{collisionMidpoint(pa, pb)}
	}
	return Contact{best.normal, best.depth, points}, true, nil
}

func boxFaceManifold(a *Body, sa Box, axesA [3]Vec3, b *Body, sb Box, axesB [3]Vec3, best boxAxis) []Vec3 {
	reference, incident := a, b
	refShape, incShape := sa, sb
	refAxes, incAxes := axesA, axesB
	refNormal, refIndex := best.normal, best.face
	if best.face >= 3 {
		reference, incident = b, a
		refShape, incShape = sb, sa
		refAxes, incAxes = axesB, axesA
		refNormal, refIndex = best.normal.Mul(-1), best.face-3
	}
	faceCenter := reference.Position.Add(refNormal.Mul(collisionComponent(refShape.HalfExtents, refIndex)))
	incidentIndex := 0
	for i := 1; i < 3; i++ {
		if math.Abs(incAxes[i].Dot(refNormal)) > math.Abs(incAxes[incidentIndex].Dot(refNormal)) {
			incidentIndex = i
		}
	}
	incidentNormal := incAxes[incidentIndex]
	if incidentNormal.Dot(refNormal) > 0 {
		incidentNormal = incidentNormal.Mul(-1)
	}
	incidentCenter := incident.Position.Add(incidentNormal.Mul(collisionComponent(incShape.HalfExtents, incidentIndex)))
	first, second := (incidentIndex+1)%3, (incidentIndex+2)%3
	u := incAxes[first].Mul(collisionComponent(incShape.HalfExtents, first))
	v := incAxes[second].Mul(collisionComponent(incShape.HalfExtents, second))
	polygon := []Vec3{
		incidentCenter.Add(u).Add(v), incidentCenter.Sub(u).Add(v),
		incidentCenter.Sub(u).Sub(v), incidentCenter.Add(u).Sub(v),
	}
	for i := 0; i < 3; i++ {
		if i == refIndex {
			continue
		}
		extent := collisionComponent(refShape.HalfExtents, i)
		polygon = clipContactPolygon(polygon, refAxes[i], faceCenter, extent)
		polygon = clipContactPolygon(polygon, refAxes[i].Mul(-1), faceCenter, extent)
	}
	refScale := math.Max(refShape.HalfExtents.X, math.Max(refShape.HalfExtents.Y, refShape.HalfExtents.Z))
	incScale := math.Max(incShape.HalfExtents.X, math.Max(incShape.HalfExtents.Y, incShape.HalfExtents.Z))
	scale := math.Min(refScale, incScale)
	tolerance := 1e-9 * scale
	points := make([]Vec3, 0, len(polygon))
	for _, point := range polygon {
		separation := point.Sub(faceCenter).Dot(refNormal)
		if separation > tolerance {
			continue
		}
		point = point.Sub(refNormal.Mul(separation * 0.5))
		duplicate := false
		for _, previous := range points {
			if point.Sub(previous).Length() <= 1e-10*scale {
				duplicate = true
				break
			}
		}
		if !duplicate {
			points = append(points, point)
		}
	}
	return reduceContactPolygon(points, refNormal)
}

// clipContactPolygon is Sutherland-Hodgman clipping in coordinates relative to
// origin, avoiding large absolute plane offsets when a scene is translated.
func clipContactPolygon(points []Vec3, normal, origin Vec3, extent float64) []Vec3 {
	if len(points) == 0 {
		return nil
	}
	result := make([]Vec3, 0, len(points)+1)
	previous := points[len(points)-1]
	dPrevious := previous.Sub(origin).Dot(normal) - extent
	for _, current := range points {
		dCurrent := current.Sub(origin).Dot(normal) - extent
		if (dCurrent < 0 && dPrevious > 0) || (dCurrent > 0 && dPrevious < 0) {
			fraction := dPrevious / (dPrevious - dCurrent)
			result = append(result, previous.Add(current.Sub(previous).Mul(fraction)))
		}
		if dCurrent <= 0 {
			result = append(result, current)
		}
		previous, dPrevious = current, dCurrent
	}
	return result
}

// Choose the maximum-area ordered quadrilateral if clipping produced more than
// four vertices. This retains a broad support polygon instead of clustering
// contacts at one corner, which would destabilize a resting box.
func reduceContactPolygon(points []Vec3, normal Vec3) []Vec3 {
	if len(points) <= 4 {
		return points
	}
	bestArea := -1.0
	var best [4]int
	for a := 0; a < len(points)-3; a++ {
		for b := a + 1; b < len(points)-2; b++ {
			for c := b + 1; c < len(points)-1; c++ {
				for d := c + 1; d < len(points); d++ {
					area := math.Abs(points[b].Sub(points[a]).Cross(points[c].Sub(points[a])).Add(points[c].Sub(points[a]).Cross(points[d].Sub(points[a]))).Dot(normal))
					if !finite(area) {
						return []Vec3{{X: math.NaN()}}
					}
					if area > bestArea {
						bestArea, best = area, [4]int{a, b, c, d}
					}
				}
			}
		}
	}
	return []Vec3{points[best[0]], points[best[1]], points[best[2]], points[best[3]]}
}

func supportingBoxEdge(body *Body, shape Box, axes [3]Vec3, edge int, direction Vec3) (Vec3, Vec3) {
	center := body.Position
	for i := 0; i < 3; i++ {
		if i == edge {
			continue
		}
		extent := collisionComponent(shape.HalfExtents, i)
		if axes[i].Dot(direction) < 0 {
			extent = -extent
		}
		center = center.Add(axes[i].Mul(extent))
	}
	delta := axes[edge].Mul(collisionComponent(shape.HalfExtents, edge))
	return center.Sub(delta), center.Add(delta)
}

func closestEdgePoints(p0, p1, q0, q1 Vec3) (Vec3, Vec3) {
	dp, dq := p1.Sub(p0), q1.Sub(q0)
	lp, lq := dp.Length(), dq.Length()
	if !finite(lp) || !finite(lq) {
		return Vec3{X: math.NaN()}, Vec3{X: math.NaN()}
	}
	if lp == 0 || lq == 0 {
		return p0, q0
	}
	u, v := dp.Normalized(), dq.Normalized()
	r := p0.Sub(q0)
	b, c, f := u.Dot(v), u.Dot(r), v.Dot(r)
	s := 0.0
	if denominator := 1 - b*b; denominator > 1e-12 {
		s = clamp((b*f-c)/denominator, 0, lp)
	}
	t := b*s + f
	if t < 0 {
		t, s = 0, clamp(-c, 0, lp)
	} else if t > lq {
		t, s = lq, clamp(b*lq-c, 0, lp)
	}
	return p0.Add(u.Mul(s)), q0.Add(v.Mul(t))
}

func boxSupport(body *Body, shape Box, axes [3]Vec3, direction Vec3) Vec3 {
	point := body.Position
	for i := 0; i < 3; i++ {
		extent := collisionComponent(shape.HalfExtents, i)
		if axes[i].Dot(direction) < 0 {
			extent = -extent
		}
		point = point.Add(axes[i].Mul(extent))
	}
	return point
}

func bodyAABB(body *Body) (min, max Vec3) {
	if body == nil {
		return Vec3{}, Vec3{}
	}
	var extent Vec3
	switch shape := body.shape.(type) {
	case Sphere:
		extent = Vec3{shape.Radius, shape.Radius, shape.Radius}
	case Box:
		axes := boxAxes(body.Rotation)
		for i := 0; i < 3; i++ {
			h := collisionComponent(shape.HalfExtents, i)
			extent = extent.Add(Vec3{math.Abs(axes[i].X) * h, math.Abs(axes[i].Y) * h, math.Abs(axes[i].Z) * h})
		}
	}
	return body.Position.Sub(extent), body.Position.Add(extent)
}

func aabbOverlap(a, b *Body) bool {
	minA, maxA := bodyAABB(a)
	minB, maxB := bodyAABB(b)
	return minA.X <= maxB.X && minB.X <= maxA.X && minA.Y <= maxB.Y && minB.Y <= maxA.Y && minA.Z <= maxB.Z && minB.Z <= maxA.Z
}

func collisionMidpoint(a, b Vec3) Vec3 { return a.Mul(0.5).Add(b.Mul(0.5)) }

func collisionComponent(v Vec3, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

func collisionSetComponent(v Vec3, axis int, value float64) Vec3 {
	switch axis {
	case 0:
		v.X = value
	case 1:
		v.Y = value
	default:
		v.Z = value
	}
	return v
}

func collisionBasis(axis int) Vec3 {
	return collisionSetComponent(Vec3{}, axis, 1)
}

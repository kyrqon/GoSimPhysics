package physics3d

import (
	"fmt"
	"math"
)

// RayHit describes the interval occupied by a solid along a point ray. Distances
// are measured in world units, regardless of the length of the input direction.
// ExitDistance is the complete solid exit, not an endpoint clipped to a segment.
type RayHit struct {
	EntryDistance, ExitDistance float64
	EntryPoint, ExitPoint       Vec3
	EntryNormal, ExitNormal     Vec3
	StartsInside                bool
	Tangent                     bool
}

// RayInterval intersects origin+t*normalized(direction), t >= 0, with a sphere
// or oriented box. StartsInside means strictly inside; a start on the boundary
// is not inside. Its entry normal is zero when the ray starts inside. Tangent
// intervals touch the surface without traversing any strict interior, including
// rays along box faces and edges. Touching an exit at t=0 is a zero interval.
// Invalid inputs and unrepresentable geometry are errors. Inputs are copied.
func RayInterval(c Collider, origin, direction Vec3) (RayHit, bool, error) {
	g, err := canonicalGeometry(c)
	if err != nil {
		return RayHit{}, false, err
	}
	if !validVec(origin) || !validVec(direction) || direction == (Vec3{}) {
		return RayHit{}, false, fmt.Errorf("physics3d: invalid ray")
	}
	if !geometryBoundsFinite(g) {
		return RayHit{}, false, fmt.Errorf("physics3d: collision bounds overflow")
	}
	direction = direction.Normalized()
	offset := origin.Sub(g.Position)
	if !validVec(offset) {
		return RayHit{}, false, fmt.Errorf("physics3d: ray offset overflow")
	}
	var result RayHit
	var hit bool
	switch s := g.shape.(type) {
	case Sphere:
		result, hit, err = raySphere(offset, direction, s.Radius)
	case Box:
		// Project onto world-space axes instead of rotating a huge offset
		// through quaternion intermediates that can overflow unnecessarily.
		axes := [3]Vec3{g.Rotation.Rotate(Vec3{X: 1}), g.Rotation.Rotate(Vec3{Y: 1}), g.Rotation.Rotate(Vec3{Z: 1})}
		localOrigin := Vec3{X: rayDot(offset, axes[0]), Y: rayDot(offset, axes[1]), Z: rayDot(offset, axes[2])}
		localDirection := Vec3{X: direction.Dot(axes[0]), Y: direction.Dot(axes[1]), Z: direction.Dot(axes[2])}
		if !validVec(localOrigin) || !validVec(localDirection) {
			return RayHit{}, false, fmt.Errorf("physics3d: ray transform overflow")
		}
		rotation := g.Rotation.Normalized()
		if rotation != Identity() && rotation != (Quaternion{W: -1}) {
			localOrigin.X = raySnapBoundary(localOrigin.X, s.HalfExtents.X, rayTransformError(offset, axes[0]))
			localOrigin.Y = raySnapBoundary(localOrigin.Y, s.HalfExtents.Y, rayTransformError(offset, axes[1]))
			localOrigin.Z = raySnapBoundary(localOrigin.Z, s.HalfExtents.Z, rayTransformError(offset, axes[2]))
			localDirection.X = raySnapParallel(localDirection.X, rayTransformError(direction, axes[0]))
			localDirection.Y = raySnapParallel(localDirection.Y, rayTransformError(direction, axes[1]))
			localDirection.Z = raySnapParallel(localDirection.Z, rayTransformError(direction, axes[2]))
		}
		result, hit, err = rayBox(localOrigin, localDirection, s.HalfExtents)
		result.EntryNormal = rayAxesVector(axes, result.EntryNormal)
		result.ExitNormal = rayAxesVector(axes, result.ExitNormal)
	}
	if err != nil || !hit {
		return RayHit{}, false, err
	}
	result.EntryPoint = rayPoint(origin, direction, result.EntryDistance)
	result.ExitPoint = rayPoint(origin, direction, result.ExitDistance)
	if !validVec(result.EntryPoint) || !validVec(result.ExitPoint) || !validVec(result.EntryNormal) || !validVec(result.ExitNormal) {
		return RayHit{}, false, fmt.Errorf("physics3d: ray intersection overflow")
	}
	return result, true, nil
}

func raySphere(offset, direction Vec3, radius float64) (RayHit, bool, error) {
	// Project a scaled offset and use the perpendicular cross product, avoiding
	// overflow and cancellation from subtracting nearly equal squared lengths.
	scale := math.Max(math.Abs(offset.X), math.Max(math.Abs(offset.Y), math.Abs(offset.Z)))
	projection, perpendicular := 0.0, 0.0
	var cross Vec3
	if scale != 0 {
		v := Vec3{X: offset.X / scale, Y: offset.Y / scale, Z: offset.Z / scale}
		projection = -v.Dot(direction) * scale
		cross = Vec3{X: rayDifference(v.Y, direction.Z, v.Z, direction.Y), Y: rayDifference(v.Z, direction.X, v.X, direction.Z), Z: rayDifference(v.X, direction.Y, v.Y, direction.X)}
		perpendicular = cross.Length() * scale
	}
	distance := offset.Length()
	if !finite(projection) || !finite(perpendicular) || !finite(distance) {
		return RayHit{}, false, fmt.Errorf("physics3d: ray projection overflow")
	}
	perpendicular = raySnapBoundary(perpendicular, radius, radius*(16*0x1p-52))
	distance = raySnapBoundary(distance, radius, radius*(16*0x1p-52))
	if perpendicular > radius {
		return RayHit{}, false, nil
	}
	halfChord := radius * (math.Sqrt((radius-perpendicular)/radius) * math.Sqrt(1+perpendicular/radius))
	near, far := projection-halfChord, projection+halfChord
	inside := distance < radius
	if distance == radius {
		if projection > 0 {
			near = 0
		} else {
			far = 0
		}
	}
	if !finite(near) || !finite(far) {
		return RayHit{}, false, fmt.Errorf("physics3d: ray distance overflow")
	}
	if halfChord > 0 && near == far {
		return RayHit{}, false, fmt.Errorf("physics3d: ray interval below distance precision")
	}
	if far < 0 {
		return RayHit{}, false, nil
	}
	entry := math.Max(near, 0)
	result := RayHit{EntryDistance: entry, ExitDistance: far, StartsInside: inside, Tangent: halfChord == 0 || entry == far}
	crossRatio := Vec3{X: (cross.X * scale) / radius, Y: (cross.Y * scale) / radius, Z: (cross.Z * scale) / radius}
	perpNormal, chordRatio := direction.Cross(crossRatio), halfChord/radius
	if !inside {
		result.EntryNormal = perpNormal.Sub(direction.Mul(chordRatio)).Normalized()
		if entry == 0 {
			result.EntryNormal = offset.Normalized()
		}
	}
	result.ExitNormal = perpNormal.Add(direction.Mul(chordRatio)).Normalized()
	if far == 0 {
		result.ExitNormal = offset.Normalized()
	}
	return result, true, nil
}

func rayBox(origin, direction, half Vec3) (RayHit, bool, error) {
	coordinates := [3]float64{origin.X, origin.Y, origin.Z}
	components := [3]float64{direction.X, direction.Y, direction.Z}
	extents := [3]float64{half.X, half.Y, half.Z}
	normals := [3]Vec3{{X: 1}, {Y: 1}, {Z: 1}}
	near, far := math.Inf(-1), math.Inf(1)
	entryNormal, exitNormal, boundaryNormal := Vec3{}, Vec3{}, Vec3{}
	inside, boundaryOnly := true, false
	for axis, coordinate := range coordinates {
		h, d := extents[axis], components[axis]
		inside = inside && math.Abs(coordinate) < h
		if d == 0 {
			if math.Abs(coordinate) > h {
				return RayHit{}, false, nil
			}
			if math.Abs(coordinate) == h {
				boundaryOnly = true
				boundaryNormal = normals[axis].Mul(math.Copysign(1, coordinate))
			}
			continue
		}
		a, b := (-h-coordinate)/d, (h-coordinate)/d
		an, bn := normals[axis].Mul(-1), normals[axis]
		if d < 0 {
			a, b, an, bn = b, a, bn, an
		}
		if a > near {
			near, entryNormal = a, an
		}
		if b < far {
			far, exitNormal = b, bn
		}
		if near > far {
			return RayHit{}, false, nil
		}
	}
	if !finite(near) || !finite(far) {
		return RayHit{}, false, fmt.Errorf("physics3d: ray distance overflow")
	}
	if near == far && entryNormal == exitNormal.Mul(-1) {
		return RayHit{}, false, fmt.Errorf("physics3d: ray interval below distance precision")
	}
	if far < 0 {
		return RayHit{}, false, nil
	}
	entry := math.Max(near, 0)
	if inside {
		entryNormal = Vec3{}
	} else if near < 0 {
		entryNormal = boundaryNormal
		if far == 0 {
			entryNormal = exitNormal
		}
	}
	return RayHit{EntryDistance: entry, ExitDistance: far, EntryNormal: entryNormal, ExitNormal: exitNormal, StartsInside: inside, Tangent: boundaryOnly || entry == far}, true, nil
}

func rayDot(v, axis Vec3) float64 {
	scale := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
	if scale == 0 {
		return 0
	}
	return math.FMA(v.X/scale, axis.X, math.FMA(v.Y/scale, axis.Y, (v.Z/scale)*axis.Z)) * scale
}

func rayAxesVector(axes [3]Vec3, v Vec3) Vec3 {
	return axes[0].Mul(v.X).Add(axes[1].Mul(v.Y)).Add(axes[2].Mul(v.Z))
}

func rayPoint(origin, direction Vec3, distance float64) Vec3 {
	return Vec3{X: math.FMA(direction.X, distance, origin.X), Y: math.FMA(direction.Y, distance, origin.Y), Z: math.FMA(direction.Z, distance, origin.Z)}
}

func rayDifference(a, b, c, d float64) float64 {
	product := c * d
	return math.FMA(a, b, -product) + math.FMA(-c, d, product)
}

func rayTransformError(v, axis Vec3) float64 {
	const roundoff = 16 * (0x1p-52)
	return math.Abs(v.X*axis.X)*roundoff + math.Abs(v.Y*axis.Y)*roundoff + math.Abs(v.Z*axis.Z)*roundoff
}

func raySnapBoundary(value, half, roundoff float64) float64 {
	if math.Abs(math.Abs(value)-half) <= roundoff {
		return math.Copysign(half, value)
	}
	return value
}

func raySnapParallel(value, roundoff float64) float64 {
	if math.Abs(value) <= roundoff {
		return 0
	}
	return value
}

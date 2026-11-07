package physics2d

import (
	"fmt"
	"math"
)

// RayHit describes the interval occupied by a solid along a point ray. Distances
// are measured in world units, regardless of the length of the input direction.
// ExitDistance is the complete solid exit, not an endpoint clipped to a segment.
type RayHit struct {
	EntryDistance, ExitDistance float64
	EntryPoint, ExitPoint       Vec2
	EntryNormal, ExitNormal     Vec2
	StartsInside                bool
	Tangent                     bool
}

// RayInterval intersects origin+t*normalized(direction), t >= 0, with a circle
// or oriented box. StartsInside means strictly inside; a start on the boundary
// is not inside. Its entry normal is zero when the ray starts inside. Tangent
// intervals touch the surface without traversing any strict interior, including
// rays along a box face. Touching an exit at t=0 is reported as a zero interval.
// Invalid inputs and unrepresentable geometry are errors. Inputs are copied.
func RayInterval(c Collider, origin, direction Vec2) (RayHit, bool, error) {
	shape, ok := CloneShape(c.Shape)
	if !ok || !validVec(origin) || !validVec(direction) || direction == (Vec2{}) {
		return RayHit{}, false, fmt.Errorf("physics2d: invalid ray or shape")
	}
	c.Shape = shape
	if _, _, err := Bounds(c); err != nil {
		return RayHit{}, false, err
	}
	direction = direction.Normalized()
	offset := origin.Sub(c.Position)
	if !validVec(offset) {
		return RayHit{}, false, fmt.Errorf("physics2d: ray offset overflow")
	}
	var result RayHit
	var hit bool
	var err error
	switch s := shape.(type) {
	case Circle:
		result, hit, err = rayCircle(offset, direction, s.Radius)
	case Box:
		localOrigin, localDirection := offset.Rotate(-c.Angle), direction.Rotate(-c.Angle)
		if !validVec(localOrigin) || !validVec(localDirection) {
			return RayHit{}, false, fmt.Errorf("physics2d: ray transform overflow")
		}
		if c.Angle != 0 {
			sine, cosine := math.Sincos(c.Angle)
			// An inverse transform can put an exactly parallel face ray a
			// few ulps inside or outside the face. Classify only within the
			// roundoff of the products that actually formed each coordinate.
			localOrigin.X = raySnapBoundary(localOrigin.X, s.HalfExtents.X, rayTransformError(cosine*offset.X, sine*offset.Y))
			localOrigin.Y = raySnapBoundary(localOrigin.Y, s.HalfExtents.Y, rayTransformError(sine*offset.X, cosine*offset.Y))
			localDirection.X = raySnapParallel(localDirection.X, rayTransformError(cosine*direction.X, sine*direction.Y))
			localDirection.Y = raySnapParallel(localDirection.Y, rayTransformError(sine*direction.X, cosine*direction.Y))
		}
		result, hit, err = rayBox(localOrigin, localDirection, s.HalfExtents)
		result.EntryNormal = result.EntryNormal.Rotate(c.Angle)
		result.ExitNormal = result.ExitNormal.Rotate(c.Angle)
	}
	if err != nil || !hit {
		return RayHit{}, false, err
	}
	result.EntryPoint = rayPoint(origin, direction, result.EntryDistance)
	result.ExitPoint = rayPoint(origin, direction, result.ExitDistance)
	if !validVec(result.EntryPoint) || !validVec(result.ExitPoint) || !validVec(result.EntryNormal) || !validVec(result.ExitNormal) {
		return RayHit{}, false, fmt.Errorf("physics2d: ray intersection overflow")
	}
	return result, true, nil
}

func rayCircle(offset, direction Vec2, radius float64) (RayHit, bool, error) {
	// Scale before projecting so neither squared lengths nor dot products
	// overflow. The perpendicular cross product avoids subtracting two nearly
	// equal squared lengths for a distant ray aimed close to the center.
	scale := math.Max(math.Abs(offset.X), math.Abs(offset.Y))
	projection, signedPerpendicular := 0.0, 0.0
	if scale != 0 {
		x, y := offset.X/scale, offset.Y/scale
		projection = -math.FMA(x, direction.X, y*direction.Y) * scale
		signedPerpendicular = rayDifference(x, direction.Y, y, direction.X) * scale
	}
	perpendicular := math.Abs(signedPerpendicular)
	distance := offset.Length()
	if !finite(projection) || !finite(perpendicular) || !finite(distance) {
		return RayHit{}, false, fmt.Errorf("physics2d: ray projection overflow")
	}
	// A tangent reconstructed after a rotation can differ from the radius by
	// a few ulps. Squaring that error into a chord would turn a surface touch
	// into a much larger, spurious interval.
	perpendicular = raySnapBoundary(perpendicular, radius, radius*(8*0x1p-52))
	distance = raySnapBoundary(distance, radius, radius*(8*0x1p-52))
	if perpendicular > radius {
		return RayHit{}, false, nil
	}
	halfChord := radius * (math.Sqrt((radius-perpendicular)/radius) * math.Sqrt(1+perpendicular/radius))
	near, far := projection-halfChord, projection+halfChord
	inside := distance < radius
	// Preserve exact boundary starts instead of letting subtraction round a
	// departing contact to a tiny negative distance and turn it into a miss.
	if distance == radius {
		if projection > 0 {
			near = 0
		} else {
			far = 0
		}
	}
	if !finite(near) || !finite(far) {
		return RayHit{}, false, fmt.Errorf("physics2d: ray distance overflow")
	}
	if halfChord > 0 && near == far {
		return RayHit{}, false, fmt.Errorf("physics2d: ray interval below distance precision")
	}
	if far < 0 {
		return RayHit{}, false, nil
	}
	entry := math.Max(near, 0)
	result := RayHit{EntryDistance: entry, ExitDistance: far, StartsInside: inside, Tangent: halfChord == 0 || entry == far}
	// Construct normals around closest approach, avoiding cancellation when a
	// small shape is far from the ray origin.
	perpRatio, chordRatio := signedPerpendicular/radius, halfChord/radius
	perpNormal := Vec2{X: direction.Y * perpRatio, Y: -direction.X * perpRatio}
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

func rayBox(origin, direction, half Vec2) (RayHit, bool, error) {
	coordinates := [2]float64{origin.X, origin.Y}
	components := [2]float64{direction.X, direction.Y}
	extents := [2]float64{half.X, half.Y}
	normals := [2]Vec2{{X: 1}, {Y: 1}}
	near, far := math.Inf(-1), math.Inf(1)
	entryNormal, exitNormal, boundaryNormal := Vec2{}, Vec2{}, Vec2{}
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
	// Infinite slab endpoints are useful for nearly parallel rays, but a
	// complete intersection must still have representable finite endpoints.
	if !finite(near) || !finite(far) {
		return RayHit{}, false, fmt.Errorf("physics2d: ray distance overflow")
	}
	if near == far && entryNormal == exitNormal.Mul(-1) {
		return RayHit{}, false, fmt.Errorf("physics2d: ray interval below distance precision")
	}
	if far < 0 {
		return RayHit{}, false, nil
	}
	entry := math.Max(near, 0)
	if inside {
		entryNormal = Vec2{}
	} else if near < 0 {
		entryNormal = boundaryNormal
		if far == 0 {
			entryNormal = exitNormal
		}
	}
	return RayHit{EntryDistance: entry, ExitDistance: far, EntryNormal: entryNormal, ExitNormal: exitNormal, StartsInside: inside, Tangent: boundaryOnly || entry == far}, true, nil
}

func rayPoint(origin, direction Vec2, distance float64) Vec2 {
	return Vec2{X: math.FMA(direction.X, distance, origin.X), Y: math.FMA(direction.Y, distance, origin.Y)}
}

// A compensated product difference retains accuracy for almost parallel rays.
func rayDifference(a, b, c, d float64) float64 {
	product := c * d
	return math.FMA(a, b, -product) + math.FMA(-c, d, product)
}

func rayTransformError(a, b float64) float64 {
	const roundoff = 8 * (0x1p-52)
	return math.Abs(a)*roundoff + math.Abs(b)*roundoff
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

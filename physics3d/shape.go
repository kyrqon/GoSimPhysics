package physics3d

// Shape is a centered collision shape. The built-in shapes are Sphere and Box.
// A body's shape is copied at creation and cannot be replaced afterward.
type Shape interface {
	valid() bool
	inertia(mass float64) Vec3
}

// Sphere is a solid sphere centered at a body's position.
type Sphere struct {
	Radius float64
}

func (s Sphere) valid() bool { return finite(s.Radius) && s.Radius > 0 }

func (s Sphere) inertia(mass float64) Vec3 {
	i := 0.4 * mass * s.Radius * s.Radius
	return Vec3{i, i, i}
}

// Box is a solid cuboid centered at a body's position and oriented by its
// rotation. HalfExtents contains half its width, height, and depth.
type Box struct {
	HalfExtents Vec3
}

func (b Box) valid() bool {
	return validVec(b.HalfExtents) && b.HalfExtents.X > 0 && b.HalfExtents.Y > 0 && b.HalfExtents.Z > 0
}

func (b Box) inertia(mass float64) Vec3 {
	h, factor := b.HalfExtents, mass/3
	return Vec3{
		factor*h.Y*h.Y + factor*h.Z*h.Z,
		factor*h.X*h.X + factor*h.Z*h.Z,
		factor*h.X*h.X + factor*h.Y*h.Y,
	}
}

func copyShape(shape Shape) (Shape, bool) {
	switch value := shape.(type) {
	case Sphere:
		return value, value.valid()
	case Box:
		return value, value.valid()
	case *Sphere:
		if value != nil {
			return *value, value.valid()
		}
	case *Box:
		if value != nil {
			return *value, value.valid()
		}
	}
	return nil, false
}

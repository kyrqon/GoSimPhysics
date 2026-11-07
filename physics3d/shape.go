package physics3d

// Shape is a centered collision shape. The built-in shapes are Sphere and Box.
// Pointer shapes are copied to validated values when passed into a query.
type Shape interface {
	valid() bool
	Inertia(mass float64) Vec3
}

// Sphere is a solid sphere centered at a collider's position.
type Sphere struct {
	Radius float64
}

func (s Sphere) valid() bool { return finite(s.Radius) && s.Radius > 0 }

// Inertia returns the principal moments of a solid sphere with the given mass.
func (s Sphere) Inertia(mass float64) Vec3 {
	i := 0.4 * mass * s.Radius * s.Radius
	return Vec3{X: i, Y: i, Z: i}
}

// Box is a solid cuboid centered at a collider's position and oriented by its
// rotation. HalfExtents contains half its width, height, and depth.
type Box struct {
	HalfExtents Vec3
}

func (b Box) valid() bool {
	return validVec(b.HalfExtents) && b.HalfExtents.X > 0 && b.HalfExtents.Y > 0 && b.HalfExtents.Z > 0
}

// Inertia returns the principal moments of a solid box with the given mass.
func (b Box) Inertia(mass float64) Vec3 {
	h, factor := b.HalfExtents, mass/3
	return Vec3{X: factor*h.Y*h.Y + factor*h.Z*h.Z, Y: factor*h.X*h.X + factor*h.Z*h.Z, Z: factor*h.X*h.X + factor*h.Y*h.Y}
}

// CloneShape validates a built-in shape and returns its canonical value copy.
// Nil pointers, nonpositive dimensions, and nonfinite dimensions are rejected.
func CloneShape(shape Shape) (Shape, bool) {
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

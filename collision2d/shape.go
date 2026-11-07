package collision2d

// Shape is a centered collision shape. The built-in shapes are Circle and Box.
// CloneShape validates and copies caller-owned shape pointers into values.
type Shape interface {
	valid() bool
	Inertia(mass float64) float64
}

// Circle is a solid disk centered at a collider's position. Radius must be positive.
type Circle struct {
	Radius float64
}

func (c Circle) valid() bool { return finite(c.Radius) && c.Radius > 0 }

// Inertia returns the solid disk moment of inertia about its center.
func (c Circle) Inertia(mass float64) float64 {
	return 0.5 * mass * c.Radius * c.Radius
}

// Box is a solid rectangle centered at a collider's position and rotated by its angle.
// HalfExtents contains half the width and half the height; both must be positive.
type Box struct {
	HalfExtents Vec2
}

func (b Box) valid() bool {
	return validVec(b.HalfExtents) && b.HalfExtents.X > 0 && b.HalfExtents.Y > 0
}

// Inertia returns the solid rectangle moment of inertia about its center.
func (b Box) Inertia(mass float64) float64 {
	return mass * (b.HalfExtents.X*b.HalfExtents.X + b.HalfExtents.Y*b.HalfExtents.Y) / 3
}

// CloneShape validates and copies a built-in shape into its immutable value form.
// Nil pointers and invalid or unsupported shapes return false.
func CloneShape(shape Shape) (Shape, bool) {
	switch value := shape.(type) {
	case Circle:
		return value, value.valid()
	case Box:
		return value, value.valid()
	case *Circle:
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

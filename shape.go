package physics

// Shape is a centered collision shape. The built-in shapes are Circle and Box.
// A body's shape is copied at creation and cannot be replaced afterward.
type Shape interface {
	valid() bool
	inertia(mass float64) float64
}

// Circle is a solid disk centered at a body's position. Radius must be positive.
type Circle struct {
	Radius float64
}

func (c Circle) valid() bool { return finite(c.Radius) && c.Radius > 0 }

func (c Circle) inertia(mass float64) float64 {
	return 0.5 * mass * c.Radius * c.Radius
}

// Box is a solid rectangle centered at a body's position and rotated by its angle.
// HalfExtents contains half the width and half the height; both must be positive.
type Box struct {
	HalfExtents Vec2
}

func (b Box) valid() bool {
	return validVec(b.HalfExtents) && b.HalfExtents.X > 0 && b.HalfExtents.Y > 0
}

func (b Box) inertia(mass float64) float64 {
	return mass * (b.HalfExtents.X*b.HalfExtents.X + b.HalfExtents.Y*b.HalfExtents.Y) / 3
}

func copyShape(shape Shape) (Shape, bool) {
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

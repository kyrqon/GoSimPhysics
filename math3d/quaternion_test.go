package math3d

import (
	"math"
	"math/rand"
	"testing"
)

func TestQuaternionAxisAngleAndComposition(t *testing.T) {
	x := QuaternionFromAxisAngle(Vec3{X: 3}, math.Pi/2)
	y := QuaternionFromAxisAngle(Vec3{Y: 4}, math.Pi/2)
	z := QuaternionFromAxisAngle(Vec3{Z: 5}, math.Pi/2)
	math3VecNear(t, "right-hand x rotation", x.Rotate(Vec3{Y: 1}), Vec3{Z: 1}, 1e-15)
	math3VecNear(t, "right-hand y rotation", y.Rotate(Vec3{Z: 1}), Vec3{X: 1}, 1e-15)
	math3VecNear(t, "right-hand z rotation", z.Rotate(Vec3{X: 1}), Vec3{Y: 1}, 1e-15)
	math3VecNear(t, "x*y applies y first", x.Mul(y).Rotate(Vec3{Z: 1}), Vec3{X: 1}, 1e-15)
	math3VecNear(t, "y*x applies x first", y.Mul(x).Rotate(Vec3{Z: 1}), Vec3{Y: -1}, 1e-15)
	if QuaternionFromAxisAngle(Vec3{}, 7) != Identity() {
		t.Fatal("zero axis did not produce identity")
	}
}

func TestQuaternionNormalizationAndInverse(t *testing.T) {
	if (Quaternion{}).Normalized() != Identity() || validQuat(Quaternion{}) {
		t.Fatal("zero quaternion must normalize to identity but is not a valid stored rotation")
	}
	for _, scale := range []float64{1, 1e300, 1e-300, math.SmallestNonzeroFloat64} {
		q := (Quaternion{scale, -scale, scale, -scale}).Normalized()
		math3Near(t, "normalized X", q.X, 0.5, 0)
		math3Near(t, "normalized Y", q.Y, -0.5, 0)
		math3Near(t, "normalized Z", q.Z, 0.5, 0)
		math3Near(t, "normalized W", q.W, -0.5, 0)
	}
	if validQuat(Quaternion{W: math.NaN()}) || validQuat(Quaternion{X: math.Inf(1), W: 1}) {
		t.Fatal("non-finite rotation accepted")
	}
	rng := rand.New(rand.NewSource(142))
	for i := 0; i < 200; i++ {
		q := QuaternionFromAxisAngle(Vec3{rng.Float64(), rng.Float64(), rng.Float64()}, rng.Float64()*10-5)
		v := Vec3{rng.Float64()*10 - 5, rng.Float64()*10 - 5, rng.Float64()*10 - 5}
		rotated := q.Rotate(v)
		math3Near(t, "rotation preserves norm", rotated.Length(), v.Length(), 8e-15)
		math3VecNear(t, "inverse rotation", q.Conjugate().Rotate(rotated), v, 8e-15)
	}
}

func TestQuaternionWorldSpaceIntegrationAndLongRunNormalization(t *testing.T) {
	q := QuaternionFromAxisAngle(Vec3{X: 1}, math.Pi/2)
	q = IntegrateRotation(q, Vec3{Y: math.Pi / 2})
	// A world-Y rotation is applied after the initial body-X rotation.
	math3VecNear(t, "world integration", q.Rotate(Vec3{Z: 1}), Vec3{Y: -1}, 1e-15)
	for i := 0; i < 10000; i++ {
		q = IntegrateRotation(q, Vec3{0.0001, -0.0002, 0.0003})
	}
	math3Near(t, "unit quaternion", q.X*q.X+q.Y*q.Y+q.Z*q.Z+q.W*q.W, 1, 8e-16)
	math3VecNear(t, "zero increment", IntegrateRotation(q, Vec3{}).Rotate(Vec3{1, 2, 3}), q.Rotate(Vec3{1, 2, 3}), 1e-14)
}

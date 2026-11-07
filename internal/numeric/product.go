package numeric

import "math"

// ProductRatio evaluates products/quotients without avoidable intermediate
// overflow or underflow. Callers must supply finite nonnegative numerators and
// finite positive denominators.
func ProductRatio(numerators, denominators []float64) float64 {
	fraction, exponent := 1.0, 0
	for _, value := range numerators {
		if value == 0 {
			return 0
		}
		part, power := math.Frexp(value)
		fraction *= part
		exponent += power
	}
	for _, value := range denominators {
		part, power := math.Frexp(value)
		fraction /= part
		exponent -= power
	}
	return math.Ldexp(fraction, exponent)
}

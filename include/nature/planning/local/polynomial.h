/**
 * \class Polynomial
 *
 * Polynomial class for defining polynomial with coefficients.
 *
 * \author Chris Goodin
 *
 * \date 8/31/2020
 */
#ifndef SPLINE_POLYNOMIAL_H
#define SPLINE_POLYNOMIAL_H
#include <vector>
#include <algorithm>

namespace nature {
namespace planning {
class Polynomial {
public:

	/**
	 * Create an uninitialized polynomial.
	 */ 
	Polynomial() {}

	/**
	 * Create an initialized polynomial.
	 * The rank will be the size of the coefficient list less one.
	 * p(x) = c[0]x^n + c[1]x^n-1 + ... c[n-2]x + c[n-1]
	 * \param coeffs The coefficients of the polynomial.
	 */
	Polynomial(std::vector<float> coeffs) {
		coeffs_ = coeffs;
		std::reverse(coeffs_.begin(), coeffs_.end());
	}

	/**
	 * Get a polynomial representing the derivative of the current polynomial. 
	 */
	Polynomial Derivative() {
		std::vector<float> coeffs;
		for (int i = 0; i < coeffs_.size(); i++) {
			float c = i * coeffs_[i];
			coeffs.push_back(c);
		}
		Polynomial poly(coeffs);
		return poly;
	}

	/**
	 * Get the value of the polynomial at x
	 * \param x Evaluate the polynomial at p(x)
	 */ 
	float At(float x) {
		float y = 0.0f;
		for (int i = 0; i < coeffs_.size(); i++) {

			y += coeffs_[i] * (float)pow(x, i);
		}
		return y;
	}

private:
	std::vector<float> coeffs_;

};

} // namespace planning
} // namespace nature


#endif
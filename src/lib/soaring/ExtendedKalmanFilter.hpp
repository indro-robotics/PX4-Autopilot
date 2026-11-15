/*
Extended Kalman Filter class by Sam Tabor, 2013.
* http://diydrones.com/forum/topics/autonomous-soaring
* Set up for identifying thermals of Gaussian form, but could be adapted to other
* purposes by adapting the equations for the jacobians.
*/

// Source: ArduPilot libraries/AP_Soaring/ExtendedKalmanFilter.h, commit 74f0f83. Licence: GPLv3 or later, see COPYING.

#pragma once

#include <matrix/math.hpp>
#include <stdint.h>

namespace soaring
{

class ExtendedKalmanFilter
{
public:
	ExtendedKalmanFilter(void) {}

	static constexpr const uint8_t N = 4;

	matrix::Vector<float, N> X;
	matrix::SquareMatrix<float, N> P;
	matrix::SquareMatrix<float, N> Q;
	float R;
	void reset(const matrix::Vector<float, N> &x, const matrix::SquareMatrix<float, N> &p, const matrix::SquareMatrix<float, N> q, float r);
	void update(float z, float Px, float Py, float driftX, float driftY);

	float measurementpredandjacobian(matrix::Vector<float, N> &A, float Px, float Py);
};

} // namespace soaring

// Source: ArduPilot libraries/AP_Soaring/ExtendedKalmanFilter.cpp, commit 74f0f83. Licence: GPLv3 or later, see COPYING.

#include "ExtendedKalmanFilter.hpp"

#include <mathlib/math/Limits.hpp>

namespace soaring
{

float ExtendedKalmanFilter::measurementpredandjacobian(matrix::Vector<float, N> &A, float Px, float Py)
{
	// This function computes the Jacobian using equations from
	// analytical derivation of Gaussian updraft distribution
	// This expression gets used lots
	float expon = expf(- (powf(X(2) - Px, 2) + powf(X(3) - Py, 2)) / powf(X(1), 2));
	// Expected measurement
	float w = X(0) * expon;

	// Elements of the Jacobian
	A(0) = expon;
	A(1) = 2 * X(0) * ((powf(X(2) - Px, 2) + powf(X(3) - Py, 2)) / powf(X(1), 3)) * expon;
	A(2) = -2 * (X(0) * (X(2) - Px) / powf(X(1), 2)) * expon;
	A(3) = -2 * (X(0) * (X(3) - Py) / powf(X(1), 2)) * expon;
	return w;
}


void ExtendedKalmanFilter::reset(const matrix::Vector<float, N> &x, const matrix::SquareMatrix<float, N> &p,
				 const matrix::SquareMatrix<float, N> q, float r)
{
	P = p;
	X = x;
	Q = q;
	R = r;
}


void ExtendedKalmanFilter::update(float z, float Px, float Py, float driftX, float driftY)
{
	matrix::SquareMatrix<float, N> tempM;
	matrix::Vector<float, N> H;
	matrix::Vector<float, N> P12;
	matrix::Vector<float, N> K;

	// LINE 28
	// Estimate new state from old.
	X(2) += driftX;
	X(3) += driftY;

	// LINE 33
	// Update the covariance matrix
	// P = A*ekf.P*A'+ekf.Q;
	// We know A is identity so
	// P = ekf.P+ekf.Q;
	P += Q;

	// What measurement do we expect to receive in the estimated
	// state
	// LINE 37
	// [z1,H] = ekf.jacobian_h(x1);
	float z1 = measurementpredandjacobian(H, Px, Py);

	// LINE 40
	// P12 = P * H';
	P12 = P * H; //cross covariance

	// LINE 41
	// Calculate the KALMAN GAIN
	// K = P12 * inv(H*P12 + ekf.R);                     %Kalman filter gain
	K = P12 * 1.0f / (H * P12 + R);

	// Correct the state estimate using the measurement residual.
	// LINE 44
	// X = x1 + K * (z - z1);
	X += K * (z - z1);

	// Make sure X[1] stays positive.
	X(1) = X(1) > 40.0f ? X(1) : 40.0f;

	// strength to 10 m/s, radius to 200 m, centre within 300 m of the aircraft on each axis: the drift step divides by the strength
	X(0) = math::constrain(X(0), 0.f, 10.f);
	X(1) = math::min(X(1), 200.f);
	X(2) = math::constrain(X(2), Px - 300.f, Px + 300.f);
	X(3) = math::constrain(X(3), Py - 300.f, Py + 300.f);

	// Correct the covariance too.
	// LINE 46
	// NB should be altered to reflect Stengel
	// P = P_predict - K * P12';
	tempM = matrix::Matrix<float, N, 1>(K) * P12.transpose();
	P -= tempM;

	P.makeBlockSymmetric<N>(0);
}

} // namespace soaring

/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file zero_order_thrust_ekf.cpp
 *
 * @author Mathieu Bresciani 	<brescianimathieu@gmail.com>
 */

#include <mathlib/mathlib.h>

#include "zero_order_hover_thrust_ekf.hpp"

void ZeroOrderHoverThrustEkf::predict(const float dt)
{
	// State is constant
	// Predict state covariance only
	_P += _Q * dt;
	_dt = dt;
}

void ZeroOrderHoverThrustEkf::fuseAccZ(const float acc_z, const float thrust)
{
	const float H = computeH(thrust);
	const float innov_var = computeInnovVar(H);
	const float innov = computeInnov(acc_z, thrust);
	const float K = computeKalmanGain(H, innov_var);
	const float innov_test_ratio = computeInnovTestRatio(innov, innov_var);

	printf("AZ = %.3f\tthr = %.3f\tH = %.3f\tinnov_var = %.3f\tinnov = %.3f\tK = %.3f\ttest_ratio = %.3f\n",
	       (double)acc_z,
	       (double)thrust,
	       (double)H,
	       (double)innov_var,
	       (double)innov,
	       (double)K,
	       (double)innov_test_ratio);

	float residual = innov;

	if (isTestRatioPassing(innov_test_ratio)) {
		updateState(K, innov);
		updateStateCovariance(K, H);
		residual = computeInnov(acc_z, thrust); // residual != innovation since the hover thrust changed
		printf("estimated hover thrust: %.3f\t std_dev = %.3fres = %.3f\n",
		       (double)_hover_thr,
		       (double)sqrtf(_P),
		       (double)residual);
	}

	updateMeasurementNoise(residual, H);
}

inline float ZeroOrderHoverThrustEkf::computeH(const float thrust)
{
	return -CONSTANTS_ONE_G * thrust / (_hover_thr * _hover_thr);
}

inline float ZeroOrderHoverThrustEkf::computeInnovVar(const float H)
{
	return math::max(H * _P * H + _R, _R);
}

float ZeroOrderHoverThrustEkf::computePredictedAccZ(const float thrust)
{
	return CONSTANTS_ONE_G * thrust / _hover_thr - CONSTANTS_ONE_G;
}

float ZeroOrderHoverThrustEkf::computeInnov(const float acc_z, const float thrust)
{
	const float predicted_acc_z = computePredictedAccZ(thrust);
	return acc_z - predicted_acc_z;
}

inline float ZeroOrderHoverThrustEkf::computeKalmanGain(const float H, const float innov_var)
{
	return _P * H / innov_var;
}

inline float ZeroOrderHoverThrustEkf::computeInnovTestRatio(const float innov, const float innov_var)
{
	return innov * innov / (_gate_size * _gate_size * innov_var);
}

inline bool ZeroOrderHoverThrustEkf::isTestRatioPassing(const float innov_test_ratio)
{
	return innov_test_ratio < 1.f;
}

inline void ZeroOrderHoverThrustEkf::updateState(const float K, const float innov)
{
	_hover_thr = math::constrain(_hover_thr + K * innov, 0.1f, 0.8f);
}

inline void ZeroOrderHoverThrustEkf::updateStateCovariance(const float K, const float H)
{
	_P = math::constrain((1.f - K * H) * _P, 1e-10f, 1.f);
}

inline void ZeroOrderHoverThrustEkf::updateMeasurementNoise(const float residual, const float H)
{
	const float alpha = _dt / (noise_learning_time_constant + _dt);
	_R = math::constrain((1.f - alpha) * _R  + alpha * (residual * residual + H * _P * H), 1e-4f, 10.f);
}
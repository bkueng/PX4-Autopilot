/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file controllib_test_main.cpp
 * Unit testing for controllib.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#include <math.h>
#include <stdio.h>
#include <float.h>

#include <controllib/blocks.hpp>

using namespace control;

#define ASSERT_CL(T) if (!(T)) { printf("FAIL\n"); return -1; }

int basic_blocks_test();
int block_limit_test();
int block_limit_sym_test();
int block_low_pass_test();
int block_high_pass_test();
int block_low_pass2_test();
int block_integral_test();
int block_integral_trap_test();
int block_derivative_test();
int block_p_test();
int block_pi_test();
int block_pd_test();
int block_pid_test();
int block_output_test();
int block_rand_uniform_test();
int block_rand_gauss_test();
int block_stats_test();
int block_delay_test();

int basic_blocks_test()
{
	bool failed = false;
	failed = failed || block_limit_test() < 0;
	failed = failed || block_limit_sym_test() < 0;
	failed = failed || block_low_pass_test() < 0;
	failed = failed || block_high_pass_test() < 0;
	failed = failed || block_low_pass2_test() < 0;
	failed = failed || block_integral_test() < 0;
	failed = failed || block_integral_trap_test() < 0;
	failed = failed || block_derivative_test() < 0;
	failed = failed || block_p_test() < 0;
	failed = failed || block_pi_test() < 0;
	failed = failed || block_pd_test() < 0;
	failed = failed || block_pid_test() < 0;
	failed = failed || block_output_test() < 0;
	failed = failed || block_rand_uniform_test() < 0;
	failed = failed || block_rand_gauss_test() < 0;
	failed = failed || block_stats_test() < 0;
	failed = failed || block_delay_test() < 0;
	return failed ? -1 : 0;
}

int block_limit_test()
{
	printf("Test BlockLimit\t\t\t: ");
	BlockLimit limit(nullptr, "TEST");
	// initial state
	ASSERT_CL(equal(1.0f, limit.getMax()));
	ASSERT_CL(equal(-1.0f, limit.getMin()));
	ASSERT_CL(equal(0.0f, limit.getDt()));
	// update
	ASSERT_CL(equal(-1.0f, limit.update(-2.0f)));
	ASSERT_CL(equal(1.0f, limit.update(2.0f)));
	ASSERT_CL(equal(0.0f, limit.update(0.0f)));
	printf("PASS\n");
	return 0;
}

int block_limit_sym_test()
{
	printf("Test BlockLimitSym\t\t: ");
	BlockLimitSym limit(nullptr, "TEST");
	// initial state
	ASSERT_CL(equal(1.0f, limit.getMax()));
	ASSERT_CL(equal(0.0f, limit.getDt()));
	// update
	ASSERT_CL(equal(-1.0f, limit.update(-2.0f)));
	ASSERT_CL(equal(1.0f, limit.update(2.0f)));
	ASSERT_CL(equal(0.0f, limit.update(0.0f)));
	printf("PASS\n");
	return 0;
}

int block_low_pass_test()
{
	printf("Test BlockLowPass\t\t: ");
	BlockLowPass low_pass(nullptr, "TEST_LP");
	// test initial state
	ASSERT_CL(equal(10.0f, low_pass.getFCut()));
	ASSERT_CL(equal(0.0f, low_pass.getState()));
	ASSERT_CL(equal(0.0f, low_pass.getDt()));
	// set dt
	low_pass.setDt(0.1f);
	ASSERT_CL(equal(0.1f, low_pass.getDt()));
	// set state
	low_pass.setState(1.0f);
	ASSERT_CL(equal(1.0f, low_pass.getState()));
	// test update
	ASSERT_CL(equal(1.8626974f, low_pass.update(2.0f)));

	// test end condition
	for (int i = 0; i < 100; i++) {
		low_pass.update(2.0f);
	}

	ASSERT_CL(equal(2.0f, low_pass.getState()));
	ASSERT_CL(equal(2.0f, low_pass.update(2.0f)));
	printf("PASS\n");
	return 0;
};

int block_high_pass_test()
{
	printf("Test BlockHighPass\t\t: ");
	BlockHighPass high_pass(nullptr, "TEST_HP");
	// test initial state
	ASSERT_CL(equal(10.0f, high_pass.getFCut()));
	ASSERT_CL(equal(0.0f, high_pass.getU()));
	ASSERT_CL(equal(0.0f, high_pass.getY()));
	ASSERT_CL(equal(0.0f, high_pass.getDt()));
	// set dt
	high_pass.setDt(0.1f);
	ASSERT_CL(equal(0.1f, high_pass.getDt()));
	// set state
	high_pass.setU(1.0f);
	ASSERT_CL(equal(1.0f, high_pass.getU()));
	high_pass.setY(1.0f);
	ASSERT_CL(equal(1.0f, high_pass.getY()));
	// test update
	ASSERT_CL(equal(0.2746051f, high_pass.update(2.0f)));

	// test end condition
	for (int i = 0; i < 100; i++) {
		high_pass.update(2.0f);
	}

	ASSERT_CL(equal(0.0f, high_pass.getY()));
	ASSERT_CL(equal(0.0f, high_pass.update(2.0f)));
	printf("PASS\n");
	return 0;
}

int block_low_pass2_test()
{
	printf("Test BlockLowPass2\t\t: ");
	BlockLowPass2 low_pass(nullptr, "TEST_LP", 100);
	// test initial state
	ASSERT_CL(equal(10.0f, low_pass.getFCutParam()));
	ASSERT_CL(equal(0.0f, low_pass.getState()));
	ASSERT_CL(equal(0.0f, low_pass.getDt()));
	// set dt
	low_pass.setDt(0.1f);
	ASSERT_CL(equal(0.1f, low_pass.getDt()));
	// set state
	low_pass.setState(1.0f);
	ASSERT_CL(equal(1.0f, low_pass.getState()));
	// test update
	ASSERT_CL(equal(1.06745527f, low_pass.update(2.0f)));

	// test end condition
	for (int i = 0; i < 100; i++) {
		low_pass.update(2.0f);
	}

	ASSERT_CL(equal(2.0f, low_pass.getState()));
	ASSERT_CL(equal(2.0f, low_pass.update(2.0f)));
	printf("PASS\n");
	return 0;
};

int block_integral_test()
{
	printf("Test BlockIntegral\t\t: ");
	BlockIntegral integral(nullptr, "TEST_I");
	// test initial state
	ASSERT_CL(equal(1.0f, integral.getMax()));
	ASSERT_CL(equal(0.0f, integral.getDt()));
	// set dt
	integral.setDt(0.1f);
	ASSERT_CL(equal(0.1f, integral.getDt()));
	// set Y
	integral.setY(0.9f);
	ASSERT_CL(equal(0.9f, integral.getY()));

	// test exceed max
	for (int i = 0; i < 100; i++) {
		integral.update(1.0f);
	}

	ASSERT_CL(equal(1.0f, integral.update(1.0f)));
	// test exceed min
	integral.setY(-0.9f);
	ASSERT_CL(equal(-0.9f, integral.getY()));

	for (int i = 0; i < 100; i++) {
		integral.update(-1.0f);
	}

	ASSERT_CL(equal(-1.0f, integral.update(-1.0f)));
	// test update
	integral.setY(0.1f);
	ASSERT_CL(equal(0.2f, integral.update(1.0)));
	ASSERT_CL(equal(0.2f, integral.getY()));
	printf("PASS\n");
	return 0;
}

int block_integral_trap_test()
{
	printf("Test BlockIntegralTrap\t\t: ");
	BlockIntegralTrap integral(nullptr, "TEST_I");
	// test initial state
	ASSERT_CL(equal(1.0f, integral.getMax()));
	ASSERT_CL(equal(0.0f, integral.getDt()));
	// set dt
	integral.setDt(0.1f);
	ASSERT_CL(equal(0.1f, integral.getDt()));
	// set U
	integral.setU(1.0f);
	ASSERT_CL(equal(1.0f, integral.getU()));
	// set Y
	integral.setY(0.9f);
	ASSERT_CL(equal(0.9f, integral.getY()));

	// test exceed max
	for (int i = 0; i < 100; i++) {
		integral.update(1.0f);
	}

	ASSERT_CL(equal(1.0f, integral.update(1.0f)));
	// test exceed min
	integral.setU(-1.0f);
	integral.setY(-0.9f);
	ASSERT_CL(equal(-0.9f, integral.getY()));

	for (int i = 0; i < 100; i++) {
		integral.update(-1.0f);
	}

	ASSERT_CL(equal(-1.0f, integral.update(-1.0f)));
	// test update
	integral.setU(2.0f);
	integral.setY(0.1f);
	ASSERT_CL(equal(0.25f, integral.update(1.0)));
	ASSERT_CL(equal(0.25f, integral.getY()));
	ASSERT_CL(equal(1.0f, integral.getU()));
	printf("PASS\n");
	return 0;
}

int block_derivative_test()
{
	printf("Test BlockDerivative\t\t: ");
	BlockDerivative derivative(nullptr, "TEST_D");
	// test initial state
	ASSERT_CL(equal(0.0f, derivative.getU()));
	ASSERT_CL(equal(10.0f, derivative.getLP()));
	// set dt
	derivative.setDt(0.1f);
	ASSERT_CL(equal(0.1f, derivative.getDt()));
	// set U
	derivative.setU(1.0f);
	ASSERT_CL(equal(1.0f, derivative.getU()));
	// perform one update so initialized is set
	derivative.update(1.0);
	ASSERT_CL(equal(1.0f, derivative.getU()));
	// test  update
	ASSERT_CL(equal(8.6269744f, derivative.update(2.0f)));
	ASSERT_CL(equal(2.0f, derivative.getU()));
	printf("PASS\n");
	return 0;
}

int block_p_test()
{
	printf("Test BlockP\t\t\t: ");
	BlockP block_p(nullptr, "TEST_P");
	// test initial state
	ASSERT_CL(equal(0.2f, block_p.getKP()));
	ASSERT_CL(equal(0.0f, block_p.getDt()));
	// set dt
	block_p.setDt(0.1f);
	ASSERT_CL(equal(0.1f, block_p.getDt()));
	// test  update
	ASSERT_CL(equal(0.4f, block_p.update(2.0f)));
	printf("PASS\n");
	return 0;
}

int block_pi_test()
{
	printf("Test BlockPI\t\t\t: ");
	BlockPI block_pi(nullptr, "TEST");
	// test initial state
	ASSERT_CL(equal(0.2f, block_pi.getKP()));
	ASSERT_CL(equal(0.1f, block_pi.getKI()));
	ASSERT_CL(equal(0.0f, block_pi.getDt()));
	ASSERT_CL(equal(1.0f, block_pi.getIntegral().getMax()));
	// set dt
	block_pi.setDt(0.1f);
	ASSERT_CL(equal(0.1f, block_pi.getDt()));
	// set integral state
	block_pi.getIntegral().setY(0.1f);
	ASSERT_CL(equal(0.1f, block_pi.getIntegral().getY()));
	// test  update
	// 0.2*2 + 0.1*(2*0.1 + 0.1) = 0.43
	ASSERT_CL(equal(0.43f, block_pi.update(2.0f)));
	printf("PASS\n");
	return 0;
}

int block_pd_test()
{
	printf("Test BlockPD\t\t\t: ");
	BlockPD block_pd(nullptr, "TEST");
	// test initial state
	ASSERT_CL(equal(0.2f, block_pd.getKP()));
	ASSERT_CL(equal(0.01f, block_pd.getKD()));
	ASSERT_CL(equal(0.0f, block_pd.getDt()));
	ASSERT_CL(equal(10.0f, block_pd.getDerivative().getLP()));
	// set dt
	block_pd.setDt(0.1f);
	ASSERT_CL(equal(0.1f, block_pd.getDt()));
	// set derivative state
	block_pd.getDerivative().setU(1.0f);
	ASSERT_CL(equal(1.0f, block_pd.getDerivative().getU()));
	// perform one update so initialized is set
	block_pd.getDerivative().update(1.0);
	ASSERT_CL(equal(1.0f, block_pd.getDerivative().getU()));
	// test  update
	// 0.2*2 + 0.1*(0.1*8.626...) = 0.486269744
	ASSERT_CL(equal(0.486269744f, block_pd.update(2.0f)));
	printf("PASS\n");
	return 0;
}

int block_pid_test()
{
	printf("Test BlockPID\t\t\t: ");
	BlockPID block_pid(nullptr, "TEST");
	// test initial state
	ASSERT_CL(equal(0.2f, block_pid.getKP()));
	ASSERT_CL(equal(0.1f, block_pid.getKI()));
	ASSERT_CL(equal(0.01f, block_pid.getKD()));
	ASSERT_CL(equal(0.0f, block_pid.getDt()));
	ASSERT_CL(equal(10.0f, block_pid.getDerivative().getLP()));
	ASSERT_CL(equal(1.0f, block_pid.getIntegral().getMax()));
	// set dt
	block_pid.setDt(0.1f);
	ASSERT_CL(equal(0.1f, block_pid.getDt()));
	// set derivative state
	block_pid.getDerivative().setU(1.0f);
	ASSERT_CL(equal(1.0f, block_pid.getDerivative().getU()));
	// perform one update so initialized is set
	block_pid.getDerivative().update(1.0);
	ASSERT_CL(equal(1.0f, block_pid.getDerivative().getU()));
	// set integral state
	block_pid.getIntegral().setY(0.1f);
	ASSERT_CL(equal(0.1f, block_pid.getIntegral().getY()));
	// test  update
	// 0.2*2 + 0.1*(2*0.1 + 0.1) + 0.1*(0.1*8.626...) = 0.5162697
	ASSERT_CL(equal(0.5162697f, block_pid.update(2.0f)));
	printf("PASS\n");
	return 0;
}

int block_output_test()
{
	printf("Test BlockOutput\t\t: ");
	BlockOutput block_output(nullptr, "TEST");
	// test initial state
	ASSERT_CL(equal(0.0f, block_output.getDt()));
	ASSERT_CL(equal(0.5f, block_output.get()));
	ASSERT_CL(equal(-1.0f, block_output.getMin()));
	ASSERT_CL(equal(1.0f, block_output.getMax()));
	// test update below min
	block_output.update(-2.0f);
	ASSERT_CL(equal(-1.0f, block_output.get()));
	// test update above max
	block_output.update(2.0f);
	ASSERT_CL(equal(1.0f, block_output.get()));
	// test trim
	block_output.update(0.0f);
	ASSERT_CL(equal(0.5f, block_output.get()));
	printf("PASS\n");
	return 0;
}

int block_rand_uniform_test()
{
	srand(1234);
	printf("Test BlockRandUniform\t\t: ");
	BlockRandUniform block_rand_uniform(nullptr, "TEST");
	// test initial state
	ASSERT_CL(equal(0.0f, block_rand_uniform.getDt()));
	ASSERT_CL(equal(-1.0f, block_rand_uniform.getMin()));
	ASSERT_CL(equal(1.0f, block_rand_uniform.getMax()));
	// test update
	int n = 10000;
	float mean = block_rand_uniform.update();

	for (int i = 2; i < n + 1; i++) {
		float val = block_rand_uniform.update();
		mean += (val - mean) / i;
		ASSERT_CL(less_than_or_equal(val, block_rand_uniform.getMax()));
		ASSERT_CL(greater_than_or_equal(val, block_rand_uniform.getMin()));
	}

	ASSERT_CL(equal(mean, (block_rand_uniform.getMin() +
			       block_rand_uniform.getMax()) / 2, 1e-1));
	printf("PASS\n");
	return 0;
}

int block_rand_gauss_test()
{
	srand(1234);
	printf("Test BlockRandGauss\t\t: ");
	BlockRandGauss block_rand_gauss(nullptr, "TEST");
	// test initial state
	ASSERT_CL(equal(0.0f, block_rand_gauss.getDt()));
	ASSERT_CL(equal(1.0f, block_rand_gauss.getMean()));
	ASSERT_CL(equal(2.0f, block_rand_gauss.getStdDev()));
	// test update
	int n = 10000;
	float mean = block_rand_gauss.update();
	float sum = 0;

	// recursive mean, stdev algorithm from Knuth
	for (int i = 2; i < n + 1; i++) {
		float val = block_rand_gauss.update();
		float new_mean = mean + (val - mean) / i;
		sum += (val - mean) * (val - new_mean);
		mean = new_mean;
	}

	float std_dev = sqrtf(sum / (n - 1));
	(void)(std_dev);
	ASSERT_CL(equal(mean, block_rand_gauss.getMean(), 1e-1));
	ASSERT_CL(equal(std_dev, block_rand_gauss.getStdDev(), 1e-1));
	printf("PASS\n");
	return 0;
}

int block_stats_test()
{
	printf("Test BlockStats\t\t\t: ");
	BlockStats<float, 1> stats(nullptr, "TEST");
	ASSERT_CL(equal(0.0f, stats.getMean()(0)));
	ASSERT_CL(equal(0.0f, stats.getStdDev()(0)));
	stats.update(matrix::Scalar<float>(1.0f));
	stats.update(matrix::Scalar<float>(2));
	ASSERT_CL(equal(1.5f, stats.getMean()(0)));
	ASSERT_CL(equal(0.5f, stats.getStdDev()(0)));
	stats.reset();
	ASSERT_CL(equal(0.0f, stats.getMean()(0)));
	ASSERT_CL(equal(0.0f, stats.getStdDev()(0)));
	printf("PASS\n");
	return 0;
}

int block_delay_test()
{
	printf("Test BlockDelay\t\t\t: ");
	using namespace matrix;
	BlockDelay<float, 2, 1, 3> delay(nullptr, "TEST");
	Vector2f u1(1, 2);
	Vector2f y1 = delay.update(u1);
	ASSERT_CL(equal(y1(0), u1(0)));
	ASSERT_CL(equal(y1(1), u1(1)));

	Vector2f u2(4, 5);
	Vector2f y2 = delay.update(u2);
	ASSERT_CL(equal(y2(0), u1(0)));
	ASSERT_CL(equal(y2(1), u1(1)));

	Vector2f u3(7, 8);
	Vector2f y3 = delay.update(u3);
	ASSERT_CL(equal(y3(0), u1(0)));
	ASSERT_CL(equal(y3(1), u1(1)));

	Vector2f u4(9, 10);
	Vector2f y4 = delay.update(u4);
	ASSERT_CL(equal(y4(0), u2(0)));
	ASSERT_CL(equal(y4(1), u2(1)));
	printf("PASS\n");
	return 0;
}

extern "C" __EXPORT int controllib_test_main(int argc, char *argv[]);

int controllib_test_main(int argc, char *argv[])
{
	return basic_blocks_test();
}

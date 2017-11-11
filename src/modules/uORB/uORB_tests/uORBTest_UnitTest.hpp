/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#ifndef _uORBTest_UnitTest_hpp_
#define _uORBTest_UnitTest_hpp_
#include "../uORBCommon.hpp"
#include "../uORB.h"
#include <px4_time.h>
#include <px4_tasks.h>

struct orb_test {
	int val;
	hrt_abstime time;
};
ORB_DECLARE(orb_test);
ORB_DECLARE(orb_multitest);


struct orb_test_medium {
	int val;
	hrt_abstime time;
	char junk[64];
};
ORB_DECLARE(orb_test_medium);
ORB_DECLARE(orb_test_medium_multi);
ORB_DECLARE(orb_test_medium_queue);
ORB_DECLARE(orb_test_medium_queue_poll);

struct orb_test_large {
	int val;
	hrt_abstime time;
	char junk[512];
};
ORB_DECLARE(orb_test_large);


namespace u_orb_test
{
class UnitTest;
}

class u_orb_test::UnitTest
{
public:

	// Singleton pattern
	static u_orb_test::UnitTest &instance();
	~UnitTest() {}
	int test();
	template<typename S> int latencyTest(orb_id_t t, bool print);
	int info();

private:
	UnitTest() : _pubsubtest_passed(false), _pubsubtest_print(false) {}

	// Disallow copy
	UnitTest(const u_orb_test::UnitTest & /*unused*/) = delete;

	static int pubsubtestThreadEntry(char *const argv[]);
	int pubsublatencyMain();

	static int pubTestMulti2Entry(char *const argv[]);
	int pubTestMulti2Main();

	volatile bool _thread_should_exit;

	bool _pubsubtest_passed;
	bool _pubsubtest_print;
	int _pubsubtest_res = OK;

	orb_advert_t _pfd[4]; ///< used for test_multi and test_multi_reversed

	int testSingle();

	/* These 3 depend on each other and must be called in this order */
	int testMulti();
	int testMultiReversed();
	int testUnadvertise();

	int testMulti2();

	/* queuing tests */
	int testQueue();
	static int pubTestQueueEntry(char *const argv[]);
	int pubTestQueueMain();
	int testQueuePollNotify();
	volatile int _num_messages_sent = 0;

	int testFail(const char *fmt, ...);
	int testNote(const char *fmt, ...);
};

template<typename S>
int u_orb_test::UnitTest::latencyTest(orb_id_t t, bool print)
{
	test_note("---------------- LATENCY TEST ------------------");
	S t;
	t.val = 308;
	t.time = hrt_absolute_time();

	orb_advert_t pfd0 = orb_advertise(t, &t);

	if (pfd0 == nullptr) {
		return test_fail("orb_advertise failed (%i)", errno);
	}

	char *const args[1] = { nullptr };

	pubsubtest_print = print;
	pubsubtest_passed = false;

	/* test pub / sub latency */

	// Can't pass a pointer in args, must be a null terminated
	// array of strings because the strings are copied to
	// prevent access if the caller data goes out of scope
	int pubsub_task = px4_task_spawn_cmd("uorb_latency",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 5,
					     1500,
					     (px4_main_t)&u_orb_test::UnitTest::pubsubtestThreadEntry,
					     args);

	/* give the test task some data */
	while (!pubsubtest_passed) {
		t.val = 308;
		t.time = hrt_absolute_time();

		if (PX4_OK != orb_publish(t, pfd0, &t)) {
			return test_fail("mult. pub0 timing fail");
		}

		/* simulate >800 Hz system operation */
		usleep(1000);
	}

	if (pubsub_task < 0) {
		return test_fail("failed launching task");
	}

	orb_unadvertise(pfd0);

	return pubsubtest_res;
}

#endif // _uORBTest_UnitTest_hpp_

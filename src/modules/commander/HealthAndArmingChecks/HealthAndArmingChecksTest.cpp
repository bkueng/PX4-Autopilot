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

#include <gtest/gtest.h>

#include "Common.hpp"
#include <uORB/topics/event.h>
#include <uORB/Subscription.hpp>

#include <stdint.h>

using namespace time_literals;

// to run: make tests TESTFILTER=HealthAndArmingChecks


class ReporterTest : public ::testing::Test

{
public:

	void SetUp() override
	{
		// ensure topic exists, otherwise we might lose first queued events
		orb_advertise_queue(ORB_ID(event), nullptr, event_s::ORB_QUEUE_LENGTH);
	}

};


TEST_F(ReporterTest, basic_no_checks)
{
	Report reporter{0_s};
	ASSERT_FALSE(reporter.canArm());

	reporter.reset(vehicle_status_s::NAVIGATION_STATE_MANUAL);
	reporter.finalize();
	reporter.report(false);

	ASSERT_TRUE(reporter.canArm());
	ASSERT_EQ((uint8_t)reporter.armingCheckResults().can_arm, 0xff);
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().warning, 0);

	ASSERT_EQ((uint64_t)reporter.healthResults().is_present, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().warning, 0);
}

TEST_F(ReporterTest, basic_fail_all_modes)
{
	Report reporter{0_s};

	// ensure arming is always denied with a ModeCategory::All failure
	for (uint8_t nav_state = 0; nav_state < vehicle_status_s::NAVIGATION_STATE_MAX; ++nav_state) {
		reporter.reset(nav_state);
		reporter.armingCheckFailure(ModeCategory::All, HealthComponentIndex::manual_control_input,
					    events::ID("arming_test_basic_fail_all_modes_fail1"), "", events::Log::Info);
		reporter.finalize();
		reporter.report(false);

		ASSERT_FALSE(reporter.canArm());
	}
}

TEST_F(ReporterTest, arming_checks_mode_category)
{
	Report reporter{0_s};

	// arming must still be possible for non-relevant failures
	reporter.reset(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	reporter.armingCheckFailure(ModeCategory::AllManualModes, HealthComponentIndex::manual_control_input,
				    events::ID("arming_test_arming_checks_mode_category_fail1"), "", events::Log::Warning);
	reporter.healthFailure(ModeCategory::Position, HealthComponentIndex::local_position_estimate,
			       events::ID("arming_test_arming_checks_mode_category_fail2"), "", events::Log::Info);
	reporter.setIsPresent(HealthComponentIndex::battery);
	reporter.finalize();
	reporter.report(false);

	ASSERT_TRUE(reporter.canArm());

	ASSERT_EQ((uint8_t)reporter.armingCheckResults().can_arm, (uint8_t)~ModeCategory::AllManualModes);
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().error, 0);
	ASSERT_EQ(reporter.armingCheckResults().warning, events::common::enums::health_component_t::manual_control_input);

	ASSERT_EQ(reporter.healthResults().is_present, events::common::enums::health_component_t::battery);
	ASSERT_EQ((uint64_t)reporter.healthResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().warning, 0);
}

TEST_F(ReporterTest, arming_checks_mode_category2)
{
	Report reporter{0_s};

	// A matching mode category must deny arming
	reporter.reset(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF);
	reporter.healthFailure(ModeCategory::Autonomous, HealthComponentIndex::manual_control_input,
			       events::ID("arming_test_arming_checks_mode_category2_fail1"), "", events::Log::Warning);
	reporter.finalize();
	reporter.report(false);

	ASSERT_FALSE(reporter.canArm());

	ASSERT_EQ((uint8_t)reporter.armingCheckResults().can_arm, (uint8_t)~(ModeCategory::Autonomous | ModeCategory::Current));
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().warning, 0);

	ASSERT_EQ((uint64_t)reporter.healthResults().is_present, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().error, 0);
	ASSERT_EQ(reporter.healthResults().warning, events::common::enums::health_component_t::manual_control_input);
}

TEST_F(ReporterTest, reporting)
{
	Report reporter{0_s};

	uORB::Subscription event_sub{ORB_ID(event)};
	event_sub.subscribe();
	event_s event;

	while (event_sub.update(&event)); // clear all updates

	for (int j = 0; j < 2; ++j) { // test with and without additional report arguments
		const bool with_arg = j == 0;

		for (int i = 0; i < 3; ++i) { // repeat same report: we expect reporting only the first time
			reporter.reset(vehicle_status_s::NAVIGATION_STATE_POSCTL);

			if (with_arg) {
				reporter.armingCheckFailure<uint16_t>(ModeCategory::All, HealthComponentIndex::manual_control_input,
								      events::ID("arming_test_reporting_fail1"), "", events::Log::Warning, 4938);

			} else {
				reporter.armingCheckFailure(ModeCategory::Position, HealthComponentIndex::manual_control_input,
							    events::ID("arming_test_reporting_fail2"), "", events::Log::Warning);
			}

			reporter.finalize();
			reporter.report(false);
			ASSERT_FALSE(reporter.canArm());

			if (i == 0) {
				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, (uint32_t)events::common::event_id_t::arming_check_summary);
				ASSERT_TRUE(event_sub.update(&event));

				if (with_arg) {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail1"));

				} else {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail2"));
				}

				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, (uint32_t)events::common::event_id_t::health_summary);

			} else {
				ASSERT_FALSE(event_sub.updated());
			}
		}
	}

	// now the same for health
	for (int j = 0; j < 2; ++j) {
		const bool with_arg = j == 0;

		for (int i = 0; i < 3; ++i) {
			reporter.reset(vehicle_status_s::NAVIGATION_STATE_POSCTL);

			if (with_arg) {
				reporter.healthFailure<uint16_t>(ModeCategory::All, HealthComponentIndex::manual_control_input,
								 events::ID("arming_test_reporting_fail3"), "", events::Log::Warning, 4938);

			} else {
				reporter.healthFailure(ModeCategory::Position, HealthComponentIndex::manual_control_input,
						       events::ID("arming_test_reporting_fail4"), "", events::Log::Warning);
			}

			reporter.finalize();
			reporter.report(false);
			ASSERT_FALSE(reporter.canArm());

			if (i == 0) {
				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, (uint32_t)events::common::event_id_t::arming_check_summary);
				ASSERT_TRUE(event_sub.update(&event));

				if (with_arg) {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail3"));

				} else {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail4"));
				}

				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, (uint32_t)events::common::event_id_t::health_summary);

			} else {
				ASSERT_FALSE(event_sub.updated());
			}
		}
	}

}

TEST_F(ReporterTest, reporting_multiple)
{
	Report reporter{0_s};

	uORB::Subscription event_sub{ORB_ID(event)};
	event_sub.subscribe();
	event_s event;

	while (event_sub.update(&event)); // clear all updates

	for (int i = 0; i < 3; ++i) {
		reporter.reset(vehicle_status_s::NAVIGATION_STATE_POSCTL);
		reporter.armingCheckFailure<uint16_t>(ModeCategory::All, HealthComponentIndex::manual_control_input,
						      events::ID("arming_test_reporting_multiple_fail1"), "", events::Log::Warning, 4938);
		reporter.armingCheckFailure<float>(ModeCategory::All, HealthComponentIndex::manual_control_input,
						   events::ID("arming_test_reporting_multiple_fail2"), "", events::Log::Warning, 123.f);
		reporter.armingCheckFailure<uint8_t>(ModeCategory::All, HealthComponentIndex::manual_control_input,
						     events::ID("arming_test_reporting_multiple_fail3"), "", events::Log::Warning, 55);
		reporter.finalize();
		reporter.report(false);
		ASSERT_FALSE(reporter.canArm());

		if (i == 0) {
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, (uint32_t)events::common::event_id_t::arming_check_summary);
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("arming_test_reporting_multiple_fail1"));
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("arming_test_reporting_multiple_fail2"));
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("arming_test_reporting_multiple_fail3"));
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, (uint32_t)events::common::event_id_t::health_summary);

		} else {
			ASSERT_FALSE(event_sub.updated());
		}
	}
}


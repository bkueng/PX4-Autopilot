/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#pragma once


#include <px4_arch/io_timer.h>
#include <px4_arch/hw_description.h>
#include <px4_platform_common/constexpr_util.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/io_timer_init.h>

#include <hardware/imxrt_tmr.h>
#include <hardware/imxrt_flexpwm.h>
#include <imxrt_gpio.h>
#include <imxrt_iomuxc.h>
#include <hardware/imxrt_pinmux.h>


static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		PWM::FlexPWMConfig pwm_config, IOMUX::Pad pad)
{
	timer_io_channels_t ret{};

	PWM::FlexPWM pwm{};

	// FlexPWM Muxing Options
	switch (pwm_config.module) {
	case PWM::PWM1_PWM_A:
		pwm = PWM::PWM1;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_23) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_23_INDEX);
				ret.gpio_portpin = GPIO_PORT4 | GPIO_PIN23;

			} else if (pad == IOMUX::Pad::GPIO_SD_B0_00) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_SD_B0_00_INDEX);
				ret.gpio_portpin = GPIO_PORT3 | GPIO_PIN12;
			}

			break;

		case PWM::Submodule1:
			// TODO
			break;

		case PWM::Submodule2:
			break;

		case PWM::Submodule3:
			break;
		}

		break;

	case PWM::PWM1_PWM_B:
		pwm = PWM::PWM1;
		break;

	case PWM::PWM1_PWM_X:
		pwm = PWM::PWM1;
		break;

	case PWM::PWM2_PWM_A:
		pwm = PWM::PWM2;
		break;

	case PWM::PWM2_PWM_B:
		pwm = PWM::PWM2;
		break;

	case PWM::PWM3_PWM_A:
		pwm = PWM::PWM3;
		break;

	case PWM::PWM3_PWM_B:
		pwm = PWM::PWM3;
		break;

	case PWM::PWM4_PWM_A:
		pwm = PWM::PWM4;
		break;

	case PWM::PWM4_PWM_B:
		pwm = PWM::PWM4;
		break;
	}

	constexpr_assert(ret.gpio_out != 0, "Invalid PWM config");
	ret.gpio_out |= IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST;

	switch (pwm_config.module) {
	case PWM::PWM1_PWM_A:
	case PWM::PWM2_PWM_A:
	case PWM::PWM3_PWM_A:
	case PWM::PWM4_PWM_A:
		ret.val_offset = PWMA_VAL;
		break;

	case PWM::PWM1_PWM_B:
	case PWM::PWM2_PWM_B:
	case PWM::PWM3_PWM_B:
	case PWM::PWM4_PWM_B:
		ret.val_offset = PWMB_VAL;
		break;

	default:
		constexpr_assert(false, "not implemented");
	}

	switch (pwm_config.submodule) {
	case PWM::Submodule0:
		ret.sub_module = SM0;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM0);
		break;

	case PWM::Submodule1:
		ret.sub_module = SM1;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM1);
		break;

	case PWM::Submodule2:
		ret.sub_module = SM2;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM2);
		break;

	case PWM::Submodule3:
		ret.sub_module = SM3;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM3);
		break;
	}

	ret.gpio_in = ret.gpio_portpin; // TODO

	// find timer index
	ret.timer_index = 0xff;
	const uint32_t timer_base = getFlexPWMBaseRegister(pwm);

	for (int i = 0; i < MAX_IO_TIMERS; ++i) {
		if (io_timers_conf[i].base == timer_base) {
			ret.timer_index = i;
			break;
		}
	}

	constexpr_assert(ret.timer_index != 0xff, "Timer not found");

	return ret;
}

static inline constexpr io_timers_t initIOPWM(PWM::FlexPWM pwm)
{
	io_timers_t ret{};

	ret.base = getFlexPWMBaseRegister(pwm);

	return ret;
}



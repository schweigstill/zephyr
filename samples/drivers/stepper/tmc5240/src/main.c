/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Nathan Perry <np@npry.dev>
 * SPDX-License-Identifier: MIT OR Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/stepper/tmc5240.h>

LOG_MODULE_REGISTER(tmc5240_demo);
static const struct device *STEPPER = DEVICE_DT_GET_ANY(adi_tmc5240);

#define READ_REG(reg, val)  TMC5240_READ_REG(STEPPER, reg, val)
#define WRITE_REG(reg, val) TMC5240_WRITE_REG(STEPPER, reg, val)

#define DEFAULT_CUR_HOLD 10 // out of 31
#define DEFAULT_CUR_RUN  7  // out of 31

#define DEFAULT_ACCEL 4000
#define DEFAULT_VEL   100000
#define DEFAULT_D1_D2 0xa

#define CHECKED(log, fn)                                                                           \
	LOG_INF(log);                                                                              \
	ret = fn;                                                                                  \
	if (ret < 0)                                                                               \
		return ret;

#define MOTION(log, fn, sleep_ms)                                                                  \
	CHECKED(log, fn)                                                                           \
	k_sleep(K_MSEC(sleep_ms));

static inline int reset_simple()
{
	WRITE_REG(GCONF, TMC5240_GCONF_DEFAULT)
	WRITE_REG(GLOBAL_SCALER, 0) // full-scale
	WRITE_REG(SW_MODE, 0)       // all limit switches disabled
	WRITE_REG(DRV_CONF, 0)
	WRITE_REG(RAMP_STAT, 0xff) // clear RAMP_STAT
	WRITE_REG(IHOLD_IRUN, (DEFAULT_CUR_RUN << 8) | DEFAULT_CUR_HOLD)
	WRITE_REG(CHOPCONF, TMC5240_CHOPCONF_DRVEN) // enable driver

	// default vstop/vstart
	WRITE_REG(VSTOP, 10)
	WRITE_REG(VSTART, 0)

	// disable V1, V2 portions of trapezoid
	WRITE_REG(V1, 0)
	WRITE_REG(V2, 0)
	WRITE_REG(A1, 0)
	WRITE_REG(A2, 0)
	WRITE_REG(D1, DEFAULT_D1_D2)
	WRITE_REG(D2, DEFAULT_D1_D2)
	WRITE_REG(VDCMIN, 0)

	WRITE_REG(THIGH, 0)
	WRITE_REG(TVMAX, 0)
	WRITE_REG(TCOOLTHRS, 0)
	WRITE_REG(TPWMTHRS, 0)

	// set ramp-up and ramp-down accel
	WRITE_REG(AMAX, DEFAULT_ACCEL)
	WRITE_REG(DMAX, DEFAULT_ACCEL)

	// first pass: ensure driver is ok
	READ_REG(IOIN, ioin)
	READ_REG(DRV_STATUS, drv_stat)

	if (ioin & TMC5240_IOIN_ADC_ERR) {
		LOG_ERR("%s: adc error", STEPPER->name);
		return -EIO;
	}

	if (ioin & TMC5240_IOIN_DRV_ENN) {
		LOG_WRN("%s: drivers not enabled", STEPPER->name);
		return -EIO;
	}

	// set to hold at 0 velocity
	WRITE_REG(VMAX, 0)
	WRITE_REG(RAMPMODE, TMC5240_RAMPMODE_HOLD)

	return 0;
}

static inline int spin_simple(int32_t velocity)
{
	int ret = reset_simple();
	if (ret < 0) {
		return ret;
	}

	if (velocity > 0) {
		WRITE_REG(RAMPMODE, TMC5240_RAMPMODE_VEL)
	} else {
		WRITE_REG(RAMPMODE, TMC5240_RAMPMODE_NVEL)
	}

	if (velocity < 0) {
		velocity = -velocity;
	}

	WRITE_REG(AMAX, DEFAULT_ACCEL)
	WRITE_REG(DMAX, DEFAULT_ACCEL)
	WRITE_REG(VMAX, (uint32_t)velocity)

	return 0;
}

static inline int pos_simple(int32_t target)
{
	int ret = reset_simple();
	if (ret < 0) {
		return ret;
	}

	WRITE_REG(RAMPMODE, TMC5240_RAMPMODE_POS)
	WRITE_REG(VMAX, DEFAULT_VEL)
	WRITE_REG(AMAX, DEFAULT_ACCEL)
	WRITE_REG(DMAX, DEFAULT_ACCEL)
	WRITE_REG(XACTUAL, 0)
	WRITE_REG(XTARGET, target)

	return 0;
}

static inline void wait_ready()
{
	while (!device_is_ready(STEPPER)) {
		LOG_WRN_ONCE("stepper not ready, waiting");

		k_sleep(Z_TIMEOUT_MS(500));
		tmc5240_init(STEPPER);
	}
}

static inline int register_api()
{
	int ret;

	MOTION("velocity mode, forward", spin_simple(DEFAULT_VEL), 2000)
	MOTION("velocity mode, reverse (fast)", spin_simple(-DEFAULT_VEL * 4), 2000)
	MOTION("stop", spin_simple(0), 1000)
	MOTION("positioning mode, forward", pos_simple(25000), 1500)
	MOTION("positioning mode, reverse", pos_simple(-25000), 1500)

	return 0;
}

static inline int stepper_api()
{
	int ret;

	MOTION("vel pos",
	       stepper_enable_constant_velocity_mode(STEPPER, STEPPER_DIRECTION_POSITIVE,
						     DEFAULT_VEL),
	       2000)

	MOTION("stop",
	       stepper_enable_constant_velocity_mode(STEPPER, STEPPER_DIRECTION_POSITIVE, 0), 1000)

	MOTION("vel neg",
	       stepper_enable_constant_velocity_mode(STEPPER, STEPPER_DIRECTION_NEGATIVE,
						     DEFAULT_VEL * 4),
	       2000)

	MOTION("stop",
	       stepper_enable_constant_velocity_mode(STEPPER, STEPPER_DIRECTION_POSITIVE, 0), 1000)

	MOTION("positioning mode, forward", stepper_move(STEPPER, 25000), 1500)
	MOTION("positioning mode, reverse", stepper_move(STEPPER, -25000), 1500);

	return ret;
}

static inline int tmc5240_api()
{
	int ret;

	MOTION("vel pos", tmc5240_mode_velocity(STEPPER, DEFAULT_VEL), 2000)
	MOTION("stop", tmc5240_mode_velocity(STEPPER, 0), 1000)

	MOTION("vel neg", tmc5240_mode_velocity(STEPPER, -DEFAULT_VEL * 4), 2000)
	MOTION("stop", tmc5240_mode_velocity(STEPPER, 0), 1000)

	CHECKED("zero", tmc5240_zero_pos(STEPPER))

	MOTION("positioning mode, forward", tmc5240_mode_position(STEPPER, 25000), 1500)
	MOTION("positioning mode, reverse", tmc5240_mode_position(STEPPER, -25000), 1500);

	return ret;
}

#define MODE_REGISTER_API 1
#define MODE_STEPPER_API  2
#define MODE_TMC5240_API  3

// #define MODE MODE_REGISTER_API
#define MODE MODE_STEPPER_API

int main(void)
{
	wait_ready();

	int ret;

	uint32_t regs[TMC5240_REG_MAX];
	CHECKED("dump regs", tmc5240_regdump(STEPPER, regs));
	for (int i = 0; i < TMC5240_NREG; i++) {
		uint8_t reg = TMC5240_ALL_REGS[i];
		k_sleep(K_MSEC(5));
		LOG_INF("loaded reg %#x with value %#x", reg, regs[reg]);
	}

	WRITE_REG(VMAX, 0xa5a5a5);

	CHECKED("write regs", tmc5240_regload(STEPPER, regs, false));

	CHECKED("check device", tmc5240_check(STEPPER));

#if MODE == MODE_REGISTER_API
	while (true) {
		register_api();
	}
#else
	struct tmc5240_motion_config motion_config = TMC5240_MOTION_CONFIG_DEFAULT;
	motion_config.amax = DEFAULT_ACCEL;
	motion_config.dmax = DEFAULT_ACCEL;

	motion_config.ihold = 13;
	motion_config.irun = 10;

	CHECKED("set motion config", tmc5240_set_motion_config(STEPPER, &motion_config));
	CHECKED("enable stepper", stepper_enable(STEPPER, true))

	while (true) {
#if MODE == MODE_STEPPER_API
		ret = stepper_api();
#elif MODE == MODE_TMC5240_API
		ret = tmc5240_api();
#else
#error "unknown mode"
#endif

		if (ret < 0) {
			LOG_ERR("motion error: %i", ret);
		}
	}
#endif
}

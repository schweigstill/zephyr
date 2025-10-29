/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Nathan Perry <np@npry.dev>
 * SPDX-License-Identifier: MIT OR Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_TMC5240_H_
#define ZEPHYR_INCLUDE_DRIVERS_TMC5240_H_

#include <unistd.h>

/**
 * A register operation for submission to `tmc5240_transact`.
 *
 * - `force` indicates that if CONFIG_TMC5240_STEPPER_SHADOW is enabled, the shadow register file
 *  is bypassed and the operation is always dispatched to the bus.
 */
struct tmc5240_op {
	bool write, force;

	union {
		uint8_t address;
		uint8_t status;
	};

	uint32_t *value;
};

/**
 * Persistent motion configuration.
 *
 * Please note the value of vmax configured here pertains only to positioning mode. This value _is_
 * updated by stepper_set_max_velocity.
 */
struct tmc5240_motion_config {
	uint32_t amax, dmax, vmax, a1, d1, a2, d2, vstop, vstart, v1, v2, vdcmin, thigh, tzerowait,
		tcoolthrs, tvmax, tpowerdown, tpwmthrs;

	uint8_t ihold, irun, iholddelay, irundelay;
};

enum tmc5240_rampmode {
	TMC5240_RAMPMODE_POS = 0x0,
	TMC5240_RAMPMODE_VEL = 0x1,
	TMC5240_RAMPMODE_NVEL = 0x2,
	TMC5240_RAMPMODE_HOLD = 0x3,
};

enum tmc5240_register {
	// General configuration
	TMC5240_REG_GCONF = 0x0,
	TMC5240_REG_GSTAT = 0x1,
	TMC5240_REG_IFCNT = 0x2,
	TMC5240_REG_NODECONF = 0x3,
	TMC5240_REG_IOIN = 0x4,
	TMC5240_REG_X_COMPARE = 0x5,
	TMC5240_REG_X_COMPARE_REPEAT = 0x6,

	TMC5240_REG_DRV_CONF = 0x0a,
	TMC5240_REG_GLOBAL_SCALER = 0x0b,

	// Velocity-dependent configuration
	TMC5240_REG_IHOLD_IRUN = 0x10,
	TMC5240_REG_TPOWERDOWN = 0x11,
	TMC5240_REG_TSTEP = 0x12,
	TMC5240_REG_TPWMTHRS = 0x13,
	TMC5240_REG_TCOOLTHRS = 0x14,
	TMC5240_REG_THIGH = 0x15,

	// Ramp generator
	TMC5240_REG_RAMPMODE = 0x20,
	TMC5240_REG_XACTUAL = 0x21,
	TMC5240_REG_VACTUAL = 0x22,
	TMC5240_REG_VSTART = 0x23,
	TMC5240_REG_A1 = 0x24,
	TMC5240_REG_V1 = 0x25,
	TMC5240_REG_AMAX = 0x26,
	TMC5240_REG_VMAX = 0x27,
	TMC5240_REG_DMAX = 0x28,
	TMC5240_REG_TVMAX = 0x29,
	TMC5240_REG_D1 = 0x2a,
	TMC5240_REG_VSTOP = 0x2b,
	TMC5240_REG_TZEROWAIT = 0x2c,
	TMC5240_REG_XTARGET = 0x2d,
	TMC5240_REG_V2 = 0x2e,
	TMC5240_REG_A2 = 0x2f,
	TMC5240_REG_D2 = 0x30,

	// Ramp generator driver features
	TMC5240_REG_VDCMIN = 0x33,
	TMC5240_REG_SW_MODE = 0x34,
	TMC5240_REG_RAMP_STAT = 0x35,
	TMC5240_REG_XLATCH = 0x36,

	// Encoder
	TMC5240_REG_ENCMODE = 0x38,
	TMC5240_REG_X_ENC = 0x39,
	TMC5240_REG_ENC_CONST = 0x3a,
	TMC5240_REG_ENC_STATUS = 0x3b,
	TMC5240_REG_ENC_LATCH = 0x3c,
	TMC5240_REG_ENC_DEVIATION = 0x3d,
	TMC5240_REG_VIRTUAL_STOP_L = 0x3e,
	TMC5240_REG_VIRTUAL_STOP_R = 0x3f,

	// ADC
	TMC5240_REG_ADC_VSUPPLY_AIN = 0x50,
	TMC5240_REG_ADC_TEMP = 0x51,
	TMC5240_REG_OTW_OV_VTH = 0x52,

	// Motor driver
	TMC5240_REG_MSLUT_0 = 0x60,
	TMC5240_REG_MSLUT_1 = 0x61,
	TMC5240_REG_MSLUT_2 = 0x62,
	TMC5240_REG_MSLUT_3 = 0x63,
	TMC5240_REG_MSLUT_4 = 0x64,
	TMC5240_REG_MSLUT_5 = 0x65,
	TMC5240_REG_MSLUT_6 = 0x66,
	TMC5240_REG_MSLUT_7 = 0x67,
	TMC5240_REG_MSLUTSEL = 0x68,
	TMC5240_REG_MSLUTSTART = 0x69,
	TMC5240_REG_MSCNT = 0x6a,
	TMC5240_REG_MSCURACT = 0x6b,
	TMC5240_REG_CHOPCONF = 0x6c,
	TMC5240_REG_COOLCONF = 0x6d,
	TMC5240_REG_DCCTRL = 0x6e,
	TMC5240_REG_DRV_STATUS = 0x6f,
	TMC5240_REG_PWMCONF = 0x70,
	TMC5240_REG_PWM_SCALE = 0x71,
	TMC5240_REG_PWM_AUTO = 0x72,
	TMC5240_REG_SG4_THRS = 0x74,
	TMC5240_REG_SG4_RESULT = 0x75,
	TMC5240_REG_SG4_IND = 0x76,
};

static uint8_t TMC5240_ALL_REGS[] = {
	TMC5240_REG_GCONF,
	TMC5240_REG_GSTAT,
	TMC5240_REG_IFCNT,
	TMC5240_REG_NODECONF,
	TMC5240_REG_IOIN,
	TMC5240_REG_X_COMPARE,
	TMC5240_REG_X_COMPARE_REPEAT,

	TMC5240_REG_DRV_CONF,
	TMC5240_REG_GLOBAL_SCALER,

	TMC5240_REG_IHOLD_IRUN,
	TMC5240_REG_TPOWERDOWN,
	TMC5240_REG_TSTEP,
	TMC5240_REG_TPWMTHRS,
	TMC5240_REG_TCOOLTHRS,
	TMC5240_REG_THIGH,

	TMC5240_REG_RAMPMODE,
	TMC5240_REG_XACTUAL,
	TMC5240_REG_VACTUAL,
	TMC5240_REG_VSTART,
	TMC5240_REG_A1,
	TMC5240_REG_V1,
	TMC5240_REG_AMAX,
	TMC5240_REG_VMAX,
	TMC5240_REG_DMAX,
	TMC5240_REG_TVMAX,
	TMC5240_REG_D1,
	TMC5240_REG_VSTOP,
	TMC5240_REG_TZEROWAIT,
	TMC5240_REG_XTARGET,
	TMC5240_REG_V2,
	TMC5240_REG_A2,
	TMC5240_REG_D2,

	TMC5240_REG_VDCMIN,
	TMC5240_REG_SW_MODE,
	TMC5240_REG_RAMP_STAT,
	TMC5240_REG_XLATCH,

	TMC5240_REG_ENCMODE,
	TMC5240_REG_X_ENC,
	TMC5240_REG_ENC_CONST,
	TMC5240_REG_ENC_STATUS,
	TMC5240_REG_ENC_LATCH,
	TMC5240_REG_ENC_DEVIATION,
	TMC5240_REG_VIRTUAL_STOP_L,
	TMC5240_REG_VIRTUAL_STOP_R,

	TMC5240_REG_ADC_VSUPPLY_AIN,
	TMC5240_REG_ADC_TEMP,
	TMC5240_REG_OTW_OV_VTH,

	TMC5240_REG_MSLUT_0,
	TMC5240_REG_MSLUT_1,
	TMC5240_REG_MSLUT_2,
	TMC5240_REG_MSLUT_3,
	TMC5240_REG_MSLUT_4,
	TMC5240_REG_MSLUT_5,
	TMC5240_REG_MSLUT_6,
	TMC5240_REG_MSLUT_7,
	TMC5240_REG_MSLUTSEL,
	TMC5240_REG_MSLUTSTART,
	TMC5240_REG_MSCNT,
	TMC5240_REG_MSCURACT,
	TMC5240_REG_CHOPCONF,
	TMC5240_REG_COOLCONF,
	TMC5240_REG_DCCTRL,
	TMC5240_REG_DRV_STATUS,
	TMC5240_REG_PWMCONF,
	TMC5240_REG_PWM_SCALE,
	TMC5240_REG_PWM_AUTO,
	TMC5240_REG_SG4_THRS,
	TMC5240_REG_SG4_RESULT,
	TMC5240_REG_SG4_IND,
};

static inline bool tmc5240_reg_is_volatile(uint8_t reg)
{
	switch (reg) {
	case TMC5240_REG_XACTUAL:
	case TMC5240_REG_VACTUAL:
	case TMC5240_REG_GSTAT:
	case TMC5240_REG_RAMP_STAT:
	case TMC5240_REG_ENC_STATUS:
	case TMC5240_REG_ENC_LATCH:
	case TMC5240_REG_XLATCH:
	case TMC5240_REG_TSTEP:
	case TMC5240_REG_IOIN:
	case TMC5240_REG_ADC_VSUPPLY_AIN:
	case TMC5240_REG_ADC_TEMP:
	case TMC5240_REG_MSCNT:
	case TMC5240_REG_MSCURACT:
	case TMC5240_REG_DRV_STATUS:
	case TMC5240_REG_PWM_SCALE:
	case TMC5240_REG_PWM_AUTO:
	case TMC5240_REG_SG4_RESULT:
	case TMC5240_REG_SG4_IND:
		return true;

	default:
		return false;
	}
}

#define TMC5240_NREG      (sizeof(TMC5240_ALL_REGS))
#define TMC5240_REG_MIN   TMC5240_REG_GSTAT
#define TMC5240_REG_MAX   TMC5240_REG_SG4_IND
#define TMC5240_REG_RANGE (TMC5240_REG_MAX - TMC5240_REG_MIN)

#define TMC5240_STATUS_STOP_R      BIT(7)
#define TMC5240_STATUS_STOP_L      BIT(6)
#define TMC5240_STATUS_POS_REACHED BIT(5)
#define TMC5240_STATUS_VEL_REACHED BIT(4)
#define TMC5240_STATUS_STANDSTILL  BIT(3)
#define TMC5240_STATUS_SG2         BIT(2)
#define TMC5240_STATUS_DRV_ERR     BIT(1)
#define TMC5240_STATUS_RESET       BIT(0)

#define TMC5240_CHOPCONF_INTPOL         BIT(28)
#define TMC5240_CHOPCONF_TBL_DEFAULT    (0b10 << 15)
#define TMC5240_CHOPCONF_HSTART_DEFAULT (0x2 << 7)
#define TMC5240_CHOPCONF_HEND_DEFAULT   (0x5 << 4)
#define TMC5240_CHOPCONF_DEFAULT                                                                   \
	(TMC5240_CHOPCONF_INTPOL | TMC5240_CHOPCONF_TBL_DEFAULT |                                  \
	 TMC5240_CHOPCONF_HSTART_DEFAULT | TMC5240_CHOPCONF_HEND_DEFAULT)
#define TMC5240_CHOPCONF_DRVEN_MASK 0xf
#define TMC5240_CHOPCONF_DRVEN      (TMC5240_CHOPCONF_DEFAULT | 0b0010)

#define TMC5240_GCONF_DEFAULT (0b110 | BIT(14))

#define TMC5240_SW_MODE_DEFAULT 0x0
#define TMC5240_RAMP_STAT_CLEAR 0xff

#define TMC5240_RAMP_STAT_VSTOP_R         BIT(15)
#define TMC5240_RAMP_STAT_VSTOP_L         BIT(14)
#define TMC5240_RAMP_STAT_SG              BIT(13)
#define TMC5240_RAMP_STAT_SECOND_MOVE     BIT(12)
#define TMC5240_RAMP_STAT_T_ZEROWAIT      BIT(11)
#define TMC5240_RAMP_STAT_VZERO           BIT(10)
#define TMC5240_RAMP_STAT_POS_REACHED     BIT(9)
#define TMC5240_RAMP_STAT_VEL_REACHED     BIT(8)
#define TMC5240_RAMP_STAT_EVT_POS_REACHED BIT(7)
#define TMC5240_RAMP_STAT_EVT_STOP_SG     BIT(6)
#define TMC5240_RAMP_STAT_EVT_STOP_R      BIT(5)
#define TMC5240_RAMP_STAT_EVT_STOP_L      BIT(4)
#define TMC5240_RAMP_STAT_LATCH_R         BIT(3)
#define TMC5240_RAMP_STAT_LATCH_L         BIT(2)
#define TMC5240_RAMP_STAT_STOP_R          BIT(1)
#define TMC5240_RAMP_STAT_STOP_L          BIT(0)

#define TMC5240_IOIN_ADC_ERR     BIT(15)
#define TMC5240_IOIN_DRV_ENN     BIT(4)
#define TMC5240_IOIN_EXT_RES_DET BIT(3)

#define TMC5240_DRV_STATUS_OLB   BIT(30)
#define TMC5240_DRV_STATUS_OLA   BIT(29)
#define TMC5240_DRV_STATUS_S2GB  BIT(28)
#define TMC5240_DRV_STATUS_S2GA  BIT(27)
#define TMC5240_DRV_STATUS_OTPW  BIT(26)
#define TMC5240_DRV_STATUS_OT    BIT(25)
#define TMC5240_DRV_STATUS_SG    BIT(24)
#define TMC5240_DRV_STATUS_S2VSB BIT(13)
#define TMC5240_DRV_STATUS_S2VSA BIT(12)

#define TMC5240_OP_WRITE(reg, value)                                                               \
	{                                                                                          \
		.address = TMC5240_REG_##reg, .write = true, .va##lue = (value), .force = false,   \
	}

#define TMC5240_OP_READ(reg, value)                                                                \
	{                                                                                          \
		.address = TMC5240_REG_##reg, .write = false, .va##lue = (value), .force = false,  \
	}

#define TMC5240_OP_WRITE_FORCED(reg, value)                                                        \
	{                                                                                          \
		.address = TMC5240_REG_##reg, .write = true, .va##lue = (value), .force = true,    \
	}

#define TMC5240_OP_READ_FORCED(reg, value)                                                         \
	{                                                                                          \
		.address = TMC5240_REG_##reg, .write = false, .va##lue = (value), .force = true,   \
	}

#define TMC5240_READ_REG(dev, reg, val)                                                            \
	int val;                                                                                   \
	int __ret_##reg##__LINE__ = tmc5240_read_reg(dev, TMC5240_REG_##reg, &val, false);         \
	if (__ret_##reg##__LINE__ < 0) {                                                           \
		return __ret_##reg##__LINE__;                                                      \
	}

#define TMC5240_WRITE_REG(dev, reg, val)                                                           \
	int __ret_wr_##reg##__LINE__ = tmc5240_write_reg(dev, TMC5240_REG_##reg, val, false);      \
	if (__ret_wr_##reg##__LINE__ < 0) {                                                        \
		return __ret_wr_##reg##__LINE__;                                                   \
	}

#define TMC5240_MOTION_CONFIG_DEFAULT                                                              \
	{                                                                                          \
		.amax = 1000, .dmax = 1000, .a1 = 0, .d1 = 0xa, .vmax = 100000, .thigh = 0,        \
		.vstop = 10, .vstart = 0, .v1 = 0, .v2 = 0, .vdcmin = 0, .tcoolthrs = 0, .a2 = 0,  \
		.d2 = 0xa, .ihold = 10, .irun = 7, .irundelay = 0x4, .iholddelay = 0x1,            \
		.tzerowait = 0, .tvmax = 0, .tpowerdown = 0xa, .tpwmthrs = 0x0,                    \
	}

#define TMC5240_MOTION_CONFIG_IHOLD_IRUN(cfg)                                                      \
	(((cfg)->ihold & 0x1f) | ((cfg)->irun & 0x1f) << 8 | (cfg)->iholddelay << 16 |             \
	 (cfg)->irundelay << 24)

#define TMC5240_MOTION_CONFIG_WRITES(cfg)                                                          \
	TMC5240_OP_WRITE(V1, &(cfg)->v1), TMC5240_OP_WRITE(V2, &(cfg)->v2),                        \
		TMC5240_OP_WRITE(A1, &(cfg)->a1), TMC5240_OP_WRITE(A2, &(cfg)->a2),                \
		TMC5240_OP_WRITE(D1, &(cfg)->d1), TMC5240_OP_WRITE(D2, &(cfg)->d2),                \
		TMC5240_OP_WRITE(AMAX, &(cfg)->amax), TMC5240_OP_WRITE(DMAX, &(cfg)->dmax),        \
		TMC5240_OP_WRITE(THIGH, &(cfg)->thigh), TMC5240_OP_WRITE(VMAX, &(cfg)->vmax),      \
		TMC5240_OP_WRITE(TCOOLTHRS, &(cfg)->tcoolthrs),                                    \
		TMC5240_OP_WRITE(TZEROWAIT, &(cfg)->tzerowait),                                    \
		TMC5240_OP_WRITE(TPOWERDOWN, &(cfg)->tpowerdown),                                  \
		TMC5240_OP_WRITE(TPWMTHRS, &(cfg)->tpwmthrs),                                      \
		TMC5240_OP_WRITE(TVMAX, &(cfg)->tvmax)

#define TMC5240_MOTION_CONFIG_NON_POS_WRITES(cfg)                                                  \
	TMC5240_OP_WRITE(THIGH, &(cfg)->thigh), TMC5240_OP_WRITE(TCOOLTHRS, &(cfg)->tcoolthrs),    \
		TMC5240_OP_WRITE(TZEROWAIT, &(cfg)->tzerowait),                                    \
		TMC5240_OP_WRITE(TPOWERDOWN, &(cfg)->tpowerdown),                                  \
		TMC5240_OP_WRITE(TPWMTHRS, &(cfg)->tpwmthrs),                                      \
		TMC5240_OP_WRITE(TVMAX, &(cfg)->tvmax)

#define TMC5240_MOTION_CONFIG_POS_FORCE_WRITES(cfg)                                                \
	TMC5240_OP_WRITE_FORCED(V1, &(cfg)->v1), TMC5240_OP_WRITE_FORCED(V2, &(cfg)->v2),          \
		TMC5240_OP_WRITE_FORCED(A1, &(cfg)->a1), TMC5240_OP_WRITE_FORCED(A2, &(cfg)->a2),  \
		TMC5240_OP_WRITE_FORCED(D1, &(cfg)->d1), TMC5240_OP_WRITE_FORCED(D2, &(cfg)->d2),  \
		TMC5240_OP_WRITE_FORCED(AMAX, &(cfg)->amax),                                       \
		TMC5240_OP_WRITE_FORCED(DMAX, &(cfg)->dmax),                                       \
		TMC5240_OP_WRITE_FORCED(VMAX, &(cfg)->vmax)

int tmc5240_init(const struct device *dev);

/**
 * Check TMC5240 for errors.
 */
int tmc5240_check(const struct device *dev);

/**
 * Check motion config for known errors.
 */
int tmc5240_check_motion_config(const struct log_source_const_data *log,
				const struct tmc5240_motion_config *motion_config);

/**
 * Retrieve the lock handle from the device.
 *
 * Only use this lock to wrap one or more calls to `tmc5240_transact`.
 */
const struct k_spinlock *tmc5240_lock_handle(const struct device *dev);

/**
 * Perform multiple SPI operations in a pipelined fashion. Caller is responsible for enforcing
 * lock when calling this function (see `tmc5240_lock_handle`, `K_SPINLOCK`).
 *
 * This is an optimization around the way the TMC5240 SPI interface works -- it returns register
 * data a full transaction late. I.e., a standalone register read requires two SPI transactions
 * (the read request and then a dummy to actually retrieve the data). Hence, if multiple operations
 * need to be performed sequentially, we can substantially improve throughput (up to 2x).
 */
int tmc5240_transact(const struct device *dev, struct tmc5240_op *ops, size_t n);

int tmc5240_write_reg(const struct device *dev, uint8_t reg, uint32_t value, bool force);
int tmc5240_read_reg(const struct device *dev, uint8_t reg, uint32_t *value, bool force);

int tmc5240_mode_velocity(const struct device *dev, int32_t velocity_ustep_sec);
int tmc5240_mode_position(const struct device *dev, int32_t pos_ustep);
int tmc5240_mode_hold(const struct device *dev);

int tmc5240_set_motion_config(const struct device *dev, const struct tmc5240_motion_config *motion);

/**
 * Dump the current value of all registers. `dst` should have capacity for at least
 * `TMC5240_REG_MAX` entries; registers can be indexed in the result by address.
 */
int tmc5240_regdump(const struct device *dev, uint32_t *dst);

/**
 * Load all registers from `src. `src` should have capacity for at least `TMC5240_REG_MAX` entries,
 * though only the indexes of actual registers will be read.' This operation results in the active
 * `tmc5240_motion_config` being loaded from the incoming registers, including VMAX.
 *
 * - `volatile_` specifies whether volatile registers should be loaded, e.g. to reset
 * clear-on-write status registers.
 */
int tmc5240_regload(const struct device *dev, const uint32_t *src, bool volatile_);

static inline int tmc5240_zero_pos(const struct device *dev)
{
	TMC5240_WRITE_REG(dev, XACTUAL, 0);
	return 0;
}
#endif

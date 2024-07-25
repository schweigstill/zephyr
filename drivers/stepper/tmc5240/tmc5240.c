/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Nathan Perry <np@npry.dev>
 * SPDX-License-Identifier: MIT OR Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/stepper.h>

#include "tmc5240_internal.h"

LOG_LEVEL_SET(CONFIG_STEPPER_LOG_LEVEL);

#define MODE_WRITE   BIT(7)
#define MASK_NOTREAD (0b01111111)

inline const struct k_spinlock *tmc5240_lock_handle(const struct device *dev)
{
	struct tmc5240_data *data = dev->data;
	return &data->lock;
}

static inline uint32_t swap_uint32(uint32_t val)
{
	val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
	return (val << 16) | ((val >> 16) & 0xFFFF);
}

static inline int tmc5240_transact_validate(const struct log_source_const_data *log,
					    struct tmc5240_op *ops, size_t n)
{
	for (unsigned int i = 0; i < n; i++) {
		struct tmc5240_op *op = &ops[i];

		if (op->value == NULL) {
			LOG_INST_ERR(log, "tmc5240_transact: element %i had NULL data", i);
			return -EINVAL;
		}
	}

	return 0;
}

int tmc5240_transact(const struct device *dev, struct tmc5240_op *ops, size_t n)
{
	const struct tmc5240_config *dt_spec = dev->config;
	struct tmc5240_data *data = dev->data;

	if (n == 0) {
		return 0;
	}

	int ret = tmc5240_transact_validate(data->log, ops, n);
	if (ret < 0) {
		return ret;
	}

	// Bufs are split into [addr, data] (tx), [status, data] (rx)
	struct spi_buf tx_buf[2] = {{
					    .len = 1,
				    },
				    {
					    .len = 4,
				    }};

	struct spi_buf rx_buf[2] = {
		{
			.len = 1,
		},
		{
			.len = 4,
		},
	};

	struct spi_buf_set txs =
				   {
					   .count = 2,
					   .buffers = tx_buf,
				   },

			   rxs = {
				   .count = 2,
				   .buffers = rx_buf,
			   };

	// We need an extra read to get the value for the final op only if it is a read.
	unsigned int n_iterations = n + 1;

	// Store the previous op's addr (it will have been overridden by status by the time we need
	// to update the status byte).
	uint8_t previous_addr = 0;
	struct tmc5240_op *prev_op = NULL;

	for (unsigned int i = 0; i < n_iterations; i++) {
		uint32_t value = 0;

		uint8_t orig_addr = 0;
		uint8_t final_dummy = 0;
		uint8_t *addr = &final_dummy;

		if (i != n) {
			// If we're not the final read, update the address according to R/W mode and
			// set value to the correct endian swap of &op->value.

			struct tmc5240_op *op = &ops[i];

			op->address &= MASK_NOTREAD;
			orig_addr = op->address;

#ifdef CONFIG_TMC5240_STEPPER_SHADOW
			// If this operation isn't forced and register is nonvolatile, reference the
			// shadow register to determine whether a SPI op is necessary.

			const uint32_t *shadow_val = &data->shadow[orig_addr];
			bool volatile_ = tmc5240_reg_is_volatile(orig_addr);
			bool is_necessary_write =
				op->write && (volatile_ || *op->value != *shadow_val);
			bool is_necessary_read = !op->write && volatile_;
			bool spi_op_necessary =
				op->force || is_necessary_read || is_necessary_write;

			// If op not necessary, set status to 0 and update read with shadow reg
			// value, then bail.
			if (!spi_op_necessary) {
				if (!op->write) {
					*op->value = *shadow_val;
				}

				op->status = 0;

				continue;
			}
#endif

			if (op->write) {
				op->address |= MODE_WRITE;
			}
			addr = &op->address;

			value = swap_uint32(*op->value);
		} else {
			if (prev_op == NULL) {
				// we're the last op, but there were no spi ops before us -- don't
				// bother running
				break;
			}
		}

		tx_buf[0].buf = addr;
		rx_buf[0].buf = addr;

		tx_buf[1].buf = &value;

		// Receive data isn't stored in the target buffer directly because we need to
		// endian-swap it first.
		uint32_t rx_data;
		rx_buf[1].buf = &rx_data;

		ret = spi_transceive_dt(dt_spec->bus, &txs, &rxs);
		if (ret != 0) {
			return ret;
		}

#ifdef CONFIG_TMC5240_STEPPER_DETECT_UNRESPONSIVE
		// If we read all 0xff, then almost certainly the device isn't receiving
		// commands
		if (rx_data == 0xffffffff && *addr == 0xff) {
			LOG_INST_ERR(&data->log, "no response from device");
			return -EIO;
		}
#endif

		data->last_status = *addr;

		uint32_t swapped = swap_uint32(rx_data);

		// If we're not the first op, we have data for a previous operation: set the
		// previous op's &value to what we read this iteration, and update the corresponding
		// shadow register.
		//
		// Note that this operation doesn't make sense for registers with 'write 1 to clear'
		// or read-only semantics, but these are in the 'volatile' register set anyway and
		// so should not be visible from the shadow register set in the first place.
		if (prev_op != NULL) {
			if (!prev_op->write) {
				*prev_op->value = swapped;
			}

			data->shadow[previous_addr] = swapped;
		}

		previous_addr = orig_addr;
		prev_op = &ops[i];
	}

	return 0;
}

int tmc5240_write_reg(const struct device *dev, uint8_t reg, uint32_t value, bool force)
{
	struct tmc5240_data *data = dev->data;
	struct tmc5240_op op = {
		.address = reg,
		.write = true,
		.value = &value,
		.force = force,
	};

	int ret;

	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, &op, 1);
	}

	return ret;
}

int tmc5240_read_reg(const struct device *dev, uint8_t reg, uint32_t *value, bool force)
{
	struct tmc5240_data *data = dev->data;
	struct tmc5240_op op = {
		.write = false,
		.address = reg,
		.value = value,
		.force = force,
	};

	int ret;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, &op, 1);
	}

	return ret;
}

inline int tmc5240_mode_position(const struct device *dev, int32_t pos)
{
	struct tmc5240_data *data = dev->data;
	enum tmc5240_rampmode rampmode = TMC5240_RAMPMODE_POS;

	uint32_t ihold_irun = TMC5240_MOTION_CONFIG_IHOLD_IRUN(&data->motion_config);

	struct tmc5240_op ops[] = {
		TMC5240_OP_WRITE(RAMPMODE, &rampmode),
		TMC5240_OP_WRITE(IHOLD_IRUN, &ihold_irun),
		TMC5240_MOTION_CONFIG_NON_POS_WRITES(&data->motion_config),
		TMC5240_MOTION_CONFIG_POS_FORCE_WRITES(&data->motion_config),
		TMC5240_OP_WRITE_FORCED(XTARGET, &pos),
	};

	int ret;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, ops, ARRAY_SIZE(ops));
	}

	return ret;
}

inline int tmc5240_mode_velocity(const struct device *dev, int32_t vel)
{
	struct tmc5240_data *data = dev->data;

	bool sgn = vel > 0;
	uint32_t vel_ = vel > 0 ? vel : -vel;

	uint32_t ihold_irun = TMC5240_MOTION_CONFIG_IHOLD_IRUN(&data->motion_config);
	enum tmc5240_rampmode rampmode = sgn ? TMC5240_RAMPMODE_VEL : TMC5240_RAMPMODE_NVEL;

	uint32_t amax = 1000, dmax = 1000, chop_conf;

	int ret;
	K_SPINLOCK(&data->lock) {
		struct tmc5240_op ops[] = {
			TMC5240_MOTION_CONFIG_WRITES(&data->motion_config),
			TMC5240_OP_WRITE(IHOLD_IRUN, &ihold_irun),

			TMC5240_OP_WRITE(RAMPMODE, &rampmode),
			TMC5240_OP_WRITE(VMAX, &vel_),
		};

		ret = tmc5240_transact(dev, ops, ARRAY_SIZE(ops));
	}

	return ret;
}

inline int tmc5240_mode_hold(const struct device *dev)
{
	struct tmc5240_data *data = dev->data;

	uint32_t ihold_irun = TMC5240_MOTION_CONFIG_IHOLD_IRUN(&data->motion_config);
	enum tmc5240_rampmode rampmode = TMC5240_RAMPMODE_HOLD;

	struct tmc5240_op ops[] = {
		TMC5240_MOTION_CONFIG_WRITES(&data->motion_config),
		TMC5240_OP_WRITE(IHOLD_IRUN, &ihold_irun),
		TMC5240_OP_WRITE(RAMPMODE, &rampmode),
	};

	int ret;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, ops, ARRAY_SIZE(ops));
	}

	return ret;
}

int32_t tmc5240_check(const struct device *dev)
{
	struct tmc5240_data *data = dev->data;

	uint32_t drv_stat, ioin, ramp_stat;
	int ret = 0;

	struct tmc5240_op check_ops[] = {
		TMC5240_OP_READ(RAMP_STAT, &ramp_stat),
		TMC5240_OP_READ(DRV_STATUS, &drv_stat),
		TMC5240_OP_READ(IOIN, &ioin),
	};

	int status;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, check_ops, ARRAY_SIZE(check_ops));
		status = data->last_status;
	}

	if (ret < 0) {
		return ret;
	}

	if ((status & TMC5240_STATUS_DRV_ERR) != 0) {
		LOG_INST_ERR(&data->log, "motor driver error");
		ret = -EIO;
	}

	if (ramp_stat & (TMC5240_RAMP_STAT_EVT_STOP_L | TMC5240_RAMP_STAT_EVT_STOP_R |
			 TMC5240_RAMP_STAT_EVT_STOP_SG)) {
		LOG_INST_ERR(&data->log, "stalled or tripping limit switch");
		ret = -EIO;
	}

	if (ioin & TMC5240_IOIN_ADC_ERR) {
		LOG_INST_ERR(&data->log, "adc error");
		ret = -EIO;
	}

	if (ioin & TMC5240_IOIN_DRV_ENN) {
		LOG_INST_ERR(&data->log, "driver enable pin is high");
		ret = -EIO;
	}

	if (!(ioin & TMC5240_IOIN_EXT_RES_DET)) {
		LOG_INST_WRN(&data->log, "REF resistor missing");
	}

	if (drv_stat & (TMC5240_DRV_STATUS_OLA | TMC5240_DRV_STATUS_OLB)) {
		LOG_INST_ERR(&data->log, "open load indicated");
		ret = -EIO;
	}

	if (drv_stat & (TMC5240_DRV_STATUS_S2GA | TMC5240_DRV_STATUS_S2GB)) {
		LOG_INST_ERR(&data->log, "short to ground indicated");
		ret = -EIO;
	}

	if (drv_stat & TMC5240_DRV_STATUS_OTPW) {
		LOG_INST_WRN(&data->log, "overtemperature warning!");
	}

	if (drv_stat & TMC5240_DRV_STATUS_OT) {
		LOG_INST_ERR(&data->log, "overtemperature");
		ret = -EIO;
	}

	if (drv_stat & TMC5240_DRV_STATUS_SG) {
		LOG_INST_WRN(&data->log, "stallguard triggered");
	}

	if (drv_stat & (TMC5240_DRV_STATUS_S2VSA | TMC5240_DRV_STATUS_S2VSB)) {
		LOG_INST_ERR(&data->log, "short to supply indicated");
		ret = -EIO;
	}

	return ret;
}

int32_t tmc5240_check_motion_config(const struct log_source_const_data *log,
				    const struct tmc5240_motion_config *config)
{
	int ret = 0;

	if (config->vstart > config->vstop) {
		LOG_INST_ERR(log, "motion config: VSTART > VSTOP");
		ret = -EINVAL;
	}

	if (config->d1 == 0 || config->d2 == 0) {
		LOG_INST_ERR(log, "motion config: D1 or D2 is zeroed");
		ret = -EINVAL;
	}

	if (config->dmax == 0) {
		LOG_INST_ERR(log, "motion config: DMAX is zeroed");
		ret = -EINVAL;
	}

	if (config->v1 > 0 && config->a1 == 0) {
		LOG_INST_ERR(log, "motion config: V1 enabled but a1 == 0");
		ret = -EINVAL;
	}

	if (config->v2 > 0 && config->a2 == 0) {
		LOG_INST_ERR(log, "motion config: V1 enabled but a1 == 0");
		ret = -EINVAL;
	}

	if (config->ihold > 31 || config->irun > 31) {
		LOG_INST_ERR(log, "motion_config: IHOLD or IRUN > 31");
		ret = -EINVAL;
	}

	return ret;
}

int32_t tmc5240_write_motion_config(const struct device *dev)
{
	struct tmc5240_data *data = dev->data;

	uint32_t ihold_irun = TMC5240_MOTION_CONFIG_IHOLD_IRUN(&data->motion_config);

	struct tmc5240_op ops[] = {
		TMC5240_MOTION_CONFIG_WRITES(&data->motion_config),
		TMC5240_OP_WRITE(IHOLD_IRUN, &ihold_irun),
	};

	int ret;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, ops, ARRAY_SIZE(ops));
	}

	return ret;
}

int tmc5240_set_motion_config(const struct device *dev, const struct tmc5240_motion_config *motion)
{
	struct tmc5240_data *data = dev->data;

	int ret = tmc5240_check_motion_config(data->log, motion);
	if (ret < 0) {
		return ret;
	}

	data->motion_config = *motion;

	ret = tmc5240_write_motion_config(dev);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

int32_t tmc5240_load_motion_config_from_regs(const struct device *dev)
{
	struct tmc5240_data *data = dev->data;
	struct tmc5240_motion_config *motion = &data->motion_config;

	uint32_t ihold_irun;

	struct tmc5240_op ops[] = {
		TMC5240_OP_READ(A1, &motion->a1),
		TMC5240_OP_READ(A2, &motion->a2),
		TMC5240_OP_READ(D1, &motion->d1),
		TMC5240_OP_READ(D2, &motion->d2),
		TMC5240_OP_READ(AMAX, &motion->amax),
		TMC5240_OP_READ(DMAX, &motion->dmax),
		TMC5240_OP_READ(V1, &motion->v1),
		TMC5240_OP_READ(V2, &motion->v2),
		TMC5240_OP_READ(VMAX, &motion->vmax),
		TMC5240_OP_READ(THIGH, &motion->thigh),
		TMC5240_OP_READ(TPOWERDOWN, &motion->tpowerdown),
		TMC5240_OP_READ(TVMAX, &motion->tvmax),
		TMC5240_OP_READ(TCOOLTHRS, &motion->tcoolthrs),
		TMC5240_OP_READ(TPWMTHRS, &motion->tpwmthrs),
		TMC5240_OP_READ(TZEROWAIT, &motion->tzerowait),
		TMC5240_OP_READ(IHOLD_IRUN, &ihold_irun),
	};

	int ret;
	ret = tmc5240_transact(dev, ops, ARRAY_SIZE(ops));
	if (ret < 0) {
		return ret;
	}

	uint8_t ihold = ihold_irun & 0x1f, irun = (ihold_irun >> 8) & 0x1f,
		iholddelay = (ihold_irun >> 16) & 0xf, irundelay = (ihold_irun >> 24) & 0xf;

	motion->ihold = ihold;
	motion->irun = irun;
	motion->iholddelay = iholddelay;
	motion->irundelay = irundelay;

	return 0;
}

int32_t tmc5240_regdump(const struct device *dev, uint32_t *restrict dst)
{
	struct tmc5240_data *data = dev->data;
	struct tmc5240_op read_ops[TMC5240_NREG];

	for (int i = 0; i < TMC5240_NREG; i++) {
		uint8_t reg = TMC5240_ALL_REGS[i];

		struct tmc5240_op *op = &read_ops[i];
		op->address = reg;
		op->write = false;
		op->value = &dst[reg];
		op->force = false;
	}

	int ret;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, read_ops, ARRAY_SIZE(read_ops));
	}

	return ret;
}

int32_t tmc5240_regload(const struct device *dev, const uint32_t *restrict src, bool volatile_)
{
	struct tmc5240_data *data = dev->data;
	struct tmc5240_op write_ops[TMC5240_NREG];

	int ops_used = 0;
	for (int i = 0; i < TMC5240_NREG; i++) {
		uint8_t reg = TMC5240_ALL_REGS[i];

		if (tmc5240_reg_is_volatile(reg) && !volatile_) {
			continue;
		}

		struct tmc5240_op *op = &write_ops[ops_used];

		op->address = reg;
		op->write = true;
		op->value = &src[reg];
		op->force = false;

		ops_used++;
	}

	int ret;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, write_ops, ops_used);
		if (ret < 0) {
			K_SPINLOCK_BREAK;
		}

		ret = tmc5240_load_motion_config_from_regs(dev);
	}

	return ret;
}

int32_t tmc5240_init(const struct device *dev)
{
	const struct tmc5240_config *config = dev->config;
	struct tmc5240_data *data = dev->data;
	data->dev = dev;

	LOG_INST_INF(&data->log, "initializing %s", dev->name);

	if (!spi_is_ready_dt(config->bus)) {
		LOG_INST_ERR(&data->log, "SPI device is not ready");
		return -ENODEV;
	}

	struct tmc5240_op read_ops[TMC5240_NREG];

	uint32_t dummy;
	for (int i = 0; i < TMC5240_NREG; i++) {
		uint8_t reg = TMC5240_ALL_REGS[i];

		read_ops[i].address = reg;
		read_ops[i].write = false;
		read_ops[i].value = &dummy;
		read_ops[i].force = true;
	}

	uint32_t nreg_reset = 0x1f, gconf = TMC5240_GCONF_DEFAULT, rampmode = TMC5240_RAMPMODE_HOLD,
		 drv_conf = 0, sw_mode = TMC5240_SW_MODE_DEFAULT,
		 ramp_stat = TMC5240_RAMP_STAT_CLEAR, global_scaler = 0,
		 chopconf = TMC5240_CHOPCONF_DEFAULT;

	struct tmc5240_op write_defaults[] = {
		TMC5240_OP_WRITE(GSTAT, &nreg_reset),
		TMC5240_OP_WRITE(GCONF, &gconf),
		TMC5240_OP_WRITE(SW_MODE, &sw_mode),
		TMC5240_OP_WRITE(RAMP_STAT, &ramp_stat),
		TMC5240_OP_WRITE(DRV_CONF, &drv_conf),
		TMC5240_OP_WRITE(RAMPMODE, &rampmode),
		TMC5240_OP_WRITE(GLOBAL_SCALER, &global_scaler),
		TMC5240_OP_WRITE(CHOPCONF, &chopconf),
	};

	int ret;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, read_ops, ARRAY_SIZE(read_ops));
		if (ret < 0) {
			K_SPINLOCK_BREAK;
		}

		ret = tmc5240_transact(dev, write_defaults, ARRAY_SIZE(write_defaults));
	}

	if (ret < 0) {
		LOG_INST_ERR(&data->log, "reading initial value of registers");
		return ret;
	}

	struct tmc5240_motion_config default_motion_config = TMC5240_MOTION_CONFIG_DEFAULT;

	ret = tmc5240_set_motion_config(dev, &default_motion_config);
	if (ret < 0) {
		LOG_INST_ERR(&data->log, "setting initial motion config");
		return ret;
	}

	ret = stepper_enable(dev, false);
	if (ret < 0) {
		LOG_INST_ERR(&data->log, "disabling stepper at init");
		return ret;
	}

	LOG_INST_INF(&data->log, "initialized");

	return 0;
}

/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Nathan Perry <np@npry.dev>
 * SPDX-License-Identifier: MIT OR Apache-2.0
 */

#define DT_DRV_COMPAT adi_tmc5240

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/stepper/tmc5240.h>

#include "tmc5240_internal.h"

LOG_LEVEL_SET(CONFIG_STEPPER_LOG_LEVEL);

static inline int32_t tmc5240_stepperapi_rel_move(const struct device *dev, int32_t micro_steps)
{
	struct tmc5240_data *data = dev->data;
	int rampmode = TMC5240_RAMPMODE_POS;
	int32_t curpos;
	uint32_t ihold_irun = TMC5240_MOTION_CONFIG_IHOLD_IRUN(&data->motion_config);

	struct tmc5240_op ops[] = {
		TMC5240_OP_WRITE(RAMPMODE, &rampmode),
		TMC5240_OP_WRITE(IHOLD_IRUN, &ihold_irun),
		TMC5240_MOTION_CONFIG_NON_POS_WRITES(&data->motion_config),
		TMC5240_MOTION_CONFIG_POS_FORCE_WRITES(&data->motion_config),
		TMC5240_OP_READ(XACTUAL, &curpos),
	};

	int ret;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, ops, ARRAY_SIZE(ops));
		if (ret < 0) {
			K_SPINLOCK_BREAK;
		}

		curpos += micro_steps;
		struct tmc5240_op force_xtarget = TMC5240_OP_WRITE_FORCED(XTARGET, &curpos);

		ret = tmc5240_transact(dev, &force_xtarget, 1);
	}

	return ret;
}

static inline int32_t tmc5240_stepperapi_set_actual_position(const struct device *dev,
							     int32_t position)
{
	TMC5240_WRITE_REG(dev, XACTUAL, position);
	return 0;
}

static inline int32_t tmc5240_stepperapi_get_actual_position(const struct device *dev,
							     int32_t *position)
{
	return tmc5240_read_reg(dev, TMC5240_REG_XACTUAL, position, false);
}

static int32_t tmc5240_stepperapi_set_target_position(const struct device *dev, int32_t position)
{
	struct tmc5240_data *data = dev->data;
	int rampmode = TMC5240_RAMPMODE_POS;

	struct tmc5240_op ops[] = {
		TMC5240_OP_WRITE(RAMPMODE, &rampmode),
		TMC5240_OP_WRITE(XTARGET, &position),
	};

	int ret;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, ops, ARRAY_SIZE(ops));
	}

	return ret;
}

static int32_t tmc5240_stepperapi_is_moving(const struct device *dev, bool *is_moving)
{
	struct tmc5240_data *data = dev->data;

	uint32_t dummy;
	struct tmc5240_op dummy_for_status = {
		.address = 0,
		.write = false,
		.value = &dummy,
	};

	int ret;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, &dummy_for_status, 1);
	}

	*is_moving = (dummy_for_status.status & TMC5240_STATUS_STANDSTILL) != 0;
	return ret;
}

static inline int32_t tmc5240_stepperapi_set_max_velocity(const struct device *dev,
							  uint32_t velocity)
{
	struct tmc5240_data *data = dev->data;
	struct tmc5240_op op = TMC5240_OP_WRITE(VMAX, &velocity);

	int ret;
	K_SPINLOCK(&data->lock) {
		data->motion_config.vmax = (int32_t)velocity;
		ret = tmc5240_transact(dev, &op, 1);
	}

	return ret;
}

static int32_t tmc5240_stepperapi_get_micro_step_res(const struct device *dev,
						     enum micro_step_resolution *micro_step_res)
{
	ARG_UNUSED(dev);
	*micro_step_res = STEPPER_MICRO_STEP_256;
	return 0;
}

int32_t tmc5240_stepperapi_enable(const struct device *dev, bool enable)
{
	struct tmc5240_data *data = dev->data;

	uint32_t chop_conf;
	struct tmc5240_op read_op = TMC5240_OP_READ(CHOPCONF, &chop_conf);

	int ret;
	K_SPINLOCK(&data->lock) {
		ret = tmc5240_transact(dev, &read_op, 1);
		if (ret < 0) {
			K_SPINLOCK_BREAK;
		}

		chop_conf &= ~TMC5240_CHOPCONF_DRVEN_MASK;

		if (enable) {
			chop_conf |= 0b0010;
		}

		struct tmc5240_op write = TMC5240_OP_WRITE(CHOPCONF, &chop_conf);
		ret = tmc5240_transact(dev, &write, 1);
	}

	return ret;
}

int32_t tmc5240_stepperapi_constant_velocity_mode(const struct device *dev,
						  enum stepper_direction direction,
						  uint32_t velocity)
{
	int32_t vel = (int32_t)(velocity & 0x7fffffff);
	if (direction == STEPPER_DIRECTION_NEGATIVE) {
		vel = -vel;
	}

	return tmc5240_mode_velocity(dev, vel);
}

static const struct stepper_api tmc5240_stepper_api = {
	.enable = tmc5240_stepperapi_enable,

	.move = tmc5240_stepperapi_rel_move,
	.is_moving = tmc5240_stepperapi_is_moving,
	.set_actual_position = tmc5240_stepperapi_set_actual_position,
	.get_actual_position = tmc5240_stepperapi_get_actual_position,
	.set_target_position = tmc5240_stepperapi_set_target_position,
	.set_max_velocity = tmc5240_stepperapi_set_max_velocity,
	.get_micro_step_res = tmc5240_stepperapi_get_micro_step_res,

	.enable_constant_velocity_mode = tmc5240_stepperapi_constant_velocity_mode,
};

#define DECLARE_TMC5240_INST(inst)                                                                 \
	LOG_INSTANCE_REGISTER(stepper_tmc5240, inst, CONFIG_STEPPER_LOG_LEVEL);                    \
	static const struct spi_dt_spec tmc5240_spi_##inst = SPI_DT_SPEC_INST_GET(                 \
		inst, SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_MODE_CPOL | SPI_WORD_SET(8), 0);    \
	static struct tmc5240_data tmc5240_data_##inst = {                                         \
		LOG_INSTANCE_PTR_INIT(log, stepper_tmc5240, inst).motion_config =                  \
			TMC5240_MOTION_CONFIG_DEFAULT,                                             \
	};                                                                                         \
	static const struct tmc5240_config tmc5240_spec_##inst = {.bus = &tmc5240_spi_##inst};     \
	DEVICE_DT_INST_DEFINE(inst, tmc5240_init, NULL, &tmc5240_data_##inst,                      \
			      &tmc5240_spec_##inst, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, \
			      &tmc5240_stepper_api);

DT_INST_FOREACH_STATUS_OKAY(DECLARE_TMC5240_INST)

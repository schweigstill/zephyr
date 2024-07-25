/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Nathan Perry <np@npry.dev>
 * SPDX-License-Identifier: MIT OR Apache-2.0
 */

#ifndef TMC5240_INTERNAL_H
#define TMC5240_INTERNAL_H

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/stepper/tmc5240.h>

struct tmc5240_config {
	const struct spi_dt_spec *bus;
};

struct tmc5240_data {
	const struct device *dev;
	struct k_spinlock lock;

	LOG_INSTANCE_PTR_DECLARE(log);

	struct tmc5240_motion_config motion_config;

	uint32_t shadow[TMC5240_REG_RANGE];
	uint8_t last_status;
};

int32_t tmc5240_write_motion_config(const struct device *dev);

/**
 * Load device's motion_config from values in registers.
 *
 * !!! Expects to be called within a critical section.
 */
int32_t tmc5240_load_motion_config_from_regs(const struct device *dev);
#endif

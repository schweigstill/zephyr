/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC524X_H_
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC524X_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/stepper/stepper_tmc524x.h>
#include "tmc524x_reg.h"

struct tmc524x_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec sleep_gpio;
	uint32_t clock_frequency;
};

struct tmc524x_data {
	struct k_mutex lock;
	struct k_mutex callback_dispatch_lock;
	struct k_spinlock callback_dispatch_state_lock;
	k_tid_t callback_dispatch_thread;
	uint32_t callback_dispatch_depth;
	atomic_t diag0_pending;
	uint8_t last_spi_status;
};

static inline bool tmc524x_callback_dispatch_is_current(const struct device *dev)
{
	struct tmc524x_data *data = dev->data;
	k_spinlock_key_t key;
	bool current;

	key = k_spin_lock(&data->callback_dispatch_state_lock);
	current = data->callback_dispatch_thread == k_current_get();
	k_spin_unlock(&data->callback_dispatch_state_lock, key);

	return current;
}

/* Serialize callbacks from both TMC524X child devices through their common
 * parent. Same-thread nesting remains legal so a callback may use either API.
 */
static inline bool tmc524x_callback_dispatch_enter(const struct device *dev)
{
	struct tmc524x_data *data = dev->data;
	k_spinlock_key_t key;
	bool nested;

	key = k_spin_lock(&data->callback_dispatch_state_lock);
	nested = data->callback_dispatch_thread == k_current_get();
	if (nested) {
		data->callback_dispatch_depth++;
	}
	k_spin_unlock(&data->callback_dispatch_state_lock, key);
	if (nested) {
		return false;
	}

	k_mutex_lock(&data->callback_dispatch_lock, K_FOREVER);
	key = k_spin_lock(&data->callback_dispatch_state_lock);
	data->callback_dispatch_thread = k_current_get();
	data->callback_dispatch_depth = 1U;
	k_spin_unlock(&data->callback_dispatch_state_lock, key);

	return true;
}

static inline void tmc524x_callback_dispatch_exit(const struct device *dev, bool root)
{
	struct tmc524x_data *data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->callback_dispatch_state_lock);
	__ASSERT_NO_MSG(data->callback_dispatch_thread == k_current_get());
	__ASSERT_NO_MSG(data->callback_dispatch_depth > 0U);
	data->callback_dispatch_depth--;
	if (data->callback_dispatch_depth == 0U) {
		data->callback_dispatch_thread = NULL;
	}
	k_spin_unlock(&data->callback_dispatch_state_lock, key);
	if (root) {
		k_mutex_unlock(&data->callback_dispatch_lock);
	}
}

static inline void tmc524x_callback_dispatch_lock(const struct device *dev)
{
	struct tmc524x_data *data = dev->data;

	k_mutex_lock(&data->callback_dispatch_lock, K_FOREVER);
}

static inline void tmc524x_callback_dispatch_unlock(const struct device *dev)
{
	struct tmc524x_data *data = dev->data;

	k_mutex_unlock(&data->callback_dispatch_lock);
}

static inline void tmc524x_diag0_mark_pending(const struct device *dev)
{
	struct tmc524x_data *data = dev->data;

	atomic_set(&data->diag0_pending, 1);
}

static inline bool tmc524x_diag0_is_pending(const struct device *dev)
{
	struct tmc524x_data *data = dev->data;

	return atomic_get(&data->diag0_pending) != 0;
}

static inline void tmc524x_diag0_clear_pending(const struct device *dev)
{
	struct tmc524x_data *data = dev->data;

	atomic_clear(&data->diag0_pending);
}

static inline int32_t tmc524x_sign_extend(uint32_t value, uint8_t bits)
{
	uint32_t sign = BIT(bits - 1U);
	return (int32_t)((value ^ sign) - sign);
}

static inline uint32_t tmc524x_sgt_to_field(int8_t sgt)
{
	return (uint32_t)sgt & 0x7fU;
}

static inline uint8_t tmc524x_microstep_to_mres(uint16_t microsteps)
{
	/* CHOPCONF.MRES: 0 => 256, 1 => 128, ..., 8 => 1 */
	return 8U - LOG2(microsteps);
}

static inline uint16_t tmc524x_mres_to_microstep(uint8_t mres)
{
	return (uint16_t)BIT(8U - mres);
}

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC524X_H_ */

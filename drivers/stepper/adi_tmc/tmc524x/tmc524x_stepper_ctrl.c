/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT adi_tmc524x_stepper_ctrl

#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "tmc524x.h"

LOG_MODULE_DECLARE(tmc524x, CONFIG_STEPPER_LOG_LEVEL);

struct tmc524x_stepper_ctrl_config {
	const struct device *controller;
	uint32_t vstart;
	uint32_t a1;
	uint32_t v1;
	uint32_t amax;
	uint32_t vmax;
	uint32_t dmax;
	uint32_t tvmax;
	uint32_t d1;
	uint32_t vstop;
	uint32_t tzerowait;
	uint32_t v2;
	uint32_t a2;
	uint32_t d2;
	uint64_t default_microstep_interval_ns;
};

enum tmc524x_stepper_ctrl_poll_mode {
	TMC524X_CTRL_POLL_INACTIVE,
	TMC524X_CTRL_POLL_POSITION,
	TMC524X_CTRL_POLL_VELOCITY,
	TMC524X_CTRL_POLL_STOPPING,
};

struct tmc524x_stepper_ctrl_data {
	struct k_mutex lock;
	struct k_spinlock callback_lock;
	stepper_ctrl_event_callback_t event_cb;
	void *event_user_data;
	uint32_t velocity_reg;
	struct k_work_delayable stop_work;
	bool stop_pending;
	const struct device *dev;
#if defined(CONFIG_STEPPER_ADI_TMC524X_CTRL_POLLING)
	struct k_work_delayable poll_work;
	enum tmc524x_stepper_ctrl_poll_mode poll_mode;
	uint32_t reported_stop_events;
#endif
};

static uint32_t tmc524x_hz_to_vreg(const struct device *controller, uint32_t microsteps_per_second)
{
	uint32_t fclk = tmc524x_get_clock_frequency(controller);
	uint64_t reg = ((uint64_t)microsteps_per_second << 24) / fclk;

	return MIN(reg, 0x7fffffU);
}

static uint32_t tmc524x_u32_lshift_div_u64_sat(uint32_t value, uint8_t shift,
					       uint64_t denom, uint32_t max)
{
	uint64_t rem = 0U;
	uint32_t quot = 0U;
	int bit;

	if (denom == 0U) {
		return max;
	}

	/*
	 * Calculate (value << shift) / denom without using __int128.
	 * Some Zephyr embedded targets/toolchains do not support 128-bit integers.
	 * The loop below is ordinary binary long division.  The remainder stays
	 * below 2 * denom; with the TMC524X fCLK range denom is well within u64.
	 */
	for (bit = 31 + shift; bit >= 0; bit--) {
		rem <<= 1;
		if (bit >= shift) {
			rem |= (value >> (bit - shift)) & 1U;
		}

		if (rem >= denom) {
			uint64_t bit_value;

			rem -= denom;
			if (bit >= 32) {
				return max;
			}

			bit_value = 1ULL << bit;
			if ((bit_value > max) || (quot > (max - (uint32_t)bit_value))) {
				return max;
			}

			quot |= (uint32_t)bit_value;
		}
	}

	return quot;
}

static uint32_t tmc524x_accel_to_areg(const struct device *controller, uint32_t microsteps_per_second2)
{
	uint32_t fclk = tmc524x_get_clock_frequency(controller);
	uint64_t denom = (uint64_t)fclk * (uint64_t)fclk;

	return tmc524x_u32_lshift_div_u64_sat(microsteps_per_second2, 42, denom, 0x3ffffU);
}

static int32_t tmc524x_read_s24(const struct device *controller, uint8_t reg, int32_t *value)
{
	uint32_t raw;
	int ret;

	ret = tmc524x_reg_read(controller, reg, &raw);
	if (ret != 0) {
		return ret;
	}
	*value = tmc524x_sign_extend(raw & 0xffffffU, 24);
	return 0;
}

static void tmc524x_stepper_ctrl_fire(const struct device *dev, enum stepper_ctrl_event event)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	stepper_ctrl_event_callback_t callback;
	void *user_data;
	k_spinlock_key_t key;
	bool dispatch_root;

	dispatch_root = tmc524x_callback_dispatch_enter(cfg->controller);
	key = k_spin_lock(&data->callback_lock);
	callback = data->event_cb;
	user_data = data->event_user_data;
	k_spin_unlock(&data->callback_lock, key);
	if (callback != NULL) {
		callback(dev, event, user_data);
	}
	tmc524x_callback_dispatch_exit(cfg->controller, dispatch_root);
}

static void tmc524x_stepper_ctrl_stop_work(struct k_work *work)
{
	struct tmc524x_stepper_ctrl_data *data = CONTAINER_OF(
		work, struct tmc524x_stepper_ctrl_data, stop_work.work);
	const struct device *dev = data->dev;
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	bool fire_stopped = false;
	int32_t vactual;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->stop_pending) {
		k_mutex_unlock(&data->lock);
		return;
	}
	ret = tmc524x_read_s24(cfg->controller, TMC524X_REG_VACTUAL, &vactual);
	if (ret == 0 && vactual == 0) {
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPMODE,
					 TMC524X_RAMPMODE_HOLD);
		fire_stopped = ret == 0;
		if (fire_stopped) {
			data->stop_pending = false;
		}
	}
	if (!fire_stopped) {
		k_work_reschedule(&data->stop_work,
				  K_MSEC(CONFIG_STEPPER_ADI_TMC524X_CTRL_POLL_INTERVAL_MS));
	}
	k_mutex_unlock(&data->lock);

	if (fire_stopped) {
		tmc524x_stepper_ctrl_fire(dev, STEPPER_CTRL_EVENT_STOPPED);
	}
}

static void tmc524x_stepper_ctrl_cancel_stop_locked(const struct device *dev)
{
	struct tmc524x_stepper_ctrl_data *data = dev->data;

	data->stop_pending = false;
	(void)k_work_cancel_delayable(&data->stop_work);
}

#if defined(CONFIG_STEPPER_ADI_TMC524X_CTRL_POLLING)
static void tmc524x_stepper_ctrl_poll(struct k_work *work)
{
	struct tmc524x_stepper_ctrl_data *data = CONTAINER_OF(work,
		struct tmc524x_stepper_ctrl_data, poll_work.work);
	const struct device *dev = data->dev;
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	uint32_t rampstat;
	uint32_t stop_events;
	uint32_t new_stop_events;
	enum tmc524x_stepper_ctrl_poll_mode poll_mode;
	bool fire_left = false;
	bool fire_right = false;
	bool fire_steps = false;
	bool fire_stopped = false;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	poll_mode = data->poll_mode;
	if (poll_mode == TMC524X_CTRL_POLL_INACTIVE) {
		goto unlock;
	}
	ret = tmc524x_get_ramp_status(cfg->controller, &rampstat);
	if (ret != 0) {
		goto reschedule;
	}

	if ((rampstat & TMC524X_RAMPSTAT_EVENT_STOP_SG) != 0U) {
		if (tmc524x_diag0_is_pending(cfg->controller)) {
			goto reschedule;
		}
		/* Clear event_stop_sg only after HOLD prevents an unintended restart. */
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPMODE,
				       TMC524X_RAMPMODE_HOLD);
		if (ret != 0) {
			goto reschedule;
		}
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPSTAT,
				       TMC524X_RAMPSTAT_EVENT_STOP_SG);
		if (ret != 0) {
			goto reschedule;
		}
		data->poll_mode = TMC524X_CTRL_POLL_INACTIVE;
		fire_stopped = true;
		goto unlock;
	}

	stop_events = rampstat & (TMC524X_RAMPSTAT_EVENT_STOP_L |
				  TMC524X_RAMPSTAT_EVENT_STOP_R);
	new_stop_events = stop_events & ~data->reported_stop_events;
	data->reported_stop_events |= new_stop_events;
	fire_left = (new_stop_events & TMC524X_RAMPSTAT_EVENT_STOP_L) != 0U;
	fire_right = (new_stop_events & TMC524X_RAMPSTAT_EVENT_STOP_R) != 0U;

	if ((rampstat & TMC524X_RAMPSTAT_EVENT_POS_REACHED) != 0U &&
	    data->reported_stop_events == 0U && poll_mode == TMC524X_CTRL_POLL_POSITION) {
		if (tmc524x_diag0_is_pending(cfg->controller)) {
			goto reschedule;
		}
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPSTAT,
				       TMC524X_RAMPSTAT_EVENT_POS_REACHED);
		if (ret != 0) {
			goto reschedule;
		}
		data->poll_mode = TMC524X_CTRL_POLL_INACTIVE;
		fire_steps = true;
		goto unlock;
	}

	if ((rampstat & TMC524X_RAMPSTAT_VZERO) != 0U &&
	    (poll_mode == TMC524X_CTRL_POLL_STOPPING ||
	     data->reported_stop_events != 0U)) {
		if (tmc524x_diag0_is_pending(cfg->controller)) {
			goto reschedule;
		}
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPMODE,
				       TMC524X_RAMPMODE_HOLD);
		if (ret != 0) {
			goto reschedule;
		}
		data->poll_mode = TMC524X_CTRL_POLL_INACTIVE;
		fire_stopped = true;
		goto unlock;
	}

reschedule:
	k_work_reschedule(&data->poll_work,
			  K_MSEC(CONFIG_STEPPER_ADI_TMC524X_CTRL_POLL_INTERVAL_MS));

unlock:
	k_mutex_unlock(&data->lock);
	if (fire_left) {
		tmc524x_stepper_ctrl_fire(dev, STEPPER_CTRL_EVENT_LEFT_END_STOP_DETECTED);
	}
	if (fire_right) {
		tmc524x_stepper_ctrl_fire(dev, STEPPER_CTRL_EVENT_RIGHT_END_STOP_DETECTED);
	}
	if (fire_steps) {
		tmc524x_stepper_ctrl_fire(dev, STEPPER_CTRL_EVENT_STEPS_COMPLETED);
	}
	if (fire_stopped) {
		tmc524x_stepper_ctrl_fire(dev, STEPPER_CTRL_EVENT_STOPPED);
	}
}

static void tmc524x_stepper_ctrl_start_poll_locked(
	const struct device *dev, enum tmc524x_stepper_ctrl_poll_mode poll_mode)
{
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	data->reported_stop_events = 0U;
	data->poll_mode = poll_mode;
	k_work_reschedule(&data->poll_work,
			  K_MSEC(CONFIG_STEPPER_ADI_TMC524X_CTRL_POLL_INTERVAL_MS));
}

static void tmc524x_stepper_ctrl_cancel_poll_locked(const struct device *dev)
{
	struct tmc524x_stepper_ctrl_data *data = dev->data;

	data->poll_mode = TMC524X_CTRL_POLL_INACTIVE;
	(void)k_work_cancel_delayable(&data->poll_work);
}
#else
static void tmc524x_stepper_ctrl_start_poll_locked(
	const struct device *dev, enum tmc524x_stepper_ctrl_poll_mode poll_mode)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(poll_mode);
}

static void tmc524x_stepper_ctrl_cancel_poll_locked(const struct device *dev)
{
	ARG_UNUSED(dev);
}
#endif

static int tmc524x_stepper_ctrl_apply_default_ramp(const struct device *dev)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	int32_t vactual;
	int ret;

	/* Freeze an inherited warm-start ramp before changing any of its parameters.
	 * HOLD preserves a nonzero velocity, so refuse initialization in that state;
	 * the hardware-driver sibling normally disabled TOFF before this point.
	 */
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPMODE,
				 TMC524X_RAMPMODE_HOLD);
	if (ret != 0) { return ret; }
	ret = tmc524x_read_s24(cfg->controller, TMC524X_REG_VACTUAL, &vactual);
	if (ret != 0) { return ret; }
	if (vactual != 0) { return -EBUSY; }

	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_VSTART, cfg->vstart);
	if (ret != 0) { return ret; }
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_A1, cfg->a1);
	if (ret != 0) { return ret; }
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_V1, cfg->v1);
	if (ret != 0) { return ret; }
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_AMAX, cfg->amax);
	if (ret != 0) { return ret; }
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_VMAX, cfg->vmax);
	if (ret != 0) { return ret; }
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_DMAX, cfg->dmax);
	if (ret != 0) { return ret; }
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_TVMAX, cfg->tvmax);
	if (ret != 0) { return ret; }
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_D1, cfg->d1);
	if (ret != 0) { return ret; }
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_VSTOP, cfg->vstop);
	if (ret != 0) { return ret; }
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_TZEROWAIT, cfg->tzerowait);
	if (ret != 0) { return ret; }
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_V2, cfg->v2);
	if (ret != 0) { return ret; }
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_A2, cfg->a2);
	if (ret != 0) { return ret; }
	return tmc524x_reg_write(cfg->controller, TMC524X_REG_D2, cfg->d2);
}

static int tmc524x_stepper_ctrl_set_reference_position(const struct device *dev,
						       const int32_t value)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	int32_t vactual;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = tmc524x_read_s24(cfg->controller, TMC524X_REG_VACTUAL, &vactual);
	if (ret == 0 && vactual != 0) {
		ret = -EBUSY;
	}
	if (ret == 0) {
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPMODE,
					 TMC524X_RAMPMODE_HOLD);
	}
	if (ret == 0) {
		tmc524x_stepper_ctrl_cancel_poll_locked(dev);
		tmc524x_stepper_ctrl_cancel_stop_locked(dev);
	}
	if (ret == 0) {
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_XACTUAL,
					 (uint32_t)value);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int tmc524x_stepper_ctrl_get_actual_position(const struct device *dev, int32_t *value)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	uint32_t raw;
	int ret;

	if (value == NULL) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = tmc524x_reg_read(cfg->controller, TMC524X_REG_XACTUAL, &raw);
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		return ret;
	}
	*value = (int32_t)raw;
	return 0;
}

static int tmc524x_stepper_ctrl_set_event_cb(const struct device *dev,
					     stepper_ctrl_event_callback_t cb, void *user_data)
{
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	k_spinlock_key_t key;
	bool called_from_callback;

	called_from_callback = tmc524x_callback_dispatch_is_current(cfg->controller);
	/* Once unregister returns, an old callback/user_data pair is no longer in use.
	 * A callback may still unregister itself without deadlocking.
	 */
	if (!called_from_callback) {
		tmc524x_callback_dispatch_lock(cfg->controller);
	}
	key = k_spin_lock(&data->callback_lock);
	data->event_user_data = user_data;
	data->event_cb = cb;
	k_spin_unlock(&data->callback_lock, key);
	if (!called_from_callback) {
		tmc524x_callback_dispatch_unlock(cfg->controller);
	}

	return 0;
}

static int tmc524x_stepper_ctrl_set_microstep_interval(const struct device *dev,
						       const uint64_t microstep_interval_ns)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	uint32_t speed;
	uint32_t velocity_reg;
	int ret;

	if (microstep_interval_ns == 0U) {
		return -EINVAL;
	}

	speed = (uint32_t)(1000000000ULL / microstep_interval_ns);
	if (speed == 0U) {
		return -ERANGE;
	}
	velocity_reg = tmc524x_hz_to_vreg(cfg->controller, speed);
	if (velocity_reg == 0U) {
		return -ERANGE;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->stop_pending) {
		ret = -EBUSY;
	} else {
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_VMAX, velocity_reg);
	}
	if (ret == 0) {
		data->velocity_reg = velocity_reg;
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int tmc524x_stepper_ctrl_configure_ramp(const struct device *dev,
					      const struct stepper_ctrl_ramp *ramp)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	uint32_t amax;
	uint32_t vmax;
	uint32_t dmax;
	int ret;

	if (ramp == NULL || ramp->speed_max == 0U || ramp->acceleration_max == 0U ||
	    ramp->deceleration_max == 0U) {
		return -EINVAL;
	}

	amax = tmc524x_accel_to_areg(cfg->controller, ramp->acceleration_max);
	vmax = tmc524x_hz_to_vreg(cfg->controller, ramp->speed_max);
	dmax = tmc524x_accel_to_areg(cfg->controller, ramp->deceleration_max);
	if (amax == 0U || vmax == 0U || dmax == 0U) {
		return -ERANGE;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->stop_pending) {
		k_mutex_unlock(&data->lock);
		return -EBUSY;
	}
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_AMAX, amax);
	if (ret == 0) {
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_VMAX, vmax);
	}
	if (ret == 0) {
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_DMAX, dmax);
	}
	if (ret == 0) {
		data->velocity_reg = vmax;
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int tmc524x_stepper_ctrl_prepare_motion_locked(const struct device *dev,
						      int32_t vactual)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	uint32_t rampstat;
	int ret;

	ret = tmc524x_get_ramp_status(cfg->controller, &rampstat);
	if (ret != 0) {
		return ret;
	}

	/* Clearing event_stop_sg releases the stopped ramp. Do it only after a
	 * stationary controller has been put in HOLD. When the event was not set in
	 * the snapshot, clear only event_pos_reached; a concurrently arriving stall
	 * therefore remains latched and cannot restart the old movement.
	 */
	if ((rampstat & TMC524X_RAMPSTAT_EVENT_STOP_SG) != 0U && vactual != 0) {
		return -EBUSY;
	}
	if (vactual == 0) {
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPMODE,
					 TMC524X_RAMPMODE_HOLD);
		if (ret != 0) {
			return ret;
		}
	}
	if ((rampstat & TMC524X_RAMPSTAT_EVENT_STOP_SG) != 0U) {
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPSTAT,
					 TMC524X_RAMPSTAT_EVENT_STOP_SG);
		if (ret != 0) {
			return ret;
		}
	}

	return tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPSTAT,
				  TMC524X_RAMPSTAT_EVENT_POS_REACHED);
}

static int tmc524x_stepper_ctrl_move_to_locked(const struct device *dev,
					       const int32_t micro_steps, bool *completed)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	uint32_t xactual;
	int32_t vactual;
	int ret;

	*completed = false;
	if (data->stop_pending) {
		return -EBUSY;
	}
	if (data->velocity_reg == 0U) {
		data->velocity_reg = cfg->vmax;
	}
	if (data->velocity_reg == 0U) {
		return -EINVAL;
	}

	ret = tmc524x_reg_read(cfg->controller, TMC524X_REG_XACTUAL, &xactual);
	if (ret != 0) {
		return ret;
	}
	ret = tmc524x_read_s24(cfg->controller, TMC524X_REG_VACTUAL, &vactual);
	if (ret != 0) {
		return ret;
	}
	/* Until the final mode write succeeds, retain the previous poll so a failed
	 * re-command never becomes unmonitored.
	 */
	ret = tmc524x_stepper_ctrl_prepare_motion_locked(dev, vactual);
	if (ret != 0) {
		return ret;
	}
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_VMAX, data->velocity_reg);
	if (ret != 0) {
		return ret;
	}
	/* Program the target before POSITION so an old XTARGET cannot cause a short
	 * unintended move between two SPI datagrams.
	 */
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_XTARGET, (uint32_t)micro_steps);
	if (ret != 0) {
		return ret;
	}
	if (vactual == 0 && (int32_t)xactual == micro_steps) {
		tmc524x_stepper_ctrl_cancel_poll_locked(dev);
		tmc524x_stepper_ctrl_cancel_stop_locked(dev);
		*completed = true;
		return 0;
	}

	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPMODE,
				 TMC524X_RAMPMODE_POSITION);
	if (ret == 0) {
		tmc524x_stepper_ctrl_cancel_stop_locked(dev);
		tmc524x_stepper_ctrl_start_poll_locked(dev, TMC524X_CTRL_POLL_POSITION);
	}

	return ret;
}

static int tmc524x_stepper_ctrl_move_to(const struct device *dev, const int32_t micro_steps)
{
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	bool completed;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = tmc524x_stepper_ctrl_move_to_locked(dev, micro_steps, &completed);
	k_mutex_unlock(&data->lock);
	if (ret == 0 && completed) {
		tmc524x_stepper_ctrl_fire(dev, STEPPER_CTRL_EVENT_STEPS_COMPLETED);
	}

	return ret;
}

static int tmc524x_stepper_ctrl_move_by(const struct device *dev, const int32_t micro_steps)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	uint32_t raw;
	int32_t xactual;
	int64_t target;
	bool completed;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = tmc524x_reg_read(cfg->controller, TMC524X_REG_XACTUAL, &raw);
	if (ret != 0) {
		goto unlock;
	}
	xactual = (int32_t)raw;

	target = (int64_t)xactual + micro_steps;
	if (target < INT32_MIN || target > INT32_MAX) {
		ret = -ERANGE;
		goto unlock;
	}

	ret = tmc524x_stepper_ctrl_move_to_locked(dev, (int32_t)target, &completed);

unlock:
	k_mutex_unlock(&data->lock);
	if (ret == 0 && completed) {
		tmc524x_stepper_ctrl_fire(dev, STEPPER_CTRL_EVENT_STEPS_COMPLETED);
	}
	return ret;
}

static int tmc524x_stepper_ctrl_run(const struct device *dev,
					    const enum stepper_ctrl_direction direction)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	uint32_t mode;
	int32_t vactual;
	int ret;

	if (direction == STEPPER_CTRL_DIRECTION_POSITIVE) {
		mode = TMC524X_RAMPMODE_VEL_POS;
	} else if (direction == STEPPER_CTRL_DIRECTION_NEGATIVE) {
		mode = TMC524X_RAMPMODE_VEL_NEG;
	} else {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->stop_pending) {
		ret = -EBUSY;
		goto unlock;
	}
	if (data->velocity_reg == 0U) {
		data->velocity_reg = cfg->vmax;
	}
	if (data->velocity_reg == 0U) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = tmc524x_read_s24(cfg->controller, TMC524X_REG_VACTUAL, &vactual);
	if (ret != 0) {
		goto unlock;
	}
	ret = tmc524x_stepper_ctrl_prepare_motion_locked(dev, vactual);
	if (ret != 0) {
		goto unlock;
	}
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_VMAX, data->velocity_reg);
	if (ret != 0) {
		goto unlock;
	}

	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPMODE, mode);
	if (ret == 0) {
		tmc524x_stepper_ctrl_cancel_stop_locked(dev);
		tmc524x_stepper_ctrl_start_poll_locked(dev, TMC524X_CTRL_POLL_VELOCITY);
	}

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int tmc524x_stepper_ctrl_stop(const struct device *dev)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	int32_t vactual;
	bool fire_stopped = false;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = tmc524x_read_s24(cfg->controller, TMC524X_REG_VACTUAL, &vactual);
	if (ret != 0) {
		goto unlock;
	}
	if (vactual == 0) {
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPMODE,
					 TMC524X_RAMPMODE_HOLD);
		if (ret == 0) {
			tmc524x_stepper_ctrl_cancel_poll_locked(dev);
			tmc524x_stepper_ctrl_cancel_stop_locked(dev);
			fire_stopped = true;
		}
		goto unlock;
	}

	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_VMAX, 0);
	if (ret != 0) {
		goto unlock;
	}
	tmc524x_stepper_ctrl_cancel_poll_locked(dev);
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPMODE,
				 vactual > 0 ? TMC524X_RAMPMODE_VEL_POS :
					       TMC524X_RAMPMODE_VEL_NEG);
	/* VMAX=0 already initiates a stop even if changing RAMPMODE failed. */
	data->stop_pending = true;
	k_work_reschedule(&data->stop_work, K_NO_WAIT);

unlock:
	k_mutex_unlock(&data->lock);
	if (fire_stopped) {
		tmc524x_stepper_ctrl_fire(dev, STEPPER_CTRL_EVENT_STOPPED);
	}
	return ret;
}

static int tmc524x_stepper_ctrl_is_moving(const struct device *dev, bool *is_moving)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	int32_t vactual;
	int ret;

	if (is_moving == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = tmc524x_read_s24(cfg->controller, TMC524X_REG_VACTUAL, &vactual);
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		return ret;
	}

	*is_moving = vactual != 0;
	return 0;
}

static int tmc524x_stepper_ctrl_init(const struct device *dev)
{
	const struct tmc524x_stepper_ctrl_config *cfg = dev->config;
	struct tmc524x_stepper_ctrl_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->controller)) {
		return -ENODEV;
	}
	k_mutex_init(&data->lock);
	data->dev = dev;
	k_work_init_delayable(&data->stop_work, tmc524x_stepper_ctrl_stop_work);

#if defined(CONFIG_STEPPER_ADI_TMC524X_CTRL_POLLING)
	data->poll_mode = TMC524X_CTRL_POLL_INACTIVE;
	k_work_init_delayable(&data->poll_work, tmc524x_stepper_ctrl_poll);
#endif

	ret = tmc524x_stepper_ctrl_apply_default_ramp(dev);
	if (ret != 0) {
		return ret;
	}

	if (cfg->default_microstep_interval_ns != 0U) {
		return tmc524x_stepper_ctrl_set_microstep_interval(dev,
				cfg->default_microstep_interval_ns);
	}

	data->velocity_reg = cfg->vmax;
	return 0;
}

static DEVICE_API(stepper_ctrl, tmc524x_stepper_ctrl_api) = {
	.set_reference_position = tmc524x_stepper_ctrl_set_reference_position,
	.get_actual_position = tmc524x_stepper_ctrl_get_actual_position,
	.set_event_cb = tmc524x_stepper_ctrl_set_event_cb,
	.set_microstep_interval = tmc524x_stepper_ctrl_set_microstep_interval,
	.configure_ramp = tmc524x_stepper_ctrl_configure_ramp,
	.move_by = tmc524x_stepper_ctrl_move_by,
	.move_to = tmc524x_stepper_ctrl_move_to,
	.run = tmc524x_stepper_ctrl_run,
	.stop = tmc524x_stepper_ctrl_stop,
	.is_moving = tmc524x_stepper_ctrl_is_moving,
};

#define TMC524X_CTRL_RANGE_ASSERT(inst, prop, minv, maxv) \
	BUILD_ASSERT(DT_INST_PROP(inst, prop) >= (minv) && DT_INST_PROP(inst, prop) <= (maxv), \
		     "TMC524X controller property " #prop " out of range")

#define TMC524X_CTRL_COMPILE_ASSERTS(inst) \
	BUILD_ASSERT(CONFIG_STEPPER_INIT_PRIORITY < \
		     CONFIG_STEPPER_ADI_TMC524X_CTRL_INIT_PRIORITY, \
		     "TMC524X controller init priority must follow the parent/driver"); \
	TMC524X_CTRL_RANGE_ASSERT(inst, vstart, 0, 0x3ffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, a1, 0, 0x3ffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, v1, 0, 0xfffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, amax, 1, 0x3ffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, vmax, 1, 0x7fffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, dmax, 1, 0x3ffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, tvmax, 0, 0xffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, d1, 1, 0x3ffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, vstop, 1, 0x3ffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, tzerowait, 0, 0xffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, v2, 0, 0xfffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, a2, 0, 0x3ffff); \
	TMC524X_CTRL_RANGE_ASSERT(inst, d2, 1, 0x3ffff); \
	BUILD_ASSERT(DT_INST_PROP(inst, vstop) >= DT_INST_PROP(inst, vstart), \
		     "vstop should be >= vstart for normal positioning mode"); \
	BUILD_ASSERT(DT_INST_PROP(inst, vmax) >= DT_INST_PROP(inst, vstart), \
		     "vmax must be >= vstart"); \
	BUILD_ASSERT(DT_INST_PROP(inst, v2) == 0 || \
		     (DT_INST_PROP(inst, v2) > DT_INST_PROP(inst, v1) && \
		      DT_INST_PROP(inst, v2) < DT_INST_PROP(inst, vmax) && \
		      DT_INST_PROP(inst, a2) > 0), \
		     "enabled eight-point segment requires v1 < v2 < vmax and a2 > 0")

#define TMC524X_STEPPER_CTRL_DEFINE(inst) \
	TMC524X_CTRL_COMPILE_ASSERTS(inst); \
	static const struct tmc524x_stepper_ctrl_config tmc524x_stepper_ctrl_config_##inst = { \
		.controller = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))), \
		.vstart = DT_INST_PROP(inst, vstart), \
		.a1 = DT_INST_PROP(inst, a1), \
		.v1 = DT_INST_PROP(inst, v1), \
		.amax = DT_INST_PROP(inst, amax), \
		.vmax = DT_INST_PROP(inst, vmax), \
		.dmax = DT_INST_PROP(inst, dmax), \
		.tvmax = DT_INST_PROP(inst, tvmax), \
		.d1 = DT_INST_PROP(inst, d1), \
		.vstop = DT_INST_PROP(inst, vstop), \
		.tzerowait = DT_INST_PROP(inst, tzerowait), \
		.v2 = DT_INST_PROP(inst, v2), \
		.a2 = DT_INST_PROP(inst, a2), \
		.d2 = DT_INST_PROP(inst, d2), \
		.default_microstep_interval_ns = DT_INST_PROP(inst, default_microstep_interval_ns), \
	}; \
	static struct tmc524x_stepper_ctrl_data tmc524x_stepper_ctrl_data_##inst; \
	DEVICE_DT_INST_DEFINE(inst, tmc524x_stepper_ctrl_init, NULL, \
			      &tmc524x_stepper_ctrl_data_##inst, \
			      &tmc524x_stepper_ctrl_config_##inst, POST_KERNEL, \
			      CONFIG_STEPPER_ADI_TMC524X_CTRL_INIT_PRIORITY, \
			      &tmc524x_stepper_ctrl_api);

DT_INST_FOREACH_STATUS_OKAY(TMC524X_STEPPER_CTRL_DEFINE)

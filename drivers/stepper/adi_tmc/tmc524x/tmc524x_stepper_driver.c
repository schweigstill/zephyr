/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT adi_tmc524x_stepper_driver

#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "tmc524x.h"

LOG_MODULE_DECLARE(tmc524x, CONFIG_STEPPER_LOG_LEVEL);

#define TMC524X_DIAG_RETRY_DELAY K_MSEC(10)

struct tmc524x_stepper_driver_config {
	const struct device *controller;
	struct gpio_dt_spec en_gpio;
	struct gpio_dt_spec diag0_stall_gpio;
	uint16_t micro_step_res;
	uint8_t ihold;
	uint8_t irun;
	uint8_t iholddelay;
	uint8_t irundelay;
	uint8_t tpowerdown;
	uint8_t global_scaler;
	uint8_t current_range;
	uint8_t slope_control;
	uint8_t toff;
	uint8_t hstrt;
	int8_t hend;
	uint8_t tbl;
	uint8_t chm;
	uint8_t tpfd;
	bool interpolation;
	bool double_edge_step;
	bool stealthchop2_enable;
	bool spreadcycle_enable;
	bool spreadcycle_stealthchop_hybrid;
	uint32_t tpwmthrs;
	uint32_t tcoolthrs;
	uint32_t thigh;
	struct tmc524x_pwm_config pwm;
	bool stallguard2_enable;
	int8_t stallguard2_threshold;
	bool stallguard2_filter;
	bool stallguard4_enable;
	struct tmc524x_sg4_config sg4;
	bool coolstep_enable;
	struct tmc524x_coolstep_config coolstep;
	bool dcstep_enable;
	uint16_t dc_time;
	uint8_t dc_sg;
	uint16_t vdcmin;
	bool sg_stop_enable;
	bool soft_stop_enable;
	bool external_stepdir_enable;
	uint8_t length_step_pulse;
};

struct tmc524x_stepper_driver_data {
	struct k_spinlock callback_lock;
	stepper_event_cb_t event_cb;
	void *event_user_data;
	struct gpio_callback diag0_cb;
	struct k_work_delayable diag0_work;
	uint32_t diag0_generation;
	uint32_t diag0_rampstat;
	uint32_t diag0_drv_status;
	uint32_t diag0_enc_status;
	bool diag0_ramp_valid;
	bool diag0_drv_valid;
	bool diag0_enc_valid;
	const struct device *dev;
};

static uint32_t tmc524x_stepper_driver_build_chopconf(const struct tmc524x_stepper_driver_config *cfg,
						      bool enabled)
{
	uint32_t hend = (uint32_t)(cfg->hend + 3);
	uint32_t value = 0;

	value |= TMC524X_FLD(TMC524X_CHOPCONF_TOFF_MASK, TMC524X_CHOPCONF_TOFF_SHIFT,
			     enabled ? cfg->toff : 0U);
	value |= TMC524X_FLD(TMC524X_CHOPCONF_HSTRT_MASK, TMC524X_CHOPCONF_HSTRT_SHIFT,
			     cfg->hstrt);
	value |= TMC524X_FLD(TMC524X_CHOPCONF_HEND_MASK, TMC524X_CHOPCONF_HEND_SHIFT, hend);
	value |= TMC524X_FLD(TMC524X_CHOPCONF_TBL_MASK, TMC524X_CHOPCONF_TBL_SHIFT, cfg->tbl);
	value |= TMC524X_FLD(TMC524X_CHOPCONF_TPFD_MASK, TMC524X_CHOPCONF_TPFD_SHIFT, cfg->tpfd);
	value |= TMC524X_FLD(TMC524X_CHOPCONF_MRES_MASK, TMC524X_CHOPCONF_MRES_SHIFT,
			     tmc524x_microstep_to_mres(cfg->micro_step_res));

	if (cfg->chm != 0U) {
		value |= TMC524X_CHOPCONF_CHM;
	}
	if (cfg->interpolation) {
		value |= TMC524X_CHOPCONF_INTPOL;
	}
	if (cfg->double_edge_step) {
		value |= TMC524X_CHOPCONF_DEDGE;
	}
	if (cfg->dcstep_enable) {
		value |= TMC524X_CHOPCONF_VHIGHFS | TMC524X_CHOPCONF_VHIGHCHM;
	}

	return value;
}

static const struct gpio_dt_spec *
tmc524x_stepper_driver_get_en_gpio(const struct tmc524x_stepper_driver_config *cfg)
{
	return &cfg->en_gpio;
}

static void tmc524x_stepper_driver_fire(const struct device *dev, enum stepper_event event)
{
	const struct tmc524x_stepper_driver_config *cfg = dev->config;
	struct tmc524x_stepper_driver_data *data = dev->data;
	stepper_event_cb_t callback;
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

static void tmc524x_stepper_driver_diag0_work(struct k_work *work)
{
	struct tmc524x_stepper_driver_data *data = CONTAINER_OF(work,
		struct tmc524x_stepper_driver_data, diag0_work.work);
	const struct device *dev = data->dev;
	const struct tmc524x_stepper_driver_config *cfg = dev->config;
	uint32_t drv_status;
	uint32_t rampstat;
	uint32_t enc_status;
	uint32_t generation;
	uint32_t enc_w1c;
	bool ramp_valid;
	bool drv_valid;
	bool enc_valid;
	bool stall;
	bool fault = false;
	bool changed;
	k_spinlock_key_t key;
	int ret;

	key = k_spin_lock(&data->callback_lock);
	generation = data->diag0_generation;
	ramp_valid = data->diag0_ramp_valid;
	drv_valid = data->diag0_drv_valid;
	enc_valid = data->diag0_enc_valid;
	rampstat = data->diag0_rampstat;
	drv_status = data->diag0_drv_status;
	enc_status = data->diag0_enc_status;
	k_spin_unlock(&data->callback_lock, key);

	if (!ramp_valid) {
		ret = tmc524x_get_ramp_status(cfg->controller, &rampstat);
		if (ret == 0) {
			key = k_spin_lock(&data->callback_lock);
			if (generation == data->diag0_generation) {
				data->diag0_rampstat = rampstat;
				data->diag0_ramp_valid = true;
				ramp_valid = true;
			}
			k_spin_unlock(&data->callback_lock, key);
		}
	}
	if (!drv_valid) {
		ret = tmc524x_get_drv_status(cfg->controller, &drv_status);
		if (ret == 0) {
			key = k_spin_lock(&data->callback_lock);
			if (generation == data->diag0_generation) {
				data->diag0_drv_status = drv_status;
				data->diag0_drv_valid = true;
				drv_valid = true;
			}
			k_spin_unlock(&data->callback_lock, key);
		}
	}
	if (!enc_valid) {
		ret = tmc524x_reg_read(cfg->controller, TMC524X_REG_ENC_STATUS, &enc_status);
		if (ret == 0) {
			key = k_spin_lock(&data->callback_lock);
			if (generation == data->diag0_generation) {
				data->diag0_enc_status = enc_status;
				data->diag0_enc_valid = true;
				enc_valid = true;
			}
			k_spin_unlock(&data->callback_lock, key);
		}
	}
	if (enc_valid) {
		enc_w1c = enc_status & TMC524X_ENC_STATUS_W1C_EVENTS;
		if (enc_w1c != 0U) {
			ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_ENC_STATUS, enc_w1c);
			if (ret != 0) {
				k_work_reschedule(&data->diag0_work, TMC524X_DIAG_RETRY_DELAY);
				return;
			}
		}
	}

	key = k_spin_lock(&data->callback_lock);
	changed = generation != data->diag0_generation;
	if (!changed && ramp_valid && drv_valid && enc_valid) {
		data->diag0_ramp_valid = false;
		data->diag0_drv_valid = false;
		data->diag0_enc_valid = false;
		tmc524x_diag0_clear_pending(cfg->controller);
	}
	k_spin_unlock(&data->callback_lock, key);
	if (changed || !ramp_valid || !drv_valid || !enc_valid) {
		k_work_reschedule(&data->diag0_work, TMC524X_DIAG_RETRY_DELAY);
		return;
	}

	/* N_event also signals normal motion-controller and encoder events. Report a
	 * stall only when a status register confirms it. Inferring a stall merely
	 * from an edge whose source is no longer latched would misclassify a normal
	 * event that another consumer acknowledged before this work item ran.
	 */
	stall = (drv_status & TMC524X_DRV_STATUS_STALLGUARD) != 0U ||
		(rampstat & TMC524X_RAMPSTAT_EVENT_STOP_SG) != 0U;
	if (stall) {
		tmc524x_stepper_driver_fire(dev, STEPPER_EVENT_STALL_DETECTED);
	}

	fault = (drv_status & (TMC524X_DRV_STATUS_OT | TMC524X_DRV_STATUS_S2GA |
				      TMC524X_DRV_STATUS_S2GB | TMC524X_DRV_STATUS_S2VSA |
				      TMC524X_DRV_STATUS_S2VSB)) != 0U;
	if (fault) {
		/* Fault reporting is opportunistic: DIAG0/N_event is not a fault IRQ. */
		tmc524x_stepper_driver_fire(dev, STEPPER_EVENT_FAULT_DETECTED);
	}
}

static void tmc524x_stepper_driver_schedule_diag0(const struct device *dev)
{
	struct tmc524x_stepper_driver_data *data = dev->data;
	const struct tmc524x_stepper_driver_config *cfg = dev->config;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->callback_lock);
	data->diag0_generation++;
	data->diag0_ramp_valid = false;
	data->diag0_drv_valid = false;
	data->diag0_enc_valid = false;
	tmc524x_diag0_mark_pending(cfg->controller);
	k_spin_unlock(&data->callback_lock, key);
	(void)k_work_reschedule(&data->diag0_work, K_NO_WAIT);
}

static void tmc524x_stepper_driver_diag0_isr(const struct device *port,
					     struct gpio_callback *cb, uint32_t pins)
{
	struct tmc524x_stepper_driver_data *data = CONTAINER_OF(cb,
		struct tmc524x_stepper_driver_data, diag0_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	tmc524x_stepper_driver_schedule_diag0(data->dev);
}

static int tmc524x_stepper_driver_set_event_cb(const struct device *dev,
					       stepper_event_cb_t cb, void *user_data)
{
	struct tmc524x_stepper_driver_data *data = dev->data;
	const struct tmc524x_stepper_driver_config *cfg = dev->config;
	k_spinlock_key_t key;
	bool called_from_callback;

	called_from_callback = tmc524x_callback_dispatch_is_current(cfg->controller);
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

static int tmc524x_stepper_driver_enable(const struct device *dev)
{
	const struct tmc524x_stepper_driver_config *cfg = dev->config;
	const struct gpio_dt_spec *en_gpio = tmc524x_stepper_driver_get_en_gpio(cfg);
	int ret;

	/* Only restore TOFF so runtime CHOPCONF changes, including MRES, survive. */
	ret = tmc524x_reg_update(cfg->controller, TMC524X_REG_CHOPCONF,
				 TMC524X_CHOPCONF_TOFF_MASK,
				 TMC524X_FLD(TMC524X_CHOPCONF_TOFF_MASK,
					     TMC524X_CHOPCONF_TOFF_SHIFT, cfg->toff));
	if (ret != 0) {
		return ret;
	}

	if (en_gpio->port != NULL) {
		ret = gpio_pin_set_dt(en_gpio, 1);
		if (ret != 0) {
			(void)tmc524x_reg_update(cfg->controller, TMC524X_REG_CHOPCONF,
						 TMC524X_CHOPCONF_TOFF_MASK, 0U);
		}
	}

	return ret;
}

static int tmc524x_stepper_driver_disable(const struct device *dev)
{
	const struct tmc524x_stepper_driver_config *cfg = dev->config;
	const struct gpio_dt_spec *en_gpio = tmc524x_stepper_driver_get_en_gpio(cfg);
	int ret;

	ret = tmc524x_reg_update(cfg->controller, TMC524X_REG_CHOPCONF,
				  TMC524X_CHOPCONF_TOFF_MASK, 0U);
	if (en_gpio->port != NULL) {
		int gpio_ret = gpio_pin_set_dt(en_gpio, 0);

		if (ret == 0) {
			ret = gpio_ret;
		}
	}

	return ret;
}

static int tmc524x_stepper_driver_set_micro_step_res(const struct device *dev,
						     enum stepper_micro_step_resolution res)
{
	const struct tmc524x_stepper_driver_config *cfg = dev->config;
	uint8_t mres;

	if (!VALID_MICRO_STEP_RES(res)) {
		return -EINVAL;
	}

	mres = tmc524x_microstep_to_mres((uint16_t)res);
	return tmc524x_reg_update(cfg->controller, TMC524X_REG_CHOPCONF,
				  TMC524X_CHOPCONF_MRES_MASK,
				  TMC524X_FLD(TMC524X_CHOPCONF_MRES_MASK,
					      TMC524X_CHOPCONF_MRES_SHIFT, mres));
}

static int tmc524x_stepper_driver_get_micro_step_res(const struct device *dev,
						     enum stepper_micro_step_resolution *res)
{
	const struct tmc524x_stepper_driver_config *cfg = dev->config;
	uint32_t chopconf;
	uint8_t mres;
	int ret;

	if (res == NULL) {
		return -EINVAL;
	}

	ret = tmc524x_reg_read(cfg->controller, TMC524X_REG_CHOPCONF, &chopconf);
	if (ret != 0) {
		return ret;
	}

	mres = TMC524X_GET(chopconf, TMC524X_CHOPCONF_MRES_MASK,
				    TMC524X_CHOPCONF_MRES_SHIFT);
	*res = (enum stepper_micro_step_resolution)tmc524x_mres_to_microstep(mres);
	return 0;
}

static int tmc524x_stepper_driver_apply_static_config(const struct device *dev)
{
	const struct tmc524x_stepper_driver_config *cfg = dev->config;
	uint32_t value;
	int ret;

	/* Disable the power stage in the IC before changing any mode that could
	 * release a warm-start movement. This is required even when no external
	 * DRV_ENN GPIO is described.
	 */
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_CHOPCONF,
				 tmc524x_stepper_driver_build_chopconf(cfg, false));
	if (ret != 0) {
		return ret;
	}

	/* Start from deterministic GCONF semantics after an MCU-only warm reset.
	 * Unsupported direct-current, encoder-stop, direction and push-pull modes
	 * must not leak in from an earlier application. The configured chopper and
	 * optional external STEP/DIR fields are applied later in this function.
	 */
	ret = tmc524x_reg_update(cfg->controller, TMC524X_REG_GCONF,
				  TMC524X_GCONF_FAST_STANDSTILL |
				  TMC524X_GCONF_EN_PWM_MODE |
				  TMC524X_GCONF_SHAFT |
				  TMC524X_GCONF_DIAG0_NINT_STEP |
				  TMC524X_GCONF_DIAG1_NPOSCOMP_DIR |
				  TMC524X_GCONF_DIAG0_INT_PUSHPULL |
				  TMC524X_GCONF_DIAG1_POSCOMP_PUSHPULL |
				  TMC524X_GCONF_SMALL_HYSTERESIS |
				  TMC524X_GCONF_STOP_ENABLE |
				  TMC524X_GCONF_DIRECT_MODE |
				  TMC524X_GCONF_LENGTH_STEP_PULSE_MASK,
				  0U);
	if (ret != 0) {
		return ret;
	}

	value = TMC524X_FLD(TMC524X_DRV_CONF_CURRENT_RANGE_MASK,
			    TMC524X_DRV_CONF_CURRENT_RANGE_SHIFT, cfg->current_range) |
		TMC524X_FLD(TMC524X_DRV_CONF_SLOPE_CONTROL_MASK,
			    TMC524X_DRV_CONF_SLOPE_CONTROL_SHIFT, cfg->slope_control);
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_DRV_CONF, value);
	if (ret != 0) {
		return ret;
	}

	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_GLOBAL_SCALER,
				 cfg->global_scaler);
	if (ret != 0) {
		return ret;
	}

	value = TMC524X_FLD(TMC524X_IHOLD_IRUN_IHOLD_MASK,
			    TMC524X_IHOLD_IRUN_IHOLD_SHIFT, cfg->ihold) |
		TMC524X_FLD(TMC524X_IHOLD_IRUN_IRUN_MASK,
			    TMC524X_IHOLD_IRUN_IRUN_SHIFT, cfg->irun) |
		TMC524X_FLD(TMC524X_IHOLD_IRUN_IHOLDDELAY_MASK,
			    TMC524X_IHOLD_IRUN_IHOLDDELAY_SHIFT, cfg->iholddelay) |
		TMC524X_FLD(TMC524X_IHOLD_IRUN_IRUNDELAY_MASK,
			    TMC524X_IHOLD_IRUN_IRUNDELAY_SHIFT, cfg->irundelay);
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_IHOLD_IRUN, value);
	if (ret != 0) {
		return ret;
	}

	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_TPOWERDOWN, cfg->tpowerdown);
	if (ret != 0) {
		return ret;
	}

	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_TPWMTHRS, cfg->tpwmthrs);
	if (ret != 0) {
		return ret;
	}
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_TCOOLTHRS, cfg->tcoolthrs);
	if (ret != 0) {
		return ret;
	}
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_THIGH, cfg->thigh);
	if (ret != 0) {
		return ret;
	}

	if (cfg->stealthchop2_enable) {
		ret = tmc524x_set_stealthchop2(cfg->controller, true, &cfg->pwm);
	} else {
		ret = tmc524x_set_spreadcycle(cfg->controller);
	}
	if (ret != 0) {
		return ret;
	}

	ret = tmc524x_set_stallguard2(cfg->controller,
				      cfg->stallguard2_enable ? cfg->stallguard2_threshold : 0,
				      cfg->stallguard2_enable && cfg->stallguard2_filter);
	if (ret != 0) {
		return ret;
	}

	if (cfg->stallguard4_enable) {
		ret = tmc524x_set_stallguard4(cfg->controller, &cfg->sg4);
	} else {
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_SG4_THRS, 0U);
	}
	if (ret != 0) {
		return ret;
	}

	if (cfg->coolstep_enable) {
		ret = tmc524x_set_coolstep(cfg->controller, &cfg->coolstep);
	} else {
		static const struct tmc524x_coolstep_config disabled_coolstep;

		ret = tmc524x_set_coolstep(cfg->controller, &disabled_coolstep);
	}
	if (ret != 0) {
		return ret;
	}

	value = 0U;
	if (cfg->dcstep_enable) {
		value = TMC524X_FLD(TMC524X_DCCTRL_DC_TIME_MASK,
				    TMC524X_DCCTRL_DC_TIME_SHIFT, cfg->dc_time) |
			TMC524X_FLD(TMC524X_DCCTRL_DC_SG_MASK,
				    TMC524X_DCCTRL_DC_SG_SHIFT, cfg->dc_sg);
	}
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_DCCTRL, value);
	if (ret != 0) {
		return ret;
	}
	value = cfg->dcstep_enable ?
		TMC524X_FLD(TMC524X_VDCMIN_MASK, TMC524X_VDCMIN_SHIFT, cfg->vdcmin) : 0U;
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_VDCMIN, value);
	if (ret != 0) {
		return ret;
	}

	value = 0;
	if (cfg->sg_stop_enable) {
		value |= TMC524X_SWMODE_SG_STOP;
	}
	if (cfg->soft_stop_enable) {
		value |= TMC524X_SWMODE_EN_SOFTSTOP;
	}
	/* No other SWMODE fields are modeled by this binding. Write the complete
	 * register so stale physical/virtual stop, polarity or latch settings from
	 * an earlier MCU image cannot affect the new motion controller.
	 */
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_SWMODE, value);
	if (ret != 0) {
		return ret;
	}

	value = 0U;
	if (cfg->external_stepdir_enable) {
		value = TMC524X_GCONF_DIAG0_NINT_STEP | TMC524X_GCONF_DIAG1_NPOSCOMP_DIR |
			TMC524X_FLD(TMC524X_GCONF_LENGTH_STEP_PULSE_MASK,
				    TMC524X_GCONF_LENGTH_STEP_PULSE_SHIFT, cfg->length_step_pulse);
	}
	return tmc524x_reg_update(cfg->controller, TMC524X_REG_GCONF,
				  TMC524X_GCONF_DIAG0_NINT_STEP |
				  TMC524X_GCONF_DIAG1_NPOSCOMP_DIR |
				  TMC524X_GCONF_LENGTH_STEP_PULSE_MASK,
				  value);
}

static int tmc524x_stepper_driver_init(const struct device *dev)
{
	const struct tmc524x_stepper_driver_config *cfg = dev->config;
	struct tmc524x_stepper_driver_data *data = dev->data;
	uint32_t enc_status;
	uint32_t rampstat;
	uint32_t vactual;
	int ret;

	if (!device_is_ready(cfg->controller)) {
		return -ENODEV;
	}

	data->dev = dev;
	k_work_init_delayable(&data->diag0_work, tmc524x_stepper_driver_diag0_work);

	if (cfg->en_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->en_gpio)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->en_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			return ret;
		}
	}

	ret = tmc524x_stepper_driver_apply_static_config(dev);
	if (ret != 0) {
		return ret;
	}

	/* event_pos_reached is set after reset and can otherwise hold N_event active.
	 * Never clear an inherited event_stop_sg until a stationary ramp is in HOLD:
	 * its W1C acknowledgement can release an old warm-start movement.
	 */
	ret = tmc524x_get_ramp_status(cfg->controller, &rampstat);
	if (ret != 0) {
		return ret;
	}
	if ((rampstat & TMC524X_RAMPSTAT_EVENT_STOP_SG) != 0U) {
		ret = tmc524x_reg_read(cfg->controller, TMC524X_REG_VACTUAL, &vactual);
		if (ret != 0) {
			return ret;
		}
		if ((vactual & 0xffffffU) != 0U) {
			return -EBUSY;
		}
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPMODE,
					 TMC524X_RAMPMODE_HOLD);
		if (ret != 0) {
			return ret;
		}
		ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPSTAT,
					 TMC524X_RAMPSTAT_EVENT_STOP_SG);
		if (ret != 0) {
			return ret;
		}
	}
	ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_RAMPSTAT,
				 TMC524X_RAMPSTAT_EVENT_POS_REACHED);
	if (ret != 0) {
		return ret;
	}

	if (cfg->diag0_stall_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->diag0_stall_gpio)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->diag0_stall_gpio, GPIO_INPUT);
		if (ret != 0) {
			return ret;
		}
		/* Remove encoder W1C sources left by an earlier MCU run before the
		 * edge-triggered input is armed. A persistent deviation may assert the
		 * line again; the active-level check below starts the worker explicitly.
		 */
		ret = tmc524x_reg_read(cfg->controller, TMC524X_REG_ENC_STATUS, &enc_status);
		if (ret != 0) {
			return ret;
		}
		enc_status &= TMC524X_ENC_STATUS_W1C_EVENTS;
		if (enc_status != 0U) {
			ret = tmc524x_reg_write(cfg->controller, TMC524X_REG_ENC_STATUS,
						 enc_status);
			if (ret != 0) {
				return ret;
			}
		}
		gpio_init_callback(&data->diag0_cb, tmc524x_stepper_driver_diag0_isr,
				   BIT(cfg->diag0_stall_gpio.pin));
		ret = gpio_add_callback(cfg->diag0_stall_gpio.port, &data->diag0_cb);
		if (ret != 0) {
			return ret;
		}
		ret = gpio_pin_interrupt_configure_dt(&cfg->diag0_stall_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		if (ret != 0) {
			(void)gpio_remove_callback(cfg->diag0_stall_gpio.port, &data->diag0_cb);
			return ret;
		}
		ret = gpio_pin_get_dt(&cfg->diag0_stall_gpio);
		if (ret < 0) {
			(void)gpio_pin_interrupt_configure_dt(&cfg->diag0_stall_gpio,
						      GPIO_INT_DISABLE);
			(void)gpio_remove_callback(cfg->diag0_stall_gpio.port, &data->diag0_cb);
			return ret;
		}
		if (ret != 0) {
			tmc524x_stepper_driver_schedule_diag0(dev);
		}
	}

	return 0;
}

static DEVICE_API(stepper, tmc524x_stepper_driver_api) = {
	.enable = tmc524x_stepper_driver_enable,
	.disable = tmc524x_stepper_driver_disable,
	.set_micro_step_res = tmc524x_stepper_driver_set_micro_step_res,
	.get_micro_step_res = tmc524x_stepper_driver_get_micro_step_res,
	.set_event_cb = tmc524x_stepper_driver_set_event_cb,
};

#define TMC524X_RANGE_ASSERT(inst, prop, minv, maxv) \
	BUILD_ASSERT(DT_INST_PROP(inst, prop) >= (minv) && DT_INST_PROP(inst, prop) <= (maxv), \
		     "TMC524X property " #prop " out of range")

#define TMC524X_DRIVER_COMPILE_ASSERTS(inst) \
	BUILD_ASSERT(VALID_MICRO_STEP_RES(DT_INST_PROP(inst, micro_step_res)), \
		     "micro-step-res must be one of 1,2,4,8,16,32,64,128,256"); \
	TMC524X_RANGE_ASSERT(inst, ihold, 0, 31); \
	TMC524X_RANGE_ASSERT(inst, irun, 0, 31); \
	TMC524X_RANGE_ASSERT(inst, iholddelay, 0, 15); \
	TMC524X_RANGE_ASSERT(inst, irundelay, 0, 15); \
	TMC524X_RANGE_ASSERT(inst, tpowerdown, 0, 255); \
	BUILD_ASSERT(DT_INST_PROP(inst, global_scaler) == 0 || \
		     (DT_INST_PROP(inst, global_scaler) >= 32 && DT_INST_PROP(inst, global_scaler) <= 255), \
		     "global-scaler must be 0 or 32..255"); \
	TMC524X_RANGE_ASSERT(inst, toff, 1, 15); \
	TMC524X_RANGE_ASSERT(inst, hstrt, 0, 7); \
	TMC524X_RANGE_ASSERT(inst, hend, -3, 12); \
	TMC524X_RANGE_ASSERT(inst, tpfd, 0, 15); \
	TMC524X_RANGE_ASSERT(inst, tpwmthrs, 0, 0xfffff); \
	TMC524X_RANGE_ASSERT(inst, tcoolthrs, 0, 0xfffff); \
	TMC524X_RANGE_ASSERT(inst, thigh, 0, 0xfffff); \
	TMC524X_RANGE_ASSERT(inst, pwm_ofs, 0, 255); \
	TMC524X_RANGE_ASSERT(inst, pwm_grad, 0, 255); \
	TMC524X_RANGE_ASSERT(inst, pwm_reg, 0, 15); \
	TMC524X_RANGE_ASSERT(inst, pwm_lim, 0, 15); \
	TMC524X_RANGE_ASSERT(inst, stallguard2_threshold, -64, 63); \
	TMC524X_RANGE_ASSERT(inst, stallguard4_threshold, 0, 255); \
	TMC524X_RANGE_ASSERT(inst, semin, 0, 15); \
	TMC524X_RANGE_ASSERT(inst, semax, 0, 15); \
	TMC524X_RANGE_ASSERT(inst, dc_time, 0, 1023); \
	TMC524X_RANGE_ASSERT(inst, dc_sg, 0, 255); \
	TMC524X_RANGE_ASSERT(inst, vdcmin, 0, 0x7fff); \
	TMC524X_RANGE_ASSERT(inst, length_step_pulse, 0, 15); \
	BUILD_ASSERT(DT_INST_PROP(inst, chm) || \
		     DT_INST_PROP(inst, hstrt) + DT_INST_PROP(inst, hend) <= 15, \
		     "SpreadCycle requires hstrt + hend <= 15"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, spreadcycle_enable) || !DT_INST_PROP(inst, chm), \
		     "spreadcycle-enable requires chm=0"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, spreadcycle_stealthchop_hybrid) || \
		     !DT_INST_PROP(inst, chm), \
		     "hybrid operation requires chm=0"); \
	BUILD_ASSERT(!(DT_INST_PROP(inst, stealthchop2_enable) && \
		       DT_INST_PROP(inst, spreadcycle_enable)) || \
		     DT_INST_PROP(inst, spreadcycle_stealthchop_hybrid), \
		     "combined StealthChop2 and SpreadCycle requires hybrid operation"); \
	BUILD_ASSERT(!(DT_INST_PROP(inst, stallguard2_enable) && \
		       DT_INST_PROP(inst, stallguard4_enable) && \
		       !DT_INST_PROP(inst, spreadcycle_stealthchop_hybrid)), \
		     "StallGuard2 and StallGuard4 together require spreadcycle-stealthchop-hybrid"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, stallguard2_enable) || \
		     DT_INST_PROP(inst, spreadcycle_enable) || \
		     DT_INST_PROP(inst, spreadcycle_stealthchop_hybrid), \
		     "stallguard2-enable requires spreadcycle-enable or spreadcycle-stealthchop-hybrid"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, stallguard4_enable) || DT_INST_PROP(inst, stealthchop2_enable), \
		     "stallguard4-enable requires stealthchop2-enable"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, spreadcycle_stealthchop_hybrid) || \
		     (DT_INST_PROP(inst, stealthchop2_enable) && DT_INST_PROP(inst, spreadcycle_enable) && \
		      DT_INST_PROP(inst, tpwmthrs) != 0), \
		     "hybrid operation requires stealthchop2-enable, spreadcycle-enable and nonzero tpwmthrs"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, stealthchop2_enable) || \
		     DT_INST_PROP(inst, tpwmthrs) == 0 || \
		     DT_INST_PROP(inst, spreadcycle_stealthchop_hybrid), \
		     "nonzero tpwmthrs with StealthChop2 requires declared hybrid operation"); \
	BUILD_ASSERT(!(DT_INST_PROP(inst, stallguard2_enable) || \
		       DT_INST_PROP(inst, stallguard4_enable) || \
		       DT_INST_PROP(inst, sg_stop_enable)) || DT_INST_PROP(inst, tcoolthrs) > 0, \
		     "StallGuard threshold and stop functions require nonzero tcoolthrs"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, stallguard4_enable) || \
		     !DT_INST_PROP(inst, spreadcycle_stealthchop_hybrid) || \
		     DT_INST_PROP(inst, tcoolthrs) > DT_INST_PROP(inst, tpwmthrs), \
		     "hybrid StallGuard4 requires tcoolthrs > tpwmthrs"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, coolstep_enable) || \
		     (DT_INST_PROP(inst, semin) > 0 && DT_INST_PROP(inst, tcoolthrs) > 0 && \
		      (DT_INST_PROP(inst, stallguard2_enable) || DT_INST_PROP(inst, stallguard4_enable))), \
		     "coolstep-enable requires semin > 0, tcoolthrs > 0 and StallGuard2 or StallGuard4"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, coolstep_enable) || \
		     DT_INST_PROP(inst, tcoolthrs) >= DT_INST_PROP(inst, thigh), \
		     "CoolStep requires tcoolthrs >= thigh for a nonempty velocity window"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, dcstep_enable) || \
		     (DT_INST_PROP(inst, spreadcycle_enable) && DT_INST_PROP(inst, vdcmin) > 0), \
		     "dcstep-enable requires spreadcycle-enable and nonzero vdcmin"); \
	BUILD_ASSERT(!(DT_INST_PROP(inst, soft_stop_enable) && \
		       DT_INST_PROP(inst, stallguard2_enable)), \
		     "soft-stop-enable must not be combined with StallGuard2"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, sg_stop_enable) || \
		     DT_INST_PROP(inst, stallguard2_enable) || DT_INST_PROP(inst, dcstep_enable), \
		     "sg-stop-enable requires StallGuard2 or DcStep"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, external_stepdir_enable) || \
		     !(DT_INST_PROP(inst, stallguard2_enable) || DT_INST_PROP(inst, stallguard4_enable) || \
		       DT_INST_PROP(inst, coolstep_enable) || DT_INST_PROP(inst, dcstep_enable)), \
		     "external-stepdir-enable conflicts with StallGuard/CoolStep/DcStep feedback features"); \
	BUILD_ASSERT(!DT_INST_PROP(inst, external_stepdir_enable) || \
		     !DT_INST_NODE_HAS_PROP(inst, diag0_stall_gpios), \
		     "external-stepdir-enable conflicts with diag0-stall-gpios")

#define TMC524X_STEPPER_DRIVER_DEFINE(inst) \
	TMC524X_DRIVER_COMPILE_ASSERTS(inst); \
	static const struct tmc524x_stepper_driver_config tmc524x_stepper_driver_config_##inst = { \
		.controller = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))), \
		.en_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}), \
		.diag0_stall_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, diag0_stall_gpios, {0}), \
		.micro_step_res = DT_INST_PROP(inst, micro_step_res), \
		.ihold = DT_INST_PROP(inst, ihold), \
		.irun = DT_INST_PROP(inst, irun), \
		.iholddelay = DT_INST_PROP(inst, iholddelay), \
		.irundelay = DT_INST_PROP(inst, irundelay), \
		.tpowerdown = DT_INST_PROP(inst, tpowerdown), \
		.global_scaler = DT_INST_PROP(inst, global_scaler), \
		.current_range = DT_INST_PROP(inst, current_range), \
		.slope_control = DT_INST_PROP(inst, slope_control), \
		.toff = DT_INST_PROP(inst, toff), \
		.hstrt = DT_INST_PROP(inst, hstrt), \
		.hend = DT_INST_PROP(inst, hend), \
		.tbl = DT_INST_PROP(inst, tbl), \
		.chm = DT_INST_PROP(inst, chm), \
		.tpfd = DT_INST_PROP(inst, tpfd), \
		.interpolation = DT_INST_PROP(inst, interpolation), \
		.double_edge_step = DT_INST_PROP(inst, double_edge_step), \
		.stealthchop2_enable = DT_INST_PROP(inst, stealthchop2_enable), \
		.spreadcycle_enable = DT_INST_PROP(inst, spreadcycle_enable), \
		.spreadcycle_stealthchop_hybrid = DT_INST_PROP(inst, spreadcycle_stealthchop_hybrid), \
		.tpwmthrs = DT_INST_PROP(inst, tpwmthrs), \
		.tcoolthrs = DT_INST_PROP(inst, tcoolthrs), \
		.thigh = DT_INST_PROP(inst, thigh), \
		.pwm = { \
			.pwm_ofs = DT_INST_PROP(inst, pwm_ofs), \
			.pwm_grad = DT_INST_PROP(inst, pwm_grad), \
			.pwm_freq = DT_INST_PROP(inst, pwm_freq), \
			.freewheel = DT_INST_PROP(inst, freewheel), \
			.pwm_reg = DT_INST_PROP(inst, pwm_reg), \
			.pwm_lim = DT_INST_PROP(inst, pwm_lim), \
			.autoscale = DT_INST_PROP(inst, pwm_autoscale), \
			.autograd = DT_INST_PROP(inst, pwm_autograd), \
		}, \
		.stallguard2_enable = DT_INST_PROP(inst, stallguard2_enable), \
		.stallguard2_threshold = DT_INST_PROP(inst, stallguard2_threshold), \
		.stallguard2_filter = DT_INST_PROP(inst, stallguard2_filter), \
		.stallguard4_enable = DT_INST_PROP(inst, stallguard4_enable), \
		.sg4 = { \
			.threshold = DT_INST_PROP(inst, stallguard4_threshold), \
			.filter_enable = DT_INST_PROP(inst, stallguard4_filter), \
			.angle_offset_enable = DT_INST_PROP(inst, stallguard4_angle_offset), \
		}, \
		.coolstep_enable = DT_INST_PROP(inst, coolstep_enable), \
		.coolstep = { \
			.semin = DT_INST_PROP(inst, semin), \
			.seup = DT_INST_PROP(inst, seup), \
			.semax = DT_INST_PROP(inst, semax), \
			.sedn = DT_INST_PROP(inst, sedn), \
			.seimin = DT_INST_PROP(inst, seimin), \
		}, \
		.dcstep_enable = DT_INST_PROP(inst, dcstep_enable), \
		.dc_time = DT_INST_PROP(inst, dc_time), \
		.dc_sg = DT_INST_PROP(inst, dc_sg), \
		.vdcmin = DT_INST_PROP(inst, vdcmin), \
		.sg_stop_enable = DT_INST_PROP(inst, sg_stop_enable), \
		.soft_stop_enable = DT_INST_PROP(inst, soft_stop_enable), \
		.external_stepdir_enable = DT_INST_PROP(inst, external_stepdir_enable), \
		.length_step_pulse = DT_INST_PROP(inst, length_step_pulse), \
	}; \
	static struct tmc524x_stepper_driver_data tmc524x_stepper_driver_data_##inst; \
	DEVICE_DT_INST_DEFINE(inst, tmc524x_stepper_driver_init, NULL, \
			      &tmc524x_stepper_driver_data_##inst, \
			      &tmc524x_stepper_driver_config_##inst, POST_KERNEL, \
			      CONFIG_STEPPER_INIT_PRIORITY, &tmc524x_stepper_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TMC524X_STEPPER_DRIVER_DEFINE)

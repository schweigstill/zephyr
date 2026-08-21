/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT adi_tmc524x

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "tmc524x.h"

LOG_MODULE_REGISTER(tmc524x, CONFIG_STEPPER_LOG_LEVEL);

static int tmc524x_reg_write_locked(const struct device *dev, uint8_t reg, uint32_t value)
{
	const struct tmc524x_config *cfg = dev->config;
	struct tmc524x_data *data = dev->data;
	uint8_t tx_buf[5];
	uint8_t rx_buf[5];
	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };
	int ret;

	tx_buf[0] = (reg & 0x7fU) | TMC524X_WRITE_BIT;
	sys_put_be32(value, &tx_buf[1]);

	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret == 0) {
		data->last_spi_status = rx_buf[0];
	}

	return ret;
}

static int tmc524x_reg_read_locked(const struct device *dev, uint8_t reg, uint32_t *value)
{
	const struct tmc524x_config *cfg = dev->config;
	struct tmc524x_data *data = dev->data;
	uint8_t tx_buf[5] = { reg & 0x7fU, 0, 0, 0, 0 };
	uint8_t rx_buf[5] = { 0 };
	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };
	int ret;

	/* TMC SPI reads are pipelined: the first datagram requests the register,
	 * the second datagram clocks out the requested value.
	 */
	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret != 0) {
		return ret;
	}
	data->last_spi_status = rx_buf[0];

	memset(rx_buf, 0, sizeof(rx_buf));
	ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
	if (ret == 0) {
		data->last_spi_status = rx_buf[0];
		*value = sys_get_be32(&rx_buf[1]);
	}

	return ret;
}

int tmc524x_reg_write(const struct device *dev, uint8_t reg, uint32_t value)
{
	struct tmc524x_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = tmc524x_reg_write_locked(dev, reg, value);
	k_mutex_unlock(&data->lock);

	return ret;
}

int tmc524x_reg_read(const struct device *dev, uint8_t reg, uint32_t *value)
{
	struct tmc524x_data *data = dev->data;
	int ret;

	if (value == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = tmc524x_reg_read_locked(dev, reg, value);
	k_mutex_unlock(&data->lock);

	return ret;
}

int tmc524x_reg_update(const struct device *dev, uint8_t reg, uint32_t mask, uint32_t value)
{
	struct tmc524x_data *data = dev->data;
	uint32_t tmp;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = tmc524x_reg_read_locked(dev, reg, &tmp);
	if (ret == 0) {
		tmp = (tmp & ~mask) | (value & mask);
		ret = tmc524x_reg_write_locked(dev, reg, tmp);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

uint8_t tmc524x_get_last_spi_status(const struct device *dev)
{
	struct tmc524x_data *data = dev->data;
	uint8_t status;

	k_mutex_lock(&data->lock, K_FOREVER);
	status = data->last_spi_status;
	k_mutex_unlock(&data->lock);

	return status;
}

uint32_t tmc524x_get_clock_frequency(const struct device *dev)
{
	const struct tmc524x_config *cfg = dev->config;

	return cfg->clock_frequency;
}

int tmc524x_clear_gstat(const struct device *dev, uint32_t mask)
{
	return tmc524x_reg_write(dev, TMC524X_REG_GSTAT, mask);
}

int tmc524x_get_gstat(const struct device *dev, uint32_t *gstat)
{
	return tmc524x_reg_read(dev, TMC524X_REG_GSTAT, gstat);
}

int tmc524x_get_drv_status(const struct device *dev, uint32_t *drv_status)
{
	return tmc524x_reg_read(dev, TMC524X_REG_DRV_STATUS, drv_status);
}

int tmc524x_get_ramp_status(const struct device *dev, uint32_t *ramp_status)
{
	return tmc524x_reg_read(dev, TMC524X_REG_RAMPSTAT, ramp_status);
}

int tmc524x_set_stealthchop2(const struct device *dev, bool enable,
			     const struct tmc524x_pwm_config *pwm)
{
	uint32_t pwmconf = 0;
	int ret;

	if (pwm != NULL) {
		pwmconf = TMC524X_FLD(TMC524X_PWMCONF_PWM_OFS_MASK,
					 TMC524X_PWMCONF_PWM_OFS_SHIFT, pwm->pwm_ofs) |
			  TMC524X_FLD(TMC524X_PWMCONF_PWM_GRAD_MASK,
					 TMC524X_PWMCONF_PWM_GRAD_SHIFT, pwm->pwm_grad) |
			  TMC524X_FLD(TMC524X_PWMCONF_PWM_FREQ_MASK,
					 TMC524X_PWMCONF_PWM_FREQ_SHIFT, pwm->pwm_freq) |
			  TMC524X_FLD(TMC524X_PWMCONF_FREEWHEEL_MASK,
					 TMC524X_PWMCONF_FREEWHEEL_SHIFT, pwm->freewheel) |
			  TMC524X_FLD(TMC524X_PWMCONF_PWM_REG_MASK,
					 TMC524X_PWMCONF_PWM_REG_SHIFT, pwm->pwm_reg) |
			  TMC524X_FLD(TMC524X_PWMCONF_PWM_LIM_MASK,
					 TMC524X_PWMCONF_PWM_LIM_SHIFT, pwm->pwm_lim);
		if (pwm->autoscale) {
			pwmconf |= TMC524X_PWMCONF_PWM_AUTOSCALE;
		}
		if (pwm->autograd) {
			pwmconf |= TMC524X_PWMCONF_PWM_AUTOGRAD;
		}
		ret = tmc524x_reg_write(dev, TMC524X_REG_PWMCONF, pwmconf);
		if (ret != 0) {
			return ret;
		}
	}

	return tmc524x_reg_update(dev, TMC524X_REG_GCONF, TMC524X_GCONF_EN_PWM_MODE,
				       enable ? TMC524X_GCONF_EN_PWM_MODE : 0U);
}

int tmc524x_set_spreadcycle(const struct device *dev)
{
	return tmc524x_reg_update(dev, TMC524X_REG_GCONF, TMC524X_GCONF_EN_PWM_MODE, 0U);
}

int tmc524x_set_coolstep(const struct device *dev,
			 const struct tmc524x_coolstep_config *cfg)
{
	uint32_t value;

	if (cfg == NULL || cfg->semin > 15 || cfg->semax > 15 ||
	    cfg->seup > 3 || cfg->sedn > 3) {
		return -EINVAL;
	}

	value = TMC524X_FLD(TMC524X_COOLCONF_SEMIN_MASK, TMC524X_COOLCONF_SEMIN_SHIFT,
			    cfg->semin) |
		TMC524X_FLD(TMC524X_COOLCONF_SEUP_MASK, TMC524X_COOLCONF_SEUP_SHIFT,
			    cfg->seup) |
		TMC524X_FLD(TMC524X_COOLCONF_SEMAX_MASK, TMC524X_COOLCONF_SEMAX_SHIFT,
			    cfg->semax) |
		TMC524X_FLD(TMC524X_COOLCONF_SEDN_MASK, TMC524X_COOLCONF_SEDN_SHIFT,
			    cfg->sedn);
	if (cfg->seimin) {
		value |= TMC524X_COOLCONF_SEIMIN;
	}

	return tmc524x_reg_update(dev, TMC524X_REG_COOLCONF,
				       TMC524X_COOLCONF_SEMIN_MASK |
				       TMC524X_COOLCONF_SEUP_MASK |
				       TMC524X_COOLCONF_SEMAX_MASK |
				       TMC524X_COOLCONF_SEDN_MASK |
				       TMC524X_COOLCONF_SEIMIN,
				       value);
}

int tmc524x_set_stallguard2(const struct device *dev, int8_t threshold, bool filter_enable)
{
	uint32_t value;

	if (threshold < -64 || threshold > 63) {
		return -EINVAL;
	}

	value = TMC524X_FLD(TMC524X_COOLCONF_SGT_MASK, TMC524X_COOLCONF_SGT_SHIFT,
			    tmc524x_sgt_to_field(threshold));
	if (filter_enable) {
		value |= TMC524X_COOLCONF_SFILT;
	}

	return tmc524x_reg_update(dev, TMC524X_REG_COOLCONF,
				       TMC524X_COOLCONF_SGT_MASK | TMC524X_COOLCONF_SFILT,
				       value);
}

int tmc524x_get_stallguard2_result(const struct device *dev, uint16_t *result)
{
	uint32_t drv_status;
	int ret;

	if (result == NULL) {
		return -EINVAL;
	}

	ret = tmc524x_get_drv_status(dev, &drv_status);
	if (ret != 0) {
		return ret;
	}

	*result = drv_status & TMC524X_DRV_STATUS_SG_RESULT_MASK;
	return 0;
}

int tmc524x_set_stallguard4(const struct device *dev, const struct tmc524x_sg4_config *cfg)
{
	uint32_t value;

	if (cfg == NULL) {
		return -EINVAL;
	}

	value = cfg->threshold;
	if (cfg->filter_enable) {
		value |= TMC524X_SG4_THRS_FILT_EN;
	}
	if (cfg->angle_offset_enable) {
		value |= TMC524X_SG4_THRS_ANGLE_OFFSET;
	}

	return tmc524x_reg_write(dev, TMC524X_REG_SG4_THRS, value);
}

int tmc524x_get_stallguard4_result(const struct device *dev, uint16_t *result)
{
	uint32_t sg4;
	int ret;

	if (result == NULL) {
		return -EINVAL;
	}

	ret = tmc524x_reg_read(dev, TMC524X_REG_SG4_RESULT, &sg4);
	if (ret != 0) {
		return ret;
	}

	*result = sg4 & TMC524X_SG4_RESULT_MASK;
	return 0;
}

int tmc524x_get_adc(const struct device *dev, struct tmc524x_adc_sample *sample)
{
	uint32_t vs_ain;
	uint32_t temp;
	int ret;

	if (sample == NULL) {
		return -EINVAL;
	}

	ret = tmc524x_reg_read(dev, TMC524X_REG_ADC_VSUPPLY_AIN, &vs_ain);
	if (ret != 0) {
		return ret;
	}
	ret = tmc524x_reg_read(dev, TMC524X_REG_ADC_TEMP, &temp);
	if (ret != 0) {
		return ret;
	}

	sample->ain_raw = TMC524X_GET(vs_ain, TMC524X_ADC_AIN_MASK,
				     TMC524X_ADC_AIN_SHIFT);
	sample->vsupply_raw = TMC524X_GET(vs_ain, TMC524X_ADC_VSUPPLY_MASK,
					 TMC524X_ADC_VSUPPLY_SHIFT);
	sample->temp_raw = TMC524X_GET(temp, TMC524X_ADC_TEMP_MASK,
				      TMC524X_ADC_TEMP_SHIFT);
	return 0;
}

int tmc524x_encoder_get_position(const struct device *dev, int32_t *xenc)
{
	uint32_t value;
	int ret;

	if (xenc == NULL) {
		return -EINVAL;
	}
	ret = tmc524x_reg_read(dev, TMC524X_REG_XENC, &value);
	if (ret != 0) {
		return ret;
	}
	*xenc = (int32_t)value;
	return 0;
}

int tmc524x_encoder_set_position(const struct device *dev, int32_t xenc)
{
	return tmc524x_reg_write(dev, TMC524X_REG_XENC, (uint32_t)xenc);
}

int tmc524x_encoder_get_latch(const struct device *dev, int32_t *enc_latch)
{
	uint32_t value;
	int ret;

	if (enc_latch == NULL) {
		return -EINVAL;
	}
	ret = tmc524x_reg_read(dev, TMC524X_REG_ENC_LATCH, &value);
	if (ret != 0) {
		return ret;
	}
	*enc_latch = (int32_t)value;
	return 0;
}

static int tmc524x_init(const struct device *dev)
{
	const struct tmc524x_config *cfg = dev->config;
	struct tmc524x_data *data = dev->data;
	uint32_t ioin;
	int ret;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	k_mutex_init(&data->lock);
	k_mutex_init(&data->callback_dispatch_lock);

	if (cfg->sleep_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->sleep_gpio)) {
			return -ENODEV;
		}
		/* SLEEPN is normally described as active-low. Keep sleep deasserted. */
		ret = gpio_pin_configure_dt(&cfg->sleep_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			return ret;
		}
		/* The TMC5240 needs 2.5 ms after SLEEPN is deasserted. */
		k_busy_wait(3000);
	}

	/* Clear sticky reset/UVLO/driver error flags without hiding later faults. */
	ret = tmc524x_clear_gstat(dev, TMC524X_GSTAT_RESET | TMC524X_GSTAT_DRV_ERR |
				 TMC524X_GSTAT_UV_CP | TMC524X_GSTAT_REGISTER_RESET |
				 TMC524X_GSTAT_VM_UVLO);
	if (ret != 0) {
		return ret;
	}

	ret = tmc524x_reg_read(dev, TMC524X_REG_IOIN, &ioin);
	if (ret != 0) {
		return ret;
	}
	if (TMC524X_GET(ioin, TMC524X_IOIN_VERSION_MASK,
			TMC524X_IOIN_VERSION_SHIFT) != TMC524X_IOIN_VERSION_SUPPORTED) {
		LOG_ERR("unsupported or missing TMC5240 (IOIN=0x%08x)", ioin);
		return -ENODEV;
	}

	return 0;
}

#define TMC524X_DEFINE(inst) \
	BUILD_ASSERT(DT_INST_PROP(inst, clock_frequency) >= 8000000 && \
		     DT_INST_PROP(inst, clock_frequency) <= 20000000, \
		     "TMC524X clock-frequency must be in the supported 8 MHz..20 MHz range"); \
	BUILD_ASSERT(DT_INST_PROP(inst, spi_max_frequency) <= 10000000, \
		     "TMC524X spi-max-frequency must not exceed 10 MHz"); \
	static struct tmc524x_data tmc524x_data_##inst; \
	static const struct tmc524x_config tmc524x_config_##inst = { \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | \
						     SPI_TRANSFER_MSB | SPI_MODE_CPOL | \
						     SPI_MODE_CPHA), \
		.sleep_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, sleep_gpios, {0}), \
		.clock_frequency = DT_INST_PROP(inst, clock_frequency), \
	}; \
	DEVICE_DT_INST_DEFINE(inst, tmc524x_init, NULL, &tmc524x_data_##inst, \
			      &tmc524x_config_##inst, POST_KERNEL, \
			      CONFIG_STEPPER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TMC524X_DEFINE)

/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_STEPPER_TMC524X_H_
#define ZEPHYR_INCLUDE_DRIVERS_STEPPER_TMC524X_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Public TMC524X extension API.
 *
 * The generic Zephyr Stepper APIs intentionally cover only portable functions.
 * This extension API exposes all TMC524X registers and helpers for features
 * that are specific to the IC: StallGuard2/4, StealthChop2, CoolStep, DcStep,
 * ADC, encoder and diagnostics.
 *
 * Unless explicitly stated otherwise, @p dev is the parent device with
 * compatible "adi,tmc524x".
 */

enum tmc524x_reg {
	TMC524X_GCONF = 0x00,
	TMC524X_GSTAT = 0x01,
	TMC524X_IFCNT = 0x02,
	TMC524X_NODECONF = 0x03,
	TMC524X_IOIN = 0x04,
	TMC524X_X_COMPARE = 0x05,
	TMC524X_X_COMPARE_REPEAT = 0x06,
	TMC524X_SHORT_CONF = 0x09,
	TMC524X_DRV_CONF = 0x0A,
	TMC524X_GLOBAL_SCALER = 0x0B,
	TMC524X_IHOLD_IRUN = 0x10,
	TMC524X_TPOWERDOWN = 0x11,
	TMC524X_TSTEP = 0x12,
	TMC524X_TPWMTHRS = 0x13,
	TMC524X_TCOOLTHRS = 0x14,
	TMC524X_THIGH = 0x15,
	TMC524X_RAMPMODE = 0x20,
	TMC524X_XACTUAL = 0x21,
	TMC524X_VACTUAL = 0x22,
	TMC524X_VSTART = 0x23,
	TMC524X_A1 = 0x24,
	TMC524X_V1 = 0x25,
	TMC524X_AMAX = 0x26,
	TMC524X_VMAX = 0x27,
	TMC524X_DMAX = 0x28,
	TMC524X_TVMAX = 0x29,
	TMC524X_D1 = 0x2A,
	TMC524X_VSTOP = 0x2B,
	TMC524X_TZEROWAIT = 0x2C,
	TMC524X_XTARGET = 0x2D,
	TMC524X_V2 = 0x2E,
	TMC524X_A2 = 0x2F,
	TMC524X_D2 = 0x30,
	TMC524X_AACTUAL = 0x31,
	TMC524X_VDCMIN = 0x33,
	TMC524X_SWMODE = 0x34,
	TMC524X_RAMPSTAT = 0x35,
	TMC524X_XLATCH = 0x36,
	TMC524X_ENCMODE = 0x38,
	TMC524X_XENC = 0x39,
	TMC524X_ENC_CONST = 0x3A,
	TMC524X_ENC_STATUS = 0x3B,
	TMC524X_ENC_LATCH = 0x3C,
	TMC524X_ENC_DEVIATION = 0x3D,
	TMC524X_VIRTUAL_STOP_L = 0x3E,
	TMC524X_VIRTUAL_STOP_R = 0x3F,
	TMC524X_ADC_VSUPPLY_AIN = 0x50,
	TMC524X_ADC_TEMP = 0x51,
	TMC524X_OTW_OV_VTH = 0x52,
	TMC524X_MSLUT0 = 0x60,
	TMC524X_MSLUT1 = 0x61,
	TMC524X_MSLUT2 = 0x62,
	TMC524X_MSLUT3 = 0x63,
	TMC524X_MSLUT4 = 0x64,
	TMC524X_MSLUT5 = 0x65,
	TMC524X_MSLUT6 = 0x66,
	TMC524X_MSLUT7 = 0x67,
	TMC524X_MSLUTSEL = 0x68,
	TMC524X_MSLUTSTART = 0x69,
	TMC524X_MSCNT = 0x6A,
	TMC524X_MSCURACT = 0x6B,
	TMC524X_CHOPCONF = 0x6C,
	TMC524X_COOLCONF = 0x6D,
	TMC524X_DCCTRL = 0x6E,
	TMC524X_DRV_STATUS = 0x6F,
	TMC524X_PWMCONF = 0x70,
	TMC524X_PWM_SCALE = 0x71,
	TMC524X_PWM_AUTO = 0x72,
	TMC524X_SG4_THRS = 0x74,
	TMC524X_SG4_RESULT = 0x75,
	TMC524X_SG4_IND = 0x76,
};

struct tmc524x_coolstep_config {
	uint8_t semin;
	uint8_t seup;
	uint8_t semax;
	uint8_t sedn;
	bool seimin;
};

struct tmc524x_pwm_config {
	uint8_t pwm_ofs;
	uint8_t pwm_grad;
	uint8_t pwm_freq;
	uint8_t freewheel;
	uint8_t pwm_reg;
	uint8_t pwm_lim;
	bool autoscale;
	bool autograd;
};

struct tmc524x_sg4_config {
	uint8_t threshold;
	bool filter_enable;
	bool angle_offset_enable;
};

struct tmc524x_adc_sample {
	uint16_t vsupply_raw;
	uint16_t ain_raw;
	uint16_t temp_raw;
};

int tmc524x_reg_read(const struct device *dev, uint8_t reg, uint32_t *value);
int tmc524x_reg_write(const struct device *dev, uint8_t reg, uint32_t value);
int tmc524x_reg_update(const struct device *dev, uint8_t reg, uint32_t mask, uint32_t value);
uint8_t tmc524x_get_last_spi_status(const struct device *dev);
uint32_t tmc524x_get_clock_frequency(const struct device *dev);

int tmc524x_clear_gstat(const struct device *dev, uint32_t mask);
int tmc524x_get_gstat(const struct device *dev, uint32_t *gstat);
int tmc524x_get_drv_status(const struct device *dev, uint32_t *drv_status);
int tmc524x_get_ramp_status(const struct device *dev, uint32_t *ramp_status);

int tmc524x_set_stealthchop2(const struct device *dev, bool enable,
			     const struct tmc524x_pwm_config *pwm);
int tmc524x_set_spreadcycle(const struct device *dev);
int tmc524x_set_coolstep(const struct device *dev,
			 const struct tmc524x_coolstep_config *cfg);
int tmc524x_set_stallguard2(const struct device *dev, int8_t threshold, bool filter_enable);
int tmc524x_get_stallguard2_result(const struct device *dev, uint16_t *result);
int tmc524x_set_stallguard4(const struct device *dev, const struct tmc524x_sg4_config *cfg);
int tmc524x_get_stallguard4_result(const struct device *dev, uint16_t *result);

int tmc524x_get_adc(const struct device *dev, struct tmc524x_adc_sample *sample);
int tmc524x_encoder_get_position(const struct device *dev, int32_t *xenc);
int tmc524x_encoder_set_position(const struct device *dev, int32_t xenc);
int tmc524x_encoder_get_latch(const struct device *dev, int32_t *enc_latch);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_STEPPER_TMC524X_H_ */

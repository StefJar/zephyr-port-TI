/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_adpd188bi

#include <stddef.h>

#include <kernel.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/byteorder.h>

#include "smokeDetector.h"
#include "adpd188bi2regs.h"

struct adi_adpd188bi_config {
	const char * i2c_port;
	const uint16_t i2c_addr;
	const struct gpio_dt_spec gpios_gpio0;
};

struct adi_adpd188bi_runtime {
	const struct device * i2cDev;
	uint16_t devVer;
};

#define DEV_CFG(dev)                                     \
	((const struct adi_adpd188bi_config *const)     \
	(dev)->config)

#define DEV_DATA(dev)					 \
	((struct adi_adpd188bi_runtime *const)          \
	(dev)->data)

static int adpd188bi_reg_read(const struct device *dev, uint8_t reg_addr, uint16_t *reg_val) {
	if (i2c_burst_read(DEV_DATA(dev)->i2cDev, DEV_CFG(dev)->i2c_addr, reg_addr, (uint8_t *) reg_val, 2) < 0) {
			return -EIO;
	}
	*reg_val = sys_be16_to_cpu(*reg_val);
	return 0;
}

static int adpd188bi_burst_read(const struct device *dev, uint8_t reg_addr, uint8_t * buffer, const uint32_t NBytes) {
	if (i2c_burst_read(DEV_DATA(dev)->i2cDev, DEV_CFG(dev)->i2c_addr, reg_addr, (uint8_t *) buffer, NBytes) < 0) {
			return -EIO;
	}
	return 0;
}


static int adpd188bi_reg_write(const struct device *dev, const uint8_t reg_addr, const uint16_t reg_val) {
	const uint8_t buf[3] = {reg_addr, reg_val >> 8, reg_val & 0xFF};

	return i2c_write(DEV_DATA(dev)->i2cDev, buf, sizeof(buf), DEV_CFG(dev)->i2c_addr);
}

/**
 * @brief Get the mode of operation of the ADPD188.
 * @param dev - The ADPD188 descriptor.
 * @param mode - The cur mode of operation. Values can be:
 *                   - ADPD188BI_STANDBY;
 *                   - ADPD188BI_PROGRAM;
 *                   - ADPD188BI_NORMAL.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_mode_get(const struct device *dev, enum adpd188bi_mode *mode) {
	uint16_t data;

	if (adpd188bi_reg_read(dev, ADPD188BI_REG_MODE, &data) < 0){
		return false;
	}

	*mode = data & ADPD188BI_MODE_MODE_MASK;

	return true;
}

/**
 * @brief Set the mode of operation of the ADPD188.
 * @param dev - The ADPD188 descriptor.
 * @param new_mode - The new mode of operation. Values can be:
 *                   - ADPD188BI_STANDBY;
 *                   - ADPD188BI_PROGRAM;
 *                   - ADPD188BI_NORMAL.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_mode_set(const struct device *dev, enum adpd188bi_mode new_mode) {
	uint16_t data;

	data = new_mode & ADPD188BI_MODE_MODE_MASK;

	return (adpd188bi_reg_write(dev, ADPD188BI_REG_MODE, data) < 0) ? false : true;
}

bool adpd188bi_data_access_ctl(const struct device *dev, const bool slotAdataHold, const bool slotBdataHold, const bool digitalClkEnable) {
	const uint16_t reg_data =
		((true == slotAdataHold) ? ADPD188BI_DATA_ACCESS_CTL_SLOTA_DATA_HOLD_MASK : 0) |
		((true == slotBdataHold) ? ADPD188BI_DATA_ACCESS_CTL_SLOTB_DATA_HOLD_MASK : 0) |
		((true == digitalClkEnable) ? ADPD188BI_DATA_ACCESS_CTL_DIGITAL_CLOCK_ENA_MASK : 0);

	return (adpd188bi_reg_write(dev, ADPD188BI_REG_DATA_ACCESS_CTL, reg_data) < 0) ? false : true;
}


/**
 * @brief Get the number of bytes currently present in FIFO.
 * @param dev - The ADPD188 descriptor.
 * @param bytes_no - Number of bytes in the FIFO.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_fifo_status_get(const struct device *dev, uint8_t *bytes_no) {
	uint16_t reg_data = 0;

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_STATUS, &reg_data) < 0) {
		return false;
	}

	// get MSB that is the bytes number
	reg_data >>= 8;
	reg_data &= 0xFF;
	*bytes_no = reg_data;

	return true;
}

/**
 * @brief Empty the FIFO.
 * @param dev - The ADPD188 descriptor.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_fifo_clear(const struct device *dev) {
	uint16_t reg_data;

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_STATUS, &reg_data) < 0) {
		return false;
	}
	// clear slot A & B intr by writing a 1 to bit 5 and 6
	reg_data |= ADPD188BI_STATUS_SLOTA_INT_MASK | ADPD188BI_STATUS_SLOTB_INT_MASK;

	return (adpd188bi_reg_write(dev, ADPD188BI_REG_STATUS, reg_data) < 0) ? false : true;
}

/**
 * @brief Set the number of 16 bit words that need to be in the FIFO to trigger
 *        an interrupt.
 * @param dev - The ADPD188 descriptor.
 * @param word_no - Number of words that trigger an interrupt.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_fifo_thresh_set(const struct device *dev, uint8_t word_no) {
	uint16_t reg_data;

	if(word_no > ADPD188BI_FIFO_THRESH_MAX_THRESHOLD) {
		return false;
	}

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_FIFO_THRESH, &reg_data) < 0) {
		return false;
	}
	reg_data &= ~ADPD188BI_FIFO_THRESH_FIFO_THRESH_MASK;
	reg_data |= (word_no << ADPD188BI_FIFO_THRESH_FIFO_THRESH_POS) & ADPD188BI_FIFO_THRESH_FIFO_THRESH_MASK;

	return (adpd188bi_reg_write(dev, ADPD188BI_REG_FIFO_THRESH, reg_data) < 0) ? false : true;
}

bool adpd188bi_fifo_read(const struct device *dev, uint32_t * buffer, const uint32_t NBytes) {
	if(adpd188bi_burst_read(dev, ADPD188BI_REG_FIFO_ACCESS, (uint8_t *)&buffer[0], NBytes) != 0) {
		return false;
	}
	return true;
}

/**
 * @brief Get the slot and FIFO interrupt flags.
 * @param dev - The ADPD188 descriptor.
 * @param flags - The ORed value of the flags. If no interrupt has triggered the
 *                value will be zero. Otherwise it can be a logical OR between:
 *                - ADPD188BI_SLOTA_INT;
 *                - ADPD188BI_SLOTB_INT;
 *                - ADPD188BI_FIFO_INT.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_interrupt_get(const struct device *dev, uint8_t *flags) {
	uint16_t reg_data;

	*flags = 0;

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_STATUS, &reg_data) < 0) {
		return false;
	}
	if(reg_data & ADPD188BI_STATUS_SLOTA_INT_MASK) *flags |= ADPD188BI_SLOTA_INT;
	if(reg_data & ADPD188BI_STATUS_SLOTB_INT_MASK) *flags |= ADPD188BI_SLOTB_INT;

	return true;
}

/**
 * @brief Clear the slot and FIFO interrupt flags.
 * @param dev - The ADPD188 descriptor.
 * @param flags - The ORed value of the flags that need to be cleared. If no
 *                interrupt needs to be cleared the value will be zero.
 *                Otherwise do a logical OR between the flags that need to be
 *                cleared:
 *                - ADPD188BI_SLOTA_INT;
 *                - ADPD188BI_SLOTB_INT;
 *                - ADPD188BI_FIFO_INT.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_interrupt_clear(const struct device *dev, uint8_t flags) {
	uint16_t reg_data;

	if(0 == flags) return true;

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_STATUS, &reg_data) < 0) {
		return false;
	}

	/*
	 * If an interrupt is not to be cleared, but happens to be asserted write 0
	 * to that spot to not clear it unintentionally.
	 */
	if(flags & ADPD188BI_SLOTA_INT) {
		reg_data |= ADPD188BI_STATUS_SLOTA_INT_MASK;
	} else {
		reg_data &= ~ADPD188BI_STATUS_SLOTA_INT_MASK;
	}
	if(flags & ADPD188BI_SLOTB_INT) {
		reg_data |= ADPD188BI_STATUS_SLOTB_INT_MASK;
	} else {
		reg_data &= ~ADPD188BI_STATUS_SLOTB_INT_MASK;
	}

	return (adpd188bi_reg_write(dev, ADPD188BI_REG_STATUS, reg_data) < 0) ? false : true;
}

/**
 * @brief Enable the slot and FIFO interrupt flags.
 * @param dev - The ADPD188 descriptor.
 * @param flags - The ORed value of the flags that need to be enabled. If no
 *                interrupt needs to be enabled the value will be zero.
 *                Otherwise do a logical OR between the flags that need to be
 *                enabled:
 *                - ADPD188BI_SLOTA_INT;
 *                - ADPD188BI_SLOTB_INT;
 *                - ADPD188BI_FIFO_INT.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_interrupt_en(const struct device *dev, uint8_t flags)
{
	uint16_t reg_data;
	if(adpd188bi_reg_read(dev, ADPD188BI_REG_INT_MASK, &reg_data) < 0) {
		return false;
	}
	if(flags & ADPD188BI_SLOTA_INT) reg_data &= ~ADPD188BI_INT_MASK_SLOTA_INT_MASK_MASK;
	if(flags & ADPD188BI_SLOTB_INT) reg_data &= ~ADPD188BI_INT_MASK_SLOTB_INT_MASK_MASK;
	if(flags & ADPD188BI_FIFO_INT) reg_data &= ~ADPD188BI_INT_MASK_FIFO_INT_MASK_MASK;

	return (adpd188bi_reg_write(dev, ADPD188BI_REG_INT_MASK, reg_data) < 0) ? false : true;
}

/**
 * @brief Setup drive and polarity of the GPIOs. Also enable GPIO if necessary.
 * @param dev - The ADPD188 descriptor.
 * @param config - Configuration structure of the GPIO.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_gpio_setup(const struct device *dev, struct adpd188bi_gpio_config config) {
	uint16_t reg_data;

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_GPIO_DRV, &reg_data) < 0) {
		return false;
	}
	if(config.gpio_id == 0) {
		if(config.gpio_pol)
			reg_data |= ADPD188BI_GPIO_DRV_GPIO0_POL_MASK;
		else
			reg_data &= ~ADPD188BI_GPIO_DRV_GPIO0_POL_MASK;
		if(config.gpio_drv)
			reg_data |= ADPD188BI_GPIO_DRV_GPIO0_DRV_MASK;
		else
			reg_data &= ~ADPD188BI_GPIO_DRV_GPIO0_DRV_MASK;
		if(config.gpio_en)
			reg_data |= ADPD188BI_GPIO_DRV_GPIO0_ENA_MASK;
		else
			reg_data &= ~ADPD188BI_GPIO_DRV_GPIO0_ENA_MASK;
	} else if(config.gpio_id == 1) {
		if(config.gpio_pol)
			reg_data |= ADPD188BI_GPIO_DRV_GPIO1_POL_MASK;
		else
			reg_data &= ~ADPD188BI_GPIO_DRV_GPIO1_POL_MASK;
		if(config.gpio_drv)
			reg_data |= ADPD188BI_GPIO_DRV_GPIO1_DRV_MASK;
		else
			reg_data &= ~ADPD188BI_GPIO_DRV_GPIO1_DRV_MASK;
	}

	return (adpd188bi_reg_write(dev, ADPD188BI_REG_GPIO_DRV, reg_data) < 0) ? false : true;
}

/**
 * @brief Setup the GPIO source.
 * @param dev - The ADPD188 descriptor.
 * @param gpio_id - ID of the GPIO (0 or 1).
 * @param config - ID of the source of the GPIO.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_gpio_alt_setup(const struct device *dev, uint8_t gpio_id, enum adpd188bi_gpio_alt_config config) {
	uint16_t reg_data;

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_GPIO_CTRL, &reg_data) < 0) {
		return false;
	}

	switch (gpio_id) {
		case 0:
			reg_data &= ~ADPD188BI_GPIO_CTRL_GPIO0_ALT_CFG_MASK;
			reg_data |= (config << ADPD188BI_GPIO_CTRL_GPIO0_ALT_CFG_POS) & ADPD188BI_GPIO_CTRL_GPIO0_ALT_CFG_MASK;
			break;
		case 1:
			reg_data &= ~ADPD188BI_GPIO_CTRL_GPIO1_ALT_CFG_MASK;
			reg_data |= (config << ADPD188BI_GPIO_CTRL_GPIO1_ALT_CFG_POS) & ADPD188BI_GPIO_CTRL_GPIO1_ALT_CFG_MASK;
			break;
		default:
			return false;
	}

	return (adpd188bi_reg_write(dev, ADPD188BI_REG_GPIO_CTRL, reg_data) < 0) ? false : true;
}

/**
 * @brief Do software reset of the device.
 * @param dev - The ADPD188 descriptor.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_sw_reset(const struct device *dev) {
	return (adpd188bi_reg_write(dev, ADPD188BI_REG_SW_RESET, 0x1) < 0) ? false : true;
}

/**
 * @brief Do internal 32MHz clock calibration. This calibration requires the
 *        32kHz clock to be calibrated first. The 32kHz calibration needs an
 *        external reference.
 * @param dev - The ADPD188 descriptor.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_clk32mhz_cal(const struct device *dev) {
	uint16_t reg_data;
	float clk_error;

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_DATA_ACCESS_CTL, &reg_data) < 0) {
		return false;
	}
	reg_data |= ADPD188BI_DATA_ACCESS_CTL_DIGITAL_CLOCK_ENA_MASK;

	if(adpd188bi_reg_write(dev, ADPD188BI_REG_DATA_ACCESS_CTL, reg_data) < 0) {
		return false;
	}

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_CLK32M_CAL_EN, &reg_data) < 0) {
		return false;
	}
	reg_data |= ADPD188BI_CLK32M_CAL_EN_CLK32M_CAL_EN_MASK;

	if(adpd188bi_reg_write(dev, ADPD188BI_REG_CLK32M_CAL_EN, reg_data) < 0) {
		return false;
	}
	k_sleep(K_MSEC(1));

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_CLK_RATIO, &reg_data) < 0) {
		return false;
	}
	reg_data &= ADPD188BI_CLK_RATIO_CLK_RATIO_MASK;

	clk_error = 32000000.0 * (1.0 - (float)reg_data/2000.0);
	switch(DEV_DATA(dev)->devVer) {
		case ADPD188BI_DEVICE_ID:
			reg_data = clk_error / 109000;
			break;
		case ADPD108X_DEVICE_ID:
			reg_data = clk_error / 112000;
			break;
		default:
			return false;
	}

	reg_data &= ADPD188BI_CLK32M_ADJUST_CLK32M_ADJUST_MASK;

	if(adpd188bi_reg_write(dev, ADPD188BI_REG_CLK32M_ADJUST, reg_data) < 0) {
		return false;
	}

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_CLK32M_CAL_EN, &reg_data) < 0) {
		return false;
	}
	reg_data &= ~ADPD188BI_CLK32M_CAL_EN_CLK32M_CAL_EN_MASK;

	if(adpd188bi_reg_write(dev, ADPD188BI_REG_CLK32M_CAL_EN, reg_data) < 0) {
		return false;
	}

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_DATA_ACCESS_CTL, &reg_data) < 0) {
		return false;
	}
	reg_data &= ~ADPD188BI_DATA_ACCESS_CTL_DIGITAL_CLOCK_ENA_MASK;

	return (adpd188bi_reg_write(dev, ADPD188BI_REG_DATA_ACCESS_CTL, reg_data) < 0) ? false : true;
}

/**
 * @brief Enable slot and setup its FIFO interaction.
 * @param dev - The ADPD188 descriptor.
 * @param config - Configuration structure for the slot.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_slot_setup(const struct device *dev, struct adpd188bi_slot_config config) {
	uint16_t reg_data;

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_SLOT_EN, &reg_data) < 0) {
		return false;
	}

	if(config.slot_id == ADPD188BI_SLOTA) {
		reg_data &= ~ADPD188BI_SLOT_EN_SLOTA_EN_MASK;
		reg_data |= (config.slot_en << ADPD188BI_SLOT_EN_SLOTA_EN_POS) & ADPD188BI_SLOT_EN_SLOTA_EN_MASK;
		reg_data &= ~ADPD188BI_SLOT_EN_SLOTA_FIFO_MODE_MASK;
		reg_data |= (config.sot_fifo_mode << ADPD188BI_SLOT_EN_SLOTA_FIFO_MODE_POS) & ADPD188BI_SLOT_EN_SLOTA_FIFO_MODE_POS;
	} else if(config.slot_id == ADPD188BI_SLOTB) {
		reg_data &= ~ADPD188BI_SLOT_EN_SLOTB_EN_MASK;
		reg_data |= (config.slot_en << ADPD188BI_SLOT_EN_SLOTB_EN_POS) & ADPD188BI_SLOT_EN_SLOTB_EN_MASK;
		reg_data &= ~ADPD188BI_SLOT_EN_SLOTB_FIFO_MODE_MASK;
		reg_data |= (config.sot_fifo_mode << ADPD188BI_SLOT_EN_SLOTB_FIFO_MODE_POS) & ADPD188BI_SLOT_EN_SLOTB_FIFO_MODE_POS;
	}

	return (adpd188bi_reg_write(dev, ADPD188BI_REG_SLOT_EN, reg_data) < 0) ? false : true;
}

/**
 * @brief Set sample frequency of the ADC.
 * @param dev - The ADPD188 descriptor.
 * @param freq_hz - Desired ADC sample frequency.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_adc_fsample_set(const struct device *dev, float freq_hz) {
	if(freq_hz > 2000) {
		return false;
	}

	const uint16_t reg_data = 32000 / (freq_hz * 4);

	return (adpd188bi_reg_write(dev, ADPD188BI_REG_FSAMPLE, reg_data) < 0) ? false : true;
}

/**
 * @brief Get sample frequency of the ADC.
 * @param dev - The ADPD188 descriptor.
 * @param freq_hz - ADC sample frequency.
 * @return true in case of success, false otherwise.
 */
bool adpd188bi_adc_fsample_get(const struct device *dev, float *freq_hz) {
	uint16_t reg_data;

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_FSAMPLE, &reg_data) < 0) {
		return false;
	}

	*freq_hz = 32000.0 / (float)(reg_data * 4);

	return true;
}

/**
 * @brief      Set averaging factor.
 * @param[in]  ADPD188BI_t *handle - Pointer to instance of ADPD188BI.
 * @param[in]  ADPD188BI_slot_t slot - Target time slot.
 * @param[in]  ADPD188BI_averageFactor_t avg - The averaging factor.
 * @retval     bool - Execution status. 'true' - if success, 'false' - otherwise.
 */
bool adpd188bi_adc_avgFactor(const struct device *dev, enum adpd188bi_slots slot, adpd188bi_averageFactor_t avg) {

	uint16_t l_regValue = 0;
    const uint16_t l_clearMaskA = 0x0070;
    const uint16_t l_clearMaskB = 0x0700;

    if(!adpd188bi_reg_read(dev, ADPD188BI_REG_NUM_AVG, &l_regValue)) {
        return false;
    }

    if(slot == ADPD188BI_SLOTA) {
        l_regValue &= ~(l_clearMaskA);
        l_regValue |= (avg << 4);
    }
    else if(slot == ADPD188BI_SLOTB) {
        l_regValue &= ~(l_clearMaskB);
        l_regValue |= (avg << 8);
    }

    return adpd188bi_reg_write(dev, ADPD188BI_REG_NUM_AVG, l_regValue) ? false : true;
}

static const uint32_t adpd188bi_smoke_detector_cfg [] = {
		0x1130A9, // Writes a 32-bit sum to the FIFO for Time Slot A and Time Slot B
//			0x120200, // 16 Hz sampling rate
		0x14011D, // Blue Slot A, IR Slot B, combine PDs
//			0x150770, // No decimation (average over 128 samples)
		0x170009, // Time Slot A chop mode, inverted, noninverted, noninverted, inverted (see the Improving SNR Using Integrator Chopping section for more information)
		0x180000, // No ADC offset
		0x193FFF, // Unused channel
		0x1A3FFF, // Unused channel
		0x1B3FFF, // Unused channel
		0x1D0009, // Time Slot B chop mode (inverted, noninverted, noninverted, inverted)
		0x1E0000, // No ADC offset
		0x1F3FFF, // Unused channel
		0x203FFF, // Unused channel
		0x213FFF, // Unused channel
		0x223539, // LED3 IR
		0x233536, // LED1 blue
		0x241530, // LED2 unused
		0x25630C, // Default LED drive trim
		0x300320, // 3 μs LED pulse
		0x31040E, // Four pulses, 15 μs LED offset
		0x350320, // 3 μs LED pulse
		0x36040E, // Four pulses, 15 μs LED offset
		0x3922F0, // Integrator timing
		0x3B22F0, // Integrator timing
		0x3C31C6, // Power down Channel 2, Channel 3, and Channel 4
		0x421C34, // 200k TIA gain
		0x43ADA5, // Signal path configuration
		0x441C34, // 200k TIA gain
		0x45ADA5, // Signal path configuration
		0x580544, // Math for chop mode inverted, noninverted, noninverted, inverted LED
		0x540AA0, // PD reverse bias, approximately 250 mV
};

bool adpd188bi_smoke_detect_setup(const struct device *dev) {

	for(unsigned int i = 0; i<ARRAY_SIZE(adpd188bi_smoke_detector_cfg); i++){
		const uint32_t regAddr = (adpd188bi_smoke_detector_cfg[i] >> 16) & 0xFF;
		const uint32_t regData = adpd188bi_smoke_detector_cfg[i] & 0xFFFF;

		if (adpd188bi_reg_write(dev, (const uint8_t)regAddr, (const uint16_t)regData)) {
		  return false;
		}
	}
	return true;
}

static bool adi_adpd188bi_checkDev(const struct device *dev) {
	if(adpd188bi_reg_read(dev, ADPD188BI_REG_DEVID, &(DEV_DATA(dev)->devVer)) < 0) {
		return false;
	}

	switch (DEV_DATA(dev)->devVer) {
		case ADPD188BI_DEVICE_ID:
			break;
		case ADPD108X_DEVICE_ID:
			break;
		default:
			return false;
	}
	if (false == adpd188bi_sw_reset(dev)) {
		return false;
	}

	return true;
}

bool adpd188bi_sample_clk(const struct device *dev, const bool enable) {
	uint16_t regVal = 0;

	if(adpd188bi_reg_read(dev, ADPD188BI_REG_SAMPLE_CLK, &regVal) < 0) {
		return false;
	}

	if (true == enable) {
		regVal |= ADPD188BI_SAMPLE_CLK_CLK32K_EN_MASK;
	} else {
		regVal &= ~(ADPD188BI_SAMPLE_CLK_CLK32K_EN_MASK);
	}

	if (adpd188bi_reg_write(dev, ADPD188BI_REG_SAMPLE_CLK, (const uint16_t)regVal)) {
	  return false;
	}
	return true;
}

static bool adi_adpd188bi_calibrate(const struct device *dev) {
	return adpd188bi_clk32mhz_cal(dev);
}

static bool adi_adpd188bi_start(const struct device *dev) {
	// clk the IC
	adpd188bi_mode_set(dev, ADPD188BI_PROGRAM);
	adpd188bi_sample_clk(dev, true);

	// clear the fifo
	adpd188bi_mode_set(dev, ADPD188BI_STANDBY);
	adpd188bi_data_access_ctl(dev, false, false, true);
	adpd188bi_fifo_clear(dev);
	adpd188bi_data_access_ctl(dev, false, false, false);

	// setup the IC as smoke detector
	adpd188bi_mode_set(dev, ADPD188BI_PROGRAM);
	// smoke detector setup
	adpd188bi_smoke_detect_setup(dev);

	// sampling freq 16Hz (0x200 reg value)
	adpd188bi_adc_fsample_set(dev, 16);

	// average over 128 samples
	adpd188bi_adc_avgFactor(dev, ADPD188BI_SLOTA, ADPD188BI_NUM_AVG_128);
	adpd188bi_adc_avgFactor(dev, ADPD188BI_SLOTB, ADPD188BI_NUM_AVG_128);

	adpd188bi_mode_set(dev, ADPD188BI_NORMAL);

	return true;
}

bool adi_adpd188bi_dev_edgeISR (const struct device *dev) {
	return 1 == gpio_pin_get_dt(&(DEV_CFG(dev)->gpios_gpio0)) ? true : false;
}

uint32_t adi_adpd188bi_getSamplesInFIFO(const struct device *dev) {
	uint8_t NBytes = 0;
	adpd188bi_fifo_status_get(dev, &NBytes);
	return NBytes / 4;
}

bool adi_adpd188bi_readSamplesFromFIFO(const struct device *dev, uint32_t * buffer, const uint32_t Nsamples) {
	if (false == adpd188bi_fifo_read(dev, buffer, Nsamples*4)) return false;
	for (uint32_t i = 0; i < Nsamples; i++) {
		uint8_t * pRS = &buffer[i];
		const uint32_t sample = pRS[1] + (pRS[0] << 8) + (pRS[3] << 16) +  (pRS[2] << 24);
		buffer[i] = sample;
	}
	adpd188bi_interrupt_clear(dev,ADPD188BI_SLOTA_INT | ADPD188BI_FIFO_INT);
	return true;
}

static int adi_adpd188bi_dev_init(const struct device *dev){
	DEV_DATA(dev)->i2cDev = device_get_binding(DEV_CFG(dev)->i2c_port);
	if (NULL == DEV_DATA(dev)->i2cDev) {
		return -ENODEV;
	}
	gpio_pin_configure_dt(&(DEV_CFG(dev)->gpios_gpio0), GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&DEV_CFG(dev)->gpios_gpio0, GPIO_INT_EDGE_RISING);

	DEV_DATA(dev)->devVer = 0;

	return 0;
}

static const struct smokeDetector_driver_api adi_adpd188bi_driver_api = {
	.checkDev = adi_adpd188bi_checkDev,
	.calibrate = adi_adpd188bi_calibrate,
	.start = adi_adpd188bi_start,
	.edgeISR = adi_adpd188bi_dev_edgeISR,
	.getSamplesInFIFO = adi_adpd188bi_getSamplesInFIFO,
	.readSamplesFromFIFO = adi_adpd188bi_readSamplesFromFIFO,
};

#define ADI_ADPD_188BI_DEVICE(n)											\
	static const struct adi_adpd188bi_config adi_adpd188bi_## n ##_cfg={	\
		.i2c_port = DT_INST_BUS_LABEL(n),									\
		.i2c_addr = DT_INST_REG_ADDR(n),									\
		.gpios_gpio0 = GPIO_DT_SPEC_INST_GET(n, gpio0_gpios),				\
	};																		\
	static struct adi_adpd188bi_runtime adi_adpd188bi_## n ##_runtime;		\
	DEVICE_DT_INST_DEFINE(n,												\
		adi_adpd188bi_dev_init,												\
		NULL,																\
		&adi_adpd188bi_## n ##_runtime,										\
		&adi_adpd188bi_## n ##_cfg,											\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,					\
		&adi_adpd188bi_driver_api); 										\


DT_INST_FOREACH_STATUS_OKAY(ADI_ADPD_188BI_DEVICE)

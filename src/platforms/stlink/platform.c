/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the ST-Link implementation. */

#include "general.h"
#include "platform.h"
#include "usb.h"
#include "aux_serial.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/adc.h>

uint16_t led_idle_run;
uint16_t nrst_pin;
static uint32_t rev;
static void adc_init(void);

int platform_hwversion(void)
{
	return rev;
}

void platform_init(void)
{
	rev = detect_rev();
	SCS_DEMCR |= SCS_DEMCR_VC_MON_EN;
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
#ifdef BLUEPILL
	led_idle_run = GPIO13;
	nrst_pin = NRST_PIN_V1;
#else
	switch (rev) {
	case 0:
		led_idle_run = GPIO8;
		nrst_pin = NRST_PIN_V1;
		break;
	case 0x101:
		led_idle_run = GPIO9;
		nrst_pin = NRST_PIN_CLONE;
		break;
	default:
		led_idle_run = GPIO9;
		nrst_pin = NRST_PIN_V2;
		break;
	}
#endif
	/* Setup GPIO ports */
	gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_INPUT_FLOAT, TMS_PIN);
	gpio_set_mode(TCK_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TCK_PIN);
	gpio_set_mode(TDI_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TDI_PIN);

	platform_nrst_set_val(false);

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, led_idle_run);

	/* Relocate interrupt vector table here */
	extern uint32_t vector_table;
	SCB_VTOR = (uintptr_t)&vector_table;

	platform_timing_init();
	/*
	 * The ST bootloader leaves the USB peripheral already initialised/enumerated
	 * (as itself, in DFU mode) when it jumps to the application. Without forcing
	 * a real electrical disconnect here, the host has no reason to notice the
	 * handoff and keeps treating us as the bootloader it already saw, so it
	 * never queries us for our actual (BMP) descriptors. This is lujji's fix
	 * from "Installing Black Magic via ST-Link bootloader": reset the USB
	 * peripheral, then directly force D+ (PA12) low as a plain GPIO long enough
	 * for the host to register a disconnect, before handing the pin back to the
	 * USB peripheral's own control. Done here, after the clock tree is fully
	 * configured, rather than before it.
	 */
	if ((rev & 0xff) > 1U) /* Reconnect USB (genuine-hardware path, board-specific) */
		gpio_set(GPIOA, GPIO15);
	rcc_periph_reset_pulse(RST_USB);
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_clear(GPIOA, GPIO12);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, GPIO12);
	/* Hold D+ low long enough for the host to register a real disconnect.
	 * Clock tree is now known/stable (72MHz), so this duration is well-defined. */
	for (volatile uint32_t i = 0; i < 200000U; ++i)
		continue;
	/* Release D+ back to floating/input so the USB peripheral's own transceiver
	 * regains control of the line once blackmagic_usb_init() brings it up. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO12);
	blackmagic_usb_init();

#ifdef SWIM_AS_UART
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_FULL_SWJ, AFIO_MAPR_USART1_REMAP);
#endif

	/* Don't enable UART if we're being debugged. */
	if (!(SCS_DEMCR & SCS_DEMCR_TRCENA))
		aux_serial_init();
	adc_init();
}

void platform_nrst_set_val(bool assert)
{
	if (assert) {
		gpio_set_mode(NRST_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, nrst_pin);
		gpio_clear(NRST_PORT, nrst_pin);
	} else {
		gpio_set_mode(NRST_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, nrst_pin);
		gpio_set(NRST_PORT, nrst_pin);
	}
}

bool platform_nrst_get_val()
{
	return gpio_get(NRST_PORT, nrst_pin) == 0;
}

static void adc_init(void)
{
	rcc_periph_clock_enable(RCC_ADC1);

	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);

	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);
	adc_enable_temperature_sensor();
	adc_power_on(ADC1);

	/* Wait for the ADC to finish starting up */
	for (volatile size_t i = 0; i < 800000U; ++i)
		continue;

	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

const char *platform_target_voltage(void)
{
	static char ret[6] = "0.00V";
	const uint8_t channel = 0;
	adc_set_regular_sequence(ADC1, 1, (uint8_t *)&channel);
	adc_start_conversion_direct(ADC1);
	/* Wait for end of conversion. */
	while (!adc_eoc(ADC1))
		continue;
	uint32_t platform_adc_value = adc_read_regular(ADC1);

	const uint8_t ref_channel = 17;
	adc_set_regular_sequence(ADC1, 1, (uint8_t *)&ref_channel);
	adc_start_conversion_direct(ADC1);
	/* Wait for end of conversion. */
	while (!adc_eoc(ADC1))
		continue;
	uint32_t vrefint_value = adc_read_regular(ADC1);

	/* Value in millivolts */
	uint32_t val = (platform_adc_value * 2400U) / vrefint_value;
	ret[0] = '0' + val / 1000U;
	ret[2] = '0' + (val / 100U) % 10U;
	ret[3] = '0' + (val / 10U) % 10U;

	return ret;
}

void platform_target_clk_output_enable(const bool enable)
{
	(void)enable;
}

bool platform_spi_init(const spi_bus_e bus)
{
	(void)bus;
	return false;
}

bool platform_spi_deinit(const spi_bus_e bus)
{
	(void)bus;
	return false;
}

bool platform_spi_chip_select(const uint8_t device_select)
{
	(void)device_select;
	return false;
}

uint8_t platform_spi_xfer(const spi_bus_e bus, const uint8_t value)
{
	(void)bus;
	return value;
}

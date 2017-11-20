/*
 * Copyright (c) 2017 I-SENSE group of ICCS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <pinmux.h>
#include <sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

/* pin assignments for STM32F3DISCOVERY board */
static const struct pin_config pinconf[] = {
#ifdef CONFIG_UART_STM32_PORT_1
	{STM32_PIN_PC4, STM32F3_PINMUX_FUNC_PC4_USART1_TX},
	{STM32_PIN_PC5, STM32F3_PINMUX_FUNC_PC5_USART1_RX},
#endif	/* CONFIG_UART_STM32_PORT_1 */
#ifdef CONFIG_UART_STM32_PORT_2
	{STM32_PIN_PA2, STM32F3_PINMUX_FUNC_PA2_USART2_TX},
	{STM32_PIN_PA3, STM32F3_PINMUX_FUNC_PA3_USART2_RX},
#endif	/* CONFIG_UART_STM32_PORT_2 */
#ifdef CONFIG_I2C_1
	{STM32_PIN_PB6, STM32F3_PINMUX_FUNC_PB6_I2C1_SCL},
	{STM32_PIN_PB7, STM32F3_PINMUX_FUNC_PB7_I2C1_SDA},
#endif /* CONFIG_I2C_1 */
#ifdef CONFIG_I2C_2
	{STM32_PIN_PF0, STM32F3_PINMUX_FUNC_PF0_I2C2_SDA},
	{STM32_PIN_PF1, STM32F3_PINMUX_FUNC_PF1_I2C2_SCL},
#endif /* CONFIG_I2C_2 */
#ifdef CONFIG_USB_DC_STM32
	{STM32_PIN_PA11, STM32F3_PINMUX_FUNC_PA11_USB_DM},
	{STM32_PIN_PA12, STM32F3_PINMUX_FUNC_PA12_USB_DP},
#endif	/* CONFIG_USB_DC_STM */
};

static int pinmux_stm32_init(struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1,
		CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);

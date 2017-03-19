/*
 * linux/arch/mips/jz4740/board-noah_np1380.c
 *
 * QI_LB60 board support
 *
 * Copyright (c) 2009 Qi Hardware inc.,
 * Author: Xiangfu Liu <xiangfu@qi-hardware.com>
 * Copyright 2010, Lars-Peter Clausen <lars@metafoo.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or later
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/gpio/machine.h>

#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/input/matrix_keypad.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/power_supply.h>
#include <linux/power/jz4740-battery.h>
#include <linux/power/gpio-charger.h>
#include <linux/pwm.h>

#include <asm/mach-jz4740/gpio.h>
#include <asm/mach-jz4740/jz4740_fb.h>
#include <asm/mach-jz4740/jz4740_mmc.h>
#include <asm/mach-jz4740/jz4740_nand.h>

#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

#include <asm/mach-jz4740/platform.h>

#include "clock.h"

/* GPIOs */
#define NOAH_NP1380_GPIO_SD_CD		JZ_GPIO_PORTD(0)
#define NOAH_NP1380_GPIO_SD_VCC_EN_N	JZ_GPIO_PORTD(2)

/* NAND */

/* Early prototypes of the QI LB60 had only 1GB of NAND.
 * In order to support these devices as well the partition and ecc layout is
 * initialized depending on the NAND size */
static struct mtd_partition noah_np1380_partitions_1gb[] = {
	{
		.name = "NAND BOOT partition",
		.offset = 0 * 0x100000,
		.size = 4 * 0x100000,
	},
	{
		.name = "NAND KERNEL partition",
		.offset = 4 * 0x100000,
		.size = 4 * 0x100000,
	},
	{
		.name = "NAND ROOTFS partition",
		.offset = 8 * 0x100000,
		.size = (504 + 512) * 0x100000,
	},
};

static struct mtd_partition noah_np1380_partitions_2gb[] = {
	{
		.name = "NAND BOOT partition",
		.offset = 0 * 0x100000,
		.size = 4 * 0x100000,
	},
	{
		.name = "NAND KERNEL partition",
		.offset = 4 * 0x100000,
		.size = 4 * 0x100000,
	},
	{
		.name = "NAND ROOTFS partition",
		.offset = 8 * 0x100000,
		.size = (504 + 512 + 1024) * 0x100000,
	},
};

static int noah_np1380_ooblayout_ecc(struct mtd_info *mtd, int section,
				 struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->length = 36;
	oobregion->offset = 6;

	if (mtd->oobsize == 128) {
		oobregion->length *= 2;
		oobregion->offset *= 2;
	}

	return 0;
}

static int noah_np1380_ooblayout_free(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *oobregion)
{
	int eccbytes = 36, eccoff = 6;

	if (section > 1)
		return -ERANGE;

	if (mtd->oobsize == 128) {
		eccbytes *= 2;
		eccoff *= 2;
	}

	if (!section) {
		oobregion->offset = 2;
		oobregion->length = eccoff - 2;
	} else {
		oobregion->offset = eccoff + eccbytes;
		oobregion->length = mtd->oobsize - oobregion->offset;
	}

	return 0;
}

static const struct mtd_ooblayout_ops noah_np1380_ooblayout_ops = {
	.ecc = noah_np1380_ooblayout_ecc,
	.free = noah_np1380_ooblayout_free,
};

static void noah_np1380_nand_ident(struct platform_device *pdev,
		struct mtd_info *mtd, struct mtd_partition **partitions,
		int *num_partitions)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if (chip->page_shift == 12) {
		*partitions = noah_np1380_partitions_2gb;
		*num_partitions = ARRAY_SIZE(noah_np1380_partitions_2gb);
	} else {
		*partitions = noah_np1380_partitions_1gb;
		*num_partitions = ARRAY_SIZE(noah_np1380_partitions_1gb);
	}

	mtd_set_ooblayout(mtd, &noah_np1380_ooblayout_ops);
}

static struct jz_nand_platform_data noah_np1380_nand_pdata = {
	.ident_callback = noah_np1380_nand_ident,
	.banks = { 1 },
};

static struct gpiod_lookup_table noah_np1380_nand_gpio_table = {
	.dev_id = "jz4740-nand.0",
	.table = {
		GPIO_LOOKUP("Bank C", 30, "busy", 0),
		{ },
	},
};


/* Keyboard*/

/* Display */
static struct fb_videomode noah_np1380_video_modes[] = {
	{
		.name = "320x240",
		.xres = 320,
		.yres = 240,
		.refresh = 30,
		.left_margin = 140,
		.right_margin = 273,
		.upper_margin = 20,
		.lower_margin = 2,
		.hsync_len = 1,
		.vsync_len = 1,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
	},
};

static struct jz4740_fb_platform_data noah_np1380_fb_pdata = {
	.width		= 60,
	.height		= 45,
	.num_modes	= ARRAY_SIZE(noah_np1380_video_modes),
	.modes		= noah_np1380_video_modes,
	.bpp		= 24,
	.lcd_type	= JZ_LCD_TYPE_8BIT_SERIAL,
	.pixclk_falling_edge = 1,
};

struct spi_gpio_platform_data spigpio_platform_data = {
	.sck = JZ_GPIO_PORTC(23),
	.mosi = JZ_GPIO_PORTC(22),
	.miso = -1,
	.num_chipselect = 1,
};

static struct platform_device spigpio_device = {
	.name = "spi_gpio",
	.id   = 1,
	.dev = {
		.platform_data = &spigpio_platform_data,
	},
};

static struct spi_board_info noah_np1380_spi_board_info[] = {
	{
		.modalias = "ili8960",
		.controller_data = (void *)JZ_GPIO_PORTC(21),
		.chip_select = 0,
		.bus_num = 1,
		.max_speed_hz = 30 * 1000,
		.mode = SPI_3WIRE,
	},
};

/* Battery */
static struct jz_battery_platform_data noah_np1380_battery_pdata = {
	.gpio_charge =	JZ_GPIO_PORTC(27),
	.gpio_charge_active_low = 1,
	.info = {
		.name = "battery",
		.technology = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design = 4200000,
		.voltage_min_design = 3600000,
	},
};

/* GPIO Key: power */
static struct gpio_keys_button noah_np1380_gpio_keys_buttons[] = {
	[0] = {
		.code		= KEY_POWER,
		.gpio		= JZ_GPIO_PORTD(29),
		.active_low	= 1,
		.desc		= "Power",
		.wakeup		= 1,
	},
};

static struct gpio_keys_platform_data noah_np1380_gpio_keys_data = {
	.nbuttons = ARRAY_SIZE(noah_np1380_gpio_keys_buttons),
	.buttons = noah_np1380_gpio_keys_buttons,
};

static struct platform_device noah_np1380_gpio_keys = {
	.name = "gpio-keys",
	.id =	-1,
	.dev = {
		.platform_data = &noah_np1380_gpio_keys_data,
	}
};

static struct jz4740_mmc_platform_data noah_np1380_mmc_pdata = {
	.gpio_card_detect	= NOAH_NP1380_GPIO_SD_CD,
	.gpio_read_only		= -1,
	.gpio_power		= NOAH_NP1380_GPIO_SD_VCC_EN_N,
	.power_active_low	= 1,
};

/* beeper */
static struct pwm_lookup noah_np1380_pwm_lookup[] = {
	PWM_LOOKUP("jz4740-pwm", 4, "pwm-beeper", NULL, 0,
		   PWM_POLARITY_NORMAL),
};

static struct platform_device noah_np1380_pwm_beeper = {
	.name = "pwm-beeper",
	.id = -1,
};

/* charger */
static char *noah_np1380_batteries[] = {
	"battery",
};

static struct gpio_charger_platform_data noah_np1380_charger_pdata = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.gpio = JZ_GPIO_PORTD(28),
	.gpio_active_low = 1,
	.supplied_to = noah_np1380_batteries,
	.num_supplicants = ARRAY_SIZE(noah_np1380_batteries),
};

static struct platform_device noah_np1380_charger_device = {
	.name = "gpio-charger",
	.dev = {
		.platform_data = &noah_np1380_charger_pdata,
	},
};

/* audio */

static struct platform_device *jz_platform_devices[] __initdata = {
	&jz4740_udc_device,
	&jz4740_udc_xceiv_device,
	&jz4740_mmc_device,
	&jz4740_nand_device,
	//&noah_np1380_keypad,
	&spigpio_device,
	&jz4740_framebuffer_device,
	&jz4740_pcm_device,
	&jz4740_i2s_device,
	&jz4740_codec_device,
	&jz4740_adc_device,
	&jz4740_pwm_device,
	&jz4740_dma_device,
	&noah_np1380_gpio_keys,
	&noah_np1380_pwm_beeper,
	&noah_np1380_charger_device,
	//&noah_np1380_audio_device,
};

static void __init board_gpio_setup(void)
{
	/* We only need to enable/disable pullup here for pins used in generic
	 * drivers. Everything else is done by the drivers themselves. */
	jz_gpio_disable_pullup(NOAH_NP1380_GPIO_SD_VCC_EN_N);
	jz_gpio_disable_pullup(NOAH_NP1380_GPIO_SD_CD);
}

static int __init noah_np1380_init_platform_devices(void)
{
	jz4740_framebuffer_device.dev.platform_data = &noah_np1380_fb_pdata;
	jz4740_nand_device.dev.platform_data = &noah_np1380_nand_pdata;
	jz4740_adc_device.dev.platform_data = &noah_np1380_battery_pdata;
	jz4740_mmc_device.dev.platform_data = &noah_np1380_mmc_pdata;

	//gpiod_add_lookup_table(&noah_np1380_audio_gpio_table);
	gpiod_add_lookup_table(&noah_np1380_nand_gpio_table);

	spi_register_board_info(noah_np1380_spi_board_info,
				ARRAY_SIZE(noah_np1380_spi_board_info));

	pwm_add_table(noah_np1380_pwm_lookup, ARRAY_SIZE(noah_np1380_pwm_lookup));

	return platform_add_devices(jz_platform_devices,
					ARRAY_SIZE(jz_platform_devices));

}

static int __init noah_np1380_board_setup(void)
{
	printk(KERN_INFO "Innovative Noah Electronic NP1380 setup\n");

	board_gpio_setup();

	if (noah_np1380_init_platform_devices())
		panic("Failed to initialize platform devices");

	return 0;
}
arch_initcall(noah_np1380_board_setup);

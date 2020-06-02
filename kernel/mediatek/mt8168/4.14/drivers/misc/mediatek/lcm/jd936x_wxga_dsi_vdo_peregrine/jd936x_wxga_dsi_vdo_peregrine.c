/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#endif
#endif
#include "lcm_drv.h"

#ifndef BUILD_LK
static unsigned int GPIO_LCD_RST;
static unsigned int GPIO_LCD_BL_EN;


void lcm_request_gpio_control(struct device *dev)
{
	GPIO_LCD_RST = of_get_named_gpio(dev->of_node, "gpio_lcd_rst", 0);
	gpio_request(GPIO_LCD_RST, "GPIO_LCD_RST");
	GPIO_LCD_BL_EN = of_get_named_gpio(dev->of_node, "gpio_bl_en", 0);
	gpio_request(GPIO_LCD_BL_EN, "GPIO_LCD_BL_EN");
}

static int lcm_driver_probe(struct device *dev, void const *data)
{
	lcm_request_gpio_control(dev);
	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "jd,jd936x",
		.data = 0,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		.name = "jd936x",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_drv_init(void)
{
	pr_notice("LCM: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_notice("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_drv_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}

late_initcall(lcm_drv_init);
module_exit(lcm_drv_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");
#endif

/* ----------------------------------------------------------------- */
/* Local Constants */
/* ----------------------------------------------------------------- */

#define FRAME_WIDTH		(800)
#define FRAME_HEIGHT		(1280)
#define GPIO_OUT_ONE		1
#define GPIO_OUT_ZERO		0

#define REGFLAG_DELAY		0xFE
#define REGFLAG_END_OF_TABLE		0x00

/* ----------------------------------------------------------------- */
/*  Local Variables */
/* ----------------------------------------------------------------- */
static struct LCM_UTIL_FUNCS lcm_util = { 0 };
#define SET_RESET_PIN(v)		(lcm_util.set_reset_pin((v)))
#define UDELAY(n)				(lcm_util.udelay(n))
#define MDELAY(n)				(lcm_util.mdelay(n))

/* ----------------------------------------------------------------- */
/* Local Functions */
/* ----------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		 (lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update))
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		 (lcm_util.dsi_set_cmdq(pdata, queue_size, force_update))
#define wrtie_cmd(cmd) \
		 (lcm_util.dsi_write_cmd(cmd))
#define write_regs(addr, pdata, byte_nums) \
		 (lcm_util.dsi_write_regs(addr, pdata, byte_nums))
#define read_reg \
		 (lcm_util.dsi_read_reg())
#define read_reg_v2(cmd, buffer, buffer_size) \
		 (lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size))

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, output);
#else
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
#endif
}

static void init_abc123_fiti_kd_lcm(void)
{
	unsigned int data_array[64];

	/* password */
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x93E11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x65E21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xF8E31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03801500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Page1 */
	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00001500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x69011500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set vcom_reverse*/
	data_array[0] = 0x00031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6C041500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Set Gamma power */
	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBF181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBF1B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x011C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x701F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2D201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2D211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7E221500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set SAP */
	data_array[0] = 0x28351500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Set Panel */
	data_array[0] = 0x19371500;
	dsi_set_cmdq(data_array, 1, 1);

	/*SET RGBCYC*/
	data_array[0] = 0x05381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x013A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7C3C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xFF3D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xFF3E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7F3F1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Set TCON */
	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xA0411500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0B441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28451500;
	dsi_set_cmdq(data_array, 1, 1);


	/* power voltage */
	data_array[0] = 0x0F551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xA8571500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A591500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2E5A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1A5B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x155C1500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Gamma */
	data_array[0] = 0x7F5D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x645E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x535F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x47601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x43611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x33621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x37631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x21641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x37661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x34671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x50681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3D691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x446A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x366B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x346C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x256D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x156E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7F701500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x64711500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x53721500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x47731500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x43741500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x33751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x37761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x21771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39781500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x37791500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x347A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x507B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3D7C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x447D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x367E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x347F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x25801500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15811500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02821500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Page2, for GIP */
	data_array[0] = 0x02E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_L Pin mapping */
	data_array[0] = 0x52001500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55021500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x50031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x77041500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x57051500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55061500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4E071500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4C081500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5F091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4A0A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x480B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x550C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x460D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x440E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x400F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55101500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55111500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55131500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55141500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55151500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_R Pin mapping */
	data_array[0] = 0x53161500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x51191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x771A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x571B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x551C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4F1D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4D1E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5F1F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4B201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x49211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x47231500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x45241500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x41251500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55271500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55281500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55291500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x552A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x552B1500;
	dsi_set_cmdq(data_array, 1, 1);

	/*GIP_L_GS Pin mapping*/
	data_array[0] = 0x132C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x152D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x152E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x012F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x37301500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15321500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F341500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05361500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07371500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x09391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0B3A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x113B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x153C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x153D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x153E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x153F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15411500;
	dsi_set_cmdq(data_array, 1, 1);

	/*GIP_R_GS Pin mapping*/
	data_array[0] = 0x12421500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00451500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x37461500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17471500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15481500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C491500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E4A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x154B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x044C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x064D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x154E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x084F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15521500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15541500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15561500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15571500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP Timing */
	data_array[0] = 0x40581500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x105B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x065C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x405D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x005E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x005F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x40601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6C631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6C641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x75651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xB4671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6C691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6C6A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C6B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x006D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x006E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x886F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBB751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2A781500;
	dsi_set_cmdq(data_array, 1, 1);


	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x11091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x480E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2B2B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x032D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x442E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x72121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02E61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0CE71500;
	dsi_set_cmdq(data_array, 1, 1);

	/*SLP OUT*/
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	/*DISP ON */
	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);

	/*TE*/
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);


}





/* ----------------------------------------------------------------- */
/* LCM Driver Implementations */
/* ----------------------------------------------------------------- */
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode   = SYNC_EVENT_VDO_MODE;

	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;

	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active		= 4;
	params->dsi.vertical_backporch			= 10;
	params->dsi.vertical_frontporch			= 30;
	params->dsi.vertical_active_line		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active		= 20;
	params->dsi.horizontal_backporch		= 43;
	params->dsi.horizontal_frontporch		= 43;
	params->dsi.horizontal_active_pixel		= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 234;
	params->dsi.clk_lp_per_line_enable = 1;

	params->dsi.cont_clock = 0;
	params->dsi.DA_HS_EXIT = 1;
	params->dsi.CLK_ZERO = 16;
	params->dsi.HS_ZERO = 9;
	params->dsi.HS_TRAIL = 5;
	params->dsi.CLK_TRAIL = 5;
	params->dsi.CLK_HS_POST = 8;
	params->dsi.CLK_HS_EXIT = 6;

	params->dsi.TA_GO = 8;
	params->dsi.TA_GET = 10;

	params->physical_width = 108;
	params->physical_height = 172;
}


static void lcm_init(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	init_abc123_fiti_kd_lcm();
}

static void lcm_resume(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(2);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(6);

	init_abc123_fiti_kd_lcm(); /* TPV panel */

	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);

}

static void lcm_init_power(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(2);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(5);
}

static void lcm_resume_power(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	display_bias_enable();

}

static void lcm_suspend(void)
{
	unsigned int data_array[16];

	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);

	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	data_array[0] = 0x00280500; /* Display Off */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);

	data_array[0] = 0x00100500; /* Sleep In */
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(120);
}

static void lcm_suspend_power(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(120);

	display_bias_disable();
	MDELAY(20);

}

struct LCM_DRIVER jd936x_wxga_dsi_vdo_peregrine_lcm_drv = {
	.name			= "jd936x_wxga_dsi_vdo_peregrine",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.init_power	= lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
};



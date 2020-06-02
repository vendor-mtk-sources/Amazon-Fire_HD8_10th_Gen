// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
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

#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#endif

#define LCM_CABC_MODE_REG	0x55
static  unsigned char vendor_id = 0xFF;
static  void get_lcm_id(void);

static struct lcm_platform_data {
	struct pinctrl *lcmctrl;
	struct pinctrl_state *lcm_pins_rst0;
	struct pinctrl_state *lcm_pins_rst1;
	struct pinctrl_state *lcm_pins_bl0;
	struct pinctrl_state *lcm_pins_bl1;
	struct regulator *vsim1_ldo;
	unsigned char lcm_id;
} lcm_data;

static int __init setup_lcm_id(char *str)
{
	int id;

	if (get_option(&str, &id))
		lcm_data.lcm_id = (unsigned char)id;
	else
		lcm_data.lcm_id = 0xFF;
	return 0;
}
__setup("LCM_VENDOR_ID=", setup_lcm_id);

static void get_lcm_id(void)
{
	if (lcm_data.lcm_id == 0xFF) {
		pr_err("get lcm_id failed.\n");
	} else {
		vendor_id = lcm_data.lcm_id;
		pr_debug("[ili9881c] get lcm_id from cmdline.\n");
	}

	pr_notice("[ili9881c] %s, vendor id = 0x%x\n", __func__, lcm_data.lcm_id);
}

static int lcm_driver_probe(struct device *dev, void const *data)
{
	int ret = 0;

	pr_notice("[Kernel/LCM] %s Default transmission mode: mipi\n",
					__func__);

	lcm_data.vsim1_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_data.vsim1_ldo)) {
		ret = PTR_ERR(lcm_data.vsim1_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	ret = regulator_enable(lcm_data.vsim1_ldo);
	if (ret < 0) {
		dev_err(dev, "failed to enable vsim1_ldo\n");
		goto lcm_power_free;
	}

	lcm_data.lcmctrl = devm_pinctrl_get(dev);
	if (IS_ERR(lcm_data.lcmctrl)) {
		dev_err(dev, "Cannot find lcm pinctrl!");
		ret = PTR_ERR(lcm_data.lcmctrl);
		goto lcm_power_enalbe_free;
	}

	/*lcm power pin lookup */
	lcm_data.lcm_pins_rst0 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_rst0");
	if (IS_ERR(lcm_data.lcm_pins_rst0)) {
		ret = PTR_ERR(lcm_data.lcm_pins_rst0);
		pr_err("pinctrl err, lcm_pins_rst0\n");
		goto lcm_pinctrl_free;
	}
	lcm_data.lcm_pins_rst1 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_rst1");
	if (IS_ERR(lcm_data.lcm_pins_rst1)) {
		ret = PTR_ERR(lcm_data.lcm_pins_rst1);
		pr_err("pinctrl err, lcm_pins_rst1\n");
		goto lcm_pinctrl_free;
	}
	lcm_data.lcm_pins_bl0 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_bl0");
	if (IS_ERR(lcm_data.lcm_pins_bl0)) {
		ret = PTR_ERR(lcm_data.lcm_pins_bl0);
		pr_err("pinctrl err, lcm_pins_bl0\n");
		goto lcm_pinctrl_free;
	}
	lcm_data.lcm_pins_bl1 = pinctrl_lookup_state(lcm_data.lcmctrl,
							"lcm_pins_bl1");
	if (IS_ERR(lcm_data.lcm_pins_bl1)) {
		ret = PTR_ERR(lcm_data.lcm_pins_bl1);
		pr_err("pinctrl err, lcm_pins_bl1\n");
		goto lcm_pinctrl_free;
	}

	return ret;

lcm_pinctrl_free:
	devm_pinctrl_put(lcm_data.lcmctrl);
	lcm_data.lcmctrl = NULL;

lcm_power_enalbe_free:
	regulator_disable(lcm_data.vsim1_ldo);

lcm_power_free:
	regulator_put(lcm_data.vsim1_ldo);
	lcm_data.vsim1_ldo = NULL;
	pr_err("[LCM] %s get lcm dev res failed.\n", __func__);
	return ret;
}

static void lcm_set_rst(int val)
{
	if (lcm_data.lcmctrl != NULL) {
		if (val == 0)
			pinctrl_select_state(lcm_data.lcmctrl,
						lcm_data.lcm_pins_rst0);
		else
			pinctrl_select_state(lcm_data.lcmctrl,
						lcm_data.lcm_pins_rst1);
	}
}

static void lcm_set_bl(int val)
{
	if (lcm_data.lcmctrl != NULL) {
		if (val == 0)
			pinctrl_select_state(lcm_data.lcmctrl,
						lcm_data.lcm_pins_bl0);
		else
			pinctrl_select_state(lcm_data.lcmctrl,
						lcm_data.lcm_pins_bl1);
	}
}

static int lcm_platform_remove(struct platform_device *pdev)
{
	/* free gpio */
	if (lcm_data.lcmctrl != NULL) {
		devm_pinctrl_put(lcm_data.lcmctrl);
		lcm_data.lcmctrl = NULL;
	}
	/* free lcm power */
	if (lcm_data.vsim1_ldo != NULL) {
		devm_regulator_put(lcm_data.vsim1_ldo);
		lcm_data.vsim1_ldo = NULL;
	}
	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{.compatible = "ili,ili9881c_inx_inx"},
	{},
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *id;
	unsigned int panel_id = 0xff;
	int ret = 0;

	get_lcm_id();

	ret = of_property_read_u32(np, "vendor_id", &panel_id);
	if (ret < 0)
		pr_err("get dts vendor_id failed\n");

	if (panel_id != vendor_id)
		return -ENODEV;

	pr_notice("[Kernel/LCM] %s panel_id = %d, vendor_id = %d\n",
			__func__, panel_id, vendor_id);

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		   .name = "ili9881c",
		   .owner = THIS_MODULE,
		   .of_match_table = lcm_platform_of_match,
	},
	.remove = lcm_platform_remove,
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
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

/* ----------------------------------------------------------------- */
/* Local Constants */
/* ----------------------------------------------------------------- */

#define FRAME_WIDTH		(800)
#define FRAME_HEIGHT		(1280)

#define REGFLAG_DELAY		0xFE
#define REGFLAG_END_OF_TABLE		0x00

/* ID1 DAh - Vendor and Build Designation
 * Bit[7:5] - Vendor Code (Innolux, LGD, Samsung)
 * Bit[4:2] - Reserved
 * Bit[2:0] - Build (proto, evt, dvt, etc.)
 */
#define HVT		0x0
#define EVT1		0x1
#define EVT2		0x2
#define DVT		0x4
#define PVT		0x7
#define MP		0x6
#define BUILD_MASK	0x7

/* Other vendor id used FITI IC
 * #define FITI_INX_INX 0x2
 * #define FITI_KD_HSD	0x3
 * #define FITI_KD_AUO	0x4
 * #define FITI_TG_BOE	0x5
 */
#define ILI_TG_INX		0x6		/* INX, using ILI IC */
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

static void init_onyx_ili_tg_inx_lcm(void)
{
	unsigned int data_array[8];
	pr_debug("[lcm/ili] %s enter.\n", __func__);

	/* CMD_Page 1 */
	data_array[0] = 0x00043902;
	data_array[1] = 0x038198ff;
	dsi_set_cmdq(data_array, 2, 1);
	/* GIP_1 */
	data_array[0] = 0x00011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x53031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x53041500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x13051500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04061500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02071500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02081500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000a1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000b1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000c1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000d1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000e1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00101500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00121500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00131500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00141500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00151500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00161500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001a1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001b1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001c1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001d1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xc01e1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x801f1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02201500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x09211500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00221500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00231500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00241500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00251500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00261500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00271500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55281500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03291500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x002a1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x002b1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x002c1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x002d1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x002e1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x002f1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00301500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00321500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00331500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00341500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00361500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00371500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3c381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x003a1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x003b1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x003c1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x003d1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x003e1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x003f1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00411500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00421500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00441500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_2 */
	data_array[0] = 0x01501500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45521500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x67531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x89541500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xab551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23571500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x67591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x895a1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xab5b1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xcd5c1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xef5d1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_3*/
	data_array[0] = 0x015e1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x085f1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0a621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x14641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x11661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x10671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0f691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0e6a1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x026b1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0d6c1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0c6d1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x066e1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x026f1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0a781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x15791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x147a1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x027b1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x107c1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x117d1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x027e1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0c7f1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0d801500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02811500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0e821500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0f831500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x08841500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02851500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02861500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02871500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02881500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02891500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x028a1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* CMD_Page 4*/
	data_array[0] = 0x00043902;
	data_array[1] = 0x048198ff;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x156c1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x306e1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x376f1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1f8d1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xba871500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x76261500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xd1b21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x07b51500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17351500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x14331500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x75311500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x853a1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x983b1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x107a1500;
	dsi_set_cmdq(data_array, 1, 1);
	/* CMD_Page 1*/
	data_array[0] = 0x00043902;
	data_array[1] = 0x018198ff;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x0a221500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xe9501500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xe4511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x28601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xc82e1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01341500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00a01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x11a11500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1fa21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x14a31500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x18a41500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2da51500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x21a61500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x21a71500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7da81500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1ba91500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x25aa1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x69ab1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1bac1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1aad1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x50ae1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x24af1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x29b01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4db11500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5ab21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23b31500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00c01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x14c11500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x22c21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x14c31500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17c41500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2ac51500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1fc61500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x20c71500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7dc81500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1cc91500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x28ca1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x68cb1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1dcc1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1dcd1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x52ce1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x27cf1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2dd01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4cd11500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x59d21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23d31500;
	dsi_set_cmdq(data_array, 1, 1);
	/* CABC page2 */
	data_array[0] = 0x00043902;
	data_array[1] = 0x028198ff;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x17041500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x12051500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40061500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0b071500;
	dsi_set_cmdq(data_array, 1, 1);
	/*page 0*/
	data_array[0] = 0x00043902;
	data_array[1] = 0x008198ff;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00033902;
	data_array[1] = 0x00f00f51;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x2c531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00291500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

/* ----------------------------------------------------------------- */
/* LCM Driver Implementations */
/* ----------------------------------------------------------------- */
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_ili_onyx_tg_inx_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));
	pr_debug("[lcm/ili] %s enter.\n", __func__);

	params->type      = LCM_TYPE_DSI;
	params->width     = FRAME_WIDTH;
	params->height    = FRAME_HEIGHT;
	params->dsi.mode  = SYNC_EVENT_VDO_MODE;

	params->dsi.LANE_NUM			= LCM_FOUR_LANE;
	params->dsi.data_format.color_order	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding		= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active		= 2;
	params->dsi.vertical_backporch			= 14;
	params->dsi.vertical_frontporch			= 20;
	params->dsi.vertical_active_line		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active		= 12;
	params->dsi.horizontal_backporch		= 46;
	params->dsi.horizontal_frontporch		= 46;
	params->dsi.horizontal_active_pixel		= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 234;
	params->dsi.clk_lp_per_line_enable = 1;

	/* params->dsi.ssc_disable = 1; */
	params->dsi.cont_clock = 0;
	params->dsi.DA_HS_EXIT = 10;
	params->dsi.CLK_ZERO = 16;
	params->dsi.HS_ZERO = 9;
	params->dsi.HS_TRAIL = 5;
	params->dsi.CLK_TRAIL = 5;
	params->dsi.CLK_HS_POST = 8;
	params->dsi.CLK_HS_EXIT = 6;
	/* params->dsi.CLK_HS_PRPR = 1; */

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

	params->dsi.TA_GO = 8;
	params->dsi.TA_GET = 10;

	params->physical_width = 108;
	params->physical_height = 172;
}

static void lcm_reset(void)
{
	/* GPIO20- Reset ili9881c */
	pr_notice("[ili9881c] %s, enter\n", __func__);

	lcm_set_rst(1);
	MDELAY(6);
	lcm_set_rst(0);
	MDELAY(2);
	lcm_set_rst(1);
	MDELAY(10);
}

static char *lcm_get_vendor_type(void)
{
	switch (vendor_id) {
	case ILI_TG_INX: return "ILI_TG_INX\0";
	default: return "Unknown\0";
	}
}

static void lcm_init(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	get_lcm_id();

	pr_info("[ili9881c] %s enter, skip power_on & init lcm since it's done by lk\n",
			 __func__);
	pr_info("[ili9881c] vendor type: %s\n",
			lcm_get_vendor_type());

	lcm_reset();
	if (vendor_id == ILI_TG_INX)
		init_onyx_ili_tg_inx_lcm();		/* ILI ONYX tg panel */
	else
		init_onyx_ili_tg_inx_lcm();		/* ILI ONYX tg panel */

	lcm_set_bl(1);
}

static void lcm_resume(void)
{
#ifdef CONFIG_AMAZON_METRICS_LOG
	char  buf[128];

	snprintf(buf, sizeof(buf), "%s:lcd:resume=1;CT;1:NR", __func__);
	log_to_metrics(ANDROID_LOG_INFO, "LCDEvent", buf);
#endif
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	get_lcm_id();

	lcm_reset();

	if (vendor_id == ILI_TG_INX)
		init_onyx_ili_tg_inx_lcm();		/* ILI ONYX TG panel */
	else
		init_onyx_ili_tg_inx_lcm();		/* ILI ONYX TG panel */

	lcm_set_bl(1);
}

static void lcm_init_power(void)
{
	int ret;

	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	if (lcm_data.vsim1_ldo == NULL) {
		pr_err("vsim1_ldo is NULL!\n");
		return;
	}
	ret = regulator_enable(lcm_data.vsim1_ldo);
	MDELAY(6);
	display_bias_enable();
	MDELAY(5);
	lcm_reset();
}

static void lcm_resume_power(void)
{
	int ret;

	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	if (lcm_data.vsim1_ldo == NULL) {
		pr_err("vsim1_ldo is NULL!\n");
		return;
	}
	ret = regulator_enable(lcm_data.vsim1_ldo);
	MDELAY(6);
	display_bias_enable();
	MDELAY(5);
}

static void lcm_suspend(void)
{
	unsigned int data_array[2];
#ifdef CONFIG_AMAZON_METRICS_LOG
	char buf[128];
	snprintf(buf, sizeof(buf), "%s:lcd:suspend=1;CT;1:NR", __func__);
	log_to_metrics(ANDROID_LOG_INFO, "LCDEvent", buf);
#endif
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	lcm_set_bl(0);
	MDELAY(2);
	data_array[0] = 0x00280500; /* Display Off */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(60);

	data_array[0] = 0x00100500; /* Sleep In */
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(120);
}

static void lcm_suspend_power(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	if (lcm_data.vsim1_ldo == NULL) {
		pr_err("vsim1_ldo is NULL!\n");
		return;
	}

	lcm_set_rst(0);
	MDELAY(2);
	display_bias_disable();
	MDELAY(5);
	regulator_disable(lcm_data.vsim1_ldo);
}

static void lcm_setbacklight_mode(unsigned int mode)
{
	if (mode >= CABC_MAX) {
		pr_err("%s invalid CABC mode=%d\n", __func__, mode);
		return;
	}
	pr_info("lcm driver cabc mode =%x\n", mode);
	dsi_set_cmdq_V2(LCM_CABC_MODE_REG, 1, ((unsigned char *)&mode), 1);
}

static void lcm_setbacklight_power_off(void)
{
	pr_notice("[Kernel/LCM] %s BL pull down.\n", __func__);

	lcm_set_bl(0);
}

struct LCM_DRIVER ili9881c_wxga_dsi_vdo_ili_tg_inx_onyx_lcm_drv = {
	.name		= "ili9881c_wxga_dsi_vdo_ili_tg_inx_onyx",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_ili_onyx_tg_inx_params,
	.init           = lcm_init,
	.init_power	= lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight_mode = lcm_setbacklight_mode,
	.set_backlight_power_off = lcm_setbacklight_power_off,
};

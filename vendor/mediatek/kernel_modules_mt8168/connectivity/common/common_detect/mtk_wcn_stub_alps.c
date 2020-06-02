/*
* Copyright (C) 2011-2014 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

/*
 *  ! \file
 *  \brief  Declaration of library functions
 *  Any definitions in this file will be shared among GLUE Layer and internal Driver Stack.
 */

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/


/* kernel wmt_build_in_adapter.c already has these, so always ignored */
#ifdef MTK_WCN_REMOVE_KERNEL_MODULE
#undef MTK_WCN_REMOVE_KERNEL_MODULE
#endif

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/
#define CMB_STUB_DBG_LOG                  3
#define CMB_STUB_INFO_LOG                 2
#define CMB_STUB_WARN_LOG                 1

int gCmbStubLogLevel = CMB_STUB_INFO_LOG;

#define CMB_STUB_LOG_PR_INFO(fmt, arg...) \
do { \
	if (gCmbStubLogLevel >= CMB_STUB_INFO_LOG) \
		pr_info(fmt, ##arg); \
} while (0)
#define CMB_STUB_LOG_PR_WARN(fmt, arg...) \
do { \
	if (gCmbStubLogLevel >= CMB_STUB_WARN_LOG) \
		pr_warn(fmt, ##arg); \
} while (0)
#define CMB_STUB_LOG_PR_DBG(fmt, arg...) \
do { \
	if (gCmbStubLogLevel >= CMB_STUB_DBG_LOG) \
		pr_info(fmt, ##arg); \
} while (0)

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <mtk_wcn_cmb_stub.h>
#include <wmt_build_in_adapter.h>
#include "wmt_detect.h"


/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

int gConnectivityChipId = -1;

/*
* current used uart port name, default is "ttyMT2",
* will be changed when wmt driver init
*/
char *wmt_uart_port_desc = "ttyMT2";
EXPORT_SYMBOL(wmt_uart_port_desc);

#ifdef MTK_WCN_REMOVE_KERNEL_MODULE
static void mtk_wcn_cmb_sdio_request_eirq(msdc_sdio_irq_handler_t irq_handler, void *data);
static void mtk_wcn_cmb_sdio_enable_eirq(void);
static void mtk_wcn_cmb_sdio_disable_eirq(void);
static void mtk_wcn_cmb_sdio_register_pm(pm_callback_t pm_cb, void *data);

struct sdio_ops mt_sdio_ops[4] = {
	{NULL, NULL, NULL, NULL},
	{NULL, NULL, NULL, NULL},
	{mtk_wcn_cmb_sdio_request_eirq, mtk_wcn_cmb_sdio_enable_eirq,
		mtk_wcn_cmb_sdio_disable_eirq, mtk_wcn_cmb_sdio_register_pm},
	{mtk_wcn_cmb_sdio_request_eirq, mtk_wcn_cmb_sdio_enable_eirq,
		mtk_wcn_cmb_sdio_disable_eirq, mtk_wcn_cmb_sdio_register_pm}
};
#endif

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

static wmt_aif_ctrl_cb cmb_stub_aif_ctrl_cb;
static wmt_func_ctrl_cb cmb_stub_func_ctrl_cb;
static wmt_thermal_query_cb cmb_stub_thermal_ctrl_cb;
/* static wmt_trigger_assert_cb cmb_stub_trigger_assert_cb; need confirm by Can */
static enum CMB_STUB_AIF_X cmb_stub_aif_stat = CMB_STUB_AIF_0;
static wmt_deep_idle_ctrl_cb cmb_stub_deep_idle_ctrl_cb;
static wmt_func_do_reset cmb_stub_do_reset_cb;
static wmt_clock_fail_dump_cb cmb_stub_clock_fail_dump_cb;
/* A temp translation table between COMBO_AUDIO_STATE_X and CMB_STUB_AIF_X.
 * This is used for ALPS backward compatible ONLY!!! Remove this table, related
 * functions, and type definition after modifying other kernel built-in modules,
 * such as AUDIO. [FixMe][GeorgeKuo]
 */
#if 0
static enum CMB_STUB_AIF_X audio2aif[] = {
	[COMBO_AUDIO_STATE_0] = CMB_STUB_AIF_0,
	[COMBO_AUDIO_STATE_1] = CMB_STUB_AIF_1,
	[COMBO_AUDIO_STATE_2] = CMB_STUB_AIF_2,
	[COMBO_AUDIO_STATE_3] = CMB_STUB_AIF_3,
};
#endif

#ifdef MTK_WCN_REMOVE_KERNEL_MODULE
static msdc_sdio_irq_handler_t mtk_wcn_cmb_sdio_eirq_handler;
static atomic_t sdio_claim_irq_enable_flag;
static atomic_t irq_enable_flag;
static pm_callback_t mtk_wcn_cmb_sdio_pm_cb;
static void *mtk_wcn_cmb_sdio_pm_data;
static void *mtk_wcn_cmb_sdio_eirq_data;

static u32 wifi_irq = 0xffffffff;
#endif
/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

#ifndef MTK_WCN_REMOVE_KERNEL_MODULE
static int _mtk_wcn_cmb_stub_query_ctrl(void);
/* static int _mtk_wcn_cmb_stub_trigger_assert(void); need confirm by Can */
static void _mtk_wcn_cmb_stub_clock_fail_dump(void);
#endif /* MTK_WCN_REMOVE_KERNEL_MODULE */

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/
/*!
 * \brief A registration function for WMT-PLAT to register itself to CMB-STUB.
 *
 * An MTK-WCN-CMB-STUB registration function provided to WMT-PLAT to register
 * itself and related callback functions when driver being loaded into kernel.
 *
 * \param p_stub_cb a pointer carrying CMB_STUB_CB information
 *
 * \retval 0 operation success
 * \retval -1 invalid parameters
 */
int mtk_wcn_cmb_stub_reg(struct _CMB_STUB_CB_ *p_stub_cb)
{
#ifndef MTK_WCN_REMOVE_KERNEL_MODULE
	struct wmt_platform_bridge pbridge;

	memset(&pbridge, 0, sizeof(struct wmt_platform_bridge));
#endif

	if ((!p_stub_cb)
	    || (p_stub_cb->size != sizeof(struct _CMB_STUB_CB_))) {
		CMB_STUB_LOG_PR_WARN("[cmb_stub] invalid p_stub_cb:0x%p size(%d)\n",
				  p_stub_cb, (p_stub_cb) ? p_stub_cb->size : 0);
		return -1;
	}

	CMB_STUB_LOG_PR_DBG("[cmb_stub] registered, p_stub_cb:0x%p size(%d)\n",
			p_stub_cb, p_stub_cb->size);

	cmb_stub_aif_ctrl_cb = p_stub_cb->aif_ctrl_cb;
	cmb_stub_func_ctrl_cb = p_stub_cb->func_ctrl_cb;
	cmb_stub_thermal_ctrl_cb = p_stub_cb->thermal_query_cb;
/*	cmb_stub_trigger_assert_cb = p_stub_cb->trigger_assert_cb; need confirm by Can */
	cmb_stub_deep_idle_ctrl_cb = p_stub_cb->deep_idle_ctrl_cb;
	cmb_stub_do_reset_cb = p_stub_cb->wmt_do_reset_cb;
	cmb_stub_clock_fail_dump_cb = p_stub_cb->clock_fail_dump_cb;

#ifndef MTK_WCN_REMOVE_KERNEL_MODULE
	pbridge.thermal_query_cb = _mtk_wcn_cmb_stub_query_ctrl;
/*	pbridge.trigger_assert_cb = _mtk_wcn_cmb_stub_trigger_assert; need confirm by Can */
	pbridge.clock_fail_dump_cb = _mtk_wcn_cmb_stub_clock_fail_dump;
	wmt_export_platform_bridge_register(&pbridge);
#endif

	return 0;
}
EXPORT_SYMBOL(mtk_wcn_cmb_stub_reg);
/*!
 * \brief A unregistration function for WMT-PLAT to unregister from CMB-STUB.
 *
 * An MTK-WCN-CMB-STUB unregistration function provided to WMT-PLAT to
 * unregister itself and clear callback function references.
 *
 * \retval 0 operation success
 */
int mtk_wcn_cmb_stub_unreg(void)
{
#ifndef MTK_WCN_REMOVE_KERNEL_MODULE
	wmt_export_platform_bridge_unregister();
#endif

	cmb_stub_aif_ctrl_cb = NULL;
	cmb_stub_func_ctrl_cb = NULL;
	cmb_stub_thermal_ctrl_cb = NULL;
	cmb_stub_deep_idle_ctrl_cb = NULL;
	cmb_stub_do_reset_cb = NULL;
	cmb_stub_clock_fail_dump_cb = NULL;
	CMB_STUB_LOG_PR_INFO("[cmb_stub] unregistered\n");	/* KERN_DEBUG */

	return 0;
}
EXPORT_SYMBOL(mtk_wcn_cmb_stub_unreg);

/* stub functions for kernel to control audio path pin mux */
int mtk_wcn_cmb_stub_aif_ctrl(enum CMB_STUB_AIF_X state, enum CMB_STUB_AIF_CTRL ctrl)
{
	int ret;

	if ((state >= CMB_STUB_AIF_MAX)
	    || (ctrl >= CMB_STUB_AIF_CTRL_MAX)) {

		CMB_STUB_LOG_PR_WARN("[cmb_stub] aif_ctrl invalid (%d, %d)\n",
				state, ctrl);
		return -1;
	}

	/* avoid the early interrupt before we register the eirq_handler */
	if (cmb_stub_aif_ctrl_cb) {
		ret = (*cmb_stub_aif_ctrl_cb) (state, ctrl);
		CMB_STUB_LOG_PR_INFO("aif state(%d->%d) ctrl(%d) ret(%d)\n",
		cmb_stub_aif_stat, state, ctrl, ret);	/* KERN_DEBUG */

		cmb_stub_aif_stat = state;
	} else {
		CMB_STUB_LOG_PR_WARN("[cmb_stub] aif_ctrl_cb null\n");
		ret = -2;
	}
	return ret;
}
EXPORT_SYMBOL(mtk_wcn_cmb_stub_aif_ctrl);

/* Use a temp translation table between COMBO_AUDIO_STATE_X and CMB_STUB_AIF_X
 * for ALPS backward compatible ONLY!!! Remove this table, related functions,
 * and type definition after modifying other kernel built-in modules, such as
 * AUDIO. [FixMe][GeorgeKuo]
 */

void mtk_wcn_cmb_stub_func_ctrl(unsigned int type, unsigned int on)
{
	if (cmb_stub_func_ctrl_cb)
		(*cmb_stub_func_ctrl_cb) (type, on);
	else
		CMB_STUB_LOG_PR_WARN("[cmb_stub] func_ctrl_cb null\n");
}
EXPORT_SYMBOL(mtk_wcn_cmb_stub_func_ctrl);

#ifdef MTK_WCN_REMOVE_KERNEL_MODULE
int mtk_wcn_cmb_stub_query_ctrl(void)
#else
static int _mtk_wcn_cmb_stub_query_ctrl(void)
#endif
{
	signed long temp = 0;

	if (cmb_stub_thermal_ctrl_cb)
		temp = (*cmb_stub_thermal_ctrl_cb) ();
	else
		CMB_STUB_LOG_PR_WARN("[cmb_stub] thermal_ctrl_cb null\n");

	return temp;
}

#if 0
#ifdef MTK_WCN_REMOVE_KERNEL_MODULE
int mtk_wcn_cmb_stub_trigger_assert(void)
#else
static int _mtk_wcn_cmb_stub_trigger_assert(void)
#endif
{
	int ret = 0;

	if (cmb_stub_trigger_assert_cb)
		ret = (*cmb_stub_trigger_assert_cb) ();
	else
		CMB_STUB_LOG_PR_INFO("[cmb_stub] trigger_assert_cb null\n");

	return ret;
}
#endif

#ifndef MTK_WCN_REMOVE_KERNEL_MODULE
void _mtk_wcn_cmb_stub_clock_fail_dump(void)
{
	if (cmb_stub_clock_fail_dump_cb)
		(*cmb_stub_clock_fail_dump_cb) ();
	else
		CMB_STUB_LOG_PR_WARN("[cmb_stub] clock_fail_dump_cb null\n");
}
#endif

/*platform-related APIs*/
/* void clr_device_working_ability(UINT32 clockId, MT6573_STATE state); */
/* void set_device_working_ability(UINT32 clockId, MT6573_STATE state); */

static int _mt_combo_plt_do_deep_idle(enum COMBO_IF src, int enter)
{
	int ret = -1;

#if 0
	if (src != COMBO_IF_UART && src != COMBO_IF_MSDC && src != COMBO_IF_BTIF) {
		CMB_STUB_LOG_PR_WARN("src = %d is error\n", src);
		return ret;
	}
	if (src >= 0 && src < COMBO_IF_MAX)
		CMB_STUB_LOG_PR_INFO("src = %s, to enter deep idle? %d\n",
				combo_if_name[src], enter);
#endif
	/*
	 *  TODO: For Common SDIO configuration, we need to do some judgement between STP and WIFI
	 *  to decide if the msdc will enter deep idle safely
	 */

	switch (src) {
	case COMBO_IF_UART:
		if (enter == 0) {
			/* clr_device_working_ability(MT65XX_PDN_PERI_UART3, DEEP_IDLE_STATE); */
			/* disable_dpidle_by_bit(MT65XX_PDN_PERI_UART2); */
#ifdef MTK_WCN_COMBO_CHIP_SUPPORT
#if 0
			ret = mtk_uart_pdn_enable(wmt_uart_port_desc, 0);
			if (ret < 0)
				CMB_STUB_LOG_PR_WARN("%s exit deepidle failed",
						wmt_uart_port_desc);
#endif
#endif
		} else {
			/* set_device_working_ability(MT65XX_PDN_PERI_UART3, DEEP_IDLE_STATE); */
			/* enable_dpidle_by_bit(MT65XX_PDN_PERI_UART2); */
#ifdef MTK_WCN_COMBO_CHIP_SUPPORT
#if 0
			ret = mtk_uart_pdn_enable(wmt_uart_port_desc, 1);
			if (ret < 0)
				CMB_STUB_LOG_PR_WARN("%s enter deepidle failed",
						wmt_uart_port_desc);
#endif
#endif
		}
		ret = 0;
		break;

	case COMBO_IF_MSDC:
		if (enter == 0) {
			/* for common sdio hif */
			/* clr_device_working_ability(MT65XX_PDN_PERI_MSDC2, DEEP_IDLE_STATE); */
		} else {
			/* for common sdio hif */
			/* set_device_working_ability(MT65XX_PDN_PERI_MSDC2, DEEP_IDLE_STATE); */
		}
		ret = 0;
		break;

	case COMBO_IF_BTIF:
		if (cmb_stub_deep_idle_ctrl_cb)
			ret = (*cmb_stub_deep_idle_ctrl_cb) (enter);
		else
			CMB_STUB_LOG_PR_WARN("NULL function pointer\n");

		if (ret)
			CMB_STUB_LOG_PR_WARN("%s deep idle fail(%d)\n",
					enter == 1 ? "enter" : "exit", ret);
		else
			CMB_STUB_LOG_PR_DBG("%s deep idle ok(%d)\n",
					enter == 1 ? "enter" : "exit", ret);
		break;
	default:
		break;
	}

	return ret;
}

int mt_combo_plt_enter_deep_idle(enum COMBO_IF src)
{
	/* return 0; */
	/* TODO: [FixMe][GeorgeKuo] handling this depends on common UART or common SDIO */
	return _mt_combo_plt_do_deep_idle(src, 1);
}
EXPORT_SYMBOL(mt_combo_plt_enter_deep_idle);

int mt_combo_plt_exit_deep_idle(enum COMBO_IF src)
{
	/* return 0; */
	/* TODO: [FixMe][GeorgeKuo] handling this depends on common UART or common SDIO */
	return _mt_combo_plt_do_deep_idle(src, 0);
}
EXPORT_SYMBOL(mt_combo_plt_exit_deep_idle);

int mtk_wcn_wmt_chipid_query(void)
{
	CMB_STUB_LOG_PR_INFO("query current consys chipid (0x%x)\n",
			gConnectivityChipId);
	return gConnectivityChipId;
}
EXPORT_SYMBOL(mtk_wcn_wmt_chipid_query);

void mtk_wcn_wmt_set_chipid(int chipid)
{
	CMB_STUB_LOG_PR_INFO("set current consys chipid (0x%x)\n", chipid);
	gConnectivityChipId = chipid;
}
EXPORT_SYMBOL(mtk_wcn_wmt_set_chipid);

int mtk_wcn_cmb_stub_do_reset(unsigned int type)
{
	if (cmb_stub_do_reset_cb)
		return (*cmb_stub_do_reset_cb) (type);
	else
		return -1;
}
EXPORT_SYMBOL(mtk_wcn_cmb_stub_do_reset);

#ifdef MTK_WCN_REMOVE_KERNEL_MODULE
static void mtk_wcn_cmb_sdio_enable_eirq(void)
{
	if (atomic_read(&irq_enable_flag))
		CMB_STUB_LOG_PR_DBG("wifi eint has been enabled\n");
	else {
		atomic_set(&irq_enable_flag, 1);
		if (wifi_irq != 0xffffffff) {
			enable_irq(wifi_irq);
			CMB_STUB_LOG_PR_DBG(" enable WIFI EINT irq %d !!\n",
					wifi_irq);
		}
	}
}

static void mtk_wcn_cmb_sdio_disable_eirq(void)
{
	if (!atomic_read(&irq_enable_flag))
		CMB_STUB_LOG_PR_DBG("wifi eint has been disabled!\n");
	else {
		if (wifi_irq != 0xffffffff) {
			disable_irq_nosync(wifi_irq);
			CMB_STUB_LOG_PR_DBG("disable WIFI EINT irq %d !!\n",
					wifi_irq);
		}
		atomic_set(&irq_enable_flag, 0);
	}
}

irqreturn_t mtk_wcn_cmb_sdio_eirq_handler_stub(int irq, void *data)
{
	if ((mtk_wcn_cmb_sdio_eirq_handler != NULL) && (atomic_read(&sdio_claim_irq_enable_flag) != 0))
		mtk_wcn_cmb_sdio_eirq_handler(mtk_wcn_cmb_sdio_eirq_data);
	return IRQ_HANDLED;
}

static void mtk_wcn_cmb_sdio_request_eirq(msdc_sdio_irq_handler_t irq_handler, void *data)
{
	struct device_node *node;
	int ret = -EINVAL;
#if 0
	unsigned int gpio_wifi_eint_pin;
#endif

	CMB_STUB_LOG_PR_INFO("enter %s\n", __func__);
	mtk_wcn_sdio_irq_flag_set(0);
	atomic_set(&irq_enable_flag, 1);
	mtk_wcn_cmb_sdio_eirq_data = data;
	mtk_wcn_cmb_sdio_eirq_handler = irq_handler;

	node = (struct device_node *)of_find_compatible_node(NULL, NULL, "mediatek,connectivity-combo");
	if (node) {
#if 0
		gpio_wifi_eint_pin = of_get_gpio(node, 5);
		CMB_STUB_LOG_PR_INFO("WIFI EINT pin %d !!\n",
				gpio_wifi_eint_pin);
		wifi_irq = gpio_to_irq(gpio_wifi_eint_pin);
#else
		wifi_irq = irq_of_parse_and_map(node, 0);/* get wifi eint num */
#endif
#if 1
		ret = request_irq(wifi_irq, mtk_wcn_cmb_sdio_eirq_handler_stub, IRQF_TRIGGER_LOW,
				"WIFI-eint", NULL);
		CMB_STUB_LOG_PR_DBG("WIFI EINT irq %d !!\n", wifi_irq);
#endif

		if (ret)
			CMB_STUB_LOG_PR_WARN("EINT IRQ LINE NOT AVAILABLE!!\n");
		else
			mtk_wcn_cmb_sdio_disable_eirq();/*not ,chip state is power off*/
	} else
		CMB_STUB_LOG_PR_WARN("[%s] can't find device node\n", __func__);

	CMB_STUB_LOG_PR_INFO("exit %s\n", __func__);
}

static void mtk_wcn_cmb_sdio_register_pm(pm_callback_t pm_cb, void *data)
{
	CMB_STUB_LOG_PR_DBG("mtk_wcn_cmb_sdio_register_pm (0x%p, 0x%p)\n",
			pm_cb, data);
	/* register pm change callback */
	mtk_wcn_cmb_sdio_pm_cb = pm_cb;
	mtk_wcn_cmb_sdio_pm_data = data;
}
#endif /* MTK_WCN_REMOVE_KERNEL_MODULE */

static void mtk_wcn_cmb_sdio_on(int sdio_port_num)
{
	pm_message_t state = {.event = PM_EVENT_USER_RESUME };

	CMB_STUB_LOG_PR_INFO("mtk_wcn_cmb_sdio_on (%d)\n", sdio_port_num);

	/* 1. disable sdio eirq */
#ifdef MTK_WCN_REMOVE_KERNEL_MODULE
	mtk_wcn_cmb_sdio_disable_eirq();
#else
	wmt_export_mtk_wcn_cmb_sdio_disable_eirq();
#endif

	/* 2. call sd callback */
	if (mtk_wcn_cmb_sdio_pm_cb) {
		/* pr_warn("mtk_wcn_cmb_sdio_pm_cb(PM_EVENT_USER_RESUME, 0x%p, 0x%p)\n",
		 * mtk_wcn_cmb_sdio_pm_cb, mtk_wcn_cmb_sdio_pm_data);
		 */
		mtk_wcn_cmb_sdio_pm_cb(state, mtk_wcn_cmb_sdio_pm_data);
	} else
		CMB_STUB_LOG_PR_WARN("mtk_wcn_cmb_sdio_on no sd callback!!\n");
}

static void mtk_wcn_cmb_sdio_off(int sdio_port_num)
{
	pm_message_t state = {.event = PM_EVENT_USER_SUSPEND };

	CMB_STUB_LOG_PR_INFO("mtk_wcn_cmb_sdio_off (%d)\n", sdio_port_num);

	/* 1. call sd callback */
	if (mtk_wcn_cmb_sdio_pm_cb) {
		/* pr_warn("mtk_wcn_cmb_sdio_off(PM_EVENT_USER_SUSPEND, 0x%p, 0x%p)\n",
		 * mtk_wcn_cmb_sdio_pm_cb, mtk_wcn_cmb_sdio_pm_data);
		*/
		mtk_wcn_cmb_sdio_pm_cb(state, mtk_wcn_cmb_sdio_pm_data);
	} else
		CMB_STUB_LOG_PR_WARN("mtk_wcn_cmb_sdio_off no sd callback!!\n");

	/* 2. disable sdio eirq */
#ifdef MTK_WCN_REMOVE_KERNEL_MODULE
	mtk_wcn_cmb_sdio_disable_eirq();
#else
	wmt_export_mtk_wcn_cmb_sdio_disable_eirq();
#endif
}

int board_sdio_ctrl(unsigned int sdio_port_num, unsigned int on)
{
	CMB_STUB_LOG_PR_DBG("mt_mtk_wcn_cmb_sdio_ctrl (%d, %d)\n",
			sdio_port_num, on);
	if (on) {
#if 1
		CMB_STUB_LOG_PR_DBG("board_sdio_ctrl force off before on\n");
		mtk_wcn_cmb_sdio_off(sdio_port_num);
#else
		CMB_STUB_LOG_PR_WARN("skip sdio off before on\n");
#endif
		/* off -> on */
		mtk_wcn_cmb_sdio_on(sdio_port_num);
		if (wifi_irq != 0xffffffff)
			irq_set_irq_wake(wifi_irq, 1);
		else
			CMB_STUB_LOG_PR_WARN("wifi_irq is not available\n");
	} else {
			if (wifi_irq != 0xffffffff)
				irq_set_irq_wake(wifi_irq, 0);
			else
				CMB_STUB_LOG_PR_WARN("wifi_irq is not available\n");
			/* on -> off */
			mtk_wcn_cmb_sdio_off(sdio_port_num);
	}

	return 0;
}
EXPORT_SYMBOL(board_sdio_ctrl);

#ifdef MTK_WCN_REMOVE_KERNEL_MODULE
int mtk_wcn_sdio_irq_flag_set(int flag)
{
	if (flag != 0)
		atomic_set(&sdio_claim_irq_enable_flag, 1);
	else
		atomic_set(&sdio_claim_irq_enable_flag, 0);

	CMB_STUB_LOG_PR_DBG("sdio_claim_irq_enable_flag:%d\n",
			atomic_read(&sdio_claim_irq_enable_flag));

	return atomic_read(&sdio_claim_irq_enable_flag);
}
EXPORT_SYMBOL(mtk_wcn_sdio_irq_flag_set);
#endif

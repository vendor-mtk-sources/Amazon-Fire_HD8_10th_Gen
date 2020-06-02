/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2016 MediaTek Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
/*
 ** Id: /os/linux/gl_proc.c
 */

/*! \file   "gl_proc.c"
 *  \brief  This file defines the interface which can interact with users
 *          in /proc fs.
 *
 *    Detail description.
 */


/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "precomp.h"
#include "gl_os.h"
#include "gl_kal.h"
#include "debug.h"
#include "wlan_lib.h"
#include "debug.h"
#include "wlan_oid.h"

#include <linux/rtc.h> /* fos_change oneline */

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
#define PROC_MCR_ACCESS                         "mcr"
#define PROC_ROOT_NAME							"wlan"

#if CFG_SUPPORT_DEBUG_FS
#define PROC_ROAM_PARAM							"roam_param"
#endif
#define PROC_COUNTRY							"country"
#define PROC_DRV_STATUS                         "status"
#define PROC_RX_STATISTICS                      "rx_statistics"
#define PROC_TX_STATISTICS                      "tx_statistics"
#define PROC_DBG_LEVEL_NAME                     "dbgLevel"
#define PROC_DRIVER_CMD                         "driver"
#define PROC_CFG                                "cfg"
#define PROC_EFUSE_DUMP                         "efuse_dump"
#define PROC_PKT_DELAY_DBG			"pktDelay"
#if CFG_SUPPORT_SET_CAM_BY_PROC
#define PROC_SET_CAM				"setCAM"
#endif
#define PROC_AUTO_PERF_CFG			"autoPerfCfg"

/* fos_change begin */
#if CFG_SUPPORT_DTIM_SKIP
#define PROC_DTIM					"dtim_skip_count"
#endif

#if CFG_SUPPORT_WIFI_POWER_DEBUG
#define PROC_WAKEUP_LOG				"wakeup_log"
#define PROC_TX_RX_STAT				"traffic_stat"
#endif /* fos_change end */

#define PROC_MCR_ACCESS_MAX_USER_INPUT_LEN      20
#define PROC_RX_STATISTICS_MAX_USER_INPUT_LEN   10
#define PROC_TX_STATISTICS_MAX_USER_INPUT_LEN   10
#define PROC_DBG_LEVEL_MAX_USER_INPUT_LEN       20
#define PROC_DBG_LEVEL_MAX_DISPLAY_STR_LEN      30
#define PROC_UID_SHELL							2000
#define PROC_GID_WIFI							1010

/* notice: str only can be an array */
#define SNPRINTF(buf, str, arg)   {buf += \
	snprintf((char *)(buf), sizeof(str)-kalStrLen(str), PRINTF_ARG arg); }

/*******************************************************************************
 *                             D A T A   T Y P E S
 *******************************************************************************
 */

/* fos_change begin */
struct BLOCKED_READING_PROC_T {
	wait_queue_head_t waitq;
	struct mutex lock;
	uint8_t *pucBuf;
	int64_t i8WrPos;
	uint32_t u2BufLen; /* Max pucBuf length is 65535 */
	bool fgEnabled;
	bool fgRemoving;
}; /* fos_change end */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */
static struct GLUE_INFO *g_prGlueInfo_proc;
static uint32_t u4McrOffset;
static struct proc_dir_entry *gprProcNetRoot;
static struct proc_dir_entry *gprProcRoot;
static uint8_t aucDbModuleName[][PROC_DBG_LEVEL_MAX_DISPLAY_STR_LEN] = {
	"INIT", "HAL", "INTR", "REQ", "TX", "RX", "RFTEST", "EMU",
	"SW1", "SW2", "SW3", "SW4", "HEM", "AIS", "RLM", "MEM",
	"CNM", "RSN", "BSS", "SCN", "SAA", "AAA", "P2P", "QM",
	"SEC", "BOW", "WAPI", "ROAMING", "TDLS", "PF", "OID", "NIC",
	"WNM", "WMM"
};

/* This buffer could be overwrite by any proc commands */
static uint8_t g_aucProcBuf[3000];

/* This u32 is only for DriverCmdRead/Write,
 * should not be used by other function
 */
static uint32_t g_u4NextDriverReadLen;

/* fos_change begin */
static struct BLOCKED_READING_PROC_T rDrvStatusProc;
#if CFG_SUPPORT_WIFI_POWER_DEBUG
static struct BLOCKED_READING_PROC_T rWakeupLogProc;
static struct BLOCKED_READING_PROC_T rAppTxRxProc;
static char acSuspendTime[32];
static char acResumeTime[32];
#endif /* fos_change end */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/* fos_change begin */

#define PROC_READ_COMMON(buf, f_pos, _u4Copy_size) \
        do { \
            _u4Copy_size = kalStrLen(g_aucProcBuf); \
            if (copy_to_user(buf, g_aucProcBuf, _u4Copy_size)) { \
                pr_err("copy to user failed\n"); \
                return -EFAULT; \
            } \
            *f_pos += _u4Copy_size; \
        } while (0)

#define PROC_WRITE_COMMON(buffer, count) \
        do { \
            uint32_t u4CopySize = sizeof(g_aucProcBuf); \
            kalMemSet(g_aucProcBuf, 0, u4CopySize); \
            if (u4CopySize >= count+1) { \
                u4CopySize = count; \
            } \
            else \
            {   u4CopySize = u4CopySize - 1; \
            } \
            if (copy_from_user(g_aucProcBuf, buffer, u4CopySize)) { \
                pr_err("error of copy from user\n"); \
                return -EFAULT; \
            } \
            g_aucProcBuf[u4CopySize] = '\0'; \
        } while (0)

#define CONFIGURE_BLOCKED_READING_PROC_ON_OFF(_pProc) \
	do {\
		if (!kalStrnCmp(g_aucProcBuf, "enable", 6) && !(_pProc)->fgEnabled) { \
			(_pProc)->fgEnabled = TRUE;\
			(_pProc)->i8WrPos = 0;\
			return 6;\
		} \
		if (!kalStrnCmp(g_aucProcBuf, "disable", 7) && (_pProc)->fgEnabled) { \
			(_pProc)->fgEnabled = FALSE;\
			glProcWakeupThreads(_pProc, 1);\
			return 7;\
		} \
	} while (0)  /* fos_change end */

/*******************************************************************************
 *                   F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/* Kernel didn't support 64bit modulus, when divisor is a variable.
** Here we implement one, efficiency is not more considered.
*/
static uint32_t calModulus64(int64_t i8Dividend, uint32_t u4Divisor)
{
	/* modulus of 0x100000000 % u4Divisor */
	const uint32_t u432BitModulus =	0xFFFFFFFFU % u4Divisor + 1;
	uint32_t u4LowModulus = 0;

	if (!u4Divisor)
		return 0;

	do {
		u4LowModulus = (uint32_t)(i8Dividend & 0xffffffff) % u4Divisor;
		/* High 32 bit modulus sum */
		i8Dividend >>= 32;
		i8Dividend *= (int64_t)u432BitModulus;
		/* Add Low 32 bit modulus */
		i8Dividend += (int64_t)u4LowModulus;
	} while (i8Dividend > (int64_t)u4Divisor);

	return (uint32_t)i8Dividend;
}

static void glProcWakeupThreads(struct BLOCKED_READING_PROC_T *prProc,
		uint32_t u4LoopTimes)
{
	/* Wake up all readers if at least one is waiting */
	while (u4LoopTimes > 0 && waitqueue_active(&prProc->waitq)) {
		wake_up_interruptible(&prProc->waitq);
		u4LoopTimes--;
		if (u4LoopTimes > 0)
			kalMsleep(10);
	}
}

static bool procInitBlockedReadProc(struct BLOCKED_READING_PROC_T *prProc,
		uint16_t u2BufLen, bool fgEnabled)
{
	prProc->fgEnabled = fgEnabled;
	prProc->fgRemoving = FALSE;
	prProc->i8WrPos = 0;
	mutex_init(&prProc->lock);
	if (u2BufLen) {
		prProc->pucBuf = kalMemAlloc(u2BufLen, VIR_MEM_TYPE);
		if (!prProc->pucBuf)
			return FALSE;
	} else
		prProc->pucBuf = NULL;
	prProc->u2BufLen = u2BufLen;
	init_waitqueue_head(&prProc->waitq);
	return TRUE;
}

static void procUninitBlockedReadProc(struct BLOCKED_READING_PROC_T *prProc)
{
	prProc->fgRemoving = TRUE;
	glProcWakeupThreads(prProc, 1);
	kalMemFree(prProc->pucBuf, VIR_MEM_TYPE, prProc->u2BufLen);
}

static ssize_t wait_data_ready(struct BLOCKED_READING_PROC_T *proc, char __user *buf, loff_t *f_pos)
{
	int32_t ret = -1;

	while (ret) {
		ret = wait_event_interruptible(proc->waitq, (proc->i8WrPos != *f_pos ||
					       !proc->fgEnabled || proc->fgRemoving));
		if (ret == -ERESTARTSYS)
			return -EINTR;
	}
	if (proc->fgRemoving || !proc->fgEnabled || !g_prGlueInfo_proc ||
		test_bit(GLUE_FLAG_HALT_BIT, &g_prGlueInfo_proc->ulFlag))
		return 0;

	if (proc->i8WrPos < *f_pos)
		return -ESTRPIPE;

	return 0xefffffff;
}

static ssize_t procHelpMessageToUser(char __user *buf, loff_t *f_pos, size_t count, char *errMsg)
{
	uint32_t u4Len = 0;

	if (!errMsg)
		return 0;

	u4Len = kalStrLen(errMsg);

	if (*f_pos >= u4Len)
		return 0;

	u4Len -= *f_pos;
	if (u4Len > count)
		u4Len = count;

	if (copy_to_user(buf, errMsg + *f_pos, u4Len)) {
		DBGLOG(INIT, WARN, "copy_to_user error\n");
		return -EFAULT;
	}
	*f_pos += u4Len;
	return (ssize_t)u4Len;
}

static ssize_t read_virtual_buf(struct BLOCKED_READING_PROC_T *prProc,
	unsigned char **ppucRdPos, loff_t *f_pos)
{
	int16_t i2CopySize = 0;
	uint8_t *pucBuf = prProc->pucBuf;
	int64_t i8WrPos = prProc->i8WrPos;
	uint16_t u2BufSize = prProc->u2BufLen;

	mutex_lock(&prProc->lock);
	if (*f_pos > 0) {/* Read again */
		if (i8WrPos - *f_pos > u2BufSize) {
			i2CopySize = (int16_t)calModulus64(i8WrPos, (uint32_t)u2BufSize);
			*ppucRdPos = pucBuf + i2CopySize;
			i2CopySize = u2BufSize - i2CopySize;
			DBGLOG(INIT, TRACE, "Lost %lld bytes, WR:%lld, RD:%lld, MaxRd:%u bytes\n",
				   (i8WrPos - *f_pos - u2BufSize), i8WrPos, *f_pos, i2CopySize);
			*f_pos = i8WrPos - u2BufSize;
		} else {
			i2CopySize = (int16_t)calModulus64(*f_pos, (uint32_t)u2BufSize);
			*ppucRdPos = pucBuf + i2CopySize;
			if (i8WrPos - *f_pos > u2BufSize - i2CopySize)
				i2CopySize = u2BufSize - i2CopySize;
			else
				i2CopySize = i8WrPos - *f_pos;
			DBGLOG(INIT, TRACE, "Continue to read, WR:%lld, RD:%lld, MaxRd:%u bytes\n",
				   i8WrPos, *f_pos, i2CopySize);
		}
	} else {/* The first time t read for current reader */
		if (i8WrPos > u2BufSize) {
			i2CopySize = (int16_t)calModulus64(i8WrPos, (uint32_t)u2BufSize);
			*ppucRdPos = pucBuf + i2CopySize;
			i2CopySize = u2BufSize - i2CopySize;
			*f_pos = i8WrPos - u2BufSize;
		} else {
			*ppucRdPos = pucBuf;
			i2CopySize = (int16_t)i8WrPos;
		}
		DBGLOG(INIT, TRACE, "First time to read, WR:%lld, RD:%lld, MaxRd:%u bytes\n",
			   i8WrPos, *f_pos, i2CopySize);
	}
	mutex_unlock(&prProc->lock);
	return (ssize_t)i2CopySize;
}

static ssize_t procReadBlockedProc(
	struct BLOCKED_READING_PROC_T *prBlockProc, char __user *buf, bool fgBlockRead,
	size_t count, loff_t *f_pos, char *errMsg)
{
	uint8_t *pucRdPos = NULL;
	ssize_t i4CopySize = 0;

	if (!prBlockProc->fgEnabled)
		return procHelpMessageToUser(buf, f_pos, count, errMsg);

	if (fgBlockRead) {
		i4CopySize = wait_data_ready(prBlockProc, buf, f_pos);
		if (i4CopySize != 0xefffffff)
			return (ssize_t)(i4CopySize & 0xffffffff);
	} else if (*f_pos == prBlockProc->i8WrPos) {
		DBGLOG(INIT, INFO, "No data available\n");
		return -EAGAIN;
	}
	i4CopySize = read_virtual_buf(prBlockProc, &pucRdPos, f_pos);
	DBGLOG(INIT, TRACE, "Read %d bytes, user buf size %u\n", i4CopySize, count);
	if (i4CopySize > count)
		i4CopySize = (ssize_t)count;
	if (!pucRdPos || copy_to_user(buf, pucRdPos, i4CopySize)) {
		DBGLOG(INIT, WARN, "copy_to_user error\n");
		return 0;
	}
	*f_pos += i4CopySize;
	return i4CopySize;
}

static void glFormatOutput(
	u_int8_t fgTimeStamp, int64_t *pi8VirtualWrPos, uint8_t *pucBufStart,
	const uint16_t u2BufSize, uint8_t **ppucWrPos, uint16_t *pu2RemainLen,
	uint8_t *pucFwt, ...)
{
#define TEMP_BUF_LEN 280
	uint8_t *pucTemp = NULL;
	int16_t i2BufUsed = 0;
	int16_t i2TimeUsed = 0;
	va_list ap;
	struct timeval tval;
	struct rtc_time tm;
	static uint8_t aucBuf[TEMP_BUF_LEN];

	pucTemp = &aucBuf[0];
	if (fgTimeStamp) {
		do_gettimeofday(&tval);
		tval.tv_sec -= sys_tz.tz_minuteswest * 60;
		rtc_time_to_tm(tval.tv_sec, &tm);

		i2TimeUsed = (int16_t)kalSnprintf(pucTemp,
			TEMP_BUF_LEN, "%04d-%02d-%02d %02d:%02d:%02d.%03d ",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
			tm.tm_min, tm.tm_sec, (int32_t)(tval.tv_usec / USEC_PER_MSEC));
		if (i2TimeUsed < 0) {
			DBGLOG(INIT, ERROR, "error to sprintf time\n");
			return;
		}
	}
	va_start(ap, pucFwt);
	i2BufUsed = (int16_t)vsnprintf(pucTemp + i2TimeUsed,
		TEMP_BUF_LEN - i2TimeUsed, pucFwt, ap);
	va_end(ap);
	if (i2BufUsed < 0) {
		DBGLOG(INIT, ERROR, "error to sprintf %s\n", pucFwt);
		return;
	}
	i2BufUsed += i2TimeUsed;
	DBGLOG(INIT, TRACE, "WrPos %lld, BufUsed %d, Remain %d, %s", *pi8VirtualWrPos, i2BufUsed, *pu2RemainLen, aucBuf);

	if (i2BufUsed > *pu2RemainLen) {
		kalMemCopy(*ppucWrPos, pucTemp, *pu2RemainLen);
		pucTemp += *pu2RemainLen;
		i2BufUsed -= *pu2RemainLen;
		*pi8VirtualWrPos += (int64_t)*pu2RemainLen;
		*pu2RemainLen = u2BufSize;
		*ppucWrPos = pucBufStart;
	}
	kalMemCopy(*ppucWrPos, pucTemp, i2BufUsed);
	*ppucWrPos += i2BufUsed;
	*pu2RemainLen -= i2BufUsed;
	*pi8VirtualWrPos += (int64_t)i2BufUsed;
}

#if CFG_SUPPORT_WIFI_POWER_DEBUG
void glNotifyWakeups(void *pvWakeup, enum ENUM_WAKE_UP_T eType)
{
	struct BLOCKED_READING_PROC_T *prBRProc = &rWakeupLogProc;
	uint16_t u2WrLen = (uint16_t)calModulus64(prBRProc->i8WrPos, prBRProc->u2BufLen);
	uint16_t u2RemainLen = prBRProc->u2BufLen - u2WrLen;
	uint8_t *pucRealWrPos = &prBRProc->pucBuf[u2WrLen];
	uint8_t *pucIp;
	uint8_t ucIpVersion;
	uint8_t ucIpProto;
	uint8_t aucAppName[32] = {0};
	uint16_t u2EthType;
	struct sk_buff *prSkb;
#define WRITE_WAKEUP(_fmt, ...)\
		glFormatOutput(TRUE, &prBRProc->i8WrPos, &prBRProc->pucBuf[0], \
				prBRProc->u2BufLen, &pucRealWrPos,&u2RemainLen, _fmt, ##__VA_ARGS__)

#define GET_APP_NAME() \
		kalGetAppNameByEth(g_prGlueInfo_proc, prSkb, aucAppName, sizeof(aucAppName) - 1)

	if (!prBRProc->fgEnabled || !pvWakeup || eType >= WAKE_TYPE_NUM)
		return;

	mutex_lock(&prBRProc->lock);
	switch (eType) {
	case WAKE_TYPE_IP:
	{
		prSkb = (struct sk_buff *)pvWakeup;
		u2EthType = (prSkb->data[ETH_TYPE_LEN_OFFSET] << 8) | (prSkb->data[ETH_TYPE_LEN_OFFSET + 1]);
		pucIp = &prSkb->data[ETH_HLEN];
		ucIpVersion = (pucIp[0] & IPVH_VERSION_MASK) >> IPVH_VERSION_OFFSET;
		if (u2EthType == ETH_P_IPV4) {
			if (ucIpVersion != IP_VERSION_4)
				break;
			ucIpProto = pucIp[9];
			if (ucIpProto == IP_PRO_TCP) {
				if (GET_APP_NAME())
					WRITE_WAKEUP("%s\n", aucAppName);
				else
					WRITE_WAKEUP("TCP4=%pI4, sport=%d,dport=%d\n", &pucIp[12],
							 (pucIp[20] << 8) | pucIp[21], (pucIp[22] << 8) | pucIp[23]);
			} else if (ucIpProto == IP_PRO_UDP) {
				if (GET_APP_NAME())
					WRITE_WAKEUP("%s\n", aucAppName);
				else
					WRITE_WAKEUP("UDP4=%pI4, sport=%d,dport=%d\n", &pucIp[12],
							 (pucIp[20] << 8) | pucIp[21], (pucIp[22] << 8) | pucIp[23]);
			} else {
				WRITE_WAKEUP("IP4=%pI4, proto=%d\n", &pucIp[12], ucIpProto);
			}
		} else if (u2EthType == ETH_P_IPV6) {
			if (ucIpVersion != IP_VERSION_6)
				break;
			ucIpProto = pucIp[6];
			if (ucIpProto == IP_PRO_TCP) {
				if (GET_APP_NAME())
					WRITE_WAKEUP("%s\n", aucAppName);
				else
					WRITE_WAKEUP("TCP6=%pI6c,sport=%d,dport=%d\n", &pucIp[8],
						(pucIp[40] << 8) | pucIp[41], (pucIp[42] << 8) | pucIp[43]);
			} else if (ucIpProto == IP_PRO_UDP) {
				if (GET_APP_NAME())
					WRITE_WAKEUP("%s\n", aucAppName);
				else
					WRITE_WAKEUP("UDP6=%pI6c,sport=%d,dport=%d\n", &pucIp[8],
						(pucIp[40] << 8) | pucIp[41], (pucIp[42] << 8) | pucIp[43]);
			} else {
				WRITE_WAKEUP("IP6=%pI6c, proto=%d\n", &pucIp[8], ucIpProto);
			}
		}
		break;
	}
	case WAKE_TYPE_ARP:
	{
		uint8_t *pucEthBody = (uint8_t *)pvWakeup;
		uint16_t u2OpCode = (pucEthBody[6] << 8) | pucEthBody[7];
		uint8_t *pucOP = NULL;

		if (u2OpCode == 1)
			pucOP = "REQ";
		else if (u2OpCode == 2)
			pucOP = "RSP";

		WRITE_WAKEUP("ARP:%s from %pI4\n", pucOP, &pucEthBody[14]);
		break;
	}
	case WAKE_TYPE_1X:
		WRITE_WAKEUP("1X:eth_type=0x%04x\n", *(uint16_t *)pvWakeup);
		break;
	case WAKE_TYPE_OTHER_DATA:
		WRITE_WAKEUP("OTHER_DATA:eth_type=0x%04x\n", *(uint16_t *)pvWakeup);
		break;
	case WAKE_TYPE_MGMT:
		WRITE_WAKEUP("MGMT:sub_type=0x%02x\n", *(uint8_t *)pvWakeup);
		break;
	case WAKE_TYPE_EVENT:
		WRITE_WAKEUP("EVENT:id=0x%02x\n", *(uint8_t *)pvWakeup);
		break;
	case WAKE_TYPE_UNKNOWN:
		WRITE_WAKEUP("UNKNOWN:packet_type=%d\n", *(uint8_t *)pvWakeup);
		break;
	case WAKE_TYPE_CHARGE_STATUS:
		WRITE_WAKEUP("%s\n", (uint8_t *)pvWakeup);
		break;
	case WAKE_TYPE_NO_PKT_DATA:
		WRITE_WAKEUP("%s is NULL\n", (uint8_t *)pvWakeup);
		break;
	case WAKE_TYPE_INVALID_SW_DEFINED:
		WRITE_WAKEUP("Invalid SW defined Packet 0x%04x\n", *(uint16_t *)pvWakeup);
		break;
	case WAKE_TYPE_BAR:
	{
		uint32_t u4BarInfo = *(uint32_t *)pvWakeup;

		WRITE_WAKEUP("BAR for SSN %d TID %d\n", u4BarInfo & 0xffff, u4BarInfo >> 16);
		break;
	}
	default:
		break;
	}
	mutex_unlock(&prBRProc->lock);
	glProcWakeupThreads(prBRProc, 1);
}

/* sample: IP:xxx.xxx.xxx.xxx;sport:xxxxx;dport:xxxxx */
static ssize_t wakeup_log_read(struct file *file, char __user *buf, size_t count, loff_t *f_pos)
{
#define NOT_ENABLE "Wake-up log is not enabled"
#define TO_ENABLE "echo enable > /proc/net/wlan/"PROC_WAKEUP_LOG" to enable"
#define TO_DISABLE "echo disable > /proc/net/wlan/"PROC_WAKEUP_LOG" to disable\n"

	return procReadBlockedProc(&rWakeupLogProc, buf, !(file->f_flags & O_NONBLOCK), count, f_pos,
				   NOT_ENABLE"\n"TO_ENABLE"\n"TO_DISABLE);
}

static ssize_t wakeup_log_configure(struct file *file, const char __user *buffer,
				size_t count, loff_t *data)
{
	PROC_WRITE_COMMON(buffer, count);
	CONFIGURE_BLOCKED_READING_PROC_ON_OFF(&rWakeupLogProc);
	return -EINVAL;
}

static const struct file_operations wakeup_log_ops = {
	.owner = THIS_MODULE,
	.read = wakeup_log_read,
	.write = wakeup_log_configure,
};

void glNotifyAppTxRx(struct GLUE_INFO *prGlueInfo, char *pcReason)
{
	struct BLOCKED_READING_PROC_T *prBRProc = &rAppTxRxProc;
	uint16_t u2WrLen = (uint16_t)calModulus64(prBRProc->i8WrPos, prBRProc->u2BufLen);
	uint16_t u2RemainLen = prBRProc->u2BufLen - u2WrLen;
	uint8_t *pucRealWrPos = &prBRProc->pucBuf[u2WrLen];
	struct LINK *prAppStatLink;
	struct LINK *prOtherDataLink;
	struct APP_TX_RX_STAT_T *prAppStat = NULL;
	struct OTHER_DATA_STAT_T *prOtherDataStat = NULL;
	struct DRV_PKT_STAT_T *prDrvPktStat = NULL;
	KAL_SPIN_LOCK_DECLARATION();
#define WRITE_APP_TX_RX(_fmt, ...)\
	glFormatOutput(FALSE, &prBRProc->i8WrPos, &prBRProc->pucBuf[0], \
			prBRProc->u2BufLen, &pucRealWrPos, &u2RemainLen, _fmt, ##__VA_ARGS__)

	if (!prGlueInfo || !prBRProc->fgEnabled)
		return;
	prAppStatLink = &prGlueInfo->rAppTxRxStat.rUsingLink;
	prOtherDataLink = &prGlueInfo->rOtherDataStat.rUsingLink;
	prDrvPktStat = &prGlueInfo->arDrvPktStat[0];
	mutex_lock(&prBRProc->lock);
	if (!pcReason)
		WRITE_APP_TX_RX("%s APP T/Rx Statistics before suspend at %s\n", acResumeTime, acSuspendTime);
	else
		glFormatOutput(TRUE, &prBRProc->i8WrPos, &prBRProc->pucBuf[0], prBRProc->u2BufLen,
			       &pucRealWrPos, &u2RemainLen, "APP T/Rx Statistics due to %s\n", pcReason);

	KAL_ACQUIRE_SPIN_LOCK(prGlueInfo->prAdapter, SPIN_LOCK_APP_TRX_STAT);
	LINK_FOR_EACH_ENTRY(prAppStat, prAppStatLink, rLinkEntry, struct APP_TX_RX_STAT_T) {
		if (prAppStat->u4RxStat || prAppStat->u4TxStat) {
			WRITE_APP_TX_RX("%s Tx=%u, Rx=%u\n", prAppStat->acAppName, prAppStat->u4TxStat,
				prAppStat->u4RxStat);
			prAppStat->u4RxStat = prAppStat->u4TxStat = 0;
		}
	}
	KAL_RELEASE_SPIN_LOCK(prGlueInfo->prAdapter, SPIN_LOCK_APP_TRX_STAT);
	KAL_ACQUIRE_SPIN_LOCK(prGlueInfo->prAdapter, SPIN_LOCK_OTHER_DATA_STAT);
	LINK_FOR_EACH_ENTRY(prOtherDataStat, prOtherDataLink, rLinkEntry, struct OTHER_DATA_STAT_T) {
		if (!prOtherDataStat->u4RxStat && !prOtherDataStat->u4TxStat)
			continue;
		if (prOtherDataStat->u2EthType == ETH_P_IPV4 || prOtherDataStat->u2EthType == ETH_P_IPV6)
			WRITE_APP_TX_RX("IP proto 0x%02x Tx=%u, Rx=%u\n", prOtherDataStat->ucIpProto,
				prOtherDataStat->u4TxStat, prOtherDataStat->u4RxStat);
		else
			WRITE_APP_TX_RX("EtherType 0x%04x Tx=%u, Rx=%u\n", prOtherDataStat->u2EthType,
				prOtherDataStat->u4TxStat, prOtherDataStat->u4RxStat);

		prOtherDataStat->u4RxStat = prOtherDataStat->u4TxStat = 0;
	}
	KAL_RELEASE_SPIN_LOCK(prGlueInfo->prAdapter, SPIN_LOCK_OTHER_DATA_STAT);
	if (prDrvPktStat[DRV_PKT_CMD].u4TxStat)
		WRITE_APP_TX_RX("Command 0xff->0x0 %016llX%016llX%016llX%016llX Tx=%u\n",
				prDrvPktStat[DRV_PKT_CMD].aulDrvPktMaps[3],
				prDrvPktStat[DRV_PKT_CMD].aulDrvPktMaps[2],
				prDrvPktStat[DRV_PKT_CMD].aulDrvPktMaps[1],
				prDrvPktStat[DRV_PKT_CMD].aulDrvPktMaps[0],
				prDrvPktStat[DRV_PKT_CMD].u4TxStat);

	KAL_ACQUIRE_SPIN_LOCK(prGlueInfo->prAdapter, SPIN_LOCK_DRV_PKT_STAT);
	if (prDrvPktStat[DRV_PKT_MGMT].u4RxStat || prDrvPktStat[DRV_PKT_MGMT].u4TxStat)
		WRITE_APP_TX_RX("MGMT SubType 15->0 %04X Tx=%u, %04X Rx=%u\n",
				 *(uint16_t *)&prDrvPktStat[DRV_PKT_MGMT].aulDrvPktMaps[0],
				 prDrvPktStat[DRV_PKT_MGMT].u4TxStat,
				 *(uint16_t *)&prDrvPktStat[DRV_PKT_MGMT].aulDrvPktMaps[1],
				 prDrvPktStat[DRV_PKT_MGMT].u4RxStat);

	if (prDrvPktStat[DRV_PKT_EVENT].u4RxStat)
		WRITE_APP_TX_RX("Event 0xff->0x0 %016llX%016llX%016llX%016llX Rx=%u\n",
				prDrvPktStat[DRV_PKT_EVENT].aulDrvPktMaps[3],
				prDrvPktStat[DRV_PKT_EVENT].aulDrvPktMaps[2],
				prDrvPktStat[DRV_PKT_EVENT].aulDrvPktMaps[1],
				prDrvPktStat[DRV_PKT_EVENT].aulDrvPktMaps[0],
				prDrvPktStat[DRV_PKT_EVENT].u4RxStat);

	kalMemZero(prGlueInfo->arDrvPktStat, sizeof(prGlueInfo->arDrvPktStat));
	KAL_RELEASE_SPIN_LOCK(prGlueInfo->prAdapter, SPIN_LOCK_DRV_PKT_STAT);
	mutex_unlock(&prBRProc->lock);
	glProcWakeupThreads(prBRProc, 1);
}

void glLogSuspendResumeTime(u_int8_t fgSuspend)
{
	struct timeval tval;
	struct rtc_time tm;
	char *pcTimeStr;
	uint32_t u4TimeStrLen;

	do_gettimeofday(&tval);
	tval.tv_sec -= sys_tz.tz_minuteswest * 60;
	rtc_time_to_tm(tval.tv_sec, &tm);
	if (fgSuspend) {
		pcTimeStr = &acSuspendTime[0];
		u4TimeStrLen = sizeof(acSuspendTime);
	} else {
		pcTimeStr = &acResumeTime[0];
		u4TimeStrLen = sizeof(acResumeTime);
	}
	kalSnprintf(pcTimeStr, u4TimeStrLen, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
				tm.tm_min, tm.tm_sec, (int32_t)(tval.tv_usec / USEC_PER_MSEC));
}


static ssize_t procAppTRxRead(struct file *file, char __user *buf, size_t count, loff_t *f_pos)
{
#undef NOT_ENABLE
#define NOT_ENABLE "APP TxRx Stat log is not enabled"
#undef TO_ENABLE
#define TO_ENABLE "echo enable > /proc/net/wlan/"PROC_TX_RX_STAT" to enable"
#undef TO_DISABLE
#define TO_DISABLE "echo disable > /proc/net/wlan/"PROC_TX_RX_STAT" to disable"
#define TO_OUTPUT "echo output > /proc/net/wlan/"PROC_TX_RX_STAT" to force output\n"

	return procReadBlockedProc(&rAppTxRxProc, buf, !(file->f_flags & O_NONBLOCK), count, f_pos,
				   NOT_ENABLE"\n"TO_ENABLE"\n"TO_DISABLE"\n"TO_OUTPUT);
}

u_int8_t glIsDataStatEnabled(void)
{
	return rAppTxRxProc.fgEnabled;
}

u_int8_t glIsWakeupLogEnabled(void)
{
	return rWakeupLogProc.fgEnabled;
}

static ssize_t procAppTRxConf(struct file *file, const char __user *buffer,
				size_t count, loff_t *data)
{
	struct BLOCKED_READING_PROC_T *prBRProc = &rAppTxRxProc;
	PROC_WRITE_COMMON(buffer, count);
	if (!prBRProc->fgEnabled) {
		if (!kalStrnCmp(g_aucProcBuf, "enable", 6)) {
			prBRProc->u2BufLen = 8192;
			prBRProc->pucBuf = kalMemAlloc(prBRProc->u2BufLen, VIR_MEM_TYPE);
			if (!prBRProc->pucBuf) {
				return -ENOMEM;
			}
			prBRProc->fgEnabled = TRUE;
			prBRProc->i8WrPos = 0;
			return 6;
		}
		return -EOPNOTSUPP;
	}

	if (!kalStrnCmp(g_aucProcBuf, "disable", 7)) {
		prBRProc->fgEnabled = FALSE;
		glProcWakeupThreads(prBRProc, 1);
		kalMemFree(prBRProc->pucBuf, VIR_MEM_TYPE, prBRProc->u2BufLen);
		prBRProc->pucBuf = NULL;
		prBRProc->u2BufLen = 0;
		return 7;
	}

	if (!kalStrnCmp(g_aucProcBuf, "output", 6)) {
		uint32_t u4InfoLen = 0;

		kalIoctl(g_prGlueInfo_proc, wlanoidNotifyTRxStats, "force output", 11,
			 FALSE, FALSE, FALSE, &u4InfoLen);
		return 6;
	}
	DBGLOG(INIT, WARN, "error sscanf or not supported command\n");
	return -EINVAL;
}

static const struct file_operations app_trx_stat_ops = {
	.owner = THIS_MODULE,
	.read = procAppTRxRead,
	.write = procAppTRxConf,
};
#endif

static ssize_t procDbgLevelRead(struct file *filp, char __user *buf,
	size_t count, loff_t *f_pos)
{
	uint8_t *temp = &g_aucProcBuf[0];
	uint8_t *str = NULL;
	uint32_t u4CopySize = 0;
	uint16_t i;
	uint16_t u2ModuleNum = 0;
	uint32_t u4StrLen = 0;
	uint32_t u4Level1, u4Level2;

	/* if *f_ops>0, we should return 0 to make cat command exit */
	if (*f_pos > 0 || buf == NULL)
		return 0;

	str = "\nTEMP|LOUD|INFO|TRACE | EVENT|STATE|WARN|ERROR\n"
	    "bit7|bit6|bit5|bit4 | bit3|bit2|bit1|bit0\n\n"
	    "Usage: Module Index:Module Level, such as 0x00:0xff\n\n"
	    "Debug Module\tIndex\tLevel\tDebug Module\tIndex\tLevel\n\n";
	u4StrLen = kalStrLen(str);
	kalStrnCpy(temp, str, u4StrLen + 1);
	temp += kalStrLen(temp);

	u2ModuleNum =
	    (sizeof(aucDbModuleName) /
	     PROC_DBG_LEVEL_MAX_DISPLAY_STR_LEN) & 0xfe;

	for (i = 0; i < u2ModuleNum; i += 2) {
		wlanGetDriverDbgLevel(i, &u4Level1);
		wlanGetDriverDbgLevel(i + 1, &u4Level2);
		SNPRINTF(temp, g_aucProcBuf,
			("DBG_%s_IDX\t(0x%02x):\t0x%02x\t"
			 "DBG_%s_IDX\t(0x%02x):\t0x%02x\n",
			 &aucDbModuleName[i][0], i, (uint8_t) u4Level1,
			 &aucDbModuleName[i + 1][0], i + 1,
			 (uint8_t) u4Level2));
	}

	if ((sizeof(aucDbModuleName) /
	     PROC_DBG_LEVEL_MAX_DISPLAY_STR_LEN) & 0x1) {
		wlanGetDriverDbgLevel(u2ModuleNum, &u4Level1);
		SNPRINTF(temp, g_aucProcBuf,
			 ("DBG_%s_IDX\t(0x%02x):\t0x%02x\n",
			  &aucDbModuleName[u2ModuleNum][0], u2ModuleNum,
			  (uint8_t) u4Level1));
	}

	u4CopySize = kalStrLen(g_aucProcBuf);
	if (u4CopySize > count)
		u4CopySize = count;
	if (copy_to_user(buf, g_aucProcBuf, u4CopySize)) {
		pr_err("copy to user failed\n");
		return -EFAULT;
	}

	*f_pos += u4CopySize;
	return (ssize_t) u4CopySize;
}

#if WLAN_INCLUDE_PROC
#if	CFG_SUPPORT_EASY_DEBUG

static void *procEfuseDump_start(struct seq_file *s, loff_t *pos)
{
	static unsigned long counter;

	if (*pos == 0)
		counter = *pos;	/* read file init */

	if (counter >= EFUSE_ADDR_MAX)
		return NULL;
	return &counter;
}

static void *procEfuseDump_next(struct seq_file *s, void *v, loff_t *pos)
{
	unsigned long *tmp_v = (unsigned long *)v;

	(*tmp_v) += EFUSE_BLOCK_SIZE;

	if (*tmp_v >= EFUSE_ADDR_MAX)
		return NULL;
	return tmp_v;
}

static void procEfuseDump_stop(struct seq_file *s, void *v)
{
	/* nothing to do, we use a static value in start() */
}

static int procEfuseDump_show(struct seq_file *s, void *v)
{
	uint32_t rStatus = WLAN_STATUS_SUCCESS;
	uint32_t u4BufLen = 0;
	struct GLUE_INFO *prGlueInfo;
	uint32_t idx_addr, idx_value;
	struct PARAM_CUSTOM_ACCESS_EFUSE rAccessEfuseInfo = { };

	prGlueInfo = g_prGlueInfo_proc;

#if  (CFG_EEPROM_PAGE_ACCESS == 1)
	if (prGlueInfo == NULL) {
		seq_puts(s, "prGlueInfo is null\n");
		return -EPERM;
	}

	if (prGlueInfo->prAdapter &&
	    prGlueInfo->prAdapter->chip_info &&
	    !prGlueInfo->prAdapter->chip_info->is_support_efuse) {
		seq_puts(s, "efuse ops is invalid\n");
		return -EPERM; /* return negative value to stop read process */
	}

	idx_addr = *(loff_t *) v;
	rAccessEfuseInfo.u4Address =
		(idx_addr / EFUSE_BLOCK_SIZE) * EFUSE_BLOCK_SIZE;

	rStatus = kalIoctl(prGlueInfo,
		wlanoidQueryProcessAccessEfuseRead,
		&rAccessEfuseInfo,
		sizeof(struct PARAM_CUSTOM_ACCESS_EFUSE), TRUE, TRUE,
		TRUE, &u4BufLen);
	if (rStatus != WLAN_STATUS_SUCCESS) {
		seq_printf(s, "efuse read fail (0x%03X)\n",
			rAccessEfuseInfo.u4Address);
		return 0;
	}

	for (idx_value = 0; idx_value < EFUSE_BLOCK_SIZE; idx_value++)
		seq_printf(s, "0x%03X=0x%02X\n",
			rAccessEfuseInfo.u4Address + idx_value,
			prGlueInfo->prAdapter->aucEepromVaule[idx_value]);
	return 0;
#else
	seq_puts(s, "efuse ops is invalid\n");
	return -EPERM; /* return negative value to stop read process */
#endif
}

static int procEfuseDumpOpen(struct inode *inode, struct file *file)
{
	static const struct seq_operations procEfuseDump_ops = {
		.start = procEfuseDump_start,
		.next = procEfuseDump_next,
		.stop = procEfuseDump_stop,
		.show = procEfuseDump_show
	};

	return seq_open(file, &procEfuseDump_ops);
}

static ssize_t procCfgRead(struct file *filp, char __user *buf, size_t count,
			   loff_t *f_pos)
{
	uint8_t *temp = &g_aucProcBuf[0];
	uint8_t *str = NULL;
	uint8_t *str2 = "\nERROR DUMP CONFIGURATION:\n";
	uint32_t u4CopySize = 0;
	uint32_t i;
	uint32_t u4StrLen = 0;

#define BUFFER_RESERVE_BYTE 50

	struct GLUE_INFO *prGlueInfo;

	struct WLAN_CFG_ENTRY *prWlanCfgEntry;
	struct ADAPTER *prAdapter;

	prGlueInfo = *((struct GLUE_INFO **)netdev_priv(gPrDev));

	if (!prGlueInfo) {
		pr_err("procCfgRead prGlueInfo is  NULL????\n");
		return 0;
	}

	prAdapter = prGlueInfo->prAdapter;

	if (!prAdapter) {
		pr_err("procCfgRead prAdapter is  NULL????\n");
		return 0;
	}

	/* if *f_ops>0, we should return 0 to make cat command exit */
	if (*f_pos > 0 || buf == NULL)
		return 0;

	str = "\nDUMP CONFIGURATION :\n"
	    "<KEY|VALUE> OR <D:KEY|VALUE>\n"
	    "'D': driver part current setting\n"
	    "===================================\n";
	u4StrLen = kalStrLen(str);
	kalStrnCpy(temp, str, u4StrLen + 1);
	temp += kalStrLen(temp);

	for (i = 0; i < WLAN_CFG_ENTRY_NUM_MAX; i++) {
		prWlanCfgEntry = wlanCfgGetEntryByIndex(prAdapter, i, 0);

		if ((!prWlanCfgEntry) || (prWlanCfgEntry->aucKey[0] == '\0'))
			break;

		SNPRINTF(temp, g_aucProcBuf,
			("%s|%s\n", prWlanCfgEntry->aucKey,
			prWlanCfgEntry->aucValue));

		if ((temp - g_aucProcBuf) != kalStrLen(g_aucProcBuf)) {
			DBGLOG(INIT, ERROR,
			       "Dump configuration error: temp offset=%d, buf length=%u, key[%d]=[%u], val[%d]=[%u]\n",
			       (int)(temp - g_aucProcBuf),
			       (unsigned int)kalStrLen(g_aucProcBuf),
			       WLAN_CFG_KEY_LEN_MAX,
			       (unsigned int)prWlanCfgEntry->aucKey[
				WLAN_CFG_KEY_LEN_MAX - 1],
			       WLAN_CFG_VALUE_LEN_MAX,
			       (unsigned int)prWlanCfgEntry->aucValue[
				WLAN_CFG_VALUE_LEN_MAX - 1]);
			kalMemSet(g_aucProcBuf, ' ', u4StrLen);
			kalStrnCpy(g_aucProcBuf, str2, kalStrLen(str2));
			g_aucProcBuf[u4StrLen-1] = '\n';
			goto procCfgReadLabel;
		}

		if (kalStrLen(g_aucProcBuf) >
			(sizeof(g_aucProcBuf) - BUFFER_RESERVE_BYTE))
			break;
	}

	for (i = 0; i < WLAN_CFG_REC_ENTRY_NUM_MAX; i++) {
		prWlanCfgEntry = wlanCfgGetEntryByIndex(prAdapter, i, 1);

		if ((!prWlanCfgEntry) || (prWlanCfgEntry->aucKey[0] == '\0'))
			break;

		SNPRINTF(temp, g_aucProcBuf,
			("D:%s|%s\n", prWlanCfgEntry->aucKey,
			prWlanCfgEntry->aucValue));

		if ((temp - g_aucProcBuf) != kalStrLen(g_aucProcBuf)) {
			DBGLOG(INIT, ERROR,
			       "D:Dump configuration error: temp offset=%u, buf length=%u, key[%d]=[%u], val[%d]=[%u]\n",
			       (int)(temp - g_aucProcBuf),
			       (unsigned int)kalStrLen(g_aucProcBuf),
			       WLAN_CFG_KEY_LEN_MAX,
			       (unsigned int)prWlanCfgEntry->aucKey[
				WLAN_CFG_KEY_LEN_MAX - 1],
			       WLAN_CFG_VALUE_LEN_MAX,
			       (unsigned int)prWlanCfgEntry->aucValue[
				WLAN_CFG_VALUE_LEN_MAX - 1]);
			kalMemSet(g_aucProcBuf, ' ', u4StrLen);
			kalStrnCpy(g_aucProcBuf, str2, kalStrLen(str2));
			g_aucProcBuf[u4StrLen-1] = '\n';
			goto procCfgReadLabel;
		}

		if (kalStrLen(g_aucProcBuf) >
			(sizeof(g_aucProcBuf) - BUFFER_RESERVE_BYTE))
			break;
	}

procCfgReadLabel:
	u4CopySize = kalStrLen(g_aucProcBuf);
	if (u4CopySize > count)
		u4CopySize = count;
	if (copy_to_user(buf, g_aucProcBuf, u4CopySize)) {
		pr_err("copy to user failed\n");
		return -EFAULT;
	}

	*f_pos += u4CopySize;
	return (ssize_t) u4CopySize;
}

static ssize_t procCfgWrite(struct file *file, const char __user *buffer,
	size_t count, loff_t *data)
{
	uint32_t u4CopySize = sizeof(g_aucProcBuf)-8;
	struct GLUE_INFO *prGlueInfo;
	uint8_t *pucTmp;
	uint32_t i = 0;

#if CFG_CHIP_RESET_SUPPORT
	if (!g_prGlueInfo_proc) {
		DBGLOG(INIT, ERROR, "g_prGlueInfo_proc is null\n");
		return 0;
	}
	if (!wlanIsDriverReady(g_prGlueInfo_proc)) {
		DBGLOG(INIT, ERROR,
		       "driver is not ready: u4ReadyFlag=%u, kalIsResetting()=%d\n",
		       g_prGlueInfo_proc->u4ReadyFlag, kalIsResetting());
		return 0; /* return 0 to make command exit */
	}
#endif

	kalMemSet(g_aucProcBuf, 0, u4CopySize);
	u4CopySize = (count < u4CopySize) ? count : (u4CopySize - 1);

	pucTmp = g_aucProcBuf;
	SNPRINTF(pucTmp, g_aucProcBuf, ("%s ", "set_cfg"));

	if (copy_from_user(pucTmp, buffer, u4CopySize)) {
		pr_err("error of copy from user\n");
		return -EFAULT;
	}
	g_aucProcBuf[u4CopySize + 8] = '\0';

	for (i = 8 ; i < u4CopySize+8; i++) {
		if (!isalnum(g_aucProcBuf[i]) && /* alphanumeric */
			g_aucProcBuf[i] != 0x20 && /* space */
			g_aucProcBuf[i] != 0x0a && /* control char */
			g_aucProcBuf[i] != 0x0d) {
			DBGLOG(INIT, ERROR, "wrong char[%d] 0x%x\n",
				i, g_aucProcBuf[i]);
			return -EFAULT;
		}
	}

	prGlueInfo = g_prGlueInfo_proc;
	/* if g_u4NextDriverReadLen >0,
	 * the content for next DriverCmdRead will be
	 * in : g_aucProcBuf with length : g_u4NextDriverReadLen
	 */
	g_u4NextDriverReadLen =
		priv_driver_set_cfg(prGlueInfo->prDevHandler, g_aucProcBuf,
		sizeof(g_aucProcBuf));

	return count;

}

static ssize_t procDriverCmdRead(struct file *filp, char __user *buf,
	size_t count, loff_t *f_pos)
{
	/* DriverCmd read should only be executed right after
	 * a DriverCmd write because content buffer 'g_aucProcBuf'
	 * is a global buffer for all proc command, otherwise ,
	 * the content could be overwrite by other proc command
	 */
	uint32_t u4CopySize = 0;

	/* if *f_ops>0, we should return 0 to make cat command exit */
	if (*f_pos > 0 || buf == NULL)
		return 0;

	if (g_u4NextDriverReadLen > 0)	/* Detect content to show */
		u4CopySize = g_u4NextDriverReadLen;

	if (u4CopySize > count) {
		pr_err("count is too small: u4CopySize=%u, count=%u\n",
		       u4CopySize, (uint32_t)count);
		return -EFAULT;
	}

	if (copy_to_user(buf, g_aucProcBuf, u4CopySize)) {
		pr_err("copy to user failed\n");
		return -EFAULT;
	}
	g_u4NextDriverReadLen = 0;

	*f_pos += u4CopySize;
	return (ssize_t) u4CopySize;
}



static ssize_t procDriverCmdWrite(struct file *file, const char __user *buffer,
	size_t count, loff_t *data)
{
	uint32_t u4CopySize = sizeof(g_aucProcBuf);
	struct GLUE_INFO *prGlueInfo;

#if CFG_CHIP_RESET_SUPPORT
	if (!g_prGlueInfo_proc) {
		DBGLOG(INIT, ERROR, "g_prGlueInfo_proc is null\n");
		return 0;
	}
	if (!wlanIsDriverReady(g_prGlueInfo_proc)) {
		DBGLOG(INIT, WARN,
		       "driver is not ready: u4ReadyFlag=%u, kalIsResetting()=%d\n",
		       g_prGlueInfo_proc->u4ReadyFlag, kalIsResetting());
		return 0; /* return 0 to make command exit */
	}
#endif

	kalMemSet(g_aucProcBuf, 0, u4CopySize);
	u4CopySize = (count < u4CopySize) ? count : (u4CopySize - 1);

	if (copy_from_user(g_aucProcBuf, buffer, u4CopySize)) {
		pr_err("error of copy from user\n");
		return -EFAULT;
	}
	g_aucProcBuf[u4CopySize] = '\0';


	prGlueInfo = g_prGlueInfo_proc;
	/* if g_u4NextDriverReadLen >0,
	 * the content for next DriverCmdRead will be
	 *  in : g_aucProcBuf with length : g_u4NextDriverReadLen
	 */
	g_u4NextDriverReadLen =
		priv_driver_cmds(prGlueInfo->prDevHandler, g_aucProcBuf,
		sizeof(g_aucProcBuf));

	return count;
}
#endif
#endif

static ssize_t procDbgLevelWrite(struct file *file, const char __user *buffer,
	size_t count, loff_t *data)
{
	uint32_t u4NewDbgModule, u4NewDbgLevel;
	uint8_t *temp = &g_aucProcBuf[0];
	uint32_t u4CopySize = sizeof(g_aucProcBuf);

	kalMemSet(g_aucProcBuf, 0, u4CopySize);
	u4CopySize = (count < u4CopySize) ? count : (u4CopySize - 1);

	if (copy_from_user(g_aucProcBuf, buffer, u4CopySize)) {
		pr_err("error of copy from user\n");
		return -EFAULT;
	}
	g_aucProcBuf[u4CopySize] = '\0';

	while (temp) {
		if (sscanf(temp,
			"0x%x:0x%x", &u4NewDbgModule, &u4NewDbgLevel) != 2) {
			pr_info("debug module and debug level should be one byte in length\n");
			break;
		}
		if (u4NewDbgModule == 0xFF) {
			wlanSetDriverDbgLevel(DBG_ALL_MODULE_IDX,
					(u4NewDbgLevel & DBG_CLASS_MASK));
			break;
		}
		if (u4NewDbgModule >= DBG_MODULE_NUM) {
			pr_info("debug module index should less than %d\n",
				DBG_MODULE_NUM);
			break;
		}
		wlanSetDriverDbgLevel(u4NewDbgModule,
				(u4NewDbgLevel & DBG_CLASS_MASK));
		temp = kalStrChr(temp, ',');
		if (!temp)
			break;
		temp++;		/* skip ',' */
	}
	return count;
}

static const struct file_operations dbglevel_ops = {
	.owner = THIS_MODULE,
	.read = procDbgLevelRead,
	.write = procDbgLevelWrite,
};

#if WLAN_INCLUDE_PROC
#if	CFG_SUPPORT_EASY_DEBUG

static const struct file_operations efusedump_ops = {
	.owner = THIS_MODULE,
	.open = procEfuseDumpOpen,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static const struct file_operations drivercmd_ops = {
	.owner = THIS_MODULE,
	.read = procDriverCmdRead,
	.write = procDriverCmdWrite,
};

static const struct file_operations cfg_ops = {
	.owner = THIS_MODULE,
	.read = procCfgRead,
	.write = procCfgWrite,
};
#endif
#endif

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */
/*----------------------------------------------------------------------------*/
/*!
 * \brief The PROC function for reading MCR register to User Space, the offset
 *        of the MCR is specified in u4McrOffset.
 *
 * \param[in] page       Buffer provided by kernel.
 * \param[in out] start  Start Address to read(3 methods).
 * \param[in] off        Offset.
 * \param[in] count      Allowable number to read.
 * \param[out] eof       End of File indication.
 * \param[in] data       Pointer to the private data structure.
 *
 * \return number of characters print to the buffer from User Space.
 */
/*----------------------------------------------------------------------------*/
static ssize_t procMCRRead(struct file *filp, char __user *buf,
	 size_t count, loff_t *f_pos)
{
	struct GLUE_INFO *prGlueInfo;
	struct PARAM_CUSTOM_MCR_RW_STRUCT rMcrInfo;
	uint32_t u4BufLen;
	uint32_t u4Count;
	uint8_t *temp = &g_aucProcBuf[0];
	uint32_t rStatus = WLAN_STATUS_SUCCESS;

	/* Kevin: Apply PROC read method 1. */
	if (*f_pos > 0)
		return 0;	/* To indicate end of file. */

#if CFG_CHIP_RESET_SUPPORT
	if (!g_prGlueInfo_proc) {
		DBGLOG(INIT, ERROR, "g_prGlueInfo_proc is null\n");
		return 0;
	}
	if (!wlanIsDriverReady(g_prGlueInfo_proc)) {
		DBGLOG(INIT, ERROR,
		       "driver is not ready: u4ReadyFlag=%u, kalIsResetting()=%d\n",
		       g_prGlueInfo_proc->u4ReadyFlag, kalIsResetting());
		return 0; /* return 0 to make command exit */
	}
#endif

	prGlueInfo = g_prGlueInfo_proc;

	rMcrInfo.u4McrOffset = u4McrOffset;

	rStatus = kalIoctl(prGlueInfo,
		wlanoidQueryMcrRead, (void *)&rMcrInfo,
		sizeof(rMcrInfo), TRUE, TRUE, TRUE, &u4BufLen);
	kalMemZero(g_aucProcBuf, sizeof(g_aucProcBuf));
	SNPRINTF(temp, g_aucProcBuf,
		("MCR (0x%08xh): 0x%08x\n", rMcrInfo.u4McrOffset,
		rMcrInfo.u4McrData));

	u4Count = kalStrLen(g_aucProcBuf);
	if (u4Count > count)
		u4Count = count;

	if (copy_to_user(buf, g_aucProcBuf, u4Count)) {
		pr_err("copy to user failed\n");
		return -EFAULT;
	}

	*f_pos += u4Count;

	return (int)u4Count;

} /* end of procMCRRead() */

/*----------------------------------------------------------------------------*/
/*!
 * \brief The PROC function for writing MCR register to HW or update u4McrOffset
 *        for reading MCR later.
 *
 * \param[in] file   pointer to file.
 * \param[in] buffer Buffer from user space.
 * \param[in] count  Number of characters to write
 * \param[in] data   Pointer to the private data structure.
 *
 * \return number of characters write from User Space.
 */
/*----------------------------------------------------------------------------*/
static ssize_t procMCRWrite(struct file *file, const char __user *buffer,
	size_t count, loff_t *data)
{
	struct GLUE_INFO *prGlueInfo;
	/* + 1 for "\0" */
	char acBuf[PROC_MCR_ACCESS_MAX_USER_INPUT_LEN + 1];
	int i4CopySize;
	struct PARAM_CUSTOM_MCR_RW_STRUCT rMcrInfo;
	uint32_t u4BufLen;
	uint32_t rStatus = WLAN_STATUS_SUCCESS;
	int num = 0;

#if CFG_CHIP_RESET_SUPPORT
	if (!g_prGlueInfo_proc) {
		DBGLOG(INIT, ERROR, "g_prGlueInfo_proc is null\n");
		return 0;
	}
	if (!wlanIsDriverReady(g_prGlueInfo_proc)) {
		DBGLOG(INIT, ERROR,
		       "driver is not ready: u4ReadyFlag=%u, kalIsResetting()=%d\n",
		       g_prGlueInfo_proc->u4ReadyFlag, kalIsResetting());
		return 0; /* return 0 to make command exit */
	}
#endif

	ASSERT(data);

	i4CopySize =
	    (count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
	if (copy_from_user(acBuf, buffer, i4CopySize))
		return 0;
	acBuf[i4CopySize] = '\0';

	num =
	    sscanf(acBuf, "0x%x 0x%x", &rMcrInfo.u4McrOffset,
		   &rMcrInfo.u4McrData);
	switch (num) {
	case 2:
		/* NOTE: Sometimes we want to test if bus will still be ok,
		 * after accessing the MCR which is not align to DW boundary.
		 */
		/* if (IS_ALIGN_4(rMcrInfo.u4McrOffset)) */
		{
			prGlueInfo =
			    (struct GLUE_INFO *)
			    netdev_priv((struct net_device *)data);

			u4McrOffset = rMcrInfo.u4McrOffset;

			/* printk("Write 0x%lx to MCR 0x%04lx\n", */
			/* rMcrInfo.u4McrOffset, rMcrInfo.u4McrData); */

			rStatus = kalIoctl(prGlueInfo,
					   wlanoidSetMcrWrite,
					   (void *)&rMcrInfo, sizeof(rMcrInfo),
					   FALSE, FALSE, TRUE, &u4BufLen);

		}
		break;
	case 1:
		/* if (IS_ALIGN_4(rMcrInfo.u4McrOffset)) */
		{
			u4McrOffset = rMcrInfo.u4McrOffset;
		}
		break;

	default:
		break;
	}

	return count;

}				/* end of procMCRWrite() */

static const struct file_operations mcr_ops = {
	.owner = THIS_MODULE,
	.read = procMCRRead,
	.write = procMCRWrite,
};

#if CFG_SUPPORT_SET_CAM_BY_PROC
static ssize_t procSetCamCfgWrite(struct file *file, const char __user *buffer,
	size_t count, loff_t *data)
{
#define MODULE_NAME_LEN_1 5

	uint32_t u4CopySize = sizeof(g_aucProcBuf);
	uint8_t *temp = &g_aucProcBuf[0];
	u_int8_t fgSetCamCfg = FALSE;
	uint8_t aucModule[MODULE_NAME_LEN_1];
	uint32_t u4Enabled;
	uint8_t aucModuleArray[MODULE_NAME_LEN_1] = "CAM";
	u_int8_t fgParamValue = TRUE;
	struct GLUE_INFO *prGlueInfo = NULL;
	struct ADAPTER *prAdapter = NULL;

#if CFG_CHIP_RESET_SUPPORT
	if (!g_prGlueInfo_proc) {
		DBGLOG(INIT, ERROR, "g_prGlueInfo_proc is null\n");
		return 0;
	}
	if (!wlanIsDriverReady(g_prGlueInfo_proc)) {
		DBGLOG(INIT, ERROR,
		       "driver is not ready: u4ReadyFlag=%u, kalIsResetting()=%d\n",
		       g_prGlueInfo_proc->u4ReadyFlag, kalIsResetting());
		return 0; /* return 0 to make command exit */
	}
#endif

	kalMemSet(g_aucProcBuf, 0, u4CopySize);
	u4CopySize = (count < u4CopySize) ? count : (u4CopySize - 1);

	if (copy_from_user(g_aucProcBuf, buffer, u4CopySize)) {
		pr_err("error of copy from user\n");
		return -EFAULT;
	}
	g_aucProcBuf[u4CopySize] = '\0';
	temp = &g_aucProcBuf[0];
	while (temp) {
		kalMemSet(aucModule, 0, MODULE_NAME_LEN_1);

		/* pick up a string and teminated after meet : */
		if (sscanf(temp, "%4s %d", aucModule, &u4Enabled) != 2) {
			pr_info("read param fail, aucModule=%s\n", aucModule);
			fgParamValue = FALSE;
			break;
		}

		if (kalStrnCmp
			(aucModule, aucModuleArray, MODULE_NAME_LEN_1) == 0) {
			if (u4Enabled)
				fgSetCamCfg = TRUE;
			else
				fgSetCamCfg = FALSE;
		}
		temp = kalStrChr(temp, ',');
		if (!temp)
			break;
		temp++;		/* skip ',' */
	}

	if (fgParamValue) {
		prGlueInfo = wlanGetGlueInfo();
		if (!prGlueInfo)
			return count;

		prAdapter = prGlueInfo->prAdapter;
		if (!prAdapter)
			return count;

		nicConfigProcSetCamCfgWrite(prAdapter, fgSetCamCfg);
	}

	return count;
}

static const struct file_operations proc_set_cam_ops = {
	.owner = THIS_MODULE,
	.write = procSetCamCfgWrite,
};
#endif /*CFG_SUPPORT_SET_CAM_BY_PROC */

static ssize_t procPktDelayDbgCfgRead(struct file *filp, char __user *buf,
	size_t count, loff_t *f_pos)
{
	uint8_t *temp = &g_aucProcBuf[0];
	uint8_t *str = NULL;
	uint32_t u4CopySize = 0;
	uint8_t ucTxRxFlag;
	uint8_t ucTxIpProto;
	uint16_t u2TxUdpPort;
	uint32_t u4TxDelayThreshold;
	uint8_t ucRxIpProto;
	uint16_t u2RxUdpPort;
	uint32_t u4RxDelayThreshold;
	uint32_t u4StrLen = 0;

	/* if *f_ops>0, we should return 0 to make cat command exit */
	if (*f_pos > 0 || buf == NULL)
		return 0;

	str = "\nUsage: txLog/rxLog/reset 1(ICMP)/6(TCP)/11(UDP) Dst/SrcPortNum DelayThreshold(us)\n"
		"Print tx delay log,                                   such as: echo txLog 0 0 0 > pktDelay\n"
		"Print tx UDP delay log,                               such as: echo txLog 11 0 0 > pktDelay\n"
		"Print tx UDP dst port19305 delay log,                 such as: echo txLog 11 19305 0 > pktDelay\n"
		"Print rx UDP src port19305 delay more than 500us log, such as: echo rxLog 11 19305 500 > pktDelay\n"
		"Print tx TCP delay more than 500us log,               such as: echo txLog 6 0 500 > pktDelay\n"
		"Close log,                                            such as: echo reset 0 0 0 > pktDelay\n\n";
	u4StrLen = kalStrLen(str);
	kalStrnCpy(temp, str, u4StrLen + 1);
	temp += kalStrLen(temp);

	StatsEnvGetPktDelay(&ucTxRxFlag, &ucTxIpProto, &u2TxUdpPort,
			&u4TxDelayThreshold, &ucRxIpProto, &u2RxUdpPort,
			&u4RxDelayThreshold);

	if (ucTxRxFlag & BIT(0)) {
		SNPRINTF(temp, g_aucProcBuf,
			("txLog %x %d %d\n", ucTxIpProto, u2TxUdpPort,
			u4TxDelayThreshold));
		temp += kalStrLen(temp);
	}
	if (ucTxRxFlag & BIT(1)) {
		SNPRINTF(temp, g_aucProcBuf,
			("rxLog %x %d %d\n", ucRxIpProto, u2RxUdpPort,
			u4RxDelayThreshold));
		temp += kalStrLen(temp);
	}
	if (ucTxRxFlag == 0)
		SNPRINTF(temp, g_aucProcBuf,
			("reset 0 0 0, there is no tx/rx delay log\n"));

	u4CopySize = kalStrLen(g_aucProcBuf);
	if (u4CopySize > count)
		u4CopySize = count;
	if (copy_to_user(buf, g_aucProcBuf, u4CopySize)) {
		pr_err("copy to user failed\n");
		return -EFAULT;
	}

	*f_pos += u4CopySize;
	return (ssize_t) u4CopySize;
}

static ssize_t procPktDelayDbgCfgWrite(struct file *file, const char *buffer,
	size_t count, loff_t *data)
{
#define MODULE_NAME_LENGTH 7
#define MODULE_RESET 0
#define MODULE_TX 1
#define MODULE_RX 2

	uint32_t u4CopySize = sizeof(g_aucProcBuf);
	uint8_t *temp = &g_aucProcBuf[0];
	uint8_t aucModule[MODULE_NAME_LENGTH];
	uint32_t u4DelayThreshold = 0;
	uint32_t u4PortNum = 0;
	uint32_t u4IpProto = 0;
	uint8_t aucResetArray[MODULE_NAME_LENGTH] = "reset";
	uint8_t aucTxArray[MODULE_NAME_LENGTH] = "txLog";
	uint8_t aucRxArray[MODULE_NAME_LENGTH] = "rxLog";
	uint8_t ucTxOrRx = 0;

	kalMemSet(g_aucProcBuf, 0, u4CopySize);
	u4CopySize = (count < u4CopySize) ? count : (u4CopySize - 1);

	if (copy_from_user(g_aucProcBuf, buffer, u4CopySize)) {
		pr_err("error of copy from user\n");
		return -EFAULT;
	}
	g_aucProcBuf[u4CopySize] = '\0';

	while (temp) {
		kalMemSet(aucModule, 0, MODULE_NAME_LENGTH);

		/* pick up a string and teminated after meet : */
		if (sscanf
		    (temp, "%6s %x %d %d", aucModule, &u4IpProto, &u4PortNum,
		     &u4DelayThreshold) != 4) {
			pr_info("read param fail, aucModule=%s\n", aucModule);
			break;
		}

		if (kalStrnCmp
			(aucModule, aucResetArray, MODULE_NAME_LENGTH) == 0) {
			ucTxOrRx = MODULE_RESET;
		} else if (kalStrnCmp
			(aucModule, aucTxArray, MODULE_NAME_LENGTH) == 0) {
			ucTxOrRx = MODULE_TX;
		} else if (kalStrnCmp
			(aucModule, aucRxArray, MODULE_NAME_LENGTH) == 0) {
			ucTxOrRx = MODULE_RX;
		} else {
			pr_info("input module error!\n");
			break;
		}

		temp = kalStrChr(temp, ',');
		if (!temp)
			break;
		temp++;		/* skip ',' */
	}

	StatsEnvSetPktDelay(ucTxOrRx, (uint8_t) u4IpProto, (uint16_t) u4PortNum,
		u4DelayThreshold);

	return count;
}

static const struct file_operations proc_pkt_delay_dbg_ops = {
	.owner = THIS_MODULE,
	.read = procPktDelayDbgCfgRead,
	.write = procPktDelayDbgCfgWrite,
};

#if CFG_SUPPORT_DEBUG_FS
static ssize_t procRoamRead(struct file *filp, char __user *buf,
	size_t count, loff_t *f_pos)
{
	uint32_t u4CopySize;
	uint32_t rStatus;
	uint32_t u4BufLen;

	/* if *f_pos > 0, it means has read successed last time,
	 * don't try again
	 */
	if (*f_pos > 0 || buf == NULL)
		return 0;

#if CFG_CHIP_RESET_SUPPORT
	if (!g_prGlueInfo_proc) {
		DBGLOG(INIT, ERROR, "g_prGlueInfo_proc is null\n");
		return 0;
	}
	if (!wlanIsDriverReady(g_prGlueInfo_proc)) {
		DBGLOG(INIT, ERROR,
		       "driver is not ready: u4ReadyFlag=%u, kalIsResetting()=%d\n",
		       g_prGlueInfo_proc->u4ReadyFlag, kalIsResetting());
		return 0; /* return 0 to make command exit */
	}
#endif

	rStatus =
	    kalIoctl(g_prGlueInfo_proc, wlanoidGetRoamParams, g_aucProcBuf,
		     sizeof(g_aucProcBuf), TRUE, FALSE, TRUE, &u4BufLen);
	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(INIT, INFO, "failed to read roam params\n");
		return -EINVAL;
	}

	u4CopySize = kalStrLen(g_aucProcBuf);
	if (u4CopySize > count)
		u4CopySize = count;

	if (copy_to_user(buf, g_aucProcBuf, u4CopySize)) {
		pr_err("copy to user failed\n");
		return -EFAULT;
	}
	*f_pos += u4CopySize;

	return (int32_t) u4CopySize;
}

static ssize_t procRoamWrite(struct file *file, const char __user *buffer,
	size_t count, loff_t *data)
{
	uint32_t rStatus;
	uint32_t u4BufLen = 0;
	uint32_t u4CopySize = sizeof(g_aucProcBuf);

#if CFG_CHIP_RESET_SUPPORT
	if (!g_prGlueInfo_proc) {
		DBGLOG(INIT, ERROR, "g_prGlueInfo_proc is null\n");
		return 0;
	}
	if (!wlanIsDriverReady(g_prGlueInfo_proc)) {
		DBGLOG(INIT, ERROR,
		       "driver is not ready: u4ReadyFlag=%u, kalIsResetting()=%d\n",
		       g_prGlueInfo_proc->u4ReadyFlag, kalIsResetting());
		return 0; /* return 0 to make command exit */
	}
#endif

	kalMemSet(g_aucProcBuf, 0, u4CopySize);
	u4CopySize = (count < u4CopySize) ? count : (u4CopySize - 1);

	if (copy_from_user(g_aucProcBuf, buffer, u4CopySize)) {
		pr_err("error of copy from user\n");
		return -EFAULT;
	}
	g_aucProcBuf[u4CopySize] = '\0';

	if (kalStrnCmp(g_aucProcBuf, "force_roam", 10) == 0)
		rStatus =
		    kalIoctl(g_prGlueInfo_proc, wlanoidSetForceRoam, NULL, 0,
			     FALSE, FALSE, TRUE, &u4BufLen);
	else
		rStatus =
		    kalIoctl(g_prGlueInfo_proc, wlanoidSetRoamParams,
			     g_aucProcBuf, kalStrLen(g_aucProcBuf), FALSE,
			     FALSE, TRUE, &u4BufLen);

	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(INIT, INFO, "failed to set roam params: %s\n",
		       g_aucProcBuf);
		return -EINVAL;
	}
	return count;
}

static const struct file_operations roam_ops = {
	.owner = THIS_MODULE,
	.read = procRoamRead,
	.write = procRoamWrite,
};
#endif

static ssize_t procCountryRead(struct file *filp, char __user *buf,
	size_t count, loff_t *f_pos)
{
	uint32_t u4CopySize;
	uint16_t u2CountryCode = 0;

	/* if *f_pos > 0, it means has read successed last time,
	 * don't try again
	 */
	if (*f_pos > 0 || buf == NULL)
		return 0;

	if (g_prGlueInfo_proc && g_prGlueInfo_proc->prAdapter) {
		u2CountryCode = g_prGlueInfo_proc->prAdapter->rWifiVar.
			rConnSettings.u2CountryCode;
	}

	if (u2CountryCode)
		kalSprintf(g_aucProcBuf, "Current Country Code: %c%c\n",
			(u2CountryCode >> 8) & 0xff, u2CountryCode & 0xff);
	else
		kalStrnCpy(g_aucProcBuf, "Current Country Code: NULL\n",
			strlen("Current Country Code: NULL\n") + 1);

	u4CopySize = kalStrLen(g_aucProcBuf);
	if (u4CopySize > count)
		u4CopySize = count;

	if (copy_to_user(buf, g_aucProcBuf, u4CopySize)) {
		pr_info("copy to user failed\n");
		return -EFAULT;
	}
	*f_pos += u4CopySize;

	return (int32_t) u4CopySize;
}

static ssize_t procCountryWrite(struct file *file, const char __user *buffer,
	size_t count, loff_t *data)
{
	uint32_t u4BufLen = 0;
	uint32_t rStatus;
	uint32_t u4CopySize = sizeof(g_aucProcBuf);

#if CFG_CHIP_RESET_SUPPORT
	if (!g_prGlueInfo_proc) {
		DBGLOG(INIT, ERROR, "g_prGlueInfo_proc is null\n");
		return 0;
	}
	if (!wlanIsDriverReady(g_prGlueInfo_proc)) {
		DBGLOG(INIT, ERROR,
		       "driver is not ready: u4ReadyFlag=%u, kalIsResetting()=%d\n",
		       g_prGlueInfo_proc->u4ReadyFlag, kalIsResetting());
		return 0; /* return 0 to make command exit */
	}
#endif

	kalMemSet(g_aucProcBuf, 0, u4CopySize);
	u4CopySize = (count < u4CopySize) ? count : (u4CopySize - 1);

	if (copy_from_user(g_aucProcBuf, buffer, u4CopySize)) {
		pr_err("error of copy from user\n");
		return -EFAULT;
	}
	g_aucProcBuf[u4CopySize] = '\0';

	rStatus = kalIoctl(g_prGlueInfo_proc, wlanoidSetCountryCode,
			   &g_aucProcBuf[0], 2, FALSE, FALSE, TRUE, &u4BufLen);
	if (rStatus != WLAN_STATUS_SUCCESS) {
		DBGLOG(INIT, INFO, "failed set country code: %s\n",
			g_aucProcBuf);
		return -EINVAL;
	}
	return count;
}

static const struct file_operations country_ops = {
	.owner = THIS_MODULE,
	.read = procCountryRead,
	.write = procCountryWrite,
};

static ssize_t procAutoPerfCfgRead(struct file *filp, char __user *buf,
	size_t count, loff_t *f_pos)
{
	uint8_t *temp = &g_aucProcBuf[0];
	uint8_t *str = NULL;
	uint32_t u4CopySize = 0;
	uint32_t u4StrLen = 0;

	/* if *f_ops>0, we should return 0 to make cat command exit */
	if (*f_pos > 0)
		return 0;

	str = "Auto Performance Configure Usage:\n"
	    "\n"
	    "echo ForceEnable:0 or 1 > /proc/net/wlan/autoPerfCfg\n"
	    "     1: always enable performance monitor\n"
	    "     0: restore performance monitor's default strategy\n";
	u4StrLen = kalStrLen(str);
	kalStrnCpy(temp, str, u4StrLen + 1);

	u4CopySize = kalStrLen(g_aucProcBuf);
	if (u4CopySize > count)
		u4CopySize = count;

	if (copy_to_user(buf, g_aucProcBuf, u4CopySize)) {
		DBGLOG(INIT, WARN, "copy_to_user error\n");
		return -EFAULT;
	}

	*f_pos += u4CopySize;
	return (ssize_t) u4CopySize;
}

static ssize_t procAutoPerfCfgWrite(struct file *file, const char *buffer,
	size_t count, loff_t *data)
{
	uint32_t u4CoreNum = 0;
	uint32_t u4CoreFreq = 0;
	uint8_t *temp = &g_aucProcBuf[0];
	uint32_t u4CopySize = count;
	uint8_t i = 0;
	uint32_t u4ForceEnable = 0;
	uint8_t aucBuf[32];

#if CFG_CHIP_RESET_SUPPORT
	if (!g_prGlueInfo_proc) {
		DBGLOG(INIT, ERROR, "g_prGlueInfo_proc is null\n");
		return 0;
	}
	if (!wlanIsDriverReady(g_prGlueInfo_proc)) {
		DBGLOG(INIT, ERROR,
		       "driver is not ready: u4ReadyFlag=%u, kalIsResetting()=%d\n",
		       g_prGlueInfo_proc->u4ReadyFlag, kalIsResetting());
		return 0; /* return 0 to make command exit */
	}
#endif

	if (u4CopySize >= sizeof(g_aucProcBuf))
		u4CopySize = sizeof(g_aucProcBuf) - 1;

	kalMemSet(g_aucProcBuf, 0, u4CopySize);

	if (copy_from_user(g_aucProcBuf, buffer, u4CopySize)) {
		DBGLOG(INIT, WARN, "copy_from_user error\n");
		return -EFAULT;
	}

	g_aucProcBuf[u4CopySize] = '\0';

	i = sscanf(temp, "%d:%d", &u4CoreNum, &u4CoreFreq);
	if (i == 2) {
		DBGLOG(INIT, INFO, "u4CoreNum:%d, u4CoreFreq:%d\n", u4CoreNum,
			u4CoreFreq);
		kalSetCpuNumFreq(u4CoreNum, u4CoreFreq);
		return u4CopySize;
	}

	if (strlen(temp) > sizeof(aucBuf)) {
		DBGLOG(INIT, WARN,
			"input string(%s) len is too long, over %d\n",
			g_aucProcBuf, (uint32_t) sizeof(aucBuf));
		return -EFAULT;
	}

	i = sscanf(temp, "%11s:%d", aucBuf, &u4ForceEnable);

	if ((i == 2) && strstr(aucBuf, "ForceEnable")) {
		kalPerMonSetForceEnableFlag(u4ForceEnable);
		return u4CopySize;
	}

	DBGLOG(INIT, WARN, "parameter format should be ForceEnable:0 or 1\n");

	return -EFAULT;
}

static const struct file_operations auto_perf_ops = {
	.owner = THIS_MODULE,
	.read = procAutoPerfCfgRead,
	.write = procAutoPerfCfgWrite,
};

/* Provide a real-time monitor mechanism to end-user to monitor wlan status */
void glNotifyDrvStatus(enum DRV_STATUS_T eDrvStatus, void *pvInfo)
{
	struct BLOCKED_READING_PROC_T *prBRProc = &rDrvStatusProc;
	uint16_t u2WrLen = (uint16_t)calModulus64(prBRProc->i8WrPos, prBRProc->u2BufLen);
	uint16_t u2RemainLen = prBRProc->u2BufLen - u2WrLen;
	uint8_t *pucRealWrPos = &prBRProc->pucBuf[u2WrLen];
#define WRITE_STATUS(_fmt, ...)\
	glFormatOutput(TRUE, &prBRProc->i8WrPos, &prBRProc->pucBuf[0], \
			prBRProc->u2BufLen, &pucRealWrPos, &u2RemainLen, _fmt, ##__VA_ARGS__)

	if (!prBRProc->fgEnabled)
		return;

	mutex_lock(&prBRProc->lock);
	switch (eDrvStatus) {
	case SND_BTM_QUERY:
		WRITE_STATUS("Send BTM query to %pM\n", (uint8_t *)pvInfo);
		break;
	case SND_NEI_REQ:
		WRITE_STATUS("Send Neighbor req to %pM\n", (uint8_t *)pvInfo);
		break;
	case SND_NEI_REQ_TIMEOUT:
		WRITE_STATUS("Neighbor req is timeout.(100ms)\n");
		break;
	case UNSOL_BTM_REQ:
	case SOL_BTM_REQ:
	{
		struct AIS_SPECIFIC_BSS_INFO *prAisSpecBssInfo =
			(struct AIS_SPECIFIC_BSS_INFO *)pvInfo;
		struct LINK *prApList =
			&prAisSpecBssInfo->rNeighborApList.rUsingLink;
		struct NEIGHBOR_AP_T *prNeighborAP = NULL;

		if (!prAisSpecBssInfo) {
			DBGLOG(INIT, ERROR, "prAisSpecBssInfo is NULL\n");
			break;
		}
		WRITE_STATUS("Receive %s Btm Req with Mode:%d\n",
			     eDrvStatus == SOL_BTM_REQ ? "solicited"
						       : "unsolicited",
			     prAisSpecBssInfo->rBTMParam.ucRequestMode);
		if (!(prAisSpecBssInfo->rBTMParam.ucRequestMode &
		      BTM_REQ_MODE_CAND_INCLUDED_BIT))
			break;
		WRITE_STATUS(
			"Candidate List(Total %u), Bssid/PrefPre/Pref/Ch\n",
			prApList->u4NumElem);
		LINK_FOR_EACH_ENTRY(prNeighborAP, prApList, rLinkEntry,
				    struct NEIGHBOR_AP_T)
		{
			WRITE_STATUS("%pM/%d/%d/%d\n", prNeighborAP->aucBssid,
				     prNeighborAP->fgPrefPresence,
				     prNeighborAP->ucPreference,
				     prNeighborAP->ucChannel);
		}
		break;
	}
	case NEIGHBOR_AP_REP:
	{
		struct AIS_SPECIFIC_BSS_INFO *prAisSpecBssInfo =
			(struct AIS_SPECIFIC_BSS_INFO *)pvInfo;
		struct LINK *prApList =
			&prAisSpecBssInfo->rNeighborApList.rUsingLink;
		struct NEIGHBOR_AP_T *prNeighborAP = NULL;

		if (!prAisSpecBssInfo) {
			DBGLOG(INIT, ERROR, "prAisSpecBssInfo is NULL\n");
			break;
		}
		WRITE_STATUS(
			"Receive Neighbor Report\nList(Total %u), Bssid/PrefPre/Pref/Ch\n",
			     prApList->u4NumElem);
		LINK_FOR_EACH_ENTRY(prNeighborAP, prApList, rLinkEntry,
				    struct NEIGHBOR_AP_T)
		{
			WRITE_STATUS("%pM/%d/%d/%d\n", prNeighborAP->aucBssid,
				     prNeighborAP->fgPrefPresence,
				     prNeighborAP->ucPreference,
				     prNeighborAP->ucChannel);
		}
		break;
	}
	case SND_BTM_RSP:
	{
		struct BSS_TRANSITION_MGT_PARAM_T *prBtm =
			(struct BSS_TRANSITION_MGT_PARAM_T *)pvInfo;

		if (!prBtm) {
			DBGLOG(INIT, ERROR, "prBtm is NULL\n");
			break;
		}
		if (prBtm->ucStatusCode == BSS_TRANSITION_MGT_STATUS_ACCEPT)
			WRITE_STATUS("Send Btm Response, Roaming Target:%pM\n",
				     prBtm->aucTargetBssid);
		else
			WRITE_STATUS("Send Btm Response, Reject reason:%d\n",
				     prBtm->ucStatusCode);
		break;
	}
	case CONNECT_AP:
		WRITE_STATUS("Connect to %pM\n", (uint8_t *)pvInfo);
		break;
	case JOIN_FAIL:
	{
		struct STA_RECORD *prStaRec = (struct STA_RECORD *)pvInfo;

		if (!prStaRec) {
			DBGLOG(INIT, ERROR, "prStaRec is NULL\n");
			break;
		}
		WRITE_STATUS("Connect with %pM was rejected %d\n",
			     prStaRec->aucMacAddr, prStaRec->u2StatusCode);
		break;
	}
	case DISCONNECT_AP:
	{
		struct BSS_INFO *prBssInfo = (struct BSS_INFO *)pvInfo;

		if (!prBssInfo)
			WRITE_STATUS("Disconnected reason: unknown\n");
		else
			WRITE_STATUS("Disconnected reason: %d, bssid %pM\n",
				     prBssInfo->u2DeauthReason,
				     prBssInfo->aucBSSID);
		break;
	}
	case BEACON_TIMEOUT:
		WRITE_STATUS("Beacon timeout with %pM\n", (uint8_t *)pvInfo);
		break;
	case RCV_FW_ROAMING:
		WRITE_STATUS("%s\n", "Receive FW roaming event");
		break;
	case ROAMING_SCAN_START:
	{
		struct MSG_SCN_SCAN_REQ_V2 *prMsg =
			(struct MSG_SCN_SCAN_REQ_V2 *)pvInfo;

		if (!prMsg) {
			DBGLOG(INIT, ERROR, "prMsg is NULL\n");
			break;
		}
		WRITE_STATUS(
			"Roaming Scan Start, eScanChannel=%d(0:FULL,1:2.4G,2:5G,3:P2P_SOCIAL,4:SPECIFIED), ChannelListNum:%d, ChannelDwellTime=%d\n",
			prMsg->eScanChannel, prMsg->ucChannelListNum,
			prMsg->u2ChannelDwellTime);
		if (prMsg->eScanChannel == SCAN_CHANNEL_SPECIFIED) {
			if (prMsg->u2ChannelDwellTime > 0)
				WRITE_STATUS(
					"Roaming Scan channel num:%d, dwell time %d\n",
					prMsg->ucChannelListNum,
					prMsg->u2ChannelDwellTime);
			else
				WRITE_STATUS(
					"Roaming Scan channel num:%d, default dwell time\n",
					     prMsg->ucChannelListNum);
		} else
			WRITE_STATUS(
				"Roaming Full Scan, excluded channel num:%d\n",
				prMsg->ucChannelListNum);
		break;
	}
	case ROAMING_SCAN_DONE:
		WRITE_STATUS("Roaming Scan done\n");
		break;
	default:
		break;
	}
	mutex_unlock(&prBRProc->lock);
	/* Wake up all readers if at least one is waiting */
	glProcWakeupThreads(prBRProc, 1);
}

/* Read callback function
** *f_pos: read position of current reader, max size: 4G * 4G bytes
** i8WrStatusPos: writing position of writer, max size: 4G * 4G bytes
*/
static ssize_t procReadDrvStatus(struct file *filp, char __user *buf,
				 size_t count, loff_t *f_pos)
{
#undef NOT_ENABLE
#define NOT_ENABLE "Driver Status is not enabled"
#undef TO_ENABLE
#define TO_ENABLE "echo enable > /proc/wlan/status to enable"
#undef TO_DISABLE
#define TO_DISABLE "echo disable > /proc/wlan/status to disable\n"

	return procReadBlockedProc(&rDrvStatusProc, buf, !(filp->f_flags & O_NONBLOCK), count, f_pos,
				   NOT_ENABLE"\n"TO_ENABLE"\n"TO_DISABLE);
}

static ssize_t procDrvStatusCfg(struct file *file, const char *buffer,
				size_t count, loff_t *data)
{
	PROC_WRITE_COMMON(buffer, count);
	CONFIGURE_BLOCKED_READING_PROC_ON_OFF(&rDrvStatusProc);
	return -EINVAL;
}

static const struct file_operations drv_status_ops = {
	.owner = THIS_MODULE,
	.read = procReadDrvStatus,
	.write = procDrvStatusCfg,
};

/* fos_change begin */
#if CFG_SUPPORT_DTIM_SKIP
static ssize_t dtim_skip_count_read(struct file *filp,
				   char __user *buf,
				   size_t count, loff_t *f_pos)
{
	uint32_t u4CopySize;
	struct ADAPTER *prAdapter = NULL;
	uint8_t ucDtimSkipCount = 0;

	if (g_prGlueInfo_proc != NULL)
		prAdapter = g_prGlueInfo_proc->prAdapter;
	else
		return -EFAULT;
	/* if *f_pos > 0, it means has read successed last time,
	 *  don't try again
	 */
	if (*f_pos > 0)
		return 0;

	ucDtimSkipCount = prAdapter->ucDtimSkipCount;

	kalSprintf(g_aucProcBuf, "DTIM Skip Count:%hhu\n",
		ucDtimSkipCount);

	u4CopySize = kalStrLen(g_aucProcBuf);
	if (u4CopySize > count)
		u4CopySize = count;

	if (copy_to_user(buf, g_aucProcBuf, u4CopySize)) {
		pr_info("copy to user failed\n");
		return -EFAULT;
	}
	*f_pos += u4CopySize;

	return (int32_t)u4CopySize;
}

static ssize_t dtim_skip_count_write(struct file *file,
				     const char __user *buffer,
				     size_t count, loff_t *data)
{
	struct ADAPTER *prAdapter = NULL;
	uint8_t ucDtimSkipCount = 0;
	uint32_t u4CopySize = sizeof(g_aucProcBuf);

	if (g_prGlueInfo_proc != NULL)
		prAdapter = g_prGlueInfo_proc->prAdapter;
	else
		return -EFAULT;

	kalMemSet(g_aucProcBuf, 0, u4CopySize);

	if (u4CopySize > count)
		u4CopySize = count;
	else
		u4CopySize = u4CopySize - 1;

	if (copy_from_user(g_aucProcBuf, buffer, u4CopySize)) {
		pr_err("error of copy from user\n");
		return -EFAULT;
	}

	g_aucProcBuf[u4CopySize] = '\0';
	if (kalkStrtou8(g_aucProcBuf, 0, &ucDtimSkipCount) == 0) {
		if (ucDtimSkipCount > 6)
			return -EINVAL;
		prAdapter->ucDtimSkipCount = ucDtimSkipCount;
	} else {
		return -EINVAL;
	}

	return count;
}

static const struct file_operations dtim_ops = {
	.owner = THIS_MODULE,
	.read = dtim_skip_count_read,
	.write = dtim_skip_count_write,
};
#endif /* fos_change end */

int32_t procInitFs(void)
{
	struct proc_dir_entry *prEntry;

	g_u4NextDriverReadLen = 0;

	/* Create folder /proc/wlan/ to avoid dump by other processes,
	** like netdiag
	*/
	gprProcRoot = proc_mkdir(PROC_ROOT_NAME, NULL);
	if (!gprProcRoot) {
		pr_err("gprProcRoot == NULL\n");
		return -ENOENT;
	}

	if (init_net.proc_net == (struct proc_dir_entry *)NULL) {
		pr_err("init proc fs fail: proc_net == NULL\n");
		return -ENOENT;
	}

	/* Create folder /proc/net/wlan */
	gprProcNetRoot = proc_mkdir(PROC_ROOT_NAME, init_net.proc_net);
	if (!gprProcNetRoot) {
		pr_err("gprProcNetRoot == NULL\n");
		return -ENOENT;
	}
	proc_set_user(gprProcNetRoot, KUIDT_INIT(PROC_UID_SHELL),
		      KGIDT_INIT(PROC_GID_WIFI));

	prEntry =
	    proc_create(PROC_DBG_LEVEL_NAME, 0664, gprProcNetRoot,
		&dbglevel_ops);
	if (prEntry == NULL) {
		/* fos_change oneline */
		pr_err("Unable to create /proc entry dbgLevel\n");
		return -1;
	}
	proc_set_user(prEntry, KUIDT_INIT(PROC_UID_SHELL),
		      KGIDT_INIT(PROC_GID_WIFI));

	prEntry =
	    proc_create(PROC_AUTO_PERF_CFG, 0664, gprProcNetRoot,
		&auto_perf_ops);
	if (prEntry == NULL) {
		DBGLOG(INIT, ERROR, "Unable to create /proc entry %s/n",
		       PROC_AUTO_PERF_CFG);
		return -1;
	}
	proc_set_user(prEntry, KUIDT_INIT(PROC_UID_SHELL),
		      KGIDT_INIT(PROC_GID_WIFI));

	procInitBlockedReadProc(&rDrvStatusProc, 2048, TRUE);

	prEntry =
	    proc_create(PROC_DRV_STATUS, 0664, gprProcRoot, &drv_status_ops);
	if (!prEntry) {
		DBGLOG(INIT, ERROR, "Unable to create /proc entry %s/n",
		       PROC_DRV_STATUS);
		return -1;
	}
	proc_set_user(prEntry, KUIDT_INIT(PROC_UID_SHELL),
		      KGIDT_INIT(PROC_GID_WIFI));
	return 0;
}				/* end of procInitProcfs() */

int32_t procUninitProcFs(void)
{
#if KERNEL_VERSION(3, 9, 0) <= LINUX_VERSION_CODE
	remove_proc_subtree(PROC_AUTO_PERF_CFG, gprProcNetRoot);
	remove_proc_subtree(PROC_DBG_LEVEL_NAME, gprProcNetRoot);
	remove_proc_subtree(PROC_DRV_STATUS, gprProcRoot);

	/*
	 * move PROC_ROOT_NAME to last since it's root directory of the others
	 * incorrect sequence would cause use-after-free error
	 */
	remove_proc_subtree(PROC_ROOT_NAME, init_net.proc_net);
	remove_proc_subtree(PROC_ROOT_NAME, NULL);
#else
	remove_proc_entry(PROC_AUTO_PERF_CFG, gprProcNetRoot);
	remove_proc_entry(PROC_DBG_LEVEL_NAME, gprProcNetRoot);
	remove_proc_entry(PROC_DRV_STATUS, gprProcRoot);
	/*
	 * move PROC_ROOT_NAME to last since it's root directory of the others
	 * incorrect sequence would cause use-after-free error
	 */
	remove_proc_entry(PROC_ROOT_NAME, init_net.proc_net);
	remove_proc_entry(PROC_ROOT_NAME, NULL);
#endif

	procUninitBlockedReadProc(&rDrvStatusProc);

	return 0;
}

/*----------------------------------------------------------------------------*/
/*!
 * \brief This function clean up a PROC fs created by procInitProcfs().
 *
 * \param[in] prDev      Pointer to the struct net_device.
 * \param[in] pucDevName Pointer to the name of net_device.
 *
 * \return N/A
 */
/*----------------------------------------------------------------------------*/
int32_t procRemoveProcfs(void)
{
	remove_proc_entry(PROC_MCR_ACCESS, gprProcNetRoot);
	remove_proc_entry(PROC_DRIVER_CMD, gprProcNetRoot);
	remove_proc_entry(PROC_CFG, gprProcNetRoot);
	remove_proc_entry(PROC_EFUSE_DUMP, gprProcNetRoot);

	remove_proc_entry(PROC_PKT_DELAY_DBG, gprProcNetRoot);
#if CFG_SUPPORT_SET_CAM_BY_PROC
	remove_proc_entry(PROC_SET_CAM, gprProcNetRoot);
#endif
#if CFG_SUPPORT_DEBUG_FS
	remove_proc_entry(PROC_ROAM_PARAM, gprProcNetRoot);
#endif
	remove_proc_entry(PROC_COUNTRY, gprProcNetRoot);
/* fos_change begin */
#if CFG_SUPPORT_DTIM_SKIP
	remove_proc_entry(PROC_DTIM, gprProcNetRoot);
#endif
#if CFG_SUPPORT_WIFI_POWER_DEBUG
	procUninitBlockedReadProc(&rWakeupLogProc);
	procUninitBlockedReadProc(&rAppTxRxProc);
	remove_proc_entry(PROC_WAKEUP_LOG, gprProcNetRoot);
	remove_proc_entry(PROC_TX_RX_STAT, gprProcNetRoot);
#endif
	g_prGlueInfo_proc = NULL; /* fos_change end */

	return 0;
} /* end of procRemoveProcfs() */

int32_t procCreateFsEntry(struct GLUE_INFO *prGlueInfo)
{
	struct proc_dir_entry *prEntry;

	DBGLOG(INIT, TRACE, "[%s]\n", __func__);
	g_prGlueInfo_proc = prGlueInfo;

	prEntry = proc_create(PROC_MCR_ACCESS, 0664, gprProcNetRoot, &mcr_ops);
	if (prEntry == NULL) {
		/* fos_change oneline */
		DBGLOG(INIT, ERROR, "Unable to create /proc entry mcr\n");
		return -1;
	}

	prEntry =
	    proc_create(PROC_PKT_DELAY_DBG, 0664, gprProcNetRoot,
			&proc_pkt_delay_dbg_ops);
	if (prEntry == NULL) {
		/* fos_change oneline */
		DBGLOG(INIT, ERROR,
		       "Unable to create /proc entry pktDelay\n");
		return -1;
	}
	proc_set_user(prEntry, KUIDT_INIT(PROC_UID_SHELL),
		      KGIDT_INIT(PROC_GID_WIFI));

#if CFG_SUPPORT_SET_CAM_BY_PROC
	prEntry =
	    proc_create(PROC_SET_CAM, 0664, gprProcNetRoot, &proc_set_cam_ops);
	if (prEntry == NULL) {
		/* fos_change oneline */
		DBGLOG(INIT, ERROR, "Unable to create /proc entry SetCAM\n");
		return -1;
	}
	proc_set_user(prEntry, KUIDT_INIT(PROC_UID_SHELL),
		      KGIDT_INIT(PROC_GID_WIFI));
#endif
#if CFG_SUPPORT_DEBUG_FS
	prEntry = proc_create(PROC_ROAM_PARAM, 0664, gprProcNetRoot, &roam_ops);
	if (prEntry == NULL) {
		/* fos_change oneline */
		DBGLOG(INIT, ERROR,
		       "Unable to create /proc entry roam_param\n");
		return -1;
	}
#endif
	prEntry = proc_create(PROC_COUNTRY, 0664, gprProcNetRoot, &country_ops);
	if (prEntry == NULL) {
		/* fos_change oneline */
		DBGLOG(INIT, ERROR, "Unable to create /proc entry country\n");
		return -1;
	}
#if	CFG_SUPPORT_EASY_DEBUG

	prEntry =
		proc_create(PROC_DRIVER_CMD, 0664, gprProcNetRoot,
			&drivercmd_ops);
	if (prEntry == NULL) {
		/* fos_change oneline */
		pr_err("Unable to create /proc entry for driver command\n");
		return -1;
	}

	prEntry = proc_create(PROC_CFG, 0664, gprProcNetRoot, &cfg_ops);
	if (prEntry == NULL) {
		/* fos_change oneline */
		pr_err("Unable to create /proc entry for driver cfg\n");
		return -1;
	}

	prEntry =
		proc_create(PROC_EFUSE_DUMP, 0664, gprProcNetRoot,
			&efusedump_ops);
	if (prEntry == NULL) {
		/* fos_change oneline */
		pr_err("Unable to create /proc entry efuse\n");
		return -1;
	}
#endif
/* fos_change begin */
#if CFG_SUPPORT_DTIM_SKIP
	prEntry = proc_create(PROC_DTIM, 0664, gprProcNetRoot, &dtim_ops);
	if (prEntry == NULL) {
		DBGLOG(INIT, ERROR, "Unable to create /proc entry\n\r");
		return -1;
	}
#endif

#if CFG_SUPPORT_WIFI_POWER_DEBUG
	procInitBlockedReadProc(&rWakeupLogProc, 1024, FALSE);
	prEntry = proc_create(PROC_WAKEUP_LOG, 0664, gprProcNetRoot, &wakeup_log_ops);
	if (prEntry == NULL) {
		DBGLOG(INIT, ERROR, "Unable to create /proc entry\n\r");
		return -1;
	}
	proc_set_user(prEntry, KUIDT_INIT(PROC_UID_SHELL), KGIDT_INIT(PROC_GID_WIFI));

	/* buf length is 0 here, because we'll allocate different buffer runtime */
	procInitBlockedReadProc(&rAppTxRxProc, 0, FALSE);
	prEntry = proc_create(PROC_TX_RX_STAT, 0664, gprProcNetRoot, &app_trx_stat_ops);
	if (prEntry == NULL) {
		DBGLOG(INIT, ERROR, "Unable to create /proc entry\n\r");
		return -1;
	}
	proc_set_user(prEntry, KUIDT_INIT(PROC_UID_SHELL), KGIDT_INIT(PROC_GID_WIFI));
#endif /* fos_change end */
	return 0;
}

#if 0
/*----------------------------------------------------------------------------*/
/*!
 * \brief The PROC function for reading Driver Status to User Space.
 *
 * \param[in] page       Buffer provided by kernel.
 * \param[in out] start  Start Address to read(3 methods).
 * \param[in] off        Offset.
 * \param[in] count      Allowable number to read.
 * \param[out] eof       End of File indication.
 * \param[in] data       Pointer to the private data structure.
 *
 * \return number of characters print to the buffer from User Space.
 */
/*----------------------------------------------------------------------------*/
static int procDrvStatusRead(char *page, char **start, off_t off, int count,
	int *eof, void *data)
{
	struct GLUE_INFO *prGlueInfo = ((struct net_device *)data)->priv;
	char *p = page;
	uint32_t u4Count;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(data);

	/* Kevin: Apply PROC read method 1. */
	if (off != 0)
		return 0;	/* To indicate end of file. */

	SNPRINTF(p, page, ("GLUE LAYER STATUS:"));
	SNPRINTF(p, page, ("\n=================="));

	SNPRINTF(p, page,
		("\n* Number of Pending Frames: %ld\n",
		prGlueInfo->u4TxPendingFrameNum));

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	wlanoidQueryDrvStatusForLinuxProc(prGlueInfo->prAdapter, p, &u4Count);

	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	u4Count += (uint32_t) (p - page);

	*eof = 1;

	return (int)u4Count;

} /* end of procDrvStatusRead() */

/*----------------------------------------------------------------------------*/
/*!
 * \brief The PROC function for reading Driver RX Statistic Counters
 *        to User Space.
 *
 * \param[in] page       Buffer provided by kernel.
 * \param[in out] start  Start Address to read(3 methods).
 * \param[in] off        Offset.
 * \param[in] count      Allowable number to read.
 * \param[out] eof       End of File indication.
 * \param[in] data       Pointer to the private data structure.
 *
 * \return number of characters print to the buffer from User Space.
 */
/*----------------------------------------------------------------------------*/
static int procRxStatisticsRead(char *page, char **start, off_t off, int count,
	int *eof, void *data)
{
	struct GLUE_INFO *prGlueInfo = ((struct net_device *)data)->priv;
	char *p = page;
	uint32_t u4Count;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(data);

	/* Kevin: Apply PROC read method 1. */
	if (off != 0)
		return 0;	/* To indicate end of file. */

	SNPRINTF(p, page, ("RX STATISTICS (Write 1 to clear):"));
	SNPRINTF(p, page, ("\n=================================\n"));

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	wlanoidQueryRxStatisticsForLinuxProc(prGlueInfo->prAdapter, p,
		&u4Count);

	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	u4Count += (uint32_t) (p - page);

	*eof = 1;

	return (int)u4Count;

} /* end of procRxStatisticsRead() */

/*----------------------------------------------------------------------------*/
/*!
 * \brief The PROC function for reset Driver RX Statistic Counters.
 *
 * \param[in] file   pointer to file.
 * \param[in] buffer Buffer from user space.
 * \param[in] count  Number of characters to write
 * \param[in] data   Pointer to the private data structure.
 *
 * \return number of characters write from User Space.
 */
/*----------------------------------------------------------------------------*/
static int procRxStatisticsWrite(struct file *file, const char *buffer,
	unsigned long count, void *data)
{
	struct GLUE_INFO *prGlueInfo = ((struct net_device *)data)->priv;
	/* + 1 for "\0" */
	char acBuf[PROC_RX_STATISTICS_MAX_USER_INPUT_LEN + 1];
	uint32_t u4CopySize;
	uint32_t u4ClearCounter;
	int32_t rv;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(data);

	u4CopySize =
		(count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
	copy_from_user(acBuf, buffer, u4CopySize);
	acBuf[u4CopySize] = '\0';

	rv = kstrtoint(acBuf, 0, &u4ClearCounter);
	if (rv == 1) {
		if (u4ClearCounter == 1) {
			GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

			wlanoidSetRxStatisticsForLinuxProc(prGlueInfo->
				prAdapter);

			GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);
		}
	}

	return count;

} /* end of procRxStatisticsWrite() */

/*----------------------------------------------------------------------------*/
/*!
 * \brief The PROC function for reading Driver TX Statistic Counters
 *        to User Space.
 *
 * \param[in] page       Buffer provided by kernel.
 * \param[in out] start  Start Address to read(3 methods).
 * \param[in] off        Offset.
 * \param[in] count      Allowable number to read.
 * \param[out] eof       End of File indication.
 * \param[in] data       Pointer to the private data structure.
 *
 * \return number of characters print to the buffer from User Space.
 */
/*----------------------------------------------------------------------------*/
static int procTxStatisticsRead(char *page, char **start, off_t off, int count,
	int *eof, void *data)
{
	struct GLUE_INFO *prGlueInfo = ((struct net_device *)data)->priv;
	char *p = page;
	uint32_t u4Count;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(data);

	/* Kevin: Apply PROC read method 1. */
	if (off != 0)
		return 0;	/* To indicate end of file. */

	SNPRINTF(p, page, ("TX STATISTICS (Write 1 to clear):"));
	SNPRINTF(p, page, ("\n=================================\n"));

	GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	wlanoidQueryTxStatisticsForLinuxProc(prGlueInfo->prAdapter, p,
		&u4Count);

	GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

	u4Count += (uint32_t) (p - page);

	*eof = 1;

	return (int)u4Count;

} /* end of procTxStatisticsRead() */

/*----------------------------------------------------------------------------*/
/*!
 * \brief The PROC function for reset Driver TX Statistic Counters.
 *
 * \param[in] file   pointer to file.
 * \param[in] buffer Buffer from user space.
 * \param[in] count  Number of characters to write
 * \param[in] data   Pointer to the private data structure.
 *
 * \return number of characters write from User Space.
 */
/*----------------------------------------------------------------------------*/
static int procTxStatisticsWrite(struct file *file, const char *buffer,
	unsigned long count, void *data)
{
	struct GLUE_INFO *prGlueInfo = ((struct net_device *)data)->priv;
	/* + 1 for "\0" */
	char acBuf[PROC_RX_STATISTICS_MAX_USER_INPUT_LEN + 1];
	uint32_t u4CopySize;
	uint32_t u4ClearCounter;
	int32_t rv;

	GLUE_SPIN_LOCK_DECLARATION();

	ASSERT(data);

	u4CopySize =
		(count < (sizeof(acBuf) - 1)) ? count : (sizeof(acBuf) - 1);
	copy_from_user(acBuf, buffer, u4CopySize);
	acBuf[u4CopySize] = '\0';

	rv = kstrtoint(acBuf, 0, &u4ClearCounter);
	if (rv == 1) {
		if (u4ClearCounter == 1) {
			GLUE_ACQUIRE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);

			wlanoidSetTxStatisticsForLinuxProc(prGlueInfo->
				prAdapter);

			GLUE_RELEASE_SPIN_LOCK(prGlueInfo, SPIN_LOCK_FSM);
		}
	}

	return count;

} /* end of procTxStatisticsWrite() */
#endif

#ifdef FW_CFG_SUPPORT
#define MAX_CFG_OUTPUT_BUF_LENGTH 1024
static uint8_t aucCfgBuf[CMD_FORMAT_V1_LENGTH];
static uint8_t aucCfgQueryKey[MAX_CMD_NAME_MAX_LENGTH];
static uint8_t aucCfgOutputBuf[MAX_CFG_OUTPUT_BUF_LENGTH];

static ssize_t cfgRead(struct file *filp, char __user *buf, size_t count,
	loff_t *f_pos)
{
	uint32_t rStatus = WLAN_STATUS_FAILURE;
	uint8_t *temp = &aucCfgOutputBuf[0];
	uint32_t u4CopySize = 0;

	struct CMD_HEADER cmdV1Header;
	struct CMD_FORMAT_V1 *pr_cmd_v1 =
		(struct CMD_FORMAT_V1 *)cmdV1Header.buffer;

	/* if *f_pos >  0, we should return 0 to make cat command exit */
	if (*f_pos > 0 || gprGlueInfo == NULL)
		return 0;

#if CFG_CHIP_RESET_SUPPORT
	if (!wlanIsDriverReady(gprGlueInfo)) {
		DBGLOG(INIT, ERROR,
		       "driver is not ready: u4ReadyFlag=%u, kalIsResetting()=%d\n",
		       gprGlueInfo->u4ReadyFlag, kalIsResetting());
		return 0; /* return 0 to make command exit */
	}
#endif

	if (!kalStrLen(aucCfgQueryKey))
		return 0;

	kalMemSet(aucCfgOutputBuf, 0, MAX_CFG_OUTPUT_BUF_LENGTH);

	SNPRINTF(temp, aucCfgOutputBuf,
		("\nprocCfgRead() %s:\n", aucCfgQueryKey));

	/* send to FW */
	cmdV1Header.cmdVersion = CMD_VER_1;
	cmdV1Header.cmdType = CMD_TYPE_QUERY;
	cmdV1Header.itemNum = 1;
	cmdV1Header.cmdBufferLen = sizeof(struct CMD_FORMAT_V1);
	kalMemSet(cmdV1Header.buffer, 0, MAX_CMD_BUFFER_LENGTH);

	pr_cmd_v1->itemStringLength = kalStrLen(aucCfgQueryKey);

	kalMemCopy(pr_cmd_v1->itemString, aucCfgQueryKey,
		kalStrLen(aucCfgQueryKey));

	rStatus = kalIoctl(gprGlueInfo,
		wlanoidQueryCfgRead,
		(void *)&cmdV1Header,
		sizeof(cmdV1Header), TRUE, TRUE, TRUE, &u4CopySize);
	if (rStatus == WLAN_STATUS_FAILURE)
		DBGLOG(INIT, ERROR,
			"kalIoctl wlanoidQueryCfgRead fail 0x%x\n",
			rStatus);

	SNPRINTF(temp, aucCfgOutputBuf,
		("%s\n", cmdV1Header.buffer));

	u4CopySize = kalStrLen(aucCfgOutputBuf);
	if (u4CopySize > count)
		u4CopySize = count;

	if (copy_to_user(buf, aucCfgOutputBuf, u4CopySize))
		DBGLOG(INIT, ERROR, "copy to user failed\n");

	*f_pos += u4CopySize;
	return (ssize_t) u4CopySize;
}

static ssize_t cfgWrite(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
	/* echo xxx xxx > /proc/net/wlan/cfg */
	uint8_t i = 0;
	uint32_t u4CopySize = sizeof(aucCfgBuf);
	uint8_t token_num = 1;

#if CFG_CHIP_RESET_SUPPORT
	if (!gprGlueInfo) {
		DBGLOG(INIT, ERROR, "g_prGlueInfo_proc is null\n");
		return 0;
	}
	if (!wlanIsDriverReady(gprGlueInfo)) {
		DBGLOG(INIT, ERROR,
		       "driver is not ready: u4ReadyFlag=%u, kalIsResetting()=%d\n",
		       gprGlueInfo->u4ReadyFlag, kalIsResetting());
		return 0; /* return 0 to make command exit */
	}
#endif

	kalMemSet(aucCfgBuf, 0, u4CopySize);
	u4CopySize = (count < u4CopySize) ? count : (u4CopySize - 1);

	if (copy_from_user(aucCfgBuf, buf, u4CopySize)) {
		DBGLOG(INIT, ERROR, "copy from user failed\n");
		return -EFAULT;
	}
	aucCfgBuf[u4CopySize] = '\0';
	for (; i < u4CopySize; i++) {
		if (aucCfgBuf[i] == ' ') {
			token_num++;
			break;
		}
	}

	if (token_num == 1) {
		kalMemSet(aucCfgQueryKey, 0, sizeof(aucCfgQueryKey));
		/* remove the 0x0a */
		memcpy(aucCfgQueryKey, aucCfgBuf, u4CopySize);
		if (aucCfgQueryKey[u4CopySize - 1] == 0x0a)
			aucCfgQueryKey[u4CopySize - 1] = '\0';
	} else {
		if (u4CopySize)
			wlanFwCfgParse(gprGlueInfo->prAdapter, aucCfgBuf);
	}

	return count;
}

static const struct file_operations fwcfg_ops = {
	.owner = THIS_MODULE,
	.read = cfgRead,
	.write = cfgWrite,
};

int32_t cfgRemoveProcEntry(void)
{
	remove_proc_entry(PROC_CFG_NAME, gprProcNetRoot);
	return 0;
}

int32_t cfgCreateProcEntry(struct GLUE_INFO *prGlueInfo)
{
	struct proc_dir_entry *prEntry;

	prGlueInfo->pProcRoot = gprProcNetRoot;
	gprGlueInfo = prGlueInfo;

	prEntry = proc_create(PROC_CFG_NAME, 0664, gprProcNetRoot, &fwcfg_ops);
	if (prEntry == NULL) {
		/* fos_change oneline */
		DBGLOG(INIT, ERROR, "Unable to create /proc entry cfg\n");
		return -1;
	}
	proc_set_user(prEntry, KUIDT_INIT(PROC_UID_SHELL),
		KGIDT_INIT(PROC_GID_WIFI));

	return 0;
}
#endif

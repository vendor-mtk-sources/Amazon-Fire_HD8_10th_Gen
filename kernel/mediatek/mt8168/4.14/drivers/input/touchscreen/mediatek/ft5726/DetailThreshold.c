/*
 * Copyright (C) 2012-2015, Focaltech Systems (R),All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/string.h>
#include "ini.h"
#include "DetailThreshold.h"
#include "test_lib.h"
#include "Global.h"
#include "tpd_custom_fts.h"

struct stCfg_MCap_DetailThreshold g_MCap;
struct stCfg_SCap_DetailThreshold g_SCap;

void set_max_channel_num(void)
{
	switch (g_ScreenSetParam.iSelectedIC >> 4) {
	case IC_FT5822 >> 4:
		g_ScreenSetParam.iUsedMaxTxNum = TX_NUM_MAX;
		g_ScreenSetParam.iUsedMaxRxNum = RX_NUM_MAX;
		break;
	default:
		g_ScreenSetParam.iUsedMaxTxNum = 30;
		g_ScreenSetParam.iUsedMaxRxNum = 30;
		break;
	}
}

void OnInit_SCap_DetailThreshold(char *strIniFile)
{
	OnGetTestItemParam("RawDataTest_Max", strIniFile, 12500);

	memcpy(g_SCap.RawDataTest_Max,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	OnGetTestItemParam("RawDataTest_Min", strIniFile, 16500);

	memcpy(g_SCap.RawDataTest_Min,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	OnGetTestItemParam("CiTest_Max", strIniFile, 5);

	memcpy(g_SCap.CiTest_Max,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	OnGetTestItemParam("CiTest_Min", strIniFile, 250);

	memcpy(g_SCap.CiTest_Min,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	OnGetTestItemParam("DeltaCiTest_Base", strIniFile, 0);

	memcpy(g_SCap.DeltaCiTest_Base,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	OnGetTestItemParam("DeltaCiTest_AnotherBase1", strIniFile, 0);

	memcpy(g_SCap.DeltaCiTest_AnotherBase1,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	OnGetTestItemParam("DeltaCiTest_AnotherBase2", strIniFile, 0);

	memcpy(g_SCap.DeltaCiTest_AnotherBase2,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	OnGetTestItemParam("NoiseTest_Max", strIniFile, 20);

	memcpy(g_SCap.NoiseTest_Max,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));
	/*OnGetTestItemParam("CiDeviation_Base", strIniFile); */
	OnGetTestItemParam("CiDeviation_Base", strIniFile, 0);

	memcpy(g_SCap.CiDeviationTest_Base,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	OnGetTestItemParam("DeltaCxTest_Sort", strIniFile, 1);

	memcpy(g_SCap.DeltaCxTest_Sort,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	OnGetTestItemParam("DeltaCxTest_Area", strIniFile, 1);

	memcpy(g_SCap.DeltaCxTest_Area,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));
	/*6x36 */
	/*OnGetTestItemParam("CbTest_Max", strIniFile); */
	OnGetTestItemParam("CbTest_Max", strIniFile, 0);

	memcpy(g_SCap.CbTest_Max,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	/*OnGetTestItemParam("CbTest_Min", strIniFile); */
	OnGetTestItemParam("CbTest_Min", strIniFile, 0);

	memcpy(g_SCap.CbTest_Min,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	/*OnGetTestItemParam("DeltaCbTest_Base", strIniFile); */
	OnGetTestItemParam("DeltaCbTest_Base", strIniFile, 0);

	memcpy(g_SCap.DeltaCbTest_Base,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	/*OnGetTestItemParam("DifferTest_Base", strIniFile); */
	OnGetTestItemParam("DifferTest_Base", strIniFile, 0);

	memcpy(g_SCap.DifferTest_Base,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	/*OnGetTestItemParam("CBDeviation_Base", strIniFile); */
	OnGetTestItemParam("CBDeviation_Base", strIniFile, 0);

	memcpy(g_SCap.CBDeviationTest_Base,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));

	/*OnGetTestItemParam("K1DifferTest_Base", strIniFile); */
	OnGetTestItemParam("K1DifferTest_Base", strIniFile, 0);

	memcpy(g_SCap.K1DifferTest_Base,
	       g_SCap.TempData,
	       MAX_CHANNEL_NUM * sizeof(int));
}

void OnGetTestItemParam(char *strItemName, char *strIniFile, int iDefautValue)
{
	char strValue[800];
	char str_tmp[128];
	int iValue = 0;
	int dividerPos = 0;
	int index = 0;
	int i = 0, j = 0, k = 0;

	memset(g_SCap.TempData, 0,
	       ARRAY_SIZE(g_SCap.TempData));

	snprintf(str_tmp, ARRAY_SIZE(str_tmp), "%d", iDefautValue);
	GetPrivateProfileString("Basic_Threshold", strItemName, str_tmp,
				strValue, strIniFile);
	iValue = atoi(strValue);

	for (i = 0; i < MAX_CHANNEL_NUM; i++)
		g_SCap.TempData[i] = iValue;

	dividerPos =
	    GetPrivateProfileString("SpecialSet", strItemName, "", strValue,
				    strIniFile);

	if (dividerPos > 0) {
		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_SCap.TempData[k] =
				    (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}

	}
}

void OnInit_MCap_DetailThreshold(char *strIniFile)
{
	set_max_channel_num();	/*set used TxRx */

	OnInit_InvalidNode(strIniFile);
	OnInit_DThreshold_RawDataTest(strIniFile);
	OnInit_DThreshold_SCapRawDataTest(strIniFile);
	OnInit_DThreshold_SCapCbTest(strIniFile);
	OnInit_DThreshold_PanelDifferTest(strIniFile);
	OnInit_DThreshold_ForceTouch_SCapRawDataTest(strIniFile);
	OnInit_DThreshold_ForceTouch_SCapCbTest(strIniFile);
/* OnInit_DThreshold_RxCrosstalkTest(strIniFile); */
	OnInit_DThreshold_RxLinearityTest(strIniFile);
	OnInit_DThreshold_TxLinearityTest(strIniFile);
}

void OnInit_InvalidNode(char *strIniFile)
{
	char str[MAX_KEY_VALUE_LEN] = { 0 }, strTemp[MAX_KEY_VALUE_LEN] = { 0 };
	int i = 0, j = 0;

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			snprintf(strTemp, ARRAY_SIZE(strTemp),
				"InvalidNode[%d][%d]", (i + 1), (j + 1));

			GetPrivateProfileString("INVALID_NODE", strTemp, "1",
						str, strIniFile);
			if (atoi(str) == 0) {
				g_MCap.InvalidNode[i][j] =
				    0;
			} else if (atoi(str) == 2) {
				g_MCap.InvalidNode[i][j] =
				    2;
			} else
				g_MCap.InvalidNode[i][j] =
				    1;
		}
	}

	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			snprintf(strTemp, ARRAY_SIZE(strTemp),
				"InvalidNodeS[%d][%d]", (i + 1),
				(j + 1));
			GetPrivateProfileString("INVALID_NODES", strTemp, "1",
						str, strIniFile);

			if (atoi(str) == 0) {
				g_MCap.InvalidNode_SC[i]
				    [j] = 0;
			} else if (atoi(str) == 2) {
				g_MCap.InvalidNode_SC[i]
				    [j] = 2;
			} else
				g_MCap.InvalidNode_SC[i]
				    [j] = 1;
		}
	}
}

void OnInit_DThreshold_PanelDifferTest(char *strIniFile)
{
	char str[128], strTemp[MAX_KEY_VALUE_LEN], strValue[MAX_KEY_VALUE_LEN];
	int MaxValue, MinValue;
	int dividerPos = 0;
	char str_tmp[128];
	int index = 0;
	int k = 0, i = 0, j = 0;

	/* RawData Test */
	GetPrivateProfileString("Basic_Threshold", "PanelDifferTest_Max",
				"1000", str, strIniFile);

	MaxValue = atoi(str);

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++)
			g_MCap.PanelDifferTest_Max[i][j] =
			    MaxValue;
	}

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"Panel_Differ_Max_Tx%d", (i + 1));
		/* pre_debug("%s\n", str); */
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "111", strTemp,
					    strIniFile);

		/*
		 * pre_debug("GetPrivateProfileString =
		 *  %d\n", dividerPos);
		 */
		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.PanelDifferTest_Max[i][k] =
				(short)(atoi(str_tmp));
				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;
				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	GetPrivateProfileString("Basic_Threshold", "PanelDifferTest_Min", "150",
				str, strIniFile);

	MinValue = atoi(str);

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++)
			g_MCap.PanelDifferTest_Min[i][j] =
			    MinValue;
	}

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"Panel_Differ_Min_Tx%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.PanelDifferTest_Min[i][k] =
				(short)(atoi(str_tmp));
				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}
}

void OnInit_DThreshold_RawDataTest(char *strIniFile)
{
	char str[128], strTemp[MAX_KEY_VALUE_LEN], strValue[MAX_KEY_VALUE_LEN];
	int MaxValue, MinValue;
	int dividerPos = 0;
	char str_tmp[128];
	int index = 0;
	int k = 0, i = 0, j = 0;

	/*RawData Test */
	GetPrivateProfileString("Basic_Threshold", "RawDataTest_Max",
				"10000", str, strIniFile);

	MaxValue = atoi(str);
	/*pre_debug("MaxValue = %d\n", MaxValue); */
	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.RawDataTest_Max[i][j] =
			    MaxValue;
		}
	}

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		snprintf(str, ARRAY_SIZE(str), "RawData_Max_Tx%d", (i + 1));
		/*pre_debug("%s\n", str); */
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "111", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.RawDataTest_Max[i]
				    [k] = (short)(atoi(str_tmp));
				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;
				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	GetPrivateProfileString("Basic_Threshold", "RawDataTest_Min", "7000",
				str, strIniFile);

	MinValue = atoi(str);

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.RawDataTest_Min[i][j] =
			    MinValue;
		}
	}

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		snprintf(str, ARRAY_SIZE(str), "RawData_Min_Tx%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.RawDataTest_Min[i]
				    [k] = (short)(atoi(str_tmp));
				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	/*RawData Test Low */
	GetPrivateProfileString("Basic_Threshold", "RawDataTest_Low_Max",
				"15000", str, strIniFile);
	MaxValue = atoi(str);

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.RawDataTest_Low_Max[i][j] =
			    MaxValue;
		}
	}

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		snprintf(str, ARRAY_SIZE(str), "RawData_Max_Low_Tx%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);
		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.RawDataTest_Low_Max
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	GetPrivateProfileString("Basic_Threshold", "RawDataTest_Low_Min",
				"3000", str, strIniFile);
	MinValue = atoi(str);

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.RawDataTest_Low_Min[i][j] =
			    MinValue;
		}
	}

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		snprintf(str, ARRAY_SIZE(str), "RawData_Min_Low_Tx%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);
		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.RawDataTest_Low_Min
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	/*RawData Test High */
	GetPrivateProfileString("Basic_Threshold", "RawDataTest_High_Max",
				"15000", str, strIniFile);
	MaxValue = atoi(str);
	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.RawDataTest_High_Max[i][j]
			    = MaxValue;
		}
	}

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"RawData_Max_High_Tx%d", (i + 1));

		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);
		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.RawDataTest_High_Max
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	GetPrivateProfileString("Basic_Threshold", "RawDataTest_High_Min",
				"3000", str, strIniFile);

	MinValue = atoi(str);
	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.RawDataTest_High_Min[i][j]
			    = MinValue;
		}
	}

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"RawData_Min_High_Tx%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.RawDataTest_High_Min
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	/* TxShortAdvance Test */

}

void OnInit_DThreshold_SCapRawDataTest(char *strIniFile)
{
	char str[128], strTemp[MAX_KEY_VALUE_LEN], strValue[MAX_KEY_VALUE_LEN];
	int MaxValue, MinValue;
	int dividerPos = 0;
	char str_tmp[128];
	int index = 0;
	int k = 0, i = 0, j = 0;

	 /*OFF*/
	    GetPrivateProfileString("Basic_Threshold",
				    "SCapRawDataTest_OFF_Min", "150", str,
				    strIniFile);
	MinValue = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "SCapRawDataTest_OFF_Max",
				"1000", str, strIniFile);
	MaxValue = atoi(str);

	/*Max */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.SCapRawDataTest_OFF_Max[i]
			    [j] = MaxValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ScapRawData_OFF_Max_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.SCapRawDataTest_OFF_Max
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}
	/*Min */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.SCapRawDataTest_OFF_Min[i]
			    [j] = MinValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ScapRawData_OFF_Min_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);
		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.SCapRawDataTest_OFF_Min
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}


	 /*ON*/
	    GetPrivateProfileString("Basic_Threshold", "SCapRawDataTest_ON_Min",
				    "150", str, strIniFile);
	MinValue = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "SCapRawDataTest_ON_Max",
				"1000", str, strIniFile);
	MaxValue = atoi(str);


	/*Max */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.SCapRawDataTest_ON_Max[i]
			    [j] = MaxValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ScapRawData_ON_Max_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.SCapRawDataTest_ON_Max
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}
	/*Min */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.SCapRawDataTest_ON_Min[i]
			    [j] = MinValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ScapRawData_ON_Min_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.SCapRawDataTest_ON_Min
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}


}

void OnInit_DThreshold_SCapCbTest(char *strIniFile)
{
	char str[128], strTemp[MAX_KEY_VALUE_LEN], strValue[MAX_KEY_VALUE_LEN];
	int MaxValue, MinValue;
	int dividerPos = 0;
	char str_tmp[128];
	int index = 0;
	int k = 0, i = 0, j = 0;

	GetPrivateProfileString("Basic_Threshold", "SCapCbTest_ON_Min", "0",
				str, strIniFile);
	MinValue = atoi(str);

	GetPrivateProfileString("Basic_Threshold", "SCapCbTest_ON_Max", "240",
				str, strIniFile);
	MaxValue = atoi(str);

	/*Max */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.SCapCbTest_ON_Max[i][j] =
			    MaxValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str), "ScapCB_ON_Max_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);
		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.SCapCbTest_ON_Max
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	/*Min */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.SCapCbTest_ON_Min[i][j] =
			    MinValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str), "ScapCB_ON_Min_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.SCapCbTest_ON_Min
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	GetPrivateProfileString("Basic_Threshold",
	"SCapCbTest_OFF_Min", "0",
				str, strIniFile);

	MinValue = atoi(str);

	GetPrivateProfileString("Basic_Threshold",
	"SCapCbTest_OFF_Max", "240",
				str, strIniFile);

	MaxValue = atoi(str);

	/*Max */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.SCapCbTest_OFF_Max[i][j] =
			    MaxValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str), "ScapCB_OFF_Max_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.SCapCbTest_OFF_Max
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	/*Min */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.SCapCbTest_OFF_Min[i][j] =
			    MinValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str), "ScapCB_OFF_Min_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.SCapCbTest_OFF_Min
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

}

void OnInit_DThreshold_RxLinearityTest(char *strIniFile)
{
	char str[128], strTemp[MAX_KEY_VALUE_LEN], strValue[MAX_KEY_VALUE_LEN];
	int MaxValue = 0;
	int dividerPos = 0;
	char str_tmp[128];
	int index = 0;
	int k = 0, i = 0, j = 0;

	/*Rx_Linearity Test */
	GetPrivateProfileString("Basic_Threshold", "RxLinearityTest_Max",
				"50", str, strIniFile);

	MaxValue = atoi(str);

	/*pre_debug("MaxValue=%d\n",MaxValue); */

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.RxLinearityTest_Max[i][j] =
			    MaxValue;
		}
	}

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"Rx_Linearity_Max_Tx%d", (i + 1));

		/*pre_debug("%s\n", str); */
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "111", strTemp,
					    strIniFile);

		/*pre_debug("GetPrivateProfileString=%d\n", dividerPos); */
		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.RxLinearityTest_Max
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}
}

void OnInit_DThreshold_TxLinearityTest(char *strIniFile)
{
	char str[128], strTemp[MAX_KEY_VALUE_LEN], strValue[MAX_KEY_VALUE_LEN];
	int MaxValue = 0;
	int dividerPos = 0;
	char str_tmp[128];
	int index = 0;
	int k = 0, i = 0, j = 0;

	/*Tx_Linearity Test */
	GetPrivateProfileString("Basic_Threshold", "TxLinearityTest_Max",
				"50", str, strIniFile);

	MaxValue = atoi(str);

	/*pre_debug("MaxValue = %d\n", MaxValue); */

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.TxLinearityTest_Max[i][j] =
			    MaxValue;
		}
	}

	for (i = 0; i < g_ScreenSetParam.iUsedMaxTxNum; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"Tx_Linearity_Max_Tx%d", (i + 1));
		/*pre_debug("%s\n", str); */
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "111", strTemp,
					    strIniFile);

		/*pre_debug("GetPrivateProfileString = %d\n", dividerPos); */
		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.TxLinearityTest_Max
				    [i][k] = (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}
}

void OnInit_DThreshold_ForceTouch_SCapRawDataTest(char *strIniFile)
{
	char str[128], strTemp[MAX_KEY_VALUE_LEN], strValue[MAX_KEY_VALUE_LEN];
	int MaxValue, MinValue;
	int dividerPos = 0;
	char str_tmp[128];
	int index = 0;
	int k = 0, i = 0, j = 0;

	 /*OFF*/
	    GetPrivateProfileString("Basic_Threshold",
				    "ForceTouch_SCapRawDataTest_OFF_Min", "150",
				    str, strIniFile);

	MinValue = atoi(str);

	GetPrivateProfileString("Basic_Threshold",
				"ForceTouch_SCapRawDataTest_OFF_Max", "1000",
				str, strIniFile);

	MaxValue = atoi(str);

	/*Max */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.ForceTouch_SCapRawDataTest_OFF_Max
			[i][j] = MaxValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ForceTouch_ScapRawData_OFF_Max_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.ForceTouch_SCapRawDataTest_OFF_Max[i][k]
					= (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	/*Min */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.ForceTouch_SCapRawDataTest_OFF_Min
			[i][j] = MinValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ForceTouch_ScapRawData_OFF_Min_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.ForceTouch_SCapRawDataTest_OFF_Min[i][k]
					= (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	 /*ON*/
	    GetPrivateProfileString("Basic_Threshold",
				    "ForceTouch_SCapRawDataTest_ON_Min", "150",
				    str, strIniFile);

	MinValue = atoi(str);

	GetPrivateProfileString("Basic_Threshold",
				"ForceTouch_SCapRawDataTest_ON_Max", "1000",
				str, strIniFile);

	MaxValue = atoi(str);

	/*pre_debug("%d:%d\r\n",MinValue, MaxValue); */

	/*Max */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.ForceTouch_SCapRawDataTest_ON_Max
			[i][j] = MaxValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ForceTouch_ScapRawData_ON_Max_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		/*pre_debug("%s:%s\r\n",str, strTemp); */
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.ForceTouch_SCapRawDataTest_ON_Max[i][k]
					= (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}

		}
	}

	/*Min */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.ForceTouch_SCapRawDataTest_ON_Min
			[i][j] = MinValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ForceTouch_ScapRawData_ON_Min_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		/*pre_debug("%s:%s\r\n",str, strTemp); */
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.ForceTouch_SCapRawDataTest_ON_Min[i][k]
					= (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}
}

void OnInit_DThreshold_ForceTouch_SCapCbTest(char *strIniFile)
{
	char str[128], strTemp[MAX_KEY_VALUE_LEN], strValue[MAX_KEY_VALUE_LEN];
	int MaxValue, MinValue;
	int dividerPos = 0;
	char str_tmp[128];
	int index = 0;
	int k = 0, i = 0, j = 0;

	GetPrivateProfileString("Basic_Threshold",
				"ForceTouch_SCapCbTest_ON_Min", "0", str,
				strIniFile);
	MinValue = atoi(str);

	GetPrivateProfileString("Basic_Threshold",
				"ForceTouch_SCapCbTest_ON_Max", "240", str,
				strIniFile);
	MaxValue = atoi(str);

	/*pre_debug("%d:%d\r\n",MinValue, MaxValue); */

	/*Max */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.ForceTouch_SCapCbTest_ON_Max
			[i][j] = MaxValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ForceTouch_ScapCB_ON_Max_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		/*pre_debug("%s:%s\r\n",str, strTemp); */
		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.ForceTouch_SCapCbTest_ON_Max[i][k]
					= (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	/*Min */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.ForceTouch_SCapCbTest_ON_Min
			[i][j] = MinValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ForceTouch_ScapCB_ON_Min_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);
		/*pre_debug("%s:%s\r\n",str, strTemp); */
		if (dividerPos == 0)
			continue;

		index = 0;
		pr_info("%s\r\n", strTemp);

		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.ForceTouch_SCapCbTest_ON_Min[i][k]
					= (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	GetPrivateProfileString("Basic_Threshold",
				"ForceTouch_SCapCbTest_OFF_Min", "0", str,
				strIniFile);
	MinValue = atoi(str);

	GetPrivateProfileString("Basic_Threshold",
				"ForceTouch_SCapCbTest_OFF_Max", "240", str,
				strIniFile);
	MaxValue = atoi(str);

	/*Max */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.ForceTouch_SCapCbTest_OFF_Max
			[i][j] = MaxValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ForceTouch_ScapCB_OFF_Max_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.ForceTouch_SCapCbTest_OFF_Max[i][k]
					= (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}

	/*Min */
	for (i = 0; i < 2; i++) {
		for (j = 0; j < g_ScreenSetParam.iUsedMaxRxNum; j++) {
			g_MCap.ForceTouch_SCapCbTest_OFF_Min
			[i][j] = MinValue;
		}
	}

	for (i = 0; i < 2; i++) {
		snprintf(str, ARRAY_SIZE(str),
			"ForceTouch_ScapCB_OFF_Min_%d", (i + 1));
		dividerPos =
		    GetPrivateProfileString("SpecialSet", str, "NULL", strTemp,
					    strIniFile);

		snprintf(strValue, ARRAY_SIZE(strValue), "%s", strTemp);

		if (dividerPos == 0)
			continue;

		index = 0;
		k = 0;
		memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));

		for (j = 0; j < dividerPos; j++) {
			if (strValue[j] == ',') {
				g_MCap.ForceTouch_SCapCbTest_OFF_Min[i][k]
						= (short)(atoi(str_tmp));

				index = 0;
				memset(str_tmp, 0x00, ARRAY_SIZE(str_tmp));
				k++;
			} else {
				if (strValue[j] == ' ')
					continue;

				str_tmp[index] = strValue[j];
				index++;
			}
		}
	}
}

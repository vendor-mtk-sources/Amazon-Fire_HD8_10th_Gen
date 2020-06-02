/*
 * Copyright (C) 2012-2015, Focaltech Systems (R), All Rights Reserved.
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
#include <linux/string.h>
#include <linux/time.h>
#include <linux/slab.h>
#include "test_lib.h"
#include "Global.h"
#include "Config_FT5822.h"
#include "Test_FT5822.h"
#include "ini.h"

FTS_I2C_READ_FUNCTION fts_i2c_read_test;
FTS_I2C_WRITE_FUNCTION fts_i2c_write_test;

char *g_testparamstring;
/* IIC communication */
int init_i2c_read_func(FTS_I2C_READ_FUNCTION fpI2C_Read)
{
	fts_i2c_read_test = fpI2C_Read;
	return 0;
}

int init_i2c_write_func(FTS_I2C_WRITE_FUNCTION fpI2C_Write)
{
	fts_i2c_write_test = fpI2C_Write;
	return 0;
}

int set_param_data(char *TestParamData)
{
	/*int time_use = 0;*/	/*ms*/

	size_t g_strIcNameLen = ARRAY_SIZE(g_strIcName);

	/*gettimeofday(&time_start, NULL); */ /*Start time*/
	pr_debug("Enter set param_data\n");

	g_testparamstring = TestParamData;	/*get param of ini file*/
	ini_get_key_data(g_testparamstring);	/*get param to struct*/

	/*Set g_ScreenSetParam.iSelectedIC*/
	OnInit_InterfaceCfg(g_testparamstring);

	/* Get IC Name */
	get_ic_name(g_ScreenSetParam.iSelectedIC, g_strIcName, g_strIcNameLen);

	/* Set test items */
	if (IC_FT5822 >> 4 == g_ScreenSetParam.iSelectedIC >> 4) {
		OnInit_FT5822_TestItem(g_testparamstring);
		OnInit_FT5822_BasicThreshold(g_testparamstring);
		OnInit_MCap_DetailThreshold(g_testparamstring);
		SetTestItem_FT5822();
	}

	/*gettimeofday(&time_end, NULL);*/ /*End time*/
	/*	time_use = (time_end.tv_sec - time_start.tv_sec)*1000
	 * + (time_end.tv_usec - time_start.tv_usec)/1000;
	   pr_info("Load Config, use time = %d ms\n", time_use);
	 */
	return 0;
}

boolean start_test_tp(void)
{
	boolean bTestResult = false;

	pr_info("[focal] %s\n", FTS_DRIVER_LIB_INFO);	/* show lib version */
	pr_info("[focal] %s start\n", __func__);
	pr_info("IC_%s Test\n", g_strIcName);

	switch (g_ScreenSetParam.iSelectedIC >> 4) {
	case IC_FT5822 >> 4:
		bTestResult = FT5822_StartTest();
		break;

	default:
		pr_debug("[focal]  Error IC, IC Name: %s, IC Code:  %d\n",
		       g_strIcName, g_ScreenSetParam.iSelectedIC);
		break;
	}

	return bTestResult;
}

int get_test_data(char *pTestData)
{
	int iLen = 0;

	switch (g_ScreenSetParam.iSelectedIC >> 4) {
	case IC_FT5822 >> 4:
		iLen = FT5822_get_test_data(pTestData);
		break;

	default:
		pr_debug("[focal]  Error IC, IC Name: %s, IC Code:  %d\n",
		       g_strIcName, g_ScreenSetParam.iSelectedIC);
		break;
	}

	return iLen;
}

void free_test_param_data(void)
{
	kfree(g_testparamstring);
	g_testparamstring = NULL;
}

int show_lib_ver(char *pLibVer)
{
	int num_read_chars = 0;

	num_read_chars = snprintf(pLibVer, 128, "%s\n", FTS_DRIVER_LIB_INFO);

	return num_read_chars;
}

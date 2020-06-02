/*
 * Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
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

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <asm/unistd.h>

#include "ini.h"
/*#include "scap_test_lib.h"*/
/* Section Symbol */
char CFG_SSL = '[';
char CFG_SSR = ']';
char CFG_NIS = ':';		/* colon character */
char CFG_NTS = '#';		/* comment symbol */
char CFG_EQS = '=';		/* equal sign */

struct ST_INI_FILE_DATA g_st_ini_file_data[MAX_KEY_NUM];

int g_used_key_num;
char *ini_str_trim_r(char *buf);
char *ini_str_trim_l(char *buf);
static int ini_file_get_line(char *filedata, char *buffer, int maxlen);
/*static int ini_split_key_value(char *buf, char **key, char **val);*/
static long atol(char *nptr);

int fts_strncmp(const char *cs, const char *ct, size_t count)
{
	unsigned char c1 = 0, c2 = 0;

	while (count) {
		c1 = TOLOWER(*cs++);
		c2 = TOLOWER(*ct++);
		if (c1 != c2)
			return c1 < c2 ? -1 : 1;

		if (!c1)
			break;

		count--;
	}

	return 0;
}

int ini_get_key(char *filedata, char *section, char *key, char *value)
{
	int i = 0;
	int ret = -2;

	for (i = 0; i < g_used_key_num; i++) {
		if (fts_strncmp
		    (section, g_st_ini_file_data[i].pSectionName,
		     g_st_ini_file_data[i].iSectionNameLen) != 0)
			continue;

		/*
		 * pr_debug("Section Name:%s, Len:%d\n\n",
		 * g_st_ini_file_data[i].pSectionName,
		 * g_st_ini_file_data[i].iSectionNameLen);
		 */
		if (fts_strncmp
		    (key, g_st_ini_file_data[i].pKeyName, strlen(key)) == 0)
			/*g_st_ini_file_data[i].iKeyNameLen) == 0) */
		{
			memcpy(value, g_st_ini_file_data[i].pKeyValue,
			       g_st_ini_file_data[i].iKeyValueLen);
			ret = 0;

			break;
		}
	}

	return ret;
}

int ini_get_sections(char *filedata, unsigned char *sections[], int max)
{
	/*FILE *fp; */
	char buf1[MAX_CFG_BUF + 1];
	int n, n_sections = 0, ret;
	int dataoff = 0;

	while (1) {		/*searching for section */
		ret = CFG_ERR_READ_FILE;
		n = ini_file_get_line(filedata + dataoff, buf1, MAX_CFG_BUF);
		dataoff += n;
		if (n < -1)
			goto cfg_scts_end;

		if (n < 0)
			break;	/* end of section */

		n = strlen(ini_str_trim_l(ini_str_trim_r(buf1)));

		if (n == 0 || buf1[0] == CFG_NTS)
			continue;	/*Blank lines or comment lines */

		ret = CFG_ERR_FILE_FORMAT;

		if (n > 2 && ((buf1[0] == CFG_SSL && buf1[n - 1] != CFG_SSR)))
			goto cfg_scts_end;

		if (buf1[0] == CFG_SSL) {
			if (max != 0) {
				buf1[n - 1] = 0x00;

				strcpy((char *)sections[n_sections], buf1 + 1);
				if (n_sections >= max)
					break;
			}
			n_sections++;
		}
	}

	ret = n_sections;

cfg_scts_end:

	return ret;
}

char *ini_str_trim_r(char *buf)
{
	int len, i;
	char tmp[512];

	memset(tmp, 0, ARRAY_SIZE(tmp));
	len = strlen(buf);
/* tmp = (char *)malloc(len);*/
	memset(tmp, 0x00, len);
	for (i = 0; i < len; i++) {
		if (buf[i] != ' ')
			break;
	}

	if (i < len)
		strncpy(tmp, (buf + i), (len - i));

	strncpy(buf, tmp, len);
	/* free(tmp); */
	return buf;
}

char *ini_str_trim_l(char *buf)
{
	int len, i;
	char tmp[512];

	memset(tmp, 0, ARRAY_SIZE(tmp));
	len = strlen(buf);
	/*tmp = (char *)malloc(len); */
	memset(tmp, 0x00, len);

	for (i = 0; i < len; i++) {
		if (buf[len - i - 1] != ' ')
			break;
	}

	if (i < len)
		strncpy(tmp, buf, len - i);

	strncpy(buf, tmp, len);
	/*free(tmp); */
	return buf;
}

static int ini_file_get_line(char *filedata, char *buffer, int maxlen)
{
	int i = 0;
	int j = 0;
	int iRetNum = -1;
	char ch1 = '\0';

	for (i = 0, j = 0; i < maxlen; j++) {
		ch1 = filedata[j];
		iRetNum = j + 1;

		if (ch1 == '\n' || ch1 == '\r') {	/*line end */
			ch1 = filedata[j + 1];
			if (ch1 == '\n' || ch1 == '\r')
				iRetNum++;

			break;	/* wrap */
		}

		if (ch1 == 0x00) {
			iRetNum = -1;
			break;	/*file end */
		}

		buffer[i++] = ch1;	/*  Carriage returns are ignored */
	}

	buffer[i] = '\0';
	return iRetNum;
#if 0
	for (i = 0, j = 0; i < maxlen; j++) {
		ch1 = filedata[j];
		if (ch1 == '\n' || ch1 == 0x00)
			break;	/* Wrap */
		/* '\f':Page breaks can be considered a valid character */
		if (ch1 == '\f' || ch1 == 0x1A) {
			buffer[i++] = ch1;
			break;
		}
		if (ch1 != '\r')
			buffer[i++] = ch1;
	}
	buffer[i] = '\0';
	return i + 2;
#endif
}

#if 0
static int ini_split_key_value(char *buf, char **key, char **val)
{
	int i, k1, k2, n;

	n = strlen((char *)buf;
	if (n < 1)
		return 0;

	for (i = 0; i < n; i++)
		if (buf[i] != ' ' && buf[i] != '\t')
			break;

	if (i >= n)
		return 0;

	if (buf[i] == '=')
		return -1;

	k1 = i;
	for (i++; i < n; i++)
		if (buf[i] == '=')
			break;

	if (i >= n)
		return -2;
	k2 = i;

	for (i++; i < n; i++)
		if (buf[i] != ' ' && buf[i] != '\t')
			break;

	buf[k2] = '\0';

	*key = buf + k1;
	*val = buf + i;
	return 1;
*}
#endif

int my_atoi(const char *str)
{
	int result = 0;
	int signal = 1;	/* The default is a positive number */

	if ((*str >= '0' && *str <= '9') || *str == '-' || *str == '+') {
		if (*str == '-' || *str == '+') {
			if (*str == '-')
				signal = -1;	/*Enter a negative */
			str++;
		}
	} else
		return 0;

	/*Start Conversion */
	while (*str >= '0' && *str <= '9')
		result = result * 10 + (*str++ - '0');

	return signal * result;
}

int isspace(int x)
{
	if (x == ' ' || x == '\t' || x == '\n' || x == '\f' || x == '\b'
	    || x == '\r')
		return 1;
	else
		return 0;
}

int isdigit(int x)
{
	if (x <= '9' && x >= '0')
		return 1;
	else
		return 0;
}

static long atol(char *nptr)
{
	int c;			/* current char */
	long total;	/* current total */
	int sign;		/*if ''-'',then negative,otherwise positive*/

	/* skip whitespace */
	while (isspace((int)(unsigned char)*nptr))
		++nptr;

	c = (int)(unsigned char)*nptr++;
	sign = c;		/* save sign indication */

	if (c == '-' || c == '+')
		c = (int)(unsigned char)*nptr++;	/* skip sign */

	total = 0;
	while (isdigit(c)) {
		total = 10 * total + (c - '0');	/* accumulate digit */
		c = (int)(unsigned char)*nptr++;	/* get next char */
	}

	if (sign == '-')
		return -total;
	else
		return total;	/* return result, negated if necessary */
}

int atoi(char *nptr)
{
	return (int)atol(nptr);
}

int init_key_data(void)
{
	int i = 0;

	g_used_key_num = 0;

	for (i = 0; i < MAX_KEY_NUM; i++) {
		memset(g_st_ini_file_data[i].pSectionName, 0, MAX_KEY_NAME_LEN);

		memset(g_st_ini_file_data[i].pKeyName, 0, MAX_KEY_NAME_LEN);
		memset(g_st_ini_file_data[i].pKeyValue, 0, MAX_KEY_VALUE_LEN);

		g_st_ini_file_data[i].iSectionNameLen = 0;
		g_st_ini_file_data[i].iKeyNameLen = 0;
		g_st_ini_file_data[i].iKeyValueLen = 0;
	}

	return 1;
}

int ini_get_key_data(char *filedata)
{
	/*FILE *fp; */
	char buf1[MAX_CFG_BUF + 1] = { 0 };
	int n = 0;
	int ret = 0;
	/* int n_sections = 0; */
	int dataoff = 0;
	int iEqualSign = 0;
	int i = 0;

	char tmpSectionName[MAX_CFG_BUF + 1] = { 0 };

	init_key_data();	/*init */
	g_used_key_num = 0;
	/* Search to find items section */
	while (1) {
		ret = CFG_ERR_READ_FILE;
		n = ini_file_get_line(filedata + dataoff, buf1, MAX_CFG_BUF);
		if (n < -1)
			goto cfg_scts_end;

		if (n < 0)
			break;	/*End of file */

		dataoff += n;
		n = strlen(ini_str_trim_l(ini_str_trim_r(buf1)));

		if (n == 0 || buf1[0] == CFG_NTS)
			continue;	/* Blank lines or comment lines */

		ret = CFG_ERR_FILE_FORMAT;
		/*get section name */
		if (n > 2 && ((buf1[0] == CFG_SSL && buf1[n - 1] != CFG_SSR))) {
			pr_debug("Bad Section:%s\n\n", buf1);
			goto cfg_scts_end;	/*bad section */
		}

		if (buf1[0] == CFG_SSL) {
			g_st_ini_file_data[g_used_key_num].iSectionNameLen =
			    n - 2;

			if (MAX_KEY_NAME_LEN <
	g_st_ini_file_data[g_used_key_num].iSectionNameLen) {
				ret = CFG_ERR_OUT_OF_LEN;
				pr_debug
				    ("MAX_KEY_NAME_LEN:CFG_ERR_OUT_OF_LEN\n");
				goto cfg_scts_end;
			}

			buf1[n - 1] = 0x00;
			strcpy((char *)tmpSectionName, buf1 + 1);

			continue;
		}
		/* get section name end */

		strcpy(g_st_ini_file_data[g_used_key_num].pSectionName,
		       tmpSectionName);

		g_st_ini_file_data[g_used_key_num].iSectionNameLen =
		    strlen(tmpSectionName);

		iEqualSign = 0;
		for (i = 0; i < n; i++) {
			if (buf1[i] == CFG_EQS) {
				iEqualSign = i;
				break;
			}
		}

		if (iEqualSign == 0)
			continue;

		/*Equals long ago assigned keys */
		g_st_ini_file_data[g_used_key_num].iKeyNameLen = iEqualSign;

		if (MAX_KEY_NAME_LEN <
		    g_st_ini_file_data[g_used_key_num].iKeyNameLen) {
			ret = CFG_ERR_OUT_OF_LEN;
			pr_debug("MAX_KEY_NAME_LEN: CFG_ERR_OUT_OF_LEN\n\n");
			goto cfg_scts_end;
		}

		memcpy(g_st_ini_file_data[g_used_key_num].pKeyName,
		       buf1, g_st_ini_file_data[g_used_key_num].iKeyNameLen);

		/*After a period equal sign key is assigned */
		g_st_ini_file_data[g_used_key_num].iKeyValueLen =
		    n - iEqualSign - 1;

		if (MAX_KEY_VALUE_LEN <
		    g_st_ini_file_data[g_used_key_num].iKeyValueLen) {
			ret = CFG_ERR_OUT_OF_LEN;
			pr_debug("MAX_KEY_VALUE_LEN: CFG_ERR_OUT_OF_LEN\n\n");
			goto cfg_scts_end;
		}

		memcpy(g_st_ini_file_data[g_used_key_num].pKeyValue,
		       buf1 + iEqualSign + 1,
		       g_st_ini_file_data[g_used_key_num].iKeyValueLen);

		ret = g_used_key_num;

		g_used_key_num++;	/*The cumulative number of parameters */

		if (g_used_key_num > MAX_KEY_NUM) {
			ret = CFG_ERR_TOO_MANY_KEY_NUM;
			pr_debug("MAX_KEY_NUM: CFG_ERR_TOO_MANY_KEY_NUM\n\n");
			goto cfg_scts_end;
		}
	}
	/* ret = n_sections; */
cfg_scts_end:
	return ret;
}

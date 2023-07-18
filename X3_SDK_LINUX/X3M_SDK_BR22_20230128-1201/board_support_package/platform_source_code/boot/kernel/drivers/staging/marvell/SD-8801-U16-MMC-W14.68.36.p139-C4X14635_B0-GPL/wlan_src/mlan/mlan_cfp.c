/**
 * @file mlan_cfp.c
 *
 *  @brief This file contains WLAN client mode channel, frequency and power
 *  related code
 *
 *  (C) Copyright 2009-2019 Marvell International Ltd. All Rights Reserved
 *
 *  MARVELL CONFIDENTIAL
 *  The source code contained or described herein and all documents related to
 *  the source code ("Material") are owned by Marvell International Ltd or its
 *  suppliers or licensors. Title to the Material remains with Marvell
 *  International Ltd or its suppliers and licensors. The Material contains
 *  trade secrets and proprietary and confidential information of Marvell or its
 *  suppliers and licensors. The Material is protected by worldwide copyright
 *  and trade secret laws and treaty provisions. No part of the Material may be
 *  used, copied, reproduced, modified, published, uploaded, posted,
 *  transmitted, distributed, or disclosed in any way without Marvell's prior
 *  express written permission.
 *
 *  No license under any patent, copyright, trade secret or other intellectual
 *  property right is granted to or conferred upon you by disclosure or delivery
 *  of the Materials, either expressly, by implication, inducement, estoppel or
 *  otherwise. Any license under such intellectual property rights must be
 *  express and approved by Marvell in writing.
 *
 */

/*************************************************************
Change Log:
    04/16/2009: initial version
************************************************************/

#include "mlan.h"
#include "mlan_util.h"
#include "mlan_fw.h"
#include "mlan_join.h"
#include "mlan_main.h"

/********************************************************
			Local Variables
********************************************************/

/** 100mW */
#define WLAN_TX_PWR_DEFAULT     20
/** 100mW */
#define WLAN_TX_PWR_00_DEFAULT      20
/** 100mW */
#define WLAN_TX_PWR_US_DEFAULT      20
/** 100mW */
#define WLAN_TX_PWR_JP_BG_DEFAULT   20
/** 200mW */
#define WLAN_TX_PWR_JP_A_DEFAULT    23
/** 100mW */
#define WLAN_TX_PWR_FR_100MW        20
/** 10mW */
#define WLAN_TX_PWR_FR_10MW         10
/** 100mW */
#define WLAN_TX_PWR_EMEA_DEFAULT    20
/** 2000mW */
#define WLAN_TX_PWR_CN_2000MW       33
/** 200mW */
#define WLAN_TX_PWR_200MW   23
/** 1000mW */
#define WLAN_TX_PWR_1000MW   30
/** 30mW */
#define WLAN_TX_PWR_SP_30MW   14
/** 60mW */
#define WLAN_TX_PWR_SP_60MW   17
/** 25mW */
#define WLAN_TX_PWR_25MW   14
/** 250mW */
#define WLAN_TX_PWR_250MW   24

/** Region code mapping */
typedef struct _country_code_mapping {
    /** Region */
	t_u8 country_code[COUNTRY_CODE_LEN];
    /** Code for B/G CFP table */
	t_u8 cfp_code_bg;
    /** Code for A CFP table */
	t_u8 cfp_code_a;
} country_code_mapping_t;

#define EU_CFP_CODE_BG  0x30
#define EU_CFP_CODE_A   0x30

/** Region code mapping table */
static country_code_mapping_t country_code_mapping[] = {
	{"US", 0x10, 0x10},	/* US FCC      */
	{"CA", 0x10, 0x20},	/* IC Canada   */
	{"SG", 0x10, 0x10},	/* Singapore   */
	{"EU", 0x30, 0x30},	/* ETSI        */
	{"AU", 0x30, 0x30},	/* Australia   */
	{"KR", 0x30, 0x30},	/* Republic Of Korea */
	{"JP", 0xFF, 0x40},	/* Japan       */
	{"CN", 0x30, 0x50},	/* China       */
	{"BR", 0x01, 0x09},	/* Brazil      */
	{"RU", 0x30, 0x0f},	/* Russia      */
	{"IN", 0x10, 0x06},	/* India       */
	{"MY", 0x30, 0x06},	/* Malaysia    */
};

/** Country code for ETSI */
static t_u8 eu_country_code_table[][COUNTRY_CODE_LEN] = {
	"AL", "AD", "AT", "AU", "BY", "BE", "BA", "BG", "HR", "CY",
	"CZ", "DK", "EE", "FI", "FR", "MK", "DE", "GR", "HU", "IS",
	"IE", "IT", "KR", "LV", "LI", "LT", "LU", "MT", "MD", "MC",
	"ME", "NL", "NO", "PL", "RO", "RU", "SM", "RS", "SI", "SK",
	"ES", "SE", "CH", "TR", "UA", "UK", "GB"
};

/**
 * The structure for Channel-Frequency-Power table
 */
typedef struct _cfp_table {
    /** Region or Code */
	t_u8 code;
    /** Frequency/Power */
	chan_freq_power_t *cfp;
    /** No of CFP flag */
	int cfp_no;
} cfp_table_t;

/* Format { Channel, Frequency (MHz), MaxTxPower } */
/** Band: 'B/G', Region: USA FCC/Canada IC */
static chan_freq_power_t channel_freq_power_US_BG[] = {
	{1, 2412, WLAN_TX_PWR_US_DEFAULT, MFALSE},
	{2, 2417, WLAN_TX_PWR_US_DEFAULT, MFALSE},
	{3, 2422, WLAN_TX_PWR_US_DEFAULT, MFALSE},
	{4, 2427, WLAN_TX_PWR_US_DEFAULT, MFALSE},
	{5, 2432, WLAN_TX_PWR_US_DEFAULT, MFALSE},
	{6, 2437, WLAN_TX_PWR_US_DEFAULT, MFALSE},
	{7, 2442, WLAN_TX_PWR_US_DEFAULT, MFALSE},
	{8, 2447, WLAN_TX_PWR_US_DEFAULT, MFALSE},
	{9, 2452, WLAN_TX_PWR_US_DEFAULT, MFALSE},
	{10, 2457, WLAN_TX_PWR_US_DEFAULT, MFALSE},
	{11, 2462, WLAN_TX_PWR_US_DEFAULT, MFALSE}
};

/** Band: 'B/G', Region: Europe ETSI/China */
static chan_freq_power_t channel_freq_power_EU_BG[] = {
	{1, 2412, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{2, 2417, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{3, 2422, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{4, 2427, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{5, 2432, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{6, 2437, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{7, 2442, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{8, 2447, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{9, 2452, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{10, 2457, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{11, 2462, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{12, 2467, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE},
	{13, 2472, WLAN_TX_PWR_EMEA_DEFAULT, MFALSE}
};

/** Band: 'B/G', Region: Japan */
static chan_freq_power_t channel_freq_power_JPN41_BG[] = {
	{1, 2412, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{2, 2417, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{3, 2422, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{4, 2427, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{5, 2432, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{6, 2437, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{7, 2442, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{8, 2447, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{9, 2452, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{10, 2457, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{11, 2462, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{12, 2467, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{13, 2472, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE}
};

/** Band: 'B/G', Region: Japan */
static chan_freq_power_t channel_freq_power_JPN40_BG[] = {
	{14, 2484, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE}
};

/** Band: 'B/G', Region: Japan */
static chan_freq_power_t channel_freq_power_JPNFE_BG[] = {
	{1, 2412, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{2, 2417, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{3, 2422, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{4, 2427, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{5, 2432, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{6, 2437, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{7, 2442, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{8, 2447, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{9, 2452, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{10, 2457, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{11, 2462, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{12, 2467, WLAN_TX_PWR_JP_BG_DEFAULT, MTRUE},
	{13, 2472, WLAN_TX_PWR_JP_BG_DEFAULT, MTRUE}
};

/** Band : 'B/G', Region: Brazil */
static chan_freq_power_t channel_freq_power_BR_BG[] = {
	{1, 2412, WLAN_TX_PWR_1000MW, MFALSE},
	{2, 2417, WLAN_TX_PWR_1000MW, MFALSE},
	{3, 2422, WLAN_TX_PWR_1000MW, MFALSE},
	{4, 2427, WLAN_TX_PWR_1000MW, MFALSE},
	{5, 2432, WLAN_TX_PWR_1000MW, MFALSE},
	{6, 2437, WLAN_TX_PWR_1000MW, MFALSE},
	{7, 2442, WLAN_TX_PWR_1000MW, MFALSE},
	{8, 2447, WLAN_TX_PWR_1000MW, MFALSE},
	{9, 2452, WLAN_TX_PWR_1000MW, MFALSE},
	{10, 2457, WLAN_TX_PWR_1000MW, MFALSE},
	{11, 2462, WLAN_TX_PWR_1000MW, MFALSE},
	{12, 2467, WLAN_TX_PWR_1000MW, MFALSE},
	{13, 2472, WLAN_TX_PWR_1000MW, MFALSE},
};

/** Band : 'B/G', Region: Special */
static chan_freq_power_t channel_freq_power_SPECIAL_BG[] = {
	{1, 2412, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{2, 2417, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{3, 2422, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{4, 2427, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{5, 2432, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{6, 2437, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{7, 2442, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{8, 2447, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{9, 2452, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{10, 2457, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{11, 2462, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{12, 2467, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{13, 2472, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE},
	{14, 2484, WLAN_TX_PWR_JP_BG_DEFAULT, MFALSE}
};

/**
 * The 2.4GHz CFP tables
 */
static cfp_table_t cfp_table_BG[] = {
	{
	 0x01,			/* Brazil */
	 channel_freq_power_BR_BG,
	 NELEMENTS(channel_freq_power_BR_BG),
	 },
	{0x10,			/* US FCC */
	 channel_freq_power_US_BG,
	 NELEMENTS(channel_freq_power_US_BG),
	 },
	{0x20,			/* CANADA IC */
	 channel_freq_power_US_BG,
	 NELEMENTS(channel_freq_power_US_BG),
	 },
	{0x30,			/* EU */
	 channel_freq_power_EU_BG,
	 NELEMENTS(channel_freq_power_EU_BG),
	 },
	{0x40,			/* JAPAN */
	 channel_freq_power_JPN40_BG,
	 NELEMENTS(channel_freq_power_JPN40_BG),
	 },
	{0x41,			/* JAPAN */
	 channel_freq_power_JPN41_BG,
	 NELEMENTS(channel_freq_power_JPN41_BG),
	 },
	{0x50,			/* China */
	 channel_freq_power_EU_BG,
	 NELEMENTS(channel_freq_power_EU_BG),
	 },
	{
	 0xfe,			/* JAPAN */
	 channel_freq_power_JPNFE_BG,
	 NELEMENTS(channel_freq_power_JPNFE_BG),
	 },
	{0xff,			/* Special */
	 channel_freq_power_SPECIAL_BG,
	 NELEMENTS(channel_freq_power_SPECIAL_BG),
	 },
/* Add new region here */
};

/** Number of the CFP tables for 2.4GHz */
#define MLAN_CFP_TABLE_SIZE_BG  (NELEMENTS(cfp_table_BG))

/********************************************************
			Global Variables
********************************************************/
/**
 * The table to keep region code
 */
t_u16 region_code_index[MRVDRV_MAX_REGION_CODE] = {
	0x10, 0x20, 0x30, 0x40, 0x41, 0x50, 0xfe, 0xff
};

/** The table to keep CFP code for BG */
t_u16 cfp_code_index_bg[MRVDRV_MAX_CFP_CODE_BG] = { };

/**
 * The rates supported for ad-hoc B mode
 */
t_u8 AdhocRates_B[B_SUPPORTED_RATES] = { 0x82, 0x84, 0x8b, 0x96, 0 };

/**
 * The rates supported for ad-hoc G mode
 */
t_u8 AdhocRates_G[G_SUPPORTED_RATES] = {
	0x8c, 0x12, 0x98, 0x24, 0xb0, 0x48, 0x60, 0x6c, 0x00
};

/**
 * The rates supported for ad-hoc BG mode
 */
t_u8 AdhocRates_BG[BG_SUPPORTED_RATES] = {
	0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24, 0x30, 0x48,
	0x60, 0x6c, 0x00
};

/**
 * The rates supported by the card
 */
t_u16 WlanDataRates[WLAN_SUPPORTED_RATES_EXT] = {
	0x02, 0x04, 0x0B, 0x16, 0x00, 0x0C, 0x12,
	0x18, 0x24, 0x30, 0x48, 0x60, 0x6C, 0x90,
	0x0D, 0x1A, 0x27, 0x34, 0x4E, 0x68, 0x75,
	0x82, 0x0C, 0x1B, 0x36, 0x51, 0x6C, 0xA2,
	0xD8, 0xF3, 0x10E, 0x00
};

/**
 * The rates supported in B mode
 */
t_u8 SupportedRates_B[B_SUPPORTED_RATES] = {
	0x02, 0x04, 0x0b, 0x16, 0x00
};

/**
 * The rates supported in G mode (BAND_G, BAND_G|BAND_GN)
 */
t_u8 SupportedRates_G[G_SUPPORTED_RATES] = {
	0x0c, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6c, 0x00
};

/**
 * The rates supported in BG mode (BAND_B|BAND_G, BAND_B|BAND_G|BAND_GN)
 */
t_u8 SupportedRates_BG[BG_SUPPORTED_RATES] = {
	0x02, 0x04, 0x0b, 0x0c, 0x12, 0x16, 0x18, 0x24, 0x30, 0x48,
	0x60, 0x6c, 0x00
};

/**
 * The rates supported in N mode
 */
t_u8 SupportedRates_N[N_SUPPORTED_RATES] = { 0x02, 0x04, 0 };

/********************************************************
			Local Functions
********************************************************/
/**
 *  @brief Find a character in a string.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param s            A pointer to string
 *  @param c            Character to be located
 *  @param n            The length of string
 *
 *  @return        A pointer to the first occurrence of c in string, or MNULL if c is not found.
 */
static void *
wlan_memchr(pmlan_adapter pmadapter, void *s, int c, int n)
{
	const t_u8 *p = (t_u8 *)s;

	ENTER();

	while (n--) {
		if ((t_u8)c == *p++) {
			LEAVE();
			return (void *)(p - 1);
		}
	}

	LEAVE();
	return MNULL;
}

/**
 *  @brief This function finds the CFP in
 *          cfp_table_BG/A based on region/code and band parameter.
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param region     The region code
 *  @param band       The band
 *  @param cfp_no     A pointer to CFP number
 *
 *  @return           A pointer to CFP
 */
static chan_freq_power_t *
wlan_get_region_cfp_table(pmlan_adapter pmadapter, t_u8 region, t_u8 band,
			  int *cfp_no)
{
	t_u32 i;
	t_u8 cfp_bg, cfp_a;

	ENTER();

	cfp_bg = cfp_a = region;
	if (!region) {
		/* Invalid region code, use CFP code */
		cfp_bg = pmadapter->cfp_code_bg;
	}
	if (band & (BAND_B | BAND_G | BAND_GN)) {
		for (i = 0; i < MLAN_CFP_TABLE_SIZE_BG; i++) {
			PRINTM(MINFO, "cfp_table_BG[%d].code=%d\n", i,
			       cfp_table_BG[i].code);
			/* Check if region/code matches for BG bands */
			if (cfp_table_BG[i].code == cfp_bg) {
				/* Select by band */
				*cfp_no = cfp_table_BG[i].cfp_no;
				LEAVE();
				return cfp_table_BG[i].cfp;
			}
		}
	}

	if (!region)
		PRINTM(MERROR, "Error Band[0x%x] or code[BG:%#x, A:%#x]\n",
		       band, cfp_bg, cfp_a);
	else
		PRINTM(MERROR, "Error Band[0x%x] or region[%#x]\n", band,
		       region);

	LEAVE();
	return MNULL;
}

/**
 *  @brief This function copies dynamic CFP elements from one table to another.
 *         Only copy elements where channel numbers match.
 *
 *  @param pmadapter   A pointer to mlan_adapter structure
 *  @param cfp         Destination table
 *  @param num_cfp     Number of elements in dest table
 *  @param cfp_src     Source table
 *  @param num_cfp_src Number of elements in source table
 */
static t_void
wlan_cfp_copy_dynamic(pmlan_adapter pmadapter,
		      chan_freq_power_t *cfp, t_u8 num_cfp,
		      chan_freq_power_t *cfp_src, t_u8 num_cfp_src)
{
	int i, j;
	ENTER();

	/* first clear dest dynamic blacklisted entries */
	for (i = 0; i < num_cfp; i++)
		cfp[i].dynamic.blacklist = MFALSE;

	/* copy dynamic blacklisted entries from source where channels match */
	if (cfp_src) {
		for (i = 0; i < num_cfp; i++)
			for (j = 0; j < num_cfp_src; j++)
				if (cfp[i].channel == cfp_src[j].channel) {
					cfp[i].dynamic.blacklist =
						cfp_src[j].dynamic.blacklist;
					break;
				}
	}

	LEAVE();
}

/********************************************************
			Global Functions
********************************************************/
/**
 *  @brief This function converts region string to integer code
 *
 *  @param pmadapter        A pointer to mlan_adapter structure
 *  @param country_code     Country string
 *  @param cfp_bg           Pointer to buffer
 *  @param cfp_a            Pointer to buffer
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_misc_country_2_cfp_table_code(pmlan_adapter pmadapter, t_u8 *country_code,
				   t_u8 *cfp_bg, t_u8 *cfp_a)
{
	t_u8 i;

	ENTER();

	/* Look for code in mapping table */
	for (i = 0; i < NELEMENTS(country_code_mapping); i++) {
		if (!memcmp(pmadapter, country_code_mapping[i].country_code,
			    country_code, COUNTRY_CODE_LEN - 1)) {
			*cfp_bg = country_code_mapping[i].cfp_code_bg;
			*cfp_a = country_code_mapping[i].cfp_code_a;
			LEAVE();
			return MLAN_STATUS_SUCCESS;
		}
	}

	/* If still not found, look for code in EU country code table */
	for (i = 0; i < NELEMENTS(eu_country_code_table); i++) {
		if (!memcmp(pmadapter, eu_country_code_table[i],
			    country_code, COUNTRY_CODE_LEN - 1)) {
			*cfp_bg = EU_CFP_CODE_BG;
			*cfp_a = EU_CFP_CODE_A;
			LEAVE();
			return MLAN_STATUS_SUCCESS;
		}
	}

	LEAVE();
	return MLAN_STATUS_FAILURE;
}

/**
 *  @brief This function finds if given country code is in EU table
 *
 *  @param pmadapter        A pointer to mlan_adapter structure
 *  @param country_code     Country string
 *
 *  @return                 MTRUE or MFALSE
 */
t_bool
wlan_is_etsi_country(pmlan_adapter pmadapter, t_u8 *country_code)
{

	t_u8 i;

	ENTER();

	/* Look for code in EU country code table */
	for (i = 0; i < NELEMENTS(eu_country_code_table); i++) {
		if (!memcmp(pmadapter, eu_country_code_table[i],
			    country_code, COUNTRY_CODE_LEN - 1)) {
			LEAVE();
			return MTRUE;
		}
	}

	LEAVE();
	return MFALSE;
}

#define BAND_MASK_5G        0x03
#define ANTENNA_OFFSET      2
/**
 *   @brief This function adjust the antenna index
 *
 *   V16_FW_API: Bit0: ant A, Bit 1:ant B, Bit0 & Bit 1: A+B
 *   8887: case1: 0 - 2.4G ant A,  1- 2.4G antB, 2-- 5G ant C
 *   case2: 0 - 2.4G ant A,  1- 2.4G antB, 0x80- 5G antA, 0x81-5G ant B
 *   @param priv 	 A pointer to mlan_private structure
 *   @param prx_pd   A pointer to the RxPD structure
 *
 *   @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
t_u8
wlan_adjust_antenna(pmlan_private priv, RxPD *prx_pd)
{
	t_u8 antenna = prx_pd->antenna;
	if (prx_pd->antenna == 0xff)
		return 0;

	return antenna;
}

/**
 *  @brief This function adjust the rate index
 *
 *  @param priv    A pointer to mlan_private structure
 *  @param rx_rate rx rate
 *  @param rate_info rate info
 *  @return        rate index
 */
t_u8
wlan_adjust_data_rate(mlan_private *priv, t_u8 rx_rate, t_u8 rate_info)
{
	t_u8 rate_index = 0;
	t_bool sgi_enable = 0;

#define MAX_MCS_NUM_SUPP    16

    /** HT40 */
	sgi_enable = (rate_info & 0x04) >> 2;
	if ((rate_info & MBIT(0)) && (rate_info & MBIT(1)))
		rate_index = MLAN_RATE_INDEX_MCS0 +
			MAX_MCS_NUM_SUPP * 2 * sgi_enable +
			MAX_MCS_NUM_SUPP + rx_rate;
	else if (rate_info & MBIT(0)) /** HT20 */
		rate_index = MLAN_RATE_INDEX_MCS0 +
			MAX_MCS_NUM_SUPP * 2 * sgi_enable + rx_rate;
	else
		rate_index =
			(rx_rate >
			 MLAN_RATE_INDEX_OFDM0) ? rx_rate - 1 : rx_rate;
	return rate_index;
}

#ifdef STA_SUPPORT
#endif /* STA_SUPPORT */

/**
 *  @brief Use index to get the data rate
 *
 *  @param pmadapter   A pointer to mlan_adapter structure
 *  @param index       The index of data rate
 *  @param ht_info     HT info
 *
 *  @return            Data rate or 0
 */
t_u32
wlan_index_to_data_rate(pmlan_adapter pmadapter, t_u8 index, t_u8 ht_info)
{
#define MCS_NUM_SUPP    8
	t_u16 mcs_num_supp = MCS_NUM_SUPP;
	t_u16 mcs_rate[4][MCS_NUM_SUPP] = { {0x1b, 0x36, 0x51, 0x6c, 0xa2, 0xd8, 0xf3, 0x10e},	/*LG 40M */
	{0x1e, 0x3c, 0x5a, 0x78, 0xb4, 0xf0, 0x10e, 0x12c},	/*SG 40M */
	{0x0d, 0x1a, 0x27, 0x34, 0x4e, 0x68, 0x75, 0x82},	/*LG 20M */
	{0x0e, 0x1c, 0x2b, 0x39, 0x56, 0x73, 0x82, 0x90}
	};			/*SG 20M */
	t_u32 rate = 0;

	ENTER();

	if (ht_info & MBIT(0)) {
		if (index == MLAN_RATE_BITMAP_MCS0) {
			if (ht_info & MBIT(2))
				rate = 0x0D;	/* MCS 32 SGI rate */
			else
				rate = 0x0C;	/* MCS 32 LGI rate */
		} else if (index < mcs_num_supp) {
			if (ht_info & MBIT(1)) {
				if (ht_info & MBIT(2))
					rate = mcs_rate[1][index];	/* SGI, 40M */
				else
					rate = mcs_rate[0][index];	/* LGI, 40M */
			} else {
				if (ht_info & MBIT(2))
					rate = mcs_rate[3][index];	/* SGI, 20M */
				else
					rate = mcs_rate[2][index];	/* LGI, 20M */
			}
		} else
			rate = WlanDataRates[0];
	} else {
		/* 11n non HT rates */
		if (index >= WLAN_SUPPORTED_RATES_EXT)
			index = 0;
		rate = WlanDataRates[index];
	}
	LEAVE();
	return rate;
}

/**
 *  @brief Use rate to get the index
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param rate         Data rate
 *
 *  @return                     Index or 0
 */
t_u8
wlan_data_rate_to_index(pmlan_adapter pmadapter, t_u32 rate)
{
	t_u16 *ptr;

	ENTER();
	if (rate) {
		ptr = wlan_memchr(pmadapter, WlanDataRates, (t_u8)rate,
				  sizeof(WlanDataRates));
		if (ptr) {
			LEAVE();
			return (t_u8)(ptr - WlanDataRates);
		}
	}
	LEAVE();
	return 0;
}

/**
 *  @brief Get active data rates
 *
 *  @param pmpriv           A pointer to mlan_private structure
 *  @param bss_mode         The specified BSS mode (Infra/IBSS)
 *  @param config_bands     The specified band configuration
 *  @param rates            The buf to return the active rates
 *
 *  @return                 The number of Rates
 */
t_u32
wlan_get_active_data_rates(mlan_private *pmpriv, t_u32 bss_mode,
			   t_u8 config_bands, WLAN_802_11_RATES rates)
{
	t_u32 k;

	ENTER();

	if (pmpriv->media_connected != MTRUE) {
		k = wlan_get_supported_rates(pmpriv, bss_mode, config_bands,
					     rates);
	} else {
		k = wlan_copy_rates(rates, 0,
				    pmpriv->curr_bss_params.data_rates,
				    pmpriv->curr_bss_params.num_of_rates);
	}

	LEAVE();
	return k;
}

#ifdef STA_SUPPORT
/**
 *  @brief This function search through all the regions cfp table to find the channel,
 *            if the channel is found then gets the MIN txpower of the channel
 *            present in all the regions.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param channel      Channel number.
 *
 *  @return             The Tx power
 */
t_u8
wlan_get_txpwr_of_chan_from_cfp(mlan_private *pmpriv, t_u8 channel)
{
	t_u8 i = 0;
	t_u8 j = 0;
	t_u8 tx_power = 0;
	t_u32 cfp_no;
	chan_freq_power_t *cfp = MNULL;

	ENTER();

	for (i = 0; i < MLAN_CFP_TABLE_SIZE_BG; i++) {
		/* Get CFP */
		cfp = cfp_table_BG[i].cfp;
		cfp_no = cfp_table_BG[i].cfp_no;
		/* Find matching channel and get Tx power */
		for (j = 0; j < cfp_no; j++) {
			if ((cfp + j)->channel == channel) {
				if (tx_power != 0)
					tx_power =
						MIN(tx_power,
						    (cfp + j)->max_tx_power);
				else
					tx_power =
						(t_u8)(cfp + j)->max_tx_power;
				break;
			}
		}
	}

	LEAVE();
	return tx_power;
}

/**
 *  @brief Get the channel frequency power info for a specific channel
 *
 *  @param pmadapter            A pointer to mlan_adapter structure
 *  @param band                 It can be BAND_A, BAND_G or BAND_B
 *  @param channel              The channel to search for
 *  @param region_channel       A pointer to region_chan_t structure
 *
 *  @return                     A pointer to chan_freq_power_t structure or MNULL if not found.
 */

chan_freq_power_t *
wlan_get_cfp_by_band_and_channel(pmlan_adapter pmadapter,
				 t_u8 band,
				 t_u16 channel, region_chan_t *region_channel)
{
	region_chan_t *rc;
	chan_freq_power_t *cfp = MNULL;
	int i, j;

	ENTER();

	for (j = 0; !cfp && (j < MAX_REGION_CHANNEL_NUM); j++) {
		rc = &region_channel[j];

		if (!rc->valid || !rc->pcfp)
			continue;
		switch (rc->band) {
		case BAND_B:
		case BAND_G:
			switch (band) {
			case BAND_GN:
			case BAND_B | BAND_G | BAND_GN:
			case BAND_G | BAND_GN:
			case BAND_B | BAND_G:
			case BAND_B:	/* Matching BAND_B/G */
			case BAND_G:
			case 0:
				break;
			default:
				continue;
			}
			break;
		default:
			continue;
		}
		if (channel == FIRST_VALID_CHANNEL)
			cfp = &rc->pcfp[0];
		else {
			for (i = 0; i < rc->num_cfp; i++) {
				if (rc->pcfp[i].channel == channel) {
					cfp = &rc->pcfp[i];
					break;
				}
			}
		}
	}

	if (!cfp && channel)
		PRINTM(MCMND, "wlan_get_cfp_by_band_and_channel(): cannot find "
		       "cfp by band %d & channel %d\n", band, channel);

	LEAVE();
	return cfp;
}

/**
 *  @brief Find the channel frequency power info for a specific channel
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param band         It can be BAND_A, BAND_G or BAND_B
 *  @param channel      The channel to search for
 *
 *  @return             A pointer to chan_freq_power_t structure or MNULL if not found.
 */
chan_freq_power_t *
wlan_find_cfp_by_band_and_channel(mlan_adapter *pmadapter,
				  t_u8 band, t_u16 channel)
{
	chan_freq_power_t *cfp = MNULL;

	ENTER();

	/* Any station(s) with 11D enabled */
	if (wlan_count_priv_cond(pmadapter, wlan_11d_is_enabled,
				 wlan_is_station) > 0)
		cfp = wlan_get_cfp_by_band_and_channel(pmadapter, band, channel,
						       pmadapter->
						       universal_channel);
	else
		cfp = wlan_get_cfp_by_band_and_channel(pmadapter, band, channel,
						       pmadapter->
						       region_channel);

	LEAVE();
	return cfp;
}

/**
 *  @brief Find the channel frequency power info for a specific frequency
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param band         It can be BAND_A, BAND_G or BAND_B
 *  @param freq         The frequency to search for
 *
 *  @return         Pointer to chan_freq_power_t structure; MNULL if not found
 */
chan_freq_power_t *
wlan_find_cfp_by_band_and_freq(mlan_adapter *pmadapter, t_u8 band, t_u32 freq)
{
	chan_freq_power_t *cfp = MNULL;
	region_chan_t *rc;
	int i, j;

	ENTER();

	for (j = 0; !cfp && (j < MAX_REGION_CHANNEL_NUM); j++) {
		rc = &pmadapter->region_channel[j];

		/* Any station(s) with 11D enabled */
		if (wlan_count_priv_cond(pmadapter, wlan_11d_is_enabled,
					 wlan_is_station) > 0)
			rc = &pmadapter->universal_channel[j];

		if (!rc->valid || !rc->pcfp)
			continue;
		switch (rc->band) {
		case BAND_B:
		case BAND_G:
			switch (band) {
			case BAND_GN:
			case BAND_B | BAND_G | BAND_GN:
			case BAND_G | BAND_GN:
			case BAND_B | BAND_G:
			case BAND_B:
			case BAND_G:
			case 0:
				break;
			default:
				continue;
			}
			break;
		default:
			continue;
		}
		for (i = 0; i < rc->num_cfp; i++) {
			if (rc->pcfp[i].freq == freq) {
				cfp = &rc->pcfp[i];
				break;
			}
		}
	}

	if (!cfp && freq)
		PRINTM(MERROR,
		       "wlan_find_cfp_by_band_and_freq(): cannot find cfp by "
		       "band %d & freq %d\n", band, freq);

	LEAVE();
	return cfp;
}
#endif /* STA_SUPPORT */

/**
 *  @brief Check if Rate Auto
 *
 *  @param pmpriv               A pointer to mlan_private structure
 *
 *  @return                     MTRUE or MFALSE
 */
t_u8
wlan_is_rate_auto(mlan_private *pmpriv)
{
	t_u32 i;
	int rate_num = 0;

	ENTER();

	for (i = 0; i < NELEMENTS(pmpriv->bitmap_rates); i++)
		if (pmpriv->bitmap_rates[i])
			rate_num++;

	LEAVE();
	if (rate_num > 1)
		return MTRUE;
	else
		return MFALSE;
}

/**
 *  @brief Covert Rate Bitmap to Rate index
 *
 *  @param pmadapter    Pointer to mlan_adapter structure
 *  @param rate_bitmap  Pointer to rate bitmap
 *  @param size         Size of the bitmap array
 *
 *  @return             Rate index
 */
int
wlan_get_rate_index(pmlan_adapter pmadapter, t_u16 *rate_bitmap, int size)
{
	int i;

	ENTER();

	for (i = 0; i < size * 8; i++) {
		if (rate_bitmap[i / 16] & (1 << (i % 16))) {
			LEAVE();
			return i;
		}
	}

	LEAVE();
	return -1;
}

/**
 *  @brief Get supported data rates
 *
 *  @param pmpriv           A pointer to mlan_private structure
 *  @param bss_mode         The specified BSS mode (Infra/IBSS)
 *  @param config_bands     The specified band configuration
 *  @param rates            The buf to return the supported rates
 *
 *  @return                 The number of Rates
 */
t_u32
wlan_get_supported_rates(mlan_private *pmpriv, t_u32 bss_mode,
			 t_u8 config_bands, WLAN_802_11_RATES rates)
{
	t_u32 k = 0;

	ENTER();

	if (bss_mode == MLAN_BSS_MODE_INFRA) {
		/* Infra. mode */
		switch (config_bands) {
		case (t_u8)BAND_B:
			PRINTM(MINFO, "Infra Band=%d SupportedRates_B\n",
			       config_bands);
			k = wlan_copy_rates(rates, k, SupportedRates_B,
					    sizeof(SupportedRates_B));
			break;
		case (t_u8)BAND_G:
		case BAND_G | BAND_GN:
			PRINTM(MINFO, "Infra band=%d SupportedRates_G\n",
			       config_bands);
			k = wlan_copy_rates(rates, k, SupportedRates_G,
					    sizeof(SupportedRates_G));
			break;
		case BAND_B | BAND_G:
		case BAND_B | BAND_G | BAND_GN:
			PRINTM(MINFO, "Infra band=%d SupportedRates_BG\n",
			       config_bands);
#ifdef WIFI_DIRECT_SUPPORT
			if (pmpriv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT)
				k = wlan_copy_rates(rates, k, SupportedRates_G,
						    sizeof(SupportedRates_G));
			else
				k = wlan_copy_rates(rates, k, SupportedRates_BG,
						    sizeof(SupportedRates_BG));
#else
			k = wlan_copy_rates(rates, k, SupportedRates_BG,
					    sizeof(SupportedRates_BG));
#endif
			break;
		case BAND_GN:
			PRINTM(MINFO, "Infra band=%d SupportedRates_N\n",
			       config_bands);
			k = wlan_copy_rates(rates, k, SupportedRates_N,
					    sizeof(SupportedRates_N));
			break;
		}
	} else {
		/* Ad-hoc mode */
		switch (config_bands) {
		case (t_u8)BAND_B:
			PRINTM(MINFO, "Band: Adhoc B\n");
			k = wlan_copy_rates(rates, k, AdhocRates_B,
					    sizeof(AdhocRates_B));
			break;
		case (t_u8)BAND_G:
		case BAND_G | BAND_GN:
			PRINTM(MINFO, "Band: Adhoc G only\n");
			k = wlan_copy_rates(rates, k, AdhocRates_G,
					    sizeof(AdhocRates_G));
			break;
		case BAND_B | BAND_G:
		case BAND_B | BAND_G | BAND_GN:
			PRINTM(MINFO, "Band: Adhoc BG\n");
			k = wlan_copy_rates(rates, k, AdhocRates_BG,
					    sizeof(AdhocRates_BG));
			break;
		}
	}

	LEAVE();
	return k;
}

#define COUNTRY_ID_US 0
#define COUNTRY_ID_JP 1
#define COUNTRY_ID_CN 2
#define COUNTRY_ID_EU 3
typedef struct _oper_bw_chan {
	/*non-global operating class */
	t_u8 oper_class;
	/*global operating class */
	t_u8 global_oper_class;
	/*bandwidth 0-20M 1-40M 2-80M 3-160M */
	t_u8 bandwidth;
	/*channel list */
	t_u8 channel_list[13];
} oper_bw_chan;

/** oper class table for US*/
static oper_bw_chan oper_bw_chan_us[] = {
    /** non-Global oper class, global oper class, bandwidth, channel list*/
	{1, 115, 0, {36, 40, 44, 48}},
	{2, 118, 0, {52, 56, 60, 64}},
	{3, 124, 0, {149, 153, 157, 161}},
	{4, 121, 0,
	 {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144}},
	{5, 125, 0, {149, 153, 157, 161, 165}},
	{12, 81, 0, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}},
	{22, 116, 1, {36, 44}},
	{23, 119, 1, {52, 60}},
	{24, 122, 1, {100, 108, 116, 124, 132, 140}},
	{25, 126, 1, {149, 157}},
	{26, 126, 1, {149, 157}},
	{27, 117, 1, {40, 48}},
	{28, 120, 1, {56, 64}},
	{29, 123, 1, {104, 112, 120, 128, 136, 144}},
	{30, 127, 1, {153, 161}},
	{31, 127, 1, {153, 161}},
	{32, 83, 1, {1, 2, 3, 4, 5, 6, 7}},
	{33, 84, 1, {5, 6, 7, 8, 9, 10, 11}},
};

/** oper class table for EU*/
static oper_bw_chan oper_bw_chan_eu[] = {
    /** non-global oper class,global oper class, bandwidth, channel list*/
	{1, 115, 0, {36, 40, 44, 48}},
	{2, 118, 0, {52, 56, 60, 64}},
	{3, 121, 0, {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140}},
	{4, 81, 0, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}},
	{5, 116, 1, {36, 44}},
	{6, 119, 1, {52, 60}},
	{7, 122, 1, {100, 108, 116, 124, 132}},
	{8, 117, 1, {40, 48}},
	{9, 120, 1, {56, 64}},
	{10, 123, 1, {104, 112, 120, 128, 136}},
	{11, 83, 1, {1, 2, 3, 4, 5, 6, 7, 8, 9}},
	{12, 84, 1, {5, 6, 7, 8, 9, 10, 11, 12, 13}},
	{17, 125, 0, {149, 153, 157, 161, 165, 169}},
};

/** oper class table for Japan*/
static oper_bw_chan oper_bw_chan_jp[] = {
    /** non-Global oper class,global oper class, bandwidth, channel list*/
	{1, 115, 0, {34, 38, 42, 46, 36, 40, 44, 48}},
	{30, 81, 0, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}},
	{31, 82, 0, {14}},
	{32, 118, 0, {52, 56, 60, 64}},
	{33, 118, 0, {52, 56, 60, 64}},
	{34, 121, 0, {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140}},
	{35, 121, 0, {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140}},
	{36, 116, 1, {36, 44}},
	{37, 119, 1, {52, 60}},
	{38, 119, 1, {52, 60}},
	{39, 122, 1, {100, 108, 116, 124, 132}},
	{40, 122, 1, {100, 108, 116, 124, 132}},
	{41, 117, 1, {40, 48}},
	{42, 120, 1, {56, 64}},
	{43, 120, 1, {56, 64}},
	{44, 123, 1, {104, 112, 120, 128, 136}},
	{45, 123, 1, {104, 112, 120, 128, 136}},
	{56, 83, 1, {1, 2, 3, 4, 5, 6, 7, 8, 9}},
	{57, 84, 1, {5, 6, 7, 8, 9, 10, 11, 12, 13}},
	{58, 121, 0, {100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140}},
};

/** oper class table for China*/
static oper_bw_chan oper_bw_chan_cn[] = {
    /** non-Global oper class,global oper class, bandwidth, channel list*/
	{1, 115, 0, {36, 40, 44, 48}},
	{2, 118, 0, {52, 56, 60, 64}},
	{3, 125, 0, {149, 153, 157, 161, 165}},
	{4, 116, 1, {36, 44}},
	{5, 119, 1, {52, 60}},
	{6, 126, 1, {149, 157}},
	{7, 81, 0, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}},
	{8, 83, 0, {1, 2, 3, 4, 5, 6, 7, 8, 9}},
	{9, 84, 1, {5, 6, 7, 8, 9, 10, 11, 12, 13}},
};

/**
 *  @brief Get non-global operaing class table according to country
 *
 *  @param pmpriv             A pointer to mlan_private structure
 *  @param arraysize          A pointer to table size
 *
 *  @return                   A pointer to oper_bw_chan
 */
oper_bw_chan *
wlan_get_nonglobal_operclass_table(mlan_private *pmpriv, int *arraysize)
{
	t_u8 country_code[][COUNTRY_CODE_LEN] = { "US", "JP", "CN" };
	int country_id = 0;
	oper_bw_chan *poper_bw_chan = MNULL;

	ENTER();

	for (country_id = 0; country_id < 3; country_id++)
		if (!memcmp
		    (pmpriv->adapter, pmpriv->adapter->country_code,
		     country_code[country_id], COUNTRY_CODE_LEN - 1))
			break;
	if (country_id >= 3)
		country_id = COUNTRY_ID_US;	/*Set default to US */
	if (wlan_is_etsi_country
	    (pmpriv->adapter, pmpriv->adapter->country_code))
		country_id = COUNTRY_ID_EU;
				    /** Country in EU */

	switch (country_id) {
	case COUNTRY_ID_US:
		poper_bw_chan = oper_bw_chan_us;
		*arraysize = sizeof(oper_bw_chan_us);
		break;
	case COUNTRY_ID_JP:
		poper_bw_chan = oper_bw_chan_jp;
		*arraysize = sizeof(oper_bw_chan_jp);
		break;
	case COUNTRY_ID_CN:
		poper_bw_chan = oper_bw_chan_cn;
		*arraysize = sizeof(oper_bw_chan_cn);
		break;
	case COUNTRY_ID_EU:
		poper_bw_chan = oper_bw_chan_eu;
		*arraysize = sizeof(oper_bw_chan_eu);
		break;
	default:
		PRINTM(MERROR, "Country not support!\n");
		break;
	}

	LEAVE();
	return poper_bw_chan;
}

/**
 *  @brief Check validation of given channel and oper class
 *
 *  @param pmpriv             A pointer to mlan_private structure
 *  @param channel            Channel number
 *  @param oper_class         operating class
 *
 *  @return                   MLAN_STATUS_PENDING --success, otherwise fail
 */
mlan_status
wlan_check_operclass_validation(mlan_private *pmpriv, t_u8 channel,
				t_u8 oper_class)
{
	int arraysize = 0, i = 0, channum = 0;
	oper_bw_chan *poper_bw_chan = MNULL;
	t_u8 center_freqs[] = { 42, 50, 58, 106, 114, 122, 138, 155 };

	ENTER();

	for (i = 0; i < sizeof(center_freqs); i++) {
		if (channel == center_freqs[i]) {
			PRINTM(MERROR, "Invalid channel number %d!\n", channel);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}
	}
	if (oper_class <= 0 || oper_class > 130) {
		PRINTM(MERROR, "Invalid operating class!\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	poper_bw_chan = wlan_get_nonglobal_operclass_table(pmpriv, &arraysize);

	if (!poper_bw_chan) {
		PRINTM(MCMND, "Operating class table do not find!\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	for (i = 0; i < arraysize / sizeof(oper_bw_chan); i++) {
		if (poper_bw_chan[i].oper_class == oper_class ||
		    poper_bw_chan[i].global_oper_class == oper_class) {
			for (channum = 0;
			     channum < sizeof(poper_bw_chan[i].channel_list);
			     channum++) {
				if (poper_bw_chan[i].channel_list[channum] &&
				    poper_bw_chan[i].channel_list[channum] ==
				    channel) {
					LEAVE();
					return MLAN_STATUS_SUCCESS;
				}
			}
		}
	}

	PRINTM(MCMND, "Operating class %d do not match channel %d!\n",
	       oper_class, channel);
	LEAVE();
	return MLAN_STATUS_FAILURE;
}

/**
 *  @brief Get current operating class from channel and bandwidth
 *
 *  @param pmpriv             A pointer to mlan_private structure
 *  @param channel            Channel number
 *  @param bw                 Bandwidth
 *  @param oper_class         A pointer to current operating class
 *
 *  @return                   MLAN_STATUS_PENDING --success, otherwise fail
 */
mlan_status
wlan_get_curr_oper_class(mlan_private *pmpriv, t_u8 channel, t_u8 bw,
			 t_u8 *oper_class)
{
	oper_bw_chan *poper_bw_chan = MNULL;
	t_u8 center_freqs[] = { 42, 50, 58, 106, 114, 122, 138, 155 };
	int i = 0, arraysize = 0, channum = 0;

	ENTER();

	poper_bw_chan = wlan_get_nonglobal_operclass_table(pmpriv, &arraysize);

	if (!poper_bw_chan) {
		PRINTM(MCMND, "Operating class table do not find!\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	for (i = 0; i < sizeof(center_freqs); i++) {
		if (channel == center_freqs[i]) {
			PRINTM(MERROR, "Invalid channel number %d!\n", channel);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}
	}

	for (i = 0; i < arraysize / sizeof(oper_bw_chan); i++) {
		if (poper_bw_chan[i].bandwidth == bw) {
			for (channum = 0;
			     channum < sizeof(poper_bw_chan[i].channel_list);
			     channum++) {
				if (poper_bw_chan[i].channel_list[channum] &&
				    poper_bw_chan[i].channel_list[channum] ==
				    channel) {
					*oper_class =
						poper_bw_chan[i].oper_class;
					return MLAN_STATUS_SUCCESS;
				}
			}
		}
	}

	PRINTM(MCMND, "Operating class not find!\n");
	LEAVE();
	return MLAN_STATUS_FAILURE;
}

/**
 *  @brief Add Supported operating classes IE
 *
 *  @param pmpriv             A pointer to mlan_private structure
 *  @param pptlv_out          A pointer to TLV to fill in
 *  @param curr_oper_class    Current operating class
 *
 *  @return                   Length
 */
int
wlan_add_supported_oper_class_ie(IN mlan_private *pmpriv, OUT t_u8 **pptlv_out,
				 t_u8 curr_oper_class)
{

	t_u8 oper_class_us[] =
		{ 1, 2, 3, 4, 5, 12, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
	 33
	};
	t_u8 oper_class_eu[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 17
	};
	t_u8 oper_class_jp[] =
		{ 1, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44,
	 45, 56, 57, 58
	};
	t_u8 oper_class_cn[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9
	};
	t_u8 country_code[][COUNTRY_CODE_LEN] = { "US", "JP", "CN" };
	int country_id = 0, ret = 0;
	MrvlIETypes_SuppOperClass_t *poper_class = MNULL;

	ENTER();

	for (country_id = 0; country_id < 3; country_id++)
		if (!memcmp
		    (pmpriv->adapter, pmpriv->adapter->country_code,
		     country_code[country_id], COUNTRY_CODE_LEN - 1))
			break;
	if (country_id >= 3)
		country_id = COUNTRY_ID_US;	/*Set default to US */
	if (wlan_is_etsi_country
	    (pmpriv->adapter, pmpriv->adapter->country_code))
		country_id = COUNTRY_ID_EU;
				    /** Country in EU */
	poper_class = (MrvlIETypes_SuppOperClass_t *) * pptlv_out;
	memset(pmpriv->adapter, poper_class, 0,
	       sizeof(MrvlIETypes_SuppOperClass_t));
	poper_class->header.type = wlan_cpu_to_le16(REGULATORY_CLASS);
	if (country_id == COUNTRY_ID_US) {
		poper_class->header.len = sizeof(oper_class_us);
		memcpy(pmpriv->adapter, &poper_class->oper_class, oper_class_us,
		       sizeof(oper_class_us));
	} else if (country_id == COUNTRY_ID_JP) {
		poper_class->header.len = sizeof(oper_class_jp);
		memcpy(pmpriv->adapter, &poper_class->oper_class, oper_class_jp,
		       sizeof(oper_class_jp));
	} else if (country_id == COUNTRY_ID_CN) {
		poper_class->header.len = sizeof(oper_class_cn);
		memcpy(pmpriv->adapter, &poper_class->oper_class, oper_class_cn,
		       sizeof(oper_class_cn));
	} else if (country_id == COUNTRY_ID_EU) {
		poper_class->header.len = sizeof(oper_class_eu);
		memcpy(pmpriv->adapter, &poper_class->oper_class, oper_class_eu,
		       sizeof(oper_class_eu));
	}
	poper_class->current_oper_class = curr_oper_class;
	poper_class->header.len += sizeof(poper_class->current_oper_class);
	DBG_HEXDUMP(MCMD_D, "Operating class", (t_u8 *)poper_class,
		    sizeof(MrvlIEtypesHeader_t) + poper_class->header.len);
	ret = sizeof(MrvlIEtypesHeader_t) + poper_class->header.len;
	*pptlv_out += ret;
	poper_class->header.len = wlan_cpu_to_le16(poper_class->header.len);

	LEAVE();
	return ret;
}

/**
 *  @brief This function sets region table.
 *
 *  @param pmpriv  A pointer to mlan_private structure
 *  @param region  The region code
 *  @param band    The band
 *
 *  @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_set_regiontable(mlan_private *pmpriv, t_u8 region, t_u8 band)
{
	mlan_adapter *pmadapter = pmpriv->adapter;
	int i = 0, j;
	chan_freq_power_t *cfp;
	int cfp_no;
	region_chan_t region_chan_old[MAX_REGION_CHANNEL_NUM];
	t_u8 cfp_code_bg = region;
	/* t_u8 cfp_code_a = region; */

	ENTER();

	memcpy(pmadapter, region_chan_old, pmadapter->region_channel,
	       sizeof(pmadapter->region_channel));
	memset(pmadapter, pmadapter->region_channel, 0,
	       sizeof(pmadapter->region_channel));

	if (band & (BAND_B | BAND_G | BAND_GN)) {
		if (pmadapter->cfp_code_bg)
			cfp_code_bg = pmadapter->cfp_code_bg;
		PRINTM(MCMND, "wlan_set_regiontable 2.4G 0x%x\n", cfp_code_bg);
		cfp = wlan_get_region_cfp_table(pmadapter, cfp_code_bg,
						BAND_G | BAND_B | BAND_GN,
						&cfp_no);
		if (cfp) {
			pmadapter->region_channel[i].num_cfp = (t_u8)cfp_no;
			pmadapter->region_channel[i].pcfp = cfp;
		} else {
			PRINTM(MERROR, "wrong region code %#x in Band B-G\n",
			       region);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}
		pmadapter->region_channel[i].valid = MTRUE;
		pmadapter->region_channel[i].region = region;
		if (band & BAND_GN)
			pmadapter->region_channel[i].band = BAND_G;
		else
			pmadapter->region_channel[i].band =
				(band & BAND_G) ? BAND_G : BAND_B;

		for (j = 0; j < MAX_REGION_CHANNEL_NUM; j++) {
			if (region_chan_old[j].band & (BAND_B | BAND_G))
				break;
		}
		if ((j < MAX_REGION_CHANNEL_NUM) && region_chan_old[j].valid)
			wlan_cfp_copy_dynamic(pmadapter, cfp, cfp_no,
					      region_chan_old[j].pcfp,
					      region_chan_old[j].num_cfp);
		else
			wlan_cfp_copy_dynamic(pmadapter, cfp, cfp_no, MNULL, 0);
		i++;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Get if scan type is passive or not on a certain channel for b/g band
 *
 *  @param priv    Private driver information structure
 *  @param chnl Channel to determine scan type
 *
 *  @return
 *    - MTRUE if scan type is passive
 *    - MFALSE otherwise
 */

t_bool
wlan_bg_scan_type_is_passive(mlan_private *priv, t_u8 chnl)
{
	int i, j;
	t_bool passive = MFALSE;
	chan_freq_power_t *pcfp = MNULL;

	ENTER();

	/*get the cfp table first */
	for (i = 0; i < MAX_REGION_CHANNEL_NUM; i++) {
		if (priv->adapter->region_channel[i].band & (BAND_B | BAND_G)) {
			pcfp = priv->adapter->region_channel[i].pcfp;
			break;
		}
	}

	if (!pcfp) {
		/*This means operation in BAND-B or BAND_G is not support, we can
		 * just return false here*/
		goto done;
	}

	/*get the bg scan type according to chan num */
	for (j = 0; j < priv->adapter->region_channel[i].num_cfp; j++) {
		if (pcfp[j].channel == chnl) {
			passive = pcfp[j].passive_scan_or_radar_detect;
			break;
		}
	}

done:
	LEAVE();
	return passive;
}

/**
 *  @brief Get if a channel is disabled or not
 *
 *  @param priv     Private driver information structure
 *  @param band     Band to check
 *  @param chan     Channel to check
 *
 *  @return
 *    - MTRUE if channel is disabled
 *    - MFALSE otherwise
 */

t_bool
wlan_is_chan_disabled(mlan_private *priv, t_u8 band, t_u8 chan)
{
	int i, j;
	t_bool disabled = MFALSE;
	chan_freq_power_t *pcfp = MNULL;

	ENTER();

	/* get the cfp table first */
	for (i = 0; i < MAX_REGION_CHANNEL_NUM; i++) {
		if (priv->adapter->region_channel[i].band & band) {
			pcfp = priv->adapter->region_channel[i].pcfp;
			break;
		}
	}

	if (pcfp) {
		/* check table according to chan num */
		for (j = 0; j < priv->adapter->region_channel[i].num_cfp; j++) {
			if (pcfp[j].channel == chan) {
				if (pcfp[j].dynamic.
				    flags & MARVELL_CHANNEL_DISABLED)
					disabled = MTRUE;
				break;
			}
		}
	}

	LEAVE();
	return disabled;
}

/**
 *  @brief Get if a channel is blacklisted or not
 *
 *  @param priv     Private driver information structure
 *  @param band     Band to check
 *  @param chan     Channel to check
 *
 *  @return
 *    - MTRUE if channel is blacklisted
 *    - MFALSE otherwise
 */

t_bool
wlan_is_chan_blacklisted(mlan_private *priv, t_u8 band, t_u8 chan)
{
	int i, j;
	t_bool blacklist = MFALSE;
	chan_freq_power_t *pcfp = MNULL;

	ENTER();

	/*get the cfp table first */
	for (i = 0; i < MAX_REGION_CHANNEL_NUM; i++) {
		if (priv->adapter->region_channel[i].band & band) {
			pcfp = priv->adapter->region_channel[i].pcfp;
			break;
		}
	}

	if (pcfp) {
		/*check table according to chan num */
		for (j = 0; j < priv->adapter->region_channel[i].num_cfp; j++) {
			if (pcfp[j].channel == chan) {
				blacklist = pcfp[j].dynamic.blacklist;
				break;
			}
		}
	}

	LEAVE();
	return blacklist;
}

/**
 *  @brief Set a channel as blacklisted or not
 *
 *  @param priv     Private driver information structure
 *  @param band     Band to check
 *  @param chan     Channel to check
 *  @param bl       Blacklist if MTRUE
 *
 *  @return
 *    - MTRUE if channel setting is updated
 *    - MFALSE otherwise
 */

t_bool
wlan_set_chan_blacklist(mlan_private *priv, t_u8 band, t_u8 chan, t_bool bl)
{
	int i, j;
	t_bool set_bl = MFALSE;
	chan_freq_power_t *pcfp = MNULL;

	ENTER();

	/*get the cfp table first */
	for (i = 0; i < MAX_REGION_CHANNEL_NUM; i++) {
		if (priv->adapter->region_channel[i].band & band) {
			pcfp = priv->adapter->region_channel[i].pcfp;
			break;
		}
	}

	if (pcfp) {
		/*check table according to chan num */
		for (j = 0; j < priv->adapter->region_channel[i].num_cfp; j++) {
			if (pcfp[j].channel == chan) {
				pcfp[j].dynamic.blacklist = bl;
				set_bl = MTRUE;
				break;
			}
		}
	}

	LEAVE();
	return set_bl;
}

/**
 *  @brief	Get power tables and cfp tables for set region code
 *			into the IOCTL request buffer
 *
 *  @param pmadapter	Private mlan adapter structure
 *  @param pioctl_req	Pointer to the IOCTL request structure
 *
 *  @return	success, otherwise fail
 *
 */
mlan_status
wlan_get_cfpinfo(IN pmlan_adapter pmadapter, IN pmlan_ioctl_req pioctl_req)
{
	chan_freq_power_t *cfp_bg = MNULL;
	t_u32 cfp_no_bg = 0;
	t_u32 len = 0, size = 0;
	t_u8 *req_buf, *tmp;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!pioctl_req || !pioctl_req->pbuf) {
		PRINTM(MERROR, "MLAN IOCTL information is not present!\n");
		ret = MLAN_STATUS_FAILURE;
		goto out;
	}
	/* Calculate the total response size required to return region,
	 * country codes, cfp tables and power tables
	 */
	size = sizeof(pmadapter->country_code) + sizeof(pmadapter->region_code);
	/* Add size to store region, country and environment codes */
	size += sizeof(t_u32);

	/* Get cfp table and its size corresponding to the region code */
	cfp_bg = wlan_get_region_cfp_table(pmadapter, pmadapter->region_code,
					   BAND_G | BAND_B, &cfp_no_bg);
	size += cfp_no_bg * sizeof(chan_freq_power_t);
	/* Check information buffer length of MLAN IOCTL */
	if (pioctl_req->buf_len < size) {
		PRINTM(MWARN,
		       "MLAN IOCTL information buffer length is too short.\n");
		pioctl_req->buf_len_needed = size;
		pioctl_req->status_code = MLAN_ERROR_INVALID_PARAMETER;
		ret = MLAN_STATUS_RESOURCE;
		goto out;
	}
	/* Copy the total size of region code, country code and environment
	 * in first four bytes of the IOCTL request buffer and then copy
	 * codes respectively in following bytes
	 */
	req_buf = (t_u8 *)pioctl_req->pbuf;
	size = sizeof(pmadapter->country_code) + sizeof(pmadapter->region_code);
	tmp = (t_u8 *)&size;
	memcpy(pmadapter, req_buf, tmp, sizeof(size));
	len += sizeof(size);
	memcpy(pmadapter, req_buf + len, &pmadapter->region_code,
	       sizeof(pmadapter->region_code));
	len += sizeof(pmadapter->region_code);
	memcpy(pmadapter, req_buf + len, &pmadapter->country_code,
	       sizeof(pmadapter->country_code));
	len += sizeof(pmadapter->country_code);
	/* copy the cfp table size followed by the entire table */
	if (!cfp_bg)
		goto out;
	size = cfp_no_bg * sizeof(chan_freq_power_t);
	memcpy(pmadapter, req_buf + len, tmp, sizeof(size));
	len += sizeof(size);
	memcpy(pmadapter, req_buf + len, cfp_bg, size);
	len += size;
out:
	if (pioctl_req)
		pioctl_req->data_read_written = len;

	LEAVE();
	return ret;
}

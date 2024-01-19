// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright Fiona Klute <fiona.klute@gmx.de> */

#include <linux/device.h>
#include <linux/of.h>
#include <linux/module.h>
#include <net/mac80211.h>
#include "main.h"
#include "coex.h"
#include "debug.h"
#include "phy.h"
#include "reg.h"
#include "rx.h"
#include "rtw8703b.h"
#include "rtw8703b_tables.h"
/* 8703b and 8723d have a lot of similarities, so I can reuse
 * code. The shared code should be moved to a support module or
 * similar, though. */
#include "rtw8723d.h"

/*
 * BIT(11) should be 1 for 8703B *and* 8723D, which means LNA uses 4
 * bit for CCK rates in report, not 3. Vendor driver logs a warning if
 * it's 0, but handles the case.
 *
 * Purpose of other parts of this register is unknown, 8723cs driver
 * code indicates some other chips use certain bits for antenna
 * diversity.
 */
#define REG_BB_AMP 0x0950
#define BIT_MASK_RX_LNA (BIT(11))

#define GET_PHY_STAT_AGC_GAIN_A(phy_stat)                                   \
	(le32_get_bits(*((__le32 *)(phy_stat) + 0x00), GENMASK(6, 0)))

#define GET_PHY_STAT_PWDB(phy_stat)                                         \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x01), GENMASK(7, 0))
#define GET_PHY_STAT_VGA(phy_stat)                                          \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x01), GENMASK(12, 8))
#define GET_PHY_STAT_LNA_L(phy_stat)                                        \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x01), GENMASK(15, 13))
/* the high LNA stat bit if 4 bit format is used */
#define GET_PHY_STAT_LNA_H(phy_stat)                                        \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x01), BIT(23))
#define BIT_LNA_H_MASK BIT(3)
#define BIT_LNA_L_MASK GENMASK(2, 0)

#define GET_PHY_STAT_CFO_TAIL_A(phy_stat)                                   \
	(le32_get_bits(*((__le32 *)(phy_stat) + 0x02), GENMASK(15, 8)))
#define GET_PHY_STAT_RXEVM_A(phy_stat)					    \
	(le32_get_bits(*((__le32 *)(phy_stat) + 0x02), GENMASK(31, 24)))
#define GET_PHY_STAT_RXSNR_A(phy_stat)                                      \
	(le32_get_bits(*((__le32 *)(phy_stat) + 0x03), GENMASK(31, 24)))

#define GET_RX_DESC_BW(rxdesc)                                              \
	(le32_get_bits(*((__le32 *)(rxdesc) + 0x04), GENMASK(31, 24)))

#define BIT_MASK_TXQ_INIT (BIT(7))
#define WLAN_RL_VAL 0x3030
/* disable BAR */
#define WLAN_BAR_VAL 0x0201ffff
#define WLAN_PIFS_VAL 0
#define WLAN_RX_PKT_LIMIT 0x18
#define WLAN_SLOT_TIME 0x09
#define WLAN_SPEC_SIFS 0x100a
#define WLAN_MAX_AGG_NR 0x1f
#define WLAN_AMPDU_MAX_TIME 0x70

/* unit is 32us */
#define TBTT_PROHIBIT_SETUP_TIME 0x04
#define TBTT_PROHIBIT_HOLD_TIME 0x80
#define TBTT_PROHIBIT_HOLD_TIME_STOP_BCN 0x64

/* slightly different from 8723d in vendor driver */
#define RTW_DEF_OFDM_SWING_INDEX_8703B	30

/* raw pkt_stat->drv_info_sz is in unit of 8-bytes */
#define RX_DRV_INFO_SZ_UNIT_8703B 8

#define TRANS_SEQ_END \
	{ \
		0xFFFF, \
		RTW_PWR_CUT_ALL_MSK, \
		RTW_PWR_INTF_ALL_MSK, \
		0, \
		RTW_PWR_CMD_END, 0, 0}


/* rssi in percentage % (dbm = % - 100) */
/* These are used to select simple signal quality levels, might need
 * tweaking. Same for rf_para tables below. */
static const u8 wl_rssi_step_8703b[] = {60, 50, 44, 30};
static const u8 bt_rssi_step_8703b[] = {30, 30, 30, 30};
static const struct coex_5g_afh_map afh_5g_8703b[] = { {0, 0, 0} };

/* wl_tx_dec_power, bt_tx_dec_power, wl_rx_gain, bt_rx_lna_constrain */
static const struct coex_rf_para rf_para_tx_8703b[] = {
	{0, 0, false, 7},  /* for normal */
	{0, 10, false, 7}, /* for WL-CPT */
	{1, 0, true, 4},
	{1, 2, true, 4},
	{1, 10, true, 4},
	{1, 15, true, 4}
};

static const struct coex_rf_para rf_para_rx_8703b[] = {
	{0, 0, false, 7},  /* for normal */
	{0, 10, false, 7}, /* for WL-CPT */
	{1, 0, true, 5},
	{1, 2, true, 5},
	{1, 10, true, 5},
	{1, 15, true, 5}
};

static const struct rtw_pwr_seq_cmd trans_carddis_to_cardemu_8703b[] = {
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(7), 0},
	TRANS_SEQ_END,
};

static const struct rtw_pwr_seq_cmd trans_cardemu_to_carddis_8703b[] = {
	{0x0023,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(4), BIT(4)},
	{0x0007,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_SDIO_MSK | RTW_PWR_INTF_USB_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, 0xFF, 0x20},
	{0x0006,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), 0},
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(7), BIT(7)},
	TRANS_SEQ_END,
};

static const struct rtw_pwr_seq_cmd trans_cardemu_to_act_8703b[] = {
	{0x0020,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},
	{0x0067,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(4), 0},
	{0x0001,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_DELAY, 1, RTW_PWR_DELAY_MS},
	{0x0000,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(5), 0},
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, (BIT(4) | BIT(3) | BIT(2)), 0},
	{0x0075,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},
	{0x0004,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(3), BIT(3)},
	{0x0004,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(3), 0},
	/* wait for power ready */
	{0x0006,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_POLLING, BIT(1), BIT(1)},
	{0x0075,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_PCI_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), 0},
	{0x0006,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(7), 0},
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, (BIT(4) | BIT(3)), 0},
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_POLLING, BIT(0), 0},
	{0x0010,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(6), BIT(6)},
	{0x0049,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), BIT(1)},
	{0x0063,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), BIT(1)},
	{0x0062,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), 0},
	{0x0058,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},
	{0x005A,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), BIT(1)},
	{0x0068,
	 RTW_PWR_CUT_TEST_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(3), BIT(3)},
	{0x0069,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(6), BIT(6)},
	TRANS_SEQ_END,
};

static const struct rtw_pwr_seq_cmd trans_act_to_cardemu_8703b[] = {
	{0x001f,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, 0xff, 0},
	{0x0049,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), 0},
	{0x0006,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), BIT(0)},
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(1), BIT(1)},
	{0x0005,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_POLLING, BIT(1), 0},
	{0x0010,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_ALL_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(6), 0},
	{0x0000,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(5), BIT(5)},
	{0x0020,
	 RTW_PWR_CUT_ALL_MSK,
	 RTW_PWR_INTF_USB_MSK | RTW_PWR_INTF_SDIO_MSK,
	 RTW_PWR_ADDR_MAC,
	 RTW_PWR_CMD_WRITE, BIT(0), 0},
	TRANS_SEQ_END,
};

static const struct rtw_pwr_seq_cmd *card_enable_flow_8703b[] = {
	trans_carddis_to_cardemu_8703b,
	trans_cardemu_to_act_8703b,
	NULL
};

static const struct rtw_pwr_seq_cmd *card_disable_flow_8703b[] = {
	trans_act_to_cardemu_8703b,
	trans_cardemu_to_carddis_8703b,
	NULL
};

static const struct rtw_rfe_def rtw8703b_rfe_defs[] = {
	[0] = { .phy_pg_tbl	= &rtw8703b_bb_pg_tbl,
		.txpwr_lmt_tbl	= &rtw8703b_txpwr_lmt_tbl,},
};


// I can't find an equivalent to gapq in the vendor driver, the rest
// is the same there. See NORMAL_PAGE_NUM_* in include/rtl8703b_hal.h
static const struct rtw_page_table page_table_8703b[] = {
	{12, 2, 2, 0, 1},
	{12, 2, 2, 0, 1},
	{12, 2, 2, 0, 1},
	{12, 2, 2, 0, 1},
	{12, 2, 2, 0, 1},
};

// can't find anything similar in the vendor driver, but it's the same
// for all the other rtw88 chipsets...
static const struct rtw_rqpn rqpn_table_8703b[] = {
	{RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_NORMAL,
	 RTW_DMA_MAPPING_LOW, RTW_DMA_MAPPING_LOW,
	 RTW_DMA_MAPPING_EXTRA, RTW_DMA_MAPPING_HIGH},
	{RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_NORMAL,
	 RTW_DMA_MAPPING_LOW, RTW_DMA_MAPPING_LOW,
	 RTW_DMA_MAPPING_EXTRA, RTW_DMA_MAPPING_HIGH},
	{RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_NORMAL,
	 RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_HIGH,
	 RTW_DMA_MAPPING_HIGH, RTW_DMA_MAPPING_HIGH},
	{RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_NORMAL,
	 RTW_DMA_MAPPING_LOW, RTW_DMA_MAPPING_LOW,
	 RTW_DMA_MAPPING_HIGH, RTW_DMA_MAPPING_HIGH},
	{RTW_DMA_MAPPING_NORMAL, RTW_DMA_MAPPING_NORMAL,
	 RTW_DMA_MAPPING_LOW, RTW_DMA_MAPPING_LOW,
	 RTW_DMA_MAPPING_EXTRA, RTW_DMA_MAPPING_HIGH},
};

static int rtw8703b_read_efuse(struct rtw_dev *rtwdev, u8 *log_map)
{
	/* include/hal_pg.h lists the eeprom/efuse offsets, the
	 * structure is the same as for 8723d. */
	int ret = rtw8723d_read_efuse(rtwdev, log_map);
	if (ret != 0)
		return ret;

	/* Prefer MAC from DT, if available. On some devices like my
	   Pinephone that might be the only way to get a valid MAC. */
	struct device_node *node = rtwdev->dev->of_node;
	const u8 *addr;
	int len;
	if (node && (addr = of_get_property(node, "local-mac-address", &len)) && len == ETH_ALEN) {
		ether_addr_copy(rtwdev->efuse.addr, addr);
		rtw_dbg(rtwdev, RTW_DBG_PHY, "got wifi mac address from DT: %pM\n", rtwdev->efuse.addr);
	}
	return 0;
}

static void rtw8703b_pwrtrack_init(struct rtw_dev *rtwdev)
{
	struct rtw_dm_info *dm_info = &rtwdev->dm_info;
	u8 path;

	/* TODO: This is the only difference from the 8723d
	 * function. Unify with a parameter when creating support
	 * module. */
	dm_info->default_ofdm_index = RTW_DEF_OFDM_SWING_INDEX_8703B;

	for (path = RF_PATH_A; path < rtwdev->hal.rf_path_num; path++) {
		ewma_thermal_init(&dm_info->avg_thermal[path]);
		dm_info->delta_power_index[path] = 0;
	}
	dm_info->pwr_trk_triggered = false;
	dm_info->pwr_trk_init_trigger = true;
	dm_info->thermal_meter_k = rtwdev->efuse.thermal_meter_k;
	dm_info->txagc_remnant_cck = 0;
	dm_info->txagc_remnant_ofdm = 0;
}

static void rtw8703b_phy_set_param(struct rtw_dev *rtwdev)
{
	/* power on BB/RF domain */
	rtw_write16_set(rtwdev, REG_SYS_FUNC_EN,
			BIT_FEN_EN_25_1 | BIT_FEN_BB_GLB_RST | BIT_FEN_BB_RSTB);
	rtw_write8_set(rtwdev, REG_RF_CTRL,
		       BIT_RF_EN | BIT_RF_RSTB | BIT_RF_SDM_RSTB);
	/*
	 * Translated from vendor driver doing this in PHY_BBConfig8703B:
	 * phy_set_rf_reg(Adapter, RF_PATH_A, 0x1, 0xfffff, 0x780);
	 *
	 * Meaning of 0x0780 is unclear, 8723d uses a different value
	 * by path in its iqk functions (but not during setup).
	 *
	 * Side note, the following comment in
	 * hal/phydm/halrf/halrf_powertracking_ce.h seems related to
	 * the PATH_S* things. The vendor driver equivalents to
	 * PATH_S0 and PATH_S1 seem to be idx_0xc94 and idx_0xc80,
	 * oddly.
	 *
	 * // { {S1: 0xc14, 0xca0} , {S0: 0xc14, 0xca0}}
	 * u32 rx_iqc_8703b[2][2];
	 */
	rtw_write_rf(rtwdev, RF_PATH_A, RF_WLINT, RFREG_MASK, 0x0780);
	rtw_write8(rtwdev, REG_AFE_CTRL1 + 1, 0x80);

	rtw_phy_load_tables(rtwdev);

	/* post init after header files config */
	rtw_write32_clr(rtwdev, REG_RCR, BIT_RCR_ADF);
	/* 0xFF if from vendor driver, 8723d uses
	 * BIT_HIQ_NO_LMT_EN_ROOT.  Comment in vendor driver: "Packet
	 * in Hi Queue Tx immediately". I wonder if setting all bytes
	 * is really necessary. */
	rtw_write8_set(rtwdev, REG_HIQ_NO_LMT_EN, 0xff);
	rtw_write16_set(rtwdev, REG_AFE_CTRL_4, BIT_CK320M_AFE_EN | BIT_EN_SYN);

	u8 xtal_cap = rtwdev->efuse.crystal_cap & 0x3F;
	rtw_write32_mask(rtwdev, REG_AFE_CTRL3, BIT_MASK_XTAL,
			 xtal_cap | (xtal_cap << 6));
	rtw_write32_set(rtwdev, REG_FPGA0_RFMOD, BIT_CCKEN | BIT_OFDMEN);
	/* 8723d conditionally does AFE config here, depending on
	 * EFUSE. 8703b doesn't seem to support it. */

	rtw_write8(rtwdev, REG_SLOT, WLAN_SLOT_TIME);
	rtw_write8_clr(rtwdev, REG_FWHW_TXQ_CTRL, BIT_MASK_TXQ_INIT);
	rtw_write16(rtwdev, REG_RETRY_LIMIT, WLAN_RL_VAL);
	rtw_write32(rtwdev, REG_BAR_MODE_CTRL, WLAN_BAR_VAL);
	rtw_write16(rtwdev, REG_ATIMWND, 0x2);
	// Vendor driver doesn't enable BIT_EN_TXBCN_RPT, but defines
	// the same meaning.
	rtw_write8(rtwdev, REG_BCN_CTRL,
		   BIT_DIS_TSF_UDT | BIT_EN_BCN_FUNCTION | BIT_EN_TXBCN_RPT);
	rtw_write8(rtwdev, REG_TBTT_PROHIBIT, TBTT_PROHIBIT_SETUP_TIME);
	rtw_write8(rtwdev, REG_TBTT_PROHIBIT + 1,
		   TBTT_PROHIBIT_HOLD_TIME_STOP_BCN & 0xFF);
	rtw_write8(rtwdev, REG_TBTT_PROHIBIT + 2,
		   (rtw_read8(rtwdev, REG_TBTT_PROHIBIT + 2) & 0xF0)
		   | (TBTT_PROHIBIT_HOLD_TIME_STOP_BCN >> 8));

	rtw_write8(rtwdev, REG_PIFS, WLAN_PIFS_VAL);
	//rtw_write8(rtwdev, REG_AGGR_BREAK_TIME, WLAN_AGG_BRK_TIME);
	//rtw_write16(rtwdev, REG_NAV_PROT_LEN, WLAN_NAV_PROT_LEN);
	rtw_write16(rtwdev, REG_MAC_SPEC_SIFS, WLAN_SPEC_SIFS);
	rtw_write16(rtwdev, REG_SIFS, WLAN_SPEC_SIFS);
	rtw_write16(rtwdev, REG_SIFS + 2, WLAN_SPEC_SIFS);
	rtw_write8_set(rtwdev, REG_SINGLE_AMPDU_CTRL, BIT_EN_SINGLE_APMDU);
	rtw_write8(rtwdev, REG_RX_PKT_LIMIT, WLAN_RX_PKT_LIMIT);
	rtw_write8(rtwdev, REG_MAX_AGGR_NUM, WLAN_MAX_AGG_NR);
	rtw_write8(rtwdev, REG_AMPDU_MAX_TIME, WLAN_AMPDU_MAX_TIME);

	rtw_phy_init(rtwdev);

	if (rtw_read32_mask(rtwdev, REG_BB_AMP, BIT_MASK_RX_LNA) != 0)
		rtwdev->dm_info.rx_cck_agc_report_type = 1;
	else {
		rtwdev->dm_info.rx_cck_agc_report_type = 0;
		rtw_warn(rtwdev, "unexpected cck agc report type");
	}

	/* We can reuse rtw8723d_lck, equivalent is
	 * _phy_lc_calibrate_8703b in
	 * hal/phydm/halrf/rtl8703b/halrf_8703b.c */
	rtw8723d_lck(rtwdev);

	/* Vendor driver describes setting the register to 0x50 (with
	 * the same mask) as "initial gain" in
	 * _phy_iq_calibrate_8703b(). It then restores a previously
	 * read value. I guess it won't hurt to start lower like
	 * 8723d does. */
	rtw_write32_mask(rtwdev, REG_OFDM0_XAAGC1, MASKBYTE0, 0x50);
	rtw_write32_mask(rtwdev, REG_OFDM0_XAAGC1, MASKBYTE0, 0x20);

	rtw8703b_pwrtrack_init(rtwdev);
}

void rtw8703b_set_channel(struct rtw_dev *rtwdev, u8 channel,
			  u8 bandwidth, u8 primary_chan_idx)
{
	rtw_warn(rtwdev, "%s: not implemented yet\n", __func__);
}

/* Not all indices are valid, based on available data. None of the
 * known valid values are positive, so use 0x7f as "invalid". */
#define LNA_IDX_INVALID 0x7f
static s8 lna_gain_table[16] = {
	-2, LNA_IDX_INVALID, LNA_IDX_INVALID, LNA_IDX_INVALID,
	-6, LNA_IDX_INVALID, LNA_IDX_INVALID, -19,
	-32, LNA_IDX_INVALID, -36, -42,
	LNA_IDX_INVALID, LNA_IDX_INVALID, LNA_IDX_INVALID, -48,
};

static s8 get_cck_rx_pwr(struct rtw_dev *rtwdev, u8 lna_idx, u8 vga_idx)
{
	s8 lna_gain = 0;
	if (lna_idx < ARRAY_SIZE(lna_gain_table))
		lna_gain = lna_gain_table[lna_idx];

	if (lna_gain >= 0) {
		rtw_warn(rtwdev, "incorrect lna index (%d)\n", lna_idx);
		return -120;
	}

	return lna_gain - 2 * vga_idx;
}

static void query_phy_status_cck(struct rtw_dev *rtwdev, u8 *phy_status,
				 struct rtw_rx_pkt_stat *pkt_stat)
{
	u8 vga_idx = GET_PHY_STAT_VGA(phy_status);
	u8 lna_idx;
	if (rtwdev->dm_info.rx_cck_agc_report_type == 1)
		lna_idx = FIELD_PREP(BIT_LNA_H_MASK, GET_PHY_STAT_LNA_H(phy_status))
			| FIELD_PREP(BIT_LNA_L_MASK, GET_PHY_STAT_LNA_L(phy_status));
	else
		lna_idx = FIELD_PREP(BIT_LNA_L_MASK, GET_PHY_STAT_LNA_L(phy_status));
	s8 rx_power = get_cck_rx_pwr(rtwdev, lna_idx, vga_idx);

	pkt_stat->rx_power[RF_PATH_A] = rx_power;
	pkt_stat->rssi = rtw_phy_rf_power_2_rssi(pkt_stat->rx_power, 1);
	rtwdev->dm_info.rssi[RF_PATH_A] = pkt_stat->rssi;
	pkt_stat->signal_power = rx_power;
}

static void query_phy_status_ofdm(struct rtw_dev *rtwdev, u8 *phy_status,
				  struct rtw_rx_pkt_stat *pkt_stat)
{
	struct rtw_dm_info *dm_info = &rtwdev->dm_info;
	s8 val_s8;

	val_s8 = GET_PHY_STAT_AGC_GAIN_A(phy_status) & 0x3F;
	pkt_stat->rx_power[RF_PATH_A] = (val_s8 * 2) - 110;
	pkt_stat->rssi = rtw_phy_rf_power_2_rssi(pkt_stat->rx_power, 1);
	pkt_stat->rx_snr[RF_PATH_A] = GET_PHY_STAT_RXSNR_A(phy_status);

	/* signal power reported by HW */
	val_s8 = GET_PHY_STAT_PWDB(phy_status) >> 1;
	pkt_stat->signal_power = (val_s8  & 0x7f) - 110;

	pkt_stat->rx_evm[RF_PATH_A] = GET_PHY_STAT_RXEVM_A(phy_status);
	pkt_stat->cfo_tail[RF_PATH_A] = GET_PHY_STAT_CFO_TAIL_A(phy_status);

	dm_info->curr_rx_rate = pkt_stat->rate;
	dm_info->rssi[RF_PATH_A] = pkt_stat->rssi;
	dm_info->rx_snr[RF_PATH_A] = pkt_stat->rx_snr[RF_PATH_A] >> 1;
	dm_info->cfo_tail[RF_PATH_A] = (pkt_stat->cfo_tail[RF_PATH_A] * 5) >> 1;

	s8 rx_evm = clamp_t(s8, -pkt_stat->rx_evm[RF_PATH_A] >> 1, 0, 64);
	rx_evm &= 0x3F;	/* 64->0: second path of 1SS rate is 64 */
	dm_info->rx_evm_dbm[RF_PATH_A] = rx_evm;
}

static void query_phy_status(struct rtw_dev *rtwdev, u8 *phy_status,
			     struct rtw_rx_pkt_stat *pkt_stat)
{
	if (pkt_stat->rate <= DESC_RATE11M)
		query_phy_status_cck(rtwdev, phy_status, pkt_stat);
	else
		query_phy_status_ofdm(rtwdev, phy_status, pkt_stat);
}

void rtw8703b_query_rx_desc(struct rtw_dev *rtwdev, u8 *rx_desc,
		      struct rtw_rx_pkt_stat *pkt_stat,
		      struct ieee80211_rx_status *rx_status)
{
	struct ieee80211_hdr *hdr;
	u32 desc_sz = rtwdev->chip->rx_pkt_desc_sz;
	u8 *phy_status = NULL;

	memset(pkt_stat, 0, sizeof(*pkt_stat));

	pkt_stat->phy_status = GET_RX_DESC_PHYST(rx_desc);
	pkt_stat->icv_err = GET_RX_DESC_ICV_ERR(rx_desc);
	pkt_stat->crc_err = GET_RX_DESC_CRC32(rx_desc);
	pkt_stat->decrypted = !GET_RX_DESC_SWDEC(rx_desc) &&
			      GET_RX_DESC_ENC_TYPE(rx_desc) != RX_DESC_ENC_NONE;
	pkt_stat->is_c2h = GET_RX_DESC_C2H(rx_desc);
	pkt_stat->pkt_len = GET_RX_DESC_PKT_LEN(rx_desc);
	pkt_stat->drv_info_sz = GET_RX_DESC_DRV_INFO_SIZE(rx_desc);
	pkt_stat->shift = GET_RX_DESC_SHIFT(rx_desc);
	pkt_stat->rate = GET_RX_DESC_RX_RATE(rx_desc);
	pkt_stat->cam_id = GET_RX_DESC_MACID(rx_desc);
	pkt_stat->ppdu_cnt = 0;
	pkt_stat->tsf_low = GET_RX_DESC_TSFL(rx_desc);

	/* functionally the same as for 8723D (just macro instead of
	 * literal 8) */
	pkt_stat->drv_info_sz *= RX_DRV_INFO_SZ_UNIT_8703B;

	/* c2h cmd pkt's rx/phy status is not interesting */
	if (pkt_stat->is_c2h)
		return;

	/* hdr is also the right value for pkt_stat->hdr, but oddly
	 * only rtw8822c.c sets that. */
	hdr = (struct ieee80211_hdr *)(rx_desc + desc_sz + pkt_stat->shift +
				       pkt_stat->drv_info_sz);

	pkt_stat->bw = GET_RX_DESC_BW(rx_desc);

	if (pkt_stat->phy_status) {
		phy_status = rx_desc + desc_sz + pkt_stat->shift;
		query_phy_status(rtwdev, phy_status, pkt_stat);
	}

	rtw_rx_fill_rx_status(rtwdev, pkt_stat, hdr, rx_status, phy_status);

	/* Old 8723cs driver checked for size < 14 or size > 8192 and
	 * simply dropped the packet. Maybe this should go into
	 * rtw_rx_fill_rx_status()?
	 *
	 * Looking at dmesg the zero length packets appear/disappear
	 * after BT coex change (BT on/idle), so maybe they're
	 * actually BT artifacts and implementing coex would fix the
	 * issue. */
	if (pkt_stat->pkt_len == 0) {
		rx_status->flag |= RX_FLAG_NO_PSDU;
		rtw_warn(rtwdev, "zero length packet");
	}
}

void rtw8703b_false_alarm_statistics(struct rtw_dev *rtwdev)
{
	rtw_warn(rtwdev, "%s: not implemented yet\n", __func__);
}

void rtw8703b_phy_calibration(struct rtw_dev *rtwdev)
{
	rtw_warn(rtwdev, "%s: not implemented yet\n", __func__);
}

void rtw8703b_pwr_track(struct rtw_dev *rtwdev)
{
	/* maybe helpful: hal/phydm/rtl8703b/halhwimg8703b_rf.c */
	rtw_warn(rtwdev, "%s: not implemented yet\n", __func__);
}

void rtw8703b_fill_txdesc_checksum(struct rtw_dev *rtwdev,
			     struct rtw_tx_pkt_info *pkt_info,
			     u8 *txdesc)
{
	rtw_warn(rtwdev, "%s: not implemented yet\n", __func__);
}

void rtw8703b_coex_set_init(struct rtw_dev *rtwdev)
{
	rtw_warn(rtwdev, "%s: not implemented yet\n", __func__);
}

void rtw8703b_coex_set_gnt_fix(struct rtw_dev *rtwdev)
{
	rtw_warn(rtwdev, "%s: not implemented yet\n", __func__);
}

void rtw8703b_coex_set_gnt_debug(struct rtw_dev *rtwdev)
{
	rtw_warn(rtwdev, "%s: not implemented yet\n", __func__);
}

void rtw8703b_coex_set_rfe_type(struct rtw_dev *rtwdev)
{
	rtw_warn(rtwdev, "%s: not implemented yet\n", __func__);
}

void rtw8703b_coex_set_wl_tx_power(struct rtw_dev *rtwdev, u8 wl_pwr)
{
	rtw_warn(rtwdev, "%s: not implemented yet\n", __func__);
}

void rtw8703b_coex_set_wl_rx_gain(struct rtw_dev *rtwdev, bool low_gain)
{
	rtw_warn(rtwdev, "%s: not implemented yet\n", __func__);
}

static const u8 rtw8703b_pwrtrk_2gb_n[] = {
	0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6,
	7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11
};

static const u8 rtw8703b_pwrtrk_2gb_p[] = {
	0, 1, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 7, 7, 7,
	8, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15
};

static const u8 rtw8703b_pwrtrk_2ga_n[] = {
	0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6,
	7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11
};

static const u8 rtw8703b_pwrtrk_2ga_p[] = {
	0, 1, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 7, 7, 7,
	8, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15
};

static const u8 rtw8703b_pwrtrk_2g_cck_b_n[] = {
	0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6,
	7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11
};

static const u8 rtw8703b_pwrtrk_2g_cck_b_p[] = {
	0, 0, 1, 1, 2, 3, 3, 3, 4, 4, 4, 5, 6, 6, 6,
	7, 7, 8, 8, 8, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13
};

static const u8 rtw8703b_pwrtrk_2g_cck_a_n[] = {
	0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6,
	7, 7, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11
};

static const u8 rtw8703b_pwrtrk_2g_cck_a_p[] = {
	0, 0, 1, 1, 2, 3, 3, 3, 4, 4, 4, 5, 6, 6, 6,
	7, 7, 8, 8, 8, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13
};

static const s8 rtw8703b_pwrtrk_xtal_n[] = {
	0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -3,
	-4, -2, -2, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1
};

static const s8 rtw8703b_pwrtrk_xtal_p[] = {
	0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 1, 0, -1, -1, -1,
	-2, -3, -7, -9, -10, -11, -14, -16, -18, -20, -22, -24, -26, -28, -30
};

static const struct rtw_pwr_track_tbl rtw8703b_rtw_pwr_track_tbl = {
	.pwrtrk_2gb_n = rtw8703b_pwrtrk_2gb_n,
	.pwrtrk_2gb_p = rtw8703b_pwrtrk_2gb_p,
	.pwrtrk_2ga_n = rtw8703b_pwrtrk_2ga_n,
	.pwrtrk_2ga_p = rtw8703b_pwrtrk_2ga_p,
	.pwrtrk_2g_cckb_n = rtw8703b_pwrtrk_2g_cck_b_n,
	.pwrtrk_2g_cckb_p = rtw8703b_pwrtrk_2g_cck_b_p,
	.pwrtrk_2g_ccka_n = rtw8703b_pwrtrk_2g_cck_a_n,
	.pwrtrk_2g_ccka_p = rtw8703b_pwrtrk_2g_cck_a_p,
	.pwrtrk_xtal_n = rtw8703b_pwrtrk_xtal_n,
	.pwrtrk_xtal_p = rtw8703b_pwrtrk_xtal_p,
};

static struct rtw_chip_ops rtw8703b_ops = {
	.mac_init		= rtw8723d_mac_init,
	.dump_fw_crash		= NULL,
	.shutdown		= NULL,
	.read_efuse		= rtw8703b_read_efuse,
	.phy_set_param		= rtw8703b_phy_set_param,
	.set_channel		= rtw8703b_set_channel,
	.query_rx_desc		= rtw8703b_query_rx_desc,
	.read_rf		= rtw_phy_read_rf_sipi,
	.write_rf		= rtw_phy_write_rf_reg_sipi,
	// the settings are exactly the same
	.set_tx_power_index	= rtw8723d_set_tx_power_index,
	.set_antenna		= NULL,
	.cfg_ldo25		= rtw8723d_cfg_ldo25,
	.efuse_grant		= rtw8723d_efuse_grant,
	.false_alarm_statistics	= rtw8703b_false_alarm_statistics,
	.phy_calibration	= rtw8703b_phy_calibration,
	.dpk_track		= NULL,
	/* 8723d uses REG_CSRATIO to set dm_info.cck_pd_default, which
	 * is used in its cck_pd_set function. According to comments
	 * in the vendor driver code it doesn't exist in this chip
	 * generation, only 0xa0a ("ODM_CCK_PD_THRESH", which is only
	 * *written* to). */
	.cck_pd_set		= NULL,
	.pwr_track		= rtw8703b_pwr_track,
	.config_bfee		= NULL,
	.set_gid_table		= NULL,
	.cfg_csi_rate		= NULL,
	.adaptivity_init	= NULL,
	.adaptivity		= NULL,
	.cfo_init		= NULL,
	.cfo_track		= NULL,
	.config_tx_path		= NULL,
	.config_txrx_mode	= NULL,
	.fill_txdesc_checksum	= rtw8703b_fill_txdesc_checksum,

	/* for coex */
	.coex_set_init		= rtw8703b_coex_set_init,
	.coex_set_ant_switch	= NULL,
	.coex_set_gnt_fix	= rtw8703b_coex_set_gnt_fix,
	.coex_set_gnt_debug	= rtw8703b_coex_set_gnt_debug,
	.coex_set_rfe_type	= rtw8703b_coex_set_rfe_type,
	.coex_set_wl_tx_power	= rtw8703b_coex_set_wl_tx_power,
	.coex_set_wl_rx_gain	= rtw8703b_coex_set_wl_rx_gain,
};

const struct rtw_chip_info rtw8703b_hw_spec = {
	.ops = &rtw8703b_ops,
	.id = RTW_CHIP_TYPE_8703B,

	.fw_name = "rtw88/rtw8703b_fw.bin",
	.wlan_cpu = RTW_WCPU_11N,
	.tx_pkt_desc_sz = 40,
	.tx_buf_desc_sz = 16,
	.rx_pkt_desc_sz = 24,
	.rx_buf_desc_sz = 8,
	.phy_efuse_size = 256,
	.log_efuse_size = 512,
	.ptct_efuse_size = 15,
	.txff_size = 32768,
	.rxff_size = 16384,
	.rsvd_drv_pg_num = 8,
	.band = RTW_BAND_2G,
	.page_size = TX_PAGE_SIZE,
	.csi_buf_pg_num = 0,
	.dig_min = 0x20,
	.txgi_factor = 1,
	.is_pwr_by_rate_dec = true,
	.rx_ldpc = false,
	.tx_stbc = false,
	.max_power_index = 0x3f,
	.ampdu_density = IEEE80211_HT_MPDU_DENSITY_16,

	.path_div_supported = false,
	/* Maybe switch to false for first tries, if that simplifies
	 * things. */
	.ht_supported = true,
	.vht_supported = false,
	.lps_deep_mode_supported = 0,

	/* This is written to the *second* byte of SYS_FUNC_EN, the
	 * value taken from rtw8723d. Bit definitions in the vendor
	 * driver are in include/hal_com_reg.h, the value seems
	 * plausible. */
	.sys_func_en = 0xFD,
	// Power sequences:
	// include/Hal8703BPwrSeq.h hal/rtl8703b/Hal8703BPwrSeq.c
	.pwr_on_seq = card_enable_flow_8703b,
	.pwr_off_seq = card_disable_flow_8703b,
	.rqpn_table = rqpn_table_8703b,
	// The "available" addresses are exactly the same in the
	// vendor driver, it doesn't seem to have an equivalent for
	// the "reserved" part.
	.prioq_addrs = &prioq_addrs_8723d,
	.page_table = page_table_8703b,
	// used only in pci.c, probably don't need
	.intf_table = NULL,

	.dig = rtw8723d_dig,
	.dig_cck = rtw8723d_dig_cck,

	/* this is just not set for 8723d */
	// .rf_base_addr
	.rf_sipi_addr = {0x840, 0x844},
	.rf_sipi_read_addr = rtw8723d_rf_sipi_addr,
	.ltecoex_addr = &rtw8723d_ltecoex_addr,

	.mac_tbl = &rtw8703b_mac_tbl,
	.agc_tbl = &rtw8703b_agc_tbl,
	.bb_tbl = &rtw8703b_bb_tbl,
	.rf_tbl = {&rtw8703b_rf_a_tbl},

	.rfe_defs = rtw8703b_rfe_defs,
	.rfe_defs_size = ARRAY_SIZE(rtw8703b_rfe_defs),

	.iqk_threshold = 8,
	.pwr_track_tbl = &rtw8703b_rtw_pwr_track_tbl,

	/* leaving this commented because I don't want any part of the
	   stack to assume wowlan is supported yet */
	.wow_fw_name = "rtw88/rtw8703b_wow_fw.bin",
	// .wowlan_stub = NULL,
	.max_scan_ie_len = IEEE80211_MAX_DATA_LEN,

	/* Vendor driver has a time-based format, converted from
	 * 20180330 */
	.coex_para_ver = 0x0133ed6a,
	.bt_desired_ver = 0x1c,
	.scbd_support = true,
	.new_scbd10_def = true,
	.ble_hid_profile_support = false,
	.wl_mimo_ps_support = false,
	.pstdma_type = COEX_PSTDMA_FORCE_LPSOFF,
	.bt_rssi_type = COEX_BTRSSI_RATIO,
	.ant_isolation = 15,
	.rssi_tolerance = 2,
	.bt_rssi_step = bt_rssi_step_8703b,
	.wl_rssi_step = wl_rssi_step_8703b,
	/*
	 * sant -> shared antenna, nsant -> non-shared antenna
	 *
	 * TODO: Used for coex, but at least nothing should crash with
	 * these unset.
	 *
	 * btc_chip_para defined in hal/btc/halbtcoutsrc.h loks very
	 * similar, also regarding rssi_step above, but doesn't seem
	 * to be assigned anywhere.
	 *
	 * halbtc8703b1ant_set_coex_table in
	 * hal/btc/halbtc8703b1ant.c? Generally lots of magic numbers
	 * in that file.
	 */
	.table_sant_num = 0,
	.table_sant = NULL,
	.table_nsant_num = 0,
	.table_nsant = NULL,
	/* halbtc8703b1ant_ps_tdma in hal/btc/halbtc8703b1ant.c seems
	 * to have similar data to tdma_sant_8723d in a massive
	 * switch/case. */
	.tdma_sant_num = 0,
	.tdma_sant = NULL,
	.tdma_nsant_num = 0,
	.tdma_nsant = NULL,
	.wl_rf_para_num = ARRAY_SIZE(rf_para_tx_8703b),
	.wl_rf_para_tx = rf_para_tx_8703b,
	.wl_rf_para_rx = rf_para_rx_8703b,
	.bt_afh_span_bw20 = 0x20,
	.bt_afh_span_bw40 = 0x30,
	.afh_5g_num = ARRAY_SIZE(afh_5g_8703b),
	.afh_5g = afh_5g_8703b,
	/* this is set ONLY in rtw8723d, can't find similar in
	 * rtl8723cs driver */
	// The REG_BTG_SEL doesn't seem to have a counterpart in the
	// vendor driver either. Maybe not supported?
	.btg_reg = NULL,
	/* Whatever this is seems to be only informational? At least
	 * its use in coex.c handles the count of 0. */
	.coex_info_hw_regs_num = 0,
	.coex_info_hw_regs = NULL,
};
EXPORT_SYMBOL(rtw8703b_hw_spec);

MODULE_FIRMWARE("rtw88/rtw8703b_fw.bin");

MODULE_AUTHOR("Fiona Klute <fiona.klute@gmx.de>");
MODULE_DESCRIPTION("Realtek 802.11n wireless 8703b driver");
MODULE_LICENSE("Dual BSD/GPL");

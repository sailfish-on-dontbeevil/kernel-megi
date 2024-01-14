// SPDX-License-Identifier: GPL-2.0
/* Copyright Fiona Klute <fiona.klute@gmx.de> */

#include "main.h"
#include "coex.h"
#include "rtw8703b.h"
/* 8703b and 8723d have a lot of similarities, so I can reuse
 * code. The shared code should be moved to a support module or
 * similar, though. */
#include "rtw8723d.h"

#define TRANS_SEQ_END \
	{ \
		0xFFFF, \
		RTW_PWR_CUT_ALL_MSK, \
		RTW_PWR_INTF_ALL_MSK, \
		0, \
		RTW_PWR_CMD_END, 0, 0}


static const struct coex_5g_afh_map afh_5g_8703b[] = { {0, 0, 0} };

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

static int rtw8703b_mac_init(struct rtw_dev *rtwdev)
{
	return -ENOTSUPP;
}

static struct rtw_chip_ops rtw8703b_ops = {
	.mac_init		= rtw8703b_mac_init, // required
	.dump_fw_crash		= NULL,
	.shutdown		= NULL,
	// include/hal_pg.h has the eeprom/efuse offsets, the
	// structure is the same as for 8723d.
	.read_efuse		= rtw8723d_read_efuse, // required
	.phy_set_param		= NULL, // required
	.set_channel		= NULL, // required
	.query_rx_desc		= NULL, // required
	.read_rf		= NULL, // required
	.write_rf		= NULL, // required
	.set_tx_power_index	= NULL, // required
	.set_antenna		= NULL,
	.cfg_ldo25		= rtw8723d_cfg_ldo25, // required
	.efuse_grant		= NULL,
	.false_alarm_statistics	= NULL, // required
	.phy_calibration	= NULL, // required
	.dpk_track		= NULL,
	.cck_pd_set		= NULL,
	// maybe helpful: hal/phydm/rtl8703b/halhwimg8703b_rf.c
	.pwr_track		= NULL, // required
	.config_bfee		= NULL,
	.set_gid_table		= NULL,
	.cfg_csi_rate		= NULL,
	.adaptivity_init	= NULL,
	.adaptivity		= NULL,
	.cfo_init		= NULL,
	.cfo_track		= NULL,
	.config_tx_path		= NULL,
	.config_txrx_mode	= NULL,
	.fill_txdesc_checksum	= NULL, // required

	/* for coex */
	.coex_set_init		= NULL, // required
	.coex_set_ant_switch	= NULL,
	.coex_set_gnt_fix	= NULL, // required
	.coex_set_gnt_debug	= NULL, // required
	.coex_set_rfe_type	= NULL, // required
	.coex_set_wl_tx_power	= NULL, // required
	.coex_set_wl_rx_gain	= NULL, // required
};

const struct rtw_chip_info rtw8703b_hw_spec = {
	.ops = &rtw8703b_ops,
	.id = RTW_CHIP_TYPE_8703B,

	.fw_name = "rtw88/rtw8703b_fw_nic.bin",
	.wlan_cpu = RTW_WCPU_11N,
	.tx_pkt_desc_sz = 40,
	.tx_buf_desc_sz = 16,
	.rx_pkt_desc_sz = 24,
	.rx_buf_desc_sz = 8,
	.phy_efuse_size = 256,
	.log_efuse_size = 256,
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
	// hal/hal_halmac.c seems to have something similar
	.rqpn_table = NULL,
	.prioq_addrs = NULL,
	// include/rtl8703b_hal.h has NORMAL_PAGE_NUM_*
	.page_table = NULL,
	.intf_table = NULL,

	.dig = rtw8723d_dig,
	.dig_cck = rtw8723d_dig_cck,

	/* this is just not set for 8723d */
	// .rf_base_addr
	.rf_sipi_addr = {0x840, 0x844},
	.rf_sipi_read_addr = rtw8723d_rf_sipi_addr,
	.fix_rf_phy_num = 2,
	.ltecoex_addr = &rtw8723d_ltecoex_addr,

	// hal/phydm/rtl8703b/halhwimg8703b_mac.c
	.mac_tbl = NULL,
	// agc & bb: hal/phydm/rtl8703b/halhwimg8703b_bb.c
	.agc_tbl = NULL,
	.bb_tbl = NULL,
	// hal/phydm/rtl8703b/halhwimg8703b_rf.c
	.rf_tbl = {NULL},

	// required during init, this is where init currently fails
	// power settings, should be in the rf table file, too
	.rfe_defs = NULL,
	.rfe_defs_size = 0,

	.iqk_threshold = 8,
	// also in rf table file
	.pwr_track_tbl = NULL,

	/* leaving this commented because I don't want any part of the
	   stack to assume wowlan is supported yet */
	// .wow_fw_name = "rtw88/rtw8703b_fw_wowlan.bin",
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
	/* TODO: whatever this is */
	.bt_rssi_step = NULL,
	.wl_rssi_step = NULL,
	.table_sant_num = 0,
	.table_sant = NULL,
	.table_nsant_num = 0,
	.table_nsant = NULL,
	.tdma_sant_num = 0,
	.tdma_sant = NULL,
	.tdma_nsant_num = 0,
	.tdma_nsant = NULL,
	.wl_rf_para_num = 0,
	.wl_rf_para_tx = NULL,
	.wl_rf_para_rx = NULL,
	.bt_afh_span_bw20 = 0x20,
	.bt_afh_span_bw40 = 0x30,
	.afh_5g_num = ARRAY_SIZE(afh_5g_8703b),
	.afh_5g = afh_5g_8703b,
	/* this is set ONLY in rtw8723d, can't find similar in
	 * rtl8723cs driver */
	.btg_reg = NULL,
	/* Whatever this is seems to be only informational? */
	.coex_info_hw_regs_num = 0,
	.coex_info_hw_regs = NULL,
};
EXPORT_SYMBOL(rtw8703b_hw_spec);

MODULE_FIRMWARE("rtw88/rtw8703b_fw_nic.bin");

MODULE_AUTHOR("Fiona Klute <fiona.klute@gmx.de>");
MODULE_DESCRIPTION("Realtek 802.11n wireless 8723cs driver");
MODULE_LICENSE("GPL");

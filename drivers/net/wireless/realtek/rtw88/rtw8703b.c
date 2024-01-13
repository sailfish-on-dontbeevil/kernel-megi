// SPDX-License-Identifier: GPL-2.0
/* Copyright Fiona Klute <fiona.klute@gmx.de> */

#include "main.h"
#include "coex.h"
#include "rtw8703b.h"

static struct rtw_chip_ops rtw8703b_ops = {
	.mac_init		= NULL, // required
	.dump_fw_crash		= NULL,
	.shutdown		= NULL,
	.read_efuse		= NULL, // required
	.phy_set_param		= NULL, // required
	.set_channel		= NULL, // required
	.query_rx_desc		= NULL, // required
	.read_rf		= NULL, // required
	.write_rf		= NULL, // required
	.set_tx_power_index	= NULL, // required
	.set_antenna		= NULL,
	.cfg_ldo25		= NULL, // required
	.efuse_grant		= NULL,
	.false_alarm_statistics	= NULL, // required
	.phy_calibration	= NULL, // required
	.dpk_track		= NULL,
	.cck_pd_set		= NULL,
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
	/* for USB/SDIO only */
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
	/* TODO: sizes below are copied from rtw8723d */
	.tx_pkt_desc_sz = 40,
	.tx_buf_desc_sz = 16,
	.rx_pkt_desc_sz = 24,
	.rx_buf_desc_sz = 8,
	.phy_efuse_size = 512,
	.log_efuse_size = 512,
	.ptct_efuse_size = 96 + 1,
	.txff_size = 32768,
	.rxff_size = 16384,
	/* TODO: this number is also copied */
	.rsvd_drv_pg_num = 8,
	.band = RTW_BAND_2G,
	.page_size = TX_PAGE_SIZE,
	.csi_buf_pg_num = 0,
	/* TODO: this number is also copied */
	.dig_min = 0x20,
	/* TODO: the next four are the same for all except rtw8822c,
	   but not sure what they mean */
	.txgi_factor = 1,
	.is_pwr_by_rate_dec = true,
	.rx_ldpc = false,
	.tx_stbc = false,
	/* TODO: this number is also copied */
	.max_power_index = 0x3f,
	/* TODO: this number is also copied */
	.ampdu_density = IEEE80211_HT_MPDU_DENSITY_16,

	.path_div_supported = false,
	/* Maybe switch to false for first tries, if that simplifies
	 * things. */
	.ht_supported = true,
	.vht_supported = false,
	.lps_deep_mode_supported = 0,

	/* TODO: this number is also copied */
	.sys_func_en = 0xFD,
	/* TODO: required data */
	.pwr_on_seq = NULL,
	.pwr_off_seq = NULL,
	.rqpn_table = NULL,
	.prioq_addrs = NULL,
	.page_table = NULL,
	.intf_table = NULL,

	/* TODO: dig is required, dig_cck not */
	.dig = NULL,
	.dig_cck = NULL,

	/* this is just not set for 8723d */
	// .rf_base_addr
	/* TODO: this number is also copied */
	.rf_sipi_addr = {0x840, 0x844},
	/* TODO: the next two are set ONLY for 8723d, maybe needed
	 * here, too? */
	.rf_sipi_read_addr = NULL,
	.fix_rf_phy_num = 2,
	/* TODO: required, apparently */
	.ltecoex_addr = NULL,

	.mac_tbl = NULL,
	.agc_tbl = NULL,
	.bb_tbl = NULL,
	.rf_tbl = {NULL},

	.rfe_defs = NULL,
	.rfe_defs_size = 0,

	/* seems to be the same for all chips, hope it works */
	.iqk_threshold = 8,
	.pwr_track_tbl = NULL,

	/* leaving this commented because I don't want any part of the
	   stack to assume wowlan is supported yet */
	// .wow_fw_name = "rtw88/rtw8703b_fw_wowlan.bin",
	// .wowlan_stub = NULL,
	.max_scan_ie_len = IEEE80211_MAX_DATA_LEN,

	/* TODO: these numbers are also copied */
	.coex_para_ver = 0x2007022f,
	.bt_desired_ver = 0x2f,
	/* TODO: check and enable what's possible */
	.scbd_support = false,
	.new_scbd10_def = false,
	.ble_hid_profile_support = false,
	.wl_mimo_ps_support = false,
	/* TODO: these numbers are also copied, same for all except
	 * rtw8822c */
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
	/* TODO: these numbers are also copied from rtw8723d */
	.bt_afh_span_bw20 = 0x20,
	.bt_afh_span_bw40 = 0x30,
	/* TODO: whatever this is */
	.afh_5g_num = 0,
	.afh_5g = NULL,
	/* TODO: this is set ONLY in rtw8723d */
	.btg_reg = NULL,
	/* TODO: whatever this is */
	.coex_info_hw_regs_num = 0,
	.coex_info_hw_regs = NULL,
};
EXPORT_SYMBOL(rtw8703b_hw_spec);

MODULE_FIRMWARE("rtw88/rtw8703b_fw_nic.bin");

MODULE_AUTHOR("Fiona Klute <fiona.klute@gmx.de>");
MODULE_DESCRIPTION("Realtek 802.11n wireless 8723cs driver");
MODULE_LICENSE("GPL");

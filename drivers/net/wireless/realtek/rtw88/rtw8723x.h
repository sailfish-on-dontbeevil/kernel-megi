// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright 2024 Fiona Klute
 *
 * Based on code originally in rtw8723d.[ch],
 * Copyright(c) 2018-2019  Realtek Corporation
 */

#ifndef __RTW8723X_H__
#define __RTW8723X_H__

#include "main.h"

enum rtw8723x_path {
	PATH_S1,
	PATH_S0,
	PATH_NR,
};

enum rtw8723x_iqk_round {
	IQK_ROUND_0,
	IQK_ROUND_1,
	IQK_ROUND_2,
	IQK_ROUND_HYBRID,
	IQK_ROUND_SIZE,
	IQK_ROUND_INVALID = 0xff,
};

enum rtw8723x_iqk_result {
	IQK_S1_TX_X,
	IQK_S1_TX_Y,
	IQK_S1_RX_X,
	IQK_S1_RX_Y,
	IQK_S0_TX_X,
	IQK_S0_TX_Y,
	IQK_S0_RX_X,
	IQK_S0_RX_Y,
	IQK_NR,
	IQK_SX_NR = IQK_NR / PATH_NR,
};

struct rtw8723xe_efuse {
	u8 mac_addr[ETH_ALEN];		/* 0xd0 */
	u8 vender_id[2];
	u8 device_id[2];
	u8 sub_vender_id[2];
	u8 sub_device_id[2];
};

struct rtw8723xu_efuse {
	u8 res4[48];                    /* 0xd0 */
	u8 vender_id[2];                /* 0x100 */
	u8 product_id[2];               /* 0x102 */
	u8 usb_option;                  /* 0x104 */
	u8 res5[2];			/* 0x105 */
	u8 mac_addr[ETH_ALEN];          /* 0x107 */
};

struct rtw8723xs_efuse {
	u8 res4[0x4a];			/* 0xd0 */
	u8 mac_addr[ETH_ALEN];		/* 0x11a */
};

struct rtw8723x_efuse {
	__le16 rtl_id;
	u8 rsvd[2];
	u8 afe;
	u8 rsvd1[11];

	/* power index for four RF paths */
	struct rtw_txpwr_idx txpwr_idx_table[4];

	u8 channel_plan;		/* 0xb8 */
	u8 xtal_k;
	u8 thermal_meter;
	u8 iqk_lck;
	u8 pa_type;			/* 0xbc */
	u8 lna_type_2g[2];		/* 0xbd */
	u8 lna_type_5g[2];
	u8 rf_board_option;
	u8 rf_feature_option;
	u8 rf_bt_setting;
	u8 eeprom_version;
	u8 eeprom_customer_id;
	u8 tx_bb_swing_setting_2g;
	u8 res_c7;
	u8 tx_pwr_calibrate_rate;
	u8 rf_antenna_option;		/* 0xc9 */
	u8 rfe_option;
	u8 country_code[2];
	u8 res[3];
	union {
		struct rtw8723xe_efuse e;
		struct rtw8723xu_efuse u;
		struct rtw8723xs_efuse s;
	};
};

#define RTW8723X_IQK_ADDA_REG_NUM	16
#define RTW8723X_IQK_MAC8_REG_NUM	3
#define RTW8723X_IQK_MAC32_REG_NUM	1
#define RTW8723X_IQK_BB_REG_NUM		9

extern const u32 rtw8723x_iqk_adda_regs[RTW8723X_IQK_ADDA_REG_NUM];
extern const u32 rtw8723x_iqk_mac8_regs[RTW8723X_IQK_MAC8_REG_NUM];
extern const u32 rtw8723x_iqk_mac32_regs[RTW8723X_IQK_MAC32_REG_NUM];
extern const u32 rtw8723x_iqk_bb_regs[RTW8723X_IQK_BB_REG_NUM];

struct rtw8723x_iqk_backup_regs {
	u32 adda[RTW8723X_IQK_ADDA_REG_NUM];
	u8 mac8[RTW8723X_IQK_MAC8_REG_NUM];
	u32 mac32[RTW8723X_IQK_MAC32_REG_NUM];
	u32 bb[RTW8723X_IQK_BB_REG_NUM];

	u32 lte_path;
	u32 lte_gnt;

	u32 bb_sel_btg;
	u8 btg_sel;

	u8 igia;
	u8 igib;
};

extern const struct rtw_ltecoex_addr rtw8723x_ltecoex_addr;
extern const struct rtw_rf_sipi_addr rtw8723x_rf_sipi_addr[];
extern const struct rtw_hw_reg rtw8723x_dig[];
extern const struct rtw_hw_reg rtw8723x_dig_cck[];
extern const struct rtw_prioq_addrs prioq_addrs_8723x;

#define WLAN_TXQ_RPT_EN		0x1F

#define SPUR_THRES		0x16
#define DIS_3WIRE		0xccf000c0
#define EN_3WIRE		0xccc000c0
#define START_PSD		0x400000
#define FREQ_CH13		0xfccd
#define FREQ_CH14		0xff9a
#define RFCFGCH_CHANNEL_MASK	GENMASK(7, 0)
#define RFCFGCH_BW_MASK		(BIT(11) | BIT(10))
#define RFCFGCH_BW_20M		(BIT(11) | BIT(10))
#define RFCFGCH_BW_40M		BIT(10)
#define BIT_MASK_RFMOD		BIT(0)
#define BIT_LCK			BIT(15)

#define REG_GPIO_INTM		0x0048
#define REG_BTG_SEL		0x0067
#define BIT_MASK_BTG_WL		BIT(7)
#define REG_LTECOEX_PATH_CONTROL	0x0070
#define REG_LTECOEX_CTRL	0x07c0
#define REG_LTECOEX_WRITE_DATA	0x07c4
#define REG_LTECOEX_READ_DATA	0x07c8
#define REG_PSDFN		0x0808
#define REG_BB_PWR_SAV1_11N	0x0874
#define REG_ANA_PARAM1		0x0880
#define REG_ANALOG_P4		0x088c
#define REG_PSDRPT		0x08b4
#define REG_FPGA1_RFMOD		0x0900
#define REG_BB_SEL_BTG		0x0948
#define REG_BBRX_DFIR		0x0954
#define BIT_MASK_RXBB_DFIR	GENMASK(27, 24)
#define BIT_RXBB_DFIR_EN	BIT(19)
#define REG_CCK0_SYS		0x0a00
#define BIT_CCK_SIDE_BAND	BIT(4)
#define REG_CCK_ANT_SEL_11N	0x0a04
#define REG_PWRTH		0x0a08
#define REG_CCK_FA_RST_11N	0x0a2c
#define BIT_MASK_CCK_CNT_KEEP	BIT(12)
#define BIT_MASK_CCK_CNT_EN	BIT(13)
#define BIT_MASK_CCK_CNT_KPEN	(BIT_MASK_CCK_CNT_KEEP | BIT_MASK_CCK_CNT_EN)
#define BIT_MASK_CCK_FA_KEEP	BIT(14)
#define BIT_MASK_CCK_FA_EN	BIT(15)
#define BIT_MASK_CCK_FA_KPEN	(BIT_MASK_CCK_FA_KEEP | BIT_MASK_CCK_FA_EN)
#define REG_CCK_FA_LSB_11N	0x0a5c
#define REG_CCK_FA_MSB_11N	0x0a58
#define REG_CCK_CCA_CNT_11N	0x0a60
#define BIT_MASK_CCK_FA_MSB	GENMASK(7, 0)
#define BIT_MASK_CCK_FA_LSB	GENMASK(15, 8)
#define REG_PWRTH2		0x0aa8
#define REG_CSRATIO		0x0aaa
#define REG_OFDM_FA_HOLDC_11N	0x0c00
#define BIT_MASK_OFDM_FA_KEEP	BIT(31)
#define REG_BB_RX_PATH_11N	0x0c04
#define REG_TRMUX_11N		0x0c08
#define REG_OFDM_FA_RSTC_11N	0x0c0c
#define BIT_MASK_OFDM_FA_RST	BIT(31)
#define REG_A_RXIQI		0x0c14
#define BIT_MASK_RXIQ_S1_X	0x000003FF
#define BIT_MASK_RXIQ_S1_Y1	0x0000FC00
#define BIT_SET_RXIQ_S1_Y1(y)	((y) & 0x3F)
#define REG_OFDM0_RXDSP		0x0c40
#define BIT_MASK_RXDSP		GENMASK(28, 24)
#define BIT_EN_RXDSP		BIT(9)
#define REG_OFDM_0_ECCA_THRESHOLD	0x0c4c
#define BIT_MASK_OFDM0_EXT_A	BIT(31)
#define BIT_MASK_OFDM0_EXT_C	BIT(29)
#define BIT_MASK_OFDM0_EXTS	(BIT(31) | BIT(29) | BIT(28))
#define BIT_SET_OFDM0_EXTS(a, c, d) (((a) << 31) | ((c) << 29) | ((d) << 28))
#define REG_OFDM0_XAAGC1	0x0c50
#define REG_OFDM0_XBAGC1	0x0c58
#define REG_AGCRSSI		0x0c78
#define REG_OFDM_0_XA_TX_IQ_IMBALANCE	0x0c80
#define BIT_MASK_TXIQ_ELM_A	0x03ff
#define BIT_SET_TXIQ_ELM_ACD(a, c, d) (((d) << 22) | (((c) & 0x3F) << 16) |    \
				       ((a) & 0x03ff))
#define BIT_MASK_TXIQ_ELM_C	GENMASK(21, 16)
#define BIT_SET_TXIQ_ELM_C2(c)	((c) & 0x3F)
#define BIT_MASK_TXIQ_ELM_D	GENMASK(31, 22)
#define REG_TXIQK_MATRIXA_LSB2_11N	0x0c94
#define BIT_SET_TXIQ_ELM_C1(c)	(((c) & 0x000003C0) >> 6)
#define REG_RXIQK_MATRIX_LSB_11N	0x0ca0
#define BIT_MASK_RXIQ_S1_Y2	0xF0000000
#define BIT_SET_RXIQ_S1_Y2(y)	(((y) >> 6) & 0xF)
#define REG_TXIQ_AB_S0		0x0cd0
#define BIT_MASK_TXIQ_A_S0	0x000007FE
#define BIT_MASK_TXIQ_A_EXT_S0	BIT(0)
#define BIT_MASK_TXIQ_B_S0	0x0007E000
#define REG_TXIQ_CD_S0		0x0cd4
#define BIT_MASK_TXIQ_C_S0	0x000007FE
#define BIT_MASK_TXIQ_C_EXT_S0	BIT(0)
#define BIT_MASK_TXIQ_D_S0	GENMASK(22, 13)
#define BIT_MASK_TXIQ_D_EXT_S0	BIT(12)
#define REG_RXIQ_AB_S0		0x0cd8
#define BIT_MASK_RXIQ_X_S0	0x000003FF
#define BIT_MASK_RXIQ_Y_S0	0x003FF000
#define REG_OFDM_FA_TYPE1_11N	0x0cf0
#define BIT_MASK_OFDM_FF_CNT	GENMASK(15, 0)
#define BIT_MASK_OFDM_SF_CNT	GENMASK(31, 16)
#define REG_OFDM_FA_RSTD_11N	0x0d00
#define BIT_MASK_OFDM_FA_RST1	BIT(27)
#define BIT_MASK_OFDM_FA_KEEP1	BIT(31)
#define REG_CTX			0x0d03
#define BIT_MASK_CTX_TYPE	GENMASK(6, 4)
#define REG_OFDM1_CFOTRK	0x0d2c
#define BIT_EN_CFOTRK		BIT(28)
#define REG_OFDM1_CSI1		0x0d40
#define REG_OFDM1_CSI2		0x0d44
#define REG_OFDM1_CSI3		0x0d48
#define REG_OFDM1_CSI4		0x0d4c
#define REG_OFDM_FA_TYPE2_11N	0x0da0
#define BIT_MASK_OFDM_CCA_CNT	GENMASK(15, 0)
#define BIT_MASK_OFDM_PF_CNT	GENMASK(31, 16)
#define REG_OFDM_FA_TYPE3_11N	0x0da4
#define BIT_MASK_OFDM_RI_CNT	GENMASK(15, 0)
#define BIT_MASK_OFDM_CRC_CNT	GENMASK(31, 16)
#define REG_OFDM_FA_TYPE4_11N	0x0da8
#define BIT_MASK_OFDM_MNS_CNT	GENMASK(15, 0)
#define REG_FPGA0_IQK_11N	0x0e28
#define BIT_MASK_IQK_MOD	0xffffff00
#define EN_IQK			0x808000
#define RST_IQK			0x000000
#define REG_TXIQK_TONE_A_11N	0x0e30
#define REG_RXIQK_TONE_A_11N	0x0e34
#define REG_TXIQK_PI_A_11N	0x0e38
#define REG_RXIQK_PI_A_11N	0x0e3c
#define REG_TXIQK_11N		0x0e40
#define BIT_SET_TXIQK_11N(x, y)	(0x80007C00 | ((x) << 16) | (y))
#define REG_RXIQK_11N		0x0e44
#define REG_IQK_AGC_PTS_11N	0x0e48
#define REG_IQK_AGC_RSP_11N	0x0e4c
#define REG_TX_IQK_TONE_B	0x0e50
#define REG_RX_IQK_TONE_B	0x0e54
#define REG_IQK_RES_TX		0x0e94
#define BIT_MASK_RES_TX		GENMASK(25, 16)
#define REG_IQK_RES_TY		0x0e9c
#define BIT_MASK_RES_TY		GENMASK(25, 16)
#define REG_IQK_RES_RX		0x0ea4
#define BIT_MASK_RES_RX		GENMASK(25, 16)
#define REG_IQK_RES_RY		0x0eac
#define BIT_IQK_TX_FAIL		BIT(28)
#define BIT_IQK_RX_FAIL		BIT(27)
#define BIT_IQK_DONE		BIT(26)
#define BIT_MASK_RES_RY		GENMASK(25, 16)
#define REG_PAGE_F_RST_11N		0x0f14
#define BIT_MASK_F_RST_ALL		BIT(16)
#define REG_IGI_C_11N			0x0f84
#define REG_IGI_D_11N			0x0f88
#define REG_HT_CRC32_CNT_11N		0x0f90
#define BIT_MASK_HT_CRC_OK		GENMASK(15, 0)
#define BIT_MASK_HT_CRC_ERR		GENMASK(31, 16)
#define REG_OFDM_CRC32_CNT_11N		0x0f94
#define BIT_MASK_OFDM_LCRC_OK		GENMASK(15, 0)
#define BIT_MASK_OFDM_LCRC_ERR		GENMASK(31, 16)
#define REG_HT_CRC32_CNT_11N_AGG	0x0fb8

void rtw8723x_lck(struct rtw_dev *rtwdev);
int rtw8723x_read_efuse(struct rtw_dev *rtwdev, u8 *log_map);
int rtw8723x_mac_init(struct rtw_dev *rtwdev);
void rtw8723x_cfg_ldo25(struct rtw_dev *rtwdev, bool enable);
void rtw8723x_set_tx_power_index(struct rtw_dev *rtwdev);
void rtw8723x_efuse_grant(struct rtw_dev *rtwdev, bool on);
void rtw8723x_iqk_backup_regs(struct rtw_dev *rtwdev,
			      struct rtw8723x_iqk_backup_regs *backup);
void rtw8723x_iqk_restore_regs(struct rtw_dev *rtwdev,
			       const struct rtw8723x_iqk_backup_regs *backup);
void rtw8723x_iqk_backup_path_ctrl(struct rtw_dev *rtwdev,
				   struct rtw8723x_iqk_backup_regs *backup);
void rtw8723x_iqk_config_path_ctrl(struct rtw_dev *rtwdev);
void rtw8723x_iqk_restore_path_ctrl(struct rtw_dev *rtwdev,
				    const struct rtw8723x_iqk_backup_regs *backup);
void rtw8723x_iqk_backup_lte_path_gnt(struct rtw_dev *rtwdev,
				      struct rtw8723x_iqk_backup_regs *backup);
void rtw8723x_iqk_config_lte_path_gnt(struct rtw_dev *rtwdev);
void rtw8723x_iqk_restore_lte_path_gnt(struct rtw_dev *rtwdev,
				       const struct rtw8723x_iqk_backup_regs *bak);
u8 rtw8723x_pwrtrack_get_limit_ofdm(struct rtw_dev *rtwdev);
void rtw8723x_pwrtrack_set_xtal(struct rtw_dev *rtwdev, u8 therm_path,
				u8 delta);
void rtw8723x_coex_cfg_init(struct rtw_dev *rtwdev);
void rtw8723x_fill_txdesc_checksum(struct rtw_dev *rtwdev,
				   struct rtw_tx_pkt_info *pkt_info,
				   u8 *txdesc);

#endif /* __RTW8723X_H__ */

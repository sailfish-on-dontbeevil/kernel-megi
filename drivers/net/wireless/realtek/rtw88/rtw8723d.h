/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/* Copyright(c) 2018-2019  Realtek Corporation
 */

#ifndef __RTW8723D_H__
#define __RTW8723D_H__

#include "rtw8723x.h"

extern const struct rtw_chip_info rtw8723d_hw_spec;

/* phy status page0 */
#define GET_PHY_STAT_P0_PWDB(phy_stat)                                         \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x00), GENMASK(15, 8))

/* phy status page1 */
#define GET_PHY_STAT_P1_PWDB_A(phy_stat)                                       \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x00), GENMASK(15, 8))
#define GET_PHY_STAT_P1_PWDB_B(phy_stat)                                       \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x00), GENMASK(23, 16))
#define GET_PHY_STAT_P1_RF_MODE(phy_stat)                                      \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x03), GENMASK(29, 28))
#define GET_PHY_STAT_P1_L_RXSC(phy_stat)                                       \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x01), GENMASK(11, 8))
#define GET_PHY_STAT_P1_HT_RXSC(phy_stat)                                      \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x01), GENMASK(15, 12))
#define GET_PHY_STAT_P1_RXEVM_A(phy_stat)                                      \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x04), GENMASK(7, 0))
#define GET_PHY_STAT_P1_CFO_TAIL_A(phy_stat)                                   \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x05), GENMASK(7, 0))
#define GET_PHY_STAT_P1_RXSNR_A(phy_stat)                                      \
	le32_get_bits(*((__le32 *)(phy_stat) + 0x06), GENMASK(7, 0))

static inline s32 iqkxy_to_s32(s32 val)
{
	/* val is Q10.8 */
	return sign_extend32(val, 9);
}

static inline s32 iqk_mult(s32 x, s32 y, s32 *ext)
{
	/* x, y and return value are Q10.8 */
	s32 t;

	t = x * y;
	if (ext)
		*ext = (t >> 7) & 0x1;	/* Q.16 --> Q.9; get LSB of Q.9 */

	return (t >> 8);	/* Q.16 --> Q.8 */
}

#define OFDM_SWING_A(swing)		FIELD_GET(GENMASK(9, 0), swing)
#define OFDM_SWING_B(swing)		FIELD_GET(GENMASK(15, 10), swing)
#define OFDM_SWING_C(swing)		FIELD_GET(GENMASK(21, 16), swing)
#define OFDM_SWING_D(swing)		FIELD_GET(GENMASK(31, 22), swing)
#define RTW_DEF_OFDM_SWING_INDEX	28
#define RTW_DEF_CCK_SWING_INDEX		28

#define MAX_TOLERANCE	5
#define IQK_TX_X_ERR	0x142
#define IQK_TX_Y_ERR	0x42
#define IQK_RX_X_UPPER	0x11a
#define IQK_RX_X_LOWER	0xe6
#define IQK_RX_Y_LMT	0x1a
#define IQK_TX_OK	BIT(0)
#define IQK_RX_OK	BIT(1)
#define PATH_IQK_RETRY	2

#define CCK_DFIR_NR		3

#endif

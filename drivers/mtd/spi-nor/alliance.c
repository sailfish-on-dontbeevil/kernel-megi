// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023, Ondrej Jirman <megi@xff.cz>
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

static const struct flash_info alliance_nor_parts[] = {
	{ "as25f1128mq", INFO(0x524218, 0, 64 * 1024, 256)
		FLAGS(SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
		NO_SFDP_FLAGS(SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
};

const struct spi_nor_manufacturer spi_nor_alliance = {
	.name = "alliance",
	.parts = alliance_nor_parts,
	.nparts = ARRAY_SIZE(alliance_nor_parts),
};

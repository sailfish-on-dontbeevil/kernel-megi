/*
 * Copyright (c) 2022, Bestechnic
 * Copyright (c) 2023, Ondrej Jirman <megi@xff.cz>
 */

#include <linux/types.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/crc32.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/fs.h>

#include "cw1200.h"
#include "hwio.h"
#include "fwio.h"

/* DPLL initial values */
#define DPLL_INIT_VAL_9000		(0x00000191)
#define DPLL_INIT_VAL_BES2600		(0x0EC4F121)

/* Hardware Type Definitions */
#define HIF_8601_VERSATILE		(0)
#define HIF_8601_SILICON		(1)
#define HIF_9000_SILICON_VERSTAILE	(2)

#define CW1250_CUT_11_ID_STR1		(0x302e3033)
#define CW1250_CUT_11_ID_STR2		(0x33302e32)
#define CW1250_CUT_11_ID_STR3		(0x3535)

#define SDIO_DEVICE_SEND_INT_LEN_SEPARATE

#define BES_TX_CTRL_REG_ID	(0x0)

#ifdef SDIO_DEVICE_SEND_INT_LEN_SEPARATE
#define BES_TX_NEXT_LEN_REG_ID	(0x104)
#else
#define BES_TX_NEXT_LEN_REG_ID	BES_TX_CTRL_REG_ID
#endif

#define BES_TX_NEXT_LEN_MASK	(0xffff)
#define BES_TX_DATA_ADDR	(0x0)

#define BES_HOST_INT_REG_ID		(0x120)
#define BES_HOST_INT			(1 << 0)
#define BES_AP_WAKEUP_CFG		(1 << 1)
#define BES_SUBSYSTEM_MCU_DEACTIVE	(1 << 2)
#define BES_SUBSYSTEM_MCU_ACTIVE	(1 << 3)
#define BES_SUBSYSTEM_WIFI_DEACTIVE	(1 << 4)
#define BES_SUBSYSTEM_WIFI_ACTIVE	(1 << 5)
#define BES_SUBSYSTEM_WIFI_DEBUG	(1 << 6)
#define BES_SUBSYSTEM_BT_DEACTIVE	(1 << 7)
#define BES_SUBSYSTEM_BT_ACTIVE		(1 << 8)
#define BES_SUBSYSTEM_SYSTEM_CLOSE	(1 << 9)
#define BES_SUBSYSTEM_BT_WAKEUP		(1 << 10)
#define BES_SUBSYSTEM_BT_SLEEP		(1 << 11)

#define BES_AP_WAKEUP_TYPE_MASK		0xC
#define BES_AP_WAKEUP_TYPE_SHIFT	2
#define BES_AP_WAKEUP_TYPE_GPIO		0
#define BES_AP_WAKEUP_TYPE_IF		1

#define BES_AP_WAKEUP_REG_ID		(0x124)
#define BES_AP_WAKEUP_CFG_VALID		(0x80)

#define BES_AP_WAKEUP_GPIO_MASK		(0x3)
#define BES_AP_WAKEUP_GPIO_HIGH		(0x0)
#define BES_AP_WAKEUP_GPIO_LOW		(0x1)
#define BES_AP_WAKEUP_GPIO_RISE		(0x2)
#define BES_AP_WAKEUP_GPIO_FALL		(0x3)

#define BES_SLAVE_STATUS_REG_ID			(0x10c)
#define BES_SLAVE_STATUS_MCU_READY		(1 << 0)
#define BES_SLAVE_STATUS_DPD_READY		(1 << 1)
#define BES_SLAVE_STATUS_WIFI_READY		(1 << 2)
#define BES_SLAVE_STATUS_BT_READY		(1 << 3)
#define BES_SLAVE_STATUS_MCU_WAKEUP_READY	(1 << 4)
#define BES_SLAVE_STATUS_BT_WAKE_READY		(1 << 5)
#define BES_SLAVE_STATUS_DPD_LOG_READY		(1 << 6)

#define PACKET_TOTAL_LEN(len)		((len) & 0xffff)
#define PACKET_COUNT(len)		(((len) >> 16) & 0xff)
#define PAKCET_CRC8(len)		(((len) >> 24) & 0xff)

#define BES_SDIO_RX_MULTIPLE_NUM (16)
#define BES_SDIO_TX_MULTIPLE_NUM (16)
#define BES_SDIO_TX_MULTIPLE_NUM_NOSIGNAL (1)

#define MAX_SDIO_TRANSFER_LEN (32768)

// dpd

#define DPD_VERSION_OFFSET      0x3AF4
#define DPD_BIN_SIZE            0x3B14
#define DPD_BIN_FILE_SIZE       0x4000
#define DPD_CUR_VERSION         7

// firmware defs

#define BUF_SIZE        49152
#define RETRY_CNT_MAX   3
#define TIMEOUT_TIME    20
#define FRAME_HEADER_SIZE           0x04
#define CODE_DATA_USELESS_SIZE      0x04

#define FRAME_HEADER_REPLY          0xB0
#define FRAME_HEADER_DOWNLOAD_INFO  0xB1
#define FRAME_HEADER_DOWNLOAD_DATA  0xB2
#define FRAME_HEADER_DOWNLOAD_END   0xB3
#define FRAME_HEADER_RUN_CODE       0xB4

/****frame length get****/
#define BES_FW_MSG_TOTAL_LEN(msg)  (sizeof(struct fw_msg_hdr_t) + ((struct fw_msg_hdr_t )(msg)).len)

#define BES2600_DPD_ADDR	0x2008C000
#define BES2600_FACTORY_ADDR	0x2008B000

enum ERR_CODE {
	ERR_NONE = 0x00,
	ERR_LEN = 0x01,
};

struct frame_struct_t {
	u8 type;
	u8 frame_num;
	u16 len;
	u32 payload;
};

struct fw_msg_hdr_t {
	u8 type;
	u8 seq;
	u16 len;
};

struct fw_info_t {
	u32 len;
	u32 addr;
};

struct download_fw_t {
	u32 addr;
	u8 data[0];
};

struct fw_crc_t {
	u32 crc32;
};

struct run_fw_t {
	u32 addr;
};

struct exec_struct_t {
	u32 entry;
	u32 param;
	u32 sp;
	u32 exec_addr;
};

static int bes_slave_rx_ready(struct cw1200_common *priv, u8* buf_cnt,
					u16* buf_len, int timeout)
{
	int ret;
	unsigned long start = jiffies;

	u8* buf_cnt_tmp = kmalloc(sizeof(*buf_cnt_tmp), GFP_KERNEL);
	if (!buf_cnt_tmp)
		return -ENOMEM;

	do {
		ret = cw1200_reg_read(priv, 0x108, buf_cnt_tmp, 1);
		if (!(ret || *buf_cnt_tmp)) {
			mdelay(50);
			continue;
		} else if (ret) {
			pr_err("%s err=%d\n", __func__, ret);
		} else {
			ret = cw1200_reg_read_16(priv, 0x109, buf_len);
		}
		break;
	} while(time_before(jiffies, start + timeout));

	*buf_cnt = *buf_cnt_tmp;
	kfree(buf_cnt_tmp);

	return ret;
}

static int bes_slave_tx_ready(struct cw1200_common *priv, u16 *tx_len, int timeout)
{
	int ret, retry = 0;

	pr_debug("%s now=%lu\n", __func__, jiffies);

	msleep(2);

	ret = wait_for_completion_interruptible_timeout(&priv->fw_completion, timeout);
	if (ret > 0) {
		do {
			ret = cw1200_reg_read_16(priv, 0, tx_len);
			if (!ret && (*tx_len))
				break;
			else
				pr_err("%s,%d ret=%d tx_len=%x retry=%d\n",
						__func__, __LINE__, ret, *tx_len, retry);
			retry++;
		} while(retry <= 5);
		reinit_completion(&priv->fw_completion);
	} else if(!ret) {
		pr_err("%s now=%lu delta=%d\n", __func__, jiffies, timeout);
		ret = -110;
	} else {
		// ret = -ERESTARTSYS, to be continued;
	}

	return ret;
}

/*
static int bes_host_slave_sync(struct cw1200_common *priv)
{
	u8 val;
	int ret;

	ret = cw1200_reg_read(priv, BES_HOST_INT_REG_ID, &val, 1);
	if (ret) {
		pr_err("%s,%d err=%d\n", __func__, __LINE__, ret);
		return ret;
	}

	val |= BES_HOST_INT;
	ret = cw1200_reg_write(priv, BES_HOST_INT_REG_ID, &val, 1);
	if (ret) {
		pr_err("%s,%d err=%d\n", __func__, __LINE__, ret);
	}
	return ret;
}
*/

static int bes_firmware_download_write_reg(struct cw1200_common *priv, u32 addr, u32 val)
{
	u8 frame_num = 0;
	u8 buf_cnt = 0;
	u16 tx_size = 0;
	u16 rx_size = 0;
	u32 length = 0;
	u8 *short_buf;
	int ret;

	struct fw_msg_hdr_t header;
	struct fw_info_t fw_info;
	struct download_fw_t download_addr;

	fw_info.addr = addr;
	fw_info.len = 4;

	ret = bes_slave_rx_ready(priv, &buf_cnt, &tx_size, HZ);
	if (!ret) {
		pr_debug("sdio slave rx buf cnt:%d,buf len max:%d\n", buf_cnt, tx_size);
	} else {
		pr_err("wait bes sdio slave rx ready tiemout:%d\n", ret);
		return ret;
	}

	short_buf = kzalloc(512, GFP_KERNEL);
	if (!short_buf)
		return -ENOMEM;

	header.type = FRAME_HEADER_DOWNLOAD_INFO;
	header.seq = frame_num;
	header.len = sizeof(struct fw_info_t);
	frame_num++;
	memcpy(short_buf, (u8 *)&header, sizeof(struct fw_msg_hdr_t));
	memcpy(short_buf + sizeof(struct fw_msg_hdr_t), (u8 *)&fw_info, sizeof(struct fw_info_t));
	length = BES_FW_MSG_TOTAL_LEN(header);
	length = length > 512 ? length : 512;
	ret = cw1200_data_write(priv, short_buf, length);
	if (ret) {
		pr_err("tx download firmware info err:%d\n", ret);
		goto err;
	}

	ret = bes_slave_tx_ready(priv, &rx_size, HZ);
	if (!ret) {
		pr_debug("sdio slave tx ready %d bytes\n", rx_size);
	} else {
		pr_err("wait slave process failed:%d\n", ret);
		goto err;
	}

	ret = cw1200_data_read(priv, short_buf, rx_size);
	if (ret) {
		pr_err("rx download firmware info rsp err:%d\n", ret);
		goto err;
	}

	header.type = FRAME_HEADER_DOWNLOAD_DATA;
	header.seq = frame_num;
	header.len = 8;
	frame_num++;

	download_addr.addr = fw_info.addr;

	memcpy(short_buf, (u8 *)&header, sizeof(struct fw_msg_hdr_t));
	memcpy(short_buf + sizeof(struct fw_msg_hdr_t), &download_addr.addr, sizeof(struct download_fw_t));
	memcpy(short_buf + sizeof(struct fw_msg_hdr_t) + sizeof(struct download_fw_t), &val, 4);
	length = BES_FW_MSG_TOTAL_LEN(header);

	length = length > 512 ? length : 512;
	ret = cw1200_data_write(priv, short_buf, length);
	if (ret) {
		pr_err("tx download fw data err:%d\n", ret);
		goto err;
	}
	ret = bes_slave_tx_ready(priv, &rx_size, HZ);
	if (!ret) {
		pr_debug("bes_slave ready tx %d bytes\n", rx_size);
	} else {
		pr_err("wait slave process download fw data err:%d\n", ret);
		goto err;
	}

	ret = cw1200_data_read(priv, short_buf, rx_size);
	if (ret) {
		pr_err("rx tx download fw data rsp err:%d\n", ret);
		goto err;
	}

err:
	kfree(short_buf);
	return ret;
}

static int bes_frame_rsp_check(struct cw1200_common *priv, void *rsp, u8 frame_num)
{
	int ret = 0;
	struct frame_struct_t *pframe = (struct frame_struct_t *)rsp;
	if (pframe->type == FRAME_HEADER_REPLY) {
		if (pframe->frame_num == frame_num) {
			if (pframe->len == 4) {
				if (pframe->payload == ERR_NONE) {
					pr_debug("bes slave  download firmware is ready\n");
				} else {
					pr_err("frame payload=0x%x\n", pframe->payload);
					ret = -200;
				}
			} else {
				pr_err("payload len error:%u\n", pframe->len);
				ret = -201;
			}
		} else {
			pr_err("frame num err. 0x%x != 0x%x. len:%u\n",
				pframe->frame_num, frame_num, pframe->len);
			ret = -202;
		}
	} else {
		pr_err("frame type err. type 0x%x num=0x%x(0x%x), len:%u\n",
			pframe->type, pframe->frame_num, frame_num, pframe->len);
		ret = -203;
	}
	return ret;
}

static int bes_firmware_download_write_mem(struct cw1200_common *priv, const u32 addr, const u8 *data, const  u32 len)
{
	u8 frame_num = 0;
	u8 last_frame_num = 0;
	u8 buf_cnt = 0;

	u16 tx_size = 0;
	u16 rx_size = 0;

	u32 length = 0;
	u32 code_length = len;
	u32 retry_cnt = 0;
	int ret;

	const u8 *data_p;
	u8 *short_buf, *long_buf;

	struct fw_msg_hdr_t header;
	struct fw_info_t fw_info;
	struct download_fw_t download_addr;
	struct fw_crc_t crc32_t;

retry:
	fw_info.addr = addr;
	fw_info.len = len;
	data_p = data;

	crc32_t.crc32 = 0;
	crc32_t.crc32 ^= 0xffffffffL;
	crc32_t.crc32 = crc32_le(crc32_t.crc32, (u8 *)data, len);
	crc32_t.crc32 ^= 0xffffffffL;

	ret = bes_slave_rx_ready(priv, &buf_cnt, &tx_size, HZ);
	if (!ret) {
		pr_debug("sdio slave rx buf cnt:%d,buf len max:%d\n", buf_cnt, tx_size);
	} else {
		pr_info("wait bes sdio slave rx ready tiemout:%d\n", ret);
		return ret;
	}

	header.type = FRAME_HEADER_DOWNLOAD_INFO;
	header.seq = frame_num;
	header.len = sizeof(struct fw_info_t);
	last_frame_num = frame_num;
	frame_num++;

	short_buf = kzalloc(512, GFP_KERNEL);
	if (!short_buf)
		return -ENOMEM;
	memcpy(short_buf, (u8 *)&header, sizeof(struct fw_msg_hdr_t));
	memcpy(short_buf + sizeof(struct fw_msg_hdr_t), (u8 *)&fw_info, sizeof(struct fw_info_t));
	length = BES_FW_MSG_TOTAL_LEN(header);

	if (tx_size > length) {
		pr_debug("%s", "tx download firmware info\n");
	} else {
		pr_info("%s:%d bes slave has no enough buffer%d/%d\n", __func__, __LINE__, tx_size, length);
		goto err1;
	}

	length = length > 512 ? length : 512;
	ret = cw1200_data_write(priv, short_buf, length);
	if (ret) {
		pr_err("tx download firmware info err:%d\n", ret);
		goto err1;
	}

	ret = bes_slave_tx_ready(priv, &rx_size, HZ);
	if (!ret) {
		pr_debug("sdio slave tx ready %d bytes\n", rx_size);
	} else {
		pr_info("wait slave process failed:%d\n", ret);
		goto err1;
	}

	ret = cw1200_data_read(priv, short_buf, rx_size);
	if (ret) {
		pr_err("rx download firmware info rsp err:%d\n", ret);
		goto err1;
	}

	//check device rx status
	ret = bes_frame_rsp_check(priv, short_buf, last_frame_num);
	if (ret) {
		pr_err("rsp download firmware info err:%d\n", ret);
		goto err1;
	}

	//download firmware
	long_buf = kmalloc(1024 * 32, GFP_KERNEL);
	if (!long_buf) {
		pr_err("%s:%d fw failed to allocate memory\n",__func__, __LINE__);
		ret = -ENOMEM;
		goto err1;
	}
	download_addr.addr = fw_info.addr;

	while (code_length) {

		ret = bes_slave_rx_ready(priv, &buf_cnt, &tx_size, HZ);
		if (ret) {
			goto err2;
		} else {
			pr_debug("bes salve rx ready %d bytes\n", tx_size);
		}

		if ((tx_size < 4) || (tx_size % 4)) {
			pr_err("%s:%d tx size=%d\n", __func__, __LINE__, tx_size);
			ret = -203;
			goto err2;
		}

		if ((code_length + sizeof(struct fw_msg_hdr_t) + sizeof(struct download_fw_t)) < tx_size) {
			length = code_length + sizeof(struct download_fw_t);
		} else {
			length = tx_size - sizeof(struct fw_msg_hdr_t);
		}

		header.type = FRAME_HEADER_DOWNLOAD_DATA;
		header.seq = frame_num;
		header.len = length;
		last_frame_num = frame_num;
		frame_num++;

		memcpy(long_buf, (u8 *)&header, sizeof(struct fw_msg_hdr_t));
		memcpy(long_buf + sizeof(struct fw_msg_hdr_t), &download_addr.addr, sizeof(struct download_fw_t));
		length -= sizeof(struct download_fw_t);//real data length
		memcpy(long_buf + sizeof(struct fw_msg_hdr_t) + sizeof(struct download_fw_t), data_p, length);

		length += (sizeof(struct fw_msg_hdr_t) + sizeof(struct download_fw_t));

		pr_debug("tx_download_firmware_data:%x %d\n", download_addr.addr, length);

		ret = cw1200_data_write(priv, long_buf, length > 512 ? length : 512);
		if (ret) {
			pr_err("tx download fw data err:%d\n", ret);
			goto err2;
		}
		length -= (sizeof(struct fw_msg_hdr_t) + sizeof(struct download_fw_t));

		ret = bes_slave_tx_ready(priv, &rx_size, HZ);
		if (!ret) {
			pr_debug("bes_slave ready tx %d bytes\n", rx_size);
		} else {
			pr_err("wait slave process download fw data err:%d\n", ret);
			goto err2;
		}

		ret = cw1200_data_read(priv, short_buf, rx_size);
		if (ret) {
			pr_err("rx tx download fw data rsp err:%d\n", ret);
			goto err2;
		}

		//check device rx status
		ret = bes_frame_rsp_check(priv, short_buf, last_frame_num);
		if (ret) {
			pr_err("rsp tx download fw err:%d\n", ret);
			goto err2;
		}

		code_length -= length;
		data_p += length;
		download_addr.addr += length;
		pr_debug("already tx fw size:%x/%x\n", download_addr.addr - fw_info.addr, fw_info.len);
	}

	//Notify Device:The firmware download is complete

	ret = bes_slave_rx_ready(priv, &buf_cnt, &tx_size, HZ);
	if (ret) {
		goto err2;
	} else {
		pr_debug("bes salve rx ready %d bytes\n", tx_size);
	}

	header.type = FRAME_HEADER_DOWNLOAD_END;
	header.seq = frame_num;
	header.len = sizeof(struct fw_crc_t);
	last_frame_num = frame_num;
	frame_num++;

	memcpy(short_buf, (u8 *)&header, sizeof(struct fw_msg_hdr_t));
	memcpy(short_buf + sizeof(struct fw_msg_hdr_t), (u8 *)&crc32_t.crc32, sizeof(struct fw_crc_t));
	length = BES_FW_MSG_TOTAL_LEN(header);

	pr_debug("%s", "tx download firmware complete command\n");

	length = length > 512 ? length : 512;
	ret = cw1200_data_write(priv, short_buf, length);
	if (ret) {
		pr_err("tx downlod firmware complete command err:%d\n", ret);
		goto err2;
	}

	ret = bes_slave_tx_ready(priv, &rx_size, HZ);
	if (!ret) {
		pr_debug("bes_slave ready tx %d bytes\n", rx_size);
	} else {
		pr_err("wait slave process download fw data err:%d\n", ret);
		goto err2;
	}

	ret = cw1200_data_read(priv, short_buf, rx_size);
	if (ret) {
		pr_err("receive download firmware complete cmd rsp err:%d\n", ret);
		goto err2;
	}

	//check device rx status
	ret = bes_frame_rsp_check(priv, short_buf, last_frame_num);
	if (ret) {
		pr_err("rsp download firmware complete err:%d\n", ret);
		goto err2;
	}
err2:
	kfree(long_buf);
err1:
	kfree(short_buf);

	if (ret && retry_cnt < 3) {
		retry_cnt++;
		goto retry;
	}
	return ret;
}

static void bes_parse_fw_info(struct cw1200_common *priv, const u8 *data, u32 data_len, u32 *load_addr, u32 *crc32)
{
	u8 buffer[16];
	struct exec_struct_t exec_struct;
	u32 exec_addr_last4byte;
	u32 crc_le = 0;

	crc_le = crc32_le(0xffffffffL, (u8 *)data, data_len - CODE_DATA_USELESS_SIZE);
	crc_le ^= 0xffffffffL;

	// read entry,param,sp,exec_addr

	memcpy((u8 *)buffer, (u8 *)data, sizeof(exec_struct));
	exec_struct.entry       = ((struct exec_struct_t *)buffer)->entry;//PC
	exec_struct.param       = ((struct exec_struct_t *)buffer)->param;
	exec_struct.sp          = ((struct exec_struct_t *)buffer)->sp;
	exec_struct.exec_addr   = ((struct exec_struct_t *)buffer)->exec_addr;//load addr

	pr_debug("crc32                :0x%08X\n", crc_le);
	pr_debug("exec_struct.entry    :0x%08X\n", exec_struct.entry);
	pr_debug("exec_struct.param    :0x%08X\n", exec_struct.param);
	pr_debug("exec_struct.sp       :0x%08X\n", exec_struct.sp);
	pr_debug("exec_struct.exec_addr:0x%08X\n", exec_struct.exec_addr);

	exec_addr_last4byte = (*((u32 *)(data + data_len - 4)));

	pr_debug("exec_addr_last4byte  :0x%08X\n", exec_addr_last4byte);
	if ((!exec_struct.exec_addr) || (exec_struct.exec_addr != exec_addr_last4byte && exec_addr_last4byte)) {
		exec_struct.exec_addr = exec_addr_last4byte;
		pr_debug("exec_addr_last4byte covered exec_struct.exec_addr\n");
	}

	pr_debug("final exec_struct.exec_addr:0x%08X\n", exec_struct.exec_addr);

	*load_addr = exec_struct.exec_addr;

	*crc32 = crc_le;
}

static const u8* bes2600_get_firmware_version_info(struct cw1200_common *priv, const u8 *data, u32 count)
{
        int i = 0;
        const u8 *tmp_ptr = NULL;
        const char month[12][4] = {
                "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
        };

        if(!data || count < 4)
                return NULL;

        for(tmp_ptr = data + count - 3; tmp_ptr > data; tmp_ptr -= 1) {
                for(i = 0; i < 12; i++) {
                        if(memcmp(tmp_ptr, month[i], 3) == 0) {
                                return tmp_ptr;
                        }
                }
        }

        return NULL;
}

static int bes_firmware_download(struct cw1200_common *priv, const char *fw_name, bool auto_run)
{
	u8 frame_num = 0;
	u8 last_frame_num = 0;
	u8 buf_cnt = 0;

	u16 tx_size = 0;
	u16 rx_size = 0;

	u32 length = 0;
	u32 code_length = 0;
	u32 retry_cnt = 0;
	int ret;
	const u8 *fw_ver_ptr;
	const u8 *data_p;
	u8 *short_buf, *long_buf;

	const struct firmware *fw_bin;

	struct fw_msg_hdr_t header;
	struct fw_info_t fw_info;
	struct download_fw_t download_addr;
	struct fw_crc_t crc32_t;
	struct run_fw_t run_addr;

retry:
	ret = request_firmware(&fw_bin, fw_name, NULL);
	if (ret) {
		pr_err("request firmware err:%d\n", ret);
		return ret;
	}
	pr_debug("%s fw.size=%ld\n", __func__, (long)fw_bin->size);

	bes_parse_fw_info(priv, fw_bin->data, fw_bin->size, &fw_info.addr, &crc32_t.crc32);

	fw_ver_ptr = bes2600_get_firmware_version_info(priv, fw_bin->data, fw_bin->size);
	if(fw_ver_ptr == NULL)
		pr_err("------Firmware version get failed\n");
	else
		pr_info("------Firmware: %s version :%s\n", fw_name ,fw_ver_ptr);

	pr_debug("------load addr  :0x%08X\n", fw_info.addr);
	pr_debug("------data crc   :0x%08X\n", crc32_t.crc32);

	code_length = fw_bin->size - CODE_DATA_USELESS_SIZE;
	pr_debug("------code size  :%d\n", code_length);

	fw_info.len = code_length;
	data_p = fw_bin->data;

	ret = bes_slave_rx_ready(priv, &buf_cnt, &tx_size, HZ);
	if (!ret) {
		pr_debug("sdio slave rx buf cnt:%d,buf len max:%d\n", buf_cnt, tx_size);
	} else {
		pr_info("wait bes sdio slave rx ready tiemout:%d\n", ret);
		return ret;
	}

	header.type = FRAME_HEADER_DOWNLOAD_INFO;
	header.seq = frame_num;
	header.len = sizeof(struct fw_info_t);
	last_frame_num = frame_num;
	frame_num++;

	short_buf = kzalloc(512, GFP_KERNEL);
	if (!short_buf)
		return -ENOMEM;
	memcpy(short_buf, (u8 *)&header, sizeof(struct fw_msg_hdr_t));
	memcpy(short_buf + sizeof(struct fw_msg_hdr_t), (u8 *)&fw_info, sizeof(struct fw_info_t));
	length = BES_FW_MSG_TOTAL_LEN(header);

	pr_info("Fw Info: %*ph", length, short_buf);

	if (tx_size > length) {
		pr_debug("%s", "tx download firmware info\n");
	} else {
		pr_info("%s:%d bes slave has no enough buffer%d/%d\n", __func__, __LINE__, tx_size, length);
		goto err1;
	}

	length = length > 512 ? length : 512;
	ret = cw1200_data_write(priv, short_buf, length);
	if (ret) {
		pr_err("tx download firmware info err:%d\n", ret);
		goto err1;
	}

	ret = bes_slave_tx_ready(priv, &rx_size, HZ);
	if (!ret) {
		pr_debug("sdio slave tx ready %d bytes\n", rx_size);
	} else {
		pr_info("wait slave process failed:%d\n", ret);
		goto err1;
	}

	ret = cw1200_data_read(priv, short_buf, rx_size);
	if (ret) {
		pr_err("rx download firmware info rsp err:%d\n", ret);
		goto err1;
	}

	//check device rx status
	ret = bes_frame_rsp_check(priv, short_buf, last_frame_num);
	if (ret) {
		pr_err("rsp download firmware info err:%d\n", ret);
		goto err1;
	}

	//download firmware
	long_buf = kmalloc(1024 * 32, GFP_KERNEL);
	if (!long_buf) {
		pr_err("%s:%d fw failed to allocate memory\n",__func__, __LINE__);
		ret = -ENOMEM;
		goto err1;
	}
	download_addr.addr = fw_info.addr;

	while (code_length) {
		ret = bes_slave_rx_ready(priv, &buf_cnt, &tx_size, HZ);
		if (ret) {
			goto err2;
		} else {
			pr_debug("bes salve rx ready %d bytes\n", tx_size);
		}

		if ((tx_size < 4) || (tx_size % 4)) {
			pr_err("%s:%d tx size=%d\n", __func__, __LINE__, tx_size);
			ret = -203;
			goto err2;
		}

		if ((code_length + sizeof(struct fw_msg_hdr_t) + sizeof(struct download_fw_t)) < tx_size) {
			length = code_length + sizeof(struct download_fw_t);
		} else {
			length = tx_size - sizeof(struct fw_msg_hdr_t);
		}

		header.type = FRAME_HEADER_DOWNLOAD_DATA;
		header.seq = frame_num;
		header.len = length;
		last_frame_num = frame_num;
		frame_num++;

		memcpy(long_buf, (u8 *)&header, sizeof(struct fw_msg_hdr_t));
		memcpy(long_buf + sizeof(struct fw_msg_hdr_t), &download_addr.addr, sizeof(struct download_fw_t));
		length -= sizeof(struct download_fw_t);//real data length
		memcpy(long_buf + sizeof(struct fw_msg_hdr_t) + sizeof(struct download_fw_t), data_p, length);

		length += (sizeof(struct fw_msg_hdr_t) + sizeof(struct download_fw_t));

		//mdelay(5000);
		pr_debug("tx_download_firmware_data:%x %d\n", download_addr.addr, length);

		ret = cw1200_data_write(priv, long_buf, length > 512 ? length : 512);
		if (ret) {
			pr_err("tx download fw data err:%d\n", ret);
			goto err2;
		}
		length -= (sizeof(struct fw_msg_hdr_t) + sizeof(struct download_fw_t));

		ret = bes_slave_tx_ready(priv, &rx_size, HZ);
		if (!ret) {
			pr_debug("bes_slave ready tx %d bytes\n", rx_size);
		} else {
			pr_err("wait slave process download fw data err:%d\n", ret);
			goto err2;
		}

		ret = cw1200_data_read(priv, short_buf, rx_size);
		if (ret) {
			pr_err("rx tx download fw data rsp err:%d\n", ret);
			goto err2;
		}

		//check device rx status
		ret = bes_frame_rsp_check(priv, short_buf, last_frame_num);
		if (ret) {
			pr_err("rsp tx download fw err:%d\n", ret);
			goto err2;
		}

		code_length -= length;
		data_p += length;
		download_addr.addr += length;
		pr_debug("already tx fw size:%x/%x\n", download_addr.addr - fw_info.addr, fw_info.len);
	}

	//Notify Device:The firmware download is complete

	ret = bes_slave_rx_ready(priv, &buf_cnt, &tx_size, HZ);
	if (ret) {
		goto err2;
	} else {
		pr_debug("bes salve rx ready %d bytes\n", tx_size);
	}

	header.type = FRAME_HEADER_DOWNLOAD_END;
	header.seq = frame_num;
	header.len = sizeof(struct fw_crc_t);
	last_frame_num = frame_num;
	frame_num++;

	memcpy(short_buf, (u8 *)&header, sizeof(struct fw_msg_hdr_t));
	memcpy(short_buf + sizeof(struct fw_msg_hdr_t), (u8 *)&crc32_t.crc32, sizeof(struct fw_crc_t));
	length = BES_FW_MSG_TOTAL_LEN(header);

	pr_debug("%s", "tx download firmware complete command\n");

	length = length > 512 ? length : 512;
	ret = cw1200_data_write(priv, short_buf, length);
	if (ret) {
		pr_err("tx downlod firmware complete command err:%d\n", ret);
		goto err2;
	}

	ret = bes_slave_tx_ready(priv, &rx_size, HZ);
	if (!ret) {
		pr_debug("bes_slave ready tx %d bytes\n", rx_size);
	} else {
		pr_err("wait slave process download fw data err:%d\n", ret);
		goto err2;
	}

	ret = cw1200_data_read(priv, short_buf, rx_size);
	if (ret) {
		pr_err("receive download firmware complete cmd rsp err:%d\n", ret);
		goto err2;
	}

	//check device rx status
	ret = bes_frame_rsp_check(priv, short_buf, last_frame_num);
	if (ret) {
		pr_err("rsp download firmware complete err:%d\n", ret);
		goto err2;
	}

	if (auto_run == false) {
		pr_info("partial firmware(%s) is downloaded successfully\n", fw_name);
		goto err2;
	}

	ret = bes_slave_rx_ready(priv, &buf_cnt, &tx_size, HZ);
	if (ret) {
		goto err2;
	} else {
		pr_debug("bes salve rx ready %d bytes\n", tx_size);
	}

	//Notify Device:Run firmware
	run_addr.addr = fw_info.addr;

	header.type = FRAME_HEADER_RUN_CODE;
	header.seq = frame_num;
	header.len = sizeof(struct run_fw_t);
	last_frame_num = frame_num;
	frame_num++;

	memcpy(short_buf, (u8 *)&header, sizeof(struct fw_msg_hdr_t));
	memcpy(short_buf + sizeof(struct fw_msg_hdr_t), (u8 *)&run_addr.addr, sizeof(struct run_fw_t));
	length = BES_FW_MSG_TOTAL_LEN(header);

	pr_debug("tx run firmware command:0x%X\n", run_addr.addr);

	length = length > 512 ? length : 512;
	ret = cw1200_data_write(priv, short_buf, length);
	if (ret) {
		pr_err("tx run firmware command err:%d\n", ret);
		goto err2;
	}

	ret = bes_slave_tx_ready(priv, &rx_size, HZ);
	if (!ret) {
		pr_debug("bes_slave ready tx %d bytes\n", rx_size);
	} else {
		pr_err("wait slave process run fw cmd err:%d\n", ret);
		goto err2;
	}

	ret = cw1200_data_read(priv, short_buf, rx_size);
	if (ret) {
		pr_err("rx run firmware command err:%d\n", ret);
		goto err2;
	}

	//check device rx status
	ret = bes_frame_rsp_check(priv, short_buf, last_frame_num);
	if (ret) {
		pr_err("rsp run firmware command err:%d\n", ret);
		goto err2;
	}

	pr_info("%s", "firmware is downloaded successfully and is already running\n");
	msleep(500);

err2:
	kfree(long_buf);
err1:
	kfree(short_buf);
	release_firmware(fw_bin);
	if (ret && retry_cnt < 3) {
		retry_cnt++;
		goto retry;
	}
	return ret;
}

static int bes_read_dpd_data(struct cw1200_common *priv)
{
	u16 dpd_size = 0;
	int ret = 0;
	u8 *dpd_buf = NULL;
	u8* mcu_status;
	unsigned long wait_timeout;

	mcu_status = kmalloc(sizeof(*mcu_status), GFP_KERNEL);
	if (!mcu_status)
		return -ENOMEM;

	*mcu_status = 0;

	/* wait for device ready */
	wait_timeout = jiffies + 15 * HZ;
	do {
		msleep(100);
		ret = cw1200_reg_read(priv, BES_SLAVE_STATUS_REG_ID, mcu_status, 1);
	} while(((ret == 0) || (ret == -84)) &&
	        !(*mcu_status & BES_SLAVE_STATUS_DPD_READY) &&
		time_before(jiffies, wait_timeout));

	kfree(mcu_status);

	/* check if read dpd error */
	if(ret < 0 || time_after(jiffies, wait_timeout)) {
		pr_err("wait dpd data ready failed:%d\n", ret);
		return -1;
	}

	/* wait dpd read ready */
	ret = bes_slave_tx_ready(priv, &dpd_size, HZ);
	if (ret)  {
		pr_err("wait dpd data failed:%d\n", ret);
		return -1;
	}

	/* dpd size check */
	if (dpd_size != DPD_BIN_SIZE) {
		pr_err("get dpd data size err:%u\n", dpd_size);
		return -1;
	}

	/* read dpd data */
	dpd_buf = kmalloc(DPD_BIN_FILE_SIZE, GFP_KERNEL);
	if(!dpd_buf) {
		pr_err("allocate dpd buffer failed.\n");
		return -1;
	}

	ret = cw1200_data_read(priv, dpd_buf, dpd_size);
	pr_info("read dpd data size:%d\n", dpd_size);
	if (ret) {
		pr_err("read dpd data failed:%d\n", ret);
		return -1;
	}

	/* update dpd data */
	u32 cal_crc = 0;
	u32 dpd_crc = le32_to_cpup((__le32 *)(dpd_buf));

	/* check if the dpd data is valid */
	cal_crc ^= 0xffffffffL;
	cal_crc = crc32_le(cal_crc, dpd_buf + 4, dpd_size - 4);
	cal_crc ^= 0xffffffffL;
	if (cal_crc != dpd_crc) {
		pr_err("bes2600 dpd data check failed, calc_crc:0x%08x dpd_crc: 0x%08x\n",
			cal_crc, dpd_crc);
		return -1;
	}

	pr_info("bes2600 dpd cali pass.\n");

	return ret;
}

int bes2600_load_firmware(struct cw1200_common *priv)
{
	int ret = 0;
	const struct firmware *fac_bin;

	init_completion(&priv->fw_completion);
	priv->fw_completion_on_irq = true;

	ret = bes_firmware_download_write_reg(priv, 0x40100000, 0x802006);
	if (ret) {
		pr_err( "failed to write 0x40100000\n");
		return ret;
	}

	ret = bes_firmware_download_write_reg(priv, 0x4008602C, 0x3E00C000);
	if (ret) {
		pr_err( "failed to write 0x4008602C\n");
		return ret;
	}

	ret = request_firmware(&fac_bin, "bes2600/factory.bin", NULL);
	if (ret)
		return ret;

	if (fac_bin->size != 72) {
		pr_err( "factory.bin size check failed\n");
		release_firmware(fac_bin);
		return -E2BIG;
	}

	ret = bes_firmware_download_write_mem(priv, BES2600_FACTORY_ADDR, fac_bin->data, fac_bin->size);
	release_firmware(fac_bin);
	if (ret) {
		pr_err("download factory data failed.\n");
		return ret;
	}

	pr_info("bes2600 download cali and wifi signal firmware.\n");
	ret = bes_firmware_download(priv, BES2600_LOAD_BOOT_NAME, true);
	if (ret) {
		pr_err("download dpd cali firmware failed\n");
		return ret;
	}

	if (!ret) {
		pr_info("bes2600 read dpd cali data.\n");
		ret = bes_read_dpd_data(priv);
		if (ret) {
			pr_err("read dpd data failed.\n");
			return ret;
		}
	}

	ret = bes_firmware_download(priv, BES2600_LOAD_FW_NAME, true);
	if (ret) {
		pr_err("download normal firmware failed.\n");
		return ret;
	}

	priv->fw_completion_on_irq = false;

	return ret;
}

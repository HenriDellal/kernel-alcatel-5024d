/*****************************************************************************
*	Copyright(c) 2014 FCI Inc. All Rights Reserved

	File name : fc8180_spi.c

	Description : source of SPI interface

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

	History :
	----------------------------------------------------------------------
*******************************************************************************/
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include "fci_types.h"
#include "fc8180_regs.h"
#include "fci_oal.h"

#ifdef FEATURE_MTK
#include <mach/mt_spi.h>
#endif
#if 1//defined(ZCFG_DTV_CLK_AUXO)
#include <linux/clk.h>

struct clk *dtv_clk_26m = NULL;
struct clk *source = NULL;
struct clk *enable = NULL;
#endif

#define SPI_BMODE           0x00
#define SPI_WMODE           0x10
#define SPI_LMODE           0x20
#define SPI_READ            0x40
#define SPI_WRITE           0x00
#define SPI_AINC            0x80
#define CHIPID              (0 << 3)

#define DRIVER_NAME "isdbt_spi"
struct spi_device *fc8180_spi;

static u8 tx_data[10];
static u8 *rdata_buf;

static DEFINE_MUTEX(fci_spi_lock);
static int fc8180_spi_probe(struct spi_device *spi)
{
	s32 ret;
#ifdef FEATURE_MTK
	struct mt_chip_conf *chip_config;

	chip_config = (struct mt_chip_conf *) spi->controller_data;

	chip_config->setuptime = 3;
	chip_config->holdtime = 3;
	chip_config->high_time = 4;
	chip_config->low_time = 4;
	chip_config->cs_idletime = 2;
	chip_config->ulthgh_thrsh = 0;

	chip_config->cpol = 0;
	chip_config->cpha = 0;

	chip_config->rx_mlsb = 1;
	chip_config->tx_mlsb = 1;

	chip_config->tx_endian = 0;
	chip_config->rx_endian = 0;

	chip_config->com_mod = DMA_TRANSFER;
	chip_config->pause = 0;
	chip_config->finish_intr = 1;
	chip_config->deassert = 0;
	chip_config->ulthigh = 0;
	chip_config->tckdly = 2;
#endif
	print_log(0, "fc8180_spi_probe\n");
	spi->max_speed_hz = 8000000;
	spi->bits_per_word = 8;
	spi->mode =  SPI_MODE_0;

	ret = spi_setup(spi);

	if (ret < 0)
		return ret;

	fc8180_spi = spi;

#if 1//defined(ZCFG_DTV_CLK_AUXO)
		dtv_clk_26m = devm_clk_get(&fc8180_spi->dev, "dtv_clk");
		if (IS_ERR(dtv_clk_26m)) {
			pr_err("get dtv_clk failed!\n");
			ret = -ENODEV;
			return ret;
		}
		
		source = devm_clk_get(&fc8180_spi->dev, "source");
		if (IS_ERR(source)) {
			pr_err("get dtv_clk's source failed!\n");
			ret = -ENODEV;
			return ret;
		}
	
		enable = devm_clk_get(&fc8180_spi->dev, "enable");
		if (IS_ERR(enable)) {
			pr_err("get dtv_clk's enable failed!\n");
			ret = -ENODEV;
			return ret;
		}
		
		clk_set_parent(dtv_clk_26m, source);
		clk_set_rate(dtv_clk_26m, 26000000);
#endif


	return ret;
}

static int fc8180_spi_remove(struct spi_device *spi)
{

	return 0;
}

const struct of_device_id fc8180_match_table[] = {
	{
		.compatible = "fci, isdbt_spi",
	},
	{}
};

static struct spi_driver fc8180_spi_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = fc8180_match_table,
	},
	.probe		= fc8180_spi_probe,
	.remove		= fc8180_spi_remove,
};

#ifdef FEATURE_MTK
static struct spi_board_info fc8180_spi_devs[] __initdata = {
	[0] = {
		.modalias = "fc8180_spi",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.max_speed_hz	 = 100000,
	},
};
#endif
static int fc8180_spi_write_then_read(struct spi_device *spi
	, u8 *txbuf, u16 tx_length, u8 *rxbuf, u16 rx_length)
{
	int res = 0;

	struct spi_message	message;
#ifdef FEATURE_MTK
	struct spi_transfer	t[2];
#endif
	struct spi_transfer	x;

	if (spi == NULL) {
		print_log(0, "[FC8180] FC8180_SPI Handle Fail...........\n");
		return BBM_NOK;
	}

#ifdef FEATURE_MTK
	if (rx_length > (1024 - tx_length)) {
		spi_message_init(&message);
		memset(&t, 0, sizeof(t));

		spi_message_add_tail(&t[0], &message);
		spi_message_add_tail(&t[1], &message);

		memcpy(rdata_buf, txbuf, tx_length);

		t[0].tx_buf = rdata_buf;
		t[0].rx_buf = rdata_buf;
		t[0].len = (rx_length % 1024 + tx_length);
		t[0].bits_per_word = 8;
		t[0].cs_change = 0;

		t[1].tx_buf = rdata_buf;
		t[1].rx_buf = &rdata_buf[rx_length % 1024 + tx_length];
		t[1].len = rx_length - (rx_length % 1024);
		t[1].bits_per_word = 8;
		t[1].cs_change = 0;
		res = spi_sync(spi, &message);
		if ((rxbuf != NULL) && (rx_length > 0))
			memcpy(rxbuf, &rdata_buf[tx_length], rx_length);
	} else
#endif
{
	spi_message_init(&message);
	memset(&x, 0, sizeof(x));

	spi_message_add_tail(&x, &message);

	memcpy(rdata_buf, txbuf, tx_length);

	x.tx_buf = rdata_buf;
	x.rx_buf = rdata_buf;
	x.len = tx_length + rx_length;
	x.cs_change = 0;
	x.bits_per_word = 8;
	res = spi_sync(spi, &message);
	if ((rxbuf != NULL) && (rx_length > 0))
		memcpy(rxbuf, x.rx_buf + tx_length, rx_length);
}
	return res;
}

static s32 spi_bulkread(HANDLE handle, u16 addr, u8 command, u8 *data,
							u32 length)
{
	int res;

	tx_data[0] = addr & 0xff;
	tx_data[1] = (addr >> 8) & 0xff;
	tx_data[2] = (command & 0xf0) | CHIPID | ((length >> 16) & 0x07);
	tx_data[3] = (length >> 8) & 0xff;
	tx_data[4] = length & 0xff;

	res = fc8180_spi_write_then_read(fc8180_spi
		, &tx_data[0], 5, data, length);

	if (res) {
		print_log(0, "[FC8180] fc8180_spi_bulkread fail : %d\n", res);
		return BBM_NOK;
	}

	return BBM_OK;
}

static s32 spi_bulkwrite(HANDLE handle, u16 addr, u8 command, u8 *data,
							u32 length)
{
	int i;
	int res;

	tx_data[0] = addr & 0xff;
	tx_data[1] = (addr >> 8) & 0xff;
	tx_data[2] = (command & 0xf0) | CHIPID | ((length >> 16) & 0x07);
	tx_data[3] = (length >> 8) & 0xff;
	tx_data[4] = length & 0xff;

	for (i = 0; i < length; i++)
		tx_data[5 + i] = data[i];

	res = fc8180_spi_write_then_read(fc8180_spi
		, &tx_data[0], length + 5, data, 0);

	if (res) {
		print_log(0, "[FC8180] fc8180_spi_bulkwrite fail : %d\n", res);
		return BBM_NOK;
	}

	return BBM_OK;
}


static s32 spi_dataread(HANDLE handle, u16 addr, u8 command, u8 *data,
							u32 length)
{
	int res;

	tx_data[0] = addr & 0xff;
	tx_data[1] = (addr >> 8) & 0xff;
	tx_data[2] = (command & 0xf0) | CHIPID | ((length >> 16) & 0x07);
	tx_data[3] = (length >> 8) & 0xff;
	tx_data[4] = length & 0xff;

	res = fc8180_spi_write_then_read(fc8180_spi
		, &tx_data[0], 5, data, length);

	if (res) {
		print_log(0, "[FC8180] fc8180_spi_dataread fail : %d\n", res);
		return BBM_NOK;
	}

	return BBM_OK;
}

s32 fc8180_spi_init(HANDLE handle, u16 param1, u16 param2)
{
	int res = 0;

	res = spi_register_driver(&fc8180_spi_driver);

	if (res) {
		print_log(0, "fc8180_spi register fail : %d\n", res);
		return BBM_NOK;
	}

	if (rdata_buf == NULL) {
		rdata_buf = kmalloc(6 * 1024, GFP_DMA | GFP_KERNEL);

		if (!rdata_buf) {
			print_log(0, "[FC8180] spi rdata_buf kmalloc fail\n");
			return BBM_NOK;
		}
	}
	return BBM_OK;
}

s32 fc8180_spi_byteread(HANDLE handle, u16 addr, u8 *data)
{
	s32 res;
	u8 command = SPI_READ;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkread(handle, addr, command, data, 1);
	mutex_unlock(&fci_spi_lock);
	return res;
}

s32 fc8180_spi_wordread(HANDLE handle, u16 addr, u16 *data)
{
	s32 res;
	u8 command = SPI_READ | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkread(handle, addr, command, (u8 *) data, 2);
	mutex_unlock(&fci_spi_lock);
	return res;
}

s32 fc8180_spi_longread(HANDLE handle, u16 addr, u32 *data)
{
	s32 res;
	u8 command = SPI_READ | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkread(handle, addr, command, (u8 *) data, 4);
	mutex_unlock(&fci_spi_lock);
	return res;
}

s32 fc8180_spi_bulkread(HANDLE handle, u16 addr, u8 *data, u16 length)
{
	s32 res;
	u8 command = SPI_READ | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkread(handle, addr, command, data, length);
	mutex_unlock(&fci_spi_lock);
	return res;
}

s32 fc8180_spi_bytewrite(HANDLE handle, u16 addr, u8 data)
{
	s32 res;
	u8 command = SPI_WRITE;

	mutex_lock(&fci_spi_lock);

#ifdef BBM_SPI_IF
	if (addr == BBM_DM_DATA) {
#ifdef BBM_SPI_PHA_1
		u8 ifcommand = 0xff;
		u16 ifaddr = 0xffff;
		u8 ifdata = 0xff;
#else
		u8 ifcommand = 0x00;
		u16 ifaddr = 0x0000;
		u8 ifdata = 0x00;
#endif
		res = spi_bulkwrite(handle, ifaddr, ifcommand, &ifdata, 1);
	} else
#endif
	res = spi_bulkwrite(handle, addr, command, (u8 *) &data, 1);
	mutex_unlock(&fci_spi_lock);
	return res;
}

s32 fc8180_spi_wordwrite(HANDLE handle, u16 addr, u16 data)
{
	s32 res;
	u8 command = SPI_WRITE | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkwrite(handle, addr, command, (u8 *) &data, 2);
	mutex_unlock(&fci_spi_lock);
	return res;
}

s32 fc8180_spi_longwrite(HANDLE handle, u16 addr, u32 data)
{
	s32 res;
	u8 command = SPI_WRITE | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkwrite(handle, addr, command, (u8 *) &data, 4);
	mutex_unlock(&fci_spi_lock);
	return res;
}

s32 fc8180_spi_bulkwrite(HANDLE handle, u16 addr, u8 *data, u16 length)
{
	s32 res;
	u8 command = SPI_WRITE | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkwrite(handle, addr, command, data, length);
	mutex_unlock(&fci_spi_lock);
	return res;
}

s32 fc8180_spi_dataread(HANDLE handle, u16 addr, u8 *data, u32 length)
{
	s32 res;
	u8 command = SPI_READ;

	mutex_lock(&fci_spi_lock);
	res = spi_dataread(handle, addr, command, data, length);
	mutex_unlock(&fci_spi_lock);

	return res;
}

s32 fc8180_spi_deinit(HANDLE handle)
{
	spi_unregister_driver(&fc8180_spi_driver);
	return BBM_OK;
}

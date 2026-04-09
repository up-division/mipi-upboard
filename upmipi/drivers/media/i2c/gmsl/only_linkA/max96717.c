// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim MAX96717 GMSL2 Serializer Driver
 * Hardware specific layer (Restored exactly from max96717.c.bak)
 */

#include <linux/bitfield.h>
#include <linux/delay.h>            
#include <linux/i2c.h>              
#include <linux/module.h>           
#include <linux/regmap.h>

#include "max_ser.h"

#define WR_REG(reg, val) do { \
	int __r = regmap_write(priv->regmap, (reg), (val)); \
	if (__r) { dev_err(priv->dev, "reg 0x%04x write 0x%02x failed: %d\n", (reg), (val), __r); return __r; } \
} while (0)

static const struct regmap_config max96717_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x1fff,
};

struct max96717_priv {
	struct max_ser ser;
	struct device *dev;
	struct regmap *regmap;
};

#define ser_to_priv(_ser) container_of(_ser, struct max96717_priv, ser)

/* 輔助函數：印出極度詳細的 MIPI 接收封包與時脈狀態 (根據 .bak 修正暫存器位址) */
static void max96717_dump_video_status(struct max96717_priv *priv, const char *prefix)
{
	unsigned int pclkdet = 0, phy_pkt = 0, csi_pkt = 0, phy_clk = 0;
	
	/* 0x0112: PCLK Detect (Bit 7) */
	regmap_read(priv->regmap, 0x0112, &pclkdet);
	/* 0x038D: PHY Pkt Cnt */
	regmap_read(priv->regmap, 0x038D, &phy_pkt);
	/* 0x038E: CSI Pkt Cnt */
	regmap_read(priv->regmap, 0x038E, &csi_pkt);
	/* 0x0390: PHY Clk Cnt */
	regmap_read(priv->regmap, 0x0390, &phy_clk);

	dev_info(priv->dev, "[SER] %s: pclkdet=%u, PHY_Pkts=0x%02x, CSI_Pkts=0x%02x, CLK_Pkts=0x%02x\n",
		 prefix, !!(pclkdet & BIT(7)), phy_pkt, csi_pkt, phy_clk);
}

static int max96717_init(struct max_ser *ser)
{
	return 0;
}

static int max96717_pre_stream(struct max_ser *ser)
{
	struct max96717_priv *priv = ser_to_priv(ser);

	dev_info(priv->dev, "[SER HW] vendor prep: set datatype + camera reset\n");
	WR_REG(0x0318, 0x5E);
	WR_REG(0x02BE, 0x10);
	msleep(200);

	max96717_dump_video_status(priv, "Status after pre-stream prep");
	return 0;
}

static int max96717_post_stream(struct max_ser *ser)
{
	struct max96717_priv *priv = ser_to_priv(ser);

	dev_info(priv->dev, "[SER HW] vendor kick: pulse MFP7/MFP8\n");
	WR_REG(0x02d3, 0x00); 
	WR_REG(0x02d6, 0x00); 
	msleep(300);
	
	WR_REG(0x02d3, 0x10); 
	WR_REG(0x02d6, 0x10); 

	/* ====== 時間軸封包追蹤 (Time-lapse Packet Tracking) ====== */
	msleep(20);
	max96717_dump_video_status(priv, "20ms after kick");

	msleep(30); /* 總計 50ms */
	max96717_dump_video_status(priv, "50ms after kick");

	msleep(50); /* 總計 100ms */
	max96717_dump_video_status(priv, "100ms after kick");

	msleep(100); /* 總計 200ms */
	max96717_dump_video_status(priv, "200ms after kick");

	msleep(300); /* 總計 500ms */
	max96717_dump_video_status(priv, "500ms after kick");

	msleep(500); /* 總計 1000ms */
	max96717_dump_video_status(priv, "1000ms after kick");

	return 0;
}

static int max96717_log_status(struct max_ser *ser)
{
	struct max96717_priv *priv = ser_to_priv(ser);
	
	dev_info(priv->dev, "\n=== MAX96717 HW STATUS DUMP ===\n");
	max96717_dump_video_status(priv, "Current Status");
	dev_info(priv->dev, "=================================\n\n");
	return 0;
}

static const struct max_ser_ops max96717_ops = {
	.init = max96717_init,
	.pre_stream = max96717_pre_stream,
	.post_stream = max96717_post_stream,
	.log_status = max96717_log_status,
};

static int max96717_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max96717_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) return -ENOMEM;

	priv->dev = dev;
	i2c_set_clientdata(client, priv);

	priv->regmap = devm_regmap_init_i2c(client, &max96717_i2c_regmap);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return PTR_ERR(priv->regmap);
	}

	priv->ser.ops = &max96717_ops;

	ret = max_ser_probe(client, &priv->ser);
	if (ret) {
		dev_err(dev, "max_ser_probe failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static void max96717_remove(struct i2c_client *client)
{
	struct max96717_priv *priv = i2c_get_clientdata(client);
	max_ser_remove(&priv->ser);
}

static const struct i2c_device_id max96717_id[] = {
	{ "max96717", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max96717_id);

static struct i2c_driver max96717_i2c_driver = {
	.driver = {
		.name = "max96717",
	},
	.probe = max96717_probe,
	.remove = max96717_remove,
	.id_table = max96717_id,
};

module_i2c_driver(max96717_i2c_driver);
MODULE_IMPORT_NS("MAX_SERDES");
MODULE_DESCRIPTION("Maxim MAX96717 Serializer Driver (Restored from .bak)");
MODULE_LICENSE("GPL");
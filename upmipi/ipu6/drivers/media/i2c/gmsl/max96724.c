// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim MAX96724 Quad GMSL2 Deserializer Driver
 *
 * Modified for Intel IPU6 / ACPI Integration
 * Integrated Power-On Logic from approgmsl/_max9296
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/notifier.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "max_des.h"

#define FORMAT 0 // 0: uyvy 1: rgb888

#define MAX96724_REG0				0x0

#define MAX96724_REG6				0x6
#define MAX96724_REG6_LINK_EN			GENMASK(3, 0)

#define MAX96724_DEBUG_EXTRA			0x9
#define MAX96724_DEBUG_EXTRA_PCLK_SRC		GENMASK(1, 0)
#define MAX96724_DEBUG_EXTRA_PCLK_SRC_25MHZ	0b00
#define MAX96724_DEBUG_EXTRA_PCLK_SRC_75MHZ	0b01
#define MAX96724_DEBUG_EXTRA_PCLK_SRC_USE_PIPE	0b10

#define MAX96724_REG26(x)			(0x10 + (x) / 2)
#define MAX96724_REG26_RX_RATE_PHY(x)		(GENMASK(1, 0) << (4 * ((x) % 2)))
#define MAX96724_REG26_RX_RATE_3GBPS		0b01
#define MAX96724_REG26_RX_RATE_6GBPS		0b10

#define MAX96724_PWR1				0x13
#define MAX96724_PWR1_RESET_ALL			BIT(6)

#define MAX96724_CTRL1				0x18
#define MAX96724_CTRL1_RESET_ONESHOT		GENMASK(3, 0)

#define MAX96724_VIDEO_PIPE_SEL(p)		(0xf0 + (p) / 2)
#define MAX96724_VIDEO_PIPE_SEL_STREAM(p)	(GENMASK(1, 0) << (4 * ((p) % 2)))
#define MAX96724_VIDEO_PIPE_SEL_LINK(p)		(GENMASK(3, 2) << (4 * ((p) % 2)))

#define MAX96724_VIDEO_PIPE_EN			0xf4
#define MAX96724_VIDEO_PIPE_EN_MASK(p)		BIT(p)
#define MAX96724_VIDEO_PIPE_EN_STREAM_SEL_ALL	BIT(4)

#define MAX96724_VPRBS(p)			(0x1dc + (p) * 0x20)
#define MAX96724_VPRBS_VIDEO_LOCK		BIT(0)
#define MAX96724_VPRBS_PATGEN_CLK_SRC		BIT(7)
#define MAX96724_VPRBS_PATGEN_CLK_SRC_150MHZ	0b0
#define MAX96724_VPRBS_PATGEN_CLK_SRC_375MHZ	0b1

#define MAX96724_BACKTOP12			0x40b
#define MAX96724_BACKTOP12_CSI_OUT_EN		BIT(1)

#define MAX96724_BACKTOP21(p)			(0x414 + (p) / 4 * 0x20)
#define MAX96724_BACKTOP21_BPP8DBL(p)		BIT(4 + (p) % 4)

#define MAX96724_BACKTOP22(x)			(0x415 + (x) * 0x3)
#define MAX96724_BACKTOP22_PHY_CSI_TX_DPLL	GENMASK(4, 0)
#define MAX96724_BACKTOP22_PHY_CSI_TX_DPLL_EN	BIT(5)

#define MAX96724_BACKTOP24(p)			(0x417 + (p) / 4 * 0x20)
#define MAX96724_BACKTOP24_BPP8DBL_MODE(p)	BIT(4 + (p) % 4)

#define MAX96724_BACKTOP30(p)			(0x41d + (p) / 4 * 0x20)
#define MAX96724_BACKTOP30_BPP10DBL3		BIT(4)
#define MAX96724_BACKTOP30_BPP10DBL3_MODE	BIT(5)

#define MAX96724_BACKTOP31(p)			(0x41e + (p) / 4 * 0x20)
#define MAX96724_BACKTOP31_BPP10DBL2		BIT(6)
#define MAX96724_BACKTOP31_BPP10DBL2_MODE	BIT(7)

#define MAX96724_BACKTOP32(p)			(0x41f + (p) / 4 * 0x20)
#define MAX96724_BACKTOP32_BPP12(p)		BIT(p)
#define MAX96724_BACKTOP32_BPP10DBL0		BIT(4)
#define MAX96724_BACKTOP32_BPP10DBL0_MODE	BIT(5)
#define MAX96724_BACKTOP32_BPP10DBL1		BIT(6)
#define MAX96724_BACKTOP32_BPP10DBL1_MODE	BIT(7)

#define MAX96724_MIPI_PHY0			0x8a0
#define MAX96724_MIPI_PHY0_PHY_CONFIG		GENMASK(4, 0)
#define MAX96724_MIPI_PHY0_PHY_4X2		BIT(0)
#define MAX96724_MIPI_PHY0_PHY_2X4		BIT(2)
#define MAX96724_MIPI_PHY0_PHY_1X4A_2X2		BIT(3)
#define MAX96724_MIPI_PHY0_PHY_1X4B_2X2		BIT(4)
#define MAX96724_MIPI_PHY0_FORCE_CSI_OUT_EN	BIT(7)

#define MAX96724_MIPI_PHY2			0x8a2
#define MAX96724_MIPI_PHY2_PHY_STDB_N_4(x)	(GENMASK(5, 4) << ((x) / 2 * 2))
#define MAX96724_MIPI_PHY2_PHY_STDB_N_2(x)	(BIT(4 + (x)))

#define MAX96724_MIPI_PHY3(x)			(0x8a3 + (x) / 2)
#define MAX96724_MIPI_PHY3_PHY_LANE_MAP_4	GENMASK(7, 0)
#define MAX96724_MIPI_PHY3_PHY_LANE_MAP_2(x)	(GENMASK(3, 0) << (4 * ((x) % 2)))

#define MAX96724_MIPI_PHY5(x)			(0x8a5 + (x) / 2)
#define MAX96724_MIPI_PHY5_PHY_POL_MAP_4_0_1	GENMASK(1, 0)
#define MAX96724_MIPI_PHY5_PHY_POL_MAP_4_2_3	GENMASK(4, 3)
#define MAX96724_MIPI_PHY5_PHY_POL_MAP_4_CLK	BIT(5)
#define MAX96724_MIPI_PHY5_PHY_POL_MAP_2(x)	(GENMASK(1, 0) << (3 * ((x) % 2)))
#define MAX96724_MIPI_PHY5_PHY_POL_MAP_2_CLK(x)	BIT(2 + 3 * ((x) % 2))

#define MAX96724_MIPI_PHY13			0x8ad
#define MAX96724_MIPI_PHY13_T_T3_PREBEGIN	GENMASK(5, 0)
#define MAX96724_MIPI_PHY13_T_T3_PREBEGIN_64X7	FIELD_PREP(MAX96724_MIPI_PHY13_T_T3_PREBEGIN, 63)

#define MAX96724_MIPI_PHY14			0x8ae
#define MAX96724_MIPI_PHY14_T_T3_PREP		GENMASK(1, 0)
#define MAX96724_MIPI_PHY14_T_T3_PREP_55NS	FIELD_PREP(MAX96724_MIPI_PHY14_T_T3_PREP, 0b01)
#define MAX96724_MIPI_PHY14_T_T3_POST		GENMASK(6, 2)
#define MAX96724_MIPI_PHY14_T_T3_POST_32X7	FIELD_PREP(MAX96724_MIPI_PHY14_T_T3_POST, 31)

#define MAX96724_MIPI_CTRL_SEL			0x8ca
#define MAX96724_MIPI_CTRL_SEL_MASK(p)		(GENMASK(1, 0) << ((p) * 2))

#define MAX96724_MIPI_PHY25(x)			(0x8d0 + (x) / 2)
#define MAX96724_MIPI_PHY25_CSI2_TX_PKT_CNT(x)	(GENMASK(3, 0) << (4 * ((x) % 2)))

#define MAX96724_MIPI_PHY27(x)			(0x8d2 + (x) / 2)
#define MAX96724_MIPI_PHY27_PHY_PKT_CNT(x)	(GENMASK(3, 0) << (4 * ((x) % 2)))

#define MAX96724_MIPI_TX3(x)			(0x903 + (x) * 0x40)
#define MAX96724_MIPI_TX3_DESKEW_INIT_8X32K	FIELD_PREP(GENMASK(2, 0), 0b001)
#define MAX96724_MIPI_TX3_DESKEW_INIT_AUTO	BIT(7)

#define MAX96724_MIPI_TX4(x)			(0x904 + (x) * 0x40)
#define MAX96724_MIPI_TX4_DESKEW_PER_2K		FIELD_PREP(GENMASK(2, 0), 0b001)
#define MAX96724_MIPI_TX4_DESKEW_PER_AUTO	BIT(7)

#define MAX96724_MIPI_TX10(x)			(0x90a + (x) * 0x40)
#define MAX96724_MIPI_TX10_CSI2_CPHY_EN		BIT(5)
#define MAX96724_MIPI_TX10_CSI2_LANE_CNT	GENMASK(7, 6)

#define MAX96724_MIPI_TX11(p)			(0x90b + (p) * 0x40)
#define MAX96724_MIPI_TX12(p)			(0x90c + (p) * 0x40)

#define MAX96724_MIPI_TX13(p, x)		(0x90d + (p) * 0x40 + (x) * 0x2)
#define MAX96724_MIPI_TX13_MAP_SRC_DT		GENMASK(5, 0)
#define MAX96724_MIPI_TX13_MAP_SRC_VC		GENMASK(7, 6)

#define MAX96724_MIPI_TX14(p, x)		(0x90e + (p) * 0x40 + (x) * 0x2)
#define MAX96724_MIPI_TX14_MAP_DST_DT		GENMASK(5, 0)
#define MAX96724_MIPI_TX14_MAP_DST_VC		GENMASK(7, 6)

#define MAX96724_MIPI_TX45(p, x)		(0x92d + (p) * 0x40 + (x) / 4)
#define MAX96724_MIPI_TX45_MAP_DPHY_DEST(x)	(GENMASK(1, 0) << (2 * ((x) % 4)))

#define MAX96724_MIPI_TX51(x)			(0x933 + (x) * 0x40)
#define MAX96724_MIPI_TX51_ALT_MEM_MAP_12	BIT(0)
#define MAX96724_MIPI_TX51_ALT_MEM_MAP_8	BIT(1)
#define MAX96724_MIPI_TX51_ALT_MEM_MAP_10	BIT(2)
#define MAX96724_MIPI_TX51_ALT2_MEM_MAP_8	BIT(4)

#define MAX96724_MIPI_TX54(x)			(0x936 + (x) * 0x40)
#define MAX96724_MIPI_TX54_TUN_EN		BIT(0)

#define MAX96724_MIPI_TX57(x)			(0x939 + (x) * 0x40)
#define MAX96724_MIPI_TX57_TUN_DEST		GENMASK(5, 4)
#define MAX96724_MIPI_TX57_DIS_AUTO_TUN_DET	BIT(6)
#define MAX96724_DET(p)				BIT(p)

#define MAX96724_PATGEN_0			0x1050
#define MAX96724_PATGEN_0_VTG_MODE		GENMASK(1, 0)
#define MAX96724_PATGEN_0_VTG_MODE_FREE_RUNNING	0b11
#define MAX96724_PATGEN_0_DE_INV		BIT(2)
#define MAX96724_PATGEN_0_HS_INV		BIT(3)
#define MAX96724_PATGEN_0_VS_INV		BIT(4)
#define MAX96724_PATGEN_0_GEN_DE		BIT(5)
#define MAX96724_PATGEN_0_GEN_HS		BIT(6)
#define MAX96724_PATGEN_0_GEN_VS		BIT(7)

#define MAX96724_PATGEN_1			0x1051
#define MAX96724_PATGEN_1_PATGEN_MODE		GENMASK(5, 4)
#define MAX96724_PATGEN_1_PATGEN_MODE_DISABLED	0b00
#define MAX96724_PATGEN_1_PATGEN_MODE_CHECKER	0b01
#define MAX96724_PATGEN_1_PATGEN_MODE_GRADIENT	0b10

#define MAX96724_VS_DLY_2			0x1052
#define MAX96724_VS_HIGH_2			0x1055
#define MAX96724_VS_LOW_2			0x1058
#define MAX96724_V2H_2				0x105b
#define MAX96724_HS_HIGH_1			0x105e
#define MAX96724_HS_LOW_1			0x1060
#define MAX96724_HS_CNT_1			0x1062
#define MAX96724_V2D_2				0x1064
#define MAX96724_DE_HIGH_1			0x1067
#define MAX96724_DE_LOW_1			0x1069
#define MAX96724_DE_CNT_1			0x106b
#define MAX96724_GRAD_INCR			0x106d
#define MAX96724_CHKR_COLOR_A_L			0x106e
#define MAX96724_CHKR_COLOR_B_L			0x1071
#define MAX96724_CHKR_RPT_A			0x1074
#define MAX96724_CHKR_RPT_B			0x1075
#define MAX96724_CHKR_ALT			0x1076

#define MAX96724_DE_DET				0x11f0
#define MAX96724_HS_DET				0x11f1
#define MAX96724_VS_DET				0x11f2
#define MAX96724_HS_POL				0x11f3
#define MAX96724_VS_POL				0x11f4
#define MAX96724_DET(p)				BIT(p)

#define MAX96724_DPLL_0(x)			(0x1c00 + (x) * 0x100)
#define MAX96724_DPLL_0_CONFIG_SOFT_RST_N	BIT(0)

#define MAX96724_PHY1_ALT_CLOCK			5

static const struct regmap_config max96724_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x1f00,
};

struct max96724_priv {
	struct max_des des;
	const struct max96724_chip_info *info;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;

	struct gpio_desc *gpiod_enable; /* 用於 Reset */
	struct gpio_desc *gpiod_poc;    /* 用於 POC 電源 */

	/* True if phys_config=5 (streaming mode) was successfully written
	 * at probe time, before the I2C mux adapter gets locked during
	 * s_stream. When set, max96724_init skips the hardware writes and
	 * returns immediately at s_stream time. */
};

struct max96724_chip_info {
	unsigned int versions;
	unsigned int modes;
	bool supports_pipe_stream_autoselect;
	unsigned int num_pipes;

	int (*set_pipe_phy)(struct max_des *des, struct max_des_pipe *pipe,
			    struct max_des_phy *phy);
	int (*set_pipe_tunnel_phy)(struct max_des *des, struct max_des_pipe *pipe,
				   struct max_des_phy *phy);
	int (*set_pipe_tunnel_enable)(struct max_des *des, struct max_des_pipe *pipe,
				      bool enable);
};

#define des_to_priv(_des) \
	container_of(_des, struct max96724_priv, des)

static void max96724_health_check(struct max_des *des, const char *tag)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int val = 0;
	int ret;

	dev_info(priv->dev,
		 "[HC:%s] adapter=%s nr=%d addr=0x%02x pm_enabled=%d active=%d runtime_status=%d\n",
		 tag,
		 priv->client->adapter->name,
		 priv->client->adapter->nr,
		 priv->client->addr,
		 pm_runtime_enabled(&priv->client->dev),
		 pm_runtime_active(&priv->client->dev),
		 priv->client->dev.power.runtime_status);

	ret = regmap_read(priv->regmap, MAX96724_REG0, &val);
	dev_info(priv->dev, "[HC:%s] REG0 ret=%d val=0x%02x\n", tag, ret, val);

	ret = regmap_read(priv->regmap, MAX96724_BACKTOP12, &val);
	dev_info(priv->dev, "[HC:%s] BACKTOP12 ret=%d val=0x%02x\n", tag, ret, val);

	ret = regmap_read(priv->regmap, MAX96724_VPRBS(0), &val);
	dev_info(priv->dev, "[HC:%s] VPRBS(pipe0) ret=%d val=0x%02x\n", tag, ret, val);

	ret = regmap_read(priv->regmap, MAX96724_MIPI_PHY25(1), &val);
	dev_info(priv->dev, "[HC:%s] MIPI_PHY25(phy1) ret=%d val=0x%02x\n", tag, ret, val);
}

static int max96724_wait_for_device(struct max96724_priv *priv)
{
	unsigned int i;
	int ret = -EREMOTEIO;

	for (i = 0; i < 20; i++) {
		unsigned int val = 0;

		ret = regmap_read(priv->regmap, MAX96724_REG0, &val);
		if (!ret && val) {
			dev_info(priv->dev,
				 "Device reachable after %u ms: reg0=0x%02x\n",
				 i * 20, val);
			return 0;
		}

		if (ret)
			dev_dbg(priv->dev,
				"Retry %u waiting for deserializer: ret=%d\n",
				i, ret);
		else
			dev_dbg(priv->dev,
				"Retry %u waiting for deserializer: reg0=0x%02x\n",
				i, val);

		msleep(20);
	}

	return ret ? ret : -EREMOTEIO;
}

static int max96724_reset(struct max96724_priv *priv)
{
	int ret;

	/*
	 * 修正重點：
	 * 先確認 des 本體可通，再送 soft reset。
	 *
	 * 你目前新版本的問題，是還沒確認晶片 I2C 可達就先寫 RESET_ALL，
	 * 很容易在尚未穩定的狀態下把控制路徑打亂，接著 wait_for_device
	 * 就一路 -121。
	 */
	ret = max96724_wait_for_device(priv);
	if (ret) {
		dev_err(priv->dev,
			"Deserializer not reachable before reset: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(priv->regmap, MAX96724_PWR1,
				 MAX96724_PWR1_RESET_ALL,
				 MAX96724_PWR1_RESET_ALL);
	if (ret) {
		dev_err(priv->dev, "Soft reset write failed: %d\n", ret);
		return ret;
	}

	fsleep(10000);

	ret = max96724_wait_for_device(priv);
	if (ret) {
		dev_err(priv->dev,
			"Deserializer not reachable after soft reset: %d\n", ret);
		return ret;
	}

	return 0;
}

static int __maybe_unused max96724_reg_read(struct max_des *des, unsigned int reg,
					    unsigned int *val)
{
	struct max96724_priv *priv = des_to_priv(des);

	return regmap_read(priv->regmap, reg, val);
}

static int __maybe_unused max96724_reg_write(struct max_des *des, unsigned int reg,
					     unsigned int val)
{
	struct max96724_priv *priv = des_to_priv(des);

	return regmap_write(priv->regmap, reg, val);
}

static unsigned int max96724_phy_id(struct max_des *des, struct max_des_phy *phy)
{
	unsigned int num_hw_data_lanes = max_des_phy_hw_data_lanes(des, phy);

	/* PHY 1 is the master PHY when combining PHY 0 and PHY 1. */
	if (phy->index == 0 && num_hw_data_lanes == 4)
		return 1;

	if (phy->index == 1 && !des->phys[1].enabled)
		return 0;

	return phy->index;
}

static int max96724_log_pipe_status(struct max_des *des,
				    struct max_des_pipe *pipe)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;
	unsigned int val, mask;
	int ret;

	max96724_health_check(des, "log-pipe");

	ret = regmap_read(priv->regmap, MAX96724_VPRBS(index), &val);
	if (ret) {
		dev_warn(priv->dev,
			 "\tlog_pipe_status: read VPRBS(pipe=%u, reg=0x%04x) failed: %d\n",
			 index, MAX96724_VPRBS(index), ret);
		return ret;
	}

	dev_info(priv->dev, "\tvideo_lock: %u\n",
		 !!(val & MAX96724_VPRBS_VIDEO_LOCK));

	mask = MAX96724_DET(index);

	ret = regmap_read(priv->regmap, MAX96724_DE_DET, &val);
	if (ret)
		return ret;
	dev_info(priv->dev, "\tde_det: %u\n", !!(val & mask));

	ret = regmap_read(priv->regmap, MAX96724_HS_DET, &val);
	if (ret)
		return ret;
	dev_info(priv->dev, "\ths_det: %u\n", !!(val & mask));

	ret = regmap_read(priv->regmap, MAX96724_VS_DET, &val);
	if (ret)
		return ret;
	dev_info(priv->dev, "\tvs_det: %u\n", !!(val & mask));

	ret = regmap_read(priv->regmap, MAX96724_HS_POL, &val);
	if (ret)
		return ret;
	dev_info(priv->dev, "\ths_pol: %u\n", !!(val & mask));

	ret = regmap_read(priv->regmap, MAX96724_VS_POL, &val);
	if (ret)
		return ret;
	dev_info(priv->dev, "\tvs_pol: %u\n", !!(val & mask));

	return 0;
}

static int max96724_log_phy_status(struct max_des *des,
				   struct max_des_phy *phy)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = max96724_phy_id(des, phy);
	unsigned int val;
	unsigned int shift;
	int ret;

	max96724_health_check(des, "log-phy");

	dev_info(priv->dev,
		 "\tlog_phy_status: phy->index=%u enabled=%u resolved_hw_index=%u lanes=%u freq=%llu\n",
		 phy->index, phy->enabled, index,
		 phy->mipi.num_data_lanes,
		 (unsigned long long)phy->link_frequency);
	dev_info(priv->dev,
		 "\tlane_cfg: data_lanes={%u,%u,%u,%u} clock_lane=%u\n",
		 phy->mipi.data_lanes[0],
		 phy->mipi.data_lanes[1],
		 phy->mipi.data_lanes[2],
		 phy->mipi.data_lanes[3],
		 phy->mipi.clock_lane);

	ret = regmap_read(priv->regmap, MAX96724_MIPI_PHY25(index), &val);
	if (ret) {
		dev_warn(priv->dev,
			 "\tlog_phy_status: read MIPI_PHY25(index=%u, reg=0x%04x) failed: %d\n",
			 index, MAX96724_MIPI_PHY25(index), ret);
		return ret;
	}

	shift = 4 * (index % 2);

	dev_info(priv->dev,
		 "\tlog_phy_status: MIPI_PHY25(index=%u)=0x%08x\n",
		 index, val);
	dev_info(priv->dev, "\tcsi2_pkt_cnt: %u\n",
		 (val >> shift) & 0x0f);

	ret = regmap_read(priv->regmap, MAX96724_MIPI_PHY27(index), &val);
	if (ret) {
		dev_warn(priv->dev,
			 "\tlog_phy_status: read MIPI_PHY27(index=%u, reg=0x%04x) failed: %d\n",
			 index, MAX96724_MIPI_PHY27(index), ret);
		return ret;
	}

	dev_info(priv->dev,
		 "\tlog_phy_status: MIPI_PHY27(index=%u)=0x%08x\n",
		 index, val);
	dev_info(priv->dev, "\tphy_pkt_cnt: %u\n",
		 (val >> shift) & 0x0f);

	return 0;
}

static int max96724_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_UYVY8_1X16; /* [FIX] 統一回報 UYVY */
	return 0;
}

static int max96724_set_enable(struct max_des *des, bool enable)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int val = 0;
	int try, ret = 0;

	max96724_health_check(des, enable ? "set-enable-on" : "set-enable-off");

	dev_info(priv->dev,
		 "[EN] enable=%d, streaming_preconfigured=%d, pulse-update CSI_OUT_EN\n",
		 enable, des->streaming_preconfigured);

	for (try = 0; try < 10; try++) {
		if (try)
			msleep(20);

		if (enable) {
			/*
			 * 關鍵：
			 * preconfig 階段常常已經把 CSI_OUT_EN 留在 1。
			 * 若 stream-on 只是再寫一次 1，可能沒有真正產生
			 * 乾淨的 TX restart 邊緣。
			 *
			 * 這裡改成先 clear 再 set。
			 */
			ret = regmap_clear_bits(priv->regmap, MAX96724_BACKTOP12,
						MAX96724_BACKTOP12_CSI_OUT_EN);
			if (ret) {
				dev_warn(priv->dev,
					 "[EN] clear CSI_OUT_EN failed (try=%d): %d\n",
					 try + 1, ret);
				continue;
			}

			usleep_range(1000, 2000);

			ret = regmap_set_bits(priv->regmap, MAX96724_BACKTOP12,
					      MAX96724_BACKTOP12_CSI_OUT_EN);
			if (ret) {
				dev_warn(priv->dev,
					 "[EN] set CSI_OUT_EN failed (try=%d): %d\n",
					 try + 1, ret);
				continue;
			}
		} else {
			ret = regmap_clear_bits(priv->regmap, MAX96724_BACKTOP12,
						MAX96724_BACKTOP12_CSI_OUT_EN);
			if (ret) {
				dev_warn(priv->dev,
					 "[EN] clear CSI_OUT_EN failed (try=%d): %d\n",
					 try + 1, ret);
				continue;
			}
		}

		ret = regmap_read(priv->regmap, MAX96724_BACKTOP12, &val);
		if (ret) {
			dev_warn(priv->dev,
				 "[EN] BACKTOP12 readback failed after CSI_OUT_EN=%d (try=%d): %d\n",
				 enable, try + 1, ret);
			continue;
		}

		dev_info(priv->dev,
			 "[EN] BACKTOP12=0x%02x after CSI_OUT_EN=%d (try=%d)\n",
			 val, enable, try + 1);

		if (!!(val & MAX96724_BACKTOP12_CSI_OUT_EN) == !!enable)
			return 0;

		dev_warn(priv->dev,
			 "[EN] BACKTOP12 mismatch after CSI_OUT_EN=%d (try=%d, val=0x%02x)\n",
			 enable, try + 1, val);
	}

	dev_warn(priv->dev,
		 "[EN] CSI_OUT_EN=%d failed after retries: %d\n",
		 enable, ret);

	return ret ? ret : -EIO;
}

struct max_des_priv_mirror {
	struct max_des *des;
	struct device *dev;
	struct i2c_client *client;
	void *mux;
	void *pads;
	void *pocs;
	void *sources;
	u64 *streams_masks;
	struct notifier_block i2c_nb;
	struct v4l2_subdev sd;
};

static const unsigned int max96724_phys_configs_reg_val[] = {
	MAX96724_MIPI_PHY0_PHY_1X4A_2X2,
	MAX96724_MIPI_PHY0_PHY_2X4,

	MAX96724_MIPI_PHY0_PHY_4X2,
	MAX96724_MIPI_PHY0_PHY_1X4A_2X2,
	MAX96724_MIPI_PHY0_PHY_1X4B_2X2,
	MAX96724_MIPI_PHY0_PHY_2X4,
};

static const struct max_serdes_phys_config max96724_phys_configs[] = {
	/*
	 * PHY 1 can be in 4-lane mode (combining lanes of PHY 0 and PHY 1)
	 * but only use the data lanes of PHY0, while continuing to use the
	 * clock lane of PHY 1.
	 * Specifying clock-lanes as 5 turns on alternate clocking mode.
	 */
	{ { 2, 0, 2, 2 }, { MAX96724_PHY1_ALT_CLOCK, 0, 0, 0 } },
	{ { 2, 0, 4, 0 }, { MAX96724_PHY1_ALT_CLOCK, 0, 0, 0 } },

	/*
	 * When combining PHY 0 and PHY 1 to make them function in 4-lane mode,
	 * PHY 1 is the master PHY, but we use PHY 0 here to maintain
	 * compatibility.
	 */
	{ { 2, 2, 2, 2 } },
	{ { 4, 0, 2, 2 } },
	{ { 2, 2, 4, 0 } },
	{ { 4, 0, 4, 0 } },
};

static int max96724_init_tpg(struct max_des *des)
{
	const struct reg_sequence regs[] = {
		{ MAX96724_GRAD_INCR, MAX_SERDES_GRAD_INCR },
		REG_SEQUENCE_3_LE(MAX96724_CHKR_COLOR_A_L,
				  MAX_SERDES_CHECKER_COLOR_A),
		REG_SEQUENCE_3_LE(MAX96724_CHKR_COLOR_B_L,
				  MAX_SERDES_CHECKER_COLOR_B),
		{ MAX96724_CHKR_RPT_A, MAX_SERDES_CHECKER_SIZE },
		{ MAX96724_CHKR_RPT_B, MAX_SERDES_CHECKER_SIZE },
		{ MAX96724_CHKR_ALT, MAX_SERDES_CHECKER_SIZE },
	};
	struct max96724_priv *priv = des_to_priv(des);

	return regmap_multi_reg_write(priv->regmap, regs, ARRAY_SIZE(regs));
}

static int max96724_init(struct max_des *des)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int i;
	int ret;

	dev_info(priv->dev, "[INIT] max96724_init: phys_config=%u, num_pipes=%u\n",
		 des->phys_config, des->ops->num_pipes);

	/* Fast path: streaming registers were pre-written at probe time
	 * (when the ACPI regmap adapter was not yet mux-locked). At s_stream
	 * time the adapter is locked and all regmap ops return -121.
	 * Skip the hardware writes entirely - they are already in effect. */
	if (des->phys_config != 0 && des->streaming_preconfigured) {
		dev_info(priv->dev,
			 "[INIT] phys_config=%u pre-applied at probe, skipping hw writes.\n",
			 des->phys_config);
		return 0;
	}

	if (priv->info->set_pipe_tunnel_enable) {
		/* Active health-check: wait until the device is reachable before the
		 * MIPI_TX57 writes.  At s_stream time the I2C bus can be temporarily
		 * unavailable (EREMOTEIO / -121) due to I2C mux state transitions
		 * or MAX96724 soft-reset (triggered by select_links).
		 * Poll REG0 for up to 2000 ms (100 × 20 ms) before proceeding.
		 * The longer timeout is needed because select_links() issues a
		 * CTRL1_RESET_ONESHOT which can keep the chip busy for > 500 ms. */
		{
			unsigned int health_val = 0;
			int health_ret;
			int wait_try;

			for (wait_try = 0; wait_try < 100; wait_try++) {
				health_ret = regmap_read(priv->regmap, MAX96724_REG0, &health_val);
				if (!health_ret && health_val) {
					dev_info(priv->dev,
						 "[INIT] Device reachable after %d ms: reg0=0x%02x\n",
						 wait_try * 20, health_val);
					break;
				}
				dev_dbg(priv->dev,
					"[INIT] Waiting for device (try %d): reg0=0x%02x ret=%d\n",
					wait_try, health_val, health_ret);
				msleep(20);
			}
			if (health_ret || !health_val)
				dev_warn(priv->dev,
					 "[INIT] Device still unreachable after 2000 ms: reg0=0x%02x ret=%d\n",
					 health_val, health_ret);
		}

		for (i = 0; i < des->ops->num_pipes; i++) {
			int try;

			dev_info(priv->dev, "[INIT] Writing MIPI_TX57[%u] @ 0x%03x (DIS_AUTO_TUN_DET)\n",
				 i, 0x939 + i * 0x40);

			/* Use write-only to avoid the failing read in read-modify-write.
			 * The health-check above already waited for the device; only retry
			 * a few times here in case of residual transient errors. */
			for (try = 0, ret = -EREMOTEIO; try < 5 && ret; try++) {
				if (try)
					msleep(50);
				ret = regmap_write(priv->regmap, MAX96724_MIPI_TX57(i),
						   MAX96724_MIPI_TX57_DIS_AUTO_TUN_DET);
			}

			if (ret)
				dev_warn(priv->dev,
					 "[INIT] MIPI_TX57[%u] failed after retries (non-fatal, pipe may be inactive): %d\n",
					 i, ret);
		}
	}

	if (priv->info->supports_pipe_stream_autoselect) {
		int pipe_en_try;

		dev_info(priv->dev, "[INIT] Writing VIDEO_PIPE_EN (0xf4) STREAM_SEL_ALL\n");
		for (pipe_en_try = 0, ret = -EREMOTEIO;
		     pipe_en_try < 5 && ret; pipe_en_try++) {
			if (pipe_en_try)
				msleep(50);
			ret = regmap_set_bits(priv->regmap, MAX96724_VIDEO_PIPE_EN,
					      MAX96724_VIDEO_PIPE_EN_STREAM_SEL_ALL);
		}
		if (ret)
			dev_warn(priv->dev,
				 "[INIT] VIDEO_PIPE_EN failed after retries (non-fatal): %d\n",
				 ret);
		/* Do NOT return on failure; let the rest of init proceed. */
	}

	dev_info(priv->dev, "[INIT] Writing MIPI_PHY0 (0x8a0) PHY_CONFIG=0x%02x\n",
		 max96724_phys_configs_reg_val[des->phys_config]);
	/* Set PHY mode.  Retry up to 20×50ms (1 s total) for transient
	 * EREMOTEIO (-121) that can occur during s_stream while the I2C mux
	 * is still settling after opening the serializer channel. */
	{
		int phy0_try;

		for (phy0_try = 0, ret = -EREMOTEIO;
		     phy0_try < 20 && ret;
		     phy0_try++) {
			if (phy0_try)
				msleep(50);
			ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_PHY0,
						 MAX96724_MIPI_PHY0_PHY_CONFIG,
						 max96724_phys_configs_reg_val[des->phys_config]);
		}
		if (ret) {
			dev_err(priv->dev, "[INIT] MIPI_PHY0 FAILED after retries: %d\n", ret);
			return ret;
		}
	}

	dev_info(priv->dev, "[INIT] Writing TPG init regs\n");
	ret = max96724_init_tpg(des);
	if (ret)
		dev_warn(priv->dev, "[INIT] max96724_init_tpg failed (non-fatal): %d\n", ret);

	dev_info(priv->dev, "[INIT] max96724_init done\n");
	return 0;
}

static int max96724_preconfig_streaming_hw(struct max96724_priv *priv) 
{
	struct regmap *r = priv->regmap;
	struct max_des *des = &priv->des;
	struct max_des_phy *phy = &des->phys[0];
	unsigned int lane_map;
	const unsigned int phy_idx = 1;
	const unsigned int pipe_idx = 0;
	const unsigned int link_idx = 0;
	const unsigned int stream_id = 0;
	const unsigned int dpll_val = 15;
	int ret;

	lane_map = ((phy->mipi.data_lanes[0] - 1) << 0) |
		   ((phy->mipi.data_lanes[1] - 1) << 2) |
		   ((phy->mipi.data_lanes[2] - 1) << 4) |
		   ((phy->mipi.data_lanes[3] - 1) << 6);

	dev_info(priv->dev,
		 "[PRECONFIG] ==== Writing full streaming HW config at probe time ====\n");
	dev_info(priv->dev,
		 "[PRECONFIG] data_lanes={%u,%u,%u,%u} -> lane_map=0x%02x, phy_idx=%u pipe_idx=%u link_idx=%u stream_id=%u dpll=%u\n",
		 phy->mipi.data_lanes[0], phy->mipi.data_lanes[1],
		 phy->mipi.data_lanes[2], phy->mipi.data_lanes[3],
		 lane_map, phy_idx, pipe_idx, link_idx, stream_id, dpll_val);

	ret = regmap_update_bits(r, MAX96724_MIPI_TX10(phy_idx),
				 MAX96724_MIPI_TX10_CSI2_LANE_CNT,
				 FIELD_PREP(MAX96724_MIPI_TX10_CSI2_LANE_CNT, 3));
	if (ret)
		goto warn;

	ret = regmap_update_bits(r, MAX96724_MIPI_PHY3(phy_idx),
				 MAX96724_MIPI_PHY3_PHY_LANE_MAP_4,
				 lane_map);
	if (ret)
		goto warn;

	ret = regmap_update_bits(r, MAX96724_MIPI_PHY5(phy_idx),
				 MAX96724_MIPI_PHY5_PHY_POL_MAP_4_0_1 |
				 MAX96724_MIPI_PHY5_PHY_POL_MAP_4_2_3, 0);
	if (ret)
		goto warn;

	ret = regmap_write(r, MAX96724_MIPI_TX3(phy_idx),
			   MAX96724_MIPI_TX3_DESKEW_INIT_AUTO |
			   MAX96724_MIPI_TX3_DESKEW_INIT_8X32K);
	if (ret)
		goto warn;

	ret = regmap_write(r, MAX96724_MIPI_TX4(phy_idx),
			   MAX96724_MIPI_TX4_DESKEW_PER_AUTO |
			   MAX96724_MIPI_TX4_DESKEW_PER_2K);
	if (ret)
		goto warn;

	ret = regmap_clear_bits(r, MAX96724_DPLL_0(phy_idx),
				MAX96724_DPLL_0_CONFIG_SOFT_RST_N);
	if (ret)
		goto warn;

	ret = regmap_update_bits(r, MAX96724_BACKTOP22(phy_idx),
				 MAX96724_BACKTOP22_PHY_CSI_TX_DPLL,
				 FIELD_PREP(MAX96724_BACKTOP22_PHY_CSI_TX_DPLL,
					    dpll_val));
	if (ret)
		goto warn;

	ret = regmap_set_bits(r, MAX96724_BACKTOP22(phy_idx),
			      MAX96724_BACKTOP22_PHY_CSI_TX_DPLL_EN);
	if (ret)
		goto warn;

	ret = regmap_set_bits(r, MAX96724_DPLL_0(phy_idx),
			      MAX96724_DPLL_0_CONFIG_SOFT_RST_N);
	if (ret)
		goto warn;

	ret = regmap_set_bits(r, MAX96724_MIPI_PHY2,
			      MAX96724_MIPI_PHY2_PHY_STDB_N_4(phy_idx));
	if (ret)
		goto warn;

	ret = regmap_update_bits(r, MAX96724_VIDEO_PIPE_SEL(pipe_idx),
				 MAX96724_VIDEO_PIPE_SEL_LINK(pipe_idx) |
				 MAX96724_VIDEO_PIPE_SEL_STREAM(pipe_idx),
				 FIELD_PREP(MAX96724_VIDEO_PIPE_SEL_LINK(pipe_idx),
					    link_idx) |
				 FIELD_PREP(MAX96724_VIDEO_PIPE_SEL_STREAM(pipe_idx),
					    stream_id));
	if (ret)
		goto warn;

	ret = regmap_update_bits(r, MAX96724_MIPI_CTRL_SEL,
				 MAX96724_MIPI_CTRL_SEL_MASK(pipe_idx),
				 FIELD_PREP(MAX96724_MIPI_CTRL_SEL_MASK(pipe_idx),
					    phy_idx));
	if (ret)
		goto warn;

	ret = regmap_write(r, MAX96724_MIPI_TX13(pipe_idx, 0),
			   FIELD_PREP(MAX96724_MIPI_TX13_MAP_SRC_DT, 0x1E) |
			   FIELD_PREP(MAX96724_MIPI_TX13_MAP_SRC_VC, 0));
	if (ret)
		goto warn;

	ret = regmap_write(r, MAX96724_MIPI_TX14(pipe_idx, 0),
			   FIELD_PREP(MAX96724_MIPI_TX14_MAP_DST_DT, 0x1E) |
			   FIELD_PREP(MAX96724_MIPI_TX14_MAP_DST_VC, 0));
	if (ret)
		goto warn;

	ret = regmap_update_bits(r, MAX96724_MIPI_TX45(pipe_idx, 0),
				 MAX96724_MIPI_TX45_MAP_DPHY_DEST(0),
				 FIELD_PREP(MAX96724_MIPI_TX45_MAP_DPHY_DEST(0),
					    phy_idx));
	if (ret)
		goto warn;

	ret = regmap_write(r, MAX96724_MIPI_TX13(pipe_idx, 1),
			   FIELD_PREP(MAX96724_MIPI_TX13_MAP_SRC_DT, 0x00) |
			   FIELD_PREP(MAX96724_MIPI_TX13_MAP_SRC_VC, 0));
	if (ret)
		goto warn;

	ret = regmap_write(r, MAX96724_MIPI_TX14(pipe_idx, 1),
			   FIELD_PREP(MAX96724_MIPI_TX14_MAP_DST_DT, 0x00) |
			   FIELD_PREP(MAX96724_MIPI_TX14_MAP_DST_VC, 0));
	if (ret)
		goto warn;

	ret = regmap_update_bits(r, MAX96724_MIPI_TX45(pipe_idx, 1),
				 MAX96724_MIPI_TX45_MAP_DPHY_DEST(1),
				 FIELD_PREP(MAX96724_MIPI_TX45_MAP_DPHY_DEST(1),
					    phy_idx));
	if (ret)
		goto warn;

	ret = regmap_write(r, MAX96724_MIPI_TX13(pipe_idx, 2),
			   FIELD_PREP(MAX96724_MIPI_TX13_MAP_SRC_DT, 0x01) |
			   FIELD_PREP(MAX96724_MIPI_TX13_MAP_SRC_VC, 0));
	if (ret)
		goto warn;

	ret = regmap_write(r, MAX96724_MIPI_TX14(pipe_idx, 2),
			   FIELD_PREP(MAX96724_MIPI_TX14_MAP_DST_DT, 0x01) |
			   FIELD_PREP(MAX96724_MIPI_TX14_MAP_DST_VC, 0));
	if (ret)
		goto warn;

	ret = regmap_update_bits(r, MAX96724_MIPI_TX45(pipe_idx, 2),
				 MAX96724_MIPI_TX45_MAP_DPHY_DEST(2),
				 FIELD_PREP(MAX96724_MIPI_TX45_MAP_DPHY_DEST(2),
					    phy_idx));
	if (ret)
		goto warn;

	ret = regmap_write(r, MAX96724_MIPI_TX11(pipe_idx), 0x07);
	if (ret)
		goto warn;

	ret = regmap_write(r, MAX96724_MIPI_TX12(pipe_idx), 0x00);
	if (ret)
		goto warn;

	ret = regmap_set_bits(r, MAX96724_VIDEO_PIPE_EN,
			      MAX96724_VIDEO_PIPE_EN_MASK(pipe_idx));
	if (ret)
		goto warn;

	ret = regmap_update_bits(r, MAX96724_BACKTOP12, 0xF8, (16 << 3));
	if (ret)
		goto warn;

	ret = regmap_write(r, 0x040C, 0x00);
	if (ret)
		goto warn;

	ret = regmap_write(r, 0x040E, 0x1E);
	if (ret)
		goto warn;

	ret = regmap_write(r, 0x0456, 0x11);
	if (ret)
		goto warn;

	dev_info(priv->dev,
		 "[PRECONFIG] ==== Done - ALL streaming HW registers pre-written OK ====\n");
	return 0;

warn:
	dev_warn(priv->dev, "[PRECONFIG] Failed: %d\n", ret);
	return ret;
}

static int max96724_apply_probe_streaming_config(struct max96724_priv *priv)
{
	struct max_des *des = &priv->des;
	int ret;
	int csi_out_ret;

	/*
	 * 關鍵修正：
	 * 不要在 legacy child-bus magic write 之後再重跑一次
	 * max96724_init(phys_config=5)。
	 *
	 * 根據目前實測 log：
	 *   legacy write OK
	 *   -> 接著 max96724_init(phys_config=5)
	 *   -> REG0 立刻變成 -121
	 *
	 * 所以這裡只做最小 streaming probe-time config：
	 *   1) 設 PHY_CONFIG
	 *   2) 預開 CSI_OUT_EN
	 *   3) 寫完整 streaming HW preconfig
	 *
	 * 避免再次執行 max96724_init() 內那批會重設/重碰 pipe/tpg/phy 的動作。
	 */

	des->phys_config = 5; /* 2x4 lane streaming mode */

	dev_info(priv->dev,
		 "[INIT] apply_probe_streaming_config: phys_config=%u\n",
		 des->phys_config);

	/* 1) 只寫 MIPI_PHY0 的 PHY_CONFIG，不重跑整套 max96724_init() */
	ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_PHY0,
				 MAX96724_MIPI_PHY0_PHY_CONFIG,
				 max96724_phys_configs_reg_val[des->phys_config]);
	if (ret) {
		dev_warn(priv->dev,
			 "[INIT] probe-time MIPI_PHY0 PHY_CONFIG failed: %d\n",
			 ret);
		des->phys_config = 0;
		des->streaming_preconfigured = false;
		return ret;
	}

	/* 2) 預先打開 CSI_OUT_EN，讓 s_stream 時可以走 fast-path */
	csi_out_ret = regmap_set_bits(priv->regmap, MAX96724_BACKTOP12,
				      MAX96724_BACKTOP12_CSI_OUT_EN);
	if (csi_out_ret)
		dev_warn(priv->dev,
			 "[INIT] Pre-write BACKTOP12_CSI_OUT_EN failed: %d\n",
			 csi_out_ret);

	/* 3) 預寫 streaming 專用 HW config */
	ret = max96724_preconfig_streaming_hw(priv);
	if (!ret && !csi_out_ret) {
		des->streaming_preconfigured = true;
		dev_info(priv->dev,
			 "[INIT] Streaming config (phys_config=5) pre-applied at probe.\n");
		return 0;
	}

	des->streaming_preconfigured = false;
	dev_warn(priv->dev,
		 "[INIT] Streaming preconfig incomplete: pre_ret=%d csi_out_ret=%d\n",
		 ret, csi_out_ret);

	/*
	 * 保留 phys_config=5，因為我們仍然想維持目前 2x4 模型；
	 * 但 streaming_preconfigured=false，讓後面 s_stream 還有機會 fallback。
	 */
	return ret ? ret : csi_out_ret;
}

static int max96724_init_phy(struct max_des *des, struct max_des_phy *phy)
{
	struct max96724_priv *priv = des_to_priv(des);
	bool is_cphy = phy->bus_type == V4L2_MBUS_CSI2_CPHY;
	unsigned int num_data_lanes = phy->mipi.num_data_lanes;
	unsigned int dpll_freq = phy->link_frequency * 2;
	unsigned int num_hw_data_lanes;
	unsigned int index;
	unsigned int used_data_lanes = 0;
	unsigned int val, mask;
	unsigned int i;
	int ret;

	if (des->streaming_preconfigured) {
		dev_dbg(priv->dev,
			"init_phy: streaming already preconfigured, skip HW writes\n");
		return 0;
	}

	if (!num_data_lanes)
		return -EINVAL;

	index = max96724_phy_id(des, phy);
	num_hw_data_lanes = max_des_phy_hw_data_lanes(des, phy);

	if (!num_hw_data_lanes)
		return -EINVAL;

	dev_info(priv->dev,
		 "init_phy: phy->index=%u resolved_index=%u bus=%s lanes=%u hw_lanes=%u freq=%u\n",
		 phy->index, index,
		 is_cphy ? "cphy" : "dphy",
		 num_data_lanes, num_hw_data_lanes, dpll_freq);

	val = 0;
	for (i = 0; i < num_hw_data_lanes; i++) {
		unsigned int map;

		if (i < num_data_lanes)
			map = phy->mipi.data_lanes[i] - 1;
		else
			map = ffz(used_data_lanes);

		val |= map << (i * 2);
		used_data_lanes |= BIT(map);
	}

	if (num_hw_data_lanes == 4) {
		mask = MAX96724_MIPI_PHY3_PHY_LANE_MAP_4;
		ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_PHY3(index),
					 mask, val);
	} else {
		unsigned int shift = 4 * (index % 2);

		mask = MAX96724_MIPI_PHY3_PHY_LANE_MAP_2(index);
		ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_PHY3(index),
					 mask, val << shift);
	}
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_TX10(index),
				 MAX96724_MIPI_TX10_CSI2_LANE_CNT,
				 FIELD_PREP(MAX96724_MIPI_TX10_CSI2_LANE_CNT,
					    num_data_lanes - 1));
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX10(index),
				 MAX96724_MIPI_TX10_CSI2_CPHY_EN, is_cphy);
	if (ret)
		return ret;

	for (i = 0, val = 0; i < num_data_lanes; i++)
		if (phy->mipi.lane_polarities[i + 1])
			val |= BIT(i);

	if (num_hw_data_lanes == 4) {
		ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_PHY5(index),
					 MAX96724_MIPI_PHY5_PHY_POL_MAP_4_0_1 |
					 MAX96724_MIPI_PHY5_PHY_POL_MAP_4_2_3,
					 FIELD_PREP(MAX96724_MIPI_PHY5_PHY_POL_MAP_4_0_1,
						    val) |
					 FIELD_PREP(MAX96724_MIPI_PHY5_PHY_POL_MAP_4_2_3,
						    val >> 2));
		if (ret)
			return ret;

		ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_PHY5(index),
					 MAX96724_MIPI_PHY5_PHY_POL_MAP_4_CLK,
					 phy->mipi.lane_polarities[0]);
		if (ret)
			return ret;
	} else {
		unsigned int shift_pol = 3 * (index % 2);

		ret = regmap_update_bits(priv->regmap, MAX96724_MIPI_PHY5(index),
					 MAX96724_MIPI_PHY5_PHY_POL_MAP_2(index),
					 val << shift_pol);
		if (ret)
			return ret;

		ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_PHY5(index),
					 MAX96724_MIPI_PHY5_PHY_POL_MAP_2_CLK(index),
					 phy->mipi.lane_polarities[0]);
		if (ret)
			return ret;
	}

	if (!is_cphy && (dpll_freq >= 1500000000U || num_data_lanes == 4)) {
		ret = regmap_write(priv->regmap, MAX96724_MIPI_TX3(index),
				   MAX96724_MIPI_TX3_DESKEW_INIT_AUTO |
				   MAX96724_MIPI_TX3_DESKEW_INIT_8X32K);
		if (ret)
			return ret;

		ret = regmap_write(priv->regmap, MAX96724_MIPI_TX4(index),
				   MAX96724_MIPI_TX4_DESKEW_PER_AUTO |
				   MAX96724_MIPI_TX4_DESKEW_PER_2K);
		if (ret)
			return ret;
	} else {
		ret = regmap_write(priv->regmap, MAX96724_MIPI_TX3(index), 0x0);
		if (ret)
			return ret;

		ret = regmap_write(priv->regmap, MAX96724_MIPI_TX4(index), 0x0);
		if (ret)
			return ret;
	}

	if (is_cphy) {
		ret = regmap_write(priv->regmap, MAX96724_MIPI_PHY13,
				   MAX96724_MIPI_PHY13_T_T3_PREBEGIN_64X7);
		if (ret)
			return ret;

		ret = regmap_write(priv->regmap, MAX96724_MIPI_PHY14,
				   MAX96724_MIPI_PHY14_T_T3_PREP_55NS |
				   MAX96724_MIPI_PHY14_T_T3_POST_32X7);
		if (ret)
			return ret;
	}

	ret = regmap_clear_bits(priv->regmap, MAX96724_DPLL_0(index),
				MAX96724_DPLL_0_CONFIG_SOFT_RST_N);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, MAX96724_BACKTOP22(index),
				 MAX96724_BACKTOP22_PHY_CSI_TX_DPLL,
				 FIELD_PREP(MAX96724_BACKTOP22_PHY_CSI_TX_DPLL,
					    div_u64(dpll_freq, 100000000)));
	if (ret)
		return ret;

	ret = regmap_set_bits(priv->regmap, MAX96724_BACKTOP22(index),
			      MAX96724_BACKTOP22_PHY_CSI_TX_DPLL_EN);
	if (ret)
		return ret;

	ret = regmap_set_bits(priv->regmap, MAX96724_DPLL_0(index),
			      MAX96724_DPLL_0_CONFIG_SOFT_RST_N);
	if (ret)
		return ret;

	dev_info(priv->dev, "init_phy: done for resolved_index=%u\n", index);
	return 0;
}

static int max96724_set_phy_mode(struct max_des *des, struct max_des_phy *phy,
				 struct max_des_phy_mode *mode)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = max96724_phy_id(des, phy);
	int ret;

	/* Set alternate memory map modes. */
	ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX51(index),
				 MAX96724_MIPI_TX51_ALT_MEM_MAP_12,
				 mode->alt_mem_map12);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX51(index),
				 MAX96724_MIPI_TX51_ALT_MEM_MAP_8,
				 mode->alt_mem_map8);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX51(index),
				 MAX96724_MIPI_TX51_ALT_MEM_MAP_10,
				 mode->alt_mem_map10);
	if (ret)
		return ret;

	return regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX51(index),
				  MAX96724_MIPI_TX51_ALT2_MEM_MAP_8,
				  mode->alt2_mem_map8);
}

static int max96724_set_phy_enable(struct max_des *des, struct max_des_phy *phy,
				   bool enable)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index;
	unsigned int num_hw_data_lanes;
	unsigned int mask;

	if (enable && des->streaming_preconfigured)
		return 0;

	index = max96724_phy_id(des, phy);

	num_hw_data_lanes = max_des_phy_hw_data_lanes(des, phy);

	if (num_hw_data_lanes == 4)
		/* PHY 1 -> bits [1:0] */
		/* PHY 2 -> bits [3:2] */
		mask = MAX96724_MIPI_PHY2_PHY_STDB_N_4(index);
	else
		mask = MAX96724_MIPI_PHY2_PHY_STDB_N_2(index);

	return regmap_assign_bits(priv->regmap, MAX96724_MIPI_PHY2, mask, enable);
}

static int max96724_set_pipe_remap(struct max_des *des,
				   struct max_des_pipe *pipe,
				   unsigned int i,
				   struct max_des_remap *remap)
{
	struct max96724_priv *priv = des_to_priv(des);
	struct max_des_phy *phy = &des->phys[remap->phy];
	unsigned int phy_id = max96724_phy_id(des, phy);
	unsigned int index = pipe->index;
	unsigned int shift; /* [FIX] */
	int ret;

	ret = regmap_write(priv->regmap, MAX96724_MIPI_TX13(index, i),
			   FIELD_PREP(MAX96724_MIPI_TX13_MAP_SRC_DT,
				      remap->from_dt) |
			   FIELD_PREP(MAX96724_MIPI_TX13_MAP_SRC_VC,
				      remap->from_vc));
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, MAX96724_MIPI_TX14(index, i),
			   FIELD_PREP(MAX96724_MIPI_TX14_MAP_DST_DT,
				      remap->to_dt) |
			   FIELD_PREP(MAX96724_MIPI_TX14_MAP_DST_VC,
				      remap->to_vc));
	if (ret)
		return ret;

	/* [FIX] 計算位移量並手動 shift */
	shift = 2 * (i % 4);
	return regmap_update_bits(priv->regmap, MAX96724_MIPI_TX45(index, i),
				  MAX96724_MIPI_TX45_MAP_DPHY_DEST(i),
				  phy_id << shift);
}

static int max96724_set_pipe_remaps_enable(struct max_des *des,
					   struct max_des_pipe *pipe,
					   unsigned int mask)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;
	int ret;

	ret = regmap_write(priv->regmap, MAX96724_MIPI_TX11(index), mask);
	if (ret)
		return ret;

	return regmap_write(priv->regmap, MAX96724_MIPI_TX12(index), mask >> 8);
}

static int max96724_set_pipe_tunnel_phy(struct max_des *des,
					struct max_des_pipe *pipe,
					struct max_des_phy *phy)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int phy_index = max96724_phy_id(des, phy);

	return regmap_update_bits(priv->regmap, MAX96724_MIPI_TX57(pipe->index),
				  MAX96724_MIPI_TX57_TUN_DEST,
				  FIELD_PREP(MAX96724_MIPI_TX57_TUN_DEST,
					     phy_index));
}

static int max96724_set_pipe_phy(struct max_des *des, struct max_des_pipe *pipe,
				 struct max_des_phy *phy)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int phy_index;
	unsigned int shift; /* [FIX] */

	if (des->streaming_preconfigured)
		return 0;

	phy_index = max96724_phy_id(des, phy);
	shift = pipe->index * 2; /* [FIX] */

	return regmap_update_bits(priv->regmap, MAX96724_MIPI_CTRL_SEL,
				  MAX96724_MIPI_CTRL_SEL_MASK(pipe->index),
				  phy_index << shift); /* [FIX] */
}

static int max96724_set_pipe_enable(struct max_des *des, struct max_des_pipe *pipe,
				    bool enable)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;

	return regmap_assign_bits(priv->regmap, MAX96724_VIDEO_PIPE_EN,
				  MAX96724_VIDEO_PIPE_EN_MASK(index), enable);
}

static int max96724_set_pipe_stream_id(struct max_des *des, struct max_des_pipe *pipe,
				       unsigned int stream_id)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;
	unsigned int shift; /* [FIX] */

	if (des->streaming_preconfigured)
		return 0;

	shift = 4 * (index % 2); /* [FIX] stream 是從 bit 0 開始的 */

	return regmap_update_bits(priv->regmap, MAX96724_VIDEO_PIPE_SEL(index),
				  MAX96724_VIDEO_PIPE_SEL_STREAM(index),
				  stream_id << shift); /* [FIX] */
}

static int max96724_set_pipe_link(struct max_des *des, struct max_des_pipe *pipe,
				  struct max_des_link *link)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;
	unsigned int shift; /* [FIX] */

	if (des->streaming_preconfigured)
		return 0;

	shift = (4 * (index % 2)) + 2; /* [FIX] link 是從 bit 2 開始的，看 marco 宣告 */

	return regmap_update_bits(priv->regmap, MAX96724_VIDEO_PIPE_SEL(index),
				  MAX96724_VIDEO_PIPE_SEL_LINK(index),
				  link->index << shift); /* [FIX] */
}

static int max96724_set_pipe_mode(struct max_des *des,
				  struct max_des_pipe *pipe,
				  struct max_des_pipe_mode *mode)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = pipe->index;
	unsigned int reg, mask, mode_mask;
	int ret;

	/* Set 8bit double mode. */
	ret = regmap_assign_bits(priv->regmap, MAX96724_BACKTOP21(index),
				 MAX96724_BACKTOP21_BPP8DBL(index), mode->dbl8);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, MAX96724_BACKTOP24(index),
				 MAX96724_BACKTOP24_BPP8DBL_MODE(index),
				 mode->dbl8mode);
	if (ret)
		return ret;

	/* Set 10bit double mode. */
	if (index % 4 == 3) {
		reg = MAX96724_BACKTOP30(index);
		mask = MAX96724_BACKTOP30_BPP10DBL3;
		mode_mask = MAX96724_BACKTOP30_BPP10DBL3_MODE;
	} else if (index % 4 == 2) {
		reg = MAX96724_BACKTOP31(index);
		mask = MAX96724_BACKTOP31_BPP10DBL2;
		mode_mask = MAX96724_BACKTOP31_BPP10DBL2_MODE;
	} else if (index % 4 == 1) {
		reg = MAX96724_BACKTOP32(index);
		mask = MAX96724_BACKTOP32_BPP10DBL1;
		mode_mask = MAX96724_BACKTOP32_BPP10DBL1_MODE;
	} else {
		reg = MAX96724_BACKTOP32(index);
		mask = MAX96724_BACKTOP32_BPP10DBL0;
		mode_mask = MAX96724_BACKTOP32_BPP10DBL0_MODE;
	}

	ret = regmap_assign_bits(priv->regmap, reg, mask, mode->dbl10);
	if (ret)
		return ret;

	ret = regmap_assign_bits(priv->regmap, reg, mode_mask, mode->dbl10mode);
	if (ret)
		return ret;

	/* Set 12bit double mode. */
	return regmap_assign_bits(priv->regmap, MAX96724_BACKTOP32(index),
				  MAX96724_BACKTOP32_BPP12(index), mode->dbl12);
}

static int max96724_set_pipe_tunnel_enable(struct max_des *des,
					   struct max_des_pipe *pipe, bool enable)
{
	struct max96724_priv *priv = des_to_priv(des);

	return regmap_assign_bits(priv->regmap, MAX96724_MIPI_TX54(pipe->index),
				  MAX96724_MIPI_TX54_TUN_EN, enable);
}

static int max96724_select_links(struct max_des *des, unsigned int mask)
{
	struct max96724_priv *priv = des_to_priv(des);

	/* Only update REG6 link enable mask.
	 * Do NOT issue CTRL1_RESET_ONESHOT here: that soft-resets the chip
	 * every time the I2C mux channel is selected (bus 18/19), which
	 * causes repeated resets during s_stream and makes the device
	 * permanently unreachable during the health-check window. */
	return regmap_update_bits(priv->regmap, MAX96724_REG6, MAX96724_REG6_LINK_EN,
				  FIELD_PREP(MAX96724_REG6_LINK_EN, mask));
}

static int max96724_set_link_version(struct max_des *des,
				     struct max_des_link *link,
				     enum max_serdes_gmsl_version version)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int index = link->index;
	unsigned int val;
	unsigned int shift; /* [FIX] */

	if (version == MAX_SERDES_GMSL_2_6GBPS)
		val = MAX96724_REG26_RX_RATE_6GBPS;
	else
		val = MAX96724_REG26_RX_RATE_3GBPS;

	shift = 4 * (index % 2); /* 計算位移量 */

	return regmap_update_bits(priv->regmap, MAX96724_REG26(index),
				  MAX96724_REG26_RX_RATE_PHY(index),
				  val << shift); /* [FIX] 手動 shift 避開 FIELD_PREP 檢查 */
}

static int max96724_set_tpg_timings(struct max96724_priv *priv,
				    const struct max_serdes_tpg_timings *tm)
{
	const struct reg_sequence regs[] = {
		REG_SEQUENCE_3(MAX96724_VS_DLY_2, tm->vs_dly),
		REG_SEQUENCE_3(MAX96724_VS_HIGH_2, tm->vs_high),
		REG_SEQUENCE_3(MAX96724_VS_LOW_2, tm->vs_low),
		REG_SEQUENCE_3(MAX96724_V2H_2, tm->v2h),
		REG_SEQUENCE_2(MAX96724_HS_HIGH_1, tm->hs_high),
		REG_SEQUENCE_2(MAX96724_HS_LOW_1, tm->hs_low),
		REG_SEQUENCE_2(MAX96724_HS_CNT_1, tm->hs_cnt),
		REG_SEQUENCE_3(MAX96724_V2D_2, tm->v2d),
		REG_SEQUENCE_2(MAX96724_DE_HIGH_1, tm->de_high),
		REG_SEQUENCE_2(MAX96724_DE_LOW_1, tm->de_low),
		REG_SEQUENCE_2(MAX96724_DE_CNT_1, tm->de_cnt),
	};
	int ret;

	ret = regmap_multi_reg_write(priv->regmap, regs, ARRAY_SIZE(regs));
	if (ret)
		return ret;

	return regmap_write(priv->regmap, MAX96724_PATGEN_0,
			    FIELD_PREP(MAX96724_PATGEN_0_VTG_MODE,
				       MAX96724_PATGEN_0_VTG_MODE_FREE_RUNNING) |
			    FIELD_PREP(MAX96724_PATGEN_0_DE_INV, tm->de_inv) |
			    FIELD_PREP(MAX96724_PATGEN_0_HS_INV, tm->hs_inv) |
			    FIELD_PREP(MAX96724_PATGEN_0_VS_INV, tm->vs_inv) |
			    FIELD_PREP(MAX96724_PATGEN_0_GEN_DE, tm->gen_de) |
			    FIELD_PREP(MAX96724_PATGEN_0_GEN_HS, tm->gen_hs) |
			    FIELD_PREP(MAX96724_PATGEN_0_GEN_VS, tm->gen_vs));
}

static int max96724_set_tpg_clk(struct max96724_priv *priv, u32 clock)
{
	bool patgen_clk_src = 0;
	u8 pclk_src;
	int ret;

	switch (clock) {
	case 25000000:
		pclk_src = MAX96724_DEBUG_EXTRA_PCLK_SRC_25MHZ;
		break;
	case 75000000:
		pclk_src = MAX96724_DEBUG_EXTRA_PCLK_SRC_75MHZ;
		break;
	case 150000000:
		pclk_src = MAX96724_DEBUG_EXTRA_PCLK_SRC_USE_PIPE;
		patgen_clk_src = MAX96724_VPRBS_PATGEN_CLK_SRC_150MHZ;
		break;
	case 375000000:
		pclk_src = MAX96724_DEBUG_EXTRA_PCLK_SRC_USE_PIPE;
		patgen_clk_src = MAX96724_VPRBS_PATGEN_CLK_SRC_375MHZ;
		break;
	case 0:
		return 0;
	default:
		return -EINVAL;
	}

	/*
	 * TPG data is always injected on link 0, which is always routed to
	 * pipe 0.
	 */
	ret = regmap_update_bits(priv->regmap, MAX96724_VPRBS(0),
				 MAX96724_VPRBS_PATGEN_CLK_SRC,
				 FIELD_PREP(MAX96724_VPRBS_PATGEN_CLK_SRC,
					    patgen_clk_src));
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, MAX96724_DEBUG_EXTRA,
				  MAX96724_DEBUG_EXTRA_PCLK_SRC,
				  FIELD_PREP(MAX96724_DEBUG_EXTRA_PCLK_SRC,
					     pclk_src));
}

static int max96724_set_tpg_mode(struct max96724_priv *priv, bool enable)
{
	unsigned int patgen_mode;

	switch (priv->des.tpg_pattern) {
	case MAX_SERDES_TPG_PATTERN_GRADIENT:
		patgen_mode = MAX96724_PATGEN_1_PATGEN_MODE_GRADIENT;
		break;
	case MAX_SERDES_TPG_PATTERN_CHECKERBOARD:
		patgen_mode = MAX96724_PATGEN_1_PATGEN_MODE_CHECKER;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(priv->regmap, MAX96724_PATGEN_1,
				  MAX96724_PATGEN_1_PATGEN_MODE,
				  FIELD_PREP(MAX96724_PATGEN_1_PATGEN_MODE,
					     enable ? patgen_mode
						    : MAX96724_PATGEN_1_PATGEN_MODE_DISABLED));
}

static int max96724_set_tpg(struct max_des *des,
			    const struct max_serdes_tpg_entry *entry)
{
	struct max96724_priv *priv = des_to_priv(des);
	struct max_serdes_tpg_timings timings = { 0 };
	int ret;
	
	dev_info(priv->dev, "TPG entry: %ux%u interval=%u/%u code=0x%x dt=0x%x bpp=%u\n",
         entry ? entry->width : 0, entry ? entry->height : 0,
         entry ? entry->interval.numerator : 0, entry ? entry->interval.denominator : 0,
         entry ? entry->code : 0, entry ? entry->dt : 0, entry ? entry->bpp : 0);

	if (!entry) {
		ret = max96724_set_tpg_mode(priv, NULL);
		if (ret) return ret;
		return regmap_clear_bits(priv->regmap, MAX96724_MIPI_PHY0,
					 MAX96724_MIPI_PHY0_FORCE_CSI_OUT_EN);
	}

	ret = max_serdes_get_tpg_timings(entry, &timings);
	if (ret) return ret;

	ret = max96724_set_tpg_timings(priv, &timings);
	if (ret) return ret;

	ret = max96724_set_tpg_clk(priv, timings.clock);
	if (ret) return ret;

	ret = max96724_set_tpg_mode(priv, true);
	if (ret) return ret;

	/* ========================================================== */
	/* [HACK] The Holy Trinity: Port A + 0xB4 Swap + Software Override */
	/* ========================================================== */
	dev_info(priv->dev, "[HACK] 0xB4 Swap + 16bpp UYVY Override...\n");
	regmap_write(priv->regmap, 0x08A0, 0x84); /* 2x4 Mode (Port A) */
	regmap_write(priv->regmap, 0x08CA, 0x01); /* Pipe 0 -> Ctrl 1 */

	/* 關閉所有舊版的 Mapping 映射 */
	regmap_write(priv->regmap, 0x090B, 0x00);
	regmap_write(priv->regmap, 0x090C, 0x00);
	regmap_write(priv->regmap, 0x092D, 0x00);

	/* 實體層線路校正：對調 D2/D3 (0xB4) 來迎合 AAEON 電路板 */
	regmap_write(priv->regmap, 0x08A3, 0xE4); 
	regmap_write(priv->regmap, 0x08A5, 0x00); 

	/* 協議層校正：強制將 24bpp 截斷為 16bpp，並標上 UYVY (0x1E)
	 * 注意: 0x040B 就是 MAX96724_BACKTOP12，包含 CSI_OUT_EN (bit 1)。
	 * 必須使用 regmap_update_bits 保留這一個 bit，否則會關閉影像輸出!
	 */
	regmap_update_bits(priv->regmap, MAX96724_BACKTOP12,
		   0xF8,
		   (16 << 3));
	regmap_write(priv->regmap, 0x040C, 0x00); /* soft_vc_0 = 0 */
	regmap_write(priv->regmap, 0x040E, 0x1E); /* soft_dt_0 = 0x1E (UYVY) */
	regmap_write(priv->regmap, 0x0456, 0x11); /* 啟用 Override */

	return 0;
}

static const struct max_serdes_tpg_entry max96724_tpg_entries[] = {
	MAX_TPG_ENTRY_640X480P60_RGB888,      
	{ 1920, 1080, { 1, 30 }, MEDIA_BUS_FMT_RGB888_1X24, 0x24, 24 }, /* entry[1] */
	MAX_TPG_ENTRY_1920X1080P60_RGB888,    
};

static const struct max_des_ops max96724_ops = {
	.num_phys = 4,
	.num_links = 4,
	.num_remaps_per_pipe = 16,
	.phys_configs = {
		.num_configs = ARRAY_SIZE(max96724_phys_configs),
		.configs = max96724_phys_configs,
	},
	.tpg_entries = {
		.num_entries = ARRAY_SIZE(max96724_tpg_entries),
		.entries = max96724_tpg_entries,
	},
	.tpg_mode = MAX_SERDES_GMSL_PIXEL_MODE,
	.tpg_patterns = BIT(MAX_SERDES_TPG_PATTERN_CHECKERBOARD) |
			BIT(MAX_SERDES_TPG_PATTERN_GRADIENT),
	// .use_atr = true,
	.use_atr = false,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.reg_read = max96724_reg_read,
	.reg_write = max96724_reg_write,
#endif
	.log_pipe_status = max96724_log_pipe_status,
	.log_phy_status = max96724_log_phy_status,
	.set_enable = max96724_set_enable,
	.init = max96724_init,
	.init_phy = max96724_init_phy,
	.set_phy_mode = max96724_set_phy_mode,
	.set_phy_enable = max96724_set_phy_enable,
	.set_pipe_stream_id = max96724_set_pipe_stream_id,
	.set_pipe_link = max96724_set_pipe_link,
	.set_pipe_enable = max96724_set_pipe_enable,
	.set_pipe_remap = max96724_set_pipe_remap,
	.set_pipe_remaps_enable = max96724_set_pipe_remaps_enable,
	.set_pipe_mode = max96724_set_pipe_mode,
	.set_tpg = max96724_set_tpg,
	.select_links = max96724_select_links,
	.set_link_version = max96724_set_link_version,
};

static const struct max96724_chip_info max96724_info = {
	.versions = BIT(MAX_SERDES_GMSL_2_3GBPS) |
		    BIT(MAX_SERDES_GMSL_2_6GBPS),
	.modes = BIT(MAX_SERDES_GMSL_PIXEL_MODE) |
		 BIT(MAX_SERDES_GMSL_TUNNEL_MODE),
	.set_pipe_tunnel_enable = max96724_set_pipe_tunnel_enable,
	.set_pipe_phy = max96724_set_pipe_phy,
	.set_pipe_tunnel_phy = max96724_set_pipe_tunnel_phy,
	.supports_pipe_stream_autoselect = true,
	.num_pipes = 4,
};

static const struct max96724_chip_info max96724f_info = {
	.versions = BIT(MAX_SERDES_GMSL_2_3GBPS),
	.modes = BIT(MAX_SERDES_GMSL_PIXEL_MODE) |
		 BIT(MAX_SERDES_GMSL_TUNNEL_MODE),
	.set_pipe_tunnel_enable = max96724_set_pipe_tunnel_enable,
	.set_pipe_phy = max96724_set_pipe_phy,
	.set_pipe_tunnel_phy = max96724_set_pipe_tunnel_phy,
	.supports_pipe_stream_autoselect = true,
	.num_pipes = 4,
};

static const struct max96724_chip_info max96712_info = {
	.versions = BIT(MAX_SERDES_GMSL_2_3GBPS) |
		    BIT(MAX_SERDES_GMSL_2_6GBPS),
	.modes = BIT(MAX_SERDES_GMSL_PIXEL_MODE),
	.num_pipes = 8,
};

static int max96724_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max96724_priv *priv;
	struct max_des_ops *ops;
	int ret;

	/* 這一輪正式打開，驗證 probe-time preconfig 能不能讓 des output 起來 */
	const bool test_probe_preconfig = true;

	dev_info(dev, "MAX96724: Starting Power-On Sequence...now13\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ops = devm_kzalloc(dev, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return -ENOMEM;

	priv->info = device_get_match_data(dev);
	if (!priv->info)
		priv->info = &max96724_info;

	priv->dev = dev;
	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->regmap = devm_regmap_init_i2c(client, &max96724_i2c_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	*ops = max96724_ops;
	ops->versions = priv->info->versions;
	ops->modes = priv->info->modes;
	ops->num_pipes = priv->info->num_pipes;
	ops->set_pipe_tunnel_enable = priv->info->set_pipe_tunnel_enable;
	ops->set_pipe_phy = priv->info->set_pipe_phy;
	ops->set_pipe_tunnel_phy = priv->info->set_pipe_tunnel_phy;
	priv->des.ops = ops;

	dev_info(dev, "  -> Attempting I2C communication...\n");
	ret = max96724_reset(priv);
	if (ret) {
		dev_err(dev, "MAX96724 failed to reset/communicate! (ret=%d)\n", ret);
		return ret;
	}

	dev_info(dev, "MAX96724: Power-On Successful! Probing core...\n");

	ret = max_des_probe(client, &priv->des);
	if (ret)
		return ret;

	/*
	 * 這一輪正式測試：
	 * 在 probe 階段先把 streaming HW routing/config 全部預寫進去，
	 * 看能不能讓 des output path 真正開始吐 CSI-2 packet。
	 */
	if (test_probe_preconfig) {
		int pre_ret;

		/* 保守直通 lane map */
		priv->des.phys[0].mipi.num_data_lanes = 4;
		priv->des.phys[0].mipi.data_lanes[0] = 1;
		priv->des.phys[0].mipi.data_lanes[1] = 2;
		priv->des.phys[0].mipi.data_lanes[2] = 3;
		priv->des.phys[0].mipi.data_lanes[3] = 4;
		priv->des.phys[0].mipi.clock_lane = 0;

		pre_ret = max96724_apply_probe_streaming_config(priv);
		dev_info(dev,
			 "MAX96724: test_probe_preconfig=%d result=%d\n",
			 test_probe_preconfig, pre_ret);
	}

	dev_info(dev, "MAX96724 probe OK\n");
	return 0;
}

static void max96724_remove(struct i2c_client *client)
{
	struct max96724_priv *priv = i2c_get_clientdata(client);

	max_des_remove(&priv->des);

	gpiod_set_value_cansleep(priv->gpiod_enable, 0);

	if (priv->gpiod_poc)
		gpiod_set_value_cansleep(priv->gpiod_poc, 0);
}

static const struct acpi_device_id max96724_acpi_ids[] = {
    { "MAX96724", (kernel_ulong_t)&max96724_info },  
    { "APR96724", (kernel_ulong_t)&max96724_info },  /* Add approgmsl compatible ID */
    { "INTC113C", (kernel_ulong_t)&max96724_info },  
    { }
};
MODULE_DEVICE_TABLE(acpi, max96724_acpi_ids);

static const struct of_device_id max96724_of_table[] = {
	{ .compatible = "maxim,max96712", .data = &max96712_info },
	{ .compatible = "maxim,max96724", .data = &max96724_info },
	{ .compatible = "maxim,max96724f", .data = &max96724f_info },
	{ .compatible = "maxim,max96724r", .data = &max96724f_info },
	{ },
};
MODULE_DEVICE_TABLE(of, max96724_of_table);

static struct i2c_driver max96724_i2c_driver = {
    .driver = {
        .name = "max96724",
#ifdef CONFIG_ACPI
        .acpi_match_table = ACPI_PTR(max96724_acpi_ids), 
#else
		.of_match_table = max96724_of_table, 
#endif
    },
    .probe = max96724_probe,
    .remove = max96724_remove,
};

module_i2c_driver(max96724_i2c_driver);
MODULE_SOFTDEP("pre: intel_ipu6_isys");

MODULE_IMPORT_NS("MAX_SERDES");
MODULE_DESCRIPTION("Maxim MAX96724 Quad GMSL2 Deserializer Driver");
MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_LICENSE("GPL");

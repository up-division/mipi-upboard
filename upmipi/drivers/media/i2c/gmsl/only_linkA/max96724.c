// SPDX-License-Identifier: GPL-2.0
#include <linux/bitops.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/acpi.h>

#include "max_des.h"

#define DES_RATE_MODE_1P5G  0
#define DES_RATE_MODE_1P0G  1

#ifndef DES_TEST_RATE_MODE
#define DES_TEST_RATE_MODE DES_RATE_MODE_1P5G
#endif

#if DES_TEST_RATE_MODE == DES_RATE_MODE_1P0G
#define DES_TEST_DPLL_REG        0x0A
#define DES_TEST_LINK_FREQ_HZ    500000000ULL
#define DES_TEST_RATE_NAME       "1.0Gbps/lane, link_freq=500MHz"
#elif DES_TEST_RATE_MODE == DES_RATE_MODE_1P5G
#define DES_TEST_DPLL_REG        0x2F
#define DES_TEST_LINK_FREQ_HZ    750000000ULL
#define DES_TEST_RATE_NAME       "1.5Gbps/lane, link_freq=750MHz"
#else
#error "Unsupported DES_TEST_RATE_MODE"
#endif

struct max96724_chip_info {
	unsigned int num_phys;
	unsigned int num_pipes;
	unsigned int num_links;
	unsigned int num_remaps_per_pipe;
};

struct max96724_priv {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct max_des des;
	const struct max96724_chip_info *info;
	bool des_hw_inited;
};

static inline struct max96724_priv *des_to_priv(struct max_des *des)
{
	return container_of(des, struct max96724_priv, des);
}

static const struct regmap_config max96724_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x1fff,
};

static const struct max96724_chip_info max96724_info = {
	.num_phys = 4,
	.num_pipes = 4,
	.num_links = 4,
	.num_remaps_per_pipe = 16,
};

static int max96724_write(struct max96724_priv *priv, u16 reg, u8 val)
{
	int ret = regmap_write(priv->regmap, reg, val);

	if (ret)
		dev_err(priv->dev, "write reg 0x%04x = 0x%02x failed: %d\n",
			reg, val, ret);
	return ret;
}

static int max96724_update_bits(struct max96724_priv *priv, u16 reg, u8 mask, u8 val)
{
	int ret = regmap_update_bits(priv->regmap, reg, mask, val);

	if (ret)
		dev_err(priv->dev,
			"update reg 0x%04x mask 0x%02x val 0x%02x failed: %d\n",
			reg, mask, val, ret);
	return ret;
}

static int max96724_reset(struct max96724_priv *priv)
{
	/*
	 * 這裡先保守不做 software reset toggle，
	 * 避免和板上的 reset/power sequencing 打架。
	 * 若你之後確認需要，可再補全域 reset。
	 */
	usleep_range(1000, 2000);
	return 0;
}

/* --------------------------------------------------------- */
/* DES-only init sequence copied from vendor i2c script      */
/* --------------------------------------------------------- */

static int max96724_des_init_script_pre(struct max96724_priv *priv)
{
	int ret;

	/* Pipe0 BPP override = 0 */
	ret = max96724_write(priv, 0x040B, 0x00);
	if (ret) return ret;

	/* Link A only, GMSL2 */
	ret = max96724_write(priv, 0x0006, 0xF1);
	if (ret) return ret;

	/* Link A-D 3Gbps forward / 187.5Mbps reverse */
	ret = max96724_write(priv, 0x0010, 0x11);
	if (ret) return ret;
	ret = max96724_write(priv, 0x0011, 0x11);
	if (ret) return ret;

	/* one-shot reset link A only */
	ret = max96724_write(priv, 0x0018, 0x01);
	if (ret) return ret;

	msleep(300);

	/*
	 * pipe source select:
	 * Link A -> pipe0, stream id = 0
	 *
	 * bits[3:2] = link id
	 * bits[1:0] = stream id
	 * => link0 + stream0 = 0x00
	 */
	ret = max96724_write(priv, 0x00F0, 0x02);
	if (ret) return ret;
	ret = max96724_write(priv, 0x00F1, 0x00);
	if (ret) return ret;

	/* enable pipe0 only */
	ret = max96724_write(priv, 0x00F4, 0x01);
	if (ret) return ret;

	/* efficiency / LIM_HEART for pipe0 only */
	ret = max96724_write(priv, 0x0106, 0x0A);
	if (ret) return ret;

	/* ---------------- pipe0 only ---------------- */
	ret = max96724_write(priv, 0x090B, 0x07); if (ret) return ret;
	ret = max96724_write(priv, 0x092D, 0x15); if (ret) return ret;
	ret = max96724_write(priv, 0x090D, 0x1E); if (ret) return ret;
	ret = max96724_write(priv, 0x090E, 0x1E); if (ret) return ret;
	ret = max96724_write(priv, 0x090F, 0x00); if (ret) return ret;
	ret = max96724_write(priv, 0x0910, 0x00); if (ret) return ret;
	ret = max96724_write(priv, 0x0911, 0x01); if (ret) return ret;
	ret = max96724_write(priv, 0x0912, 0x01); if (ret) return ret;

	msleep(300);

	/*
	 * MIPI PHY setting:
	 * 改成 4x2 mode（四個獨立 2-lane PHY）
	 * bit0 = 1
	 */
	ret = max96724_write(priv, 0x08A0, 0x01); if (ret) return ret;

	/*
	 * Lane map:
	 * 4x2 mode 下，每個 PHY 都是 D0/D1
	 * 0x08A3 controls PHY0/PHY1
	 * - PHY1 lane1 = D1
	 * - PHY1 lane0 = D0
	 * - PHY0 lane1 = D1
	 * - PHY0 lane0 = D0
	 * => 0x44
	 */
	ret = max96724_write(priv, 0x08A3, 0x44); if (ret) return ret;

	/*
	 * 0x08A4 controls PHY2/PHY3
	 * 同樣保守設成 D0/D1
	 */
	ret = max96724_write(priv, 0x08A4, 0x44); if (ret) return ret;

	/*
	 * lane count:
	 * 2-lane => field = 1 => 0x40
	 *
	 * 保守起見四個 controller 都設，
	 * 但實際 route 只會用到 controller1 / PHY1。
	 */
	ret = max96724_write(priv, 0x090A, 0x40); if (ret) return ret;
	ret = max96724_write(priv, 0x094A, 0x40); if (ret) return ret;
	ret = max96724_write(priv, 0x098A, 0x40); if (ret) return ret;
	ret = max96724_write(priv, 0x09CA, 0x40); if (ret) return ret;

	/*
	 * PHY enable:
	 * 只 enable PHY1
	 * bit5 = PHY1
	 */
	ret = max96724_write(priv, 0x08A2, 0x20); if (ret) return ret;

	/*
	 * DPLL:
	 * 目前先維持你既有巨集切換
	 */
	ret = max96724_write(priv, 0x0415, DES_TEST_DPLL_REG); if (ret) return ret;
	ret = max96724_write(priv, 0x0418, DES_TEST_DPLL_REG); if (ret) return ret;
	ret = max96724_write(priv, 0x041B, DES_TEST_DPLL_REG); if (ret) return ret;
	ret = max96724_write(priv, 0x041E, DES_TEST_DPLL_REG); if (ret) return ret;

	dev_info(priv->dev,
		 "[DES] rate preset = %s, fixed route = link0/pipe0/stream0 -> PHY1 2-lane\n",
		 DES_TEST_RATE_NAME);

	return 0;
}

static int max96724_des_init_script_post(struct max96724_priv *priv)
{
	int ret;

	/*
	 * 腳本最後的 DES-only 收尾：
	 * 這些原本出現在 per-link serializer 改地址之後。
	 * 既然你要求把 DES 端都補完，這裡也補上。
	 */
	ret = max96724_write(priv, 0x0006, 0xFF);
	if (ret) return ret;

	ret = max96724_write(priv, 0x0018, 0x0F);
	if (ret) return ret;

	ret = max96724_write(priv, 0x040B, 0x02);
	if (ret) return ret;

	ret = max96724_write(priv, 0x08A0, 0x84);
	if (ret) return ret;

	return 0;
}

/* --------------------------------------------------------- */
/* max_des ops                                               */
/* --------------------------------------------------------- */

static void max96724_dump_regs(struct max96724_priv *priv, const char *tag)
{
	static const u16 regs[] = {
		0x0006, 0x0010, 0x0011, 0x0018,
		0x00F0, 0x00F1, 0x00F4,
		0x0106,
		0x040B,
		0x08A0, 0x08A2, 0x08A3, 0x08A4,
		0x090A, 0x090B, 0x090C, 0x090D, 0x090E, 0x090F, 0x0910, 0x0911, 0x0912, 0x092D,
		0x0415, 0x0418, 0x041B, 0x041E,
		0x001A, /* linkA lock */
	};
	unsigned int val;
	int i;

	dev_info(priv->dev, "[DUMP][%s] ===== register dump start =====\n", tag);
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		if (!regmap_read(priv->regmap, regs[i], &val))
			dev_info(priv->dev, "[DUMP][%s] reg 0x%04x = 0x%02x\n",
				 tag, regs[i], val);
		else
			dev_warn(priv->dev, "[DUMP][%s] reg 0x%04x read failed\n",
				 tag, regs[i]);
	}
	dev_info(priv->dev, "[DUMP][%s] ===== register dump end =====\n", tag);
}

static void max96724_des_sync_sw_state_from_script(struct max_des *des)
{
	struct max96724_priv *priv = des_to_priv(des);
	struct max_des_pipe *pipe0 = &des->pipes[0];
	struct max_des_phy *phy1 = &des->phys[1];
	unsigned int i;

	des->mode = MAX_SERDES_GMSL_PIXEL_MODE;

	for (i = 0; i < des->ops->num_pipes; i++) {
		des->pipes[i].enabled = false;
		des->pipes[i].link_id = i;
		des->pipes[i].stream_id = 2;
		des->pipes[i].phy_id = 0;
	}

	for (i = 0; i < des->ops->num_phys; i++) {
		des->phys[i].enabled = false;
		des->phys[i].bus_type = V4L2_MBUS_CSI2_DPHY;
		des->phys[i].mipi.clock_lane = 0;
		des->phys[i].link_frequency = DES_TEST_LINK_FREQ_HZ;
		memset(des->phys[i].mipi.data_lanes, 0,
		       sizeof(des->phys[i].mipi.data_lanes));
		des->phys[i].mipi.num_data_lanes = 0;
	}

	/*
	 * 固定單路:
	 * link0 -> pipe0 -> stream0 -> PHY1(2-lane)
	 */
	phy1->enabled = true;
	phy1->mipi.num_data_lanes = 2;
	phy1->mipi.clock_lane = 0;
	phy1->mipi.data_lanes[0] = 1;
	phy1->mipi.data_lanes[1] = 2;
	phy1->link_frequency = DES_TEST_LINK_FREQ_HZ;

	pipe0->enabled = true;
	pipe0->link_id = 0;
	pipe0->stream_id = 0;
	pipe0->phy_id = 1;

	dev_info(priv->dev,
		 "[SYNC] SW state <- fixed single-camera route: link0 -> pipe0 -> stream0 -> PHY1 (2-lane), %s\n",
		 DES_TEST_RATE_NAME);
}

static int max96724_des_init(struct max_des *des)
{
	struct max96724_priv *priv = des_to_priv(des);
	int ret;

	if (priv->des_hw_inited)
		return 0;

	dev_info(priv->dev, "MAX96724: run DES init sequence from vendor script (pre only)\n");

	ret = max96724_des_init_script_pre(priv);
	if (ret)
		return ret;

	max96724_des_sync_sw_state_from_script(des);
	max96724_dump_regs(priv, "after_init_pre");

	priv->des_hw_inited = true;
	return 0;
}

static int max96724_des_stream_prepare(struct max_des *des)
{
	struct max96724_priv *priv = des_to_priv(des);
	int ret;

	dev_info(priv->dev, "[DES] stream_prepare: re-apply fixed single-camera 2-lane PHY1 setup\n");

	/* Link A only */
	ret = max96724_write(priv, 0x0006, 0xF1);
	if (ret)
		return ret;

	/* pipe0: link0 + stream0 */
	ret = max96724_write(priv, 0x00F0, 0x02);
	if (ret)
		return ret;

	/* pipe0 BPP override = 2 */
	ret = max96724_write(priv, 0x040B, 0x02);
	if (ret)
		return ret;

	/* 4x2 mode + force clocks running */
	ret = max96724_write(priv, 0x08A0, 0x81);
	if (ret)
		return ret;

	/* only PHY1 enabled */
	ret = max96724_write(priv, 0x08A2, 0x20);
	if (ret)
		return ret;

	/* controller lane count = 2 lanes */
	ret = max96724_write(priv, 0x094A, 0x40);
	if (ret)
		return ret;

	max96724_des_sync_sw_state_from_script(des);
	max96724_dump_regs(priv, "after_stream_prepare");

	return 0;
}

static int max96724_des_init_phy(struct max_des *des, struct max_des_phy *phy)
{
	/* 腳本的 PHY 初始化已在 max96724_des_init() 全做完 */
	return 0;
}

static int max96724_des_set_phy_enable(struct max_des *des,
				       struct max_des_phy *phy, bool enable)
{
	struct max96724_priv *priv = des_to_priv(des);
	u8 mask;

	if (phy->index >= 4)
		return -EINVAL;

	mask = BIT(phy->index + 4);

	return max96724_update_bits(priv, 0x08A2, mask, enable ? mask : 0);
}

static int max96724_des_set_pipe_link(struct max_des *des,
				      struct max_des_pipe *pipe,
				      struct max_des_link *link)
{
	struct max96724_priv *priv = des_to_priv(des);
	u16 reg;
	u8 shift, mask, val;
	unsigned int stream_id;

	if (pipe->index >= 4 || link->index >= 4)
		return -EINVAL;

	reg = 0x00F0 + (pipe->index / 2);
	shift = (pipe->index % 2) ? 4 : 0;
	mask = 0x0F << shift;

	stream_id = pipe->stream_id & 0x3;
	val = (((link->index & 0x3) << 2) | stream_id) << shift;

	return max96724_update_bits(priv, reg, mask, val);
}

static int max96724_des_set_pipe_stream_id(struct max_des *des,
					   struct max_des_pipe *pipe,
					   unsigned int stream_id)
{
	struct max96724_priv *priv = des_to_priv(des);
	u16 reg;
	u8 shift, mask, val;

	if (pipe->index >= 4)
		return -EINVAL;

	reg = 0x00F0 + (pipe->index / 2);
	shift = (pipe->index % 2) ? 4 : 0;
	mask = 0x03 << shift;
	val = (stream_id & 0x3) << shift;

	return max96724_update_bits(priv, reg, mask, val);
}

static int max96724_des_set_pipe_phy(struct max_des *des,
				     struct max_des_pipe *pipe,
				     struct max_des_phy *phy)
{
	/*
	 * 這版先不做額外 pipe->phy 邏輯切換。
	 * vendor script 是靠 MAP_DPHY_DEST / controller mapping 完成。
	 */
	pipe->phy_id = phy->index;
	return 0;
}

static int max96724_des_set_pipe_remap(struct max_des *des,
				       struct max_des_pipe *pipe,
				       unsigned int index,
				       struct max_des_remap *remap)
{
	struct max96724_priv *priv = des_to_priv(des);
	u16 base = 0x0900 + (pipe->index * 0x40);
	u16 src_reg, dst_reg, dphy_reg;
	u8 src, dst;
	u8 shift, mask, dphy_val;
	int ret;

	if (pipe->index >= 4 || index >= 16)
		return -EINVAL;

	src_reg = base + 0x0D + (index * 2);
	dst_reg = src_reg + 1;
	dphy_reg = base + 0x2D + (index / 4);

	src = ((remap->from_vc & 0x3) << 6) | (remap->from_dt & 0x3f);
	dst = ((remap->to_vc   & 0x3) << 6) | (remap->to_dt   & 0x3f);

	ret = max96724_write(priv, src_reg, src);
	if (ret) return ret;

	ret = max96724_write(priv, dst_reg, dst);
	if (ret) return ret;

	shift = (index % 4) * 2;
	mask = 0x3 << shift;
	dphy_val = (remap->phy & 0x3) << shift;

	return max96724_update_bits(priv, dphy_reg, mask, dphy_val);
}

static int max96724_des_set_pipe_remaps_enable(struct max_des *des,
					       struct max_des_pipe *pipe,
					       unsigned int mask)
{
	struct max96724_priv *priv = des_to_priv(des);
	u16 base = 0x0900 + (pipe->index * 0x40);
	u8 low = mask & 0xFF;
	u8 high = (mask >> 8) & 0xFF;
	int ret;

	ret = max96724_write(priv, base + 0x0B, low);
	if (ret)
		return ret;

	return max96724_write(priv, base + 0x0C, high);
}

static int max96724_des_set_pipe_enable(struct max_des *des,
					struct max_des_pipe *pipe,
					bool enable)
{
	struct max96724_priv *priv = des_to_priv(des);
	u8 mask = BIT(pipe->index);

	/* bit4 保持 0，legacy mode */
	return max96724_update_bits(priv, 0x00F4, mask, enable ? mask : 0);
}

static int max96724_des_set_enable(struct max_des *des, bool enable)
{
	struct max96724_priv *priv = des_to_priv(des);

	/*
	 * 這版先讓 set_enable 保守，只控制所有 PHY clocks forced running bit。
	 * 真正的 pipe/phy/map 初始化已在 init 完成。
	 */
	return max96724_update_bits(priv, 0x08A0, BIT(7), enable ? BIT(7) : 0);
}

static int max96724_des_init_link(struct max_des *des, struct max_des_link *link)
{
	struct max96724_priv *priv = des_to_priv(des);
	u8 val = 0xF0 | BIT(link->index);

	return max96724_write(priv, 0x0006, val);
}

static int max96724_des_select_links(struct max_des *des, unsigned int mask)
{
	struct max96724_priv *priv = des_to_priv(des);
	u8 val = 0xF0 | (mask & 0x0F);

	return max96724_write(priv, 0x0006, val);
}

static void max96724_decode_pkt_regs(struct device *dev,
				     unsigned int reg8d0,
				     unsigned int reg8d1,
				     unsigned int reg8d2,
				     unsigned int reg8d3,
				     const char *tag)
{
	unsigned int csi0 = reg8d0 & 0x0f;
	unsigned int csi1 = (reg8d0 >> 4) & 0x0f;
	unsigned int csi2 = reg8d1 & 0x0f;
	unsigned int csi3 = (reg8d1 >> 4) & 0x0f;
	unsigned int phy0 = reg8d2 & 0x0f;
	unsigned int phy1 = (reg8d2 >> 4) & 0x0f;
	unsigned int phy2 = reg8d3 & 0x0f;
	unsigned int phy3 = (reg8d3 >> 4) & 0x0f;

	dev_info(dev,
		 "[%s] PKT raw: 0x08D0=0x%02x 0x08D1=0x%02x 0x08D2=0x%02x 0x08D3=0x%02x\n",
		 tag, reg8d0, reg8d1, reg8d2, reg8d3);

	dev_info(dev,
		 "[%s] CSI_TX pkt nibble: ctrl0=%u ctrl1=%u ctrl2=%u ctrl3=%u\n",
		 tag, csi0, csi1, csi2, csi3);

	dev_info(dev,
		 "[%s] PHY pkt nibble: phy0=%u phy1=%u phy2=%u phy3=%u\n",
		 tag, phy0, phy1, phy2, phy3);
}

static void max96724_log_pkt_delta(struct device *dev,
				   unsigned int prev8d0,
				   unsigned int prev8d1,
				   unsigned int prev8d2,
				   unsigned int prev8d3,
				   unsigned int cur8d0,
				   unsigned int cur8d1,
				   unsigned int cur8d2,
				   unsigned int cur8d3,
				   const char *tag)
{
	unsigned int prev_csi0 = prev8d0 & 0x0f;
	unsigned int prev_csi1 = (prev8d0 >> 4) & 0x0f;
	unsigned int prev_csi2 = prev8d1 & 0x0f;
	unsigned int prev_csi3 = (prev8d1 >> 4) & 0x0f;
	unsigned int prev_phy0 = prev8d2 & 0x0f;
	unsigned int prev_phy1 = (prev8d2 >> 4) & 0x0f;
	unsigned int prev_phy2 = prev8d3 & 0x0f;
	unsigned int prev_phy3 = (prev8d3 >> 4) & 0x0f;

	unsigned int cur_csi0 = cur8d0 & 0x0f;
	unsigned int cur_csi1 = (cur8d0 >> 4) & 0x0f;
	unsigned int cur_csi2 = cur8d1 & 0x0f;
	unsigned int cur_csi3 = (cur8d1 >> 4) & 0x0f;
	unsigned int cur_phy0 = cur8d2 & 0x0f;
	unsigned int cur_phy1 = (cur8d2 >> 4) & 0x0f;
	unsigned int cur_phy2 = cur8d3 & 0x0f;
	unsigned int cur_phy3 = (cur8d3 >> 4) & 0x0f;

	dev_info(dev,
		 "[%s] PKT delta: "
		 "ctrl0 %u->%u ctrl1 %u->%u ctrl2 %u->%u ctrl3 %u->%u | "
		 "phy0 %u->%u phy1 %u->%u phy2 %u->%u phy3 %u->%u\n",
		 tag,
		 prev_csi0, cur_csi0,
		 prev_csi1, cur_csi1,
		 prev_csi2, cur_csi2,
		 prev_csi3, cur_csi3,
		 prev_phy0, cur_phy0,
		 prev_phy1, cur_phy1,
		 prev_phy2, cur_phy2,
		 prev_phy3, cur_phy3);
}


static int max96724_des_log_pipe_status(struct max_des *des,
					struct max_des_pipe *pipe)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int val;
	unsigned int map_src0 = 0, map_dst0 = 0;
	unsigned int map_src1 = 0, map_dst1 = 0;
	unsigned int map_src2 = 0, map_dst2 = 0;
	unsigned int map_dest = 0;
	u16 base = 0x0900 + (pipe->index * 0x40);

	regmap_read(priv->regmap, 0x00F0 + (pipe->index / 2), &val);
	dev_info(priv->dev, "pipe%u VIDEO_PIPE_SEL=0x%02x\n", pipe->index, val);

	regmap_read(priv->regmap, 0x00F4, &val);
	dev_info(priv->dev, "pipe%u VIDEO_PIPE_EN=0x%02x\n", pipe->index, val);

	regmap_read(priv->regmap, base + 0x0A, &val);
	dev_info(priv->dev, "pipe%u MIPI_TX_LANE_CNT=0x%02x\n", pipe->index, val);

	regmap_read(priv->regmap, base + 0x0B, &val);
	dev_info(priv->dev, "pipe%u MAP_EN_L=0x%02x\n", pipe->index, val);

	regmap_read(priv->regmap, base + 0x0C, &val);
	dev_info(priv->dev, "pipe%u MAP_EN_H=0x%02x\n", pipe->index, val);

	regmap_read(priv->regmap, base + 0x0D, &map_src0);
	regmap_read(priv->regmap, base + 0x0E, &map_dst0);
	regmap_read(priv->regmap, base + 0x0F, &map_src1);
	regmap_read(priv->regmap, base + 0x10, &map_dst1);
	regmap_read(priv->regmap, base + 0x11, &map_src2);
	regmap_read(priv->regmap, base + 0x12, &map_dst2);
	regmap_read(priv->regmap, base + 0x2D, &map_dest);

	dev_info(priv->dev, "pipe%u MAP0_SRC=0x%02x MAP0_DST=0x%02x\n",
		 pipe->index, map_src0, map_dst0);
	dev_info(priv->dev, "pipe%u MAP1_SRC=0x%02x MAP1_DST=0x%02x\n",
		 pipe->index, map_src1, map_dst1);
	dev_info(priv->dev, "pipe%u MAP2_SRC=0x%02x MAP2_DST=0x%02x\n",
		 pipe->index, map_src2, map_dst2);
	dev_info(priv->dev, "pipe%u MAP_DPHY_DEST=0x%02x\n",
		 pipe->index, map_dest);

	dev_info(priv->dev,
		 "pipe%u decode: MAP0 image VC=%u DT=0x%02x -> VC=%u DT=0x%02x\n",
		 pipe->index,
		 (map_src0 >> 6) & 0x3, map_src0 & 0x3f,
		 (map_dst0 >> 6) & 0x3, map_dst0 & 0x3f);

	dev_info(priv->dev,
		 "pipe%u decode: MAP1 FS    VC=%u DT=0x%02x -> VC=%u DT=0x%02x\n",
		 pipe->index,
		 (map_src1 >> 6) & 0x3, map_src1 & 0x3f,
		 (map_dst1 >> 6) & 0x3, map_dst1 & 0x3f);

	dev_info(priv->dev,
		 "pipe%u decode: MAP2 FE    VC=%u DT=0x%02x -> VC=%u DT=0x%02x\n",
		 pipe->index,
		 (map_src2 >> 6) & 0x3, map_src2 & 0x3f,
		 (map_dst2 >> 6) & 0x3, map_dst2 & 0x3f);

	dev_info(priv->dev,
		 "pipe%u decode: map0->ctrl%u map1->ctrl%u map2->ctrl%u map3->ctrl%u\n",
		 pipe->index,
		 (map_dest >> 0) & 0x3,
		 (map_dest >> 2) & 0x3,
		 (map_dest >> 4) & 0x3,
		 (map_dest >> 6) & 0x3);

	return 0;
}

static int max96724_des_log_phy_status(struct max_des *des,
				       struct max_des_phy *phy)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int lock = 0;
	unsigned int phy_mode = 0, phy_en = 0;
	unsigned int lane_map01 = 0, lane_map23 = 0;
	unsigned int ctrl0_lane_cnt = 0, ctrl1_lane_cnt = 0;
	unsigned int dpll0 = 0, dpll1 = 0, dpll2 = 0, dpll3 = 0;
	unsigned int reg8d0 = 0, reg8d1 = 0, reg8d2 = 0, reg8d3 = 0;

	regmap_read(priv->regmap, 0x001A, &lock);
	regmap_read(priv->regmap, 0x08A0, &phy_mode);
	regmap_read(priv->regmap, 0x08A2, &phy_en);
	regmap_read(priv->regmap, 0x08A3, &lane_map01);
	regmap_read(priv->regmap, 0x08A4, &lane_map23);
	regmap_read(priv->regmap, 0x090A, &ctrl0_lane_cnt);
	regmap_read(priv->regmap, 0x094A, &ctrl1_lane_cnt);
	regmap_read(priv->regmap, 0x0415, &dpll0);
	regmap_read(priv->regmap, 0x0418, &dpll1);
	regmap_read(priv->regmap, 0x041B, &dpll2);
	regmap_read(priv->regmap, 0x041E, &dpll3);

	regmap_read(priv->regmap, 0x08D0, &reg8d0);
	regmap_read(priv->regmap, 0x08D1, &reg8d1);
	regmap_read(priv->regmap, 0x08D2, &reg8d2);
	regmap_read(priv->regmap, 0x08D3, &reg8d3);

	dev_info(priv->dev, "LINKA_LOCK=0x%02x\n", lock);
	dev_info(priv->dev, "MIPI_PHY_MODE=0x%02x\n", phy_mode);
	dev_info(priv->dev, "MIPI_PHY_ENABLE=0x%02x\n", phy_en);
	dev_info(priv->dev, "LANE_MAP01=0x%02x\n", lane_map01);
	dev_info(priv->dev, "LANE_MAP23=0x%02x\n", lane_map23);
	dev_info(priv->dev, "CTRL0_LANE_CNT=0x%02x CTRL1_LANE_CNT=0x%02x\n",
		 ctrl0_lane_cnt, ctrl1_lane_cnt);
	dev_info(priv->dev, "DPLL0=0x%02x DPLL1=0x%02x DPLL2=0x%02x DPLL3=0x%02x\n",
		 dpll0, dpll1, dpll2, dpll3);

	dev_info(priv->dev, "[DECODE] LINKA lock=%u\n", !!(lock & BIT(0)));

	dev_info(priv->dev,
		 "[DECODE] PHY mode bits: force_all_clk=%u phy3_clk=%u phy0_clk=%u mode_low_nibble=0x%x\n",
		 !!(phy_mode & BIT(7)),
		 !!(phy_mode & BIT(6)),
		 !!(phy_mode & BIT(5)),
		 phy_mode & 0x0f);

	dev_info(priv->dev,
		 "[DECODE] PHY enable bits: phy3=%u phy2=%u phy1=%u phy0=%u\n",
		 !!(phy_en & BIT(7)),
		 !!(phy_en & BIT(6)),
		 !!(phy_en & BIT(5)),
		 !!(phy_en & BIT(4)));

	dev_info(priv->dev,
		 "[DECODE] LANE_MAP01: phy1_l1=D%u phy1_l0=D%u phy0_l1=D%u phy0_l0=D%u\n",
		 (lane_map01 >> 6) & 0x3,
		 (lane_map01 >> 4) & 0x3,
		 (lane_map01 >> 2) & 0x3,
		 (lane_map01 >> 0) & 0x3);

	dev_info(priv->dev,
		 "[DECODE] controller1 lane count field=%u (%s)\n",
		 (ctrl1_lane_cnt >> 6) & 0x3,
		 (((ctrl1_lane_cnt >> 6) & 0x3) == 0x3) ? "4-lane" :
		 (((ctrl1_lane_cnt >> 6) & 0x3) == 0x2) ? "3-lane" :
		 (((ctrl1_lane_cnt >> 6) & 0x3) == 0x1) ? "2-lane" : "1-lane");

	dev_info(priv->dev,
		 "[DECODE] DPLL raw=0x%02x, selected preset=%s\n",
		 dpll1, DES_TEST_RATE_NAME);

	max96724_decode_pkt_regs(priv->dev, reg8d0, reg8d1, reg8d2, reg8d3, "PHY");

	return 0;
}

static int max96724_des_log_status(struct max_des *des)
{
	struct max96724_priv *priv = des_to_priv(des);
	unsigned int reg_00f0 = 0, reg_00f4 = 0;
	unsigned int reg_08a0 = 0, reg_08a2 = 0;
	unsigned int reg_090a = 0, reg_094a = 0;
	unsigned int reg_092d = 0;
	unsigned int dpll0 = 0, dpll1 = 0, dpll2 = 0, dpll3 = 0;
	unsigned int reg_0018 = 0, reg_001a = 0;
	int lane_cnt_field1;
	int link_sel, stream_id;
	int i;
	static const unsigned int sample_ms[] = { 0, 20, 50, 100, 200 };

	dev_info(priv->dev, "================= MAX96724 STATUS =================\n");

	regmap_read(priv->regmap, 0x00F0, &reg_00f0);
	regmap_read(priv->regmap, 0x00F4, &reg_00f4);
	regmap_read(priv->regmap, 0x08A0, &reg_08a0);
	regmap_read(priv->regmap, 0x08A2, &reg_08a2);
	regmap_read(priv->regmap, 0x090A, &reg_090a);
	regmap_read(priv->regmap, 0x094A, &reg_094a);
	regmap_read(priv->regmap, 0x092D, &reg_092d);
	regmap_read(priv->regmap, 0x0415, &dpll0);
	regmap_read(priv->regmap, 0x0418, &dpll1);
	regmap_read(priv->regmap, 0x041B, &dpll2);
	regmap_read(priv->regmap, 0x041E, &dpll3);
	regmap_read(priv->regmap, 0x0018, &reg_0018);
	regmap_read(priv->regmap, 0x001A, &reg_001a);

	link_sel = (reg_00f0 >> 2) & 0x3;
	stream_id = reg_00f0 & 0x3;
	lane_cnt_field1 = (reg_094a >> 6) & 0x3;

	dev_info(priv->dev, "[HW] active=%d\n", des->active);
	dev_info(priv->dev, "[HW] 0x0006/0x0018/0x001A: RESET=0x%02x LOCK=0x%02x\n",
		 reg_0018, reg_001a);
	dev_info(priv->dev, "[HW] 0x00F0=0x%02x -> pipe0: link=%d stream_id=%d\n",
		 reg_00f0, link_sel, stream_id);
	dev_info(priv->dev, "[HW] 0x00F4=0x%02x -> pipe0_en=%d\n",
		 reg_00f4, !!(reg_00f4 & BIT(0)));
	dev_info(priv->dev,
		 "[HW] 0x08A0=0x%02x 0x08A2=0x%02x 0x090A=0x%02x 0x094A=0x%02x 0x092D=0x%02x\n",
		 reg_08a0, reg_08a2, reg_090a, reg_094a, reg_092d);
	dev_info(priv->dev, "[HW] DPLL0=0x%02x DPLL1=0x%02x DPLL2=0x%02x DPLL3=0x%02x\n",
		 dpll0, dpll1, dpll2, dpll3);

	if ((reg_08a0 & 0x1f) == 0x01)
		dev_info(priv->dev,
			 "[HW-DECODE] PHY mode looks like 4x2 mode (independent 2-lane PHYs)\n");

	if (lane_cnt_field1 == 0x1)
		dev_info(priv->dev,
			 "[HW-DECODE] controller1 lane count field indicates 2 lanes\n");

	dev_info(priv->dev,
		 "[SW-SYNC] pipe0: enabled=%d link_id=%u stream_id=%u phy_id=%u\n",
		 des->pipes[0].enabled,
		 des->pipes[0].link_id,
		 des->pipes[0].stream_id,
		 des->pipes[0].phy_id);

	dev_info(priv->dev,
		 "[SW-SYNC] phy1: enabled=%d lanes=%u clk_lane=%u freq=%lld\n",
		 des->phys[1].enabled,
		 des->phys[1].mipi.num_data_lanes,
		 des->phys[1].mipi.clock_lane,
		 des->phys[1].link_frequency);

	max96724_des_log_pipe_status(des, &des->pipes[0]);
	max96724_des_log_phy_status(des, &des->phys[1]);

	for (i = 0; i < ARRAY_SIZE(sample_ms); i++) {
		unsigned int reg8d0 = 0, reg8d1 = 0, reg8d2 = 0, reg8d3 = 0;
		unsigned int next8d0 = 0, next8d1 = 0, next8d2 = 0, next8d3 = 0;

		if (sample_ms[i])
			msleep(sample_ms[i]);

		regmap_read(priv->regmap, 0x08D0, &reg8d0);
		regmap_read(priv->regmap, 0x08D1, &reg8d1);
		regmap_read(priv->regmap, 0x08D2, &reg8d2);
		regmap_read(priv->regmap, 0x08D3, &reg8d3);

		max96724_decode_pkt_regs(priv->dev, reg8d0, reg8d1, reg8d2, reg8d3, "SAMPLE-A");

		usleep_range(3000, 5000);

		regmap_read(priv->regmap, 0x08D0, &next8d0);
		regmap_read(priv->regmap, 0x08D1, &next8d1);
		regmap_read(priv->regmap, 0x08D2, &next8d2);
		regmap_read(priv->regmap, 0x08D3, &next8d3);

		max96724_decode_pkt_regs(priv->dev, next8d0, next8d1, next8d2, next8d3, "SAMPLE-B");
		max96724_log_pkt_delta(priv->dev,
				       reg8d0, reg8d1, reg8d2, reg8d3,
				       next8d0, next8d1, next8d2, next8d3,
				       "SAMPLE-DELTA");

		dev_info(priv->dev,
			 "[SAMPLE @%ums] ctrl/phy packet counters %s changing\n",
			 sample_ms[i],
			 (reg8d0 != next8d0 || reg8d1 != next8d1 ||
			  reg8d2 != next8d2 || reg8d3 != next8d3) ?
			 "ARE" : "are NOT");
	}

	dev_info(priv->dev, "===================================================\n");
	return 0;
}
static const struct max_des_ops max96724_ops = {
	.num_phys = 4,
	.num_pipes = 4,
	.num_links = 4,
	.num_remaps_per_pipe = 16,

	.init = max96724_des_init,
	.init_phy = max96724_des_init_phy,
	.set_phy_enable = max96724_des_set_phy_enable,

	.set_pipe_link = max96724_des_set_pipe_link,
	.set_pipe_stream_id = max96724_des_set_pipe_stream_id,
	.set_pipe_phy = max96724_des_set_pipe_phy,
	.set_pipe_remap = max96724_des_set_pipe_remap,
	.set_pipe_remaps_enable = max96724_des_set_pipe_remaps_enable,
	.set_pipe_enable = max96724_des_set_pipe_enable,

	.set_enable = max96724_des_set_enable,

	.init_link = max96724_des_init_link,
	.select_links = max96724_des_select_links,

	.log_status = max96724_des_log_status,
	.log_pipe_status = max96724_des_log_pipe_status,
	.log_phy_status = max96724_des_log_phy_status,

	.stream_prepare = max96724_des_stream_prepare,
};

static int max96724_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max96724_priv *priv;
	struct max_des_ops *ops;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ops = devm_kzalloc(dev, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return -ENOMEM;

	priv->info = &max96724_info;
	priv->dev = dev;
	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->regmap = devm_regmap_init_i2c(client, &max96724_i2c_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	*ops = max96724_ops;
	priv->des.ops = ops;

	ret = max96724_reset(priv);
	if (ret)
		return ret;

	return max_des_probe(client, &priv->des);
}

static void max96724_remove(struct i2c_client *client)
{
	struct max96724_priv *priv = i2c_get_clientdata(client);

	max_des_remove(&priv->des);
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id max96724_acpi_ids[] = {
	{ "MAX96724", (kernel_ulong_t)&max96724_info },
	{ "APR96724", (kernel_ulong_t)&max96724_info },
	{ "INTC113C", (kernel_ulong_t)&max96724_info },
	{ }
};
MODULE_DEVICE_TABLE(acpi, max96724_acpi_ids);
#else
static const struct of_device_id max96724_of_table[] = {
	{ .compatible = "maxim,max96724", .data = &max96724_info },
	{ .compatible = "maxim,max96724f", .data = &max96724_info },
	{ .compatible = "maxim,max96724r", .data = &max96724_info },
	{ }
};
MODULE_DEVICE_TABLE(of, max96724_of_table);
#endif

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
MODULE_LICENSE("GPL");
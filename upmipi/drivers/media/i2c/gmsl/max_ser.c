// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim GMSL2 Serializer Driver (Forced Link to I2C-1)
 */

#include <linux/delay.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

#include "max_ser.h"
#include "max_serdes.h"

#ifndef MEDIA_PAD_FL_INTERNAL
#define MEDIA_PAD_FL_INTERNAL 0
#endif

#define MAX_SER_NUM_LINKS	1
#define MAX_SER_NUM_PHYS	1

#define MAX96717_CAM_RESET_MFP0_REG   0x02BE
#define MAX96717_MFP7_TRIGGER_REG     0x02D3
#define MAX96717_MFP8_TRIGGER_REG     0x02D6
#define MAX96717_DATATYPE_REG         0x0318

#define MAX96717_GPIO_LOW             0x00
#define MAX96717_GPIO_HIGH            0x10

/* 廠商 script 標的是 YUV422 datatype */
#define MAX96717_DATATYPE_YUV422      0x5E

struct max_ser_priv {
	struct max_ser *ser;
	struct device *dev;
	struct i2c_client *client;
	struct i2c_mux_core *mux;
	struct media_pad *pads;
	struct max_serdes_source *sources;
	u64 *streams_masks;
	u32 double_bpps;
	struct v4l2_subdev sd;
	struct v4l2_async_notifier nf;
	struct v4l2_ctrl_handler ctrl_handler;
};

struct max_ser_route_hw {
	struct max_serdes_source *source;
	struct max_ser_pipe *pipe;
	struct v4l2_mbus_frame_desc_entry entry;
	bool is_tpg;
};

static inline struct max_ser_priv *sd_to_priv(struct v4l2_subdev *sd)
{
	return container_of(sd, struct max_ser_priv, sd);
}

static inline struct max_ser_priv *nf_to_priv(struct v4l2_async_notifier *nf)
{
	return container_of(nf, struct max_ser_priv, nf);
}

static inline struct max_ser_priv *ctrl_to_priv(struct v4l2_ctrl_handler *handler)
{
	return container_of(handler, struct max_ser_priv, ctrl_handler);
}

static inline bool max_ser_pad_is_sink(struct max_ser *ser, u32 pad)
{
	return pad < ser->ops->num_phys;
}

static inline bool max_ser_pad_is_source(struct max_ser *ser, u32 pad)
{
	return pad >= ser->ops->num_phys &&
	       pad < ser->ops->num_phys + MAX_SER_NUM_LINKS;
}

static inline u32 max_ser_source_pad(struct max_ser *ser)
{
	return ser->ops->num_phys;
}

static inline bool max_ser_pad_is_tpg(struct max_ser *ser, u32 pad)
{
	return pad >= ser->ops->num_phys + MAX_SER_NUM_LINKS;
}

static inline unsigned int max_ser_phy_to_pad(struct max_ser *ser,
					      struct max_ser_phy *phy)
{
	return phy->index;
}

static inline unsigned int max_ser_num_pads(struct max_ser *ser)
{
	return ser->ops->num_phys + MAX_SER_NUM_LINKS +
	       (ser->ops->set_tpg ? 1 : 0);
}

static struct max_ser_phy *max_ser_pad_to_phy(struct max_ser *ser, u32 pad)
{
	if (!max_ser_pad_is_sink(ser, pad))
		return NULL;
	return &ser->phys[pad];
}

static struct max_ser_pipe *
max_ser_find_phy_pipe(struct max_ser *ser, struct max_ser_phy *phy)
{
	unsigned int i;
	for (i = 0; i < ser->ops->num_pipes; i++) {
		struct max_ser_pipe *pipe = &ser->pipes[i];
		if (pipe->phy_id == phy->index)
			return pipe;
	}
	return NULL;
}

static struct max_serdes_source *
max_ser_get_phy_source(struct max_ser_priv *priv, struct max_ser_phy *phy)
{
	return &priv->sources[phy->index];
}

static const struct max_serdes_tpg_entry *
max_ser_find_tpg_entry(struct max_ser *ser, u32 target_index,
		       u32 width, u32 height, u32 code,
		       u32 numerator, u32 denominator)
{
	const struct max_serdes_tpg_entry *entry;
	unsigned int index = 0;
	unsigned int i;

	for (i = 0; i < ser->ops->tpg_entries.num_entries; i++) {
		entry = &ser->ops->tpg_entries.entries[i];
		if ((width != 0 && width != entry->width) ||
		    (height != 0 && height != entry->height) ||
		    (code != 0 && code != entry->code) ||
		    (numerator != 0 && numerator != entry->interval.numerator) ||
		    (denominator != 0 && denominator != entry->interval.denominator))
			continue;
		if (index == target_index)
			break;
		index++;
	}
	if (i == ser->ops->tpg_entries.num_entries)
		return NULL;
	return &ser->ops->tpg_entries.entries[i];
}

static const struct max_serdes_tpg_entry *
max_ser_find_state_tpg_entry(struct max_ser *ser, struct v4l2_subdev_state *state,
			     unsigned int pad)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_fract *in;

	fmt = v4l2_subdev_state_get_format(state, pad, MAX_SERDES_TPG_STREAM);
	if (!fmt) return NULL;
	in = v4l2_subdev_state_get_interval(state, pad, MAX_SERDES_TPG_STREAM);
	if (!in) return NULL;
	return max_ser_find_tpg_entry(ser, 0, fmt->width, fmt->height, fmt->code,
				      in->numerator, in->denominator);
}

static int max_ser_get_tpg_fd_entry_state(struct max_ser *ser,
					  struct v4l2_subdev_state *state,
					  struct v4l2_mbus_frame_desc_entry *fd_entry,
					  unsigned int pad)
{
	const struct max_serdes_tpg_entry *entry;
	entry = max_ser_find_state_tpg_entry(ser, state, pad);
	if (!entry) return -EINVAL;
	fd_entry->stream = MAX_SERDES_TPG_STREAM;
	fd_entry->flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
	fd_entry->length = entry->width * entry->height * entry->bpp / 8;
	fd_entry->pixelcode = entry->code;
	fd_entry->bus.csi2.vc = 0;
	fd_entry->bus.csi2.dt = entry->dt;
	return 0;
}

static int max_ser_tpg_route_to_hw(struct max_ser_priv *priv,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_route *route,
				   struct max_ser_route_hw *hw)
{
	struct max_ser *ser = priv->ser;
	hw->pipe = &ser->pipes[0];
	return max_ser_get_tpg_fd_entry_state(ser, state, &hw->entry,
					      route->sink_pad);
}

static int max_ser_route_to_hw(struct max_ser_priv *priv,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_route *route,
			       struct max_ser_route_hw *hw)
{
	struct max_ser *ser = priv->ser;
	struct v4l2_mbus_frame_desc fd;
	struct max_ser_phy *phy;
	unsigned int i;
	int ret;

	memset(hw, 0, sizeof(*hw));
	hw->is_tpg = max_ser_pad_is_tpg(ser, route->sink_pad);
	if (hw->is_tpg)
		return max_ser_tpg_route_to_hw(priv, state, route, hw);

	phy = max_ser_pad_to_phy(ser, route->sink_pad);
	if (!phy) return -ENOENT;

	hw->pipe = max_ser_find_phy_pipe(ser, phy);
	if (!hw->pipe) return -ENOENT;

	hw->source = max_ser_get_phy_source(priv, phy);
	if (!hw->source->sd) return 0;

	ret = v4l2_subdev_call(hw->source->sd, pad, get_frame_desc,
			       hw->source->pad, &fd);
	if (ret) return ret;

	for (i = 0; i < fd.num_entries; i++)
		if (fd.entry[i].stream == route->sink_stream)
			break;

	if (i == fd.num_entries) return -ENOENT;
	hw->entry = fd.entry[i];
	return 0;
}

static int max_ser_phy_set_active(struct max_ser *ser, struct max_ser_phy *phy,
				  bool active)
{
	int ret;
	if (ser->ops->set_phy_active) {
		ret = ser->ops->set_phy_active(ser, phy, active);
		if (ret) return ret;
	}
	phy->active = active;
	return 0;
}

static int max_ser_set_pipe_dts(struct max_ser_priv *priv, struct max_ser_pipe *pipe,
				unsigned int *dts, unsigned int num_dts)
{
	struct max_ser *ser = priv->ser;
	unsigned int i;
	int ret;

	if (!ser->ops->set_pipe_dt || !ser->ops->set_pipe_dt_en)
		return 0;

	for (i = 0; i < num_dts; i++) {
		ret = ser->ops->set_pipe_dt(ser, pipe, i, dts[i]);
		if (ret) return ret;
		ret = ser->ops->set_pipe_dt_en(ser, pipe, i, true);
		if (ret) return ret;
	}
	if (num_dts == pipe->num_dts) return 0;
	for (i = num_dts; i < ser->ops->num_dts_per_pipe; i++) {
		ret = ser->ops->set_pipe_dt_en(ser, pipe, i, false);
		if (ret) return ret;
	}
	return 0;
}

static int max_ser_set_pipe_mode(struct max_ser_priv *priv, struct max_ser_pipe *pipe,
				 struct max_ser_pipe_mode *mode)
{
	struct max_ser *ser = priv->ser;
	if (mode->bpp == pipe->mode.bpp &&
	    mode->soft_bpp == pipe->mode.soft_bpp &&
	    mode->dbl8 == pipe->mode.dbl8 &&
	    mode->dbl10 == pipe->mode.dbl10 &&
	    mode->dbl12 == pipe->mode.dbl12)
		return 0;
	if (!ser->ops->set_pipe_mode) return 0;
	return ser->ops->set_pipe_mode(ser, pipe, mode);
}

static int max_ser_i2c_mux_select(struct i2c_mux_core *mux, u32 chan)
{
	return 0;
}

static void max_ser_i2c_mux_deinit(struct max_ser_priv *priv)
{
	i2c_mux_del_adapters(priv->mux);
}

static int max_ser_i2c_mux_init(struct max_ser_priv *priv)
{
	u32 flags = I2C_MUX_LOCKED;
	int ret;

	/*
	 * 關鍵修正：
	 * serializer 這層也先不要用 I2C_MUX_GATE。
	 *
	 * 目前 child bus 主要用途是掛 sensor。
	 * 在目前手動 new_device / parent-bus 直掛 serializer 的 bring-up 架構下，
	 * GATE 很可能造成 parent bus 與 child bus 間 access 狀態不一致，
	 * 間接讓 des 在 parent bus 上變成 -121。
	 */
	dev_info(priv->dev,
		 "[MUX] init ser i2c mux: parent=i2c-%d flags=0x%x (GATE disabled)\n",
		 priv->client->adapter->nr, flags);

	priv->mux = i2c_mux_alloc(priv->client->adapter, &priv->client->dev,
				  1, 0, flags,
				  max_ser_i2c_mux_select, NULL);
	if (!priv->mux)
		return -ENOMEM;

	ret = i2c_mux_add_adapter(priv->mux, 0, 0);
	if (ret) {
		max_ser_i2c_mux_deinit(priv);
		return ret;
	}

	if (priv->mux->adapter[0])
		dev_info(priv->dev,
			 "[MUX] ser child adapter[0] -> i2c-%d\n",
			 priv->mux->adapter[0]->nr);

	return 0;
}

static int max_ser_i2c_adapter_init(struct max_ser_priv *priv)
{
	return max_ser_i2c_mux_init(priv);
}

static void max_ser_i2c_adapter_deinit(struct max_ser_priv *priv)
{
	max_ser_i2c_mux_deinit(priv);
}

static int max_ser_set_tpg_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct max_ser_priv *priv = v4l2_get_subdevdata(sd);
	struct max_ser *ser = priv->ser;
	const struct max_serdes_tpg_entry *entry;
	struct v4l2_fract *in;

	if (format->stream != MAX_SERDES_TPG_STREAM) return -EINVAL;
	entry = max_ser_find_tpg_entry(ser, 0, fmt->width, fmt->height,
				       fmt->code, 0, 0);
	if (!entry) return -EINVAL;
	in = v4l2_subdev_state_get_interval(state, format->pad, format->stream);
	if (!in) return -EINVAL;
	in->numerator = entry->interval.numerator;
	in->denominator = entry->interval.denominator;
	return 0;
}

static int max_ser_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	/* 強制寫死 1920x1536 UYVY */
	format->format.width = 1920;
	format->format.height = 1536;
	format->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
	format->format.field = V4L2_FIELD_NONE;
	format->format.colorspace = V4L2_COLORSPACE_SRGB;

	struct v4l2_mbus_framefmt *f = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (f) *f = format->format;
	return 0;
}

static int max_ser_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	/* 無視使用者的設定，死守 1920x1536 UYVY */
	format->format.width = 1920;
	format->format.height = 1536;
	format->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
	format->format.field = V4L2_FIELD_NONE;
	format->format.colorspace = V4L2_COLORSPACE_SRGB;

	struct v4l2_mbus_framefmt *f = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (f) *f = format->format;
	return 0;
}

static int max_ser_log_status(struct v4l2_subdev *sd)
{
    return 0;
}

static int max_ser_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct max_ser_priv *priv = ctrl_to_priv(ctrl->handler);
	struct max_ser *ser = priv->ser;
	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		ser->tpg_pattern = ctrl->val;
		return 0;
	}
	return -EINVAL;
}

static int max_ser_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	return 0; 
}

static int max_ser_set_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_interval *fi)
{
	return 0;
}

static int max_ser_get_frame_desc_state(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					struct v4l2_mbus_frame_desc *fd,
					unsigned int pad)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct max_ser *ser = priv->ser;
	struct v4l2_subdev_route *route;
	int ret;

	if (!max_ser_pad_is_source(ser, pad)) return -ENOENT;
	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	for_each_active_route(&state->routing, route) {
		struct max_ser_route_hw hw;
		if (pad != route->source_pad) continue;
		ret = max_ser_route_to_hw(priv, state, route, &hw);
		if (ret) return ret;
		hw.entry.stream = route->source_stream;
		fd->entry[fd->num_entries++] = hw.entry;
	}
	return 0;
}

static int max_ser_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	/* 只有 Pad 1 (Source) 需要回報給 Deserializer */
	if (pad != 1) {
		fd->num_entries = 0;
		return 0;
	}

	/* 同樣無腦塞入 1920x1536 UYVY */
	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;
	fd->entry[0].stream = 0;
	fd->entry[0].flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
	fd->entry[0].length = 1920 * 1536 * 2; /* 16bpp */
	fd->entry[0].pixelcode = MEDIA_BUS_FMT_UYVY8_1X16;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = 0x1E; /* UYVY 標籤 */

	return 0;
}

static int max_ser_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				   struct v4l2_mbus_config *cfg)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct max_ser *ser = priv->ser;
	struct max_ser_phy *phy;
	phy = max_ser_pad_to_phy(ser, pad);
	if (!phy) return -EINVAL;
	cfg->type = V4L2_MBUS_CSI2_DPHY; 
	cfg->bus.mipi_csi2 = phy->mipi;
	return 0;
}

static int max_ser_set_routing(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       enum v4l2_subdev_format_whence which,
			       struct v4l2_subdev_krouting *routing)
{
	return v4l2_subdev_set_routing(sd, state, routing);
}

static int max_ser_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[1] = { 0 };
	struct v4l2_subdev_krouting routing = {
		.routes = routes,
		.num_routes = 1,
	};
	struct v4l2_mbus_framefmt *fmt;
	int i, ret;

	/* [借鏡 DES] 強制建立一條從 Pad 0 到 Pad 1 的直通路由 */
	routes[0].sink_pad = 0;
	routes[0].sink_stream = 0;
	routes[0].source_pad = 1;
	routes[0].source_stream = 0;
	routes[0].flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;

	ret = v4l2_subdev_set_routing(sd, state, &routing);
	if (ret) {
		/* 即使 V4L2 內部路由驗證有小抱怨，我們也印個 log 就好，不要 return 錯誤 */
		dev_warn(sd->dev, "Routing setup returned %d, ignoring...\n", ret);
	}

	/* [借鏡 DES] 強制把所有的 Pad 預設格式塞入 1920x1536 UYVY */
	for (i = 0; i < sd->entity.num_pads; i++) {
		fmt = v4l2_subdev_state_get_format(state, i, 0);
		if (fmt) {
			fmt->width = 1920;
			fmt->height = 1536;
			fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
			fmt->field = V4L2_FIELD_NONE;
			fmt->colorspace = V4L2_COLORSPACE_SRGB;
		}
	}

	/* 為了確保 /dev/v4l-subdevX 能順利生成，一律回傳 0 (成功)！ */
	return 0;
}

static int max_ser_write_local_reg(struct max_ser_priv *priv, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	struct i2c_msg msg = {
		.addr  = priv->client->addr,
		.flags = 0,
		.len   = sizeof(buf),
		.buf   = buf,
	};
	int ret;

	ret = i2c_transfer(priv->client->adapter, &msg, 1);
	if (ret != 1) {
		if (ret >= 0)
			ret = -EIO;
		dev_err(priv->dev,
			"[SER] local reg write failed: addr=0x%02x reg=0x%04x val=0x%02x ret=%d\n",
			priv->client->addr, reg, val, ret);
		return ret;
	}

	return 0;
}

static int max_ser_vendor_prep_before_sensor(struct max_ser_priv *priv)
{
	int ret;

	/*
	 * 依廠商 script：
	 *   0x0318 = 0x5E  (YUV422 datatype)
	 *   0x02BE = 0x10  (camera reset via MAX96717F MFP0)
	 */
	dev_info(priv->dev, "[SER] vendor prep: set datatype + camera reset\n");

	ret = max_ser_write_local_reg(priv, MAX96717_DATATYPE_REG,
				      MAX96717_DATATYPE_YUV422);
	if (ret)
		return ret;

	ret = max_ser_write_local_reg(priv, MAX96717_CAM_RESET_MFP0_REG,
				      MAX96717_GPIO_HIGH);
	if (ret)
		return ret;

	/* 給模組 reset 緩衝時間 */
	msleep(20);

	return 0;
}

static int max_ser_vendor_kick_after_sensor(struct max_ser_priv *priv)
{
	int ret;

	/*
	 * 承接「des 有小封包」的基線版本：
	 * 只做 MFP7 / MFP8 pulse，不加入 0x0000=0x82，
	 * 也不改成 300ms 長 delay。
	 *
	 * 原因：
	 * 你後面幾輪退步幾乎都和 vendor kick 被改動有關。
	 * 這份基線已證明：這個較小的 kick 至少能讓 ser/des 都看到 packet。
	 */
	dev_info(priv->dev, "[SER] vendor kick: pulse MFP7/MFP8\n");

	ret = max_ser_write_local_reg(priv, 0x02d3, 0x00);
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	ret = max_ser_write_local_reg(priv, 0x02d3, 0x10);
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	ret = max_ser_write_local_reg(priv, 0x02d6, 0x00);
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	ret = max_ser_write_local_reg(priv, 0x02d6, 0x10);
	if (ret)
		return ret;

	usleep_range(1000, 2000);
	return 0;
}

#include <linux/delay.h>  // msleep

static int max_ser_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct max_ser_priv *priv = v4l2_get_subdevdata(sd);
	struct max_ser *ser = priv->ser;
	struct max_ser_phy *phy;
	struct max_ser_pipe *pipe;
	unsigned int dts[] = { 0x1e, 0x00, 0x01 };
	int ret = 0, n;

	dev_info(priv->dev, "[SER][ABTEST-NOVC-DT1E00FE-CONTCLK] enter enable_streams\n");

	if (!ser || !ser->ops)
		return -EINVAL;

	phy  = &ser->phys[0];
	pipe = &ser->pipes[0];

	dev_info(priv->dev, "[SER] sources[0].sd=%s\n",
		 priv->sources && priv->sources[0].sd ?
		 priv->sources[0].sd->name : "NULL");
	dev_info(priv->dev, "[SER] enable_streams pad=%u mask=0x%llx\n",
		 pad, streams_mask);

	/* 1) serializer MIPI RX 基本參數 */
	memset(&phy->mipi, 0, sizeof(phy->mipi));
	phy->enabled = true;
	phy->mipi.num_data_lanes = 4;
	phy->mipi.data_lanes[0] = 1;
	phy->mipi.data_lanes[1] = 2;
	phy->mipi.data_lanes[2] = 3;
	phy->mipi.data_lanes[3] = 4;
	phy->mipi.clock_lane = 0;

	/* 先維持 continuous clock，不在這輪動 */
	phy->mipi.flags = 0;

	if (ser->ops->init_phy) {
		for (n = 0; n < 20; n++) {
			ret = ser->ops->init_phy(ser, phy);
			if (ret != -EREMOTEIO)
				break;
			msleep(20);
		}
		if (ret) {
			dev_err(priv->dev, "[SER] init_phy failed: %d\n", ret);
			return ret;
		}
	}

	if (ser->ops->log_status) {
		dev_info(priv->dev, "[SER] status after init_phy:\n");
		ser->ops->log_status(ser);
	}

	/* 2) 開 serializer RX port */
	if (ser->ops->set_phy_active) {
		for (n = 0; n < 20; n++) {
			ret = ser->ops->set_phy_active(ser, phy, true);
			if (ret != -EREMOTEIO)
				break;
			msleep(20);
		}
		if (ret) {
			dev_err(priv->dev, "[SER] set_phy_active(true) failed: %d\n", ret);
			return ret;
		}
	}

	if (ser->ops->log_status) {
		dev_info(priv->dev, "[SER] status after set_phy_active:\n");
		ser->ops->log_status(ser);
	}

	/* 3) 配 serializer pipe */
	pipe->enabled = true;
	pipe->stream_id = 0;

	if (ser->ops->set_pipe_phy) {
		ret = ser->ops->set_pipe_phy(ser, pipe, phy);
		if (ret) {
			dev_err(priv->dev, "[SER] set_pipe_phy failed: %d\n", ret);
			goto err_phy_off;
		}
	}

	if (ser->ops->set_pipe_stream_id) {
		ret = ser->ops->set_pipe_stream_id(ser, pipe, 0);
		if (ret) {
			dev_err(priv->dev, "[SER] set_pipe_stream_id failed: %d\n", ret);
			goto err_phy_off;
		}
	}

	/*
	 * 關鍵修改：
	 * 不設 VC filter，但只放行 UYVY + FS + FE。
	 * 目標是把 serializer 端的雜 packet 先收斂掉，
	 * 避免 host 看到過多不需要的 header/long packet。
	 */
	ret = max_ser_set_pipe_dts(priv, pipe, dts, ARRAY_SIZE(dts));
	if (ret) {
		dev_err(priv->dev, "[SER] set_pipe_dts(1e/00/01) failed: %d\n", ret);
		goto err_phy_off;
	}

	if (ser->ops->log_status) {
		dev_info(priv->dev,
			 "[SER] status after forwarding config (no VC filter, DT={0x1e,0x00,0x01}):\n");
		ser->ops->log_status(ser);
	}

	/* 先 enable pipe，讓 forwarding path ready */
	if (ser->ops->set_pipe_enable) {
		for (n = 0; n < 20; n++) {
			ret = ser->ops->set_pipe_enable(ser, pipe, true);
			if (ret != -EREMOTEIO)
				break;
			msleep(20);
		}
		if (ret) {
			dev_err(priv->dev, "[SER] set_pipe_enable(true) failed: %d\n", ret);
			goto err_phy_off;
		}
	}

	if (ser->ops->log_status) {
		dev_info(priv->dev, "[SER] status after early pipe enable:\n");
		ser->ops->log_status(ser);
	}

	/* 4) 廠商 script：datatype + camera reset */
	ret = max_ser_vendor_prep_before_sensor(priv);
	if (ret) {
		dev_err(priv->dev, "[SER] vendor prep failed: %d\n", ret);
		goto err_pipe_off;
	}

	if (ser->ops->log_status) {
		dev_info(priv->dev, "[SER] status after vendor prep:\n");
		ser->ops->log_status(ser);
	}

	/* 5) 再 enable sensor */
	if (priv->sources && priv->sources[0].sd) {
		dev_info(priv->dev, "[SER] Enabling ISX031 sensor...\n");
		ret = v4l2_subdev_enable_streams(priv->sources[0].sd,
						 priv->sources[0].pad,
						 BIT_ULL(0));
		if (ret) {
			dev_err(priv->dev,
				"[SER] Failed to enable ISX031 sensor: %d\n", ret);
			goto err_pipe_off;
		}
	} else {
		dev_err(priv->dev, "[SER] No sensor connected in sources[0]\n");
		ret = -ENODEV;
		goto err_pipe_off;
	}

	if (ser->ops->log_status) {
		dev_info(priv->dev, "[SER] status after sensor enable:\n");
		ser->ops->log_status(ser);
	}

	/* 6) 廠商 kick：維持目前小改版本 */
	ret = max_ser_vendor_kick_after_sensor(priv);
	if (ret) {
		dev_err(priv->dev, "[SER] vendor kick failed: %d\n", ret);
		goto err_sensor_off;
	}

	if (ser->ops->log_status) {
		dev_info(priv->dev, "[SER] status after vendor kick (t=0ms):\n");
		ser->ops->log_status(ser);

		msleep(20);
		dev_info(priv->dev, "[SER] status 20ms after vendor kick:\n");
		ser->ops->log_status(ser);

		msleep(30); /* cumulative 50ms */
		dev_info(priv->dev, "[SER] status 50ms after vendor kick:\n");
		ser->ops->log_status(ser);

		msleep(50); /* cumulative 100ms */
		dev_info(priv->dev, "[SER] status 100ms after vendor kick:\n");
		ser->ops->log_status(ser);

		msleep(100); /* cumulative 200ms */
		dev_info(priv->dev, "[SER] status 200ms after vendor kick:\n");
		ser->ops->log_status(ser);

		msleep(300); /* cumulative 500ms */
		dev_info(priv->dev, "[SER] status 500ms after vendor kick:\n");
		ser->ops->log_status(ser);
	}

	dev_info(priv->dev, "[SER] enable_streams done\n");
	return 0;

err_sensor_off:
	if (priv->sources && priv->sources[0].sd)
		v4l2_subdev_disable_streams(priv->sources[0].sd,
					    priv->sources[0].pad,
					    BIT_ULL(0));
err_pipe_off:
	if (ser->ops->set_pipe_enable)
		ser->ops->set_pipe_enable(ser, pipe, false);
err_phy_off:
	if (ser->ops->set_phy_active)
		ser->ops->set_phy_active(ser, phy, false);
	return ret;
}

static int max_ser_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 pad, u64 streams_mask)
{
	struct max_ser_priv *priv = v4l2_get_subdevdata(sd);
	struct max_ser *ser = priv->ser;
	struct max_ser_phy *phy;
	struct max_ser_pipe *pipe;
	int ret = 0;

	if (!ser || !ser->ops)
		return -EINVAL;

	phy  = &ser->phys[0];
	pipe = &ser->pipes[0];

	dev_info(priv->dev, "[SER] disable_streams pad=%u mask=0x%llx\n",
		 pad, streams_mask);

	/* 1) 先關 serializer pipe */
	if (ser->ops->set_pipe_enable) {
		ret = ser->ops->set_pipe_enable(ser, pipe, false);
		if (ret)
			dev_err(priv->dev, "[SER] set_pipe_enable(false) failed: %d\n", ret);
	}

	/* 2) 再關 sensor */
	if (priv->sources && priv->sources[0].sd) {
		int r2;

		dev_info(priv->dev, "[SER] Disabling ISX031 sensor...\n");
		r2 = v4l2_subdev_disable_streams(priv->sources[0].sd,
						 priv->sources[0].pad,
						 BIT_ULL(0));
		if (r2) {
			dev_err(priv->dev,
				"[SER] sensor disable_streams failed: %d\n", r2);
			if (!ret)
				ret = r2;
		}
	}

	/* 3) 最後關 serializer 的 MIPI RX */
	if (ser->ops->set_phy_active) {
		int r3 = ser->ops->set_phy_active(ser, phy, false);
		if (r3) {
			dev_err(priv->dev, "[SER] set_phy_active(false) failed: %d\n", r3);
			if (!ret)
				ret = r3;
		}
	}

	phy->enabled = false;
	pipe->enabled = false;

	return ret;
}

static const struct v4l2_subdev_core_ops max_ser_core_ops = {
	.log_status = max_ser_log_status,
};

static const struct v4l2_ctrl_ops max_ser_ctrl_ops = {
	.s_ctrl = max_ser_s_ctrl,
};

static const struct v4l2_subdev_pad_ops max_ser_pad_ops = {
	/* [新增] 綁定 Streams API */
	.enable_streams = max_ser_enable_streams,
	.disable_streams = max_ser_disable_streams,
	
	.set_routing = max_ser_set_routing,
	.get_frame_desc = max_ser_get_frame_desc,
	.get_mbus_config = max_ser_get_mbus_config,
	.get_fmt = max_ser_get_fmt,
	.set_fmt = max_ser_set_fmt,
};
static const struct v4l2_subdev_ops max_ser_subdev_ops = {
	.core = &max_ser_core_ops,
	.pad = &max_ser_pad_ops,
};

static const struct v4l2_subdev_internal_ops max_ser_internal_ops = {
	.init_state = &max_ser_init_state,
};

static const struct media_entity_operations max_ser_media_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.has_pad_interdep = v4l2_subdev_has_pad_interdep,
	.link_validate = v4l2_subdev_link_validate,
};

static int __maybe_unused max_ser_init(struct max_ser_priv *priv)
{
	struct max_ser *ser = priv->ser;
	int ret;
	if (ser->ops->init) {
		ret = ser->ops->init(ser);
		if (ret) return ret;
	}
    return 0;
}

static int max_ser_notify_bound(struct v4l2_async_notifier *nf,
				struct v4l2_subdev *subdev,
				struct v4l2_async_connection *base_asc)
{
	struct max_ser_priv *priv = nf_to_priv(nf);
	struct max_serdes_asc *asc;
	struct max_serdes_source *source;
	u32 sink_pad;
	u16 remote_pad = 0;
	int i, ret;

	if (!base_asc || !subdev) {
		dev_err(priv->dev, "[SER] notify_bound invalid args\n");
		return -EINVAL;
	}

	asc = asc_to_max(base_asc);
	source = asc ? asc->source : NULL;
	if (!source) {
		dev_err(priv->dev, "[SER] notify_bound: asc->source is NULL for %s\n",
			subdev->name);
		return -EINVAL;
	}

	sink_pad = source->index;

	if (sink_pad >= priv->sd.entity.num_pads) {
		dev_err(priv->dev,
			"[SER] notify_bound: sink_pad=%u out of range (num_pads=%u)\n",
			sink_pad, priv->sd.entity.num_pads);
		return -EINVAL;
	}

	for (i = 0; i < subdev->entity.num_pads; i++) {
		if (subdev->entity.pads[i].flags & MEDIA_PAD_FL_SOURCE) {
			remote_pad = i;
			break;
		}
	}

	if (!(subdev->entity.pads[remote_pad].flags & MEDIA_PAD_FL_SOURCE)) {
		dev_warn(priv->dev,
			 "[SER] %s has no explicit SOURCE pad, fallback to pad 0\n",
			 subdev->name);
		remote_pad = 0;
	}

	source->sd = subdev;
	source->pad = remote_pad;

	dev_info(priv->dev, "[SER] Bound subdev %s:%u -> %s:%u\n",
		 subdev->name, remote_pad, priv->sd.name, sink_pad);

	ret = media_create_pad_link(&subdev->entity, remote_pad,
				    &priv->sd.entity, sink_pad,
				    MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);

	if (ret == -EEXIST) {
		dev_warn(priv->dev, "[SER] link already exists, ignore\n");
		ret = 0;
	}

	if (ret) {
		dev_err(priv->dev,
			"[SER] media_create_pad_link failed: %d\n", ret);
		return ret;
	}

	if (priv->sd.v4l2_dev) {
		int err;

		dev_info(priv->dev,
			 "[SER] register subdev nodes for hotplugged sensor\n");
		err = v4l2_device_register_subdev_nodes(priv->sd.v4l2_dev);
		if (err)
			dev_warn(priv->dev,
				 "[SER] Failed to register nodes: %d\n", err);
	}

	return 0;
}

static void max_ser_notify_unbind(struct v4l2_async_notifier *nf,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_connection *base_asc)
{
	struct max_serdes_asc *asc = base_asc ? asc_to_max(base_asc) : NULL;
	if (asc && asc->source)
		asc->source->sd = NULL;
}

static const struct v4l2_async_notifier_operations max_ser_notify_ops = {
	.bound = max_ser_notify_bound,
	.unbind = max_ser_notify_unbind,
};

static int max_ser_v4l2_notifier_register(struct max_ser_priv *priv)
{
	struct max_ser *ser = priv->ser;
	unsigned int i;
	int ret;

	v4l2_async_subdev_nf_init(&priv->nf, &priv->sd);

	for (i = 0; i < ser->ops->num_phys; i++) {
		struct max_ser_phy *phy = &ser->phys[i];
		struct max_serdes_source *source;
		struct max_serdes_asc *asc = NULL;

		source = max_ser_get_phy_source(priv, phy);
		if (!source) {
			dev_err(priv->dev, "[SER] source is NULL for phy %u\n", i);
			v4l2_async_nf_cleanup(&priv->nf);
			return -EINVAL;
		}

		source->index = i;
		source->sd = NULL;
		source->pad = 0;

		/*
		 * 手動 bring-up 主線：
		 * 只要 serializer 自己的 child mux bus 已存在，就優先等 i2c waiter。
		 */
		if (priv->mux && priv->mux->adapter[0]) {
			struct i2c_adapter *mux_adap = priv->mux->adapter[0];

			dev_info(priv->dev,
				 "[SER] waiting for ISX031 (0x1a) on Mux Bus %d\n",
				 mux_adap->nr);

			asc = v4l2_async_nf_add_i2c(&priv->nf, mux_adap->nr, 0x1a,
						    struct max_serdes_asc);
			if (IS_ERR(asc)) {
				dev_err(priv->dev,
					"[SER] add_i2c waiter failed on bus %d: %ld\n",
					mux_adap->nr, PTR_ERR(asc));
				v4l2_async_nf_cleanup(&priv->nf);
				return PTR_ERR(asc);
			}

			asc->source = source;
			break; /* 單顆 sensor */
		}

		/*
		 * 備援：沒有 mux child bus 時才退回 fwnode
		 */
		if (source->ep_fwnode) {
			asc = v4l2_async_nf_add_fwnode(&priv->nf, source->ep_fwnode,
						       struct max_serdes_asc);
			if (IS_ERR(asc)) {
				dev_err(priv->dev,
					"[SER] add_fwnode waiter failed for phy %u: %ld\n",
					i, PTR_ERR(asc));
				v4l2_async_nf_cleanup(&priv->nf);
				return PTR_ERR(asc);
			}

			asc->source = source;
			dev_info(priv->dev,
				 "[SER] fallback: waiting for sensor via fwnode on phy %u\n", i);
			continue;
		}

		dev_warn(priv->dev,
			 "[SER] no child mux adapter and no fwnode for phy %u\n", i);
	}

	priv->nf.ops = &max_ser_notify_ops;

	ret = v4l2_async_nf_register(&priv->nf);
	if (ret) {
		dev_err(priv->dev, "[SER] Failed to register subdev notifier: %d\n", ret);
		v4l2_async_nf_cleanup(&priv->nf);
		return ret;
	}

	return 0;
}

static void max_ser_v4l2_notifier_unregister(struct max_ser_priv *priv)
{
	v4l2_async_nf_unregister(&priv->nf);
	v4l2_async_nf_cleanup(&priv->nf);
}

static int max_ser_v4l2_register(struct max_ser_priv *priv)
{
	struct v4l2_subdev *sd = &priv->sd;
	struct max_ser *ser = priv->ser;
	unsigned int num_pads = max_ser_num_pads(ser);
	unsigned int i;
	int ret;
	struct fwnode_handle *fwnode = dev_fwnode(priv->dev);

	v4l2_i2c_subdev_init(sd, priv->client, &max_ser_subdev_ops);
	sd->internal_ops = &max_ser_internal_ops;
	
	/* [借鏡 1] 宣告這是一個 Video Bridge，幫助 media-ctl 辨識 */
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &max_ser_media_ops;

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS |
		     V4L2_SUBDEV_FL_STREAMS;
	snprintf(sd->name, sizeof(sd->name), "%s %s",
		 ser->ops->tpg_mode == MAX_SERDES_GMSL_TUNNEL_MODE ?
		 "tun" : "ser", dev_name(priv->dev));

	v4l2_set_subdevdata(sd, priv);

	if (ser->ops->set_vc_remap) {
		ret = v4l2_ctrl_handler_init(&priv->ctrl_handler, 1);
		if (ret) return ret;
		v4l2_ctrl_new_std(&priv->ctrl_handler, &max_ser_ctrl_ops,
				  V4L2_CID_TEST_PATTERN, 0,
				  ser->ops->tpg_patterns, 0, 0);
		if (priv->ctrl_handler.error) {
			ret = priv->ctrl_handler.error;
			goto err_free_ctrl;
		}
		sd->ctrl_handler = &priv->ctrl_handler;
	}

	priv->pads = devm_kcalloc(priv->dev, num_pads, sizeof(*priv->pads), GFP_KERNEL);
	if (!priv->pads) {
		ret = -ENOMEM;
		goto err_free_ctrl;
	}

	for (i = 0; i < ser->ops->num_phys; i++)
		priv->pads[i].flags = MEDIA_PAD_FL_SINK;
	for (; i < ser->ops->num_phys + MAX_SER_NUM_LINKS; i++)
		priv->pads[i].flags = MEDIA_PAD_FL_SOURCE;
	if (ser->ops->set_tpg)
		priv->pads[i].flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_INTERNAL;

	ret = media_entity_pads_init(&sd->entity, num_pads, priv->pads);
	if (ret) goto err_free_ctrl;

	/* [Patch] Always register Notifier if Deserializer (to find Serializer) */
	ret = max_ser_v4l2_notifier_register(priv);
	if (ret)
		goto err_media_entity_cleanup;

	/* ========================================================= */
	/* [借鏡 2] 呼叫 finalize 配置 active_state 讓格式生效！     */
	/* ========================================================= */
	ret = v4l2_subdev_init_finalize(sd);
	if (ret) goto err_nf_cleanup;

	ret = v4l2_async_register_subdev(sd);
	if (ret) goto err_sd_cleanup;

	return 0;

/* [借鏡 3] 補上完整的錯誤清理路徑 */
err_sd_cleanup:
	v4l2_subdev_cleanup(sd);
err_nf_cleanup:
	if (fwnode) max_ser_v4l2_notifier_unregister(priv);
err_media_entity_cleanup:
	media_entity_cleanup(&sd->entity);
err_free_ctrl:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return ret;
}

/* 在 max_ser.c 中 */
static void max_ser_v4l2_unregister(struct max_ser_priv *priv)
{
	struct v4l2_subdev *sd = &priv->sd;

	/* [修復] 必須檢查 fwnode，有註冊過 Notifier 才可以解除註冊 */
	if (dev_fwnode(priv->dev)) {
		max_ser_v4l2_notifier_unregister(priv);
	}

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
}

static int max_ser_parse_sink_dt_endpoint(struct max_ser_priv *priv,
					  struct max_ser_phy *phy,
					  struct max_serdes_source *source,
					  struct fwnode_handle *fwnode)
{
	struct max_ser *ser = priv->ser;
	u32 pad = max_ser_phy_to_pad(ser, phy);
	struct v4l2_fwnode_endpoint v4l2_ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(fwnode, pad, 0, 0);
	if (!ep) return 0;

	source->ep_fwnode = fwnode_graph_get_remote_endpoint(ep);
	fwnode_handle_put(ep);
	if (!source->ep_fwnode) {
		dev_dbg(priv->dev, "No remote endpoint on port %u\n", pad);
		return 0; 
	}

	ret = v4l2_fwnode_endpoint_parse(ep, &v4l2_ep);
	if (ret) return ret;

	phy->mipi = v4l2_ep.bus.mipi_csi2;
	phy->enabled = true;
	return 0;
}

static int max_ser_find_phys_config(struct max_ser_priv *priv)
{
	struct max_ser *ser = priv->ser;
	ser->phys_config = 0;
	return 0;
}

static int max_ser_parse_dt(struct max_ser_priv *priv)
{
	struct fwnode_handle *fwnode = dev_fwnode(priv->dev);
	struct max_ser *ser = priv->ser;
	struct max_ser_pipe *pipe;
	struct max_ser_phy *phy;
	unsigned int i;
	int ret;

	for (i = 0; i < ser->ops->num_phys; i++) {
		phy = &ser->phys[i];
		phy->index = i;
	}
	for (i = 0; i < ser->ops->num_pipes; i++) {
		pipe = &ser->pipes[i];
		pipe->index = i;
		pipe->phy_id = i % ser->ops->num_phys;
		pipe->stream_id = i % MAX_SERDES_STREAMS_NUM;
	}

	if (!fwnode) {
        if (ser->ops->num_phys > 0) {
            phy = &ser->phys[0];
            phy->enabled = true;
            phy->mipi.num_data_lanes = 4;
            phy->mipi.data_lanes[0] = 1; phy->mipi.data_lanes[1] = 2;
            phy->mipi.data_lanes[2] = 3; phy->mipi.data_lanes[3] = 4;
        }
        return 0;
    }

	for (i = 0; i < ser->ops->num_phys; i++) {
		struct max_ser_phy *phy = &ser->phys[i];
		struct max_serdes_source *source;
		source = max_ser_get_phy_source(priv, phy);
		source->index = i;
		ret = max_ser_parse_sink_dt_endpoint(priv, phy, source, fwnode);
		if (ret) return ret;
	}
	return max_ser_find_phys_config(priv);
}

static int max_ser_allocate(struct max_ser_priv *priv)
{
	struct max_ser *ser = priv->ser;
	unsigned int num_pads = max_ser_num_pads(ser);

	ser->phys = devm_kcalloc(priv->dev, ser->ops->num_phys, sizeof(*ser->phys), GFP_KERNEL);
	if (!ser->phys) return -ENOMEM;
	ser->pipes = devm_kcalloc(priv->dev, ser->ops->num_pipes, sizeof(*ser->pipes), GFP_KERNEL);
	if (!ser->pipes) return -ENOMEM;
	ser->vc_remaps = devm_kcalloc(priv->dev, ser->ops->num_vc_remaps, sizeof(*ser->vc_remaps), GFP_KERNEL);
	if (!ser->vc_remaps) return -ENOMEM;
	ser->i2c_xlates = devm_kcalloc(priv->dev, ser->ops->num_i2c_xlates, sizeof(*ser->i2c_xlates), GFP_KERNEL);
	if (!ser->i2c_xlates) return -ENOMEM;
	priv->sources = devm_kcalloc(priv->dev, ser->ops->num_phys, sizeof(*priv->sources), GFP_KERNEL);
	if (!priv->sources) return -ENOMEM;
	priv->streams_masks = devm_kcalloc(priv->dev, num_pads, sizeof(*priv->streams_masks), GFP_KERNEL);
	if (!priv->streams_masks) return -ENOMEM;
	return 0;
}

int max_ser_probe(struct i2c_client *client, struct max_ser *ser)
{
	struct device *dev = &client->dev;
	struct max_ser_priv *priv;
	int ret;

	if (ser->ops->num_phys > MAX_SER_NUM_PHYS) return -E2BIG;
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) return -ENOMEM;

	priv->client = client;
	priv->dev = dev;
	priv->ser = ser;
	ser->priv = priv;
	ser->mode = __ffs(ser->ops->modes);

	ret = max_ser_allocate(priv);
	if (ret) return ret;
	ret = max_ser_parse_dt(priv);
	if (ret) return ret;

    ret = max_ser_init(priv);
	if (ret) return ret;

	ret = max_ser_i2c_adapter_init(priv);
	if (ret) return ret;
	ret = max_ser_v4l2_register(priv);
	if (ret) goto err_i2c_adapter_deinit;
	return 0;

err_i2c_adapter_deinit:
	max_ser_i2c_adapter_deinit(priv);
	return ret;
}
EXPORT_SYMBOL_NS_GPL(max_ser_probe, "MAX_SERDES");

int max_ser_remove(struct max_ser *ser)
{
	struct max_ser_priv *priv = ser->priv;
	max_ser_v4l2_unregister(priv);
	max_ser_i2c_adapter_deinit(priv);
	return 0;
}
EXPORT_SYMBOL_NS_GPL(max_ser_remove, "MAX_SERDES");

int max_ser_set_double_bpps(struct v4l2_subdev *sd, u32 double_bpps) { return 0; }
int max_ser_set_stream_id(struct v4l2_subdev *sd, unsigned int stream_id) { return 0; }
int max_ser_get_stream_id(struct v4l2_subdev *sd, unsigned int *stream_id) { return 0; }
bool max_ser_supports_vc_remap(struct v4l2_subdev *sd) { return false; }
int max_ser_set_mode(struct v4l2_subdev *sd, enum max_serdes_gmsl_mode mode) { return 0; }
int max_ser_set_vc_remaps(struct v4l2_subdev *sd, struct max_serdes_vc_remap *vc_remaps, int num_vc_remaps) { return 0; }


unsigned int max_ser_get_supported_modes(struct v4l2_subdev *sd)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	if (!priv || !priv->ser || !priv->ser->ops)
		return 0;
	return priv->ser->ops->modes;
}

static int max_ser_read_reg(struct i2c_adapter *adapter, u8 addr,
			    u16 reg, u8 *val)
{
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msg[2] = {
		{ .addr = addr, .flags = 0, .buf = buf, .len = sizeof(buf) },
		{ .addr = addr, .flags = I2C_M_RD, .buf = buf, .len = 1 },
	};
	int ret = i2c_transfer(adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) return ret;
	*val = buf[0];
	return 0;
}

static int max_ser_write_reg(struct i2c_adapter *adapter, u8 addr,
			     u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	struct i2c_msg msg[1] = {
		{ .addr = addr, .flags = 0, .buf = buf, .len = sizeof(buf) },
	};
	return i2c_transfer(adapter, msg, ARRAY_SIZE(msg));
}

int max_ser_reset(struct i2c_adapter *adapter, u8 addr)
{
	int ret;
	u8 val;
	ret = max_ser_read_reg(adapter, addr, MAX_SER_CTRL0, &val);
	if (ret) return ret;
	val |= MAX_SER_CTRL0_RESET_ALL;
	return max_ser_write_reg(adapter, addr, MAX_SER_CTRL0, val);
}
EXPORT_SYMBOL_NS_GPL(max_ser_reset, "MAX_SERDES");

int max_ser_wait_for_multiple(struct i2c_adapter *adapter, u8 *addrs,
			      unsigned int num_addrs, u8 *current_addr)
{
	unsigned int i, j;
	int ret = 0;
	u8 val;
	for (i = 0; i < 10; i++) {
		for (j = 0; j < num_addrs; j++) {
			ret = max_ser_read_reg(adapter, addrs[j], MAX_SER_REG0, &val);
			if (!ret && val) {
				*current_addr = addrs[j];
				return 0;
			}
			msleep(100);
		}
	}
	return ret;
}
EXPORT_SYMBOL_NS_GPL(max_ser_wait_for_multiple, "MAX_SERDES");

int max_ser_wait(struct i2c_adapter *adapter, u8 addr)
{
	u8 current_addr;
	return max_ser_wait_for_multiple(adapter, &addr, 1, &current_addr);
}
EXPORT_SYMBOL_NS_GPL(max_ser_wait, "MAX_SERDES");

int max_ser_fix_tx_ids(struct i2c_adapter *adapter, u8 addr)
{
	unsigned int addr_regs[] = {
		MAX_SER_CFGI_INFOFR_TR3, MAX_SER_CFGL_SPI_TR3,
		MAX_SER_CFGC_CC_TR3, MAX_SER_CFGC_GPIO_TR3,
		MAX_SER_CFGL_IIC_X_TR3, MAX_SER_CFGL_IIC_Y_TR3,
	};
	unsigned int i;
	int ret;
	for (i = 0; i < ARRAY_SIZE(addr_regs); i++) {
		ret = max_ser_write_reg(adapter, addr, addr_regs[i], addr);
		if (ret) return ret;
	}
	return 0;
}
EXPORT_SYMBOL_NS_GPL(max_ser_fix_tx_ids, "MAX_SERDES");

int max_ser_change_address(struct i2c_adapter *adapter, u8 addr, u8 new_addr)
{
	u8 val = FIELD_PREP(MAX_SER_REG0_DEV_ADDR, new_addr);
	return max_ser_write_reg(adapter, addr, MAX_SER_REG0, val);
}
EXPORT_SYMBOL_NS_GPL(max_ser_change_address, "MAX_SERDES");

MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("I2C_ATR");
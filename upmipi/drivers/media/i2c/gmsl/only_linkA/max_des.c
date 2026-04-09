// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim GMSL2 Deserializer Driver (V4L2 Pad Architecture Aligned with Vendor)
 * V4L2 Framework & Software Logic with Detailed Logging
 */

#include <linux/delay.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

#include "max_des.h"
#include "max_ser.h"
#include "max_serdes.h"

#ifndef MEDIA_PAD_FL_INTERNAL
#define MEDIA_PAD_FL_INTERNAL 0
#endif

#define DES_RATE_MODE_1P5G  0
#define DES_RATE_MODE_1P0G  1

#ifndef DES_TEST_RATE_MODE
#define DES_TEST_RATE_MODE DES_RATE_MODE_1P5G
#endif

#if DES_TEST_RATE_MODE == DES_RATE_MODE_1P0G
#define MAX_DES_LINK_FREQUENCY_DEFAULT 500000000ULL
#elif DES_TEST_RATE_MODE == DES_RATE_MODE_1P5G
#define MAX_DES_LINK_FREQUENCY_DEFAULT 750000000ULL
#else
#error "Unsupported DES_TEST_RATE_MODE"
#endif

#define TEST_LINK_ID 0

#define DES_PAD_SOURCE 0
#define DES_PAD_SINK   1
#define DES_NUM_PADS   2

struct max_des_priv {
	struct max_des *des;
	struct device *dev;
	struct i2c_client *client;
	struct i2c_mux_core *mux;

	struct media_pad *pads;
	struct regulator **pocs;
	struct max_serdes_source *sources;

	struct v4l2_subdev sd;
	struct v4l2_async_notifier nf;
	struct v4l2_ctrl_handler ctrl_handler;
};

static inline struct max_des_priv *sd_to_priv(struct v4l2_subdev *sd)
{
	return container_of(sd, struct max_des_priv, sd);
}

static inline struct max_des_priv *nf_to_priv(struct v4l2_async_notifier *nf)
{
	return container_of(nf, struct max_des_priv, nf);
}

static struct max_des_phy *max_des_pad_to_phy(struct max_des *des, u32 pad)
{
	/*
	 * 目前固定單路:
	 * Link0 / Pipe0 / Stream0 -> PHY1 (2-lane)
	 *
	 * 不再以 4-lane PortA (PHY1+PHY0) 為前提。
	 */
	return &des->phys[1];
}
static struct max_serdes_source *max_des_get_link_source(struct max_des_priv *priv,
							 struct max_des_link *link)
{
	return &priv->sources[link->index];
}

static int max_des_i2c_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
	return 0;
}

static int max_des_i2c_mux_init(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	u32 flags = I2C_MUX_LOCKED;
	unsigned int i;
	int ret;

	dev_info(priv->dev, "[DES-PROBE] 3. Registering I2C MUX for GMSL links...\n");

	priv->mux = i2c_mux_alloc(priv->client->adapter, priv->dev,
				  des->ops->num_links, 0, flags,
				  max_des_i2c_mux_select, NULL);
	if (!priv->mux) return -ENOMEM;

	priv->mux->priv = priv;

	for (i = 0; i < des->ops->num_links; i++) {
		if (!des->links[i].enabled) continue;
		
		dev_info(priv->dev, "[DES-PROBE] 3a. Adding MUX adapter for Link %u...\n", i);
		ret = i2c_mux_add_adapter(priv->mux, 0, i);
		if (ret) goto err_add_adapters;

		if (priv->mux->adapter[i]) {
			struct i2c_board_info ser_info = { I2C_BOARD_INFO("max96717", 0x40) };
			dev_info(priv->dev, "[DES-PROBE] 3b. Auto-probing MAX96717 at 0x40 on Link %u...\n", i);
			i2c_new_client_device(priv->mux->adapter[i], &ser_info);
		}
	}
	return 0;

err_add_adapters:
	dev_err(priv->dev, "[DES-PROBE] Failed to add MUX adapter for link %d\n", i);
	i2c_mux_del_adapters(priv->mux);
	return ret;
}

static void max_des_fill_default_fmt(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = 1920;
	fmt->height = 1536;
	fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

static int max_des_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[1] = { 0 };
	struct v4l2_subdev_krouting routing = { .routes = routes, .num_routes = 1 };
	struct v4l2_mbus_framefmt *fmt;
	int i, ret;

	/* 將 V4L2 內部路由設定為 Pad 1 (Sink) -> Pad 0 (Source) */
	routes[0].sink_pad = DES_PAD_SINK;
	routes[0].sink_stream = 0;
	routes[0].source_pad = DES_PAD_SOURCE;
	routes[0].source_stream = 0;
	routes[0].flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;

	ret = v4l2_subdev_set_routing(sd, state, &routing);
	if (ret) return ret;

	for (i = 0; i < DES_NUM_PADS; i++) {
		fmt = v4l2_subdev_state_get_format(state, i, 0);
		if (fmt) max_des_fill_default_fmt(fmt);
	}
	return 0;
}

static int max_des_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *f = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (f && f->width && f->height && f->code) {
		format->format = *f;
		return 0;
	}
	max_des_fill_default_fmt(&format->format);
	if (f) *f = format->format;
	return 0;
}

static int max_des_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *f;
	if (!format->format.width || !format->format.height || !format->format.code)
		max_des_fill_default_fmt(&format->format);
	f = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (f) *f = format->format;
	return 0;
}

static int max_des_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	if (pad != DES_PAD_SOURCE) {
		fd->num_entries = 0;
		return 0;
	}

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

static int max_des_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				   struct v4l2_mbus_config *cfg)
{
	struct max_des_priv *priv = sd_to_priv(sd);
	struct max_des_phy *phy = max_des_pad_to_phy(priv->des, pad);

	if (!phy) return -EINVAL;
	cfg->type = V4L2_MBUS_CSI2_DPHY; /* 強制覆寫避免未初始化錯誤 */
	cfg->bus.mipi_csi2 = phy->mipi;
	return 0;
}

static int max_des_set_routing(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			       enum v4l2_subdev_format_whence which,
			       struct v4l2_subdev_krouting *routing)
{
	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && sd_to_priv(sd)->des->active)
		return -EBUSY;
	return v4l2_subdev_set_routing(sd, state, routing);
}

static int max_des_set_pipe_remaps(struct max_des_priv *priv, struct max_des_pipe *pipe,
				   struct max_des_remap *remaps, unsigned int num_remaps)
{
	struct max_des *des = priv->des;
	unsigned int mask = 0;
	int i, ret;

	if (!des->ops->set_pipe_remap) return 0;

	for (i = 0; i < num_remaps; i++) {
		dev_info(priv->dev, "[DES-STREAM] Remap %d: DT 0x%02x -> DT 0x%02x (Target PHY %u)\n", 
			 i, remaps[i].from_dt, remaps[i].to_dt, remaps[i].phy);
		ret = des->ops->set_pipe_remap(des, pipe, i, &remaps[i]);
		if (ret) return ret;
		mask |= BIT(i);
	}
	dev_info(priv->dev, "[DES-STREAM] Enabling remaps with mask 0x%02x\n", mask);
	return des->ops->set_pipe_remaps_enable(des, pipe, mask);
}

static int max_des_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct max_des_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;
	struct max_des *des = priv->des;
	struct max_serdes_source *source = &priv->sources[TEST_LINK_ID];
	int ret = 0;

	dev_info(priv->dev, "\n============================================\n");
	dev_info(priv->dev, "[DES-STREAM] s_stream(%d) Sequence START\n", enable);

	if (enable) {
		if (pm_runtime_enabled(&client->dev)) {
			ret = pm_runtime_resume_and_get(&client->dev);
			if (ret < 0)
				return ret;
		}

		if (des->ops->stream_prepare) {
			ret = des->ops->stream_prepare(des);
			if (ret) {
				dev_err(priv->dev,
					"[DES-STREAM] stream_prepare failed: %d\n", ret);
				goto err_pm_put;
			}
		}

		if (!source->sd) {
			dev_err(priv->dev,
				"[DES-STREAM] No remote source bound on link %u\n",
				TEST_LINK_ID);
			ret = -ENODEV;
			goto err_pm_put;
		}

		ret = v4l2_subdev_enable_streams(source->sd, source->pad, BIT_ULL(0));
		if (ret && ret != -EALREADY) {
			dev_err(priv->dev,
				"[DES-STREAM] Remote source enable_streams failed: %d\n",
				ret);
			goto err_pm_put;
		}

		des->active = true;
		dev_info(priv->dev,
			 "[DES-STREAM] Remote source started successfully.\n");
		dev_info(priv->dev, "============================================\n\n");
		return 0;
	}

	if (source->sd)
		v4l2_subdev_disable_streams(source->sd, source->pad, BIT_ULL(0));

	des->active = false;

	if (pm_runtime_enabled(&client->dev))
		pm_runtime_put(&client->dev);

	dev_info(priv->dev, "[DES-STREAM] Stop sequence complete.\n");
	dev_info(priv->dev, "============================================\n\n");
	return 0;

err_pm_put:
	if (pm_runtime_enabled(&client->dev))
		pm_runtime_put(&client->dev);
	return ret;
}

static int max_des_enable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, u32 pad, u64 streams_mask)
{
	return max_des_s_stream(sd, 1);
}

static int max_des_disable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, u32 pad, u64 streams_mask)
{
	return max_des_s_stream(sd, 0);
}

/* 呼叫底層的詳細硬體暫存器狀態印出 */
static int max_des_log_status(struct v4l2_subdev *sd)
{
	struct max_des_priv *priv = v4l2_get_subdevdata(sd);
	struct max_des *des = priv->des;

	if (des->ops->log_status)
		des->ops->log_status(des);

	return 0;
}

static const struct v4l2_subdev_video_ops max_des_video_ops = { .s_stream = max_des_s_stream };
static const struct v4l2_subdev_core_ops max_des_core_ops = { .log_status = max_des_log_status };
static const struct v4l2_subdev_pad_ops max_des_pad_ops = {
	.enable_streams = max_des_enable_streams,
	.disable_streams = max_des_disable_streams,
	.set_routing = max_des_set_routing,
	.get_frame_desc = max_des_get_frame_desc,
	.get_mbus_config = max_des_get_mbus_config,
	.get_fmt = max_des_get_fmt,
	.set_fmt = max_des_set_fmt,
};
static const struct v4l2_subdev_ops max_des_subdev_ops = {
	.core = &max_des_core_ops,
	.pad = &max_des_pad_ops,
	.video = &max_des_video_ops,
};
static const struct v4l2_subdev_internal_ops max_des_internal_ops = {
	.init_state = &max_des_init_state,
};

static const struct media_entity_operations max_des_media_ops = {
	/* 回歸 V4L2 標準 1_to_1 對接：ACPI Port 0 自動對接 V4L2 Pad 0 */
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.has_pad_interdep = v4l2_subdev_has_pad_interdep,
	.link_validate = v4l2_subdev_link_validate,
};

static int max_des_notify_bound(struct v4l2_async_notifier *nf, struct v4l2_subdev *subdev,
				struct v4l2_async_connection *base_asc)
{
	struct max_des_priv *priv = nf_to_priv(nf);
	struct max_serdes_asc *asc = asc_to_max(base_asc);
	struct max_serdes_source *source = asc->source;
	int src_pad = 1; 
	int ret;

	source->sd = subdev;
	source->pad = src_pad;

	dev_info(priv->dev, "[DES-V4L2] Bound remote subdev %s:%d -> local %s:SinkPad(%u)\n",
		 subdev->name, src_pad, priv->sd.name, DES_PAD_SINK);

	/* 將 Serializer 綁定到 Deserializer 的 Sink Pad (Pad 1) */
	ret = media_create_pad_link(&subdev->entity, src_pad, &priv->sd.entity, DES_PAD_SINK,
			      MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		dev_warn(priv->dev, "[DES-V4L2] Failed to create pad link: %d\n", ret);
	
	if (priv->sd.v4l2_dev) {
		ret = v4l2_device_register_subdev_nodes(priv->sd.v4l2_dev);
		if (ret)
			dev_warn(priv->dev, "[DES-V4L2] Failed to register subdev nodes: %d\n", ret);
	}

	return 0;
}

static void max_des_notify_unbind(struct v4l2_async_notifier *nf, struct v4l2_subdev *subdev,
				  struct v4l2_async_connection *base_asc)
{
	struct max_serdes_asc *asc = asc_to_max(base_asc);
	asc->source->sd = NULL;
}

static const struct v4l2_async_notifier_operations max_des_notify_ops = {
	.bound = max_des_notify_bound,
	.unbind = max_des_notify_unbind,
};

/* 【修正】補上 static 宣告 */
static int max_des_v4l2_notifier_register(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	struct max_serdes_source *source = max_des_get_link_source(priv, &des->links[TEST_LINK_ID]);
	struct max_serdes_asc *asc;

	dev_info(priv->dev,
		 "[DES-PROBE] 5. Registering V4L2 Async Notifier for remote sensor on Link %u...\n",
		 TEST_LINK_ID);

	v4l2_async_subdev_nf_init(&priv->nf, &priv->sd);
	source->index = TEST_LINK_ID;

	if (priv->mux && priv->mux->adapter[TEST_LINK_ID]) {
		asc = v4l2_async_nf_add_i2c(&priv->nf,
					    priv->mux->adapter[TEST_LINK_ID]->nr,
					    0x40,
					    struct max_serdes_asc);
		if (!IS_ERR(asc))
			asc->source = source;
	}

	priv->nf.ops = &max_des_notify_ops;
	return v4l2_async_nf_register(&priv->nf);
}

static int max_des_v4l2_register(struct max_des_priv *priv)
{
	struct v4l2_subdev *sd = &priv->sd;
	struct v4l2_ctrl *ctrl;
	int ret;

	dev_info(priv->dev, "[DES-PROBE] 4. Initializing V4L2 Subdev and Media Pads...\n");

	v4l2_i2c_subdev_init(sd, priv->client, &max_des_subdev_ops);
	sd->internal_ops = &max_des_internal_ops;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &max_des_media_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	priv->pads = devm_kcalloc(priv->dev, DES_NUM_PADS, sizeof(*priv->pads), GFP_KERNEL);
	if (!priv->pads) return -ENOMEM;

	/* 套用極簡 Pad 架構 */
	priv->pads[DES_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	priv->pads[DES_PAD_SINK].flags = MEDIA_PAD_FL_SINK;

	v4l2_set_subdevdata(sd, priv);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, 3);
	priv->sd.ctrl_handler = &priv->ctrl_handler;

	ctrl = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_PIXEL_RATE, 187500000, 187500000, 1, 187500000);
	if (ctrl) ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	static const s64 link_freq_menu_items[] = { MAX_DES_LINK_FREQUENCY_DEFAULT }; 
	ctrl = v4l2_ctrl_new_int_menu(&priv->ctrl_handler, NULL, V4L2_CID_LINK_FREQ, 0, 0, link_freq_menu_items);
	if (ctrl) ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (priv->ctrl_handler.error) {
		dev_err(priv->dev, "[DES-PROBE] Ctrl handler error: %d\n", priv->ctrl_handler.error);
		return priv->ctrl_handler.error;
	}

	ret = media_entity_pads_init(&sd->entity, DES_NUM_PADS, priv->pads);
	if (ret) return ret;

	ret = max_des_v4l2_notifier_register(priv);
	if (ret) return ret;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret) return ret;

	return v4l2_async_register_subdev(sd);
}

int max_des_probe(struct i2c_client *client, struct max_des *des)
{
	struct max_des_priv *priv;
	int ret, i;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	priv->dev = &client->dev;
	priv->des = des;
	des->priv = priv;

	dev_info(priv->dev, "\n============================================\n");
	dev_info(priv->dev, "[DES-PROBE] 1. Allocating driver memory...\n");

	des->phys = devm_kcalloc(priv->dev, des->ops->num_phys, sizeof(*des->phys), GFP_KERNEL);
	des->pipes = devm_kcalloc(priv->dev, des->ops->num_pipes, sizeof(*des->pipes), GFP_KERNEL);
	des->links = devm_kcalloc(priv->dev, des->ops->num_links, sizeof(*des->links), GFP_KERNEL);
	priv->sources = devm_kcalloc(priv->dev, des->ops->num_links, sizeof(*priv->sources), GFP_KERNEL);

	if (!des->phys || !des->pipes || !des->links || !priv->sources)
		return -ENOMEM;

	for (i = 0; i < des->ops->num_phys; i++) {
		des->phys[i].index = i;
		des->phys[i].bus_type = V4L2_MBUS_CSI2_DPHY;
		des->phys[i].mipi.num_data_lanes = 2;
		des->phys[i].mipi.clock_lane = 0;
		des->phys[i].mipi.data_lanes[0] = 1;
		des->phys[i].mipi.data_lanes[1] = 2;
		des->phys[i].link_frequency = MAX_DES_LINK_FREQUENCY_DEFAULT;
	}

	for (i = 0; i < des->ops->num_pipes; i++)
		des->pipes[i].index = i;

	for (i = 0; i < des->ops->num_links; i++) {
		des->links[i].index = i;
		des->links[i].enabled = (i == TEST_LINK_ID);
		priv->sources[i].index = i;
	}

	dev_info(priv->dev,
		 "[DES-PROBE] 2. Invoking Hardware Init (GMSL Link Wakeup), TEST_LINK_ID=%u...\n",
		 TEST_LINK_ID);

	if (des->ops->init)
		des->ops->init(des);

	ret = max_des_i2c_mux_init(priv);
	if (ret)
		return ret;

	ret = max_des_v4l2_register(priv);

	dev_info(priv->dev, "[DES-PROBE] Probe complete! Status: %d\n", ret);
	dev_info(priv->dev, "============================================\n\n");
	return ret;
}
EXPORT_SYMBOL_NS_GPL(max_des_probe, "MAX_SERDES");

int max_des_remove(struct max_des *des)
{
	struct max_des_priv *priv = des->priv;
	dev_info(priv->dev, "[DES] Removing driver...\n");
	v4l2_async_unregister_subdev(&priv->sd);
	v4l2_subdev_cleanup(&priv->sd);
	v4l2_async_nf_unregister(&priv->nf);
	v4l2_async_nf_cleanup(&priv->nf);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	i2c_mux_del_adapters(priv->mux);
	return 0;
}
EXPORT_SYMBOL_NS_GPL(max_des_remove, "MAX_SERDES");

int max_des_phy_hw_data_lanes(struct max_des *des, struct max_des_phy *phy)
{
	return des->ops->phys_configs.configs[des->phys_config].lanes[phy->index];
}
EXPORT_SYMBOL_NS_GPL(max_des_phy_hw_data_lanes, "MAX_SERDES");

MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("I2C_ATR");
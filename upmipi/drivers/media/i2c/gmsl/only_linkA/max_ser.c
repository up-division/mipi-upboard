// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim GMSL2 Serializer Driver (Simplified Architecture)
 * V4L2 Framework & Software Logic with Detailed Logging
 */

#include <linux/delay.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

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

/* 擁抱極簡 Pad 架構：
 * Pad 0 = Sink (接 ISX031 Sensor)
 * Pad 1 = Source (接 MAX96724 Deserializer)
 */
#define SER_PAD_SINK   0
#define SER_PAD_SOURCE 1
#define SER_NUM_PADS   2

struct max_ser_priv {
	struct max_ser *ser;
	struct device *dev;
	struct i2c_client *client;
	struct i2c_mux_core *mux;

	struct media_pad *pads;
	struct max_serdes_source *sources;

	struct v4l2_subdev sd;
	struct v4l2_async_notifier nf;
	struct v4l2_ctrl_handler ctrl_handler;
};

static inline struct max_ser_priv *sd_to_priv(struct v4l2_subdev *sd)
{
	return container_of(sd, struct max_ser_priv, sd);
}

static inline struct max_ser_priv *nf_to_priv(struct v4l2_async_notifier *nf)
{
	return container_of(nf, struct max_ser_priv, nf);
}

static int max_ser_i2c_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
	return 0;
}

static int max_ser_i2c_mux_init(struct max_ser_priv *priv)
{
	u32 flags = I2C_MUX_LOCKED;
	int ret;

	dev_info(priv->dev, "[SER-PROBE] 3. Registering I2C MUX for Sensor...\n");

	priv->mux = i2c_mux_alloc(priv->client->adapter, priv->dev,
				  1, 0, flags, max_ser_i2c_mux_select, NULL);
	if (!priv->mux) return -ENOMEM;
	priv->mux->priv = priv;

	ret = i2c_mux_add_adapter(priv->mux, 0, 0);
	if (ret) {
		i2c_mux_del_adapters(priv->mux);
		return ret;
	}

	/* 自動掛載 ISX031 於 MUX 通道 */
	if (priv->mux->adapter[0]) {
		struct i2c_board_info sensor_info = { I2C_BOARD_INFO("isx031", 0x1A) };
		dev_info(priv->dev, "[SER-PROBE] 3a. Auto-probing ISX031 at 0x1A...\n");
		i2c_new_client_device(priv->mux->adapter[0], &sensor_info);
	}

	return 0;
}

static void max_ser_fill_default_fmt(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = 1920;
	fmt->height = 1536;
	fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

static int max_ser_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[1] = { 0 };
	struct v4l2_subdev_krouting routing = { .routes = routes, .num_routes = 1 };
	struct v4l2_mbus_framefmt *fmt;
	int i, ret;

	/* 內部路由: Sink(0) -> Source(1) */
	routes[0].sink_pad = SER_PAD_SINK;
	routes[0].sink_stream = 0;
	routes[0].source_pad = SER_PAD_SOURCE;
	routes[0].source_stream = 0;
	routes[0].flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;

	ret = v4l2_subdev_set_routing(sd, state, &routing);
	if (ret) return ret;

	for (i = 0; i < SER_NUM_PADS; i++) {
		fmt = v4l2_subdev_state_get_format(state, i, 0);
		if (fmt) max_ser_fill_default_fmt(fmt);
	}
	return 0;
}

static int max_ser_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *f = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (f && f->width && f->height && f->code) {
		format->format = *f;
		return 0;
	}
	max_ser_fill_default_fmt(&format->format);
	if (f) *f = format->format;
	return 0;
}

static int max_ser_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *f;
	if (!format->format.width || !format->format.height || !format->format.code)
		max_ser_fill_default_fmt(&format->format);
	f = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (f) *f = format->format;
	return 0;
}

static int max_ser_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct v4l2_mbus_frame_desc upstream_fd = { 0 };

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 0;

	if (pad != SER_PAD_SOURCE)
		return 0;

	if (!priv->sources || !priv->sources[0].sd)
		return 0;

	/* 向感測器詢問 Descriptor 並透傳給 DES */
	if (v4l2_subdev_call(priv->sources[0].sd, pad, get_frame_desc, priv->sources[0].pad, &upstream_fd) == 0) {
		*fd = upstream_fd;
	}
	return 0;
}

static int max_ser_set_routing(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			       enum v4l2_subdev_format_whence which,
			       struct v4l2_subdev_krouting *routing)
{
	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && sd_to_priv(sd)->ser->active)
		return -EBUSY;
	return v4l2_subdev_set_routing(sd, state, routing);
}

static int max_ser_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct max_ser_priv *priv = v4l2_get_subdevdata(sd);
	struct max_ser *ser = priv->ser;
	int ret = 0;

	dev_info(priv->dev, "[SER-STREAM] s_stream(%d) sequence start\n", enable);

	if (enable) {
		ser->active = true;

		/* 1. 硬體前置作業 (包含 MFP0 Reset 與 Datatype 設定) */
		if (ser->ops->pre_stream) {
			dev_info(priv->dev, "[SER-STREAM] Executing HW pre-stream prep...\n");
			ret = ser->ops->pre_stream(ser);
			if (ret) goto err;
		}

		/* 2. 啟動末端 Sensor (ISX031) */
		if (priv->sources && priv->sources[0].sd) {
			dev_info(priv->dev, "[SER-STREAM] Enabling Sensor via V4L2 API...\n");
			ret = v4l2_subdev_enable_streams(priv->sources[0].sd, priv->sources[0].pad, BIT_ULL(0));
			if (ret && ret != -EALREADY) goto err;
		}

		/* 3. 軟體踢發 (包含 MFP7 喚醒訊號與超詳細 Log 追蹤) */
		if (ser->ops->post_stream) {
			dev_info(priv->dev, "[SER-STREAM] Executing HW post-stream kick...\n");
			ret = ser->ops->post_stream(ser);
			if (ret) goto err;
		}

		dev_info(priv->dev, "[SER-STREAM] Sequence complete. Streaming!\n");
		return 0;
err:
		dev_err(priv->dev, "[SER-STREAM] Failed to start stream.\n");
		ser->active = false;
		return ret;
	}

	/* 關閉串流 */
	if (priv->sources && priv->sources[0].sd) {
		dev_info(priv->dev, "[SER-STREAM] Disabling Sensor...\n");
		v4l2_subdev_disable_streams(priv->sources[0].sd, priv->sources[0].pad, BIT_ULL(0));
	}
	ser->active = false;
	dev_info(priv->dev, "[SER-STREAM] Stream stopped.\n");
	return 0;
}

static int max_ser_enable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, u32 pad, u64 streams_mask)
{
	return max_ser_s_stream(sd, 1);
}

static int max_ser_disable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, u32 pad, u64 streams_mask)
{
	return max_ser_s_stream(sd, 0);
}

static int max_ser_log_status(struct v4l2_subdev *sd)
{
	struct max_ser_priv *priv = v4l2_get_subdevdata(sd);
	dev_info(priv->dev, "[SER] v4l2-ctl --log-status triggered.\n");
	if (priv->ser->ops->log_status)
		priv->ser->ops->log_status(priv->ser);
	return 0;
}

static const struct v4l2_subdev_video_ops max_ser_video_ops = { .s_stream = max_ser_s_stream };
static const struct v4l2_subdev_core_ops max_ser_core_ops = { .log_status = max_ser_log_status };
static const struct v4l2_subdev_pad_ops max_ser_pad_ops = {
	.enable_streams = max_ser_enable_streams,
	.disable_streams = max_ser_disable_streams,
	.set_routing = max_ser_set_routing,
	.get_frame_desc = max_ser_get_frame_desc,
	.get_fmt = max_ser_get_fmt,
	.set_fmt = max_ser_set_fmt,
};
static const struct v4l2_subdev_ops max_ser_subdev_ops = {
	.core = &max_ser_core_ops,
	.pad = &max_ser_pad_ops,
	.video = &max_ser_video_ops,
};
static const struct v4l2_subdev_internal_ops max_ser_internal_ops = {
	.init_state = &max_ser_init_state,
};
static const struct media_entity_operations max_ser_media_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.has_pad_interdep = v4l2_subdev_has_pad_interdep,
	.link_validate = v4l2_subdev_link_validate,
};

static int max_ser_notify_bound(struct v4l2_async_notifier *nf, struct v4l2_subdev *subdev,
				struct v4l2_async_connection *base_asc)
{
	struct max_ser_priv *priv = nf_to_priv(nf);
	struct max_serdes_asc *asc = asc_to_max(base_asc);
	struct max_serdes_source *source = asc->source;
	int src_pad = 0; /* Sensor 的 Output 通常是 Pad 0 */
	int ret;

	source->sd = subdev;
	source->pad = src_pad;

	dev_info(priv->dev, "[SER-V4L2] Bound remote sensor %s:%d -> local %s:SinkPad(%u)\n",
		 subdev->name, src_pad, priv->sd.name, SER_PAD_SINK);

	/* 【修正】捕捉 media_create_pad_link 與 v4l2_device_register_subdev_nodes 的回傳值 */
	ret = media_create_pad_link(&subdev->entity, src_pad, &priv->sd.entity, SER_PAD_SINK,
			      MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		dev_warn(priv->dev, "[SER-V4L2] Failed to create pad link: %d\n", ret);
	
	if (priv->sd.v4l2_dev) {
		ret = v4l2_device_register_subdev_nodes(priv->sd.v4l2_dev);
		if (ret)
			dev_warn(priv->dev, "[SER-V4L2] Failed to register subdev nodes: %d\n", ret);
	}

	return 0;
}

static void max_ser_notify_unbind(struct v4l2_async_notifier *nf, struct v4l2_subdev *subdev,
				  struct v4l2_async_connection *base_asc)
{
	asc_to_max(base_asc)->source->sd = NULL;
}

static const struct v4l2_async_notifier_operations max_ser_notify_ops = {
	.bound = max_ser_notify_bound,
	.unbind = max_ser_notify_unbind,
};

static int max_ser_v4l2_notifier_register(struct max_ser_priv *priv)
{
	struct max_serdes_source *source = &priv->sources[0];
	struct max_serdes_asc *asc;

	dev_info(priv->dev, "[SER-PROBE] 5. Registering Async Notifier for ISX031 Sensor...\n");

	v4l2_async_subdev_nf_init(&priv->nf, &priv->sd);
	source->index = 0;

	if (priv->mux && priv->mux->adapter[0]) {
		/* 監聽 0x1A 位址的裝置 (ISX031) */
		asc = v4l2_async_nf_add_i2c(&priv->nf, priv->mux->adapter[0]->nr, 0x1A, struct max_serdes_asc);
		if (!IS_ERR(asc)) asc->source = source;
	}

	priv->nf.ops = &max_ser_notify_ops;
	return v4l2_async_nf_register(&priv->nf);
}

static int max_ser_v4l2_register(struct max_ser_priv *priv)
{
	struct v4l2_subdev *sd = &priv->sd;
	int ret;

	dev_info(priv->dev, "[SER-PROBE] 4. Initializing V4L2 Subdev and Media Pads...\n");

	v4l2_i2c_subdev_init(sd, priv->client, &max_ser_subdev_ops);
	sd->internal_ops = &max_ser_internal_ops;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &max_ser_media_ops;
	
	/* 移除 FL_STREAMS 標籤，採用極簡路由 */
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	priv->pads = devm_kcalloc(priv->dev, SER_NUM_PADS, sizeof(*priv->pads), GFP_KERNEL);
	if (!priv->pads) return -ENOMEM;

	priv->pads[SER_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	priv->pads[SER_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	v4l2_set_subdevdata(sd, priv);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, 1);
	priv->sd.ctrl_handler = &priv->ctrl_handler;

	ret = media_entity_pads_init(&sd->entity, SER_NUM_PADS, priv->pads);
	if (ret) return ret;

	ret = max_ser_v4l2_notifier_register(priv);
	if (ret) return ret;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret) return ret;

	return v4l2_async_register_subdev(sd);
}

int max_ser_probe(struct i2c_client *client, struct max_ser *ser)
{
	struct max_ser_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) return -ENOMEM;

	priv->client = client;
	priv->dev = &client->dev;
	priv->ser = ser;
	ser->priv = priv;

	dev_info(priv->dev, "\n============================================\n");
	dev_info(priv->dev, "[SER-PROBE] 1. Allocating driver memory...\n");

	priv->sources = devm_kcalloc(priv->dev, 1, sizeof(*priv->sources), GFP_KERNEL);

	dev_info(priv->dev, "[SER-PROBE] 2. Invoking Hardware Init...\n");
	if (ser->ops->init) ser->ops->init(ser);
	
	max_ser_i2c_mux_init(priv);
	
	ret = max_ser_v4l2_register(priv);
	
	dev_info(priv->dev, "[SER-PROBE] Probe complete! Status: %d\n", ret);
	dev_info(priv->dev, "============================================\n\n");
	return ret;
}
EXPORT_SYMBOL_NS_GPL(max_ser_probe, "MAX_SERDES");

int max_ser_remove(struct max_ser *ser)
{
	struct max_ser_priv *priv = ser->priv;
	dev_info(priv->dev, "[SER] Removing driver...\n");
	v4l2_async_unregister_subdev(&priv->sd);
	v4l2_subdev_cleanup(&priv->sd);
	v4l2_async_nf_unregister(&priv->nf);
	v4l2_async_nf_cleanup(&priv->nf);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	i2c_mux_del_adapters(priv->mux);
	return 0;
}
EXPORT_SYMBOL_NS_GPL(max_ser_remove, "MAX_SERDES");

MODULE_LICENSE("GPL");
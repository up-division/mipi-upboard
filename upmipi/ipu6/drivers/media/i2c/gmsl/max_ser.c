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

#include "max_ser.h"
#include "max_serdes.h"

#ifndef MEDIA_PAD_FL_INTERNAL
#define MEDIA_PAD_FL_INTERNAL 0
#endif

#define MAX_SER_NUM_LINKS	1
#define MAX_SER_NUM_PHYS	1

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
	priv->mux = i2c_mux_alloc(priv->client->adapter, &priv->client->dev,
				  1, 0, I2C_MUX_LOCKED | I2C_MUX_GATE,
				  max_ser_i2c_mux_select, NULL);
	if (!priv->mux) return -ENOMEM;
	return i2c_mux_add_adapter(priv->mux, 0, 0);
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

static int max_ser_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	struct max_ser_priv *priv = v4l2_get_subdevdata(sd);
	struct max_ser *ser = priv->ser;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && ser->active)
		return -EBUSY;
	if (max_ser_pad_is_source(ser, format->pad))
		return v4l2_subdev_get_fmt(sd, state, format);
	if (max_ser_pad_is_tpg(ser, format->pad)) {
		ret = max_ser_set_tpg_fmt(sd, state, format);
		if (ret) return ret;
	}
	fmt = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (!fmt) return -EINVAL;
	*fmt = format->format;
	fmt = v4l2_subdev_state_get_opposite_stream_format(state, format->pad,
							   format->stream);
	if (!fmt) return -EINVAL;
	*fmt = format->format;
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
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct v4l2_subdev_state *state;
	int ret;
	state = v4l2_subdev_lock_and_get_active_state(&priv->sd);
	ret = max_ser_get_frame_desc_state(sd, state, fd, pad);
	v4l2_subdev_unlock_state(state);
	return ret;
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
	struct v4l2_subdev_route routes[MAX_SER_NUM_PHYS] = { 0 };
	struct v4l2_subdev_krouting routing = {
		.routes = routes,
	};
	struct max_ser_priv *priv = v4l2_get_subdevdata(sd);
	struct max_ser *ser = priv->ser;
	unsigned int stream = 0;
	unsigned int i;

	for (i = 0; i < ser->ops->num_phys; i++) {
		struct max_ser_phy *phy = &ser->phys[i];
		if (!phy->enabled) continue;
		routing.routes[routing.num_routes++] = (struct v4l2_subdev_route) {
			.sink_pad = max_ser_phy_to_pad(ser, phy),
			.sink_stream = 0,
			.source_pad = max_ser_source_pad(ser),
			.source_stream = stream,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		};
		stream++;
		break;
	}
	return v4l2_subdev_set_routing(sd, state, &routing);
}

static const struct v4l2_subdev_core_ops max_ser_core_ops = {
	.log_status = max_ser_log_status,
};

static const struct v4l2_ctrl_ops max_ser_ctrl_ops = {
	.s_ctrl = max_ser_s_ctrl,
};

static const struct v4l2_subdev_pad_ops max_ser_pad_ops = {
	.set_routing = max_ser_set_routing,
	.get_frame_desc = max_ser_get_frame_desc,
	.get_mbus_config = max_ser_get_mbus_config,
	.get_fmt = v4l2_subdev_get_fmt,
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
	struct max_serdes_asc *asc = asc_to_max(base_asc);
	struct max_serdes_source *source = asc->source;
	u32 pad = source->index;
	int ret;

    /* [Patch] Hardcode remote pad for manual link (Sensor usually output at pad 0) */
    source->pad = 0; 
	source->sd = subdev;

	dev_info(priv->dev, "Bound subdev %s to pad %u\n", subdev->name, pad);

	ret = media_create_pad_link(&source->sd->entity, source->pad,
				    &priv->sd.entity, pad,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(priv->dev, "Unable to link %s:%u -> %s:%u\n",
			source->sd->name, source->pad, priv->sd.name, pad);
		return ret;
	}
	return 0;
}

static void max_ser_notify_unbind(struct v4l2_async_notifier *nf,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_connection *base_asc)
{
	struct max_serdes_asc *asc = asc_to_max(base_asc);
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
		struct max_serdes_asc *asc;

		source = max_ser_get_phy_source(priv, phy);
		if (!source->ep_fwnode)
			continue;

		asc = v4l2_async_nf_add_fwnode(&priv->nf, source->ep_fwnode,
					       struct max_serdes_asc);
		if (IS_ERR(asc)) {
			v4l2_async_nf_cleanup(&priv->nf);
			return PTR_ERR(asc);
		}
		asc->source = source;
	}

    /* [Patch] Force Link for Manual Probe (Looking on Bus 1) */
    if (ser->ops->num_phys > 1) { // Deserializer Mode
        struct i2c_adapter *adapter;
        struct max_serdes_asc *asc;
        struct max_serdes_source *source;
        
        adapter = i2c_get_adapter(1); /* HARDCODED: Monitor Bus 1 */
        if (adapter) {
            dev_info(priv->dev, "HACK: Force waiting for I2C Client 0x21 on Bus %d\n", adapter->nr);
            asc = v4l2_async_nf_add_i2c(&priv->nf, adapter->nr, 0x21, struct max_serdes_asc);
            
            if (!IS_ERR(asc)) {
                source = &priv->sources[0]; 
                asc->source = source;
                dev_info(priv->dev, "HACK: Success! Added async waiter.\n");
            } else {
                dev_err(priv->dev, "HACK: Failed to add I2C waiter: %ld\n", PTR_ERR(asc));
            }
            i2c_put_adapter(adapter);
        }
    }

	priv->nf.ops = &max_ser_notify_ops;
	ret = v4l2_async_nf_register(&priv->nf);
	if (ret) {
		dev_err(priv->dev, "Failed to register subdev notifier");
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
    if (!fwnode) {
        dev_warn(priv->dev, "Manual probe: Skipping Notifier (Serializer).\n");
    } else {
        ret = max_ser_v4l2_notifier_register(priv);
        if (ret) goto err_media_entity_cleanup;
    }

	ret = v4l2_async_register_subdev(sd);
	if (ret) goto err_nf_cleanup;

	return 0;

err_nf_cleanup:
    if (fwnode) max_ser_v4l2_notifier_unregister(priv);
err_media_entity_cleanup:
	media_entity_cleanup(&sd->entity);
err_free_ctrl:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return ret;
}

static void max_ser_v4l2_unregister(struct max_ser_priv *priv)
{
	struct v4l2_subdev *sd = &priv->sd;
	max_ser_v4l2_notifier_unregister(priv);
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

    /* Skip init if manual probe to preserve tunnel mode */
    if (dev_fwnode(dev)) {
        ret = max_ser_init(priv);
        if (ret) return ret;
    } else {
        dev_warn(dev, "Manual Probe: Skipped max_ser_init to preserve Tunnel Mode.\n");
    }

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
unsigned int max_ser_get_supported_modes(struct v4l2_subdev *sd) { return 0; }
bool max_ser_supports_vc_remap(struct v4l2_subdev *sd) { return false; }
int max_ser_set_mode(struct v4l2_subdev *sd, enum max_serdes_gmsl_mode mode) { return 0; }
int max_ser_set_vc_remaps(struct v4l2_subdev *sd, struct max_serdes_vc_remap *vc_remaps, int num_vc_remaps) { return 0; }

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
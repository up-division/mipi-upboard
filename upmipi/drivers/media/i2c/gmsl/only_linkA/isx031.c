// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022-2025 Intel Corporation.
/*
 * ISX031 Sensor Driver (Refactored for GMSL2 Module Architecture)
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/property.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
#include <asm/unaligned.h>
#else
#include <linux/unaligned.h>
#endif
#include <media/v4l2-device.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
#include <media/mipi-csi2.h>
#endif

#include <linux/types.h>

#define ISX031_NAME "isx031_module"

static const struct software_node isx031_swnode = {
	.name = "isx031_manual",
};

#define to_isx031(_sd)			container_of(_sd, struct isx031, sd)

#define ISX031_OTP_TYPE_NAME_L		0x7E8A
#define ISX031_OTP_TYPE_NAME_H		0x7E8B
#define ISX031_OTP_TYPE_NAME_H_FIELD	0x0F
#define ISX031_OTP_MODULE_ID_L		0x031

#define ISX031_REG_MODE_SET_F		0x8A01
#define ISX031_MODE_STANDBY		0x00
#define ISX031_MODE_STREAMING		0x80

#define ISX031_REG_SENSOR_STATE		0x6005
#define ISX031_STATE_STREAMING		0x05
#define ISX031_STATE_STARTUP		0x02

#define ISX031_REG_MODE_SET_F_LOCK	0xBEF0
#define ISX031_MODE_UNLOCK		0x53

#define ISX031_REG_MODE_SELECT		0x8A00
#define ISX031_MODE_4LANES_30FPS	0x17

/* To serialize asynchronous callbacks */
static DEFINE_MUTEX(isx031_mutex);

struct isx031 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct i2c_client *client;
	bool streaming;
	bool hw_initialized;
};

#define ISX031_REG_LEN_08BIT 1

static int isx031_read_reg(struct i2c_client *client, u16 reg, u16 len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[4] = {0};
	int ret;

	if (len > 4) return -EINVAL;

	put_unaligned_be16(reg, addr_buf);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(addr_buf);
	msgs[0].buf = addr_buf;
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);
	return 0;
}

static int isx031_read_reg_state(struct i2c_client *client, u32 *val)
{
	int ret, i;
	for (i = 0; i < 50; i++) {
		ret = isx031_read_reg(client, ISX031_REG_SENSOR_STATE, ISX031_REG_LEN_08BIT, val);
		if (ret == 0) break;
		usleep_range(10000, 10500);
	}
	return ret;
}

static int isx031_write_reg(struct isx031 *isx031, u16 reg, u16 len, u32 val)
{
	struct i2c_client *client = isx031->client;
	u8 buf[6];

	if (len > 4) return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << 8 * (4 - len), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2) {
		dev_err(&client->dev, "%s:failed: reg=%2x\n", __func__, reg);
		return -EIO;
	}
	return 0;
}

static int isx031_mode_transit(struct isx031 *isx031, int state)
{
	struct i2c_client *client = isx031->client;
	int ret, cur_mode, mode = ISX031_MODE_STANDBY;
	u32 val = 0;

	if (state == ISX031_STATE_STREAMING)
		mode = ISX031_MODE_STREAMING;

	ret = isx031_read_reg_state(client, &val);
	if (ret) return ret;
	cur_mode = val;

	/* GMSL Module Fixed Drive Mode: 4 Lanes, 30 FPS */
	ret = isx031_write_reg(isx031, ISX031_REG_MODE_SELECT, 1, ISX031_MODE_4LANES_30FPS);
	if (ret) return ret;

	ret = isx031_write_reg(isx031, ISX031_REG_MODE_SET_F_LOCK, 1, ISX031_MODE_UNLOCK);
	if (ret) return ret;

	ret = isx031_write_reg(isx031, ISX031_REG_MODE_SET_F, 1, mode);
	if (ret) {
		dev_err(&client->dev, "failed to transit mode from 0x%x to 0x%x", cur_mode, mode);
		return ret;
	}

	ret = isx031_read_reg_state(client, &val);
	return ret;
}

static int isx031_initialize_module(struct isx031 *isx031)
{
	struct i2c_client *client = isx031->client;
	int ret;
	u32 val = 0;

	ret = isx031_read_reg_state(client, &val);
	if (ret) return ret;

	dev_dbg(&client->dev, "sensor in mode 0x%x", val);

	if (val == ISX031_STATE_STREAMING) {
		if (isx031_mode_transit(isx031, ISX031_STATE_STARTUP))
			return ret;
	}

	dev_info(&client->dev, "[Sensor] Camera pre-configured by internal MCU.\n");
	return 0;
}

static int isx031_identify_module(struct i2c_client *client)
{
	u32 NAME_L = 0, NAME_H = 0;
	int ret = 0, i = 0;

	for (i = 0; i < 50; i++) {
		ret = isx031_read_reg(client, ISX031_OTP_TYPE_NAME_L, ISX031_REG_LEN_08BIT, &NAME_L);
		if (!ret) break;
	}
	if (i == 50) return ret;

	for (i = 0; i < 50; i++) {
		ret = isx031_read_reg(client, ISX031_OTP_TYPE_NAME_H, ISX031_REG_LEN_08BIT, &NAME_H);
		if (!ret) break;
	}
	if (i == 50) return ret;

	if (((NAME_H & ISX031_OTP_TYPE_NAME_H_FIELD) << 8 | NAME_L) != ISX031_OTP_MODULE_ID_L) {
		dev_err(&client->dev, "isx031 module id mismatch\n");
		return -ENODEV;
	}

	return 0;
}

static int isx031_start_streaming(struct isx031 *isx031)
{
	struct i2c_client *client = isx031->client;
	u32 state = 0;
	int ret;

	ret = isx031_mode_transit(isx031, ISX031_STATE_STREAMING);
	if (ret) return ret;

	ret = isx031_read_reg_state(client, &state);
	if (ret) return ret;

	dev_info(&client->dev, "streaming state readback: 0x%02x\n", state);

	if (state != ISX031_STATE_STREAMING)
		return -EIO;

	return 0;
}

static void isx031_stop_streaming(struct isx031 *isx031)
{
	isx031_mode_transit(isx031, ISX031_STATE_STARTUP);
}

static int isx031_lazy_hw_init(struct isx031 *isx031)
{
	int ret;

	if (isx031->hw_initialized) return 0;

	dev_info(&isx031->client->dev, "lazy init: identify/init module\n");

	ret = isx031_identify_module(isx031->client);
	if (ret) return ret;

	ret = isx031_initialize_module(isx031);
	if (ret) return ret;

	isx031->hw_initialized = true;
	dev_info(&isx031->client->dev, "lazy init done\n");
	return 0;
}

static int isx031_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct isx031 *isx031 = to_isx031(sd);
	struct i2c_client *client = isx031->client;
	int ret = 0;

	enable = !!enable;
	if (isx031->streaming == enable) return 0;

	mutex_lock(&isx031_mutex);

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0) goto err_unlock;

		ret = isx031_lazy_hw_init(isx031);
		if (ret) {
			pm_runtime_put(&client->dev);
			goto err_unlock;
		}

		ret = isx031_start_streaming(isx031);
		if (ret) {
			isx031_stop_streaming(isx031);
			pm_runtime_put(&client->dev);
			goto err_unlock;
		}
	} else {
		isx031_stop_streaming(isx031);
		pm_runtime_put(&client->dev);
	}

	isx031->streaming = enable;

err_unlock:
	mutex_unlock(&isx031_mutex);
	return ret;
}

static int isx031_enable_streams(struct v4l2_subdev *subdev, struct v4l2_subdev_state *state, u32 pad, u64 streams_mask)
{
	return isx031_set_stream(subdev, 1);
}

static int isx031_disable_streams(struct v4l2_subdev *subdev, struct v4l2_subdev_state *state, u32 pad, u64 streams_mask)
{
	return isx031_set_stream(subdev, 0);
}

static int __maybe_unused isx031_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isx031 *isx031 = to_isx031(sd);

	mutex_lock(&isx031_mutex);
	if (isx031->streaming)
		isx031_stop_streaming(isx031);
	mutex_unlock(&isx031_mutex);

	/* 由於沒有硬體 Reset 腳，我們單純依賴 GMSL 斷電 */
	isx031->hw_initialized = false;
	return 0;
}

static int __maybe_unused isx031_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isx031 *isx031 = to_isx031(sd);
	int ret = 0;

	mutex_lock(&isx031_mutex);
	if (isx031->streaming) {
		ret = isx031_lazy_hw_init(isx031);
		if (!ret) ret = isx031_start_streaming(isx031);
		
		if (ret) {
			isx031->streaming = false;
			isx031_stop_streaming(isx031);
		}
	}
	mutex_unlock(&isx031_mutex);
	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
static int isx031_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad, struct v4l2_mbus_frame_desc *desc)
{
	desc->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	desc->num_entries = 1;
	desc->entry[0].flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
	desc->entry[0].stream = 0;
	desc->entry[0].pixelcode = MEDIA_BUS_FMT_UYVY8_1X16;
	desc->entry[0].length = 0;
	desc->entry[0].bus.csi2.vc = 0;
	desc->entry[0].bus.csi2.dt = MIPI_CSI2_DT_YUV422_8B;
	return 0;
}
#else
static int isx031_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad, struct v4l2_mbus_frame_desc *desc)
{
	desc->num_entries = 1;
	desc->entry[0].flags = 0;
	desc->entry[0].pixelcode = MEDIA_BUS_FMT_UYVY8_1X16;
	desc->entry[0].length = 0;
	return 0;
}
#endif

/* GMSL Module Fixed Format: 1920x1536 UYVY */
static void isx031_fill_fixed_format(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = 1920;
	fmt->height = 1536;
	fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int isx031_set_format(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
			     struct v4l2_subdev_pad_config *cfg,
#else
			     struct v4l2_subdev_state *sd_state,
#endif
			     struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;

	isx031_fill_fixed_format(&fmt->format);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
	framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
	else
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
#else
	framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
#endif

	if (framefmt)
		*framefmt = fmt->format;

	return 0;
}

static int isx031_get_format(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
			     struct v4l2_subdev_pad_config *cfg,
#else
			     struct v4l2_subdev_state *sd_state,
#endif
			     struct v4l2_subdev_format *fmt)
{
	isx031_fill_fixed_format(&fmt->format);
	return 0;
}

static int isx031_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt = v4l2_subdev_state_get_format(state, 0);
	if (fmt) isx031_fill_fixed_format(fmt);
	return 0;
}

static const struct v4l2_subdev_video_ops isx031_video_ops = {
	.s_stream = isx031_set_stream,
};

static const struct v4l2_subdev_pad_ops isx031_pad_ops = {
	.set_fmt = isx031_set_format,
	.get_fmt = isx031_get_format,
	.get_frame_desc = isx031_get_frame_desc,
	.enable_streams = isx031_enable_streams,
	.disable_streams = isx031_disable_streams,
};

static const struct v4l2_subdev_ops isx031_subdev_ops = {
	.video = &isx031_video_ops,
	.pad = &isx031_pad_ops,
};

static const struct media_entity_operations isx031_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops isx031_internal_ops = {
	.init_state = isx031_init_state,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int isx031_remove(struct i2c_client *client)
#else
static void isx031_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	pm_runtime_disable(&client->dev);
	device_remove_software_node(&client->dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static int isx031_probe(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct isx031 *isx031;
	int ret;

	isx031 = devm_kzalloc(&client->dev, sizeof(*isx031), GFP_KERNEL);
	if (!isx031) return -ENOMEM;

	isx031->client = client;

	/* 手動載入時，掛上軟體節點，防止 V4L2 核心崩潰 */
	ret = device_add_software_node(&client->dev, &isx031_swnode);
	if (ret && ret != -EEXIST)
		dev_warn(&client->dev, "Failed to add software node: %d\n", ret);

	sd = &isx031->sd;
	v4l2_i2c_subdev_init(sd, client, &isx031_subdev_ops);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
#else
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
	sd->internal_ops = &isx031_internal_ops;
	sd->entity.ops = &isx031_subdev_entity_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	isx031->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &isx031->pad);
	if (ret < 0) return ret;

	v4l2_subdev_init_finalize(sd);

	snprintf(sd->name, sizeof(sd->name), ISX031_NAME);

	dev_info(&client->dev, "Module probe: skip identify/init/preset in probe\n");

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0)
	ret = v4l2_async_register_subdev_sensor_common(sd);
#else
	ret = v4l2_async_register_subdev_sensor(sd);
#endif
	if (ret < 0) {
		dev_err(&client->dev, "failed to register V4L2 subdev: %d\n", ret);
		media_entity_cleanup(&sd->entity);
		return ret;
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	dev_info(&client->dev, "isx031 module probe OK\n");
	return 0;
}

static const struct dev_pm_ops isx031_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(isx031_suspend, isx031_resume)
};

static const struct i2c_device_id isx031_id_table[] = {
	{ "isx031", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, isx031_id_table);

static const struct acpi_device_id isx031_acpi_ids[] = {
	{ "INTC113C" },
	{}
};
MODULE_DEVICE_TABLE(acpi, isx031_acpi_ids);

static struct i2c_driver isx031_i2c_driver = {
	.driver = {
		.name = "isx031",
		.acpi_match_table = ACPI_PTR(isx031_acpi_ids),
		.pm = &isx031_pm_ops,
	},
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
	.probe_new = isx031_probe,
#else
	.probe = isx031_probe,
#endif
	.remove = isx031_remove,
	.id_table = isx031_id_table,
};

module_i2c_driver(isx031_i2c_driver);

MODULE_DESCRIPTION("isx031 GMSL module driver");
MODULE_LICENSE("GPL v2");
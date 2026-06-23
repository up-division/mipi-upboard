// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022-2025 Intel Corporation.

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
#include <asm/unaligned.h>
#else
#include <linux/unaligned.h>
#endif
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
#include <media/mipi-csi2.h>
#endif
#define to_isx031f(_sd)			container_of(_sd, struct isx031f, sd)

struct isx031f_reg {
	enum {
		ISX031F_REG_LEN_DELAY = 0,
		ISX031F_REG_LEN_08BIT = 1,
		ISX031F_REG_LEN_16BIT = 2,
	} mode;
	u16 address;
	u16 val;
};

struct isx031f_reg_list {
	u32 num_of_regs;
	const struct isx031f_reg *regs;
};

struct isx031f_link_freq_config {
	const struct isx031f_reg_list reg_list;
};

struct isx031f_mode {
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

	/* MEDIA_BUS_FMT */
	u32 code;

	/* MODE_FPS*/
	u32 fps;

	/* Sensor register settings for this resolution */
	const struct isx031f_reg_list reg_list;
};

struct isx031f_platform_data {
	unsigned int port;
	unsigned int lanes;
	uint32_t i2c_slave_address;
	int irq_pin;
	unsigned int irq_pin_flags;
	char irq_pin_name[16];
	char suffix;
	int gpios[4];
};

struct isx031f {
	struct v4l2_subdev sd;
	struct media_pad pad;

	/* Current mode */
	const struct isx031f_mode *cur_mode;
	/* Previous mode */
	const struct isx031f_mode *pre_mode;

	/* To serialize asynchronus callbacks */
	struct mutex mutex;

	/* i2c client */
	struct i2c_client *client;

	struct isx031f_platform_data *platform_data;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *fsin_gpio;

	/* Streaming on/off */
	bool streaming;
};

static const struct isx031f_reg isx031f_1920_1536_30fps_reg[] = {

};

static const struct isx031f_reg_list isx031f_1920_1536_30fps_reg_list = {
	.num_of_regs = ARRAY_SIZE(isx031f_1920_1536_30fps_reg),
	.regs = isx031f_1920_1536_30fps_reg,
};

static const struct isx031f_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1536,
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.fps = 30,
		.reg_list = isx031f_1920_1536_30fps_reg_list,
	},
};

static int isx031f_reset(struct gpio_desc *reset_gpio)
{
	if (!IS_ERR_OR_NULL(reset_gpio)) {
		gpiod_set_value_cansleep(reset_gpio, 0);
		usleep_range(500, 1000);
		gpiod_set_value_cansleep(reset_gpio, 1);
		/*Needs to sleep for quite a while before register writes*/
		usleep_range(200 * 1000, 200 * 1000 + 500);

		return 0;
	}

	return -EINVAL;
}

static int isx031f_read_reg(struct isx031f *isx031f, u16 reg, u16 len, u32 *val)
{
	struct i2c_client *client = isx031f->client;
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[4] = {0};
	int ret;

	if (len > 4)
		return -EINVAL;

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

static int isx031f_write_reg(struct isx031f *isx031f, u16 reg, u16 len, u32 val)
{
	struct i2c_client *client = isx031f->client;
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	dev_dbg(&client->dev, "%s, reg %x len %x, val %x\n", __func__, reg, len, val);
	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << 8 * (4 - len), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2) {
		dev_err(&client->dev, "%s:failed: reg=%2x\n", __func__, reg);
		return -EIO;
	}

	return 0;
}

static int isx031f_write_reg_list(struct isx031f *isx031f,
				 const struct isx031f_reg_list *r_list)
{
	struct i2c_client *client = v4l2_get_subdevdata(&isx031f->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < r_list->num_of_regs; i++) {
		if (r_list->regs[i].mode == ISX031F_REG_LEN_DELAY) {
			msleep(r_list->regs[i].val);
			continue;
		}
		ret = isx031f_write_reg(isx031f, r_list->regs[i].address,
				       ISX031F_REG_LEN_08BIT,
				       r_list->regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
				"failed to write reg 0x%4.4x. error = %d",
				r_list->regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static void isx031f_update_pad_format(const struct isx031f_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = mode->code;
	fmt->field = V4L2_FIELD_NONE;
}

static int isx031f_start_streaming(struct isx031f *isx031f)
{
	int ret;
	struct i2c_client *client = isx031f->client;
	const struct isx031f_reg_list *reg_list;
	
	if (isx031f->cur_mode != isx031f->pre_mode) {
		reg_list = &isx031f->cur_mode->reg_list;
		ret = isx031f_write_reg_list(isx031f, reg_list);
		if (ret) {
			dev_err(&client->dev, "failed to set stream mode");
			return ret;
		}
		isx031f->pre_mode = isx031f->cur_mode;
	} else {
		dev_dbg(&client->dev, "same mode, skip write reg list");
	}

	/* Drive FSIN GPIO high to enable frame sync */
	if (isx031f->fsin_gpio){
		dev_warn(&client->dev, "Set FSIN GPIO high to enable frame sync\n");
		gpiod_set_value_cansleep(isx031f->fsin_gpio, 0);
		usleep_range(300000, 500000);
		gpiod_set_value_cansleep(isx031f->fsin_gpio, 1);
	} else {
		dev_warn(&client->dev, "FSIN GPIO not available during streaming start\n");
	}

	return 0;
}

static void isx031f_stop_streaming(struct isx031f *isx031f)
{
	struct i2c_client *client = isx031f->client;

	/* Drive FSIN GPIO low to disable frame sync */
	if (isx031f->fsin_gpio){
		dev_warn(&client->dev, "Set FSIN GPIO low to disable frame sync\n");
		gpiod_set_value_cansleep(isx031f->fsin_gpio, 1);
		usleep_range(300000, 500000);
		gpiod_set_value_cansleep(isx031f->fsin_gpio, 0);
	} else {
		dev_warn(&client->dev, "FSIN GPIO not available during streaming stop\n");
	}
}

static int isx031f_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct isx031f *isx031f = to_isx031f(sd);
	struct i2c_client *client = isx031f->client;
	int ret = 0;

	if (isx031f->streaming == enable)
		return 0;

	mutex_lock(&isx031f->mutex);
	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			mutex_unlock(&isx031f->mutex);
			return ret;
		}

		ret = isx031f_start_streaming(isx031f);
		if (ret) {
			enable = 0;
			isx031f_stop_streaming(isx031f);
			pm_runtime_put(&client->dev);
		}
	} else {
		isx031f_stop_streaming(isx031f);
		pm_runtime_put(&client->dev);
	}

	isx031f->streaming = enable;

	mutex_unlock(&isx031f->mutex);

	return ret;
}

static int isx031f_enable_streams(struct v4l2_subdev *subdev,
	struct v4l2_subdev_state *state,
	u32 pad, u64 streams_mask)
{
	return isx031f_set_stream(subdev, true);
}

static int isx031f_disable_streams(struct v4l2_subdev *subdev,
	 struct v4l2_subdev_state *state,
	 u32 pad, u64 streams_mask)
{
	return isx031f_set_stream(subdev, false);
}

static int __maybe_unused isx031f_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isx031f *isx031f = to_isx031f(sd);

	mutex_lock(&isx031f->mutex);
	if (isx031f->streaming)
		isx031f_stop_streaming(isx031f);

	mutex_unlock(&isx031f->mutex);

	return 0;
}

static int __maybe_unused isx031f_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isx031f *isx031f = to_isx031f(sd);
	const struct isx031f_reg_list *reg_list;
	int ret;

	if (isx031f->reset_gpio != NULL)
		isx031f_reset(isx031f->reset_gpio);
	
	mutex_lock(&isx031f->mutex);
	if (isx031f->streaming) {
		ret = isx031f_start_streaming(isx031f);
		if (ret) {
			isx031f->streaming = false;
			isx031f_stop_streaming(isx031f);
			mutex_unlock(&isx031f->mutex);
			return ret;
		}
	}

	mutex_unlock(&isx031f->mutex);

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
static unsigned int isx031f_mbus_code_to_mipi(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		return MIPI_CSI2_DT_RGB565;
	case MEDIA_BUS_FMT_RGB888_1X24:
		return MIPI_CSI2_DT_RGB888;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
		return MIPI_CSI2_DT_YUV422_8B;
	case MEDIA_BUS_FMT_SBGGR16_1X16:
	case MEDIA_BUS_FMT_SGBRG16_1X16:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
	case MEDIA_BUS_FMT_SRGGB16_1X16:
		return MIPI_CSI2_DT_RAW16;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return MIPI_CSI2_DT_RAW12;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return MIPI_CSI2_DT_RAW10;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return MIPI_CSI2_DT_RAW8;
	default:
		/* return unavailable MIPI data type - 0x3f */
		WARN_ON(1);
		return 0x3f;
	}
}
#endif

static int isx031f_get_frame_desc(struct v4l2_subdev *sd,
	unsigned int pad, struct v4l2_mbus_frame_desc *desc)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
	struct isx031f *isx031f = to_isx031f(sd);
#endif
	unsigned int i;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
	desc->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
#endif
	desc->num_entries = V4L2_FRAME_DESC_ENTRY_MAX;

	for (i = 0; i < desc->num_entries; i++) {
		desc->entry[i].flags = 0;
		desc->entry[i].pixelcode = MEDIA_BUS_FMT_FIXED;
		desc->entry[i].length = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
		desc->entry[i].stream = i;
		desc->entry[i].bus.csi2.vc = i;
		desc->entry[i].bus.csi2.dt = isx031f_mbus_code_to_mipi(isx031f->cur_mode->code);
#endif
	}

	return 0;
}

static int isx031f_set_format(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
			     struct v4l2_subdev_pad_config *cfg,
#else
			     struct v4l2_subdev_state *sd_state,
#endif
			     struct v4l2_subdev_format *fmt)
{
	struct isx031f *isx031f = to_isx031f(sd);
	const struct isx031f_mode *mode;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++)
		if (supported_modes[i].code == fmt->format.code &&
		    supported_modes[i].width == fmt->format.width &&
		    supported_modes[i].height == fmt->format.height) {
			mode = &supported_modes[i];
			break;
		}

	if (i >= ARRAY_SIZE(supported_modes))
		mode = &supported_modes[0];

	mutex_lock(&isx031f->mutex);

	isx031f_update_pad_format(mode, &fmt->format);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
#else
		*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;
#endif
	} else {
		isx031f->cur_mode = mode;
	}

	mutex_unlock(&isx031f->mutex);

	return 0;
}

static int isx031f_get_format(struct v4l2_subdev *sd,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
			     struct v4l2_subdev_pad_config *cfg,
#else
			     struct v4l2_subdev_state *sd_state,
#endif
			     struct v4l2_subdev_format *fmt)
{
	struct isx031f *isx031f = to_isx031f(sd);

	mutex_lock(&isx031f->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
		fmt->format = *v4l2_subdev_get_try_format(&isx031f->sd, cfg,
							  fmt->pad);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
		fmt->format = *v4l2_subdev_get_try_format(&isx031f->sd, sd_state,
							  fmt->pad);
#else
		fmt->format = *v4l2_subdev_state_get_format(sd_state,
							  fmt->pad);
#endif
	else
		isx031f_update_pad_format(isx031f->cur_mode, &fmt->format);

	mutex_unlock(&isx031f->mutex);

	return 0;
}

static int isx031f_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct isx031f *isx031f = to_isx031f(sd);

	mutex_lock(&isx031f->mutex);
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
	isx031f_update_pad_format(&supported_modes[0],
				 v4l2_subdev_get_try_format(sd, fh->pad, 0));
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
	isx031f_update_pad_format(&supported_modes[0],
				 v4l2_subdev_get_try_format(sd, fh->state, 0));
#else
	isx031f_update_pad_format(&supported_modes[0],
				 v4l2_subdev_state_get_format(fh->state, 0));
#endif
	mutex_unlock(&isx031f->mutex);

	return 0;
}

static const struct v4l2_subdev_video_ops isx031f_video_ops = {
	.s_stream = isx031f_set_stream,
};

static const struct v4l2_subdev_pad_ops isx031f_pad_ops = {
	.set_fmt = isx031f_set_format,
	.get_fmt = isx031f_get_format,
	.get_frame_desc = isx031f_get_frame_desc,
	.enable_streams = isx031f_enable_streams,
	.disable_streams = isx031f_disable_streams,
};

static const struct v4l2_subdev_ops isx031f_subdev_ops = {
	.video = &isx031f_video_ops,
	.pad = &isx031f_pad_ops,
};

static const struct media_entity_operations isx031f_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops isx031f_internal_ops = {
	.open = isx031f_open,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int isx031f_remove(struct i2c_client *client)
#else
static void isx031f_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isx031f *isx031f = to_isx031f(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	pm_runtime_disable(&client->dev);
	mutex_destroy(&isx031f->mutex);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static int isx031f_probe(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct isx031f *isx031f;
	const struct isx031f_reg_list *reg_list;
	int ret;

	isx031f = devm_kzalloc(&client->dev, sizeof(*isx031f), GFP_KERNEL);
	if (!isx031f)
		return -ENOMEM;

	isx031f->client = client;
	isx031f->platform_data = client->dev.platform_data;
	if (isx031f->platform_data == NULL)
		dev_warn(&client->dev, "no platform data provided\n");

	isx031f->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
						     GPIOD_OUT_HIGH);
	isx031f->fsin_gpio = devm_gpiod_get_optional(&client->dev, "fsin",
						     GPIOD_OUT_LOW);
	if (IS_ERR(isx031f->reset_gpio))
		return -EPROBE_DEFER;
	else if (isx031f->reset_gpio == NULL)
		dev_warn(&client->dev, "Reset GPIO not found");
	else {
		dev_dbg(&client->dev, "Found reset GPIO");
		isx031f_reset(isx031f->reset_gpio);
	}

	/* initialize subdevice */
	sd = &isx031f->sd;
	v4l2_i2c_subdev_init(sd, client, &isx031f_subdev_ops);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
#else
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
	sd->internal_ops = &isx031f_internal_ops;
	sd->entity.ops = &isx031f_subdev_entity_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* initialize subdev media pad */
	isx031f->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &isx031f->pad);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s : media entity init Failed %d\n", __func__, ret);
		return ret;
	}

	if (isx031f->platform_data && isx031f->platform_data->suffix)
		snprintf(isx031f->sd.name, sizeof(isx031f->sd.name), "isx031f %c",
			 isx031f->platform_data->suffix);

	mutex_init(&isx031f->mutex);

	/* 1920x1536 default */
	isx031f->cur_mode = NULL;
	isx031f->pre_mode = &supported_modes[0];
	reg_list = &isx031f->pre_mode->reg_list;
	ret = isx031f_write_reg_list(isx031f, reg_list);
	if (ret) {
		dev_err(&client->dev, "failed to apply preset mode");
		goto probe_error_media_entity_cleanup;
	}
	isx031f->cur_mode = isx031f->pre_mode;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 13, 0)
	ret = v4l2_async_register_subdev_sensor_common(&isx031f->sd);
#else
	ret = v4l2_async_register_subdev_sensor(&isx031f->sd);
#endif
	if (ret < 0) {
		dev_err(&client->dev, "failed to register V4L2 subdev: %d",
			ret);
		goto probe_error_media_entity_cleanup;
	}

	/*
	 * Device is already turned on by i2c-core with ACPI domain PM.
	 * Enable runtime PM and turn off the device.
	 */
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

probe_error_media_entity_cleanup:
	media_entity_cleanup(&isx031f->sd.entity);
	pm_runtime_disable(&client->dev);
	mutex_destroy(&isx031f->mutex);

	return ret;
}

static const struct dev_pm_ops isx031f_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(isx031f_suspend, isx031f_resume)
};

static const struct i2c_device_id isx031f_id_table[] = {
	{ "isx031f", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, isx031f_id_table);

static struct i2c_driver isx031f_i2c_driver = {
	.driver = {
		.name = "isx031f",
		.pm = &isx031f_pm_ops,
	},
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
	.probe_new = isx031f_probe,
#else
	.probe = isx031f_probe,
#endif
	.remove = isx031f_remove,
	.id_table = isx031f_id_table,
};

module_i2c_driver(isx031f_i2c_driver);

MODULE_AUTHOR("Wang Yong <wangyong@sensing-world.com>");
MODULE_DESCRIPTION("isx031f sensor driver");
MODULE_LICENSE("GPL v2");

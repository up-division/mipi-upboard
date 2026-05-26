// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013--2024 Intel Corporation
 */

#include <linux/align.h>
#include <linux/bits.h>
#include <linux/bug.h>
#include <linux/completion.h>
#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#include "ipu6.h"
#include "ipu6-bus.h"
#include "ipu6-cpd.h"
#include "ipu6-fw-isys.h"
#include "ipu6-isys.h"
#include "ipu6-isys-csi2.h"
#include "ipu6-isys-queue.h"
#include "ipu6-isys-video.h"
#include "ipu6-platform-regs.h"

const struct ipu6_isys_pixelformat ipu6_isys_pfmts[] = {
	{ V4L2_PIX_FMT_SBGGR12, 16, 12, MEDIA_BUS_FMT_SBGGR12_1X12,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW16 },
	{ V4L2_PIX_FMT_SGBRG12, 16, 12, MEDIA_BUS_FMT_SGBRG12_1X12,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW16 },
	{ V4L2_PIX_FMT_SGRBG12, 16, 12, MEDIA_BUS_FMT_SGRBG12_1X12,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW16 },
	{ V4L2_PIX_FMT_SRGGB12, 16, 12, MEDIA_BUS_FMT_SRGGB12_1X12,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW16 },
	{ V4L2_PIX_FMT_SBGGR10, 16, 10, MEDIA_BUS_FMT_SBGGR10_1X10,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW16 },
	{ V4L2_PIX_FMT_SGBRG10, 16, 10, MEDIA_BUS_FMT_SGBRG10_1X10,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW16 },
	{ V4L2_PIX_FMT_SGRBG10, 16, 10, MEDIA_BUS_FMT_SGRBG10_1X10,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW16 },
	{ V4L2_PIX_FMT_SRGGB10, 16, 10, MEDIA_BUS_FMT_SRGGB10_1X10,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW16 },
	{ V4L2_PIX_FMT_SBGGR8, 8, 8, MEDIA_BUS_FMT_SBGGR8_1X8,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW8 },
	{ V4L2_PIX_FMT_SGBRG8, 8, 8, MEDIA_BUS_FMT_SGBRG8_1X8,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW8 },
	{ V4L2_PIX_FMT_SGRBG8, 8, 8, MEDIA_BUS_FMT_SGRBG8_1X8,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW8 },
	{ V4L2_PIX_FMT_SRGGB8, 8, 8, MEDIA_BUS_FMT_SRGGB8_1X8,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW8 },
	{ V4L2_PIX_FMT_SBGGR12P, 12, 12, MEDIA_BUS_FMT_SBGGR12_1X12,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW12 },
	{ V4L2_PIX_FMT_SGBRG12P, 12, 12, MEDIA_BUS_FMT_SGBRG12_1X12,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW12 },
	{ V4L2_PIX_FMT_SGRBG12P, 12, 12, MEDIA_BUS_FMT_SGRBG12_1X12,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW12 },
	{ V4L2_PIX_FMT_SRGGB12P, 12, 12, MEDIA_BUS_FMT_SRGGB12_1X12,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW12 },
	{ V4L2_PIX_FMT_SBGGR10P, 10, 10, MEDIA_BUS_FMT_SBGGR10_1X10,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW10 },
	{ V4L2_PIX_FMT_SGBRG10P, 10, 10, MEDIA_BUS_FMT_SGBRG10_1X10,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW10 },
	{ V4L2_PIX_FMT_SGRBG10P, 10, 10, MEDIA_BUS_FMT_SGRBG10_1X10,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW10 },
	{ V4L2_PIX_FMT_SRGGB10P, 10, 10, MEDIA_BUS_FMT_SRGGB10_1X10,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW10 },
	{ V4L2_PIX_FMT_UYVY, 16, 16, MEDIA_BUS_FMT_UYVY8_1X16,
	  IPU6_FW_ISYS_FRAME_FORMAT_UYVY},
	{ V4L2_PIX_FMT_YUYV, 16, 16, MEDIA_BUS_FMT_YUYV8_1X16,
	  IPU6_FW_ISYS_FRAME_FORMAT_YUYV},
	{ V4L2_PIX_FMT_RGB565, 16, 16, MEDIA_BUS_FMT_RGB565_1X16,
	  IPU6_FW_ISYS_FRAME_FORMAT_RGB565 },
	{ V4L2_PIX_FMT_BGR24, 24, 24, MEDIA_BUS_FMT_RGB888_1X24,
	  IPU6_FW_ISYS_FRAME_FORMAT_RGBA888 },
	{ V4L2_META_FMT_GENERIC_8, 8, 8, MEDIA_BUS_FMT_META_8,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW8, true },
	{ V4L2_META_FMT_GENERIC_CSI2_10, 10, 10, MEDIA_BUS_FMT_META_10,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW10, true },
	{ V4L2_META_FMT_GENERIC_CSI2_12, 12, 12, MEDIA_BUS_FMT_META_12,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW12, true },
	{ V4L2_META_FMT_GENERIC_CSI2_16, 16, 16, MEDIA_BUS_FMT_META_16,
	  IPU6_FW_ISYS_FRAME_FORMAT_RAW16, true },
};

static int video_open(struct file *file)
{
	struct ipu6_isys_video *av = video_drvdata(file);
	struct ipu6_isys *isys = av->isys;
	struct ipu6_bus_device *adev = isys->adev;

	mutex_lock(&isys->mutex);
	if (isys->need_reset) {
		mutex_unlock(&isys->mutex);
		dev_warn(&adev->auxdev.dev, "isys power cycle required\n");
		return -EIO;
	}
	mutex_unlock(&isys->mutex);

	return v4l2_fh_open(file);
}

const struct ipu6_isys_pixelformat *
ipu6_isys_get_isys_format(u32 pixelformat, u32 type)
{
	const struct ipu6_isys_pixelformat *default_pfmt = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ipu6_isys_pfmts); i++) {
		const struct ipu6_isys_pixelformat *pfmt = &ipu6_isys_pfmts[i];

		if (type && ((!pfmt->is_meta &&
			      type != V4L2_BUF_TYPE_VIDEO_CAPTURE) ||
			     (pfmt->is_meta &&
			      type != V4L2_BUF_TYPE_META_CAPTURE)))
			continue;

		if (!default_pfmt)
			default_pfmt = pfmt;

		if (pfmt->pixelformat != pixelformat)
			continue;

		return pfmt;
	}

	return default_pfmt;
}

static int ipu6_isys_vidioc_querycap(struct file *file, void *fh,
				     struct v4l2_capability *cap)
{
	struct ipu6_isys_video *av = video_drvdata(file);

	strscpy(cap->driver, IPU6_ISYS_NAME, sizeof(cap->driver));
	strscpy(cap->card, av->isys->media_dev.model, sizeof(cap->card));

	return 0;
}

static int ipu6_isys_vidioc_enum_fmt(struct file *file, void *fh,
				     struct v4l2_fmtdesc *f)
{
	unsigned int i, num_found;

	for (i = 0, num_found = 0; i < ARRAY_SIZE(ipu6_isys_pfmts); i++) {
		if ((ipu6_isys_pfmts[i].is_meta &&
		     f->type != V4L2_BUF_TYPE_META_CAPTURE) ||
		    (!ipu6_isys_pfmts[i].is_meta &&
		     f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE))
			continue;

		if (f->mbus_code && f->mbus_code != ipu6_isys_pfmts[i].code)
			continue;

		if (num_found < f->index) {
			num_found++;
			continue;
		}

		f->flags = 0;
		f->pixelformat = ipu6_isys_pfmts[i].pixelformat;

		return 0;
	}

	return -EINVAL;
}

static int ipu6_isys_vidioc_enum_framesizes(struct file *file, void *fh,
					    struct v4l2_frmsizeenum *fsize)
{
	unsigned int i;

	if (fsize->index > 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ipu6_isys_pfmts); i++) {
		if (fsize->pixel_format != ipu6_isys_pfmts[i].pixelformat)
			continue;

		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise.min_width = IPU6_ISYS_MIN_WIDTH;
		fsize->stepwise.max_width = IPU6_ISYS_MAX_WIDTH;
		fsize->stepwise.min_height = IPU6_ISYS_MIN_HEIGHT;
		fsize->stepwise.max_height = IPU6_ISYS_MAX_HEIGHT;
		fsize->stepwise.step_width = 2;
		fsize->stepwise.step_height = 2;

		return 0;
	}

	return -EINVAL;
}

static int ipu6_isys_vidioc_g_fmt_vid_cap(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	struct ipu6_isys_video *av = video_drvdata(file);

	f->fmt.pix = av->pix_fmt;

	return 0;
}

static int ipu6_isys_vidioc_g_fmt_meta_cap(struct file *file, void *fh,
					   struct v4l2_format *f)
{
	struct ipu6_isys_video *av = video_drvdata(file);

	f->fmt.meta = av->meta_fmt;

	return 0;
}

static void ipu6_isys_try_fmt_cap(struct ipu6_isys_video *av, u32 type,
				  u32 *format, u32 *width, u32 *height,
				  u32 *bytesperline, u32 *sizeimage)
{
	const struct ipu6_isys_pixelformat *pfmt =
		ipu6_isys_get_isys_format(*format, type);

	*format = pfmt->pixelformat;
	*width = clamp(*width, IPU6_ISYS_MIN_WIDTH, IPU6_ISYS_MAX_WIDTH);
	*height = clamp(*height, IPU6_ISYS_MIN_HEIGHT, IPU6_ISYS_MAX_HEIGHT);

	if (pfmt->bpp != pfmt->bpp_packed)
		*bytesperline = *width * DIV_ROUND_UP(pfmt->bpp, BITS_PER_BYTE);
	else
		*bytesperline = DIV_ROUND_UP(*width * pfmt->bpp, BITS_PER_BYTE);

	*bytesperline = ALIGN(*bytesperline, av->isys->line_align);

	/*
	 * (height + 1) * bytesperline due to a hardware issue: the DMA unit
	 * is a power of two, and a line should be transferred as few units
	 * as possible. The result is that up to line length more data than
	 * the image size may be transferred to memory after the image.
	 * Another limitation is the GDA allocation unit size. For low
	 * resolution it gives a bigger number. Use larger one to avoid
	 * memory corruption.
	 */
	*sizeimage = *bytesperline * *height +
		max(*bytesperline,
		    av->isys->pdata->ipdata->isys_dma_overshoot);
}

static void __ipu6_isys_vidioc_try_fmt_vid_cap(struct ipu6_isys_video *av,
					       struct v4l2_format *f)
{
	ipu6_isys_try_fmt_cap(av, f->type, &f->fmt.pix.pixelformat,
			      &f->fmt.pix.width, &f->fmt.pix.height,
			      &f->fmt.pix.bytesperline, &f->fmt.pix.sizeimage);

	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_RAW;
	f->fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	f->fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
	f->fmt.pix.xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int ipu6_isys_vidioc_try_fmt_vid_cap(struct file *file, void *fh,
					    struct v4l2_format *f)
{
	struct ipu6_isys_video *av = video_drvdata(file);

	if (vb2_is_busy(&av->aq.vbq))
		return -EBUSY;

	__ipu6_isys_vidioc_try_fmt_vid_cap(av, f);

	return 0;
}

static int __ipu6_isys_vidioc_try_fmt_meta_cap(struct ipu6_isys_video *av,
					       struct v4l2_format *f)
{
	ipu6_isys_try_fmt_cap(av, f->type, &f->fmt.meta.dataformat,
			      &f->fmt.meta.width, &f->fmt.meta.height,
			      &f->fmt.meta.bytesperline,
			      &f->fmt.meta.buffersize);

	return 0;
}

static int ipu6_isys_vidioc_try_fmt_meta_cap(struct file *file, void *fh,
					     struct v4l2_format *f)
{
	struct ipu6_isys_video *av = video_drvdata(file);

	__ipu6_isys_vidioc_try_fmt_meta_cap(av, f);

	return 0;
}

static int ipu6_isys_vidioc_s_fmt_vid_cap(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	struct ipu6_isys_video *av = video_drvdata(file);

	ipu6_isys_vidioc_try_fmt_vid_cap(file, fh, f);
	av->pix_fmt = f->fmt.pix;

	return 0;
}

static int ipu6_isys_vidioc_s_fmt_meta_cap(struct file *file, void *fh,
					   struct v4l2_format *f)
{
	struct ipu6_isys_video *av = video_drvdata(file);

	if (vb2_is_busy(&av->aq.vbq))
		return -EBUSY;

	ipu6_isys_vidioc_try_fmt_meta_cap(file, fh, f);
	av->meta_fmt = f->fmt.meta;

	return 0;
}

static int ipu6_isys_vidioc_reqbufs(struct file *file, void *priv,
				    struct v4l2_requestbuffers *p)
{
	struct ipu6_isys_video *av = video_drvdata(file);
	int ret;

	av->aq.vbq.is_multiplanar = V4L2_TYPE_IS_MULTIPLANAR(p->type);
	av->aq.vbq.is_output = V4L2_TYPE_IS_OUTPUT(p->type);

	ret = vb2_queue_change_type(&av->aq.vbq, p->type);
	if (ret)
		return ret;

	return vb2_ioctl_reqbufs(file, priv, p);
}

static int ipu6_isys_vidioc_create_bufs(struct file *file, void *priv,
					struct v4l2_create_buffers *p)
{
	struct ipu6_isys_video *av = video_drvdata(file);
	int ret;

	av->aq.vbq.is_multiplanar = V4L2_TYPE_IS_MULTIPLANAR(p->format.type);
	av->aq.vbq.is_output = V4L2_TYPE_IS_OUTPUT(p->format.type);

	ret = vb2_queue_change_type(&av->aq.vbq, p->format.type);
	if (ret)
		return ret;

	return vb2_ioctl_create_bufs(file, priv, p);
}

static int link_validate(struct media_link *link)
{
	struct ipu6_isys_video *av =
		container_of(link->sink, struct ipu6_isys_video, pad);
	struct device *dev = &av->isys->adev->auxdev.dev;

	dev_info(dev,
		 "[IPU-CHK] video_link_validate: force pass %s:%u -> %s\n",
		 link->source->entity->name, link->source->index,
		 link->sink->entity->name);

	return 0;
}

static void get_stream_opened(struct ipu6_isys_video *av)
{
	unsigned long flags;

	spin_lock_irqsave(&av->isys->streams_lock, flags);
	av->isys->stream_opened++;
	spin_unlock_irqrestore(&av->isys->streams_lock, flags);
}

static void put_stream_opened(struct ipu6_isys_video *av)
{
	unsigned long flags;

	spin_lock_irqsave(&av->isys->streams_lock, flags);
	av->isys->stream_opened--;
	spin_unlock_irqrestore(&av->isys->streams_lock, flags);
}

static int ipu6_isys_fw_pin_cfg(struct ipu6_isys_video *av,
				struct ipu6_fw_isys_stream_cfg_data_abi *cfg)
{
	struct media_pad *src_pad = media_pad_remote_pad_first(&av->pad);
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(src_pad->entity);
	struct ipu6_fw_isys_input_pin_info_abi *input_pin;
	struct ipu6_fw_isys_output_pin_info_abi *output_pin;
	struct ipu6_isys_stream *stream = av->stream;
	struct ipu6_isys_queue *aq = &av->aq;
	struct v4l2_mbus_framefmt fmt;
	const struct ipu6_isys_pixelformat *pfmt =
		ipu6_isys_get_isys_format(ipu6_isys_get_format(av), 0);
	struct v4l2_rect v4l2_crop;
	struct ipu6_isys *isys = av->isys;
	struct device *dev = &isys->adev->auxdev.dev;
	int input_pins = cfg->nof_input_pins++;
	int output_pins;
	u32 src_stream;
	int ret;

	src_stream = ipu6_isys_get_src_stream_by_src_pad(sd, src_pad->index);
	ret = ipu6_isys_get_stream_pad_fmt(sd, src_pad->index, src_stream,
					   &fmt);
	if (ret < 0) {
		dev_err(dev, "can't get stream format (%d)\n", ret);
		return ret;
	}

	ret = ipu6_isys_get_stream_pad_crop(sd, src_pad->index, src_stream,
					    &v4l2_crop);
	if (ret < 0) {
		dev_err(dev, "can't get stream crop (%d)\n", ret);
		return ret;
	}

	input_pin = &cfg->input_pins[input_pins];
	input_pin->input_res.width = fmt.width;
	input_pin->input_res.height = fmt.height;
	input_pin->dt = av->dt;
	input_pin->bits_per_pix = pfmt->bpp_packed;
	input_pin->mapped_dt = 0x40; /* invalid mipi data type */
	input_pin->mipi_decompression = 0;
	input_pin->capture_mode = IPU6_FW_ISYS_CAPTURE_MODE_REGULAR;
	input_pin->mipi_store_mode = pfmt->bpp == pfmt->bpp_packed ?
		IPU6_FW_ISYS_MIPI_STORE_MODE_DISCARD_LONG_HEADER :
		IPU6_FW_ISYS_MIPI_STORE_MODE_NORMAL;
	input_pin->crop_first_and_last_lines = v4l2_crop.top & 1;

	output_pins = cfg->nof_output_pins++;
	aq->fw_output = output_pins;
	stream->output_pins[output_pins].pin_ready = ipu6_isys_queue_buf_ready;
	stream->output_pins[output_pins].aq = aq;

	output_pin = &cfg->output_pins[output_pins];
	output_pin->input_pin_id = input_pins;
	output_pin->output_res.width = ipu6_isys_get_frame_width(av);
	output_pin->output_res.height = ipu6_isys_get_frame_height(av);

	output_pin->stride = ipu6_isys_get_bytes_per_line(av);
	if (pfmt->bpp != pfmt->bpp_packed)
		output_pin->pt = IPU6_FW_ISYS_PIN_TYPE_RAW_SOC;
	else
		output_pin->pt = IPU6_FW_ISYS_PIN_TYPE_MIPI;
	output_pin->ft = pfmt->css_pixelformat;
	output_pin->send_irq = 1;
	memset(output_pin->ts_offsets, 0, sizeof(output_pin->ts_offsets));
	output_pin->s2m_pixel_soc_pixel_remapping =
		S2M_PIXEL_SOC_PIXEL_REMAPPING_FLAG_NO_REMAPPING;
	output_pin->csi_be_soc_pixel_remapping =
		CSI_BE_SOC_PIXEL_REMAPPING_FLAG_NO_REMAPPING;

	output_pin->snoopable = true;
	output_pin->error_handling_enable = false;
	output_pin->sensor_type = isys->sensor_type++;
	if (isys->sensor_type > isys->pdata->ipdata->sensor_type_end)
		isys->sensor_type = isys->pdata->ipdata->sensor_type_start;

	return 0;
}

static int start_stream_firmware(struct ipu6_isys_video *av,
				 struct ipu6_isys_buffer_list *bl)
{
	struct ipu6_fw_isys_stream_cfg_data_abi *stream_cfg;
	struct ipu6_fw_isys_frame_buff_set_abi *buf = NULL;
	struct ipu6_isys_stream *stream = av->stream;
	struct device *dev = &av->isys->adev->auxdev.dev;
	struct isys_fw_msgs *msg = NULL;
	struct ipu6_isys_queue *aq;
	int ret, retout, tout;
	u16 send_type;

	dev_info(dev,
		 "[IPU-CHK] start_stream_fw: vdev=%s stream=%p handle=%u src=%u vc=%u bl=%p\n",
		 av->vdev.name, stream,
		 stream ? stream->stream_handle : 0,
		 stream ? stream->stream_source : 0,
		 stream ? stream->vc : 0,
		 bl);

	msg = ipu6_get_fw_msg_buf(stream);
	if (!msg) {
		dev_err(dev, "[IPU-CHK] start_stream_fw: no fw msg buf\n");
		return -ENOMEM;
	}

	stream_cfg = &msg->fw_msg.stream;
	stream_cfg->src = stream->stream_source;
	stream_cfg->vc = stream->vc;
	stream_cfg->isl_use = 0;
	stream_cfg->sensor_type = IPU6_FW_ISYS_SENSOR_MODE_NORMAL;

	dev_info(dev,
	 "[IPU-CHK] start_stream_fw: building stream cfg, current nr_queues=%u\n",
	 stream->nr_queues);

	list_for_each_entry(aq, &stream->queues, node) {
		struct ipu6_isys_video *__av = ipu6_isys_queue_to_video(aq);

		dev_info(dev,
			"[IPU-CHK] start_stream_fw: queue video=%s vc=%u source_stream=%u\n",
			__av->vdev.name, __av->vc, __av->source_stream);

		ret = ipu6_isys_fw_pin_cfg(__av, stream_cfg);
		if (ret < 0) {
			dev_err(dev,
				"[IPU-CHK] start_stream_fw: fw_pin_cfg failed for %s: %d\n",
				__av->vdev.name, ret);
			ipu6_put_fw_msg_buf(av->isys, (u64)stream_cfg);
			return ret;
		}
	}
	
	ipu6_fw_isys_dump_stream_cfg(dev, stream_cfg);

	stream->nr_output_pins = stream_cfg->nof_output_pins;

	reinit_completion(&stream->stream_open_completion);

	ret = ipu6_fw_isys_complex_cmd(av->isys, stream->stream_handle,
				       stream_cfg, msg->dma_addr,
				       sizeof(*stream_cfg),
				       IPU6_FW_ISYS_SEND_TYPE_STREAM_OPEN);
	if (ret < 0) {
		dev_err(dev, "can't open stream (%d)\n", ret);
		ipu6_put_fw_msg_buf(av->isys, (u64)stream_cfg);
		return ret;
	}

	get_stream_opened(av);

	tout = wait_for_completion_timeout(&stream->stream_open_completion,
					   IPU6_FW_CALL_TIMEOUT_JIFFIES);

	ipu6_put_fw_msg_buf(av->isys, (u64)stream_cfg);

	if (!tout) {
		dev_err(dev, "stream open time out\n");
		ret = -ETIMEDOUT;
		goto out_put_stream_opened;
	}
	if (stream->error) {
		dev_err(dev, "stream open error: %d\n", stream->error);
		ret = -EIO;
		goto out_put_stream_opened;
	}
	dev_dbg(dev, "start stream: open complete\n");

	if (bl) {
		msg = ipu6_get_fw_msg_buf(stream);
		if (!msg) {
			ret = -ENOMEM;
			goto out_put_stream_opened;
		}
		buf = &msg->fw_msg.frame;
		ipu6_isys_buf_to_fw_frame_buf(buf, stream, bl);
		ipu6_isys_buffer_list_queue(bl,
					    IPU6_ISYS_BUFFER_LIST_FL_ACTIVE, 0);
	}

	reinit_completion(&stream->stream_start_completion);

	if (bl) {
		send_type = IPU6_FW_ISYS_SEND_TYPE_STREAM_START_AND_CAPTURE;
		ipu6_fw_isys_dump_frame_buff_set(dev, buf,
						 stream_cfg->nof_output_pins);
		ret = ipu6_fw_isys_complex_cmd(av->isys, stream->stream_handle,
					       buf, msg->dma_addr,
					       sizeof(*buf), send_type);
	} else {
		send_type = IPU6_FW_ISYS_SEND_TYPE_STREAM_START;
		ret = ipu6_fw_isys_simple_cmd(av->isys, stream->stream_handle,
					      send_type);
	}

	if (ret < 0) {
		dev_err(dev, "can't start streaming (%d)\n", ret);
		goto out_stream_close;
	}

	tout = wait_for_completion_timeout(&stream->stream_start_completion,
					   IPU6_FW_CALL_TIMEOUT_JIFFIES);
	if (!tout) {
		dev_err(dev, "stream start time out\n");
		ret = -ETIMEDOUT;
		goto out_stream_close;
	}
	if (stream->error) {
		dev_err(dev, "stream start error: %d\n", stream->error);
		ret = -EIO;
		goto out_stream_close;
	}
	dev_dbg(dev, "start stream: complete\n");

	return 0;

out_stream_close:
	reinit_completion(&stream->stream_close_completion);

	retout = ipu6_fw_isys_simple_cmd(av->isys,
					 stream->stream_handle,
					 IPU6_FW_ISYS_SEND_TYPE_STREAM_CLOSE);
	if (retout < 0) {
		dev_dbg(dev, "can't close stream (%d)\n", retout);
		goto out_put_stream_opened;
	}

	tout = wait_for_completion_timeout(&stream->stream_close_completion,
					   IPU6_FW_CALL_TIMEOUT_JIFFIES);
	if (!tout)
		dev_err(dev, "stream close time out\n");
	else if (stream->error)
		dev_err(dev, "stream close error: %d\n", stream->error);
	else
		dev_dbg(dev, "stream close complete\n");

out_put_stream_opened:
	put_stream_opened(av);

	return ret;
}

static void stop_streaming_firmware(struct ipu6_isys_video *av)
{
	struct device *dev = &av->isys->adev->auxdev.dev;
	struct ipu6_isys_stream *stream = av->stream;
	int ret, tout;

	reinit_completion(&stream->stream_stop_completion);

	ret = ipu6_fw_isys_simple_cmd(av->isys, stream->stream_handle,
				      IPU6_FW_ISYS_SEND_TYPE_STREAM_FLUSH);

	if (ret < 0) {
		dev_err(dev, "can't stop stream (%d)\n", ret);
		return;
	}

	tout = wait_for_completion_timeout(&stream->stream_stop_completion,
					   IPU6_FW_CALL_TIMEOUT_JIFFIES);
	if (!tout)
		dev_warn(dev, "stream stop time out\n");
	else if (stream->error)
		dev_warn(dev, "stream stop error: %d\n", stream->error);
	else
		dev_dbg(dev, "stop stream: complete\n");
}

static void close_streaming_firmware(struct ipu6_isys_video *av)
{
	struct ipu6_isys_stream *stream = av->stream;
	struct device *dev = &av->isys->adev->auxdev.dev;
	int ret, tout;

	reinit_completion(&stream->stream_close_completion);

	ret = ipu6_fw_isys_simple_cmd(av->isys, stream->stream_handle,
				      IPU6_FW_ISYS_SEND_TYPE_STREAM_CLOSE);
	if (ret < 0) {
		dev_err(dev, "can't close stream (%d)\n", ret);
		return;
	}

	tout = wait_for_completion_timeout(&stream->stream_close_completion,
					   IPU6_FW_CALL_TIMEOUT_JIFFIES);
	if (!tout)
		dev_warn(dev, "stream close time out\n");
	else if (stream->error)
		dev_warn(dev, "stream close error: %d\n", stream->error);
	else
		dev_dbg(dev, "close stream: complete\n");

	put_stream_opened(av);
}

int ipu6_isys_video_prepare_stream(struct ipu6_isys_video *av,
				   struct media_entity *source_entity,
				   int nr_queues)
{
	struct ipu6_isys_stream *stream = av->stream;
	struct ipu6_isys_csi2 *csi2;
	struct device *dev = &av->isys->adev->auxdev.dev;

	dev_info(dev,
		 "[IPU-CHK] prepare_stream: vdev=%s stream=%p vc=%u nr_queues=%d nr_streaming=%u queues_empty=%d source=%s\n",
		 av->vdev.name, stream, av->vc, nr_queues,
		 stream ? stream->nr_streaming : 0,
		 stream ? list_empty(&stream->queues) : -1,
		 source_entity ? source_entity->name : "(null)");

	if (WARN_ON(stream->nr_streaming)) {
		dev_err(dev,
			"[IPU-CHK] prepare_stream: reject because nr_streaming=%u\n",
			stream->nr_streaming);
		return -EINVAL;
	}

	stream->nr_queues = nr_queues;
	atomic_set(&stream->sequence, 0);

	stream->seq_index = 0;
	memset(stream->seq, 0, sizeof(stream->seq));

	if (WARN_ON(!list_empty(&stream->queues))) {
		dev_err(dev,
			"[IPU-CHK] prepare_stream: reject because stream->queues is not empty\n");
		return -EINVAL;
	}

	stream->stream_source = stream->asd->source;
	csi2 = ipu6_isys_subdev_to_csi2(stream->asd);
	csi2->receiver_errors = 0;
	stream->source_entity = source_entity;

	dev_info(dev,
		 "[IPU-CHK] prepare_stream: done vdev=%s stream_source=%d vc=%u nr_queues=%d\n",
		 av->vdev.name, stream->stream_source, stream->vc,
		 stream->nr_queues);

	return 0;
}

void ipu6_isys_configure_stream_watermark(struct ipu6_isys_video *av,
					  bool state)
{
	struct ipu6_isys *isys = av->isys;
	struct ipu6_isys_csi2 *csi2 = NULL;
	struct isys_iwake_watermark *iwake_watermark = &isys->iwake_watermark;
	struct device *dev = &isys->adev->auxdev.dev;
	struct v4l2_mbus_framefmt format;
	struct v4l2_subdev *esd;
	struct v4l2_control hb = { .id = V4L2_CID_HBLANK, .value = 0 };
	unsigned int bpp, lanes;
	s64 link_freq = 0;
	u64 pixel_rate = 0;
	int ret;

	if (!state)
		return;

	esd = media_entity_to_v4l2_subdev(av->stream->source_entity);

	av->watermark.width = ipu6_isys_get_frame_width(av);
	av->watermark.height = ipu6_isys_get_frame_height(av);
	av->watermark.sram_gran_shift = isys->pdata->ipdata->sram_gran_shift;
	av->watermark.sram_gran_size = isys->pdata->ipdata->sram_gran_size;

	ret = v4l2_g_ctrl(esd->ctrl_handler, &hb);
	if (!ret && hb.value >= 0)
		av->watermark.hblank = hb.value;
	else
		av->watermark.hblank = 0;

	csi2 = ipu6_isys_subdev_to_csi2(av->stream->asd);
	link_freq = ipu6_isys_csi2_get_link_freq(csi2);
	if (link_freq > 0) {
		lanes = csi2->nlanes;
		ret = ipu6_isys_get_stream_pad_fmt(&csi2->asd.sd, 0,
						   av->source_stream, &format);
		if (!ret) {
			bpp = ipu6_isys_mbus_code_to_bpp(format.code);
			pixel_rate = mul_u64_u32_div(link_freq, lanes * 2, bpp);
		}
	}

	av->watermark.pixel_rate = pixel_rate;

	if (!pixel_rate) {
		mutex_lock(&iwake_watermark->mutex);
		iwake_watermark->force_iwake_disable = true;
		mutex_unlock(&iwake_watermark->mutex);
		dev_warn(dev, "unexpected pixel_rate from %s, disable iwake.\n",
			 av->stream->source_entity->name);
	}
}

static void calculate_stream_datarate(struct ipu6_isys_video *av)
{
	struct video_stream_watermark *watermark = &av->watermark;
	const struct ipu6_isys_pixelformat *pfmt =
		ipu6_isys_get_isys_format(ipu6_isys_get_format(av), 0);
	u32 pages_per_line, pb_bytes_per_line, pixels_per_line, bytes_per_line;
	u64 line_time_ns, stream_data_rate;
	u16 shift, size;

	shift = watermark->sram_gran_shift;
	size = watermark->sram_gran_size;

	pixels_per_line = watermark->width + watermark->hblank;
	line_time_ns =  div_u64(pixels_per_line * NSEC_PER_SEC,
				watermark->pixel_rate);
	bytes_per_line = watermark->width * pfmt->bpp / 8;
	pages_per_line = DIV_ROUND_UP(bytes_per_line, size);
	pb_bytes_per_line = pages_per_line << shift;
	stream_data_rate = div64_u64(pb_bytes_per_line * 1000, line_time_ns);

	watermark->stream_data_rate = stream_data_rate;
}

void ipu6_isys_update_stream_watermark(struct ipu6_isys_video *av, bool state)
{
	struct isys_iwake_watermark *iwake_watermark =
		&av->isys->iwake_watermark;

	if (!av->watermark.pixel_rate)
		return;

	if (state) {
		calculate_stream_datarate(av);
		mutex_lock(&iwake_watermark->mutex);
		list_add(&av->watermark.stream_node,
			 &iwake_watermark->video_list);
		mutex_unlock(&iwake_watermark->mutex);
	} else {
		av->watermark.stream_data_rate = 0;
		mutex_lock(&iwake_watermark->mutex);
		list_del(&av->watermark.stream_node);
		mutex_unlock(&iwake_watermark->mutex);
	}

	update_watermark_setting(av->isys);
}

void ipu6_isys_put_stream(struct ipu6_isys_stream *stream)
{
	struct device *dev;
	unsigned int i;
	unsigned long flags;

	if (!stream) {
		pr_err("ipu6-isys: no available stream\n");
		return;
	}

	dev = &stream->isys->adev->auxdev.dev;

	spin_lock_irqsave(&stream->isys->streams_lock, flags);
	for (i = 0; i < IPU6_ISYS_MAX_STREAMS; i++) {
		if (&stream->isys->streams[i] == stream) {
			if (stream->isys->streams_ref_count[i] > 0)
				stream->isys->streams_ref_count[i]--;
			else
				dev_warn(dev, "invalid stream %d\n", i);

			break;
		}
	}
	spin_unlock_irqrestore(&stream->isys->streams_lock, flags);
}

static struct ipu6_isys_stream *
ipu6_isys_get_stream(struct ipu6_isys_video *av, struct ipu6_isys_subdev *asd)
{
	struct ipu6_isys_stream *stream = NULL;
	struct ipu6_isys *isys = av->isys;
	unsigned long flags;
	unsigned int i;
	u8 vc = av->vc;

	if (!isys)
		return NULL;

	spin_lock_irqsave(&isys->streams_lock, flags);
	for (i = 0; i < IPU6_ISYS_MAX_STREAMS; i++) {
		if (isys->streams_ref_count[i] && isys->streams[i].vc == vc &&
		    isys->streams[i].asd == asd) {
			isys->streams_ref_count[i]++;
			stream = &isys->streams[i];
			break;
		}
	}

	if (!stream) {
		for (i = 0; i < IPU6_ISYS_MAX_STREAMS; i++) {
			if (!isys->streams_ref_count[i]) {
				isys->streams_ref_count[i]++;
				stream = &isys->streams[i];
				stream->vc = vc;
				stream->asd = asd;
				break;
			}
		}
	}
	spin_unlock_irqrestore(&isys->streams_lock, flags);

	return stream;
}

struct ipu6_isys_stream *
ipu6_isys_query_stream_by_handle(struct ipu6_isys *isys, u8 stream_handle)
{
	unsigned long flags;
	struct ipu6_isys_stream *stream = NULL;

	if (!isys)
		return NULL;

	if (stream_handle >= IPU6_ISYS_MAX_STREAMS) {
		dev_err(&isys->adev->auxdev.dev,
			"stream_handle %d is invalid\n", stream_handle);
		return NULL;
	}

	spin_lock_irqsave(&isys->streams_lock, flags);
	if (isys->streams_ref_count[stream_handle] > 0) {
		isys->streams_ref_count[stream_handle]++;
		stream = &isys->streams[stream_handle];
	}
	spin_unlock_irqrestore(&isys->streams_lock, flags);

	return stream;
}

struct ipu6_isys_stream *
ipu6_isys_query_stream_by_source(struct ipu6_isys *isys, int source, u8 vc)
{
	struct ipu6_isys_stream *stream = NULL;
	unsigned long flags;
	unsigned int i;

	if (!isys)
		return NULL;

	if (source < 0) {
		dev_err(&isys->adev->auxdev.dev,
			"query stream with invalid port number\n");
		return NULL;
	}

	spin_lock_irqsave(&isys->streams_lock, flags);
	for (i = 0; i < IPU6_ISYS_MAX_STREAMS; i++) {
		if (!isys->streams_ref_count[i])
			continue;

		if (isys->streams[i].stream_source == source &&
		    isys->streams[i].vc == vc) {
			stream = &isys->streams[i];
			isys->streams_ref_count[i]++;
			break;
		}
	}
	spin_unlock_irqrestore(&isys->streams_lock, flags);

	return stream;
}

static u64 get_stream_mask_by_pipeline(struct ipu6_isys_video *__av)
{
	struct media_pipeline *pipeline =
		media_entity_pipeline(&__av->vdev.entity);
	unsigned int i;
	u64 stream_mask = 0;

	for (i = 0; i < NR_OF_CSI2_SRC_PADS; i++) {
		struct ipu6_isys_video *av = &__av->csi2->av[i];

		if (pipeline == media_entity_pipeline(&av->vdev.entity))
			stream_mask |= BIT_ULL(av->source_stream);
	}

	return stream_mask;
}

int ipu6_isys_video_set_streaming(struct ipu6_isys_video *av, int state,
				  struct ipu6_isys_buffer_list *bl)
{
	struct v4l2_subdev_krouting *routing;
	struct ipu6_isys_stream *stream = av->stream;
	struct v4l2_subdev_state *subdev_state;
	struct device *dev = &av->isys->adev->auxdev.dev;
	struct v4l2_subdev *sd;
	struct media_pad *r_pad;
	u32 sink_pad, sink_stream;
	u64 r_stream;
	u64 stream_mask = 0;
	int ret = 0;

	dev_dbg(dev, "set stream: %d\n", state);

	if (WARN(!stream->source_entity, "No source entity for stream\n"))
		return -ENODEV;

	sd = &stream->asd->sd;
	r_pad = media_pad_remote_pad_first(&av->pad);
	if (!r_pad) {
		dev_err(dev, "[IPU-CHK] set_streaming: no remote pad for %s\n",
			av->vdev.name);
		return -ENODEV;
	}

	r_stream = ipu6_isys_get_src_stream_by_src_pad(sd, r_pad->index);

	subdev_state = v4l2_subdev_lock_and_get_active_state(sd);
	routing = &subdev_state->routing;
	ret = v4l2_subdev_routing_find_opposite_end(routing, r_pad->index,
						    r_stream, &sink_pad,
						    &sink_stream);
	v4l2_subdev_unlock_state(subdev_state);
	if (ret) {
		dev_err(dev,
			"[IPU-CHK] set_streaming: find_opposite_end failed: vdev=%s r_pad=%u r_stream=%llu ret=%d\n",
			av->vdev.name, r_pad->index,
			(unsigned long long)r_stream, ret);
		return ret;
	}

	/*
	 * 關鍵修正：
	 * 這裡不能再用 get_stream_mask_by_pipeline(av)。
	 *
	 * 對於 video33：
	 *   r_pad = 2
	 *   r_stream = 1
	 * 所以要送給 source pad2 的 mask 應該是 BIT_ULL(1) = 0x2，
	 * 不是 pipeline 算出來的 0x1。
	 */
	stream_mask = BIT_ULL(r_stream);

	dev_info(dev,
		 "[IPU-CHK] set_streaming(pre): vdev=%s state=%d r_pad=%u r_stream=%llu sink_pad=%u sink_stream=%u stream_mask=0x%llx\n",
		 av->vdev.name, state, r_pad->index,
		 (unsigned long long)r_stream,
		 sink_pad, sink_stream,
		 (unsigned long long)stream_mask);

	if (!state) {
		stop_streaming_firmware(av);

		dev_info(dev,
			 "[IPU-CHK] set_streaming(off): sd=%s pad=%u mask=0x%llx\n",
			 sd->name, r_pad->index,
			 (unsigned long long)stream_mask);

		ret = v4l2_subdev_disable_streams(sd, r_pad->index, stream_mask);
		if (ret) {
			dev_err(dev, "stream off %s failed with %d\n",
				sd->name, ret);
			return ret;
		}

		close_streaming_firmware(av);
	} else {
		ret = start_stream_firmware(av, bl);
		if (ret) {
			dev_err(dev, "start stream of firmware failed\n");
			return ret;
		}

		dev_info(dev,
			 "[IPU-CHK] set_streaming(on): sd=%s pad=%u mask=0x%llx\n",
			 sd->name, r_pad->index,
			 (unsigned long long)stream_mask);

		ret = v4l2_subdev_enable_streams(sd, r_pad->index, stream_mask);
		if (ret) {
			dev_err(dev,
				"stream on %s failed with %d (pad=%u r_stream=%llu mask=0x%llx)\n",
				sd->name, ret, r_pad->index,
				(unsigned long long)r_stream,
				(unsigned long long)stream_mask);
			goto out_media_entity_stop_streaming_firmware;
		}
	}

	av->streaming = state;

	dev_info(dev,
		 "[IPU-CHK] set_streaming(done): vdev=%s r_pad=%u r_stream=%llu stream_mask=0x%llx state=%d\n",
		 av->vdev.name, r_pad->index,
		 (unsigned long long)r_stream,
		 (unsigned long long)stream_mask, state);

	return 0;

out_media_entity_stop_streaming_firmware:
	stop_streaming_firmware(av);
	return ret;
}

static const struct v4l2_ioctl_ops ipu6_v4l2_ioctl_ops = {
	.vidioc_querycap = ipu6_isys_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = ipu6_isys_vidioc_enum_fmt,
	.vidioc_enum_fmt_meta_cap = ipu6_isys_vidioc_enum_fmt,
	.vidioc_enum_framesizes = ipu6_isys_vidioc_enum_framesizes,
	.vidioc_g_fmt_vid_cap = ipu6_isys_vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = ipu6_isys_vidioc_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = ipu6_isys_vidioc_try_fmt_vid_cap,
	.vidioc_g_fmt_meta_cap = ipu6_isys_vidioc_g_fmt_meta_cap,
	.vidioc_s_fmt_meta_cap = ipu6_isys_vidioc_s_fmt_meta_cap,
	.vidioc_try_fmt_meta_cap = ipu6_isys_vidioc_try_fmt_meta_cap,
	.vidioc_reqbufs = ipu6_isys_vidioc_reqbufs,
	.vidioc_create_bufs = ipu6_isys_vidioc_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_expbuf = vb2_ioctl_expbuf,
};

static const struct media_entity_operations entity_ops = {
	.link_validate = link_validate,
};

static const struct v4l2_file_operations isys_fops = {
	.owner = THIS_MODULE,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.open = video_open,
	.release = vb2_fop_release,
};

int ipu6_isys_fw_open(struct ipu6_isys *isys)
{
	struct ipu6_bus_device *adev = isys->adev;
	const struct ipu6_isys_internal_pdata *ipdata = isys->pdata->ipdata;
	int ret;

	ret = pm_runtime_resume_and_get(&adev->auxdev.dev);
	if (ret < 0)
		return ret;

	mutex_lock(&isys->mutex);

	if (isys->ref_count++)
		goto unlock;

	ipu6_configure_spc(adev->isp, &ipdata->hw_variant,
			   IPU6_CPD_PKG_DIR_ISYS_SERVER_IDX, isys->pdata->base,
			   adev->pkg_dir, adev->pkg_dir_dma_addr);

	/*
	 * Buffers could have been left to wrong queue at last closure.
	 * Move them now back to empty buffer queue.
	 */
	ipu6_cleanup_fw_msg_bufs(isys);

	if (isys->fwcom) {
		/*
		 * Something went wrong in previous shutdown. As we are now
		 * restarting isys we can safely delete old context.
		 */
		dev_warn(&adev->auxdev.dev, "clearing old context\n");
		ipu6_fw_isys_cleanup(isys);
	}

	ret = ipu6_fw_isys_init(isys, ipdata->num_parallel_streams);
	if (ret < 0)
		goto out;

unlock:
	mutex_unlock(&isys->mutex);

	return 0;

out:
	isys->ref_count--;
	mutex_unlock(&isys->mutex);
	pm_runtime_put(&adev->auxdev.dev);

	return ret;
}

void ipu6_isys_fw_close(struct ipu6_isys *isys)
{
	mutex_lock(&isys->mutex);

	isys->ref_count--;
	if (!isys->ref_count) {
		ipu6_fw_isys_close(isys);
		if (isys->fwcom) {
			isys->need_reset = true;
			dev_warn(&isys->adev->auxdev.dev,
				 "failed to close fw isys\n");
		}
	}

	mutex_unlock(&isys->mutex);

	if (isys->need_reset)
		pm_runtime_put_sync(&isys->adev->auxdev.dev);
	else
		pm_runtime_put(&isys->adev->auxdev.dev);
}

int ipu6_isys_setup_video(struct ipu6_isys_video *av,
			  struct media_entity **source_entity, int *nr_queues)
{
	const struct ipu6_isys_pixelformat *pfmt =
		ipu6_isys_get_isys_format(ipu6_isys_get_format(av), 0);
	struct device *dev = &av->isys->adev->auxdev.dev;
	struct v4l2_mbus_frame_desc_entry entry;
	struct v4l2_subdev_route *route = NULL;
	struct v4l2_subdev_route *r;
	struct v4l2_subdev_state *state;
	struct ipu6_isys_subdev *asd;
	struct v4l2_subdev *remote_sd;
	struct media_pipeline *pipeline;
	struct media_pad *source_pad, *remote_pad;
	int ret = -EINVAL;

	*nr_queues = 0;

	dev_info(dev, "[IPU-CHK] setup_video() start: vdev=%s\n",
		 av->vdev.name);

	remote_pad = media_pad_remote_pad_unique(&av->pad);
	if (IS_ERR(remote_pad)) {
		dev_err(dev, "[IPU-CHK] failed to get remote pad: %ld\n",
			PTR_ERR(remote_pad));
		return PTR_ERR(remote_pad);
	}

	dev_info(dev,
		 "[IPU-CHK] remote_pad: entity=%s index=%u flags=0x%x\n",
		 remote_pad->entity->name, remote_pad->index, remote_pad->flags);

	remote_sd = media_entity_to_v4l2_subdev(remote_pad->entity);
	asd = to_ipu6_isys_subdev(remote_sd);

	dev_info(dev, "[IPU-CHK] remote_sd=%s\n", remote_sd->name);

	source_pad = media_pad_remote_pad_first(&remote_pad->entity->pads[0]);
	if (!source_pad) {
		dev_err(dev, "[IPU-CHK] no external source entity\n");
		return -ENODEV;
	}

	*source_entity = source_pad->entity;

	dev_info(dev,
		 "[IPU-CHK] source_entity=%s source_pad=%u\n",
		 (*source_entity)->name, source_pad->index);

	state = v4l2_subdev_lock_and_get_active_state(remote_sd);

	dev_info(dev,
		 "[IPU-CHK] active routing num_routes=%u for remote_sd=%s\n",
		 state->routing.num_routes, remote_sd->name);

	for_each_active_route(&state->routing, r) {
		(*nr_queues)++;

		dev_info(dev,
			 "[IPU-CHK] route[%d]: sink %u:%u -> source %u:%u flags=0x%x\n",
			 *nr_queues - 1,
			 r->sink_pad, r->sink_stream,
			 r->source_pad, r->source_stream,
			 r->flags);

		if (r->source_pad == remote_pad->index) {
			route = r;
			dev_info(dev,
				 "[IPU-CHK] matched route for remote_pad=%u: sink %u:%u -> source %u:%u\n",
				 remote_pad->index,
				 r->sink_pad, r->sink_stream,
				 r->source_pad, r->source_stream);
		}
	}

	if (!route) {
		v4l2_subdev_unlock_state(state);
		dev_err(dev,
			"[IPU-CHK] failed to find route for remote_pad=%u\n",
			remote_pad->index);
		return -ENODEV;
	}

	av->source_stream = route->sink_stream;

	dev_info(dev,
		 "[IPU-CHK] chosen source_stream=%u for vdev=%s\n",
		 av->source_stream, av->vdev.name);

	v4l2_subdev_unlock_state(state);

	ret = ipu6_isys_csi2_get_remote_desc(av->source_stream,
					     to_ipu6_isys_csi2(asd),
					     *source_entity, &entry);

	if (ret == -ENOIOCTLCMD) {
		av->vc = 0;
		av->dt = ipu6_isys_mbus_code_to_mipi(pfmt->code);

		dev_info(dev,
			 "[IPU-CHK] get_remote_desc: ENOIOCTLCMD, fallback vc=%u dt=0x%x\n",
			 av->vc, av->dt);
	} else if (!ret) {
		dev_info(dev,
			 "[IPU-CHK] get_remote_desc OK: entry.stream=%u len=%u vc=%u dt=0x%x\n",
			 entry.stream, entry.length,
			 entry.bus.csi2.vc, entry.bus.csi2.dt);

		av->vc = entry.bus.csi2.vc;
		av->dt = entry.bus.csi2.dt;
	} else {
		dev_err(dev,
			"[IPU-CHK] failed to get remote frame desc: %d\n", ret);
		return ret;
	}

	pipeline = media_entity_pipeline(&av->vdev.entity);
	if (!pipeline) {
		dev_info(dev,
			 "[IPU-CHK] no pipeline yet, alloc_start for vdev=%s\n",
			 av->vdev.name);
		ret = video_device_pipeline_alloc_start(&av->vdev);
	} else {
		dev_info(dev,
			 "[IPU-CHK] existing pipeline, pipeline_start for vdev=%s\n",
			 av->vdev.name);
		ret = video_device_pipeline_start(&av->vdev, pipeline);
	}

	if (ret < 0) {
		dev_err(dev,
			"[IPU-CHK] media pipeline start failed: %d\n", ret);
		return ret;
	}

	av->stream = ipu6_isys_get_stream(av, asd);
	if (!av->stream) {
		video_device_pipeline_stop(&av->vdev);
		dev_err(dev,
			"[IPU-CHK] no available firmware stream for vdev=%s\n",
			av->vdev.name);
		return -EINVAL;
	}

	/*
	 * DEBUG MODE:
	 * 目前 video32 / video33 已經各自拿到不同 stream object，
	 * 不該再要求 queue 2 of 2 才 start。
	 * 先強制每個 video node 自己用 1 queue 驗證流程。
	 */
	*nr_queues = 1;

	dev_info(dev,
		 "[IPU-CHK] setup_video done: vdev=%s source_stream=%u vc=%u dt=0x%x nr_queues=%d stream=%p (force nr_queues=1)\n",
		 av->vdev.name, av->source_stream, av->vc, av->dt,
		 *nr_queues, av->stream);

	return 0;
}
/*
 * Do everything that's needed to initialise things related to video
 * buffer queue, video node, and the related media entity. The caller
 * is expected to assign isys field and set the name of the video
 * device.
 */
int ipu6_isys_video_init(struct ipu6_isys_video *av)
{
	struct v4l2_format format = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.fmt.pix = {
			.width = 1920,
			.height = 1080,
		},
	};
	struct v4l2_format format_meta = {
		.type = V4L2_BUF_TYPE_META_CAPTURE,
		.fmt.meta = {
			.width = 1920,
			.height = 4,
		},
	};
	int ret;

	mutex_init(&av->mutex);
	av->vdev.device_caps = V4L2_CAP_STREAMING | V4L2_CAP_IO_MC |
			       V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_META_CAPTURE;
	av->vdev.vfl_dir = VFL_DIR_RX;

	ret = ipu6_isys_queue_init(&av->aq);
	if (ret)
		goto out_free_watermark;

	av->pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&av->vdev.entity, 1, &av->pad);
	if (ret)
		goto out_vb2_queue_release;

	av->vdev.entity.ops = &entity_ops;
	av->vdev.release = video_device_release_empty;
	av->vdev.fops = &isys_fops;
	av->vdev.v4l2_dev = &av->isys->v4l2_dev;
	if (!av->vdev.ioctl_ops)
		av->vdev.ioctl_ops = &ipu6_v4l2_ioctl_ops;
	av->vdev.queue = &av->aq.vbq;
	av->vdev.lock = &av->mutex;

	__ipu6_isys_vidioc_try_fmt_vid_cap(av, &format);
	av->pix_fmt = format.fmt.pix;
	__ipu6_isys_vidioc_try_fmt_meta_cap(av, &format_meta);
	av->meta_fmt = format_meta.fmt.meta;

	set_bit(V4L2_FL_USES_V4L2_FH, &av->vdev.flags);
	video_set_drvdata(&av->vdev, av);

	ret = video_register_device(&av->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto out_media_entity_cleanup;

	return ret;

out_media_entity_cleanup:
	vb2_video_unregister_device(&av->vdev);
	media_entity_cleanup(&av->vdev.entity);

out_vb2_queue_release:
	vb2_queue_release(&av->aq.vbq);

out_free_watermark:
	mutex_destroy(&av->mutex);

	return ret;
}

void ipu6_isys_video_cleanup(struct ipu6_isys_video *av)
{
	vb2_video_unregister_device(&av->vdev);
	media_entity_cleanup(&av->vdev.entity);
	mutex_destroy(&av->mutex);
}

u32 ipu6_isys_get_format(struct ipu6_isys_video *av)
{
	if (av->aq.vbq.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return av->pix_fmt.pixelformat;

	if (av->aq.vbq.type == V4L2_BUF_TYPE_META_CAPTURE)
		return av->meta_fmt.dataformat;

	return 0;
}

u32 ipu6_isys_get_data_size(struct ipu6_isys_video *av)
{
	if (av->aq.vbq.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return av->pix_fmt.sizeimage;

	if (av->aq.vbq.type == V4L2_BUF_TYPE_META_CAPTURE)
		return av->meta_fmt.buffersize;

	return 0;
}

u32 ipu6_isys_get_bytes_per_line(struct ipu6_isys_video *av)
{
	if (av->aq.vbq.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return av->pix_fmt.bytesperline;

	if (av->aq.vbq.type == V4L2_BUF_TYPE_META_CAPTURE)
		return av->meta_fmt.bytesperline;

	return 0;
}

u32 ipu6_isys_get_frame_width(struct ipu6_isys_video *av)
{
	if (av->aq.vbq.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return av->pix_fmt.width;

	if (av->aq.vbq.type == V4L2_BUF_TYPE_META_CAPTURE)
		return av->meta_fmt.width;

	return 0;
}

u32 ipu6_isys_get_frame_height(struct ipu6_isys_video *av)
{
	if (av->aq.vbq.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return av->pix_fmt.height;

	if (av->aq.vbq.type == V4L2_BUF_TYPE_META_CAPTURE)
		return av->meta_fmt.height;

	return 0;
}

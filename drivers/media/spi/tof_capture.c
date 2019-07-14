/*
 * GS1662 device registration.
 *
 * Copyright (C) 2015-2016 Nexvision
 * Author: Charles-Antoine Couret <charles-antoine.couret@nexvision.fr>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>

#define dprintk(t, level, fmt, arg...) \
	v4l2_dbg(level, 1, &t->v4l2_dev, fmt, ## arg)

struct tof {
	struct spi_device *spi;
	struct v4l2_device		v4l2_dev;
	struct video_device		tof_cap_dev;
	u32 caps;
	struct vb2_queue                queue;
	int enabled;
	struct mutex			mutex;

	u32	fourcc;          /* v4l2 format id */
	enum v4l2_field			field;
	struct v4l2_rect		src_rect;

	unsigned long tof_cap_seq_count;

	struct list_head		active;
	spinlock_t			slock;

	struct spi_transfer	xfer;
};

/* buffer for one video frame */
struct tof_buffer {
	/* common v4l buffer stuff -- must be first */
	struct vb2_v4l2_buffer vb;
	struct list_head	list;
};

static unsigned effective_width_bytes(struct tof *t)
{
	/* around 80 * 4 * 3 + blanking */
	return 1024;
}

static irqreturn_t tof_irq(int irq, void *devid)
{
	struct tof *t = devid;
	int ret;
	struct spi_message msg;
	struct spi_transfer *xfer = &t->xfer;
	struct tof_buffer *buf = NULL;
	struct timespec tp1,tp2;
	static struct timespec prev;
	void *vbuf;


	spin_lock(&t->slock);

	if (!list_empty(&t->active)) {
		buf = list_entry(t->active.next, struct tof_buffer, list);
		list_del(&buf->list);
	}
	spin_unlock(&t->slock);

	if (!buf)
		return IRQ_HANDLED;

	buf->vb.sequence = t->tof_cap_seq_count;
	buf->vb.field = t->field;
	buf->vb.vb2_buf.timestamp = ktime_get_ns();

	vbuf = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);

	ktime_get_real_ts(&tp1);
	prev = tp1;

	//printk(KERN_ERR "VD interrupt %lld\n", timespec_to_ns(&tp1));
	//printk(KERN_ERR "vbuf %x\n", vbuf);

	spi_message_init(&msg);

	xfer->bits_per_word = 32;
	xfer->delay_usecs = 0;
	xfer->speed_hz = 18000000;
	xfer->len = effective_width_bytes(t) * t->src_rect.height;
	xfer->rx_buf = vbuf;

        spi_message_add_tail(xfer, &msg);

        ret = spi_sync(t->spi, &msg);

	ktime_get_real_ts(&tp2);

	//printk(KERN_ERR "got RET %d, delta: %lld\n", ret, timespec_to_ns(&tp2) -timespec_to_ns(&tp1));

	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	dprintk(t, 2, "buffer %d done\n", buf->vb.vb2_buf.index);

	return ret < 0 ? IRQ_NONE : IRQ_HANDLED;
}

static void tof_v4l2dev_release(struct v4l2_device *v4l2_dev)
{
	struct tof *t = container_of(v4l2_dev, struct tof, v4l2_dev);

	v4l2_device_unregister(&t->v4l2_dev);
}

static int tof_cap_queue_setup(struct vb2_queue *vq,
		       unsigned *nbuffers, unsigned *nplanes,
		       unsigned sizes[], struct device *alloc_devs[])
{
	struct tof *t = vb2_get_drv_priv(vq);
	unsigned buffers = 1;
	unsigned p;

	if (*nplanes) {
		/*
		 * Check if the number of requested planes match
		 * the number of buffers in the current format. You can't mix that.
		 */
		if (*nplanes != buffers)
			return -EINVAL;
		for (p = 0; p < buffers; p++) {
			if (sizes[p] < effective_width_bytes(t) * t->src_rect.height)
				return -EINVAL;
		}
	} else {
		for (p = 0; p < buffers; p++)
			sizes[p] = effective_width_bytes(t) * t->src_rect.height;
	}

	if (vq->num_buffers + *nbuffers < 2)
		*nbuffers = 2 - vq->num_buffers;

	*nplanes = buffers;

	dprintk(t, 1, "%s: count=%d\n", __func__, *nbuffers);
	for (p = 0; p < buffers; p++)
		dprintk(t, 1, "%s: size[%u]=%u\n", __func__, p, sizes[p]);

	return 0;
}

static int tof_cap_buf_prepare(struct vb2_buffer *vb)
{
	struct tof *t = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size;
	unsigned buffers = 1;
	unsigned p;

	for (p = 0; p < buffers; p++) {
		size = effective_width_bytes(t) * t->src_rect.height;

		if (vb2_plane_size(vb, p) < size) {
			dprintk(t, 1, "%s data will not fit into plane %u (%lu < %lu)\n",
					__func__, p, vb2_plane_size(vb, p), size);
			return -EINVAL;
		}

		vb2_set_plane_payload(vb, p, size);
		vb->planes[p].data_offset = 0;
	}

	return 0;
}

static void tof_cap_buf_finish(struct vb2_buffer *vb)
{
}

static void tof_cap_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct tof *t = vb2_get_drv_priv(vb->vb2_queue);
	struct tof_buffer *buf = container_of(vbuf, struct tof_buffer, vb);

	spin_lock(&t->slock);
	list_add_tail(&buf->list, &t->active);
	spin_unlock(&t->slock);
}

static int tof_cap_start_streaming(struct vb2_queue *vq, unsigned count)
{
	struct tof *t = vb2_get_drv_priv(vq);

	t->tof_cap_seq_count = 0;
	dprintk(t, 1, "%s\n", __func__);

	enable_irq(t->spi->irq);

	return 0;
}

/* abort streaming and wait for last buffer */
static void tof_cap_stop_streaming(struct vb2_queue *vq)
{
	struct tof *t = vb2_get_drv_priv(vq);

	dprintk(t, 1, "%s\n", __func__);
	disable_irq(t->spi->irq);

	//we need to dequeue the undone buffers here
	while (!list_empty(&t->active)) {
		struct tof_buffer *buf;

		buf = list_entry(t->active.next, struct tof_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		dprintk(t, 2, "vid_cap buffer %d done\n",
			buf->vb.vb2_buf.index);
	}
}

const struct vb2_ops tof_cap_qops = {
	.queue_setup		= tof_cap_queue_setup,
	.buf_prepare		= tof_cap_buf_prepare,
	.buf_finish		= tof_cap_buf_finish,
	.buf_queue		= tof_cap_buf_queue,
	.start_streaming	= tof_cap_start_streaming,
	.stop_streaming		= tof_cap_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int tof_fop_release(struct file *file)
{
	struct tof *t = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);

	if (vdev->queue)
		return vb2_fop_release(file);

	return v4l2_fh_release(file);
}

static const struct v4l2_file_operations tof_fops = {
	.owner		= THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = tof_fop_release,
	.read           = vb2_fop_read,
	.write          = vb2_fop_write,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = vb2_fop_mmap,
};

static int tof_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{
	struct tof *t = video_drvdata(file);

	strcpy(cap->driver, "tof");
	strcpy(cap->card, "tof");
	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s", t->v4l2_dev.name);

	cap->capabilities = t->caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int tof_enum_fmt_vid(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{
	struct tof *t = video_drvdata(file);

	f->pixelformat = V4L2_PIX_FMT_ARGB32;

	return 0;
}

int tof_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct tof *t = video_drvdata(file);
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	unsigned p;

	mp->width        = 80;
	mp->height       = 60;
	mp->field        = V4L2_FIELD_NONE;
	mp->pixelformat  = V4L2_PIX_FMT_ARGB32;
	mp->colorspace   = V4L2_COLORSPACE_SRGB;
	mp->xfer_func    = V4L2_XFER_FUNC_DEFAULT;
	mp->ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	mp->quantization = V4L2_QUANTIZATION_DEFAULT;
	mp->num_planes = 1;
	for (p = 0; p < mp->num_planes; p++) {
		mp->plane_fmt[p].bytesperline = effective_width_bytes(t);
		mp->plane_fmt[p].sizeimage = effective_width_bytes(t) * mp->height;
	}

	return 0;
}

int tof_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *pfmt = mp->plane_fmt;
	struct tof *t = video_drvdata(file);
	unsigned bytesperline, max_bpl;
	unsigned factor = 1;
	unsigned w, h;
	unsigned p;

	mp->pixelformat = V4L2_PIX_FMT_ARGB32;
	mp->field        = V4L2_FIELD_NONE;
	mp->width        = 80;
	mp->height       = 60;
	mp->num_planes = 1;
	for (p = 0; p < mp->num_planes; p++) {
		bytesperline = effective_width_bytes(t);

		pfmt[p].bytesperline = bytesperline;
		pfmt[p].sizeimage = bytesperline * mp->height;
		memset(pfmt[p].reserved, 0, sizeof(pfmt[p].reserved));
	}
	mp->colorspace   = V4L2_COLORSPACE_SRGB;
	mp->xfer_func    = V4L2_XFER_FUNC_DEFAULT;
	mp->ycbcr_enc    = V4L2_YCBCR_ENC_DEFAULT;
	mp->quantization = V4L2_QUANTIZATION_DEFAULT;
	memset(mp->reserved, 0, sizeof(mp->reserved));

	return 0;
}

int tof_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	struct tof *t = video_drvdata(file);
	struct vb2_queue *q = &t->queue;
	int ret = tof_try_fmt_vid_cap(file, priv, f);

	if (ret < 0)
		return ret;

	if (vb2_is_busy(q)) {
		dprintk(t, 1, "%s device busy\n", __func__);
		return -EBUSY;
	}

	t->fourcc = mp->pixelformat;
	t->src_rect.left = 0;
	t->src_rect.top = 0;
	t->src_rect.width = mp->width;
	t->src_rect.height = mp->height;
	t->field = mp->field;

	return 0;
}

static const struct v4l2_ioctl_ops tof_ioctl_ops = {
	.vidioc_querycap		= tof_querycap,

	.vidioc_enum_fmt_vid_cap	= tof_enum_fmt_vid,
	.vidioc_enum_fmt_vid_cap_mplane = tof_enum_fmt_vid,

	.vidioc_g_fmt_vid_cap_mplane	= tof_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap_mplane	= tof_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap_mplane	= tof_s_fmt_vid_cap,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
};

static int tof_probe(struct spi_device *spi)
{
	int ret;
	struct tof *t;
	struct video_device *vfd;
	struct vb2_queue *q;

	t = devm_kzalloc(&spi->dev, sizeof(struct tof), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	/* register v4l2_device */
	snprintf(t->v4l2_dev.name, sizeof(t->v4l2_dev.name),
			"%s-%03d", "TOF", 0);

	ret = v4l2_device_register(&spi->dev, &t->v4l2_dev);
	if (ret) {
		return ret;
	}

	t->v4l2_dev.release = tof_v4l2dev_release;

	t->caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
	t->src_rect.left = 0;
	t->src_rect.top = 0;
	t->src_rect.width = 80;
	t->src_rect.height = 60;

	q = &t->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
	q->drv_priv = t;
	q->buf_struct_size = sizeof(struct tof_buffer);
	q->ops = &tof_cap_qops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed = 2;
	q->lock = &t->mutex;

	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	vfd = &t->tof_cap_dev;
	snprintf(vfd->name, sizeof(vfd->name),
			 "TOF-%03d-vid-cap", 0);
	vfd->fops = &tof_fops;
	vfd->ioctl_ops = &tof_ioctl_ops;
	vfd->device_caps = t->caps;
	vfd->release = video_device_release_empty;
	vfd->v4l2_dev = &t->v4l2_dev;
	vfd->queue = q;

	vfd->lock = &t->mutex;
	video_set_drvdata(vfd, t);

	/* initialize locks */
	spin_lock_init(&t->slock);
	mutex_init(&t->mutex);

	/* init dma queues */
	INIT_LIST_HEAD(&t->active);

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		return ret;

	v4l2_info(&t->v4l2_dev, "V4L2 capture device registered as %s\n",
		video_device_node_name(vfd));

	ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
					tof_irq,
					IRQF_ONESHOT, dev_name(&spi->dev),
					t);
	if (ret)
		return ret;

	disable_irq(spi->irq);

	/* SPI init */
	t->spi = spi;

	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = 50000000;
	spi->bits_per_word = 32;
	ret = spi_setup(spi);

	printk(KERN_ERR "ToF SPI driver probed %d\n", ret);

	return ret;
}

static int tof_remove(struct spi_device *spi)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tof_of_match[] = {
	{ .compatible = "ti,tof" },
	{ }
};
MODULE_DEVICE_TABLE(of, tof_of_match);
#endif

static const struct spi_device_id tof_id[] = {
	{ "tof", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, tof_id);

static struct spi_driver tof_driver = {
	.driver = {
		.name		= "tof",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(tof_of_match),
	},

	.probe		= tof_probe,
	.remove		= tof_remove,
	.id_table	= tof_id,
};

module_spi_driver(tof_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Charles-Antoine Couret <charles-antoine.couret@nexvision.fr>");
MODULE_DESCRIPTION("Gennum GS1662 HD/SD-SDI Serializer driver");

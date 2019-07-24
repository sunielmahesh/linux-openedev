/*
 * Driver for the OV5645 camera sensor.
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 By Tech Design S.L. All Rights Reserved.
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-of.h>
#include <media/v4l2-subdev.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

static DEFINE_MUTEX(ov9732_lock);

/* General status */
#define REG_MODE_SELECT			0x0100
#define REG_MODE_SELECT_SLEEP		0x0
#define REG_MODE_SELECT_STREAMING		0x1

#define REG_SOFT_RESET			0x0103

/* Chip ID */
#define REG_CHIP_ID_HIGH		0x300a
#define	REG_CHIP_ID_HIGH_BYTE	0x97
#define REG_CHIP_ID_LOW		0x300b
#define	REG_CHIP_ID_LOW_BYTE		0x32

/* I/O control */
#define REG_IO_CTRL			0x3001
#define REG_MIPI_IO_CTRL		0x3002
#define REG_DRV_CTRL			0x3009

/* I2C */
#define REG_SCCB			0x3031

/* PLL */
#define REG_PRE_PLL_CLK_DIV		0x3080
#define REG_PLL_MULTIPLIER		0x3081
#define REG_VT_SYS_CLK_DIV		0x3082
#define REG_VT_PIXEL_CLK_DIV		0x3083
#define REG_VT_MIPI_DIV		0x3089
#define REG_VT_PHY_DIV		0x308a
#define REG_PIX_DIV			0x301e
#define REG_SYS_CLK_DIV		0x3103

#define OV5645_AWB_MANUAL_CONTROL	0x3406
#define	OV5645_AWB_MANUAL_ENABLE	BIT(0)

#define REG_AEC_PK_MANUAL		0x3503
#define	REG_AEC_MANUAL_ENABLE	BIT(0)
#define	REG_AGC_MANUAL_ENABLE	BIT(1)

#define REG_FORMAT0			0x3820
#define	REG_VBIN			BIT(0)
#define	REG_HBIN			BIT(1)
#define	REG_VFLIP			BIT(2)
#define	REG_HFLIP			BIT(3)

#define REG_FORMAT1			0x3821

#define REG_FMT_CTL			0x4308

#define REG_PRE_ISP_CTRL0		0x5080
#define	REG_TEST_PATTERN_MASK	0x3
#define	REG_SET_TEST_PATTERN(x)	(((x) & REG_TEST_PATTERN_MASK) << 2)
#define	REG_TEST_PATTERN_ENABLE	BIT(7)

#define REG_WINC_CTRL04			0x5704
#define REG_WINC_CTRL05			0x5705
#define REG_WINC_CTRL06			0x5706
#define REG_WINC_CTRL07			0x5707
#define REG_WINC_CTRL08			0x5708

#define OV5645_SDE_SAT_U		0x5583
#define OV5645_SDE_SAT_V		0x5584

struct reg_value {
	u16 reg;
	u8 val;
	u8 mask;
};

struct ov9732_mode_info {
	u32 width;
	u32 height;
	const struct reg_value *data;
	u32 data_size;
	u32 pixel_clock;
	u32 link_freq;
	u32 code;
};

struct ov9732 {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_of_endpoint ep;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;
	struct clk *xclk;

	struct regulator *dvdd;
	struct regulator *avdd;

	const struct ov9732_mode_info *current_mode;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_clock;
	struct v4l2_ctrl *link_freq;

	/* Cached register values */
	u8 aec_pk_manual;
	u8 format0;
	u8 format1;

	u8 red_msb, red_lsb;
	u8 blue_msb, blue_lsb;
	u8 green_msb, green_lsb;

	struct mutex power_lock; /* lock to protect power state */
	int power_count;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *rst_gpio;
	struct gpio_desc *tof_vdin_gpio;
	struct gpio_desc *fsin_gpio;

	u32 reg;

	struct hrtimer timer;

	struct delayed_work exposure_write_work;
	unsigned long write_delay;
	u16 exposure;
	bool exposure_changed;
};

static inline struct ov9732 *to_ov9732(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov9732, sd);
}

/* TODO : Fill this up with the common register values */
static const struct reg_value ov9732_global_init_setting[] = {
};

static const struct reg_value ov9732_setting_720p_raw10[] = {
	{ 0x0103, 0x01, 0xff },
	{ 0x0100, 0x00, 0xff },
	{ 0x3001, 0x00, 0xff },
	{ 0x3002, 0x00, 0xff },
	{ 0x3007, 0x1f, 0xff },
	{ 0x3008, 0xff, 0xff },
	{ 0x3009, 0x02, 0xff },
	{ 0x3010, 0x00, 0xff },
	{ 0x3011, 0x08, 0xff },
	{ 0x3014, 0x22, 0xff },
	{ 0x301e, 0x15, 0xff },
	{ 0x3030, 0x19, 0xff },
	{ 0x3080, 0x02, 0xff },
	{ 0x3081, 0x3c, 0xff },
	{ 0x3082, 0x04, 0xff },
	{ 0x3083, 0x00, 0xff },
	{ 0x3084, 0x02, 0xff },
	{ 0x3085, 0x01, 0xff },
	{ 0x3086, 0x01, 0xff },
	{ 0x3089, 0x01, 0xff },
	{ 0x308a, 0x00, 0xff },
	{ 0x3103, 0x01, 0xff },
	{ 0x3600, 0xf6, 0xff },
	{ 0x3601, 0x72, 0xff },
	{ 0x3610, 0x0c, 0xff },
	{ 0x3611, 0xf0, 0xff },
	{ 0x3612, 0x35, 0xff },
	{ 0x3654, 0x10, 0xff },
	{ 0x3655, 0x77, 0xff },
	{ 0x3656, 0x77, 0xff },
	{ 0x3657, 0x07, 0xff },
	{ 0x3658, 0x22, 0xff },
	{ 0x3659, 0x22, 0xff },
	{ 0x365a, 0x02, 0xff },
	{ 0x3700, 0x1f, 0xff },
	{ 0x3701, 0x10, 0xff },
	{ 0x3702, 0x0c, 0xff },
	{ 0x3703, 0x07, 0xff },
	{ 0x3704, 0x3c, 0xff },
	{ 0x3705, 0x81, 0xff },
	{ 0x370d, 0x20, 0xff },
	{ 0x3710, 0x0c, 0xff },
	{ 0x3782, 0x58, 0xff },
	{ 0x3783, 0x60, 0xff },
	{ 0x3784, 0x05, 0xff },
	{ 0x3785, 0x55, 0xff },
	{ 0x37c0, 0x07, 0xff },
	{ 0x3800, 0x00, 0xff },
	{ 0x3801, 0x04, 0xff },
	{ 0x3802, 0x00, 0xff },
	{ 0x3803, 0x04, 0xff },
	{ 0x3804, 0x05, 0xff },
	{ 0x3805, 0x0b, 0xff },
	{ 0x3806, 0x02, 0xff },
	{ 0x3807, 0xdb, 0xff },
	{ 0x3808, 0x05, 0xff },
	{ 0x3809, 0x00, 0xff },
	{ 0x380a, 0x02, 0xff },
	{ 0x380b, 0xd0, 0xff },
	{ 0x380c, 0x05, 0xff },
	{ 0x380d, 0xc6, 0xff },
	{ 0x380e, 0x03, 0xff },
	{ 0x380f, 0x22, 0xff },
	{ 0x3810, 0x00, 0xff },
	{ 0x3811, 0x04, 0xff },
	{ 0x3812, 0x00, 0xff },
	{ 0x3813, 0x04, 0xff },
	{ 0x3816, 0x00, 0xff },
	{ 0x3817, 0x00, 0xff },
	{ 0x3818, 0x00, 0xff },
	{ 0x3819, 0x04, 0xff },
	{ 0x3820, 0x10, 0xff },
	{ 0x3821, 0x00, 0xff },
	{ 0x382c, 0x06, 0xff },
	{ 0x3500, 0x00, 0xff },
	{ 0x3501, 0x31, 0xff },
	{ 0x3502, 0x00, 0xff },
	{ 0x3503, 0x03, 0xff },
	{ 0x3504, 0x00, 0xff },
	{ 0x3505, 0x00, 0xff },
	{ 0x3509, 0x10, 0xff },
	{ 0x350a, 0x00, 0xff },
	{ 0x350b, 0x40, 0xff },
	{ 0x3d00, 0x00, 0xff },
	{ 0x3d01, 0x00, 0xff },
	{ 0x3d02, 0x00, 0xff },
	{ 0x3d03, 0x00, 0xff },
	{ 0x3d04, 0x00, 0xff },
	{ 0x3d05, 0x00, 0xff },
	{ 0x3d06, 0x00, 0xff },
	{ 0x3d07, 0x00, 0xff },
	{ 0x3d08, 0x00, 0xff },
	{ 0x3d09, 0x00, 0xff },
	{ 0x3d0a, 0x00, 0xff },
	{ 0x3d0b, 0x00, 0xff },
	{ 0x3d0c, 0x00, 0xff },
	{ 0x3d0d, 0x00, 0xff },
	{ 0x3d0e, 0x00, 0xff },
	{ 0x3d0f, 0x00, 0xff },
	{ 0x3d80, 0x00, 0xff },
	{ 0x3d81, 0x00, 0xff },
	{ 0x3d82, 0x38, 0xff },
	{ 0x3d83, 0xa4, 0xff },
	{ 0x3d84, 0x00, 0xff },
	{ 0x3d85, 0x00, 0xff },
	{ 0x3d86, 0x1f, 0xff },
	{ 0x3d87, 0x03, 0xff },
	{ 0x3d8b, 0x00, 0xff },
	{ 0x3d8f, 0x00, 0xff },
	{ 0x4001, 0xe0, 0xff },
	{ 0x4004, 0x00, 0xff },
	{ 0x4005, 0x02, 0xff },
	{ 0x4006, 0x01, 0xff },
	{ 0x4007, 0x40, 0xff },
	{ 0x4009, 0x0b, 0xff },
	{ 0x4300, 0x03, 0xff },
	{ 0x4301, 0xff, 0xff },
	{ 0x4304, 0x00, 0xff },
	{ 0x4305, 0x00, 0xff },
	{ 0x4309, 0x00, 0xff },
	{ 0x4600, 0x00, 0xff },
	{ 0x4601, 0x04, 0xff },
	{ 0x4800, 0x00, 0xff },
	{ 0x4805, 0x00, 0xff },
	{ 0x4821, 0x50, 0xff },
	{ 0x4823, 0x50, 0xff },
	{ 0x4837, 0x2d, 0xff },
	{ 0x4a00, 0x00, 0xff },
	{ 0x4f00, 0x80, 0xff },
	{ 0x4f01, 0x10, 0xff },
	{ 0x4f02, 0x00, 0xff },
	{ 0x4f03, 0x00, 0xff },
	{ 0x4f04, 0x00, 0xff },
	{ 0x4f05, 0x00, 0xff },
	{ 0x4f06, 0x00, 0xff },
	{ 0x4f07, 0x00, 0xff },
	{ 0x4f08, 0x00, 0xff },
	{ 0x4f09, 0x00, 0xff },
	{ 0x5000, 0x0f, 0xff },
	{ 0x500c, 0x00, 0xff },
	{ 0x500d, 0x00, 0xff },
	{ 0x500e, 0x00, 0xff },
	{ 0x500f, 0x00, 0xff },
	{ 0x5010, 0x00, 0xff },
	{ 0x5011, 0x00, 0xff },
	{ 0x5012, 0x00, 0xff },
	{ 0x5013, 0x00, 0xff },
	{ 0x5014, 0x00, 0xff },
	{ 0x5015, 0x00, 0xff },
	{ 0x5016, 0x00, 0xff },
	{ 0x5017, 0x00, 0xff },
	{ 0x5080, 0x80, 0xff },
	{ 0x5180, 0x01, 0xff },
	{ 0x5181, 0x00, 0xff },
	{ 0x5182, 0x01, 0xff },
	{ 0x5183, 0x00, 0xff },
	{ 0x5184, 0x01, 0xff },
	{ 0x5185, 0x00, 0xff },
	{ 0x5708, 0x06, 0xff },
	{ 0x5781, 0x00, 0xff },
	{ 0x5783, 0x0f, 0xff },
	{ 0x0100, 0x01, 0xff },
	{ 0x3703, 0x0b, 0xff },
	{ 0x3705, 0x51, 0xff },
};

static const struct reg_value ov9732_setting_720p_raw8[] = {
	{ 0x0103, 0x01, 0xff },
	{ 0x0100, 0x00, 0xff },
	{ 0x3001, 0x00, 0xff },
	{ 0x3002, 0x00, 0xff },
	{ 0x3007, 0x1f, 0xff },
	{ 0x3008, 0xff, 0xff },
	{ 0x3009, 0x02, 0xff },
	{ 0x3010, 0x00, 0xff },
	{ 0x3011, 0x08, 0xff },
	{ 0x3014, 0x22, 0xff },
	{ 0x301e, 0x15, 0xff },
	{ 0x3030, 0x19, 0xff },
	{ 0x3080, 0x02, 0xff },
	{ 0x3081, 0x3c, 0xff },
	{ 0x3082, 0x04, 0xff },
	{ 0x3083, 0x00, 0xff },
	{ 0x3084, 0x02, 0xff },
	{ 0x3085, 0x01, 0xff },
	{ 0x3086, 0x01, 0xff },
	{ 0x3089, 0x01, 0xff },
	{ 0x308a, 0x00, 0xff },
	{ 0x3103, 0x01, 0xff },
	{ 0x3600, 0xf6, 0xff },
	{ 0x3601, 0x72, 0xff },
	{ 0x3610, 0x0c, 0xff },
	{ 0x3611, 0xf0, 0xff },
	{ 0x3612, 0x35, 0xff },
	{ 0x3654, 0x30, 0xff },
	{ 0x3655, 0x77, 0xff },
	{ 0x3656, 0x77, 0xff },
	{ 0x3657, 0x07, 0xff },
	{ 0x3658, 0x22, 0xff },
	{ 0x3659, 0x22, 0xff },
	{ 0x365a, 0x02, 0xff },
	{ 0x3700, 0x1f, 0xff },
	{ 0x3701, 0x10, 0xff },
	{ 0x3702, 0x0c, 0xff },
	{ 0x3703, 0x07, 0xff },
	{ 0x3704, 0x3c, 0xff },
	{ 0x3705, 0x81, 0xff },
	{ 0x370d, 0x20, 0xff },
	{ 0x3710, 0x0c, 0xff },
	{ 0x3782, 0x58, 0xff },
	{ 0x3783, 0x60, 0xff },
	{ 0x3784, 0x05, 0xff },
	{ 0x3785, 0x55, 0xff },
	{ 0x37c0, 0x07, 0xff },
	{ 0x3800, 0x00, 0xff },
	{ 0x3801, 0x04, 0xff },
	{ 0x3802, 0x00, 0xff },
	{ 0x3803, 0x04, 0xff },
	{ 0x3804, 0x05, 0xff },
	{ 0x3805, 0x0b, 0xff },
	{ 0x3806, 0x02, 0xff },
	{ 0x3807, 0xdb, 0xff },
	{ 0x3808, 0x05, 0xff },
	{ 0x3809, 0x00, 0xff },
	{ 0x380a, 0x02, 0xff },
	{ 0x380b, 0xd0, 0xff },
	{ 0x380c, 0x05, 0xff },
	{ 0x380d, 0xc6, 0xff },
	{ 0x380e, 0x03, 0xff },
	{ 0x380f, 0x22, 0xff },
	{ 0x3810, 0x00, 0xff },
	{ 0x3811, 0x04, 0xff },
	{ 0x3812, 0x00, 0xff },
	{ 0x3813, 0x04, 0xff },
	{ 0x3816, 0x00, 0xff },
	{ 0x3817, 0x00, 0xff },
	{ 0x3818, 0x00, 0xff },
	{ 0x3819, 0x04, 0xff },
	{ 0x3820, 0x10, 0xff },
	{ 0x3821, 0x00, 0xff },
	{ 0x382c, 0x06, 0xff },
	{ 0x3500, 0x00, 0xff },
	{ 0x3501, 0x31, 0xff },
	{ 0x3502, 0x00, 0xff },
	{ 0x3503, 0x03, 0xff },
	{ 0x3504, 0x00, 0xff },
	{ 0x3505, 0x00, 0xff },
	{ 0x3509, 0x10, 0xff },
	{ 0x350a, 0x00, 0xff },
	{ 0x350b, 0x40, 0xff },
	{ 0x3d00, 0x00, 0xff },
	{ 0x3d01, 0x00, 0xff },
	{ 0x3d02, 0x00, 0xff },
	{ 0x3d03, 0x00, 0xff },
	{ 0x3d04, 0x00, 0xff },
	{ 0x3d05, 0x00, 0xff },
	{ 0x3d06, 0x00, 0xff },
	{ 0x3d07, 0x00, 0xff },
	{ 0x3d08, 0x00, 0xff },
	{ 0x3d09, 0x00, 0xff },
	{ 0x3d0a, 0x00, 0xff },
	{ 0x3d0b, 0x00, 0xff },
	{ 0x3d0c, 0x00, 0xff },
	{ 0x3d0d, 0x00, 0xff },
	{ 0x3d0e, 0x00, 0xff },
	{ 0x3d0f, 0x00, 0xff },
	{ 0x3d80, 0x00, 0xff },
	{ 0x3d81, 0x00, 0xff },
	{ 0x3d82, 0x38, 0xff },
	{ 0x3d83, 0xa4, 0xff },
	{ 0x3d84, 0x00, 0xff },
	{ 0x3d85, 0x00, 0xff },
	{ 0x3d86, 0x1f, 0xff },
	{ 0x3d87, 0x03, 0xff },
	{ 0x3d8b, 0x00, 0xff },
	{ 0x3d8f, 0x00, 0xff },
	{ 0x4001, 0xe0, 0xff },
	{ 0x4004, 0x00, 0xff },
	{ 0x4005, 0x02, 0xff },
	{ 0x4006, 0x01, 0xff },
	{ 0x4007, 0x40, 0xff },
	{ 0x4009, 0x0b, 0xff },
	{ 0x4300, 0x03, 0xff },
	{ 0x4301, 0xff, 0xff },
	{ 0x4304, 0x00, 0xff },
	{ 0x4305, 0x00, 0xff },
	//{ 0x4308, 0x00, 0xff },
	{ 0x4309, 0x00, 0xff },
	{ 0x4600, 0x00, 0xff },
	{ 0x4601, 0xf0, 0xff },
	{ 0x4800, 0x00, 0xff },
	{ 0x4805, 0x00, 0xff },
	{ 0x4821, 0x50, 0xff },
	{ 0x4823, 0x50, 0xff },
	{ 0x4837, 0x2d, 0xff },
	{ 0x4a00, 0x00, 0xff },
	{ 0x4f00, 0x80, 0xff },
	{ 0x4f01, 0x10, 0xff },
	{ 0x4f02, 0x00, 0xff },
	{ 0x4f03, 0x00, 0xff },
	{ 0x4f04, 0x00, 0xff },
	{ 0x4f05, 0x00, 0xff },
	{ 0x4f06, 0x00, 0xff },
	{ 0x4f07, 0x00, 0xff },
	{ 0x4f08, 0x00, 0xff },
	{ 0x4f09, 0x00, 0xff },
	{ 0x5000, 0x0f, 0xff },
	{ 0x500c, 0x00, 0xff },
	{ 0x500d, 0x00, 0xff },
	{ 0x500e, 0x00, 0xff },
	{ 0x500f, 0x00, 0xff },
	{ 0x5010, 0x00, 0xff },
	{ 0x5011, 0x00, 0xff },
	{ 0x5012, 0x00, 0xff },
	{ 0x5013, 0x00, 0xff },
	{ 0x5014, 0x00, 0xff },
	{ 0x5015, 0x00, 0xff },
	{ 0x5016, 0x00, 0xff },
	{ 0x5017, 0x00, 0xff },
	//{ 0x5080, 0x80, 0xff },
	{ 0x5180, 0x01, 0xff },
	{ 0x5181, 0x00, 0xff },
	{ 0x5182, 0x01, 0xff },
	{ 0x5183, 0x00, 0xff },
	{ 0x5184, 0x01, 0xff },
	{ 0x5185, 0x00, 0xff },
	{ 0x5708, 0x06, 0xff },
	{ 0x5781, 0x00, 0xff },
	{ 0x5783, 0x0f, 0xff },
	{ 0x0100, 0x01, 0xff },
	{ 0x3703, 0x0b, 0xff },
	{ 0x3705, 0x51, 0xff },
};

static const struct reg_value ov9732_setting_640x360_raw8[] = {
	{ 0x0103, 0x01, 0xff },
	{ 0x0100, 0x00, 0xff },
	{ 0x3001, 0x01, 0xff },	/* adjusted to set FSIN to input mode */
	{ 0x3002, 0x00, 0xff },
	{ 0x3007, 0x00, 0xff },	/* this seems to be needed for FSIN input to work too */
	{ 0x3008, 0xff, 0xff },
	{ 0x3009, 0x02, 0xff },
	{ 0x3010, 0x00, 0xff },
	{ 0x3011, 0x08, 0xff },
	{ 0x3014, 0x22, 0xff },
	{ 0x301e, 0x15, 0xff },
	{ 0x3030, 0x19, 0xff },
	{ 0x3080, 0x02, 0xff },
	{ 0x3081, 0x3c, 0xff },
	{ 0x3082, 0x04, 0xff },
	{ 0x3083, 0x00, 0xff },
	{ 0x3084, 0x02, 0xff },
	{ 0x3085, 0x01, 0xff },
	{ 0x3086, 0x01, 0xff },
	{ 0x3089, 0x01, 0xff },
	{ 0x308a, 0x00, 0xff },
	{ 0x3103, 0x01, 0xff },
	{ 0x3600, 0xf6, 0xff },
	{ 0x3601, 0x72, 0xff },
	{ 0x3605, 0x66, 0xff },
	{ 0x3610, 0x0c, 0xff },
	{ 0x3611, 0x60, 0xff },
	{ 0x3612, 0x35, 0xff },
	{ 0x3654, 0x30, 0xff },
	{ 0x3655, 0x77, 0xff },
	{ 0x3656, 0x77, 0xff },
	{ 0x3657, 0x07, 0xff },
	{ 0x3658, 0x22, 0xff },
	{ 0x3659, 0x22, 0xff },
	{ 0x365a, 0x02, 0xff },
	{ 0x3700, 0x1f, 0xff },
	{ 0x3701, 0x10, 0xff },
	{ 0x3702, 0x0c, 0xff },
	{ 0x3703, 0x0b, 0xff },
	{ 0x3704, 0x3c, 0xff },
	{ 0x3705, 0x51, 0xff },
	{ 0x370d, 0x20, 0xff },
	{ 0x3710, 0x0d, 0xff },
	{ 0x3782, 0x58, 0xff },
	{ 0x3783, 0x60, 0xff },
	{ 0x3784, 0x05, 0xff },
	{ 0x3785, 0x55, 0xff },
	{ 0x37c0, 0x03, 0xff },
	{ 0x3800, 0x00, 0xff },
	{ 0x3801, 0x00, 0xff },
	{ 0x3802, 0x00, 0xff },
	{ 0x3803, 0x00, 0xff },
	{ 0x3804, 0x05, 0xff },
	{ 0x3805, 0x0f, 0xff },
	{ 0x3806, 0x02, 0xff },
	{ 0x3807, 0xdf, 0xff },
	{ 0x3808, 0x02, 0xff },
	{ 0x3809, 0x80, 0xff },
	{ 0x380a, 0x01, 0xff },
	{ 0x380b, 0x68, 0xff },
	{ 0x380c, 0x07, 0xff },
	{ 0x380d, 0xd0, 0xff },
	{ 0x380e, 0x0e, 0xff },
	{ 0x380f, 0x10, 0xff },
	{ 0x3810, 0x00, 0xff },
	{ 0x3811, 0x04, 0xff },
	{ 0x3812, 0x00, 0xff },
	{ 0x3813, 0x04, 0xff },
	{ 0x3816, 0x00, 0xff },
	{ 0x3817, 0x00, 0xff },
	{ 0x3818, 0x00, 0xff },
	{ 0x3819, 0x04, 0xff },
	{ 0x381c, 0x05, 0xff },	/* this seems to be needed to enable FSIN */
	{ 0x381d, 0x20, 0xff },	/* sync more often */
	{ 0x3820, 0x13, 0xff },
	{ 0x3821, 0x09, 0xff },
	{ 0x382c, 0x06, 0xff },
	{ 0x3500, 0x00, 0xff },
	{ 0x3501, 0x18, 0xff },
	{ 0x3502, 0x00, 0xff },
	{ 0x3503, 0x03, 0xff },
	{ 0x3504, 0x00, 0xff },
	{ 0x3505, 0x00, 0xff },
	{ 0x3509, 0x10, 0xff },
	{ 0x350a, 0x00, 0xff },
	{ 0x350b, 0x40, 0xff },
	{ 0x3d00, 0x00, 0xff },
	{ 0x3d01, 0x00, 0xff },
	{ 0x3d02, 0x00, 0xff },
	{ 0x3d03, 0x00, 0xff },
	{ 0x3d04, 0x00, 0xff },
	{ 0x3d05, 0x00, 0xff },
	{ 0x3d06, 0x00, 0xff },
	{ 0x3d07, 0x00, 0xff },
	{ 0x3d08, 0x00, 0xff },
	{ 0x3d09, 0x00, 0xff },
	{ 0x3d0a, 0x00, 0xff },
	{ 0x3d0b, 0x00, 0xff },
	{ 0x3d0c, 0x00, 0xff },
	{ 0x3d0d, 0x00, 0xff },
	{ 0x3d0e, 0x00, 0xff },
	{ 0x3d0f, 0x00, 0xff },
	{ 0x3d80, 0x00, 0xff },
	{ 0x3d81, 0x00, 0xff },
	{ 0x3d82, 0x38, 0xff },
	{ 0x3d83, 0xa4, 0xff },
	{ 0x3d84, 0x00, 0xff },
	{ 0x3d85, 0x00, 0xff },
	{ 0x3d86, 0x1f, 0xff },
	{ 0x3d87, 0x03, 0xff },
	{ 0x3d8b, 0x00, 0xff },
	{ 0x3d8f, 0x00, 0xff },
	{ 0x4001, 0xe0, 0xff },
	{ 0x4004, 0x00, 0xff },
	{ 0x4005, 0x02, 0xff },
	{ 0x4006, 0x01, 0xff },
	{ 0x4007, 0x40, 0xff },
	{ 0x4009, 0x05, 0xff },
	{ 0x4300, 0x03, 0xff },
	{ 0x4301, 0xff, 0xff },
	{ 0x4304, 0x00, 0xff },
	{ 0x4305, 0x00, 0xff },
	{ 0x4309, 0x00, 0xff },
	{ 0x4600, 0x00, 0xff },
	{ 0x4601, 0x20, 0xff },
	{ 0x4800, 0x00, 0xff },
	{ 0x4805, 0x00, 0xff },
	{ 0x4821, 0x50, 0xff },
	{ 0x4823, 0x50, 0xff },
	{ 0x4837, 0x2d, 0xff },
	{ 0x4a00, 0x00, 0xff },
	{ 0x4f00, 0x80, 0xff },
	{ 0x4f01, 0x10, 0xff },
	{ 0x4f02, 0x00, 0xff },
	{ 0x4f03, 0x00, 0xff },
	{ 0x4f04, 0x00, 0xff },
	{ 0x4f05, 0x00, 0xff },
	{ 0x4f06, 0x00, 0xff },
	{ 0x4f07, 0x00, 0xff },
	{ 0x4f08, 0x00, 0xff },
	{ 0x4f09, 0x00, 0xff },
	{ 0x5000, 0x17, 0xff },
	{ 0x500c, 0x00, 0xff },
	{ 0x500d, 0x00, 0xff },
	{ 0x500e, 0x00, 0xff },
	{ 0x500f, 0x00, 0xff },
	{ 0x5010, 0x00, 0xff },
	{ 0x5011, 0x00, 0xff },
	{ 0x5012, 0x00, 0xff },
	{ 0x5013, 0x00, 0xff },
	{ 0x5014, 0x00, 0xff },
	{ 0x5015, 0x00, 0xff },
	{ 0x5016, 0x00, 0xff },
	{ 0x5017, 0x00, 0xff },
	{ 0x5080, 0x00, 0xff },
	{ 0x5180, 0x01, 0xff },
	{ 0x5181, 0x80, 0xff },
	{ 0x5182, 0x01, 0xff },
	{ 0x5183, 0x00, 0xff },
	{ 0x5184, 0x01, 0xff },
	{ 0x5185, 0x80, 0xff },
	{ 0x5708, 0x06, 0xff },
	{ 0x5781, 0x0e, 0xff },
	{ 0x5783, 0x0f, 0xff },
	{ 0x3603, 0x70, 0xff },
	{ 0x3620, 0x1e, 0xff },
	{ 0x400a, 0x01, 0xff },
	{ 0x400b, 0xc0, 0xff },
	{ 0x0100, 0x01, 0xff },
};

static const u32 ov9732_formats[] = {
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SBGGR10_1X10
};

/* TODO: Fix this */
static const s64 link_freq[] = {
	360000000,
};

static const struct ov9732_mode_info ov9732_mode_info_data[] = {
	{
		.width = 1280,
		.height = 720,
		.data = ov9732_setting_720p_raw8,
		.data_size = ARRAY_SIZE(ov9732_setting_720p_raw8),
		.pixel_clock = 36000000,
		.link_freq = 0, /* an index in link_freq[] */
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
	},
	{
		.width = 1280,
		.height = 720,
		.data = ov9732_setting_720p_raw10,
		.data_size = ARRAY_SIZE(ov9732_setting_720p_raw10),
		.pixel_clock = 36000000,
		.link_freq = 0,
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
	},
	{
		.width = 640,
		.height = 360,
		.data = ov9732_setting_640x360_raw8,
		.data_size = ARRAY_SIZE(ov9732_setting_640x360_raw8),
		.pixel_clock = 36000000,
		.link_freq = 0,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
	},
};

#if 0
static int ov9732_regulators_enable(struct ov9732 *ov9732)
{
	int ret;

	ret = regulator_enable(ov9732->avdd);
	if (ret) {
		dev_err(ov9732->dev, "set analog voltage failed\n");
		return ret;
	}

	ret = regulator_enable(ov9732->dvdd);
	if (ret) {
		dev_err(ov9732->dev, "set core voltage failed\n");
		goto err_disable_analog;
	}

	return 0;

err_disable_analog:
	regulator_disable(ov9732->avdd);

	return ret;
}

static void ov9732_regulators_disable(struct ov9732 *ov9732)
{
	int ret;

	ret = regulator_disable(ov9732->dvdd);
	if (ret < 0)
		dev_err(ov9732->dev, "core regulator disable failed\n");

	ret = regulator_disable(ov9732->avdd);
	if (ret < 0)
		dev_err(ov9732->dev, "analog regulator disable failed\n");
}
#endif

#if 1
static int ov9732_write_reg_to(struct ov9732 *ov9732, u16 reg, u8 val, u16 i2c_addr)
{
	u8 regbuf[3] = {
		(u8) (reg >> 8),
		(u8) (reg & 0xff),
		val
	};
	struct i2c_msg msg[1];
	int ret;

	msg->addr = i2c_addr;
	msg->flags = 0;
	msg->len = 3;
	msg->buf = regbuf;

	//struct i2c_msg msgs = {
	//	.addr = i2c_addr,
	//	.flags = 0,
	//	.len = 3,
	//	.buf = regbuf
	//};

	ret = i2c_transfer(ov9732->i2c_client->adapter, msg, 1);
	if (ret < 0)
		dev_err(ov9732->dev,
			"%s: write reg error %d on addr 0x%x: reg=0x%x, val=0x%x\n",
			__func__, ret, i2c_addr, reg, val);

	return ret;
}
#endif

static int ov9732_write_reg(struct ov9732 *ov9732, u16 reg, u8 val)
{
	u8 regbuf[3];
	int ret;

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;
	regbuf[2] = val;

	ret = i2c_master_send(ov9732->i2c_client, regbuf, 3);
	if (ret < 0)
		dev_err(ov9732->dev, "%s: write reg error %d: reg=%x, val=%x\n",
			__func__, ret, reg, val);

	return ret;
}

static int ov9732_read_reg(struct ov9732 *ov9732, u16 reg, u8 *val)
{
	u8 regbuf[2];
	int ret;

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;

	ret = i2c_master_send(ov9732->i2c_client, regbuf, 2);
	if (ret < 0) {
		dev_err(ov9732->dev, "%s: write reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	ret = i2c_master_recv(ov9732->i2c_client, val, 1);
	if (ret < 0) {
		dev_err(ov9732->dev, "%s: read reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	return 0;
}

static int ov9732_mod_reg(struct ov9732 *sensor, u16 reg,
			  u8 mask, u8 val)
{
	u8 readval;
	int ret;

	ret = ov9732_read_reg(sensor, reg, &readval);
	if (ret)
		return ret;

	readval &= ~mask;
	val &= mask;
	val |= readval;

	return ov9732_write_reg(sensor, reg, val);
}

static int ov9732_set_aec_mode(struct ov9732 *ov9732, u32 mode)
{
	u8 val = ov9732->aec_pk_manual;
	int ret;

	return 0;

	if (mode == V4L2_EXPOSURE_AUTO)
		val &= ~REG_AEC_MANUAL_ENABLE;
	else /* V4L2_EXPOSURE_MANUAL */
		val |= REG_AEC_MANUAL_ENABLE;

	ret = ov9732_write_reg(ov9732, REG_AEC_PK_MANUAL, val);
	if (!ret)
		ov9732->aec_pk_manual = val;

	return ret;
}

static int ov9732_set_agc_mode(struct ov9732 *ov9732, u32 enable)
{
	u8 val = ov9732->aec_pk_manual;
	int ret;

	return 0;

	if (enable)
		val &= ~REG_AGC_MANUAL_ENABLE;
	else
		val |= REG_AGC_MANUAL_ENABLE;

	ret = ov9732_write_reg(ov9732, REG_AEC_PK_MANUAL, val);
	if (!ret)
		ov9732->aec_pk_manual = val;

	return ret;
}

static int ov9732_set_register_array(struct ov9732 *ov9732,
				     const struct reg_value *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		if (settings->mask)
			ret = ov9732_mod_reg(ov9732, settings->reg,
					     settings->mask, settings->val);
		else
			ret = ov9732_write_reg(ov9732, settings->reg,
					       settings->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

#if 0
static irqreturn_t ov9732_irq(int irq, void *devid)
{
	struct ov9732 *ov9732 = devid;

	/* TOGGLE TOF's VD_IN GPIO */
	gpiod_set_value_cansleep(ov9732->tof_vdin_gpio, 1);
	usleep_range(90,100);
	gpiod_set_value_cansleep(ov9732->tof_vdin_gpio, 0);

	return IRQ_HANDLED;
}
#endif

static void _set_exposure_reg(struct ov9732 *ov9732)
{
	u8 val1, val2, val3;
	int ret;
	u16 exposure = ov9732->exposure;

	val1 = (exposure & 0x0f000) >> 12;
	val2 = (exposure & 0x00ff0) >> 4;
	val3 = (exposure & 0xf) << 4;

	ret = ov9732_write_reg(ov9732, 0x3500, val1);
	if (ret < 0) {
		printk(KERN_ERR "register write failed %d!\n", ret);
		//return ret;
	}

	ret = ov9732_write_reg(ov9732, 0x3501, val2);
	if (ret < 0) {
		printk(KERN_ERR "register write failed %d!\n", ret);
		//return ret;
	}

	ret = ov9732_write_reg(ov9732, 0x3502, val3);
	if (ret < 0) {
		printk(KERN_ERR "register write failed %d!\n", ret);
		//return ret;
	}
}

static void
exposure_set_func(struct work_struct *work)
{
        struct ov9732 *ov9732 = container_of(to_delayed_work(work),
                             struct ov9732, exposure_write_work);

	_set_exposure_reg(ov9732);
}

static enum hrtimer_restart ov9732_hrtimer_callback(struct hrtimer *timer)
{
	struct ov9732 *ov9732 = container_of(timer, struct ov9732, timer);
	static bool toggle;
	const int TOF_DELAY=75*NSEC_PER_SEC/1000;
	const int TOTAL_PERIOD=NSEC_PER_SEC/10;
	const int RGB_DELAY=TOTAL_PERIOD-TOF_DELAY;
	ktime_t currtime, interval;

	toggle=!toggle;

	if(toggle)
	{
		interval = ktime_set(0, TOF_DELAY);
		/* TOGGLE TOF's VD_IN GPIO */
		gpiod_set_value_cansleep(ov9732->fsin_gpio, 1);
		udelay(100);
		gpiod_set_value_cansleep(ov9732->fsin_gpio, 0);
		if (ov9732->exposure_changed)
		{
			schedule_delayed_work(&ov9732->exposure_write_work, usecs_to_jiffies(ov9732->write_delay));
			ov9732->exposure_changed = false;
		}
	}
	else
	{
		interval = ktime_set(0, RGB_DELAY);
		gpiod_set_value_cansleep(ov9732->tof_vdin_gpio, 1);
		udelay(100);
		gpiod_set_value_cansleep(ov9732->tof_vdin_gpio, 0);
	}
	currtime  = ktime_get();
	hrtimer_forward(timer, currtime , interval);

	return HRTIMER_RESTART;
}

static int ov9732_set_power_on(struct ov9732 *ov9732)
{
#if 0
	int ret;

	ret = ov9732_regulators_enable(ov9732);
	if (ret < 0) {
		return ret;
	}
#endif
	if (ov9732->enable_gpio) {
		usleep_range(5000, 15000);
		gpiod_set_value_cansleep(ov9732->enable_gpio, 1);
	}

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ov9732->rst_gpio, 1);

	msleep(200);

	return 0;
}


static void ov9732_set_power_off(struct ov9732 *ov9732)
{
	gpiod_set_value_cansleep(ov9732->rst_gpio, 1);
	if (ov9732->enable_gpio)
		gpiod_set_value_cansleep(ov9732->enable_gpio, 0);
	//clk_disable_unprepare(ov9732->xclk);
	//ov9732_regulators_disable(ov9732);
}

static int ov9732_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov9732 *ov9732 = to_ov9732(sd);
	int ret = 0;

	mutex_lock(&ov9732->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	printk(KERN_ERR "I2C addr %x\n", ov9732->i2c_client->addr);

	if (ov9732->power_count == !on) {
		if (on) {
			mutex_lock(&ov9732_lock);

			ret = ov9732_set_power_on(ov9732);
			if (ret < 0)
				goto exit;

			ret = ov9732_write_reg_to(ov9732, 0x107,
					       ov9732->i2c_client->addr << 1, 0x36);
			if (ret < 0) {
				dev_err(ov9732->dev,
					"could not change i2c address\n");
				//ov9732_set_power_off(ov9732);
				//mutex_unlock(&ov9732_lock);
				goto exit;
			}

			mutex_unlock(&ov9732_lock);

			ret = ov9732_set_register_array(ov9732,
					ov9732_global_init_setting,
					ARRAY_SIZE(ov9732_global_init_setting));
			if (ret < 0) {
				dev_err(ov9732->dev,
					"could not set init registers\n");
				//ov9732_set_power_off(ov9732);
				goto exit;
			}

			msleep(20);

			ret = ov9732_mod_reg(ov9732, REG_MODE_SELECT, 0x1,
					     REG_MODE_SELECT_SLEEP);
			if (ret < 0) {
				//ov9732_set_power_off(ov9732);
				//goto exit;
			}
		} else {
			ov9732_set_power_off(ov9732);
		}
	}

	/* Update the power count. */
	ov9732->power_count += on ? 1 : -1;
	WARN_ON(ov9732->power_count < 0);

exit:
	mutex_unlock(&ov9732->power_lock);

	return ret;
}

static int ov9732_set_saturation(struct ov9732 *ov9732, s32 value)
{
	u32 reg_value = (value * 0x10) + 0x40;
	int ret;

	return 0;

	ret = ov9732_write_reg(ov9732, OV5645_SDE_SAT_U, reg_value);
	if (ret < 0)
		return ret;

	return ov9732_write_reg(ov9732, OV5645_SDE_SAT_V, reg_value);
}

static int ov9732_set_hflip(struct ov9732 *ov9732, s32 value)
{
	u8 val = ov9732->format0;
	int ret;

	return 0;

	if (value == 0)
		val &= ~(REG_HFLIP);
	else
		val |= (REG_HFLIP);

	ret = ov9732_write_reg(ov9732, REG_FORMAT0, val);
	if (!ret)
		ov9732->format0 = val;

	return ret;
}

static int ov9732_set_vflip(struct ov9732 *ov9732, s32 value)
{
	u8 val = ov9732->format0;
	int ret;

	return 0;

	if (value == 0)
		val |= (REG_VFLIP);
	else
		val &= ~(REG_VFLIP);

	ret = ov9732_write_reg(ov9732, REG_FORMAT0, val);
	if (!ret)
		ov9732->format0 = val;

	return ret;
}

static int ov9732_set_test_pattern(struct ov9732 *ov9732, s32 value)
{
	u8 val = 0;

	return 0;

	if (value) {
		val = REG_SET_TEST_PATTERN(value - 1);
		val |= REG_TEST_PATTERN_ENABLE;
	}

	return ov9732_write_reg(ov9732, REG_PRE_ISP_CTRL0, val);
}

static const char * const ov9732_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bars",
	"Pseudo-Random Data",
	"Color Square",
	"Black Image",
};

static int ov9732_set_awb(struct ov9732 *ov9732, s32 enable_auto)
{
	u8 val = 0;

	return 0;

	if (!enable_auto)
		val = OV5645_AWB_MANUAL_ENABLE;

	return ov9732_write_reg(ov9732, OV5645_AWB_MANUAL_CONTROL, val);
}

static int ov9732_set_red(struct ov9732 *ov9732, u16 red)
{
	ov9732->red_msb = (red & 0x0f00) >> 8;
	ov9732->red_lsb = (red & 0x00ff);

	if (ov9732->power_count) {
		ov9732_mod_reg(ov9732, 0x5180, 0x0f, ov9732->red_msb);
		ov9732_mod_reg(ov9732, 0x5181, 0xff, ov9732->red_lsb);
	}

	return 0;
}

static int ov9732_set_green(struct ov9732 *ov9732, u16 green)
{
	ov9732->green_msb = (green & 0x0f00) >> 8;
	ov9732->green_lsb = (green & 0x00ff);

	if (ov9732->power_count) {
		ov9732_mod_reg(ov9732, 0x5182, 0x0f, ov9732->green_msb);
		ov9732_mod_reg(ov9732, 0x5183, 0xff, ov9732->green_lsb);
	}

	return 0;
}

static int ov9732_set_blue(struct ov9732 *ov9732, u16 blue)
{
	ov9732->blue_msb = (blue & 0x0f00) >> 8;
	ov9732->blue_lsb = (blue & 0x00ff);

	if (ov9732->power_count) {
		ov9732_mod_reg(ov9732, 0x5184, 0x0f, ov9732->blue_msb);
		ov9732_mod_reg(ov9732, 0x5185, 0xff, ov9732->blue_lsb);
	}

	return 0;
}

static int ov9732_set_mwb(struct ov9732 *ov9732)
{
	ov9732_mod_reg(ov9732, 0x5180, 0x0f, ov9732->red_msb);
	ov9732_mod_reg(ov9732, 0x5181, 0xff, ov9732->red_lsb);

	ov9732_mod_reg(ov9732, 0x5182, 0x0f, ov9732->green_msb);
	ov9732_mod_reg(ov9732, 0x5183, 0xff, ov9732->green_lsb);

	ov9732_mod_reg(ov9732, 0x5184, 0x0f, ov9732->blue_msb);
	ov9732_mod_reg(ov9732, 0x5185, 0xff, ov9732->blue_lsb);

	return 0;
}

static int ov9732_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov9732 *ov9732 = container_of(ctrl->handler,
					     struct ov9732, ctrls);
	int ret;

	mutex_lock(&ov9732->power_lock);
	if (!ov9732->power_count && !(ctrl->id == V4L2_CID_RED_BALANCE ||
				     ctrl->id == V4L2_CID_BLUE_BALANCE ||
				     ctrl->id == V4L2_CID_GAIN)) {
		mutex_unlock(&ov9732->power_lock);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_SATURATION:
		ret = ov9732_set_saturation(ov9732, ctrl->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = ov9732_set_awb(ov9732, ctrl->val);
		break;
	case V4L2_CID_RED_BALANCE:
		ret = ov9732_set_red(ov9732, ctrl->val);
		break;
	case V4L2_CID_BLUE_BALANCE:
		ret = ov9732_set_blue(ov9732, ctrl->val);
		break;
	case V4L2_CID_GAIN:
		ret = ov9732_set_green(ov9732, ctrl->val);
		break;
	case V4L2_CID_AUTOGAIN:
		ret = ov9732_set_agc_mode(ov9732, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = ov9732_set_aec_mode(ov9732, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov9732_set_test_pattern(ov9732, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ov9732_set_hflip(ov9732, ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = ov9732_set_vflip(ov9732, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&ov9732->power_lock);

	return ret;
}

static struct v4l2_ctrl_ops ov9732_ctrl_ops = {
	.s_ctrl = ov9732_s_ctrl,
};

static int ov9732_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(ov9732_formats))
		return -EINVAL;

	code->code = ov9732_formats[code->index];

	return 0;
}

static int ov9732_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_SBGGR8_1X8 ||
	    fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(ov9732_mode_info_data))
		return -EINVAL;

	fse->min_width = ov9732_mode_info_data[fse->index].width;
	fse->max_width = ov9732_mode_info_data[fse->index].width;
	fse->min_height = ov9732_mode_info_data[fse->index].height;
	fse->max_height = ov9732_mode_info_data[fse->index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ov9732_get_pad_format(struct ov9732 *ov9732,
			struct v4l2_subdev_pad_config *cfg,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&ov9732->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov9732->fmt;
	default:
		return NULL;
	}
}

static int ov9732_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct ov9732 *ov9732 = to_ov9732(sd);

	format->format = *__ov9732_get_pad_format(ov9732, cfg, format->pad,
						  format->which);
	return 0;
}

static struct v4l2_rect *
__ov9732_get_pad_crop(struct ov9732 *ov9732, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&ov9732->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov9732->crop;
	default:
		return NULL;
	}
}

static const struct ov9732_mode_info *
ov9732_find_nearest_mode(unsigned int width, unsigned int height, u32 code)
{
	int i;

	for (i = ARRAY_SIZE(ov9732_mode_info_data) - 1; i >= 0; i--) {
		if (ov9732_mode_info_data[i].width == width &&
		    ov9732_mode_info_data[i].height == height &&
		    ov9732_mode_info_data[i].code == code) {
			break;
		}
	}

	if (i < 0)
		i = 0;

	return &ov9732_mode_info_data[i];
}

static int ov9732_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct ov9732 *ov9732 = to_ov9732(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	const struct ov9732_mode_info *new_mode;
	int ret;

	__crop = __ov9732_get_pad_crop(ov9732, cfg, format->pad,
			format->which);

	new_mode = ov9732_find_nearest_mode(format->format.width,
					    format->format.height,
					    format->format.code);
	__crop->width = new_mode->width;
	__crop->height = new_mode->height;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ret = v4l2_ctrl_s_ctrl_int64(ov9732->pixel_clock,
					     new_mode->pixel_clock);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_s_ctrl(ov9732->link_freq,
				       new_mode->link_freq);
		if (ret < 0)
			return ret;

		ov9732->current_mode = new_mode;
	}

	__format = __ov9732_get_pad_format(ov9732, cfg, format->pad,
			format->which);
	__format->width = __crop->width;
	__format->height = __crop->height;
	__format->code = new_mode->code;
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;

	format->format = *__format;

	return 0;
}

static int ov9732_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 1280;
	fmt.format.height = 720;

	ov9732_set_format(subdev, cfg, &fmt);

	return 0;
}

static int ov9732_get_selection(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_selection *sel)
{
	struct ov9732 *ov9732 = to_ov9732(sd);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__ov9732_get_pad_crop(ov9732, cfg, sel->pad,
					sel->which);
	return 0;
}

static int ov9732_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ov9732 *ov9732 = to_ov9732(subdev);
	int ret;

	if (enable) {
		ret = ov9732_set_register_array(ov9732,
					ov9732->current_mode->data,
					ov9732->current_mode->data_size);
		if (ret < 0) {
			dev_err(ov9732->dev, "could not set mode %dx%d\n",
				ov9732->current_mode->width,
				ov9732->current_mode->height);
			return ret;
		}

		ov9732_set_mwb(ov9732);

		ret = v4l2_ctrl_handler_setup(&ov9732->ctrls);
		if (ret < 0) {
			dev_err(ov9732->dev, "could not sync v4l2 controls\n");
			return ret;
		}
		//ret = ov9732_mod_reg(ov9732, REG_MODE_SELECT, 0x1,
		//		       REG_MODE_SELECT_STREAMING);
		//if (ret < 0)
		//	return ret;
	} else {
		ret = ov9732_mod_reg(ov9732, REG_MODE_SELECT, 0x1,
				       REG_MODE_SELECT_SLEEP);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops ov9732_core_ops = {
	.s_power = ov9732_s_power,
};

static const struct v4l2_subdev_video_ops ov9732_video_ops = {
	.s_stream = ov9732_s_stream,
};

static const struct v4l2_subdev_pad_ops ov9732_subdev_pad_ops = {
	//.init_cfg = ov9732_entity_init_cfg,
	.enum_mbus_code = ov9732_enum_mbus_code,
	.enum_frame_size = ov9732_enum_frame_size,
	.get_fmt = ov9732_get_format,
	.set_fmt = ov9732_set_format,
	.get_selection = ov9732_get_selection,
};

static const struct v4l2_subdev_ops ov9732_subdev_ops = {
	.core = &ov9732_core_ops,
	.video = &ov9732_video_ops,
	.pad = &ov9732_subdev_pad_ops,
};

static ssize_t reg_read(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(to_i2c_client(dev));
	struct ov9732 *ov9732 = to_ov9732(subdev);
	u8 val = 0;
	int ret;

	ret = ov9732_read_reg(ov9732, ov9732->reg, &val);
	if (ret) {
		printk(KERN_ERR "register read failed!\n");
		return ret;
	}

	return sprintf(buf, "reg = %x, val = %x\n", ov9732->reg, val);
}

static ssize_t reg_write(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t size)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(to_i2c_client(dev));
	struct ov9732 *ov9732 = to_ov9732(subdev);
	u32 reg = 0;
	u32 val = 0;
	int ret;

	if (sscanf(buf, "%x %x", &reg, &val) != 2) {
		printk(KERN_ERR "error reading reg");
		return -EINVAL;
	}

	ret = ov9732_write_reg(ov9732, (u16) reg, (u8) val);
	if (ret) {
		printk(KERN_ERR "register read failed!\n");
		return ret;
	}
	ov9732->reg = reg;

	return size;
}
static DEVICE_ATTR(reg, 0664, reg_read, reg_write);

static ssize_t exposure_read(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(to_i2c_client(dev));
	struct ov9732 *ov9732 = to_ov9732(subdev);
	u16 exposure;
	u8 val1, val2, val3;
	int ret;

	ret = ov9732_read_reg(ov9732, 0x3500, &val1);
	if (ret) {
		printk(KERN_ERR "register read failed!\n");
		return ret;
	}

	ret = ov9732_read_reg(ov9732, 0x3501, &val2);
	if (ret) {
		printk(KERN_ERR "register read failed!\n");
		return ret;
	}

	ret = ov9732_read_reg(ov9732, 0x3502, &val3);
	if (ret) {
		printk(KERN_ERR "register read failed!\n");
		return ret;
	}

	exposure = (val1 & 0xf) << 12 | val2 << 4 | (val3 & 0xf0) >> 4;

	return sprintf(buf, "%x", exposure);
}

static ssize_t exposure_write(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t size)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(to_i2c_client(dev));
	struct ov9732 *ov9732 = to_ov9732(subdev);
	u16 exposure;

	if (sscanf(buf, "%hu", &exposure) != 1) {
		printk(KERN_ERR "error reading reg");
		return -EINVAL;
	}

	ov9732->exposure = exposure;

	ov9732->exposure_changed = true;

	return size;
}
static DEVICE_ATTR(exposure, 0664, exposure_read, exposure_write);


static ssize_t delay_read(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(to_i2c_client(dev));
	struct ov9732 *ov9732 = to_ov9732(subdev);

	return sprintf(buf, "%lu", ov9732->write_delay);
}


static ssize_t delay_write(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t size)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(to_i2c_client(dev));
	struct ov9732 *ov9732 = to_ov9732(subdev);
	unsigned long write_delay = 0;

	if (sscanf(buf, "%lu", &write_delay) != 1) {
		printk(KERN_ERR "error reading write_delay");
		return -EINVAL;
	}

	ov9732->write_delay = write_delay;

	return size;
}
static DEVICE_ATTR(delay, 0664, delay_read, delay_write);

struct reg_ranges {
	u16 min, max;
	char name[50];
};

static const struct reg_ranges ov9732_reg_ranges[] = {
	{ 0x100, 0x106, "general" },
	{ 0x3000, 0x3009, "IO" },
	{ 0x3080, 0x3103, "PLL" },
	{ 0x3500, 0x3513, "AEC" },
	{ 0x3800, 0x3829, "FT" },
	{ 0x4000, 0x405d, "BLC" },
	{ 0x4201, 0x4202, "FC" },
	{ 0x4300, 0x4303, "Clip" },
	{ 0x4308, 0x4308, "Fmt Ctrl" },
	{ 0x4700, 0x470f, "DVP" },
	{ 0x4800, 0x484f, "MIPI" },
	{ 0x5000, 0x5017, "ISP" },
	{ 0x5080, 0x5080, "pre ISP" },
	{ 0x5100, 0x5105, "DGC" },
	{ 0x5180, 0x5185, "MWB" },
	{ 0x5700, 0x570c, "WINC" },
	{ 0x5780, 0x578f, "DPC" },
};

static void dump_read(struct ov9732 *ov9732, struct seq_file *s)
{
	u8 val;
	int ret;
	int i, j;
	const struct reg_ranges *ranges = ov9732_reg_ranges;

	for (i = 0; i < ARRAY_SIZE(ov9732_reg_ranges); i++, ranges++) {
		seq_printf(s, "%s:\n", ranges->name);
		for (j = ranges->min; j <= ranges->max; j++) {
			ret = ov9732_read_reg(ov9732, j, &val);
			if (ret) {
				printk(KERN_ERR "register read failed!\n");
				return;
			}
			seq_printf(s, "reg = %x, val = %x\n", j, val);
		}
	}
}

static int ov9732_debug_show(struct seq_file *s, void *unused)
{
	struct ov9732 *ov9732 = s->private;

	dump_read(ov9732, s);

	return 0;
}

static int ov9732_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, ov9732_debug_show, inode->i_private);
}

static const struct file_operations ov9732_debug_fops = {
	.open           = ov9732_debug_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static struct dentry *ov9732_debugfs_dir;

static int ov9732_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
//	struct device_node *endpoint;
	struct ov9732 *ov9732;
	u8 chip_id_high, chip_id_low;
	u32 xclk_freq;
	ktime_t ktime;
	int ret;

	ov9732 = devm_kzalloc(dev, sizeof(struct ov9732), GFP_KERNEL);
	if (!ov9732)
		return -ENOMEM;

	ov9732->i2c_client = client;
	ov9732->dev = dev;

#if 0
	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_of_parse_endpoint(endpoint, &ov9732->ep);
	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}

	of_node_put(endpoint);

	if (ov9732->ep.bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "invalid bus type, must be CSI2\n");
		return -EINVAL;
	}
#endif
	/* get system clock (xclk) */
	ov9732->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(ov9732->xclk)) {
		dev_err(dev, "could not get xclk");
		return PTR_ERR(ov9732->xclk);
	}

	ret = of_property_read_u32(dev->of_node, "clock-frequency", &xclk_freq);
	if (ret) {
		dev_err(dev, "could not get xclk frequency\n");
		return ret;
	}

	if (xclk_freq != 24000000) {
		dev_err(dev, "external clock frequency %u is not supported\n",
			xclk_freq);
		return -EINVAL;
	}

	ret = clk_set_rate(ov9732->xclk, xclk_freq);
	if (ret) {
		dev_err(dev, "could not set xclk frequency\n");
		return ret;
	}

	ret = clk_prepare_enable(ov9732->xclk);
	if (ret < 0) {
		dev_err(ov9732->dev, "clk prepare enable failed\n");
		//ov9732_regulators_disable(ov9732);
		return ret;
	}

#if 0
	ov9732->dvdd = devm_regulator_get(dev, "dvdd");
	if (IS_ERR(ov9732->dvdd)) {
		dev_err(dev, "cannot get core regulator\n");
		return PTR_ERR(ov9732->dvdd);
	}

	ret = regulator_set_voltage(ov9732->dvdd,
				    1800000, 1800000);
	if (ret < 0) {
		dev_err(dev, "cannot set core voltage\n");
		return ret;
	}

	ov9732->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(ov9732->avdd)) {
		dev_err(dev, "cannot get analog regulator\n");
		return PTR_ERR(ov9732->avdd);
	}

	ret = regulator_set_voltage(ov9732->avdd,
				    2800000, 2800000);
	if (ret < 0) {
		dev_err(dev, "cannot set analog voltage\n");
		return ret;
	}
#endif

	ov9732->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ov9732->enable_gpio)) {
		dev_err(dev, "cannot get enable gpio\n");
		return PTR_ERR(ov9732->enable_gpio);
	}

	ov9732->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov9732->rst_gpio)) {
		dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(ov9732->rst_gpio);
	}

#if 0
	ov9732->tof_vdin_gpio = devm_gpiod_get(dev, "tof", GPIOD_OUT_LOW);
	if (IS_ERR(ov9732->tof_vdin_gpio)) {
		dev_err(dev, "cannot get tof gpio\n");
		return PTR_ERR(ov9732->tof_vdin_gpio);
	}
	ov9732->fsin_gpio = devm_gpiod_get(dev, "fsin", GPIOD_OUT_LOW);
	if (IS_ERR(ov9732->fsin_gpio)) {
		dev_err(dev, "cannot get tof gpio\n");
		return PTR_ERR(ov9732->fsin_gpio);
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					ov9732_irq,
					IRQF_ONESHOT, dev_name(dev),
					ov9732);
	if (ret)
		return ret;
#endif
	mutex_init(&ov9732->power_lock);

	v4l2_ctrl_handler_init(&ov9732->ctrls, 9);
	v4l2_ctrl_new_std(&ov9732->ctrls, &ov9732_ctrl_ops,
			  V4L2_CID_SATURATION, -4, 4, 1, 0);
	v4l2_ctrl_new_std(&ov9732->ctrls, &ov9732_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ov9732->ctrls, &ov9732_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&ov9732->ctrls, &ov9732_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	v4l2_ctrl_new_std(&ov9732->ctrls, &ov9732_ctrl_ops,
			  V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
	v4l2_ctrl_new_std_menu(&ov9732->ctrls, &ov9732_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
			       0, V4L2_EXPOSURE_AUTO);
	v4l2_ctrl_new_std_menu_items(&ov9732->ctrls, &ov9732_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov9732_test_pattern_menu) - 1,
				     0, 0, ov9732_test_pattern_menu);
	ov9732->pixel_clock = v4l2_ctrl_new_std(&ov9732->ctrls,
						&ov9732_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						1, INT_MAX, 1, 1);
	ov9732->link_freq = v4l2_ctrl_new_int_menu(&ov9732->ctrls,
						   &ov9732_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq) - 1,
						   0, link_freq);

	v4l2_ctrl_new_std(&ov9732->ctrls, &ov9732_ctrl_ops,
			  V4L2_CID_RED_BALANCE, 0, 8191, 1, 0x180);
	v4l2_ctrl_new_std(&ov9732->ctrls, &ov9732_ctrl_ops,
			  V4L2_CID_BLUE_BALANCE, 0, 8191, 1, 0x180);
	v4l2_ctrl_new_std(&ov9732->ctrls, &ov9732_ctrl_ops,
			  V4L2_CID_GAIN, 0, 8191, 1, 0x100);

	ov9732->red_msb = 0x1;
	ov9732->red_lsb = 0x80;

	ov9732->green_msb = 0x1;
	ov9732->green_lsb = 0x00;

	ov9732->blue_msb = 0x1;
	ov9732->blue_lsb = 0x80;

	if (ov9732->link_freq)
		ov9732->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ov9732->sd.ctrl_handler = &ov9732->ctrls;

	if (ov9732->ctrls.error) {
		dev_err(dev, "%s: control initialization error %d\n",
		       __func__, ov9732->ctrls.error);
		ret = ov9732->ctrls.error;
		goto free_ctrl;
	}


	v4l2_i2c_subdev_init(&ov9732->sd, client, &ov9732_subdev_ops);
	ov9732->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov9732->pad.flags = MEDIA_PAD_FL_SOURCE;
	ov9732->sd.dev = &client->dev;
	ov9732->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	//ov9732->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	if (device_create_file(ov9732->dev, &dev_attr_reg))
		device_remove_file(ov9732->dev, &dev_attr_reg);

	if (device_create_file(ov9732->dev, &dev_attr_exposure))
		device_remove_file(ov9732->dev, &dev_attr_exposure);

	if (device_create_file(ov9732->dev, &dev_attr_delay))
		device_remove_file(ov9732->dev, &dev_attr_delay);

	ov9732_debugfs_dir = debugfs_create_dir("ov9732", NULL);
	if (IS_ERR(ov9732_debugfs_dir)) {
		int err = PTR_ERR(ov9732_debugfs_dir);
		ov9732_debugfs_dir = NULL;
		return err;
	}

	debugfs_create_file("dump", S_IRUGO, ov9732_debugfs_dir,
                        ov9732, &ov9732_debug_fops);

	ret = media_entity_init(&ov9732->sd.entity, 1, &ov9732->pad, 0);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		goto free_ctrl;
	}

	ret = ov9732_s_power(&ov9732->sd, true);
	if (ret < 0) {
		dev_err(dev, "could not power up OV5645\n");
		return -EPROBE_DEFER;
	}

	ret = ov9732_read_reg(ov9732, REG_CHIP_ID_HIGH, &chip_id_high);
	if (ret < 0 || chip_id_high != REG_CHIP_ID_HIGH_BYTE) {
		dev_err(dev, "could not read ID high\n");
		//ret = -ENODEV;
		//goto power_down;
	}
	ret = ov9732_read_reg(ov9732, REG_CHIP_ID_LOW, &chip_id_low);
	if (ret < 0 || chip_id_low != REG_CHIP_ID_LOW_BYTE) {
		dev_err(dev, "could not read ID low\n");
		//ret = -ENODEV;
		//goto power_down;
	}

	printk(KERN_ERR "chip id high %x low %x\n", chip_id_high, chip_id_low);

#if 0
	dev_info(dev, "OV9732 detected at address 0x%02x\n", client->addr);

	ret = ov9732_read_reg(ov9732, REG_AEC_PK_MANUAL,
			      &ov9732->aec_pk_manual);
	if (ret < 0) {
		dev_err(dev, "could not read AEC/AGC mode\n");
		ret = -ENODEV;
		goto power_down;
	}

	ret = ov9732_read_reg(ov9732, REG_FORMAT0,
			      &ov9732->format0);
	if (ret < 0) {
		dev_err(dev, "could not read vflip/hflip value\n");
		ret = -ENODEV;
		goto power_down;
	}

	ret = ov9732_read_reg(ov9732, REG_FORMAT1,
			      &ov9732->format1);
	if (ret < 0) {
		dev_err(dev, "could not read format1 value\n");
		ret = -ENODEV;
		goto power_down;
	}

	ov9732_s_power(&ov9732->sd, false);
#endif
	ret = v4l2_async_register_subdev(&ov9732->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	ov9732_entity_init_cfg(&ov9732->sd, NULL);

	return 0;

	INIT_DELAYED_WORK(&ov9732->exposure_write_work,exposure_set_func);
	ov9732->write_delay = 100;

	hrtimer_init(&ov9732->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ov9732->timer.function = ov9732_hrtimer_callback;

	ktime = ktime_set(0, NSEC_PER_SEC / 20);

	hrtimer_start(&ov9732->timer, ktime, HRTIMER_MODE_REL);

	return 0;

//power_down:
	ov9732_s_power(&ov9732->sd, false);
free_entity:
	media_entity_cleanup(&ov9732->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&ov9732->ctrls);
	mutex_destroy(&ov9732->power_lock);

	return ret;
}

static int ov9732_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9732 *ov9732 = to_ov9732(sd);

	v4l2_async_unregister_subdev(&ov9732->sd);
	media_entity_cleanup(&ov9732->sd.entity);
	v4l2_ctrl_handler_free(&ov9732->ctrls);
	mutex_destroy(&ov9732->power_lock);

	return 0;
}

static const struct i2c_device_id ov9732_id[] = {
	{ "ov9732", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ov9732_id);

static const struct of_device_id ov9732_of_match[] = {
	{ .compatible = "ovti,ov9732" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov9732_of_match);

static struct i2c_driver ov9732_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ov9732_of_match),
		.name  = "ov9732",
	},
	.probe  = ov9732_probe,
	.remove = ov9732_remove,
	.id_table = ov9732_id,
};

module_i2c_driver(ov9732_i2c_driver);

MODULE_DESCRIPTION("Omnivision OV5645 Camera Driver");
MODULE_AUTHOR("Todor Tomov <todor.tomov@linaro.org>");
MODULE_LICENSE("GPL v2");

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
#include <linux/io.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/mfd/qcom_pdm.h>

struct opt8320 {
	struct i2c_client *i2c_client;
	struct device *dev;

	struct regmap *regmap;

	struct regulator *dvdd;
	struct regulator *avdd;

	int psen_status;
	struct gpio_desc *psen_gpio;
	struct gpio_desc *rst_gpio;
	int ir_on;

	struct clk *mclk;

	struct qcom_pdm *pdm;

	u8 reg;
};

static const struct regmap_config opt8320_regmap_config = {
	.reg_bits = 8,
	.val_bits = 24,

	.max_register = 0xff,
};

static void check_resetz(void)
{
	void __iomem *base = ioremap(0x1072000, SZ_1K);

	printk(KERN_ERR "%x %x %x\n", __raw_readl(base), __raw_readl(base + 0x4), __raw_readl(base + 0x8));

	iounmap(base);
}

static ssize_t reg_read(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct opt8320 *opt = i2c_get_clientdata(to_i2c_client(dev));
	u32 val = 0;
	int ret;

	ret = regmap_read(opt->regmap, opt->reg, &val);
	if (ret) {
		printk(KERN_ERR "register read failed!\n");
		return ret;
	}

	return sprintf(buf, "reg = %x, val = %x\n", opt->reg, val);
}

static ssize_t reg_write(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t size)
{
	struct opt8320 *opt = i2c_get_clientdata(to_i2c_client(dev));
	u32 reg = 0;
	u32 val = 0;
	int ret;

	if (sscanf(buf, "%x %x", &reg, &val) != 2) {
		printk(KERN_ERR "error reading reg");
		return -EINVAL;
	}

	ret = regmap_write(opt->regmap, (u8) reg, val);
	if (ret) {
		printk(KERN_ERR "register read failed!\n");
		return ret;
	}
	opt->reg = (u8) reg;

	return size;
}
static DEVICE_ATTR(reg, 0664, reg_read, reg_write);

static ssize_t mix_illum_en_read(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct opt8320 *opt = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%d\n", opt->psen_status);
}

static ssize_t mix_illum_en_write(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t size)
{
	struct opt8320 *opt = i2c_get_clientdata(to_i2c_client(dev));
	int val = 0;

	if (sscanf(buf, "%d", &val) != 1) {
		printk(KERN_ERR "error reading sysfs");
		return -EINVAL;
	}

	if (val != 0 && val != 1) {
		return -ERANGE;
	}

	gpiod_set_value_cansleep(opt->psen_gpio, val);
	opt->psen_status = val;

	return size;
}
static DEVICE_ATTR(mix_illum_en, 0664, mix_illum_en_read, mix_illum_en_write);

static void tof_sw_reset(struct opt8320 *opt)
{
	/* regmap strangely doesn't support little endian for 24 bit registers,
	 * so let's write reverse while writing it.
	 */
	/* software_reset */
	regmap_update_bits(opt->regmap, 0x00, 0x10000, 0x10000);

	msleep(100);

	/* enable tg_en */
	regmap_update_bits(opt->regmap, 0x80, 0x10000, 0x10000);
	/* disable standby */
	regmap_update_bits(opt->regmap, 0x08, 0x40000, 0x0);
	/* disable tg_en */
	regmap_update_bits(opt->regmap, 0x80, 0x10000, 0x0);

	msleep(100);

	if (opt->ir_on == 0)
		regmap_write(opt->regmap, 0xd6, 0x801a06);
	else
		regmap_write(opt->regmap, 0xd6, 0x40420f);
}

static ssize_t reset_write(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t size)
{
	struct opt8320 *opt = i2c_get_clientdata(to_i2c_client(dev));
	u32 val = 0;

	if (sscanf(buf, "%d", &val) != 1) {
		printk(KERN_ERR "error reading sysfs");
		return -EINVAL;
	}

	if (val != 1) {
		return -EINVAL;
	}

	tof_sw_reset(opt);

	return size;
}
static DEVICE_ATTR(reset, 0200, NULL, reset_write);

static ssize_t ir_on_read(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct opt8320 *opt = i2c_get_clientdata(to_i2c_client(dev));
	u32 val;
	int ir_on;

	regmap_read(opt->regmap, 0xd6, &val);

	if (val == 0x801a06)
		ir_on = 0;
	else if (val == 0x40420f)
		ir_on = 1;
	else
		ir_on = -1;

	return sprintf(buf, "%d\n", ir_on);
}

static ssize_t ir_on_write(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t size)
{
	struct opt8320 *opt = i2c_get_clientdata(to_i2c_client(dev));
	int val = 0;

	if (sscanf(buf, "%d", &val) != 1) {
		printk(KERN_ERR "error reading sysfs");
		return -EINVAL;
	}

	if (val != 0 && val != 1) {
		return -ERANGE;
	}

	if (val == 0)
		regmap_write(opt->regmap, 0xd6, 0x801a06);	/* 0x061a80: 400000 */
	else
		regmap_write(opt->regmap, 0xd6, 0x40420f);	/* 0x0f4240: 1000000 */

	opt->ir_on = val;

	return size;
}
static DEVICE_ATTR(ir_on, 0664, ir_on_read, ir_on_write);

static int opt8320_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct opt8320 *opt;
	struct device_node *pdm_node;
	struct qcom_pdm *pdm;
	int ret;
	int i;
	u32 val;
	u16 pdm_vals[] = { 0x1000, 0x3000, 0x6000, 0x9000, 0xb000, 0xffff };
	

	opt = devm_kzalloc(dev, sizeof(struct opt8320), GFP_KERNEL);
	if (!opt)
		return -ENOMEM;

	opt->i2c_client = client;
	opt->dev = dev;

	i2c_set_clientdata(client, opt);

	pdm_node = of_parse_phandle(dev->of_node, "pdm", 0);
	if (!pdm_node) {
		dev_err(dev, "couldn't find pdm node\n");
		return -ENODEV;
	}

	pdm = of_get_pdm_dev(pdm_node);
	if (!pdm) {
		dev_dbg(dev, "couldn't find pdm, deferring probe\n");
		of_node_put(pdm_node);
		return -EPROBE_DEFER;
	}

	of_node_put(pdm_node);

	opt->pdm = pdm;

	opt->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(opt->avdd)) {
		dev_err(dev, "cannot get AVDD 1.8V regulator\n");
		return PTR_ERR(opt->avdd);
	}

	opt->psen_gpio = devm_gpiod_get_optional(dev, "psen", GPIOD_OUT_LOW);
	if (IS_ERR(opt->psen_gpio)) {
		dev_err(dev, "cannot get enable gpio\n");
		return PTR_ERR(opt->psen_gpio);
	}

	opt->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(opt->rst_gpio)) {
		dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(opt->rst_gpio);
	}

	/* get system clock (xclk) */
	opt->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(opt->mclk)) {
		dev_err(dev, "could not get mclk");
		return PTR_ERR(opt->mclk);
	}

	ret = pinctrl_pm_select_default_state(dev);
	if (ret)
		return ret;

	ret = regulator_enable(opt->avdd);
	if (ret) {
		dev_err(dev, "failed to enable reg");
		return PTR_ERR(opt->avdd);
	}

	msleep(10);

	ret = clk_set_rate(opt->mclk, 24000000);
	printk(KERN_ERR "set rate returned %d\n", ret);

	clk_prepare_enable(opt->mclk);

	msleep(10);

	/* Test sequence */
	gpiod_set_value_cansleep(opt->rst_gpio, 0);
	usleep_range(100,150);
	gpiod_set_value_cansleep(opt->rst_gpio, 1);
	usleep_range(100,150);
	//gpiod_set_value_cansleep(opt->rst_gpio, 0);
	//msleep(10);

	
	//gpiod_set_value_cansleep(opt->rst_gpio, 1);
	//msleep(10);

	check_resetz();

	gpiod_set_value_cansleep(opt->psen_gpio, 1);
	opt->psen_status = 1;
	msleep(10);

	for (i = 0; i < ARRAY_SIZE(pdm_vals); i++) {
		qcom_pdm_write(opt->pdm, 2, pdm_vals[i]);
		msleep(100);
	}

	opt->regmap = devm_regmap_init_i2c(client, &opt8320_regmap_config);
	if (IS_ERR(opt->regmap)) {
		ret = PTR_ERR(opt->regmap);
		return ret;
	}

	if (device_create_file(opt->dev, &dev_attr_reg))
		device_remove_file(opt->dev, &dev_attr_reg);

	if (device_create_file(opt->dev, &dev_attr_mix_illum_en))
		device_remove_file(opt->dev, &dev_attr_mix_illum_en);

	if (device_create_file(opt->dev, &dev_attr_reset))
		device_remove_file(opt->dev, &dev_attr_reset);

	if (device_create_file(opt->dev, &dev_attr_ir_on))
		device_remove_file(opt->dev, &dev_attr_ir_on);

	tof_sw_reset(opt);

	ret = regmap_read(opt->regmap, 0x08, &val);
	if (ret < 0) {
		printk(KERN_ERR "failed to read STANDBY\n");
	} else {
		printk(KERN_ERR "OPT STANDBY state: %x\n", val);
	}

	/* set ir_on to 0 */
	opt->ir_on = 0;
	regmap_write(opt->regmap, 0xd6, 0x801a06);

	return 0;
}

static int opt8320_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id opt8320_id[] = {
	{ "opt8320", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, opt8320_id);

static const struct of_device_id opt8320_of_match[] = {
	{ .compatible = "ti,opt8320" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, opt8320_of_match);

static struct i2c_driver opt8320_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(opt8320_of_match),
		.name  = "opt8320",
	},
	.probe  = opt8320_probe,
	.remove = opt8320_remove,
	.id_table = opt8320_id,
};

module_i2c_driver(opt8320_i2c_driver);
MODULE_AUTHOR("Archit Taneja <archit@cradlewise.com>");
MODULE_LICENSE("GPL v2");
MODULE_SOFTDEP("pre: ov9732");

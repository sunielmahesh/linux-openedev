// SPDX-License-Identifier: GPL-2.0-only
/*
 * Rockchip machine ASoC driver for boards using MAX98357A/RT5514/DA7219
 *
 * Copyright (c) 2016, ROCKCHIP CORPORATION.  All rights reserved.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "rockchip_i2s.h"

#define DRV_NAME "rk3399-gru-sound"

#define SOUND_FS	256

static unsigned int dmic_wakeup_delay;

static const struct snd_soc_dapm_widget rockchip_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Int Mic", NULL),
};

static const struct snd_kcontrol_new rockchip_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Mic"),
};

static int rockchip_sound_dmic_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int mclk;
	int ret;

	mclk = params_rate(params) * SOUND_FS;

	ret = snd_soc_dai_set_sysclk(rtd->cpu_dai, 0, mclk, 0);
	if (ret) {
		dev_err(rtd->card->dev, "%s() error setting sysclk to %u: %d\n",
				__func__, mclk, ret);
		return ret;
	}

	/* Wait for DMIC stable */
	msleep(dmic_wakeup_delay);

	return 0;
}

static const struct snd_soc_ops rockchip_sound_dmic_ops = {
	.hw_params = rockchip_sound_dmic_hw_params,
};

enum {
	DAILINK_CDNDP,
	DAILINK_DA7219,
	DAILINK_DMIC,
	DAILINK_MAX98357A,
	DAILINK_RT5514,
	DAILINK_RT5514_DSP,
};

static const struct snd_soc_dai_link rockchip_dais[] = {
	[DAILINK_DMIC] = {
		.name = "DMIC",
		.stream_name = "DMIC PCM",
		.codec_name = "dmic-hifi",
		.ops = &rockchip_sound_dmic_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
};

static const struct snd_soc_dapm_route rockchip_sound_cdndp_routes[] = {
	/* Output */
	{"HDMI", NULL, "TX"},
};

static const struct snd_soc_dapm_route rockchip_sound_da7219_routes[] = {
	/* Output */
	{"Headphones", NULL, "HPL"},
	{"Headphones", NULL, "HPR"},

	/* Input */
	{"MIC", NULL, "Headset Mic"},
};

static const struct snd_soc_dapm_route rockchip_sound_dmic_routes[] = {
	/* Input */
	{"DMic", NULL, "Int Mic"},
};

static const struct snd_soc_dapm_route rockchip_sound_max98357a_routes[] = {
	/* Output */
	{"Speakers", NULL, "Speaker"},
};

static const struct snd_soc_dapm_route rockchip_sound_rt5514_routes[] = {
	/* Input */
	{"DMIC1L", NULL, "Int Mic"},
	{"DMIC1R", NULL, "Int Mic"},
};

struct rockchip_sound_route {
	const struct snd_soc_dapm_route *routes;
	int num_routes;
};

static const struct rockchip_sound_route rockchip_routes[] = {
	[DAILINK_CDNDP] = {
		.routes = rockchip_sound_cdndp_routes,
		.num_routes = ARRAY_SIZE(rockchip_sound_cdndp_routes),
	},
	[DAILINK_DA7219] = {
		.routes = rockchip_sound_da7219_routes,
		.num_routes = ARRAY_SIZE(rockchip_sound_da7219_routes),
	},
	[DAILINK_DMIC] = {
		.routes = rockchip_sound_dmic_routes,
		.num_routes = ARRAY_SIZE(rockchip_sound_dmic_routes),
	},
	[DAILINK_MAX98357A] = {
		.routes = rockchip_sound_max98357a_routes,
		.num_routes = ARRAY_SIZE(rockchip_sound_max98357a_routes),
	},
	[DAILINK_RT5514] = {
		.routes = rockchip_sound_rt5514_routes,
		.num_routes = ARRAY_SIZE(rockchip_sound_rt5514_routes),
	},
	[DAILINK_RT5514_DSP] = {},
};

static struct snd_soc_dai_link dmic_dailink = {
		.name = "DMIC",
		.stream_name = "DMIC PCM",
		.codec_dai_name = "dmic-hifi",
		.ops = &rockchip_sound_dmic_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
};

static struct snd_soc_card rockchip_sound_card = {
	.name = "rk3399-gru-sound",
	.owner = THIS_MODULE,
	.dapm_widgets = rockchip_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rockchip_dapm_widgets),
	.controls = rockchip_controls,
	.num_controls = ARRAY_SIZE(rockchip_controls),
	.dai_link = &dmic_dailink,
	.num_links = 1,
	.dapm_routes = rockchip_sound_dmic_routes,
};

static int rockchip_sound_of_parse_dais(struct device *dev,
					struct snd_soc_card *card)
{
	struct device_node *np_cpu, *np_cpu0;
	struct device_node *np_codec;
	struct snd_soc_dai_link *dai = &dmic_dailink;

	np_cpu0 = of_parse_phandle(dev->of_node, "rockchip,cpu", 0);
	np_codec = of_parse_phandle(dev->of_node, "rockchip,codec", 0);

	np_cpu = np_cpu0;

	dai->platform_of_node = np_cpu;
	dai->cpu_of_node = np_cpu;
	dai->codec_of_node = np_codec;

	of_node_put(np_cpu0);
	of_node_put(np_codec);

	return 0;
}

static int rockchip_sound_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &rockchip_sound_card;
	int ret;

	printk(KERN_ERR "ROCKCHIP_SOUND_PROBE\n");

	ret = rockchip_sound_of_parse_dais(&pdev->dev, card);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to parse dais: %d\n", ret);
		return ret;
	}

	/* Set DMIC wakeup delay */
	ret = device_property_read_u32(&pdev->dev, "dmic-wakeup-delay-ms",
					&dmic_wakeup_delay);
	if (ret) {
		dmic_wakeup_delay = 0;
		dev_dbg(&pdev->dev,
			"no optional property 'dmic-wakeup-delay-ms' found, default: no delay\n");
	}

	card->dev = &pdev->dev;
	ret = devm_snd_soc_register_card(&pdev->dev, card);
	printk(KERN_ERR "RETURNING %d\n", ret);

	return ret;
}

static const struct of_device_id rockchip_sound_of_match[] = {
	{ .compatible = "rockchip,rk3399-gru-sound", },
	{},
};

static struct platform_driver rockchip_sound_driver = {
	.probe = rockchip_sound_probe,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = rockchip_sound_of_match,
#ifdef CONFIG_PM
		.pm = &snd_soc_pm_ops,
#endif
	},
};

module_platform_driver(rockchip_sound_driver);

MODULE_AUTHOR("Xing Zheng <zhengxing@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip ASoC Machine Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, rockchip_sound_of_match);

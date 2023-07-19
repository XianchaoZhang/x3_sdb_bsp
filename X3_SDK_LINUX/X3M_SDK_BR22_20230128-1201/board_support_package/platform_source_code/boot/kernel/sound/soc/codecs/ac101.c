/*
 * ac101.c
 *
 * (C) Copyright 2017-2018
 * Seeed Technology Co., Ltd. <www.seeedstudio.com>
 *
 * PeterYang <linsheng.yang@seeed.cc>
 *
 * (C) Copyright 2014-2017
 * Reuuimlla Technology Co., Ltd. <www.reuuimllatech.com>
 *
 * huangxin <huangxin@Reuuimllatech.com>
 * liushaohua <liushaohua@allwinnertech.com>
 *
 * X-Powers AC101 codec driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

/* #undef AC101_DEBG
 * use 'make DEBUG=1' to enable debugging
 */
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>
#include <linux/input.h>
#include <linux/delay.h>
#include "ac101_regs.h"
#include "ac10x.h"
#include <linux/gpio.h>

#define AC101_CHANNELS_MAX 2
#define _MASTER_INDEX	0
#define AC101_BCLK_DIV 8
/*
 * *** To sync channels ***
 *
 * 1. disable clock in codec   hw_params()
 * 2. clear   fifo  in bcm2835 hw_params()
 * 3. clear   fifo  in bcm2385 prepare()
 * 4. enable  RX    in bcm2835 trigger()
 * 5. enable  clock in machine trigger()
 */

/*Default initialize configuration*/
static bool speaker_double_used = true;
static int double_speaker_val	= 0x1B;
static int single_speaker_val	= 0x19;
static int headset_val		= 0x3B;
static int mainmic_val		= 0x4;
static int headsetmic_val	= 0x4;
static bool dmic_used		= true;
static int adc_digital_val	= 0xb0b0;
static bool drc_used		= true;

#define AC101_RATES SNDRV_PCM_RATE_8000_96000
#define AC101_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | \
                        SNDRV_PCM_FMTBIT_S24_LE |       \
                        SNDRV_PCM_FMTBIT_S32_LE | \
                        0)

static struct ac10x_priv* static_ac10x;
struct i2c_client *i2c_driver_test[(AC101_CHANNELS_MAX+3)/4];

static int ac101_set_clock(int y_start_n_stop);

int ac101_read(struct snd_soc_codec *codec, unsigned reg) {
	struct ac10x_priv *ac10x = static_ac10x;
	int r, v = 0;

	if ((r = regmap_read(ac10x->regmap101, reg, &v)) < 0) {
		dev_err(codec->dev, "read reg %02X fail\n",
			 reg);
		return r;
	}
	return v;
}

int ac101_write(struct snd_soc_codec *codec, unsigned reg, unsigned val) {
	struct ac10x_priv *ac10x = static_ac10x;
	int v;

	v = regmap_write(ac10x->regmap101, reg, val);
	return v;
}

int ac101_update_bits(struct snd_soc_codec *codec, unsigned reg,
			unsigned mask, unsigned value) {
	struct ac10x_priv *ac10x = static_ac10x;
	int v;

	v = regmap_update_bits(ac10x->regmap101, reg, mask, value);
	return v;
}



#ifdef CONFIG_AC101_SWITCH_DETECT
#define KEY_HEADSETHOOK         226		/* key define */
#define HEADSET_FILTER_CNT	(10)

/*
 * switch_hw_config:config the 53 codec register
 */
static void switch_hw_config(struct snd_soc_codec *codec)
{
	int r;

	AC101_DBG();

	/*HMIC/MMIC BIAS voltage level select:2.5v*/
	ac101_update_bits(codec, OMIXER_BST1_CTRL, (0xf<<BIASVOLTAGE), (0xf<<BIASVOLTAGE));
	/*debounce when Key down or keyup*/
	ac101_update_bits(codec, HMIC_CTRL1, (0xf<<HMIC_M), (0x0<<HMIC_M));
	/*debounce when earphone plugin or pullout*/
	ac101_update_bits(codec, HMIC_CTRL1, (0xf<<HMIC_N), (0x0<<HMIC_N));
	/*Down Sample Setting Select: Downby 4,32Hz*/
	ac101_update_bits(codec, HMIC_CTRL2, (0x3<<HMIC_SAMPLE_SELECT), (0x02<<HMIC_SAMPLE_SELECT));
	/*Hmic_th2 for detecting Keydown or Keyup.*/
	ac101_update_bits(codec, HMIC_CTRL2, (0x1f<<HMIC_TH2), (0x8<<HMIC_TH2));
	/*Hmic_th1[4:0],detecting eraphone plugin or pullout*/
	ac101_update_bits(codec, HMIC_CTRL2, (0x1f<<HMIC_TH1), (0x1<<HMIC_TH1));
	/*Headset microphone BIAS working mode: when HBIASEN = 1 */
	ac101_update_bits(codec, ADC_APC_CTRL, (0x1<<HBIASMOD), (0x1<<HBIASMOD));
	/*Headset microphone BIAS Enable*/
	ac101_update_bits(codec, ADC_APC_CTRL, (0x1<<HBIASEN), (0x1<<HBIASEN));
	/*Headset microphone BIAS Current sensor & ADC Enable*/
	ac101_update_bits(codec, ADC_APC_CTRL, (0x1<<HBIASADCEN), (0x1<<HBIASADCEN));
	/*Earphone Plugin/out Irq Enable*/
	ac101_update_bits(codec, HMIC_CTRL1, (0x1<<HMIC_PULLOUT_IRQ), (0x1<<HMIC_PULLOUT_IRQ));
	ac101_update_bits(codec, HMIC_CTRL1, (0x1<<HMIC_PLUGIN_IRQ), (0x1<<HMIC_PLUGIN_IRQ));

	/*Hmic KeyUp/key down Irq Enable*/
	ac101_update_bits(codec, HMIC_CTRL1, (0x1<<HMIC_KEYDOWN_IRQ), (0x1<<HMIC_KEYDOWN_IRQ));
	ac101_update_bits(codec, HMIC_CTRL1, (0x1<<HMIC_KEYUP_IRQ), (0x1<<HMIC_KEYUP_IRQ));

	/*headphone calibration clock frequency select*/
	ac101_update_bits(codec, SPKOUT_CTRL, (0x7<<HPCALICKS), (0x7<<HPCALICKS));

	/*clear hmic interrupt */
	r = HMIC_PEND_ALL;
	ac101_write(codec, HMIC_STS, r);

	return;
}

#if 0
/*
 * switch_status_update: update the switch state.
 */
static void switch_status_update(struct ac10x_priv *ac10x)
{
	AC101_DBG("ac10x->state:%d\n", ac10x->state);

        input_report_switch(ac10x->inpdev, SW_HEADPHONE_INSERT, ac10x->state);
        input_sync(ac10x->inpdev);
        return;
}
#endif

#if 0
/*
 * work_cb_clear_irq: clear audiocodec pending and Record the interrupt.
 */
static void work_cb_clear_irq(struct work_struct *work)
{
	int reg_val = 0;
	struct ac10x_priv *ac10x = container_of(work, struct ac10x_priv, work_clear_irq);
	struct snd_soc_codec *codec = ac10x->codec;

	ac10x->irq_cntr++;

	reg_val = ac101_read(codec, HMIC_STS);
	if (BIT(HMIC_PULLOUT_PEND) & reg_val) {
		ac10x->pullout_cntr++;
		AC101_DBG("ac10x->pullout_cntr: %d\n", ac10x->pullout_cntr);
	}

	reg_val |= HMIC_PEND_ALL;
	ac101_write(codec, HMIC_STS, reg_val);

	reg_val = ac101_read(codec, HMIC_STS);
	if ((reg_val & HMIC_PEND_ALL) != 0){
		reg_val |= HMIC_PEND_ALL;
		ac101_write(codec, HMIC_STS, reg_val);
	}

	if (cancel_work_sync(&ac10x->work_switch) != 0) {
		ac10x->irq_cntr--;
	}

	if (0 == schedule_work(&ac10x->work_switch)) {
		ac10x->irq_cntr--;
		AC101_DBG("[work_cb_clear_irq] add work struct failed!\n");
	}
}

enum {
	HBIAS_LEVEL_1 = 0x02,
	HBIAS_LEVEL_2 = 0x0B,
	HBIAS_LEVEL_3 = 0x13,
	HBIAS_LEVEL_4 = 0x17,
	HBIAS_LEVEL_5 = 0x19,
};
#endif

#if 0
static int __ac101_get_hmic_data(struct snd_soc_codec *codec) {
	#ifdef AC101_DEBG
	static long counter;
	#endif
	int r, d;

	d = GET_HMIC_DATA(ac101_read(codec, HMIC_STS));

	r = 0x1 << HMIC_DATA_PEND;
	ac101_write(codec, HMIC_STS, r);

	/* prevent i2c accessing too frequently */
	usleep_range(1500, 3000);

	AC101_DBG("HMIC_DATA(%3ld): %02X\n", counter++, d);
	return d;
}
#endif

#if 0
/*
 * work_cb_earphone_switch: judge the status of the headphone
 */
static void work_cb_earphone_switch(struct work_struct *work)
{
	struct ac10x_priv *ac10x = container_of(work, struct ac10x_priv, work_switch);
	struct snd_soc_codec *codec = ac10x->codec;

	static int hook_flag1 = 0, hook_flag2 = 0;
	static int KEY_VOLUME_FLAG = 0;

	unsigned filter_buf = 0;
	int filt_index = 0;
	int t = 0;

	ac10x->irq_cntr--;

	/* read HMIC_DATA */
	t = __ac101_get_hmic_data(codec);

	if ((t >= HBIAS_LEVEL_2) && (ac10x->mode == FOUR_HEADPHONE_PLUGIN)) {
		t = __ac101_get_hmic_data(codec);

		if (t >= HBIAS_LEVEL_5){
			msleep(150);
			t = __ac101_get_hmic_data(codec);
			if (((t < HBIAS_LEVEL_2 && t >= HBIAS_LEVEL_1 - 1) || t >= HBIAS_LEVEL_5)
			&& (ac10x->pullout_cntr == 0)) {
				input_report_key(ac10x->inpdev, KEY_HEADSETHOOK, 1);
				input_sync(ac10x->inpdev);

				AC101_DBG("KEY_HEADSETHOOK1\n");

				if (hook_flag1 != hook_flag2)
					hook_flag1 = hook_flag2 = 0;
				hook_flag1++;
			}
			if (ac10x->pullout_cntr)
				ac10x->pullout_cntr--;
		} else if (t >= HBIAS_LEVEL_4) {
			msleep(80);
			t = __ac101_get_hmic_data(codec);
			if (t < HBIAS_LEVEL_5 && t >= HBIAS_LEVEL_4 && (ac10x->pullout_cntr == 0)) {
				KEY_VOLUME_FLAG = 1;
				input_report_key(ac10x->inpdev, KEY_VOLUMEUP, 1);
				input_sync(ac10x->inpdev);
				input_report_key(ac10x->inpdev, KEY_VOLUMEUP, 0);
				input_sync(ac10x->inpdev);

				AC101_DBG("HMIC_DATA: %d KEY_VOLUMEUP\n", t);
			}
			if (ac10x->pullout_cntr)
				ac10x->pullout_cntr--;
		} else if (t >= HBIAS_LEVEL_3){
			msleep(80);
			t = __ac101_get_hmic_data(codec);
			if (t < HBIAS_LEVEL_4 && t >= HBIAS_LEVEL_3  && (ac10x->pullout_cntr == 0)){
				KEY_VOLUME_FLAG = 1;
				input_report_key(ac10x->inpdev, KEY_VOLUMEDOWN, 1);
				input_sync(ac10x->inpdev);
				input_report_key(ac10x->inpdev, KEY_VOLUMEDOWN, 0);
				input_sync(ac10x->inpdev);
				AC101_DBG("KEY_VOLUMEDOWN\n");
			}
			if (ac10x->pullout_cntr)
				ac10x->pullout_cntr--;
		}
	} else if ((t < HBIAS_LEVEL_2 && t >= HBIAS_LEVEL_1) && (ac10x->mode == FOUR_HEADPHONE_PLUGIN)) {
		t = __ac101_get_hmic_data(codec);
		if (t < HBIAS_LEVEL_2 && t >= HBIAS_LEVEL_1) {
			if (KEY_VOLUME_FLAG) {
				KEY_VOLUME_FLAG = 0;
			}
			if (hook_flag1 == (++hook_flag2)) {
				hook_flag1 = hook_flag2 = 0;
				input_report_key(ac10x->inpdev, KEY_HEADSETHOOK, 0);
				input_sync(ac10x->inpdev);

				AC101_DBG("KEY_HEADSETHOOK0\n");
			}
		}
	} else {
		while (ac10x->irq_cntr == 0 && ac10x->irq != 0) {
			msleep(20);

			t = __ac101_get_hmic_data(codec);

			if (filt_index <= HEADSET_FILTER_CNT) {
				if (filt_index++ == 0) {
					filter_buf = t;
				} else if (filter_buf != t) {
					filt_index = 0;
				}
				continue;
			}

			filt_index = 0;
			if (filter_buf >= HBIAS_LEVEL_2) {
				ac10x->mode = THREE_HEADPHONE_PLUGIN;
				ac10x->state = 2;
			} else if (filter_buf >= HBIAS_LEVEL_1 - 1) {
				ac10x->mode = FOUR_HEADPHONE_PLUGIN;
				ac10x->state = 1;
			} else {
				ac10x->mode = HEADPHONE_IDLE;
				ac10x->state = 0;
			}
			switch_status_update(ac10x);
			ac10x->pullout_cntr = 0;
			break;
		}
	}
}
#endif

#if 0
/*
 * audio_hmic_irq:  the interrupt handlers
 */
static irqreturn_t audio_hmic_irq(int irq, void *para)
{
	struct ac10x_priv *ac10x = (struct ac10x_priv *)para;
	if (ac10x == NULL) {
		return -EINVAL;
	}

	if (0 == schedule_work(&ac10x->work_clear_irq)){
		AC101_DBG("[audio_hmic_irq] work already in queue_codec_irq, adding failed!\n");
	}
	return IRQ_HANDLED;
}
#endif

#if 0
static int ac101_switch_probe(struct ac10x_priv *ac10x) {
	struct i2c_client *i2c = ac10x->i2c101;
	long ret;
	int gpiod_irq = 0;
	gpiod_irq = devm_gpio_request(&i2c->dev, 72, "switch-irq");
	if (IS_ERR(ac10x->gpiod_irq)) {
		ac10x->gpiod_irq = NULL;
		dev_err(&i2c->dev, "failed get switch-irq in device tree\n");
		goto _err_irq;
	}

	ac10x->gpiod_irq = gpio_to_desc(gpiod_irq);

	gpiod_direction_input(ac10x->gpiod_irq);

	ac10x->irq = gpiod_to_irq(ac10x->gpiod_irq);
	if (IS_ERR_VALUE(ac10x->irq)) {
		pr_info("[ac101] map gpio to irq failed, errno = %ld\n", ac10x->irq);
		ac10x->irq = 0;
		goto _err_irq;
	}

	/* request irq, set irq type to falling edge trigger */
	ret = devm_request_irq(ac10x->codec->dev, ac10x->irq, audio_hmic_irq, IRQF_TRIGGER_FALLING, "SWTICH_EINT", ac10x);
	if (IS_ERR_VALUE(ret)) {
		pr_info("[ac101] request virq %ld failed, errno = %ld\n", ac10x->irq, ret);
		goto _err_irq;
	}

	ac10x->mode = HEADPHONE_IDLE;
	ac10x->state = -1;

	/*use for judge the state of switch*/
	INIT_WORK(&ac10x->work_switch, work_cb_earphone_switch);
	INIT_WORK(&ac10x->work_clear_irq, work_cb_clear_irq);

	/********************create input device************************/
	ac10x->inpdev = devm_input_allocate_device(ac10x->codec->dev);
	if (!ac10x->inpdev) {
		AC101_DBG("input_allocate_device: not enough memory for input device\n");
		ret = -ENOMEM;
		goto _err_input_allocate_device;
	}

	ac10x->inpdev->name          = "seed-voicecard-headset";
	ac10x->inpdev->phys          = dev_name(ac10x->codec->dev);
	ac10x->inpdev->id.bustype    = BUS_I2C;
	ac10x->inpdev->dev.parent    = ac10x->codec->dev;
	input_set_drvdata(ac10x->inpdev, ac10x->codec);

	ac10x->inpdev->evbit[0] = BIT_MASK(EV_KEY) | BIT(EV_SW);

	set_bit(KEY_HEADSETHOOK, ac10x->inpdev->keybit);
	set_bit(KEY_VOLUMEUP,    ac10x->inpdev->keybit);
	set_bit(KEY_VOLUMEDOWN,  ac10x->inpdev->keybit);
	input_set_capability(ac10x->inpdev, EV_SW, SW_HEADPHONE_INSERT);

	ret = input_register_device(ac10x->inpdev);
	if (ret) {
		AC101_DBG("input_register_device: input_register_device failed\n");
		goto _err_input_register_device;
	}

	/* the first headset state checking */
	switch_hw_config(ac10x->codec);
	ac10x->irq_cntr = 1;
	schedule_work(&ac10x->work_switch);

	return 0;

_err_input_register_device:
_err_input_allocate_device:

	if (ac10x->irq) {
		devm_free_irq(&i2c->dev, ac10x->irq, ac10x);
		ac10x->irq = 0;
	}
_err_irq:
	return ret;
}
#endif
#endif

void drc_config(struct snd_soc_codec *codec)
{
	int reg_val;
	reg_val = ac101_read(codec, 0xa3);
	reg_val &= ~(0x7ff<<0);
	reg_val |= 1<<0;
	ac101_write(codec, 0xa3, reg_val);
	ac101_write(codec, 0xa4, 0x2baf);

	reg_val = ac101_read(codec, 0xa5);
	reg_val &= ~(0x7ff<<0);
	reg_val |= 1<<0;
	ac101_write(codec, 0xa5, reg_val);
	ac101_write(codec, 0xa6, 0x2baf);

	reg_val = ac101_read(codec, 0xa7);
	reg_val &= ~(0x7ff<<0);
	ac101_write(codec, 0xa7, reg_val);
	ac101_write(codec, 0xa8, 0x44a);

	reg_val = ac101_read(codec, 0xa9);
	reg_val &= ~(0x7ff<<0);
	ac101_write(codec, 0xa9, reg_val);
	ac101_write(codec, 0xaa, 0x1e06);

	reg_val = ac101_read(codec, 0xab);
	reg_val &= ~(0x7ff<<0);
	reg_val |= (0x352<<0);
	ac101_write(codec, 0xab, reg_val);
	ac101_write(codec, 0xac, 0x6910);

	reg_val = ac101_read(codec, 0xad);
	reg_val &= ~(0x7ff<<0);
	reg_val |= (0x77a<<0);
	ac101_write(codec, 0xad, reg_val);
	ac101_write(codec, 0xae, 0xaaaa);

	reg_val = ac101_read(codec, 0xaf);
	reg_val &= ~(0x7ff<<0);
	reg_val |= (0x2de<<0);
	ac101_write(codec, 0xaf, reg_val);
	ac101_write(codec, 0xb0, 0xc982);

	ac101_write(codec, 0x16, 0x9f9f);

}

void drc_enable(struct snd_soc_codec *codec,bool on)
{
	int reg_val;
	if (on) {
		ac101_write(codec, 0xb5, 0xA080);
		reg_val = ac101_read(codec, MOD_CLK_ENA);
		reg_val |= (0x1<<6);
		ac101_write(codec, MOD_CLK_ENA, reg_val);
		reg_val = ac101_read(codec, MOD_RST_CTRL);
		reg_val |= (0x1<<6);
		ac101_write(codec, MOD_RST_CTRL, reg_val);

		reg_val = ac101_read(codec, 0xa0);
		reg_val |= (0x7<<0);
		ac101_write(codec, 0xa0, reg_val);
	} else {
		ac101_write(codec, 0xb5, 0x0);
		reg_val = ac101_read(codec, MOD_CLK_ENA);
		reg_val &= ~(0x1<<6);
		ac101_write(codec, MOD_CLK_ENA, reg_val);
		reg_val = ac101_read(codec, MOD_RST_CTRL);
		reg_val &= ~(0x1<<6);
		ac101_write(codec, MOD_RST_CTRL, reg_val);

		reg_val = ac101_read(codec, 0xa0);
		reg_val &= ~(0x7<<0);
		ac101_write(codec, 0xa0, reg_val);
	}
}

void set_configuration(struct snd_soc_codec *codec)
{
	if (speaker_double_used) {
		ac101_update_bits(codec, SPKOUT_CTRL, (0x1f<<SPK_VOL), (double_speaker_val<<SPK_VOL));
	} else {
		ac101_update_bits(codec, SPKOUT_CTRL, (0x1f<<SPK_VOL), (single_speaker_val<<SPK_VOL));
	}
	ac101_update_bits(codec, HPOUT_CTRL, (0x3f<<HP_VOL), (headset_val<<HP_VOL));
	ac101_update_bits(codec, ADC_SRCBST_CTRL, (0x7<<ADC_MIC1G), (mainmic_val<<ADC_MIC1G));
	ac101_update_bits(codec, ADC_SRCBST_CTRL, (0x7<<ADC_MIC2G), (headsetmic_val<<ADC_MIC2G));
	if (dmic_used) {
		ac101_write(codec, ADC_VOL_CTRL, adc_digital_val);
	}
	if (drc_used) {
		drc_config(codec);
	}
	/*headphone calibration clock frequency select*/
	ac101_update_bits(codec, SPKOUT_CTRL, (0x7<<HPCALICKS), (0x7<<HPCALICKS));

	/* I2S1 DAC Timeslot 0 data <- I2S1 DAC channel 0 */
	// "AIF1IN0L Mux" <= "AIF1DACL"
	// "AIF1IN0R Mux" <= "AIF1DACR"
	ac101_update_bits(codec, AIF1_DACDAT_CTRL, 0x3 << AIF1_DA0L_SRC, 0x0 << AIF1_DA0L_SRC);
	ac101_update_bits(codec, AIF1_DACDAT_CTRL, 0x3 << AIF1_DA0R_SRC, 0x0 << AIF1_DA0R_SRC);
	/* Timeslot 0 Left & Right Channel enable */
	ac101_update_bits(codec, AIF1_DACDAT_CTRL, 0x3 << AIF1_DA0R_ENA, 0x3 << AIF1_DA0R_ENA);

	/* DAC Digital Mixer Source Select <- I2S1 DA0 */
	// "DACL Mixer"	+= "AIF1IN0L Mux"
	// "DACR Mixer" += "AIF1IN0R Mux"
	ac101_update_bits(codec, DAC_MXR_SRC, 0xF << DACL_MXR_ADCL, 0x8 << DACL_MXR_ADCL);
	ac101_update_bits(codec, DAC_MXR_SRC, 0xF << DACR_MXR_ADCR, 0x8 << DACR_MXR_ADCR);
	/* Internal DAC Analog Left & Right Channel enable */
	ac101_update_bits(codec, OMIXER_DACA_CTRL, 0x3 << DACALEN, 0x3 << DACALEN);

	/* Output Mixer Source Select */
	// "Left Output Mixer"  += "DACL Mixer"
	// "Right Output Mixer" += "DACR Mixer"
	ac101_update_bits(codec, OMIXER_SR, 0x1 << LMIXMUTEDACL, 0x1 << LMIXMUTEDACL);
	ac101_update_bits(codec, OMIXER_SR, 0x1 << RMIXMUTEDACR, 0x1 << RMIXMUTEDACR);
	/* Left & Right Analog Output Mixer enable */
	ac101_update_bits(codec, OMIXER_DACA_CTRL, 0x3 << LMIXEN, 0x3 << LMIXEN);

	/* Headphone Ouput Control */
	// "HP_R Mux" <= "DACR Mixer"
	// "HP_L Mux" <= "DACL Mixer"
	ac101_update_bits(codec, HPOUT_CTRL, 0x1 << LHPS, 0x0 << LHPS);
	ac101_update_bits(codec, HPOUT_CTRL, 0x1 << RHPS, 0x0 << RHPS);

	/* Speaker Output Control */
	// "SPK_L Mux" <= "SPK_LR Adder"
	// "SPK_R Mux" <= "SPK_LR Adder"
	ac101_update_bits(codec, SPKOUT_CTRL, (0x1 << LSPKS) | (0x1 << RSPKS), (0x1 << LSPKS) | (0x1 << RSPKS));
	/* Enable Left & Right Speaker */
	ac101_update_bits(codec, SPKOUT_CTRL, (0x1 << LSPK_EN) | (0x1 << RSPK_EN), (0x1 << LSPK_EN) | (0x1 << RSPK_EN));
	return;
}

static int late_enable_dac(struct snd_soc_codec* codec, int event) {
	//struct ac10x_priv *ac10x = snd_soc_codec_get_drvdata(codec);
	struct ac10x_priv *ac10x = static_ac10x;

	mutex_lock(&ac10x->dac_mutex);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		AC101_DBG();
		if (ac10x->dac_enable == 0){
			/*enable dac module clk*/
			ac101_update_bits(codec, MOD_CLK_ENA, (0x1<<MOD_CLK_DAC_DIG), (0x1<<MOD_CLK_DAC_DIG));
			ac101_update_bits(codec, MOD_RST_CTRL, (0x1<<MOD_RESET_DAC_DIG), (0x1<<MOD_RESET_DAC_DIG));
			ac101_update_bits(codec, DAC_DIG_CTRL, (0x1<<ENDA), (0x1<<ENDA));
			ac101_update_bits(codec, DAC_DIG_CTRL, (0x1<<ENHPF),(0x1<<ENHPF));
		}
		ac10x->dac_enable++;
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (ac10x->dac_enable != 0){
			ac10x->dac_enable = 0;

			ac101_update_bits(codec, DAC_DIG_CTRL, (0x1<<ENHPF),(0x0<<ENHPF));
			ac101_update_bits(codec, DAC_DIG_CTRL, (0x1<<ENDA), (0x0<<ENDA));
			/*disable dac module clk*/
			ac101_update_bits(codec, MOD_CLK_ENA, (0x1<<MOD_CLK_DAC_DIG), (0x0<<MOD_CLK_DAC_DIG));
			ac101_update_bits(codec, MOD_RST_CTRL, (0x1<<MOD_RESET_DAC_DIG), (0x0<<MOD_RESET_DAC_DIG));
		}
		break;
	}
	mutex_unlock(&ac10x->dac_mutex);
	return 0;
}

static int ac101_headphone_event(struct snd_soc_codec* codec, int event) {
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/*open*/
		AC101_DBG("post:open\n");
		ac101_update_bits(codec, OMIXER_DACA_CTRL, (0xf<<HPOUTPUTENABLE), (0xf<<HPOUTPUTENABLE));
		msleep(10);
		ac101_update_bits(codec, HPOUT_CTRL, (0x1<<HPPA_EN), (0x1<<HPPA_EN));
		ac101_update_bits(codec, HPOUT_CTRL, (0x3<<LHPPA_MUTE), (0x3<<LHPPA_MUTE));
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/*close*/
		AC101_DBG("pre:close\n");
		ac101_update_bits(codec, HPOUT_CTRL, (0x3<<LHPPA_MUTE), (0x0<<LHPPA_MUTE));
		msleep(10);
		ac101_update_bits(codec, OMIXER_DACA_CTRL, (0xf<<HPOUTPUTENABLE), (0x0<<HPOUTPUTENABLE));
		ac101_update_bits(codec, HPOUT_CTRL, (0x1<<HPPA_EN), (0x0<<HPPA_EN));
		break;
	}
	return 0;
}

static int ac101_sysclk_started(void) {
	int reg_val;

	reg_val = ac101_read(static_ac10x->codec, SYSCLK_CTRL);
	return (reg_val & (0x1<<SYSCLK_ENA));
}

static int ac101_aif1clk(struct snd_soc_codec* codec, int event, int quick) {
	struct ac10x_priv *ac10x = static_ac10x;
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (ac10x->aif1_clken == 0) {
			ret = ac101_update_bits(codec, MOD_CLK_ENA, (0x1 << MOD_CLK_AIF1),
				(0x1 << MOD_CLK_AIF1));
			ret = ret || ac101_update_bits(codec, MOD_RST_CTRL, (0x1 << MOD_RESET_AIF1),
					(0x1 << MOD_RESET_AIF1));
			ret = ret || ac101_update_bits(codec, SYSCLK_CTRL, (0x1 << AIF1CLK_ENA),
					(0x1 << AIF1CLK_ENA));
			ret = ret || ac101_update_bits(codec, SYSCLK_CTRL, (0x1 << SYSCLK_ENA),
					(0x1 << SYSCLK_ENA));

			if (ret) {
				printk("start sysclk failed\n");
			} else {
				printk("hw sysclk enable\n");
				ac10x->aif1_clken++;
			}
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (ac10x->aif1_clken != 0) {
			/* disable aif1clk & sysclk */
			ret = ac101_update_bits(codec, SYSCLK_CTRL, (0x1<<AIF1CLK_ENA),(0x0<<AIF1CLK_ENA));
			ret = ret || ac101_update_bits(codec, MOD_CLK_ENA, (0x1<<MOD_CLK_AIF1), (0x0<<MOD_CLK_AIF1));
			ret = ret || ac101_update_bits(codec, MOD_RST_CTRL, (0x1<<MOD_RESET_AIF1), (0x0<<MOD_RESET_AIF1));
			ret = ret || ac101_update_bits(codec, SYSCLK_CTRL, (0x1<<SYSCLK_ENA), (0x0<<SYSCLK_ENA));
			ret = ret || ac101_write(codec, PLL_CTRL1, 0x0141);
			ret = ret || ac101_write(codec, PLL_CTRL2, 0x0000);
			ret = ret || ac101_write(codec, SYSCLK_CTRL, 0x0000);

			if (ret) {
				printk("stop sysclk failed\n");
			} else {
				printk("hw sysclk disable\n");
				ac10x->aif1_clken = 0;
			}
			break;
		}
	}

	AC101_DBG("event=%d pre_up/%d post_down/%d\n", event, SND_SOC_DAPM_PRE_PMU, SND_SOC_DAPM_POST_PMD);

	return ret;
}

/**
 * snd_ac101_get_volsw - single mixer get callback
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Callback to get the value of a single mixer control, or a double mixer
 * control that spans 2 registers.
 *
 * Returns 0 for success.
 */
static int snd_ac101_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol
){
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val, mask = (1 << fls(mc->max)) - 1;
	unsigned int invert = mc->invert;
	int ret;

	if ((ret = ac101_read(static_ac10x->codec, mc->reg)) < 0)
		return ret;

	val = (ret >> mc->shift) & mask;
	ucontrol->value.integer.value[0] = val - mc->min;
	if (invert) {
		ucontrol->value.integer.value[0] =
			mc->max - ucontrol->value.integer.value[0];
	}

	if (snd_soc_volsw_is_stereo(mc)) {
		val = (ret >> mc->rshift) & mask;
		ucontrol->value.integer.value[1] = val - mc->min;
		if (invert) {
			ucontrol->value.integer.value[1] =
				mc->max - ucontrol->value.integer.value[1];
		}
	}
	return 0;
}

/**
 * snd_ac101_put_volsw - single mixer put callback
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Callback to set the value of a single mixer control, or a double mixer
 * control that spans 2 registers.
 *
 * Returns 0 for success.
 */
static int snd_ac101_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol
){
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int sign_bit = mc->sign_bit;
	unsigned int val, mask = (1 << fls(mc->max)) - 1;
	unsigned int invert = mc->invert;
	int ret;

	if (sign_bit)
		mask = (uint32_t)(BIT(sign_bit + 1) - 1);

	val = ((ucontrol->value.enumerated.item[0] + mc->min) & mask);
	if (invert) {
		val = mc->max - val;
	}

	ret = ac101_update_bits(static_ac10x->codec, mc->reg, mask << mc->shift, val << mc->shift);

	if (! snd_soc_volsw_is_stereo(mc)) {
		return ret;
	}
	val = ((ucontrol->value.enumerated.item[1] + mc->min) & mask);
	if (invert) {
		val = mc->max - val;
	}

	ret = ac101_update_bits(static_ac10x->codec, mc->reg, mask << mc->rshift, val << mc->rshift);
	return ret;
}


static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -11925, 75, 0);
static const DECLARE_TLV_DB_SCALE(dac_mix_vol_tlv, -600, 600, 0);
static const DECLARE_TLV_DB_SCALE(dig_vol_tlv, -7308, 116, 0);
static const DECLARE_TLV_DB_SCALE(speaker_vol_tlv, -4800, 150, 0);
static const DECLARE_TLV_DB_SCALE(headphone_vol_tlv, -6300, 100, 0);

static struct snd_kcontrol_new ac101_controls[] = {
	/*DAC*/
	SOC_DOUBLE_TLV("DAC volume", DAC_VOL_CTRL, DAC_VOL_L, DAC_VOL_R, 0xff, 0, dac_vol_tlv),
	SOC_DOUBLE_TLV("DAC mixer gain", DAC_MXR_GAIN, DACL_MXR_GAIN, DACR_MXR_GAIN, 0xf, 0, dac_mix_vol_tlv),
	SOC_SINGLE_TLV("digital volume", DAC_DBG_CTRL, DVC, 0x3f, 1, dig_vol_tlv),
	SOC_SINGLE_TLV("speaker volume", SPKOUT_CTRL, SPK_VOL, 0x1f, 0, speaker_vol_tlv),
	SOC_SINGLE_TLV("headphone volume", HPOUT_CTRL, HP_VOL, 0x3f, 0, headphone_vol_tlv),
};

/* PLL divisors */
struct pll_div {
	unsigned int pll_in;
	unsigned int pll_out;
	int m;
	int n_i;
	int n_f;
};

struct aif1_fs {
	unsigned samp_rate;
	int bclk_div;
	int srbit;
	#define _SERIES_24_576K		0
	#define _SERIES_22_579K		1
	int series;
};

struct kv_map {
	int val;
	int bit;
};

/*
 * Note : pll code from original tdm/i2s driver.
 * freq_out = freq_in * N/(M*(2k+1)) , k=1,N=N_i+N_f,N_f=factor*0.2;
 * 		N_i[0,1023], N_f_factor[0,7], m[1,64]=REG_VAL[1-63,0]
 */
static const struct pll_div codec_pll_div[] = {
	{128000,   _FREQ_22_579K,  1, 529, 1},
	{192000,   _FREQ_22_579K,  1, 352, 4},
	{256000,   _FREQ_22_579K,  1, 264, 3},
	{384000,   _FREQ_22_579K,  1, 176, 2}, /*((176+2*0.2)*6000000)/(38*(2*1+1))*/
	{1411200,  _FREQ_22_579K,  1,  48, 0},
	{2822400,  _FREQ_22_579K,  1,  24, 0}, /* accurate, 11025 * 256 */
	{5644800,  _FREQ_22_579K,  1,  12, 0}, /* accurate, 22050 * 256 */
	{6000000,  _FREQ_22_579K, 38, 429, 0}, /*((429+0*0.2)*6000000)/(38*(2*1+1))*/
	{11289600, _FREQ_22_579K,  1,   6, 0}, /* accurate, 44100 * 256 */
	{13000000, _FREQ_22_579K, 19,  99, 0},
	{19200000, _FREQ_22_579K, 25,  88, 1},
	{24000000, _FREQ_22_579K, 63, 177, 4}, /* 22577778 Hz */

	{128000,   _FREQ_24_576K,  1, 576, 0},
	{192000,   _FREQ_24_576K,  1, 384, 0},
	{256000,   _FREQ_24_576K,  1, 288, 0},
	{384000,   _FREQ_24_576K,  1, 192, 0},
	{2048000,  _FREQ_24_576K,  1,  36, 0}, /* accurate,  8000 * 256 */
	{3072000,  _FREQ_24_576K,  1,  24, 0}, /* accurate, 12000 * 256 */
	{4096000,  _FREQ_24_576K,  1,  18, 0}, /* accurate, 16000 * 256 */
	{6000000,  _FREQ_24_576K, 25, 307, 1},
	{6144000,  _FREQ_24_576K,  4,  48, 0}, /* accurate, 24000 * 256 */
	{12288000, _FREQ_24_576K,  8,  48, 0}, /* accurate, 48000 * 256 */
	{13000000, _FREQ_24_576K, 42, 238, 1},
	{19200000, _FREQ_24_576K, 25,  96, 0},
	{24000000, _FREQ_24_576K, 25,  76, 4}, /* accurate */

	{_FREQ_22_579K, _FREQ_22_579K,  8,  24, 0}, /* accurate, 88200 * 256 */
	{_FREQ_24_576K, _FREQ_24_576K,  8,  24, 0}, /* accurate, 96000 * 256 */
};

static const struct aif1_fs codec_aif1_fs[] = {
	{8000, 12, 0},
	{11025, 8, 1, _SERIES_22_579K},
	{12000, 8, 2},
	{16000, 6, 3},
	{22050, 4, 4, _SERIES_22_579K},
	{24000, 4, 5},
	/* {32000, 3, 6}, dividing by 3 is not support */
	{44100, 2, 7, _SERIES_22_579K},
	{48000, 2, 8},
	{96000, 1, 9},
};

static const struct kv_map codec_aif1_lrck[] = {
	{16, 0},
	{32, 1},
	{64, 2},
	{128, 3},
	{256, 4},
};

static const struct kv_map codec_aif1_wsize[] = {
	{8, 0},
	{16, 1},
	{20, 2},
	{24, 3},
	{32, 3},
};

static const unsigned ac101_bclkdivs[] = {
	  1,   2,   4,   6,
	  8,  12,  16,  24,
	 32,  48,  64,  96,
	128, 192,   0,   0,
};

static int ac101_aif_play(struct ac10x_priv* ac10x) {
	struct snd_soc_codec * codec = ac10x->codec;

	late_enable_dac(codec, SND_SOC_DAPM_PRE_PMU);
	ac101_headphone_event(codec, SND_SOC_DAPM_POST_PMU);
	if (drc_used) {
		drc_enable(codec, true);
	}

	/* Enable Left & Right Speaker */
	ac101_update_bits(codec, SPKOUT_CTRL, (0x1 << LSPK_EN) | (0x1 << RSPK_EN), (0x1 << LSPK_EN) | (0x1 << RSPK_EN));
	if (ac10x->gpiod_spk_amp_gate) {
		gpiod_set_value(ac10x->gpiod_spk_amp_gate, 1);
	}
	return 0;
}

static void ac10x_work_aif_play(struct work_struct *work) {
	struct ac10x_priv *ac10x = container_of(work, struct ac10x_priv, dlywork.work);

	ac101_aif_play(ac10x);
	return;
}

int ac101_aif_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	ac101_write(codec, DAC_VOL_CTRL, mute? 0: 0xA0A0);
#if 0
	if (!mute) {
		#if _MASTER_MULTI_CODEC != _MASTER_AC101
		/* enable global clock */
		ac10x->aif1_clken = 0;
		ac101_aif1clk(codec, SND_SOC_DAPM_PRE_PMU, 0);
		ac101_aif_play(ac10x);
		#else
		schedule_delayed_work(&ac10x->dlywork, msecs_to_jiffies(50));
		#endif
	}
	else {
		#if _MASTER_MULTI_CODEC == _MASTER_AC101
		cancel_delayed_work_sync(&ac10x->dlywork);
		#endif

		if (ac10x->gpiod_spk_amp_gate) {
			gpiod_set_value(ac10x->gpiod_spk_amp_gate, 0);
		}
		/* Disable Left & Right Speaker */
		ac101_update_bits(codec, SPKOUT_CTRL, (0x1 << LSPK_EN) | (0x1 << RSPK_EN), (0x0 << LSPK_EN) | (0x0 << RSPK_EN));
		if (drc_used) {
			drc_enable(codec, 0);
		}
		ac101_headphone_event(codec, SND_SOC_DAPM_PRE_PMD);
		late_enable_dac(codec, SND_SOC_DAPM_POST_PMD);

		#if _MASTER_MULTI_CODEC != _MASTER_AC101
		ac10x->aif1_clken = 1;
		ac101_aif1clk(codec, SND_SOC_DAPM_POST_PMD, 0);
		#endif
	}
#endif
	return 0;
}

void ac101_aif_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ac10x_priv *ac10x = static_ac10x;

	AC101_DBG("stream = %s, play: %d, capt: %d, active: %d\n",
		snd_pcm_stream_str(substream),
		codec_dai->playback_active, codec_dai->capture_active,
		codec_dai->active);

	if (!codec_dai->active) {
		ac10x->aif1_clken = 1;
		ac101_aif1clk(codec, SND_SOC_DAPM_POST_PMD, 0);
	} else {
		ac101_aif1clk(codec, SND_SOC_DAPM_PRE_PMU, 0);
	}
}

static int ac101_set_pll(struct snd_soc_dai *codec_dai, int pll_id, int source,
			unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	int i, m, n_i, n_f;

	/* clear volatile reserved bits*/
	ac101_update_bits(codec, SYSCLK_CTRL, 0xFF & ~(0x1 << SYSCLK_ENA), 0x0);

	/* select aif1 clk srouce from mclk1 */
	ac101_update_bits(codec, SYSCLK_CTRL, (0x3<<AIF1CLK_SRC), (0x0<<AIF1CLK_SRC));
	/* disable pll */
	ac101_update_bits(codec, PLL_CTRL2, (0x1<<PLL_EN), (0<<PLL_EN));

	if (!freq_out)
		return 0;
	if ((freq_in < 128000) || (freq_in > _FREQ_24_576K)) {
		return -EINVAL;
	} else if ((freq_in == _FREQ_24_576K) || (freq_in == _FREQ_22_579K)) {
		if (pll_id == AC101_MCLK1) {
			/*select aif1 clk source from mclk1*/
			ac101_update_bits(codec, SYSCLK_CTRL, (0x3<<AIF1CLK_SRC), (0x0<<AIF1CLK_SRC));
			return 0;
		}
	}

	switch (pll_id) {
	case AC101_MCLK1:
		/*pll source from MCLK1*/
		ac101_update_bits(codec, SYSCLK_CTRL, (0x3<<PLLCLK_SRC), (0x0<<PLLCLK_SRC));
		break;
	case AC101_BCLK1:
		/*pll source from BCLK1*/
		ac101_update_bits(codec, SYSCLK_CTRL, (0x3<<PLLCLK_SRC), (0x2<<PLLCLK_SRC));
		break;
	default:
		return -EINVAL;
	}

	/* freq_out = freq_in * n/(m*(2k+1)) , k=1,N=N_i+N_f */
	for (i = m = n_i = n_f = 0; i < ARRAY_SIZE(codec_pll_div); i++) {
		if ((codec_pll_div[i].pll_in == freq_in) && (codec_pll_div[i].pll_out == freq_out)) {
			m   = codec_pll_div[i].m;
			n_i = codec_pll_div[i].n_i;
			n_f = codec_pll_div[i].n_f;
			break;
		}
	}
	/* config pll m */
	if (m  == 64) m = 0;
	ac101_update_bits(codec, PLL_CTRL1, (0x3f<<PLL_POSTDIV_M), (m<<PLL_POSTDIV_M));
	/* config pll n */
	ac101_update_bits(codec, PLL_CTRL2, (0x3ff<<PLL_PREDIV_NI), (n_i<<PLL_PREDIV_NI));
	ac101_update_bits(codec, PLL_CTRL2, (0x7<<PLL_POSTDIV_NF), (n_f<<PLL_POSTDIV_NF));
	/* enable pll */
	ac101_update_bits(codec, PLL_CTRL2, (0x1<<PLL_EN), (1<<PLL_EN));
	ac101_update_bits(codec, SYSCLK_CTRL, (0x1<<PLLCLK_ENA),  (0x1<<PLLCLK_ENA));
	ac101_update_bits(codec, SYSCLK_CTRL, (0x3<<AIF1CLK_SRC), (0x3<<AIF1CLK_SRC));

	return 0;
}

int ac101_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *codec_dai)
{
	int i = 0;
	int AIF_CLK_CTRL = AIF1_CLK_CTRL;
	int aif1_word_size = 24;
	int aif1_slot_size = 32;
	int aif1_lrck_div = 64;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ac10x_priv *ac10x = static_ac10x;
	int freq_out;
	unsigned channels;
	unsigned bclkdiv;

	if (_MASTER_MULTI_CODEC == _MASTER_AC101 && ac101_sysclk_started()) {
		/* not configure hw_param twice if stream is playback, tell the caller it's started */
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			printk("pre return");
			return 1;
		}
	}

	/* get channels count & slot size */
	channels = params_channels(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		aif1_slot_size = 8;
		aif1_word_size = 8;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		aif1_slot_size = 32;
		aif1_word_size = 32;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
	default:
		aif1_slot_size = 16;
		aif1_word_size = 16;
		break;
	}

	/* set LRCK/BCLK ratio */
	aif1_lrck_div = aif1_slot_size * channels;
	if (aif1_lrck_div < 32)
		aif1_lrck_div = 32;
	#if 0
	switch(params_rate(params)) {
	case 16000:
	case 22050:
		aif1_lrck_div = 128;
		break;
	case 8000:
	case 32000:
	case 44100:
		aif1_lrck_div = 64;
		break;
	case 48000:
		aif1_lrck_div = 32;
		break;
	}
	#endif
	for (i = 0; i < ARRAY_SIZE(codec_aif1_lrck); i++) {
		if (codec_aif1_lrck[i].val == aif1_lrck_div) {
			ac101_update_bits(codec, AIF_CLK_CTRL,
				(0x7 << AIF1_LRCK_DIV),
				codec_aif1_lrck[i].bit << AIF1_LRCK_DIV);
			break;
		}
	}

	/* set PLL output freq */
	freq_out = _FREQ_24_576K;
	for (i = 0; i < ARRAY_SIZE(codec_aif1_fs); i++) {
		if (codec_aif1_fs[i].samp_rate == params_rate(params)) {
			if (codec_dai->capture_active && dmic_used && codec_aif1_fs[i].samp_rate == 44100) {
				ac101_update_bits(codec, AIF_SR_CTRL, (0xf<<AIF1_FS), (0x4<<AIF1_FS));
			} else {
				ac101_update_bits(codec, AIF_SR_CTRL, (0xf<<AIF1_FS), ((codec_aif1_fs[i].srbit)<<AIF1_FS));
			}
			if (codec_aif1_fs[i].series == _SERIES_22_579K)
				freq_out = _FREQ_22_579K;
			break;
		}
	}

	/* set I2S word size */
	for (i = 0; i < ARRAY_SIZE(codec_aif1_wsize); i++) {
		if (codec_aif1_wsize[i].val == aif1_word_size) {
			ac101_update_bits(codec, AIF_CLK_CTRL,
				(0x3 << AIF1_WORK_SIZ),
				((codec_aif1_wsize[i].bit) << AIF1_WORK_SIZ));
			break;
		}
	}

	/* set TDM slot size */
	/*
	if ((reg_val = codec_aif1_wsize[i].bit) > 2) reg_val = 2;
	ac101_update_bits(codec, AIF1_ADCDAT_CTRL, 0x3 << AIF1_SLOT_SIZ, reg_val << AIF1_SLOT_SIZ);
	*/

	/* setting pll if it's master mode */
	//reg_val = ac101_read(codec, AIF_CLK_CTRL);
	//if ((reg_val & (0x1 << AIF1_MSTR_MOD)) == 0) {
	ac10x->sysclk = params_rate(params)*aif1_lrck_div*AC101_BCLK_DIV;
	if (params_rate(params) == 32000)
		ac10x->sysclk = params_rate(params)*aif1_lrck_div*(AC101_BCLK_DIV - 2);
	if (params_rate(params) != 48000) {
		ac101_set_pll(codec_dai, ac10x->clk_id, 0, ac10x->sysclk, freq_out);
	}
	bclkdiv = freq_out / (aif1_lrck_div * params_rate(params));
	for (i = 0; i < ARRAY_SIZE(ac101_bclkdivs) - 1; i++) {
		if (ac101_bclkdivs[i] >= bclkdiv) {
			break;
		}
	}
	ac101_update_bits(codec, AIF_CLK_CTRL, (0xf<<AIF1_BCLK_DIV), i<<AIF1_BCLK_DIV);

	/*
	#if _MASTER_MULTI_CODEC == _MASTER_AC101
	//Master mode, to clear cpu_dai fifos, disable output bclk & lrck
	ac101_aif1clk(codec, SND_SOC_DAPM_POST_PMD, 0);
	#endif
	*/

	/* I2S1 enable */
	ac101_update_bits(codec, MOD_CLK_ENA, (0x1<<MOD_CLK_AIF1), (0x1<<MOD_CLK_AIF1));
        ac101_update_bits(codec, MOD_RST_CTRL, (0x1<<MOD_RESET_AIF1), (0x1<<MOD_RESET_AIF1));
	msleep(10);

	AC101_DBG("rate: %d , channels: %d , samp_res: %d",
		params_rate(params), channels, aif1_slot_size);

	return 0;
}

int ac101_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	int reg_val;
	int AIF_CLK_CTRL = AIF1_CLK_CTRL;
	struct snd_soc_codec *codec = codec_dai->codec;

	AC101_DBG();
	/*
	 * 	master or slave selection
	 *	0 = Master mode
	 *	1 = Slave mode
	 */
	reg_val = ac101_read(codec, AIF_CLK_CTRL);
	reg_val &= ~(0x1<<AIF1_MSTR_MOD);
	switch(fmt & SND_SOC_DAIFMT_MASTER_MASK) {
#ifdef CONFIG_HOBOT_SOC
	case SND_SOC_DAIFMT_CBS_CFS:
		pr_warn("AC101 as Slave\n");
                reg_val |= (0x1<<AIF1_MSTR_MOD);
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		pr_warn("AC101 as Slave\n");
                reg_val |= (0x1<<AIF1_MSTR_MOD);
                break;
#else
	case SND_SOC_DAIFMT_CBM_CFM:   /* codec clk & frm master, ap is slave*/
		#if _MASTER_MULTI_CODEC == _MASTER_AC101
		pr_warn("AC101 as Master\n");
		reg_val |= (0x0<<AIF1_MSTR_MOD);
		break;
		#else
		pr_warn("AC108 as Master\n");
		#endif
	case SND_SOC_DAIFMT_CBS_CFS:   /* codec clk & frm slave, ap is master*/
		pr_warn("AC101 as Slave\n");
		reg_val |= (0x1<<AIF1_MSTR_MOD);
		break;
#endif
	default:
		pr_err("unknwon master/slave format\n");
		return -EINVAL;
	}

	/*
	 * Enable TDM mode
	 */
	//reg_val |=  (0x1 << AIF1_TDMM_ENA);
	ac101_write(codec, AIF_CLK_CTRL, reg_val);

	/* i2s mode selection */
	reg_val = ac101_read(codec, AIF_CLK_CTRL);
	reg_val&=~(3<<AIF1_DATA_FMT);
	switch(fmt & SND_SOC_DAIFMT_FORMAT_MASK){
	case SND_SOC_DAIFMT_I2S:        /* I2S1 mode */
		printk("ac101 config I2S1 format\n");
		reg_val |= (0x0<<AIF1_DATA_FMT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:    /* Right Justified mode */
		printk("ac101 config Right format\n");
		reg_val |= (0x2<<AIF1_DATA_FMT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:     /* Left Justified mode */
		printk("ac101 config Left format\n");
		reg_val |= (0x1<<AIF1_DATA_FMT);
		break;
	case SND_SOC_DAIFMT_DSP_A:      /* L reg_val msb after FRM LRC */
		printk("ac101 config PCM-A format\n");
		reg_val |= (0x3<<AIF1_DATA_FMT);
		break;
	case SND_SOC_DAIFMT_DSP_B:
		printk("ac101 config PCM-B format\n");
		/* TODO: data offset set to 0 */
		reg_val |= (0x3<<AIF1_DATA_FMT);
		break;
	default:
		pr_err("%s, line:%d\n", __func__, __LINE__);
		return -EINVAL;
	}
	ac101_write(codec, AIF_CLK_CTRL, reg_val);


	/* DAI signal inversions */
	reg_val = ac101_read(codec, AIF_CLK_CTRL);
	switch(fmt & SND_SOC_DAIFMT_INV_MASK){
	case SND_SOC_DAIFMT_NB_NF:     /* normal bit clock + nor frame */
		printk("AC101 config BCLK&LRCK polarity: BCLK_normal,LRCK_normal\n");
		reg_val &= ~(0x1<<AIF1_LRCK_INV);
		reg_val &= ~(0x1<<AIF1_BCLK_INV);
		break;
	case SND_SOC_DAIFMT_NB_IF:     /* normal bclk + inv frm */
		printk("AC101 config BCLK&LRCK polarity: BCLK_normal,LRCK_invert\n");
		reg_val |= (0x1<<AIF1_LRCK_INV);
		reg_val &= ~(0x1<<AIF1_BCLK_INV);
		//reg_val |= (0x1<<AIF1_BCLK_INV);
		break;
	case SND_SOC_DAIFMT_IB_NF:     /* invert bclk + nor frm */
		printk("AC101 config BCLK&LRCK polarity: BCLK_invert,LRCK_normal\n");
		reg_val &= ~(0x1<<AIF1_LRCK_INV);
		reg_val |= (0x1<<AIF1_BCLK_INV);
		break;
	case SND_SOC_DAIFMT_IB_IF:     /* invert bclk + inv frm */
		printk("AC101 config BCLK&LRCK polarity: BCLK_invert,LRCK_invert\n");
		reg_val |= (0x1<<AIF1_LRCK_INV);
		reg_val |= (0x1<<AIF1_BCLK_INV);
		break;
	}
	ac101_write(codec, AIF_CLK_CTRL, reg_val);

	ac101_set_clock(1);
	return 0;
}

int ac101_audio_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	AC101_DBG("\n\n\n");
	ac101_headphone_event(codec, SND_SOC_DAPM_POST_PMU);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
	}
	return 0;
}

static int ac101_set_clock(int y_start_n_stop) {
	int r;

	if (y_start_n_stop) {
		/* enable global clock */
		r = ac101_aif1clk(static_ac10x->codec, SND_SOC_DAPM_PRE_PMU, 1);
	} else {
		/* disable global clock */
		static_ac10x->aif1_clken = 1;
		r = ac101_aif1clk(static_ac10x->codec, SND_SOC_DAPM_POST_PMD, 0);
	}
	return r;
}

int ac101_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	int ret = 0;

	AC101_DBG("stream=%s  cmd=%d\n",
		snd_pcm_stream_str(substream),
		cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		//#if _MASTER_MULTI_CODEC == _MASTER_AC101
		//if (ac10x->aif1_clken == 0){
			/*
			 * enable aif1clk, it' here due to reduce time between 'AC108 Sysclk Enable' and 'AC101 Sysclk Enable'
			 * Or else the two AC108 chips lost the sync.
			 */
		//	ret = 0;
		//	ret = ret || ac101_update_bits(codec, MOD_CLK_ENA, (0x1<<MOD_CLK_AIF1), (0x1<<MOD_CLK_AIF1));
		//	ret = ret || ac101_update_bits(codec, MOD_RST_CTRL, (0x1<<MOD_RESET_AIF1), (0x1<<MOD_RESET_AIF1));
		//}
		//#endif
		//ret = ac101_set_clock(1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		//ret = ac101_set_clock(0);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int ac101_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct ac10x_priv *ac10x = static_ac10x;

	AC101_DBG("id=%d freq=%d, dir=%d\n",
		clk_id, freq, dir);

	ac10x->sysclk = freq;
	ac10x->clk_id = clk_id;

	return 0;
}

static int ac101_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div) {
	return 0;
}

static const struct snd_soc_dai_ops ac101_aif1_dai_ops = {
	.startup	= ac101_audio_startup,
	.shutdown	= ac101_aif_shutdown,
	.set_sysclk	= ac101_set_dai_sysclk,
	.set_pll	= ac101_set_pll,
	.set_fmt	= ac101_set_dai_fmt,
	.hw_params	= ac101_hw_params,
	//.trigger	= ac101_trigger,
	.digital_mute	= ac101_aif_mute,
	.set_clkdiv     = ac101_set_clkdiv,
};

static struct snd_soc_dai_driver ac101_dai = {
	.name = "ac101-ic-pcm0",
	//.id = AIF1_CLK,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AC101_RATES,
		.formats = AC101_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
                .channels_min = 1,
                .channels_max = 2,
                .rates = AC101_RATES,
                .formats = AC101_FORMATS,
        },

	.ops = &ac101_aif1_dai_ops,
};

static struct snd_soc_codec_driver ac101_soc_codec_driver = {
	.probe 		= ac101_codec_probe,
	.remove 	= ac101_codec_remove,
	.suspend 	= ac101_codec_suspend,
	.resume 	= ac101_codec_resume,
	.set_bias_level = ac101_set_bias_level,
	//.read		= ac108_codec_read,
	//.write		= ac108_codec_write,
};


static void codec_resume_work(struct work_struct *work)
{
	struct ac10x_priv *ac10x = container_of(work, struct ac10x_priv, codec_resume);
	struct snd_soc_codec *codec = ac10x->codec;

	AC101_DBG("+++\n");

	set_configuration(codec);
	if (drc_used) {
		drc_config(codec);
	}
	/*enable this bit to prevent leakage from ldoin*/
	ac101_update_bits(codec, ADDA_TUNE3, (0x1<<OSCEN), (0x1<<OSCEN));

	AC101_DBG("---\n");
	return;
}

int ac101_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		AC101_DBG("SND_SOC_BIAS_ON\n");
		break;
	case SND_SOC_BIAS_PREPARE:
		AC101_DBG("SND_SOC_BIAS_PREPARE\n");
		break;
	case SND_SOC_BIAS_STANDBY:
		AC101_DBG("SND_SOC_BIAS_STANDBY\n");
		#ifdef CONFIG_AC101_SWITCH_DETECT
		switch_hw_config(codec);
		#endif
		break;
	case SND_SOC_BIAS_OFF:
		#ifdef CONFIG_AC101_SWITCH_DETECT
		ac101_update_bits(codec, ADC_APC_CTRL, (0x1<<HBIASEN), (0<<HBIASEN));
		ac101_update_bits(codec, ADC_APC_CTRL, (0x1<<HBIASADCEN), (0<<HBIASADCEN));
		#endif
		ac101_update_bits(codec, OMIXER_DACA_CTRL, (0xf<<HPOUTPUTENABLE), (0<<HPOUTPUTENABLE));
		ac101_update_bits(codec, ADDA_TUNE3, (0x1<<OSCEN), (0<<OSCEN));
		AC101_DBG("SND_SOC_BIAS_OFF\n");
		break;
	}
	snd_soc_codec_get_dapm(codec)->bias_level = level;
	return 0;
}

#define _SET_CLOCK_CNT          2
static int (* _set_clock[_SET_CLOCK_CNT])(int y_start_n_stop);

int seeed_voice_card_register_set_clock(int stream, int (*set_clock)(int)) {
        if (! _set_clock[stream]) {
                _set_clock[stream] = set_clock;
        }
        return 0;
}

int ac101_codec_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	struct ac10x_priv *ac10x;

	ac10x = static_ac10x;

	if (ac10x == NULL) {
		AC101_DBG("not set client data!\n");
		return -ENOMEM;
	}
	ac10x->codec = codec;
	dev_set_drvdata(codec->dev, ac10x);

	INIT_DELAYED_WORK(&ac10x->dlywork, ac10x_work_aif_play);
	INIT_WORK(&ac10x->codec_resume, codec_resume_work);
	ac10x->dac_enable = 0;
	ac10x->aif1_clken = 0;
	mutex_init(&ac10x->dac_mutex);

	#if _MASTER_MULTI_CODEC == _MASTER_AC101
	//seeed_voice_card_register_set_clock(SNDRV_PCM_STREAM_PLAYBACK, ac101_set_clock);
	#endif

	set_configuration(ac10x->codec);

	/*enable this bit to prevent leakage from ldoin*/
	ac101_update_bits(codec, ADDA_TUNE3, (0x1<<OSCEN), (0x1<<OSCEN));
	ac101_write(codec, DAC_VOL_CTRL, 0);

	/* customized get/put inteface */
	for (ret = 0; ret < ARRAY_SIZE(ac101_controls); ret++) {
		struct snd_kcontrol_new* skn = &ac101_controls[ret];

		skn->get = snd_ac101_get_volsw;
		skn->put = snd_ac101_put_volsw;
	}
	ret = snd_soc_add_codec_controls(codec, ac101_controls,	ARRAY_SIZE(ac101_controls));
	if (ret) {
		pr_err("[ac10x] Failed to register audio mode control, "
				"will continue without it.\n");
	}

	#ifdef CONFIG_AC101_SWITCH_DETECT
	//ret = ac101_switch_probe(ac10x);
	if (ret) {
		// not care the switch return value
	}
	#endif

	return 0;
}

/* power down chip */
int ac101_codec_remove(struct snd_soc_codec *codec)
{
	#ifdef CONFIG_AC101_SWITCH_DETECT
	struct ac10x_priv *ac10x = static_ac10x;

	if (ac10x->irq) {
		devm_free_irq(codec->dev, ac10x->irq, ac10x);
		ac10x->irq = 0;
	}

	if (cancel_work_sync(&ac10x->work_switch) != 0) {
	}

	if (cancel_work_sync(&ac10x->work_clear_irq) != 0) {
	}

	if (ac10x->inpdev) {
		input_unregister_device(ac10x->inpdev);
		ac10x->inpdev = NULL;
	}
	#endif

	return 0;
}

int ac101_codec_suspend(struct snd_soc_codec *codec)
{
	//struct ac10x_priv *ac10x = snd_soc_codec_get_drvdata(codec);
	struct ac10x_priv *ac10x = static_ac10x;

	AC101_DBG("[codec]:suspend\n");
	regcache_cache_only(ac10x->regmap101, true);
	return 0;
}

int ac101_codec_resume(struct snd_soc_codec *codec)
{
	struct ac10x_priv *ac10x = snd_soc_codec_get_drvdata(codec);
	int ret;

	AC101_DBG("[codec]:resume");

	/* Sync reg_cache with the hardware */
	regcache_cache_only(ac10x->regmap101, false);
	ret = regcache_sync(ac10x->regmap101);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to sync register cache: %d\n", ret);
		regcache_cache_only(ac10x->regmap101, true);
		return ret;
	}

	#ifdef CONFIG_AC101_SWITCH_DETECT
	ac10x->mode = HEADPHONE_IDLE;
	ac10x->state = -1;
	#endif

	ac101_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	schedule_work(&ac10x->codec_resume);
	return 0;
}

/***************************************************************************/
static ssize_t ac101_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ac10x_priv *ac10x = static_ac10x;
	u32 value_w, value_r, val = 0, flag = 0;
	u32 reg, num, i = 0;

	val = (u32)simple_strtol(buf, NULL, 16);
	flag = (val >> 24) & 0xF;
	if (flag) {
		reg = (val >> 16) & 0xFF;
		value_w =  val & 0xFFFF;
		ac101_write(ac10x->codec, reg, value_w);
		printk("write 0x%x to reg:0x%x\n", value_w, reg);
	} else {
		reg = (val >> 8) & 0xFF;
		num = val & 0xff;
		printk("\n");
		printk("read:start add:0x%x,count:0x%x\n", reg, num);

		regcache_cache_bypass(ac10x->regmap101, true);
		do {
			value_r = ac101_read(ac10x->codec, reg);
			printk("0x%x: 0x%04x ", reg++, value_r);
			if (++i % 4 == 0 || i == num)
				printk("\n");
		} while (i < num);
		regcache_cache_bypass(ac10x->regmap101, false);
	}
	return count;
}
static ssize_t ac101_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int val;
	char *s = buf;
	val= ac101_read(NULL,PLL_CTRL1);
	s += sprintf(s, " PLL_CTRL1(0x1): 0x%08x \n", val);
	val= ac101_read(NULL,PLL_CTRL2);
	s += sprintf(s, " PLL_CTRL2(0x2): 0x%08x \n", val);
	val= ac101_read(NULL,SYSCLK_CTRL);
	s += sprintf(s, " SYSCLK_CTRL(0x3): 0x%08x \n", val);
	val= ac101_read(NULL,MOD_CLK_ENA);
	s += sprintf(s, " MOD_CLK_ENA(0x4): 0x%08x \n", val);
	val= ac101_read(NULL,MOD_RST_CTRL);
	s += sprintf(s, " MOD_RST_CTRL(0x5): 0x%08x \n", val);
	val= ac101_read(NULL,AIF_SR_CTRL);
	s += sprintf(s, " AIF_SR_CTRL(0x6): 0x%08x \n", val);

	val= ac101_read(NULL,AIF1_CLK_CTRL);
	s += sprintf(s, " AIF1_CLK_CTRL(0x10): 0x%08x \n", val);
	val= ac101_read(NULL,AIF1_ADCDAT_CTRL);
	s += sprintf(s, " AIF1_ADCDAT_CTRL(0x11): 0x%08x \n", val);
	val= ac101_read(NULL,AIF1_DACDAT_CTRL);
	s += sprintf(s, " AIF1_DACDAT_CTRL(0x12): 0x%08x \n", val);
	val= ac101_read(NULL,AIF1_MXR_SRC);
	s += sprintf(s, " AIF1_MXR_SRC(0x13): 0x%08x \n", val);
	val= ac101_read(NULL,AIF1_VOL_CTRL1);
	s += sprintf(s, " AIF1_VOL_CTRL1(0x14): 0x%08x \n", val);
	val= ac101_read(NULL,AIF1_VOL_CTRL2);
	s += sprintf(s, " AIF1_VOL_CTRL2(0x15): 0x%08x \n", val);

	val= ac101_read(NULL,AIF1_VOL_CTRL3);
	s += sprintf(s, " AIF1_VOL_CTRL3(0x16): 0x%08x \n", val);
	val= ac101_read(NULL,AIF1_VOL_CTRL4);
	s += sprintf(s, " AIF1_VOL_CTRL4(0x17): 0x%08x \n", val);
	val= ac101_read(NULL,AIF1_MXR_GAIN);
	s += sprintf(s, " AIF1_MXR_GAIN(0x18): 0x%08x \n", val);
	val= ac101_read(NULL,AIF1_RXD_CTRL);
	s += sprintf(s, " AIF1_RXD_CTRL(0x19): 0x%08x \n", val);
	val= ac101_read(NULL,ADC_DIG_CTRL);
	s += sprintf(s, " ADC_DIG_CTRL(0x40): 0x%08x \n", val);
	val= ac101_read(NULL,ADC_VOL_CTRL);
	s += sprintf(s, " ADC_VOL_CTRL(0x41): 0x%08x \n", val);

	val= ac101_read(NULL,ADC_DBG_CTRL);
	s += sprintf(s, " ADC_DBG_CTRL(0x42): 0x%08x \n", val);
	val= ac101_read(NULL,HMIC_CTRL1);
	s += sprintf(s, " HMIC_CTRL1(0x44): 0x%08x \n", val);
	val= ac101_read(NULL,HMIC_CTRL2);
	s += sprintf(s, " HMIC_CTRL2(0x45): 0x%08x \n", val);
	val= ac101_read(NULL,HMIC_STS);
	s += sprintf(s, " HMIC_STS(0x46): 0x%08x \n", val);
	val= ac101_read(NULL,DAC_DIG_CTRL);
	s += sprintf(s, " DAC_DIG_CTRL(0x48): 0x%08x \n", val);
	val= ac101_read(NULL,DAC_VOL_CTRL);
	s += sprintf(s, " DAC_VOL_CTRL(0x49): 0x%08x \n", val);

	val= ac101_read(NULL,DAC_DBG_CTRL);
	s += sprintf(s, " DAC_DBG_CTRL(0x4a): 0x%08x \n", val);
	val= ac101_read(NULL,DAC_MXR_SRC);
	s += sprintf(s, " DAC_MXR_SRC(0x4c): 0x%08x \n", val);
	val= ac101_read(NULL,DAC_MXR_GAIN);
	s += sprintf(s, " DAC_MXR_GAIN(0x4d): 0x%08x \n", val);
	val= ac101_read(NULL,ADC_APC_CTRL);
	s += sprintf(s, " ADC_APC_CTRL(0x50): 0x%08x \n", val);
	val= ac101_read(NULL,ADC_SRC);
	s += sprintf(s, " ADC_SRC(0x51): 0x%08x \n", val);
	val= ac101_read(NULL, ADC_SRCBST_CTRL);
	s += sprintf(s, "  ADC_SRCBST_CTRL(0x52): 0x%08x \n", val);

	val= ac101_read(NULL,OMIXER_DACA_CTRL);
	s += sprintf(s, " OMIXER_DACA_CTRL(0x53): 0x%08x \n", val);
	val= ac101_read(NULL,OMIXER_SR);
	s += sprintf(s, " OMIXER_SR(0x54): 0x%08x \n", val);
	val= ac101_read(NULL,OMIXER_BST1_CTRL);
	s += sprintf(s, " OMIXER_BST1_CTRL(0x55): 0x%08x \n", val);
	val= ac101_read(NULL,HPOUT_CTRL);
	s += sprintf(s, " HPOUT_CTRL(0x56): 0x%08x \n", val);
	val= ac101_read(NULL, ESPKOUT_CTRL);
	s += sprintf(s, "  ESPKOUT_CTRL(0x57): 0x%08x \n", val);
	val= ac101_read(NULL,SPKOUT_CTRL);
	s += sprintf(s, " SPKOUT_CTRL(0x58): 0x%08x \n", val);

	val= ac101_read(NULL,LOUT_CTRL);
	s += sprintf(s, " LOUT_CTRL(0x59): 0x%08x \n", val);
	val= ac101_read(NULL,ADDA_TUNE1);
	s += sprintf(s, " ADDA_TUNE1(0x5a): 0x%08x \n", val);
	val= ac101_read(NULL,ADDA_TUNE2);
	s += sprintf(s, " ADDA_TUNE2(0x5b): 0x%08x \n", val);
	val= ac101_read(NULL,ADDA_TUNE3);
	s += sprintf(s, " ADDA_TUNE3(0x5c): 0x%08x \n", val);
	val= ac101_read(NULL,HPOUT_STR);
	s += sprintf(s, " HPOUT_STR(0x5d): 0x%08x \n", val);

	if(s != buf)
		*(s-1) = '\n';
	return (s-buf);
}
static DEVICE_ATTR(ac10x, 0644, ac101_debug_show, ac101_debug_store);

static struct attribute *audio_debug_attrs[] = {
	&dev_attr_ac10x.attr,
	NULL,
};

static struct attribute_group audio_debug_attr_group = {
	.name   = "ac101_debug",
	.attrs  = audio_debug_attrs,
};

static bool ac101_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case PLL_CTRL2:
	case HMIC_STS:
		return true;
	}
	return false;
}

#if 0
static int ac101_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir) {
	return 0;
}
#endif

static const struct regmap_config ac101_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.reg_stride = 1,
	.max_register = 0xB5,
	.cache_type = REGCACHE_FLAT,
	.volatile_reg = ac101_volatile_reg,
};

/* Sync reg_cache from the hardware */
int ac10x_fill_regcache(struct device* dev, struct regmap* map) {
	int r, i, n;
	int v;

	n = regmap_get_max_register(map);
	for (i = 0; i < n; i++) {
		regcache_cache_bypass(map, true);
		r = regmap_read(map, i, &v);
		if (r) {
			dev_dbg(dev, "failed to read register %d\n", i);
			continue;
		}
		regcache_cache_bypass(map, false);

		regcache_cache_only(map, true);
		r = regmap_write(map, i, v);
		regcache_cache_only(map, false);
	}
	regcache_cache_bypass(map, false);
	regcache_cache_only(map, false);

	return 0;
}
#if 0
static unsigned int ac101_codec_read(struct snd_soc_codec *codec, unsigned int reg) {
	return 0;
}

static unsigned int ac101_codec_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int val) {
	return 0;
}

static int ac101_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai) {
	dev_dbg(dai->dev, "%s() stream = %s\n", __func__, snd_pcm_stream_str(substream));
	return 0;
}
#endif

void ac10x_init(struct snd_soc_codec *codec) {
	ac101_write(codec, 0x01, 0x0141);
	ac101_write(codec, 0x02, 0x0000);
	ac101_write(codec, 0x03, 0x0808);

	ac101_write(codec, 0x06, 0x8000);
	ac101_write(codec, 0x10, 0x2C50);

	ac101_write(codec, 0x04, 0x808c);
	ac101_write(codec, 0x05, 0x808c);
	ac101_write(codec, 0x11, 0xc000);
	ac101_write(codec, 0x12, 0xc000);
	ac101_write(codec, 0x13, 0x2200);
	ac101_write(codec, 0x40, 0x8000);
	ac101_write(codec, 0x45, 0x0010);
	ac101_write(codec, 0x48, 0x8000);
	ac101_write(codec, 0x4c, 0x8800);
	ac101_write(codec, 0x50, 0xbbbc);
	ac101_write(codec, 0x51, 0x1040);
	ac101_write(codec, 0x52, 0xddc4);
	ac101_write(codec, 0x53, 0xff80);
	ac101_write(codec, 0x54, 0x0102);
	ac101_write(codec, 0x56, 0x3bc1);
	ac101_write(codec, 0x58, 0xeabb);
	ac101_write(codec, 0x82, 0x2000);
	ac101_write(codec, 0x83, 0x2000);
	ac101_write(codec, 0xb4, 0x00c0);

}

int ac101_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct ac10x_priv *ac10x;
	int ret = 0;
	unsigned v = 0;

	ac10x = devm_kzalloc(&i2c->dev, sizeof(struct ac10x_priv), GFP_KERNEL);
	if(ac10x == NULL) {
		dev_err(&i2c->dev, "Unable to allocate ac101 private data\n");
		return -ENOMEM;
	}

	ac10x->i2c101 = i2c;
	dev_set_drvdata(&i2c->dev, ac10x);

	ac10x->regmap101 = devm_regmap_init_i2c(i2c, &ac101_regmap);
	if (IS_ERR(ac10x->regmap101)) {
		ret = PTR_ERR_OR_ZERO(ac10x->regmap101);
		dev_err(&i2c->dev, "Fail to initialize I/O: %d\n", ret);
		return ret;
	}

	/* Chip reset */
	regcache_cache_only(ac10x->regmap101, false);
	ret = regmap_write(ac10x->regmap101, CHIP_AUDIO_RST, 0);
	msleep(50);

	/* sync regcache for FLAT type */
	ac10x_fill_regcache(&i2c->dev, ac10x->regmap101);

	ret = regmap_read(ac10x->regmap101, CHIP_AUDIO_RST, &v);
	if (ret < 0) {
		dev_err(&i2c->dev, "failed to read vendor ID: %d\n", ret);
		return ret;
	}

	if (v != AC101_CHIP_ID) {
		dev_err(&i2c->dev, "chip is not AC101 (%X)\n", v);
		dev_err(&i2c->dev, "Expected %X\n", AC101_CHIP_ID);
		return -ENODEV;
	}

	ret = snd_soc_register_codec(&ac10x->i2c101->dev, &ac101_soc_codec_driver, &ac101_dai, 1);
	if(ret) {
		pr_err("failed to register codec, ret = %d\n", ret);
	}

	ret = sysfs_create_group(&i2c->dev.kobj, &audio_debug_attr_group);
	if (ret) {
		pr_err("failed to create attr group\n");
	}

	static_ac10x = ac10x;

	/*int gpiod_spk_amp_gate = 0;
	gpiod_spk_amp_gate = devm_gpio_request(&i2c->dev, 71, "spk-amp-switch");
	if (IS_ERR(ac10x->gpiod_spk_amp_gate)) {
		ac10x->gpiod_spk_amp_gate = NULL;
		dev_err(&i2c->dev, "failed get spk-amp-switch in device tree\n");
		return gpiod_spk_amp_gate;
	}
	gpio_direction_output(71, 0);
	ac10x->gpiod_spk_amp_gate = gpio_to_desc(gpiod_spk_amp_gate);*/

	ac10x_init(ac10x->codec);
	pr_info("%s register success\n", __func__);
	return 0;
}

void ac101_shutdown(struct i2c_client *i2c)
{
	struct ac10x_priv *ac10x = i2c_get_clientdata(i2c);
	struct snd_soc_codec *codec = ac10x->codec;
	int reg_val;

	if (codec == NULL) {
		pr_err(": no sound card.\n");
		return;
	}

	/*set headphone volume to 0*/
	reg_val = ac101_read(codec, HPOUT_CTRL);
	reg_val &= ~(0x3f<<HP_VOL);
	ac101_write(codec, HPOUT_CTRL, reg_val);

	/*disable pa*/
	reg_val = ac101_read(codec, HPOUT_CTRL);
	reg_val &= ~(0x1<<HPPA_EN);
	ac101_write(codec, HPOUT_CTRL, reg_val);

	/*hardware xzh support*/
	reg_val = ac101_read(codec, OMIXER_DACA_CTRL);
	reg_val &= ~(0xf<<HPOUTPUTENABLE);
	ac101_write(codec, OMIXER_DACA_CTRL, reg_val);

	/*unmute l/r headphone pa*/
	reg_val = ac101_read(codec, HPOUT_CTRL);
	reg_val &= ~((0x1<<RHPPA_MUTE)|(0x1<<LHPPA_MUTE));
	ac101_write(codec, HPOUT_CTRL, reg_val);
	return;
}

int ac101_i2c_remove(struct i2c_client *i2c)
{
	sysfs_remove_group(&i2c->dev.kobj, &audio_debug_attr_group);
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static const struct i2c_device_id ac101_i2c_id[] = {
	{ "ac101", AC101_I2C_ID },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ac101_i2c_id);

static const struct of_device_id ac101_of_match[] = {
	{ .compatible = "ac101",   },
	{ }
};
MODULE_DEVICE_TABLE(of, ac101_of_match);

static struct i2c_driver ac101_i2c_driver = {
	.driver = {
		.name = "ac101-codec",
		.of_match_table = ac101_of_match,
	},
	.probe =    ac101_i2c_probe,
	.remove =   ac101_i2c_remove,
	.id_table = ac101_i2c_id,
};

module_i2c_driver(ac101_i2c_driver);


MODULE_DESCRIPTION("ASoC ac10x driver");
MODULE_AUTHOR("huangxin,liushaohua");
MODULE_AUTHOR("PeterYang<linsheng.yang@seeed.cc>");
MODULE_LICENSE("GPL");

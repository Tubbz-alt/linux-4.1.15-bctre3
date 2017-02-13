/*
 * drivers/media/video/mxc/capture/tvp5147.c
 *
 * TI TVP5147 Decoder MXC V4L2 Driver
 *
 * Copyright (C) 2011 DiVA Group
 * Authors: 
 * 
 *     Lautaro Carmona <lautarocarmona@gmail.com>
 *     Fernando Tolassi <fmtolassi@gmail.com>
 *     Jose Alvite <josealvite86@gmail.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/wait.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"
#include "tvp514x_regs.h"

#define TVP5147_VOLTAGE_ANALOG               1800000
#define TVP5147_VOLTAGE_DIGITAL_CORE         1800000
#define TVP5147_VOLTAGE_DIGITAL_IO           3300000
#define TVP5147_VOLTAGE_PLL                  1800000


static struct regulator *dvddio_regulator;
static struct regulator *dvdd_regulator;
static struct regulator *avdd_regulator;
static struct regulator *pvdd_regulator;
static int pwn_gpio;
static int reset_gpio;

static int tvp5147_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *id);
static int tvp5147_detach(struct i2c_client *client);

static const struct i2c_device_id tvp5147_id[] = {
	{"tvp5147", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tvp5147_id);

static struct i2c_driver tvp5147_i2c_driver = {		
	.driver = {										
		   .owner = THIS_MODULE,
		   .name = "tvp5147",
		   },
	.probe = tvp5147_probe,
	.remove = tvp5147_detach,
	.id_table = tvp5147_id,
};

/* This effectively appends an extra member onto struct sensor_data */
static struct sensor {
	struct sensor_data sen;
	v4l2_std_id std_id;
} tvp5147_data;

/*Supported formats*/
typedef enum {
	TVP5147_NTSC = 0,	/*!< Locked on (M) NTSC video signal. */
	TVP5147_PAL,		/*!< (B, G, H, I, N)PAL video signal. */
	TVP5147_NOT_LOCKED,	/*!< Not locked on a signal. */
} video_fmt_idx;

#define TVP5147_STD_MAX		(TVP5147_PAL + 1)

/*Video format struct*/
typedef struct {
	int v4l2_id;		/*!< Video for linux ID. */
	char name[16];		/*!< Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width;		/*!< Raw width. */
	u16 raw_height;		/*!< Raw height. */
	u16 active_width;	/*!< Active width. */
	u16 active_height;	/*!< Active height. */
	int frame_rate;		/*!< Frame rate. */
} video_fmt_t;


/*! Supported video structs
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {
	{			/*! NTSC */					
	 .v4l2_id = V4L2_STD_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,	/* SENS_FRM_WIDTH */
	 .raw_height = 525,	/* SENS_FRM_HEIGHT */
	 .active_width = 720,	/* ACT_FRM_WIDTH plus 1 */
	 .active_height = 480,	/* ACT_FRM_WIDTH plus 1 */
	 .frame_rate = 30,
	 },
	{			/*! (B, G, H, I, N) PAL */
	 .v4l2_id = V4L2_STD_PAL,
	 .name = "PAL",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .frame_rate = 25,

	 },
	{			/*! Unlocked standard */
	 .v4l2_id = V4L2_STD_ALL,
	 .name = "Autodetect",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .frame_rate = 0,
	 },
};

//static video_fmt_idx video_idx = TVP5147_NTSC;//TVP5147_PAL;
static video_fmt_idx video_idx = TVP5147_PAL;

static DEFINE_SEMAPHORE(mutex);

#define IF_NAME                    "tvp5147"


/*Supported controls*/
/*(Not finished)*/
static struct v4l2_queryctrl tvp5147_qctrl[] = {
	{
	.id = V4L2_CID_BRIGHTNESS,					
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Brightness",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 1,		/* check this value */
	.default_value = 128,	/* check this value */
	.flags = 0,
	}, {
	.id = V4L2_CID_SATURATION,					
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Saturation",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 0x1,		/* check this value */
	.default_value = 128	,	/* check this value */
	.flags = 0,
	},{
	.id = V4L2_CID_CONTRAST,					
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Constrast",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 1,		/* check this value */
	.default_value = 128,	/* check this value */
	.flags = 0,
	},{
	.id = V4L2_CID_HUE,					
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Hue",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 1,		/* check this value */
	.default_value = 0,	/* check this value */
	.flags = 0,
	}
	
};

/* TVP514x default register values */
static struct tvp514x_reg tvp514x_reg_list_default[] = {
	/* Composite selected */
	{TOK_WRITE, REG_INPUT_SEL, 0x05},
	{TOK_WRITE, REG_AFE_GAIN_CTRL, 0x0F},
	/* Auto mode */
	{TOK_WRITE, REG_VIDEO_STD, 0x00},
	{TOK_WRITE, REG_OPERATION_MODE, 0x00},
	{TOK_SKIP, REG_AUTOSWITCH_MASK, 0x3F},
	{TOK_WRITE, REG_COLOR_KILLER, 0x10},
	{TOK_WRITE, REG_LUMA_CONTROL1, 0x00},
	{TOK_WRITE, REG_LUMA_CONTROL2, 0x00},
	{TOK_WRITE, REG_LUMA_CONTROL3, 0x02},
	{TOK_WRITE, REG_BRIGHTNESS, 0x80},
	{TOK_WRITE, REG_CONTRAST, 0x80},
	{TOK_WRITE, REG_SATURATION, 0x80},
	{TOK_WRITE, REG_HUE, 0x00},
	{TOK_WRITE, REG_CHROMA_CONTROL1, 0x00},
	{TOK_WRITE, REG_CHROMA_CONTROL2, 0x0E},
	/* Reserved */
	{TOK_SKIP, 0x0F, 0x00},
	{TOK_WRITE, REG_COMP_PR_SATURATION, 0x80},
	{TOK_WRITE, REG_COMP_Y_CONTRAST, 0x80},
	{TOK_WRITE, REG_COMP_PB_SATURATION, 0x80},
	/* Reserved */
	{TOK_SKIP, 0x13, 0x00},
	{TOK_WRITE, REG_COMP_Y_BRIGHTNESS, 0x80},
	/* Reserved */
	{TOK_SKIP, 0x15, 0x00},
	/* NTSC timing */
	{TOK_SKIP, REG_AVID_START_PIXEL_LSB, 0x55},
	{TOK_SKIP, REG_AVID_START_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_AVID_STOP_PIXEL_LSB, 0x25},
	{TOK_SKIP, REG_AVID_STOP_PIXEL_MSB, 0x03},
	/* NTSC timing */
	{TOK_SKIP, REG_HSYNC_START_PIXEL_LSB, 0x00},
	{TOK_SKIP, REG_HSYNC_START_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_HSYNC_STOP_PIXEL_LSB, 0x40},
	{TOK_SKIP, REG_HSYNC_STOP_PIXEL_MSB, 0x00},
	/* NTSC timing */
	{TOK_SKIP, REG_VSYNC_START_LINE_LSB, 0x04},
	{TOK_SKIP, REG_VSYNC_START_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VSYNC_STOP_LINE_LSB, 0x07},
	{TOK_SKIP, REG_VSYNC_STOP_LINE_MSB, 0x00},
	/* NTSC timing */
	{TOK_SKIP, REG_VBLK_START_LINE_LSB, 0x01},
	{TOK_SKIP, REG_VBLK_START_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VBLK_STOP_LINE_LSB, 0x15},
	{TOK_SKIP, REG_VBLK_STOP_LINE_MSB, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x26, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x27, 0x00},
	{TOK_SKIP, REG_FAST_SWTICH_CONTROL, 0xCC},
	/* Reserved */
	{TOK_SKIP, 0x29, 0x00},
	{TOK_SKIP, REG_FAST_SWTICH_SCART_DELAY, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x2B, 0x00},
	{TOK_SKIP, REG_SCART_DELAY, 0x00},
	{TOK_SKIP, REG_CTI_DELAY, 0x00},
	{TOK_SKIP, REG_CTI_CONTROL, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x2F, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x30, 0x00},
	/* Reserved */
	{TOK_SKIP, 0x31, 0x00},
	/* HS, VS active high */
	{TOK_WRITE, REG_SYNC_CONTROL, 0x00},
	/* 10-bit BT.656 */
	{TOK_WRITE, REG_OUTPUT_FORMATTER1, 0x00},
	/* Enable clk & data */
	{TOK_WRITE, REG_OUTPUT_FORMATTER2, 0x11},
	/* Enable AVID & FLD */
	{TOK_WRITE, REG_OUTPUT_FORMATTER3, 0xEE},
	/* Enable VS & HS */
	{TOK_WRITE, REG_OUTPUT_FORMATTER4, 0xAF},
	{TOK_WRITE, REG_OUTPUT_FORMATTER5, 0xFF},
	{TOK_WRITE, REG_OUTPUT_FORMATTER6, 0xFF},
	/* Clear status */
	{TOK_WRITE, REG_CLEAR_LOST_LOCK, 0x01},
	{TOK_TERM, 0, 0},
};

static inline void tvp5147_power_down(int enable)
{
	gpio_set_value_cansleep(pwn_gpio, !enable);
	msleep(2);
}

static int tvp5147_regulator_enable(struct device *dev)
{
	int ret = 0;

	dvddio_regulator = devm_regulator_get(dev, "DOVDD");

	if (!IS_ERR(dvddio_regulator)) {
		regulator_set_voltage(dvddio_regulator,
				      TVP5147_VOLTAGE_DIGITAL_IO,
				      TVP5147_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(dvddio_regulator);
		if (ret) {
			dev_err(dev, "set io voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set io voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get io voltage\n");
	}

	dvdd_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(dvdd_regulator)) {
		regulator_set_voltage(dvdd_regulator,
				      TVP5147_VOLTAGE_DIGITAL_CORE,
				      TVP5147_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(dvdd_regulator);
		if (ret) {
			dev_err(dev, "set core voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set core voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get core voltage\n");
	}

	avdd_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(avdd_regulator)) {
		regulator_set_voltage(avdd_regulator,
				      TVP5147_VOLTAGE_ANALOG,
				      TVP5147_VOLTAGE_ANALOG);
		ret = regulator_enable(avdd_regulator);
		if (ret) {
			dev_err(dev, "set analog voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set analog voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get analog voltage\n");
	}

	pvdd_regulator = devm_regulator_get(dev, "PVDD");
	if (!IS_ERR(pvdd_regulator)) {
		regulator_set_voltage(pvdd_regulator,
				      TVP5147_VOLTAGE_PLL,
				      TVP5147_VOLTAGE_PLL);
		ret = regulator_enable(pvdd_regulator);
		if (ret) {
			dev_err(dev, "set pll voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set pll voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get pll voltage\n");
	}

	return ret;
}


/***********************************************************************
 * I2C transfert.
 ***********************************************************************/

static inline int tvp5147_read(u8 reg)
{
	int val;
	val = i2c_smbus_read_byte_data(tvp5147_data.sen.i2c_client, reg);
	if (val < 0) {
		printk(
			"%s:read reg error: reg=%2x \n", __func__, reg);
		return -1;
	}
	return val;
}

static int tvp5147_write_reg(u8 reg, u8 val)
{
	if (i2c_smbus_write_byte_data(tvp5147_data.sen.i2c_client, reg, val) < 0) {
		printk(
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return -1;
	}
	return 0;
}

/**
 * tvp5147_write_regs() : Initializes a list of TVP5146/47 registers
 * @reglist: list of TVP5146/47 registers and values
 *
 * Initializes a list of TVP5146/47 registers:-
 *		if token is TOK_TERM, then entire write operation terminates
 *		if token is TOK_DELAY, then a delay of 'val' msec is introduced
 *		if token is TOK_SKIP, then the register write is skipped
 *		if token is TOK_WRITE, then the register write is performed
 * Returns zero if successful, or non-zero otherwise.
 */
static int tvp5147_write_regs( const struct tvp514x_reg reglist[])
{
	int err;
	const struct tvp514x_reg *next = reglist;

	for (; next->token != TOK_TERM; next++) {
		if (next->token == TOK_DELAY) {
			msleep(next->val);
			continue;
		}

		if (next->token == TOK_SKIP)
			continue;

		err = tvp5147_write_reg(next->reg, (u8) next->val);
		if (err) {
			printk("Write failed. Err[%d]\n", err);
			return err;
		}
	}
	return 0;
}

/***********************************************************************
 * mxc_v4l2_capture interface.
 ***********************************************************************/

/*!
 * Return attributes of current video standard.
 * Since this device autodetects the current standard, this function also
 * sets the values that need to be changed if the standard changes.
 * There is no set std equivalent function.
 *
 *  @return		None.
*/
static void tvp5147_get_std(v4l2_std_id *std)
{
	int tmp;
	int idx;

	printk("In tvp5147_get_std\n");

	/* Make sure power on */
	tvp5147_power_down(0);

	tmp = tvp5147_read(REG_VIDEO_STD_STATUS) & 0x07;

	down(&mutex);
	if (tmp == 0x02) {			
		/* PAL */
		*std = V4L2_STD_PAL;
		idx = TVP5147_PAL;
	} else if (tmp == 0x01) {
		/*NTSC*/
		*std = V4L2_STD_NTSC;
		idx = TVP5147_NTSC;
	} else {
		*std = V4L2_STD_ALL;
		idx = TVP5147_NOT_LOCKED;
		printk(
			"Got invalid video standard! \n");
	}
	up(&mutex);

	/* This assumes autodetect which this device uses. */
	if (*std != tvp5147_data.std_id) {
		video_idx = idx;
		tvp5147_data.std_id = *std;
		tvp5147_data.sen.pix.width = video_fmts[video_idx].raw_width;
		tvp5147_data.sen.pix.height = video_fmts[video_idx].raw_height;
	}
}



/***********************************************************************
 * IOCTL Functions from v4l2_int_ioctl_desc.
 ***********************************************************************/

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	printk("In tvp5147:ioctl_g_ifparm\n");

	if (s == NULL) {
		printk("<3>""   ERROR!! no slave device set!\n");
		return -1;
	}

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility.*/
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_10BIT;
	p->u.bt656.bt_sync_correct = 1;

	/* tvp5147 has a dedicated clock so no clock settings needed. */

	return 0;
}

/*!
 * Sets the camera power.
 *
 * s  pointer to the camera device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on open, close, suspend and resume.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor *sensor = s->priv;

	printk("In tvp5147:ioctl_s_power (%d)\n", on);

	if (on && !sensor->sen.on) 
        {
                        printk("tvp5147 power on\n");
	                tvp5147_power_down(0);
        }
	else if (!on && sensor->sen.on)
        {
                        printk("tvp5147 power off\n");
	                tvp5147_power_down(1);
        }
			
			
	sensor->sen.on = on;
	
	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	printk("In tvp5147:ioctl_g_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		printk("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->sen.streamcap.capability;
		cparm->timeperframe = sensor->sen.streamcap.timeperframe;
		cparm->capturemode = sensor->sen.streamcap.capturemode;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		printk("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 *
 * This driver cannot change these settings.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	printk("In tvp5147:ioctl_s_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		printk("   type is unknown - %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;

	printk("In tvp5147:ioctl_g_fmt_cap\n");

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		printk("   Returning size of %dx%d\n",
			 sensor->sen.pix.width, sensor->sen.pix.height);
		f->fmt.pix = sensor->sen.pix;						/*Carga en la estrucuta v4l2 el formato de pixeles*/
		break;

	case V4L2_BUF_TYPE_PRIVATE: {
		v4l2_std_id std;
		tvp5147_get_std(&std);
		f->fmt.pix.pixelformat = (u32)std;
		}
		break;

	default:
		f->fmt.pix = sensor->sen.pix;
		break;
	}

	return 0;
}

/*!
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
			   struct v4l2_queryctrl *qc)
{
	int i;

	printk("In tvp5147:ioctl_queryctrl\n");

	for (i = 0; i < ARRAY_SIZE(tvp5147_qctrl); i++)
		if (qc->id && qc->id == tvp5147_qctrl[i].id) {
			memcpy(qc, &(tvp5147_qctrl[i]),
				sizeof(*qc));
			return (0);
		}

	return -EINVAL;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	printk("In tvp5147:ioctl_g_ctrl\n");

	/* Make sure power on */
	tvp5147_power_down(0);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		printk("   V4L2_CID_BRIGHTNESS\n");
		tvp5147_data.sen.brightness = tvp5147_read(REG_BRIGHTNESS);
		vc->value = tvp5147_data.sen.brightness;
		break;
	case V4L2_CID_CONTRAST:
		printk("   V4L2_CID_CONTRAST\n");
		tvp5147_data.sen.contrast = tvp5147_read(REG_CONTRAST);
		vc->value = tvp5147_data.sen.contrast;
		break;
	case V4L2_CID_SATURATION:
		printk("   V4L2_CID_SATURATION\n");
		tvp5147_data.sen.saturation = tvp5147_read(REG_SATURATION);
		vc->value = tvp5147_data.sen.saturation;
		break;
	case V4L2_CID_HUE:
		printk("   V4L2_CID_HUE\n");
		tvp5147_data.sen.hue = tvp5147_read(REG_HUE);
		vc->value = tvp5147_data.sen.hue;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		printk("   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		printk("   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		printk("   V4L2_CID_RED_BALANCE\n");
		break;
	case V4L2_CID_BLUE_BALANCE:
		printk("   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		printk("   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		printk("   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_AUTOGAIN:
		printk("   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		printk("   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		printk("   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		printk("   V4L2_CID_VFLIP\n");
		break;
	default:
		printk("   Default case\n");
		vc->value = 0;
		ret = -EPERM;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	u8 tmp;

	printk("In tvp5147:ioctl_s_ctrl\n");
	/* Make sure power on */
	tvp5147_power_down(0);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		printk("   V4L2_CID_BRIGHTNESS\n");
		tmp = vc->value;
		tvp5147_write_reg(REG_BRIGHTNESS, tmp);
		tvp5147_data.sen.brightness = vc->value;
		break;
	case V4L2_CID_CONTRAST:
		printk("   V4L2_CID_CONTRAST\n");
		tmp = vc->value;
		tvp5147_write_reg(REG_CONTRAST, tmp);
		tvp5147_data.sen.contrast = vc->value;
		break;
	case V4L2_CID_SATURATION:
		printk("   V4L2_CID_SATURATION\n");
		tmp = vc->value;
		tvp5147_write_reg(REG_SATURATION, tmp);
		tvp5147_data.sen.saturation = vc->value;
		break;
	case V4L2_CID_HUE:
		printk("   V4L2_CID_HUE\n");
		tmp = vc->value;
		tvp5147_write_reg(REG_HUE, tmp);
		tvp5147_data.sen.hue = vc->value;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		printk("   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		printk("   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		printk("   V4L2_CID_RED_BALANCE\n");
		break;
	case V4L2_CID_BLUE_BALANCE:
		printk("   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		printk("   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		printk("   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_AUTOGAIN:
		printk("   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		printk("   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		printk("   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		printk("   V4L2_CID_VFLIP\n");
		break;
	default:
		printk("   Default case\n");
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	printk("In tvp5147:ioctl_init\n");
	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	printk("In tvp5147:ioctl_dev_init\n");
	return 0;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index >= ARRAY_SIZE(video_fmts))
		return -EINVAL;

	fsize->pixel_format = tvp5147_data.sen.pix.pixelformat;
	fsize->discrete.width = video_fmts[fsize->index].active_width;
	fsize->discrete.height = video_fmts[fsize->index].active_height;
	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	video_fmt_t fmt;
	int i;

	if (fival->index != 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(video_fmts) - 1; i++) {
		fmt = video_fmts[i];
		if (fival->width  == fmt.active_width &&
		    fival->height == fmt.active_height) {
			fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
			fival->discrete.numerator = 1;
			fival->discrete.denominator = fmt.frame_rate;
			return 0;
		}
	}

	return -EINVAL;
}


/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "tvp5147_tvin");

	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */static struct v4l2_int_ioctl_desc tvp5147_ioctl_desc[] = {

	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},

	/*!
	 * Delinitialise the dev. at slave detach.
	 * The complement of ioctl_dev_init.
	 */
/*	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func *)ioctl_dev_exit}, */

	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},

	/*!
	 * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
	 */
/*	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap}, */

	/*!
	 * VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
	 * This ioctl is used to negotiate the image capture size and
	 * pixel format without actually making it take effect.
	 */
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */

	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},

	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */

	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
				(v4l2_int_ioctl_func *)
				ioctl_enum_frameintervals},			
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave tvp5147_slave = {
	.ioctls = tvp5147_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tvp5147_ioctl_desc),
};

static struct v4l2_int_device tvp5147_int_device = {
	.module = THIS_MODULE,
	.name = "tvp5147",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &tvp5147_slave,
	},
};


/***********************************************************************
 * I2C client and driver.
 ***********************************************************************/

/*! tvp5147 Reset function.
 *
 *  @return		None.
 */
static void tvp5147_hard_reset(void)
{
	printk("In tvp5147:tvp5147_hard_reset\n");
       
        gpio_set_value_cansleep(reset_gpio, 1);
        msleep(2);
        gpio_set_value_cansleep(reset_gpio, 0);

	/*! Datasheet Reset and Initialization: */
	/*
	 * The following register writes must be made before normal operation of the device.
	 * STEP | I2C SUBADDRESS	| I2C DATA
	 *  1	|		0x03		|	0x01
	 *  2	|		0x03		|   0x00
	 */
	tvp5147_write_reg(REG_OPERATION_MODE, 0x01);
	tvp5147_write_reg(REG_OPERATION_MODE, 0x00);

#if 0
	/*
	 * Input Select Register:
	 * 
	 * INPUT(S) SELECTED: CVBS VI_2_A
	 */
	tvp5147_write_reg(REG_INPUT_SEL, 0x04);
	
	/*
	 * Analog Output Control 1 Register:
	 * 
	 * AGC enable: Enabled
	 * Input select: Input selected by TVP5147M1 decoder.
	 * Analog output enable: VI_1_A is analog video output.
	 */
	tvp5147_write_reg(REG_ANALOG_OUTPUT_CONTROL1, 0x01);

	/*! Datasheet recommends: */
	/*
	 * Luminance Processing Control 3 Register:
	 * 
	 * Filter select[1:0] = 00
	 * 
	 * Trap filter stop-band bandwidth (MHz):
	 * NTSC ITU-R BT.601 = 1.2129 
	 * PAL ITU-R BT.601 = 1.2129
	 * 
	 * (Optimizes the trap filter selection for NTSC and PAL)
	 */
	tvp5147_write_reg(REG_LUMA_CONTROL3, 0x00);
	/*
	 * Chrominance Processing Control 2 Register:
	 * 
	 * PAL compensation: Disabled
	 * Wideband chroma LPF filter (WCF): Enabled
	 * Chrominance filter select: Disabled 
	 * 
	 */
	tvp5147_write_reg(REG_CHROMA_CONTROL2, 0x04);
	/* 
	 * Output Formatter 1 Register:
	 * 
	 * Output Format: 10-bit 4:2:2 (pixel x 2 rate) with embedded syncs (ITU-R BT.656) 
	 * YCbCr output code range: Extended coding range (Y, Cb, and Cr range from 4 to 1016)
	 * CbCr code format: Offset binary code (2s complement + 512)
	 * 
	 */
	tvp5147_write_reg(REG_OUTPUT_FORMATTER1, 0x40);
	/*
	 * Output Formatter 2 Register:
	 * 
	 * Data enable: Y[9:0] and C[9:0] active
	 * Black Screen: Normal operation
	 * CLK polarity: Data clocked out on the falling edge of DATACLK
	 * Clock enable: DATACLK outputs are enabled
	 * 
	 */ 
	tvp5147_write_reg(REG_OUTPUT_FORMATTER2, 0x11);
	/*
	 * Output Formatter 4 Register:
	 * 
	 * VS terminal function select: VS/VBLK is vertical sync or vertical blank output corresponding to bit 1 (VS/VBLK) in the sync control register at subaddress 32h
	 * HS terminal function select: HS/CS is horizontal sync or composite sync output corresponding to bit 0 (HS/CS) in the sync control register at subaddress 32h
	 * C_1 terminal function select: C_1 is logic input
	 * C_0 terminal function select: C_0 is logic input
	 * 
	 */
	tvp5147_write_reg(REG_OUTPUT_FORMATTER4, 0xAF);
#endif
        /* write iniitialisation sequence */
        tvp5147_write_regs(tvp514x_reg_list_default);
}

/*! tvp5147 I2C attach function.
 *
 *  @param *adapter	struct i2c_adapter *.
 *
 *  @return		Error code indicating success or failure.
 */

/*!
 * tvp5147 I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *
 *  @return		Error code indicating success or failure.
 */

static int tvp5147_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret = 0;
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;

	printk("In tvp5147_probe\n");


	/* ov5640 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

	/* request power down pin */
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwn_gpio)) {
		dev_err(dev, "no sensor pwdn pin available\n");
		return -ENODEV;
	}
	ret = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,
					"adv7180_pwdn");
	if (ret < 0) {
		dev_err(dev, "no power pin available!\n");
		return ret;
	}

	/* request reset pin */
	reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpios", 0);
	if (!gpio_is_valid(reset_gpio)) {
		dev_err(dev, "no sensor reset pin available\n");
		return -ENODEV;
	}
	ret = devm_gpio_request_one(dev, reset_gpio, GPIOF_OUT_INIT_HIGH,
					"adv7180_reset");

	tvp5147_regulator_enable(dev);

	tvp5147_power_down(0);

	msleep(1);

	/* Set initial values for the sensor struct. */
	memset(&tvp5147_data, 0, sizeof(tvp5147_data));
	tvp5147_data.sen.i2c_client = client;
	tvp5147_data.sen.streamcap.timeperframe.denominator = 30;
	tvp5147_data.sen.streamcap.timeperframe.numerator = 1;
	tvp5147_data.std_id = V4L2_STD_ALL;
	video_idx = TVP5147_NOT_LOCKED;
	tvp5147_data.sen.pix.width = video_fmts[video_idx].raw_width;
	tvp5147_data.sen.pix.height = video_fmts[video_idx].raw_height;
	/*
	 * Posible v4l2 YUV 4:2:2 values:
	 * 
	 * #define V4L2_PIX_FMT_YUYV    v4l2_fourcc('Y', 'U', 'Y', 'V') // 16  YUV 4:2:2     
	 * #define V4L2_PIX_FMT_UYVY    v4l2_fourcc('U', 'Y', 'V', 'Y') // 16  YUV 4:2:2     
	 * #define V4L2_PIX_FMT_VYUY    v4l2_fourcc('V', 'Y', 'U', 'Y') // 16  YUV 4:2:2     
	 * #define V4L2_PIX_FMT_YUV422P v4l2_fourcc('4', '2', '2', 'P') // 16  YVU422 planar 
	 */ 
	tvp5147_data.sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;  /* YUV422 */
	tvp5147_data.sen.pix.priv = 1;  /* 1 is used to indicate TV in */
	tvp5147_data.sen.on = true;

	tvp5147_data.sen.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(tvp5147_data.sen.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(tvp5147_data.sen.sensor_clk);
	}

	ret = of_property_read_u32(dev->of_node, "mclk",
					&tvp5147_data.sen.mclk);
	if (ret) {
		dev_err(dev, "mclk frequency is invalid\n");
		return ret;
	}

	ret = of_property_read_u32(
		dev->of_node, "mclk_source",
		(u32 *) &(tvp5147_data.sen.mclk_source));
	if (ret) {
		dev_err(dev, "mclk_source invalid\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "csi_id",
					&(tvp5147_data.sen.csi));
	if (ret) {
		dev_err(dev, "csi_id invalid\n");
		return ret;
	}

	clk_prepare_enable(tvp5147_data.sen.sensor_clk);


	printk("%s:tvp5147 probe i2c address is 0x%02X \n",
		__func__, tvp5147_data.sen.i2c_client->addr);

	/*! tvp5147 initialization. */
	tvp5147_hard_reset();

	printk("   type is %d (expect %d)\n",
		 tvp5147_int_device.type, v4l2_int_type_slave);

	printk("   num ioctls is %d\n",
		 tvp5147_int_device.u.slave->num_ioctls);

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the tvp5147_data structure here.*/
	tvp5147_int_device.priv = &tvp5147_data;
	ret = v4l2_int_device_register(&tvp5147_int_device);
	
	clk_disable_unprepare(tvp5147_data.sen.sensor_clk);

	pr_info("TV input TVP5147 is found\n");

	return ret;
}

static int tvp5147_detach(struct i2c_client *client)
{
	printk("%s:Removing %s video decoder @ 0x%02X from adapter %s \n",
		__func__, IF_NAME, client->addr, client->adapter->name);

	if (dvddio_regulator) {
		regulator_disable(dvddio_regulator);
	}

	if (dvdd_regulator) {
		regulator_disable(dvdd_regulator);
	}

	if (avdd_regulator) {
		regulator_disable(avdd_regulator);
	}

	if (pvdd_regulator) {
		regulator_disable(pvdd_regulator);
	}
	
	v4l2_int_device_unregister(&tvp5147_int_device);

	return 0;
}

static __init int tvp5147_init(void)
{
	u8 err = 0;

	printk("In tvp5147_init\n");
	
	err = i2c_add_driver(&tvp5147_i2c_driver);
	if (err != 0)
		printk("<3>""%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}


static void __exit tvp5147_exit(void)
{
	printk("In tvp5147_clean\n");
	i2c_del_driver(&tvp5147_i2c_driver);
}

module_init(tvp5147_init);
module_exit(tvp5147_exit);

MODULE_AUTHOR("DiVA Group");
MODULE_DESCRIPTION("Texas Instrument TVP5147 MXC V4L2 video decoder driver");
MODULE_LICENSE("GPL");

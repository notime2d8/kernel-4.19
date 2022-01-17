// SPDX-License-Identifier: GPL-2.0
/*
 * os04a10 driver
 *
 * Copyright (C) 2020 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 * V0.0X01.0X01 support conversion gain switch.
 * V0.0X01.0X02 add debug interface for conversion gain switch.
 * V0.0X01.0X03 support enum sensor fmt
 * V0.0X01.0X04 add quick stream on/off
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-event.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-dv-timings.h>
#include <linux/videodev2.h>
#include <linux/pinctrl/consumer.h>

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x0a, 0x01)
#define CHIP_ID				0x1605
#define REG_SC_CHIP_ID_H		0x00
#define REG_SC_CHIP_ID_L		0x01

#define OF_CAMERA_PINCTRL_STATE_DEFAULT "rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP   "rockchip,camera_sleep"

#define LT7911D_NAME                    "lt7911d"

#define MIPI_FREQ_320M                  320000000
#define MIPI_FREQ_620M                  620000000
#define PIXEL_RATE_WITH_320M            (MIPI_FREQ_320M * 2 / 8 * 4)
#define PIXEL_RATE_WITH_620M            (MIPI_FREQ_620M * 2 / 8 * 4)

static const struct v4l2_dv_timings_cap lt7911d_timings_cap = {
        .type = V4L2_DV_BT_656_1120,
        /* keep this initialization for compatibility with GCC < 4.4.6 */
        .reserved = { 0 },
        /* Pixel clock from REF_01 p. 20. Min/max height/width are unknown */
        V4L2_INIT_BT_TIMINGS(1, 10000, 1, 10000, 0, MIPI_FREQ_620M,
                        V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
                        V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,
                        V4L2_DV_BT_CAP_PROGRESSIVE | V4L2_DV_BT_CAP_INTERLACED |
                        V4L2_DV_BT_CAP_REDUCED_BLANKING |
                        V4L2_DV_BT_CAP_CUSTOM)
};

static const s64 link_freq_menu_items[] = {
        MIPI_FREQ_320M,
        MIPI_FREQ_620M,
};

struct lt7911d {
        struct i2c_client       *client;
        struct clk              *xvclk;
        struct gpio_desc        *reset_gpio;
        struct gpio_desc        *power_gpio;
        struct gpio_desc        *irq_gpio;

        struct pinctrl          *pinctrl;
        struct pinctrl_state    *pins_default;
        struct pinctrl_state    *pins_sleep;

        struct v4l2_subdev      subdev;
        struct media_pad        pad;
        struct mutex            mutex;
        u32 mbus_fmt_code;
        struct v4l2_dv_timings timings;

        struct v4l2_ctrl_handler ctrl_hdl;
        struct v4l2_ctrl        *pixel_rate;
        struct v4l2_ctrl        *link_freq;

        int irq;
        struct workqueue_struct *queue;
        struct work_struct work;
        struct delayed_work delaywork;
        int irq_flag;
};

static int lt7911d_s_dv_timings(struct v4l2_subdev *sd,
                                 struct v4l2_dv_timings *timings);

#define to_lt7911d(sd) container_of(sd, struct lt7911d, subdev)

static int lt7911d_write(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	dev_dbg(&client->dev, "write reg(0x%x val:0x%x)!\n", reg, val);

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"lt7911d write reg(0x%x val:0x%x) failed ! %d \n", reg, val,ret);

	return ret;
}

static int lt7911d_read(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[1];
	int ret;

	//buf[0] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = &reg;
	msg[0].len = 1;//sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	dev_err(&client->dev,
		"lt7911d read reg:0x%x failed !\n", reg);

	return ret;
}

static int lt7911d_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct lt7911d *lt7911d = to_lt7911d(sd);
	printk("enter %s fmt->which=%d, fmt->width=%d,height=%d, code=%d \n",__func__, 
                fmt->which,fmt->format.width, fmt->format.height, fmt->format.code);
        if (fmt->format.code != MEDIA_BUS_FMT_UYVY8_2X8) {
            printk("media bus format invalid\n");
            return -EINVAL;
        } else {
			fmt->format.code = lt7911d->mbus_fmt_code;
			fmt->format.width = lt7911d->timings.bt.width;
			fmt->format.height = lt7911d->timings.bt.height;
			fmt->format.field = V4L2_FIELD_NONE;
			__v4l2_ctrl_s_ctrl(lt7911d->link_freq,
			link_freq_menu_items[0]);
		__v4l2_ctrl_s_ctrl_int64(lt7911d->pixel_rate,
			link_freq_menu_items[1]);
		}
	return 0;
}

static int lt7911d_query_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct lt7911d *lt7911d = to_lt7911d(sd);

	*timings = lt7911d->timings;

	if (debug)
		v4l2_print_dv_timings(sd->name, "lt7911d_query_dv_timings: ",
				timings, false);

	if (!v4l2_valid_dv_timings(timings,
				&lt7911d_timings_cap, NULL, NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	return 0;
}

static int lt7911d_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
        struct lt7911d *lt7911d = to_lt7911d(sd);
        fmt->format.code = lt7911d->mbus_fmt_code;
        fmt->format.width = lt7911d->timings.bt.width;
        fmt->format.height = lt7911d->timings.bt.height;
        fmt->format.field = V4L2_FIELD_NONE;
        printk("enter %s fmt->which=%d, fmt->width=%d height=%d\n",__func__, fmt->which, 
                fmt->format.width,fmt->format.height);
	return 0;
}

static int lt7911d_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
        printk("%s code->index=%d\n",__func__, code->index);
        switch (code->index) {
        case 0:
                code->code = MEDIA_BUS_FMT_UYVY8_2X8;
                break;
        /*
        case 1:
                code->code = MEDIA_BUS_FMT_RGB888_1X24;
                break;
        */
        default:
                return -EINVAL;
        }
        return 0;
}

static int lt7911d_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	printk("enter %s\n",__func__);
        config->type = V4L2_MBUS_CSI2;
        config->flags = V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK;
        config->flags |= V4L2_MBUS_CSI2_4_LANE;
        config->flags |= V4L2_MBUS_CSI2_CHANNEL_3;
        printk("%s flags = %d\n",__func__,config->flags);
	return 0;
}

static int lt7911d_s_stream(struct v4l2_subdev *sd, int on)
{
	printk("enter %s\n, stream on/off= %d",__func__, on);
	return 0;
}

static int lt7911d_s_power(struct v4l2_subdev *sd, int on)
{
	printk("%s\n",__func__);
	return 0;
}

static int lt7911d_get_resolution(struct i2c_client *client, unsigned int *Hactive, unsigned int * Vactive)
{
    int ret;
	//unsigned char VtotalH,VtotalL;		//Vtotal[11:0]
	unsigned char VactiveH,VactiveL;	//Vactive[11:0]
	//unsigned char VsyncW,VBP,VFP;		//V blank timing 8bits

	//特别注意：H方向的读取Timing等于实际Timing的二分之一
	//例如读出的Hactived[15:0] = 0x021c，则实际的Hactive=0x021c*2=0x0438=1080
	//unsigned char HtotalH,HtotalL;		//Htotal[15:0]
	unsigned char HactiveH,HactiveL;	//Hactive[15:0]
	unsigned char pcrHH, pcrHL, pcrVH, pcrVL;
	//unsigned char msaHH, msaHL, msaVH, msaVL;
	unsigned char pll=0, BKD2_17_REG, BKD2_08_REG = 0, BKD2_09_REG;
	/*
	unsigned char HsyncWH,HsyncWL;		//HsyncW[11:0]
	unsigned char HBPH,HBPL;			//HBP[11:0]
	unsigned char HFPH,HFPL;			//HFP[11:0]
	*/

	//step 1: enable chip IIC, (bank reg is 0xff, bank value is 0x80)
	ret=lt7911d_write(client,0xFF, 0x80);
	//step 2: offset is 0xee, value is 0x1
	ret=lt7911d_write(client,0xEE, 0x01);
	    
     /* lt6911_i2c_write(0xd2, 0x83,0x11);// 0xd283[0] = Video source select from hmdi rx*/
	//SetRegisterBank(0xd2);
	//step 1: enable chip IIC, (bank reg is 0xff, bank value is 0xd2)
	ret=lt7911d_write(client,0xFF, 0xd2);
	//step 2: offset is 0x83, value is 0x10
	ret=lt7911d_write(client,0x83, 0x10); // Video source select from dp rx
	usleep_range(4000, 5000);
	/*
	lt7911d_read(client,0x9e, &VtotalH);//0xd29e[3:0]=Vtotal[11:8]
	lt7911d_read(client,0x9f, &VtotalL);//0xd29f[7:0]=Vtotal[7:0]
	*/
	lt7911d_read(client,0x96, &VactiveH);//0xd296[3:0]=VactiveH[11:8]
	lt7911d_read(client,0x97, &VactiveL);//0xd297[7:0]=VactiveH[7:0]
	pcrVH = VactiveH;
	pcrVL = VactiveL;
	/*
	lt7911d_read(client,0x86, &VsyncW);//0xd286[7:0]=VsyncW[7:0]
	lt7911d_read(client,0x87, &VBP);//0xd287[7:0]=VBP[7:0]
	lt7911d_read(client,0x88, &VFP);//0xd288[7:0]=VFP[7:0]
	
	lt7911d_read(client,0x89, &HtotalH);//0xd289[3:0]=Htotal[15:8]
	lt7911d_read(client,0x8a, &HtotalL);//0xd28a[7:0]=Htotal[7:0]
	*/
	lt7911d_read(client,0x8b, &HactiveH);//0xd28b[3:0]=Hactive[15:8]
	lt7911d_read(client,0x8c, &HactiveL);//0xd28c[7:0]=Hactive[7:0]
	pcrHH = HactiveH;
	pcrHL = HactiveL;
	lt7911d_read(client,0x08, &BKD2_08_REG);
	lt7911d_read(client, 0x09, &BKD2_09_REG);
	lt7911d_read(client,0x17, &BKD2_17_REG);
	/*
	lt7911d_read(client,0x94, &HsyncWH);//0xd294[3:0]=HsyncWH[11:8]
	lt7911d_read(client,0x95, &HsyncWL);//0xd295[7:0]=HsyncWL[7:0]
	lt7911d_read(client,0x98, &HBPH);//0xd298[3:0]=HBPH[11:8]
	lt7911d_read(client,0x99, &HBPL);//0xd299[7:0]=HBPL[7:0]
	lt7911d_read(client,0x9c, &HFPH);//0xd29c[3:0]=HFPH[11:8]
	lt7911d_read(client,0x9d, &HFPL);//0xd29d[7:0]=HFPL[7:0]
	*/
#if 0 //msa
	ret=lt7911d_write(client,0xFF, 0xd1);
	usleep_range(4000, 5000);
	lt7911d_read(client,0xe7, &msaHH);
	lt7911d_read(client,0xe8, &msaHL);

	lt7911d_read(client,0xe9, &msaVH);
	lt7911d_read(client,0xea, &msaVL);

	ret=lt7911d_write(client,0xFF, 0xb8);
	lt7911d_read(client,0xb0, &pll);
	#endif
	//disable IIC
	ret=lt7911d_write(client,0xFF, 0x80);
	ret=lt7911d_write(client,0xEE, 0x00);
	/*
	printk("Vtotal = %d\n", VtotalH<<8|VtotalL);
	printk("Htotal = %d\n", HtotalH<<8|HtotalL);
	
	printk("Vactive = %d\n", VactiveH<<8|VactiveL);
	printk("Hactive = %d\n", HactiveH<<8|HactiveL);
	
	printk("HsyncW = %d\n", HsyncWH<<8|HsyncWL);
	printk("HBP = %d\n", HBPH<<8|HBPL);
	printk("HFP = %d\n", HFPH<<8|HFPL);

	printk("HtotalH = 0x%x\n", HtotalH);
	printk("HtotalL = 0x%x\n\n", HtotalL);
	*/
	*Hactive =  (HactiveH<<8|HactiveL)*2;
	*Vactive = VactiveH<<8|VactiveL;
	printk(">> PCR Hactive = %d;Vactive = %d\n\n", *Hactive, *Vactive);
	//printk(">> MSA Hactive = %d;Vactive = %d\n\n", msaHH << 8|msaHL,msaVH<<8|msaVL);
	printk(">> BKD2_17_REG = 0x%x\n", BKD2_17_REG&0x40);
	printk(">> BKD2_08_REG = 0x%x\n", BKD2_08_REG);
	printk(">> BKD2_09_REG = 0x%x\n", BKD2_09_REG);
	printk(">> PLL 0x%x\n", pll);
	if (ret != 0) {
		printk("i2c transfer error \n");
	}

	if ((BKD2_17_REG&0x40) == 0 || *Hactive == 0 || *Vactive == 0) {
		//workaround way to ensure width and height > 1
		*Hactive = 640;
		*Vactive = 480;
	}

    return 0;
}

static int lt7911d_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	unsigned int Hactive=0;  // report to app
	unsigned int Vactive=0;  // report to app
	struct lt7911d *lt7911d = to_lt7911d(sd);
	struct i2c_client *client = lt7911d->client;
	const struct v4l2_event lt7911d_ev_fmt = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

    lt7911d_get_resolution(client, &Hactive, &Vactive);
	/* handle width and height change event */
	if (lt7911d->timings.bt.width != Hactive || lt7911d->timings.bt.height != Vactive) {
	struct v4l2_dv_timings timings;
	struct v4l2_bt_timings *bt = &timings.bt;
	memset(&timings, 0, sizeof(struct v4l2_dv_timings));
	timings.type = V4L2_DV_BT_656_1120;

	bt->width = Hactive;
	bt->height = Vactive;
	timings.bt.height = Vactive;
	timings.bt.width = Hactive;
	printk("current=%d:%d, new = %d:%d\n", lt7911d->timings.bt.width,
	lt7911d->timings.bt.height, bt->width, bt->height);
	lt7911d_s_dv_timings(sd, &timings);
	if (sd->devnode)
		v4l2_subdev_notify_event(sd, &lt7911d_ev_fmt);
	}
	return 0;
}

static irqreturn_t lt7911d_irq_handler(int irq, void *dev_id)
{
	struct lt7911d *lt7911d = dev_id;
	//struct device *dev = &lt7911d->client->dev;
	bool handled = 0;
    lt7911d->irq_flag = 1;
	printk("gpio5 interrupt occur\n");
	gpiod_direction_input(lt7911d->irq_gpio);

	lt7911d_isr(&lt7911d->subdev, 0, &handled);
	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static long lt7911d_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
        long ret = 0;
        u32 stream = 0;
        switch (cmd) {
        case RKMODULE_SET_QUICK_STREAM:
                stream = *((u32 *)arg);
                if (stream) {
                    printk("stream on \n"); 
                } else {
                    printk("stream off \n");
                }
                break;
        default:
                ret = -ENOIOCTLCMD;
                break;
        }
	
        return ret;
}

#ifdef CONFIG_COMPAT
static long lt7911d_compat_ioctl32(struct v4l2_subdev *sd,
                                  unsigned int cmd, unsigned long arg)
{
        void __user *up = compat_ptr(arg);
        long ret;
        u32 stream = 0;
        switch (cmd) {
       	case RKMODULE_SET_QUICK_STREAM:
                ret = copy_from_user(&stream, up, sizeof(u32));
                if (!ret)
                        ret = lt7911d_ioctl(sd, cmd, &stream);
                break;
       	default:
                ret = -ENOIOCTLCMD;
                break;
	}
	return ret;
}
#endif

static int lt7911d_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
                                    struct v4l2_event_subscription *sub)
{
        switch (sub->type) {
        case V4L2_EVENT_SOURCE_CHANGE:
                return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
        case V4L2_EVENT_CTRL:
                return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
        default:
                return -EINVAL;
        }
}

static int lt7911d_s_dv_timings(struct v4l2_subdev *sd,
                                 struct v4l2_dv_timings *timings)
{
        struct lt7911d *lt7911d = to_lt7911d(sd);

        if (!timings)
                return -EINVAL;

        v4l2_print_dv_timings(sd->name, "lt7911d_s_dv_timings: ",
                                timings, false);

        if (v4l2_match_dv_timings(&lt7911d->timings, timings, 0, false)) {
                v4l2_dbg(1, debug, sd, "%s: no change\n", __func__);
                return 0;
        }

        if (!v4l2_valid_dv_timings(timings,
                                &lt7911d_timings_cap, NULL, NULL)) {
                v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
                return -ERANGE;
        }

        lt7911d->timings = *timings;

        return 0;
}

static int lt7911d_g_dv_timings(struct v4l2_subdev *sd,
                                 struct v4l2_dv_timings *timings)
{
	struct lt7911d *lt7911d = to_lt7911d(sd);

	*timings = lt7911d->timings;

	printk("%s width=%d,height=%d\n", __func__,timings->bt.width, timings->bt.height);

	return 0;
}

static int lt7911d_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
        struct lt7911d *lt7911d = to_lt7911d(sd);
        struct v4l2_mbus_framefmt *try_fmt =
                                v4l2_subdev_get_try_format(sd, fh->pad, 0);
        //printk("%s\n",__func__);
        mutex_lock(&lt7911d->mutex);
        /* Initialize try_fmt */
        try_fmt->width = lt7911d->timings.bt.width;
        try_fmt->height = lt7911d->timings.bt.height;
        try_fmt->code = lt7911d->mbus_fmt_code;
        try_fmt->field = V4L2_FIELD_NONE;

        mutex_unlock(&lt7911d->mutex);
        /* No crop or compose */
        return 0;
}

static const struct v4l2_subdev_core_ops lt7911d_core_ops = {
	.s_power = lt7911d_s_power,
	.ioctl = lt7911d_ioctl,
#ifdef CONFIG_COMPAT
        .compat_ioctl32 = lt7911d_compat_ioctl32,
#endif
        .subscribe_event = lt7911d_subscribe_event,
        .unsubscribe_event = v4l2_event_subdev_unsubscribe,

};

static const struct v4l2_subdev_video_ops lt7911d_video_ops = {
	.s_stream = lt7911d_s_stream,
        .s_dv_timings = lt7911d_s_dv_timings,
        .g_dv_timings = lt7911d_g_dv_timings,
	.query_dv_timings = lt7911d_query_dv_timings,
	.g_mbus_config = lt7911d_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops lt7911d_pad_ops = {
	.enum_mbus_code = lt7911d_enum_mbus_code,
	.get_fmt = lt7911d_get_fmt,
	.set_fmt = lt7911d_set_fmt,
};

static const struct v4l2_subdev_ops lt7911d_subdev_ops = {
	.core	= &lt7911d_core_ops,
	.video	= &lt7911d_video_ops,
	.pad	= &lt7911d_pad_ops,
};

static const struct v4l2_subdev_internal_ops lt7911d_internal_ops = {
        .open = lt7911d_open,
};

// static int lt7911d_check_device_id(struct lt7911d *lt7911d,
// 				  struct i2c_client *client)
// {

// 	struct device *dev = &lt7911d->client->dev;
// 	u16 id = 0;
// 	u8 pid, ver;
// 	int ret;

//         lt7911d_write(client, 0xff, 0x80);
// 	lt7911d_write(client, 0xee, 0x01);
//         lt7911d_write(client, 0xff,  0xa0);
// 	ret = lt7911d_read(client, REG_SC_CHIP_ID_H, &pid);
//         ret = lt7911d_read(client, REG_SC_CHIP_ID_L, &ver);
// 	lt7911d_write(client, 0xff, 0x80);
// 	lt7911d_write(client, 0xee, 0x00);
// 	id = (pid << 8) | ver;
// 	if (id != CHIP_ID) {
// 		dev_err(dev, "Unexpected device id(%06x), ret(%d)\n", id, ret);
// 		//return -ENODEV;
// 	}else
// 		dev_info(dev, "Detected lontium%04x, expected id=%04x \n", id, CHIP_ID);

// 	return 0;
// }

void get_init_resolution(struct work_struct *work)
{
    struct lt7911d *lt7911d =
    container_of(to_delayed_work(work), struct lt7911d, delaywork);
    bool handled = 0;
    if (lt7911d->irq_flag == 1) {
        return;
    }
    lt7911d_isr(&lt7911d->subdev, 0, &handled);
    //mdelay(1000);
    //queue_work(workqueue_test, &work_test);
}

static void lt7911d_power_on(struct lt7911d *lt7911d) {
        struct device *dev = &lt7911d->client->dev;
	//	usleep_range(1500000, 2000000);
        if (!IS_ERR(lt7911d->power_gpio)) {
           gpiod_direction_output(lt7911d->power_gpio, 0);
           usleep_range(2000, 5000);
        }

        // if (!IS_ERR(lt7911d->reset_gpio))
        //    gpiod_direction_output(lt7911d->reset_gpio, 1);

        // usleep_range(1000, 2000);

        // if (!IS_ERR(lt7911d->reset_gpio))
        //    gpiod_direction_output(lt7911d->reset_gpio, 0);

        // usleep_range(1000, 1500);
        /*
         * There is no need to wait for the delay of RC circuit
         * if the reset signal is directly controlled by GPIO.
         */
        if (!IS_ERR(lt7911d->power_gpio)) {
           gpiod_set_value_cansleep(lt7911d->power_gpio, 1);
           usleep_range(2000, 5000);
        }
        //msleep(200);
    //     if (!IS_ERR(lt7911d->reset_gpio)) {
    //        gpiod_set_value_cansleep(lt7911d->reset_gpio, 1);
    //        usleep_range(2000, 5000);
    //     }
        lt7911d->queue=create_workqueue("lt7911d_thread");
        if (lt7911d->queue) {
            INIT_DELAYED_WORK(&lt7911d->delaywork, get_init_resolution);
            queue_delayed_work(lt7911d->queue, &lt7911d->delaywork, 1000);
        }
        //msleep(300);
        dev_info(dev, "lt7911d power up \n");
}

static int lt7911d_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct lt7911d *lt7911d;
	struct v4l2_subdev *sd;
        struct v4l2_dv_timings default_timing = V4L2_DV_BT_DMT_640X480P60;
	int ret=0;

	dev_info(dev, "driver version: %02x.%02x.%02x\n",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	lt7911d = devm_kzalloc(dev, sizeof(*lt7911d), GFP_KERNEL);
	if (!lt7911d)
		return -ENOMEM;

	lt7911d->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(lt7911d->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

        lt7911d->power_gpio = devm_gpiod_get(dev, "pwren", GPIOD_ASIS);
	if (IS_ERR(lt7911d->power_gpio))
		dev_warn(dev, "Failed to get power_gpios\n");
		
	lt7911d->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(lt7911d->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	lt7911d->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(lt7911d->pinctrl)) {
		lt7911d->pins_default =
			pinctrl_lookup_state(lt7911d->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(lt7911d->pins_default))
			dev_err(dev, "could not get default pinstate\n");
		lt7911d->pins_sleep =
			pinctrl_lookup_state(lt7911d->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(lt7911d->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	mutex_init(&lt7911d->mutex);

        lt7911d->mbus_fmt_code = MEDIA_BUS_FMT_UYVY8_2X8;
        lt7911d->client = client;

	sd = &lt7911d->subdev;
	v4l2_i2c_subdev_init(sd, client, &lt7911d_subdev_ops);
        sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
        sd->internal_ops = &lt7911d_internal_ops;

        /* initial control handler */
        v4l2_ctrl_handler_init(&lt7911d->ctrl_hdl, 2);
        /**
                * v4l2_ctrl_new_int_menu() - Create a new standard V4L2 integer menu control.
                *
                * @hdl:        The control handler.
                * @ops:        The control ops.
                * @id: The control ID.
                * @max:        The control's maximum value.
                * @def:        The control's default value.
                * @qmenu_int:  The control's menu entries.
                *
                * Same as v4l2_ctrl_new_std_menu(), but @mask is set to 0 and it additionaly
                * takes as an argument an array of integers determining the menu items.
                *
                * If @id refers to a non-integer-menu control, then this function will
                * return %NULL.
               struct v4l2_ctrl *v4l2_ctrl_new_int_menu(struct v4l2_ctrl_handler *hdl,
                                                        const struct v4l2_ctrl_ops *ops,
                                                        u32 id, u8 max, u8 def,
                                                        const s64 *qmenu_int);
        */
        lt7911d->link_freq = v4l2_ctrl_new_int_menu(&lt7911d->ctrl_hdl, NULL,
                                  V4L2_CID_LINK_FREQ,
                                  1, 1, link_freq_menu_items);
           /**
                * v4l2_ctrl_new_std() - Allocate and initialize a new standard V4L2 non-menu
                *      control.
                *
                * @hdl:        The control handler.
                * @ops:        The control ops.
                * @id:         The control ID.
                * @min:        The control's minimum value.
                * @max:        The control's maximum value.
                * @step:       The control's step value
                * @def:        The control's default value.
                *
                * If the &v4l2_ctrl struct could not be allocated, or the control
                * ID is not known, then NULL is returned and @hdl->error is set to the
                * appropriate error code (if it wasn't set already).
                *
                * If @id refers to a menu control, then this function will return NULL.
                *
                * Use v4l2_ctrl_new_std_menu() when adding menu controls.

                * struct v4l2_ctrl *v4l2_ctrl_new_std(struct v4l2_ctrl_handler *hdl,
                *                                       const struct v4l2_ctrl_ops *ops,
                *                                       u32 id, s64 min, s64 max, u64 step,
                *                                       s64 def);
                *
                */
        lt7911d->pixel_rate = v4l2_ctrl_new_std(&lt7911d->ctrl_hdl, NULL,
                                  V4L2_CID_PIXEL_RATE,
                                  0, PIXEL_RATE_WITH_620M,
                                  1, PIXEL_RATE_WITH_620M);

        __v4l2_ctrl_s_ctrl(lt7911d->link_freq, link_freq_menu_items[1]);
               __v4l2_ctrl_s_ctrl_int64(lt7911d->pixel_rate,
                       PIXEL_RATE_WITH_620M);

        sd->ctrl_handler = &lt7911d->ctrl_hdl;
        if (lt7911d->ctrl_hdl.error) {
                dev_err(dev, "handler error=%d", lt7911d->ctrl_hdl.error);
                goto err_destroy_mutex;
        }

        lt7911d->irq = -1;
        lt7911d->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
        if (IS_ERR(lt7911d->irq_gpio)) {
        	dev_err(dev, "can not find irq-gpio\n");
        }

        if (!IS_ERR(lt7911d->irq_gpio)) {
                lt7911d->irq = gpiod_to_irq(lt7911d->irq_gpio);
                dev_info(dev, "has irq type =%d",lt7911d->irq);
                ret = devm_request_threaded_irq(&client->dev,
                                                lt7911d->irq,
                                                NULL, lt7911d_irq_handler,
                                                IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                                                "lt7911d", lt7911d);
                if (ret) {
         		dev_info(dev, "regiser irq failed"); 
                }
                disable_irq(lt7911d->irq);
        } 

	lt7911d->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &lt7911d->pad);
	if (ret < 0)
		goto err_clean_entity;

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}
        lt7911d_s_dv_timings(sd, &default_timing);
        lt7911d_power_on(lt7911d);
        enable_irq(lt7911d->irq);
        // ret = lt7911d_check_device_id(lt7911d, client);
        // if (ret)
        //        goto err_clean_entity;

	return 0;

err_clean_entity:
	media_entity_cleanup(&sd->entity);
err_destroy_mutex:
	mutex_destroy(&lt7911d->mutex);

	return ret;
}

static int lt7911d_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt7911d *lt7911d = to_lt7911d(sd);

    destroy_workqueue(lt7911d->queue);
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&lt7911d->mutex);

	return 0;
}

static const struct of_device_id lt7911d_of_match[] = {
	{ .compatible = "lontium,lt7911d" },
	{},
};
MODULE_DEVICE_TABLE(of, lt7911d_of_match);

static const struct i2c_device_id lt7911d_match_id[] = {
	{ "lontium,lt7911d", 0 },
	{ },
};

static struct i2c_driver lt7911d_i2c_driver = {
	.driver = {
		.name = LT7911D_NAME,
		.of_match_table = of_match_ptr(lt7911d_of_match),
	},
	.probe		= &lt7911d_probe,
	.remove		= &lt7911d_remove,
	.id_table	= lt7911d_match_id,
};

static int __init lt7911d_driver_init(void)
{
	return i2c_add_driver(&lt7911d_i2c_driver);
}

static void __exit lt7911d_driver_exit(void)
{
	i2c_del_driver(&lt7911d_i2c_driver);
}

device_initcall_sync(lt7911d_driver_init);
module_exit(lt7911d_driver_exit);

MODULE_DESCRIPTION("LT9711D type-c dp to csi-2 bridge driver");
MODULE_LICENSE("GPL v2");

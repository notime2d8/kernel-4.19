/*
 * drivers/net/phy/motorcomm.c
 *
 * Driver for Motorcomm PHYs
 *
 * Author: Leilei Zhao <leilei.zhao@motorcomm.com>
 *
 * Copyright (c) 2019 Motorcomm, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Support Motorcomm Phys:
 *	 Giga phys: yt8511, yt8521
 *	 100/10 Phys : yt8512, yt8512b, yt8510
 *	 Automotive 100Mb Phys : yt8010
 *	 Automotive 100/10 hyper range Phys: yt8510
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/clk.h>

#define MOTORCOMM_PHY_ID_MASK	0x00000fff
#define PHY_ID_YT8010		0x00000309
#define PHY_ID_YT8510		0x00000109
#define PHY_ID_YT8511		0x0000010a
#define PHY_ID_YT8512		0x00000118
#define PHY_ID_YT8512B		0x00000128
#define PHY_ID_YT8521		0x0000011a

#define REG_PHY_SPEC_STATUS		0x11
#define REG_DEBUG_ADDR_OFFSET		0x1e
#define REG_DEBUG_DATA			0x1f

#define YT8512_EXTREG_AFE_PLL		0x50
#define YT8512_EXTREG_EXTEND_COMBO	0x4000
#define YT8512_EXTREG_LED0		0x40c0
#define YT8512_EXTREG_LED1		0x40c3

#define YT8512_EXTREG_SLEEP_CONTROL1	0x2027

#define YT_SOFTWARE_RESET		0x8000

#define YT8512_PLL_REFCLK_SEL_EN	0x0040
#define YT8512_CONTROL1_RMII_EN	0x0001
#define YT8512_LED0_ACT_BLK_IND	0x1000
#define YT8512_LED0_DIS_LED_AN_TRY	0x0001
#define YT8512_LED0_BT_BLK_EN		0x0002
#define YT8512_LED0_HT_BLK_EN		0x0004
#define YT8512_LED0_COL_BLK_EN	0x0008
#define YT8512_LED0_BT_ON_EN		0x0010
#define YT8512_LED1_BT_ON_EN		0x0010
#define YT8512_LED1_TXACT_BLK_EN	0x0100
#define YT8512_LED1_RXACT_BLK_EN	0x0200
#define YT8512_SPEED_MODE		0xc000
#define YT8512_DUPLEX			0x2000

#define YT8512_SPEED_MODE_BIT		14
#define YT8512_DUPLEX_BIT		13
#define YT8512_EN_SLEEP_SW_BIT	15

#define YT8521_EXTREG_SLEEP_CONTROL1	0x27
#define YT8521_EN_SLEEP_SW_BIT	15

#define YT8521_SPEED_MODE		0xc000
#define YT8521_DUPLEX			0x2000
#define YT8521_SPEED_MODE_BIT		14
#define YT8521_DUPLEX_BIT		13
#define YT8521_LINK_STATUS_BIT	10

#define YT8521_PHY_MODE_FIBER	1 /* fiber mode only */
#define YT8521_PHY_MODE_UTP	2 /* utp mode only */
#define YT8521_PHY_MODE_POLL	3 /* fiber and utp, poll mode */

/*
 * please make choice according to system design,
 * for Fiber only system, please define YT8521_PHY_MODE_CURR 1
 * for UTP only system, please define YT8521_PHY_MODE_CURR 2
 * for combo system, please define YT8521_PHY_MODE_CURR 3 
 */
#define YT8521_PHY_MODE_CURR	YT8521_PHY_MODE_POLL

static int ytphy_read_ext(struct phy_device *phydev, u32 regnum)
{
	int ret;

	ret = phy_write(phydev, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	return phy_read(phydev, REG_DEBUG_DATA);
}

static int ytphy_write_ext(struct phy_device *phydev, u32 regnum, u16 val)
{
	int ret;

	ret = phy_write(phydev, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	return phy_write(phydev, REG_DEBUG_DATA, val);
}

static int yt8010_config_aneg(struct phy_device *phydev)
{
	phydev->speed = SPEED_100;
	return 0;
}

static int yt8512_led_init(struct phy_device *phydev)
{
	int ret;
	int val;
	int mask;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_LED0);
	if (val < 0)
		return val;

	val |= YT8512_LED0_ACT_BLK_IND;
	mask = YT8512_LED0_DIS_LED_AN_TRY | YT8512_LED0_BT_BLK_EN |
		YT8512_LED0_HT_BLK_EN | YT8512_LED0_COL_BLK_EN |
		YT8512_LED0_BT_ON_EN;
	val &= ~mask;

	ret = ytphy_write_ext(phydev, YT8512_EXTREG_LED0, val);
	if (ret < 0)
		return ret;

	val = ytphy_read_ext(phydev, YT8512_EXTREG_LED1);
	if (val < 0)
		return val;

	val |= YT8512_LED1_BT_ON_EN;

	mask = YT8512_LED1_TXACT_BLK_EN | YT8512_LED1_RXACT_BLK_EN;
	val &= ~mask;

	ret = ytphy_write_ext(phydev, YT8512_LED1_BT_ON_EN, val);

	return ret;
}

static int yt8512_config_init(struct phy_device *phydev)
{
	int ret;
	int val;

	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;

	ret = yt8512_led_init(phydev);
	if (ret < 0)
		return ret;

	/* disable auto sleep */
	val = ytphy_read_ext(phydev, YT8512_EXTREG_SLEEP_CONTROL1);
	if (val < 0)
		return val;

	val &= (~BIT(YT8512_EN_SLEEP_SW_BIT));
	ret = ytphy_write_ext(phydev, YT8512_EXTREG_SLEEP_CONTROL1, val);
	if (ret < 0)
		return ret;

	return ret;
}

static int yt8512_read_status(struct phy_device *phydev)
{
	int ret;
	int val;
	int speed, speed_mode, duplex;

	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	duplex = (val & YT8512_DUPLEX) >> YT8512_DUPLEX_BIT;
	speed_mode = (val & YT8512_SPEED_MODE) >> YT8512_SPEED_MODE_BIT;
	switch (speed_mode) {
	case 0:
		speed = SPEED_10;
		break;
	case 1:
		speed = SPEED_100;
		break;
	case 2:
	case 3:
	default:
		speed = SPEED_UNKNOWN;
		break;
	}

	phydev->speed = speed;
	phydev->duplex = duplex;

	return 0;
}

int yt8521_soft_reset(struct phy_device *phydev)
{
	int ret;

	ytphy_write_ext(phydev, 0xa000, 0);
	ret = genphy_soft_reset(phydev);
	if (ret < 0)
		return ret;

	ytphy_write_ext(phydev, 0xa000, 2);
	ret = genphy_soft_reset(phydev);
	if (ret < 0) {
		ytphy_write_ext(phydev, 0xa000, 0);
		return ret;
	}

	return 0;
}

static int ytphy_mii_rd_ext(struct mii_bus *bus, int phy_id, u32 regnum)
{
	int ret, val;

	ret = bus->write(bus, phy_id, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	val = bus->read(bus, phy_id, REG_DEBUG_DATA);

	return val;
}

static int ytphy_mii_wr_ext(struct mii_bus *bus, int phy_id, u32 regnum, u16 val)
{
	int ret;

	ret = bus->write(bus, phy_id, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	ret = bus->write(bus, phy_id, REG_DEBUG_DATA, val);

	return ret;
}

static int yt8511_config_delay(struct mii_bus *bus, int phy_id, bool tx, bool rx)
{
	int ret, val;

	/* disable auto sleep */
	val = ytphy_mii_rd_ext(bus, phy_id, 0x27);
	if (val < 0)
		return val;

	val &= (~BIT(15));
	ret = ytphy_mii_wr_ext(bus, phy_id, 0x27, val);
	if (ret < 0)
		return ret;

	val = ytphy_mii_rd_ext(bus, phy_id, 0xc);
	if (val < 0)
		return val;

	if (!tx) {
		/* disable tx delay */
		val &= ~(0xf << 4);
		ret = ytphy_mii_wr_ext(bus, phy_id, 0xc, val);
		if (ret < 0)
			return ret;
	}

	/* disable rx delay */
	val = ytphy_mii_rd_ext(bus, phy_id, 0xc);
	if (val < 0)
		return val;

	/* disable rx delay */
	if (!rx)
		val &= ~BIT(0);
	else
		val |= BIT(0);

	ret = ytphy_mii_wr_ext(bus, phy_id, 0xc, val);
	if (ret < 0)
		return ret;

	return ret;
}

static int yt8511_config_out_125m(struct mii_bus *bus, int phy_id)
{
	int ret, val;

	/* disable auto sleep */
	val = ytphy_mii_rd_ext(bus, phy_id, 0x27);
	if (val < 0)
		return val;

	val &= (~BIT(15));
	ret = ytphy_mii_wr_ext(bus, phy_id, 0x27, val);
	if (ret < 0)
		return ret;

	val = ytphy_mii_rd_ext(bus, phy_id, 0xc);
	if (val < 0)
		return val;

	/*
	 * ext reg 0xc.b[2:1]
	 * 00-----25M from pll;
	 * 01---- 25M from xtl;(default)
	 * 10-----62.5M from pll;
	 * 11----125M from pll(here set to this value)
	 */
	val |= (3 << 1);
	val &= ~BIT(3);
	ret = ytphy_mii_wr_ext(bus, phy_id, 0xc, val);

	return ret;
}

static int yt8511_config_init(struct phy_device *phydev)
{
	int ret;

	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;

	/*
	 * enable TX-delay for rgmii-id and rgmii-txid,
	 * otherwise disable it
	 */
	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_ID ||
		phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID) {
		ret = yt8511_config_delay(phydev->mdio.bus,
					   phydev->mdio.addr,
					   false, true);
		if (ret < 0)
			return ret;
	} else {
		ret = yt8511_config_delay(phydev->mdio.bus,
					   phydev->mdio.addr,
					   false, false);
		if (ret < 0)
			return ret;
	}

	return yt8511_config_out_125m(phydev->mdio.bus, phydev->mdio.addr);
}

/* 
 * This is only for YT8521 fiber 100m mode to get/restore autoneg state
 * please change the definition of YT8521_NUM_OF_FIBER_PHY if there is
 * more 8521 phy used in system.
 */
#define YT8521_NUM_OF_FIBER_PHY	3
//static unsigned long autoneg_phydev[YT8521_NUM_OF_FIBER_PHY];
//static int autoneg_pre_val[YT8521_NUM_OF_FIBER_PHY];
//static int autoneg_changed[YT8521_NUM_OF_FIBER_PHY];
//static int autoneg_actual_num_phy = 0;

static int yt8521_config_init(struct phy_device *phydev)
{
	int ret, val;

	phydev->irq = PHY_POLL;

	printk("start %s\n",__func__);
	ytphy_write_ext(phydev, 0xa000, 0);

	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;

	/* disable auto sleep extreg 0x27 bit[15] = 0 */
	val = ytphy_read_ext(phydev, YT8521_EXTREG_SLEEP_CONTROL1);
	if (val < 0)
		return val;

	val &= (~BIT(YT8521_EN_SLEEP_SW_BIT));
	ret = ytphy_write_ext(phydev, YT8521_EXTREG_SLEEP_CONTROL1, val);
	if (ret < 0)
		return ret;

	
	//0xa012[5]=1 force output 125M mac_clk
	ret = ytphy_write_ext(phydev, 0x12, 0xa012);
	if (ret < 0)
		return ret;
		
	val = ytphy_read_ext(phydev, 0x1f);
	if (val < 0)
		return val;

	val |= (3 << 1);
	ret = ytphy_write_ext(phydev, 0x1f, val);
	if (ret < 0)
		return ret;

	/*start enable RXC clock when no wire plug */
	ret = ytphy_write_ext(phydev, 0xa000, 0);
	if (ret < 0)
		return ret;
		
	val = ytphy_read_ext(phydev, 0xc);
	if (val < 0)
		return val;

	val &= ~(1 << 12);
	ret = ytphy_write_ext(phydev, 0xc, val);
	if (ret < 0)
		return ret;
	
	/*disable phy rxdelay txdelay*/
	val = ytphy_read_ext(phydev, 0xa001);
	val &= ~(1 << 8); //bit[8]=0
	ret = ytphy_write_ext(phydev, 0xa001, val);
        if (ret < 0)
                return ret;
				
	val = ytphy_read_ext(phydev, 0xa003);
	val &= ~(0xf << 0); //bit[3:0]=0
	ret = ytphy_write_ext(phydev, 0xa003, val);
        if (ret < 0)
                return ret;

	printk("end %s\n",__func__);
	return ret;
}

static int yt8521_adjust_status(struct phy_device *phydev, int val, int is_utp)
{
	int speed_mode, duplex;
	int speed = SPEED_UNKNOWN;

	//printk("%s\n", __func__);

	duplex = (val & YT8512_DUPLEX) >> YT8521_DUPLEX_BIT;
	speed_mode = (val & YT8521_SPEED_MODE) >> YT8521_SPEED_MODE_BIT;
	switch (speed_mode) {
	case 0:
		if (is_utp)
			speed = SPEED_10;
		break;
	case 1:
		speed = SPEED_100;
		break;
	case 2:
		speed = SPEED_1000;
		break;
	case 3:
		break;
	default:
		speed = SPEED_UNKNOWN;
		break;
	}

	phydev->speed = speed;
	phydev->duplex = duplex;

	return 0;
}

/* workaround for 8521 fiber 100m mode */
static int link_mode_8521 = 0; //0: no link; 1: utp; 32: fiber. traced that 1000m fiber uses 32.

int yt8521_aneg_done (struct phy_device *phydev)
{
	printk("%s\n", __func__);
	if((32 == link_mode_8521) && (SPEED_100 == phydev->speed))
		return 1; /*link_mode_8521*/

	return genphy_aneg_done(phydev);
}

static int yt8521_read_status(struct phy_device *phydev)
{
	int ret;
	volatile int val, yt8521_fiber_latch_val, yt8521_fiber_curr_val;
	volatile int link;
	int link_utp = 0, link_fiber = 0;

	//printk("%s\n", __func__);

#if (YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)
	/* reading UTP */
	ret = ytphy_write_ext(phydev, 0xa000, 0);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;

	link = val & (BIT(YT8521_LINK_STATUS_BIT));
	if (link) {
		link_utp = 1;
		link_mode_8521 = 1;
		yt8521_adjust_status(phydev, val, 1);
	} else {
		link_utp = 0;
	}
#endif //(YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)

#if (YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_UTP)
	/* reading Fiber */
	ret = ytphy_write_ext(phydev, 0xa000, 2);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_PHY_SPEC_STATUS);
	if (val < 0)
		return val;
	
	//printk (KERN_INFO "yzhang..8521 read fiber status=%04x,macbase=0x%08lx\n", val,(unsigned long)phydev->attached_dev);

	/* for fiber, from 1000m to 100m, there is not link down from 0x11, and check reg 1 to identify such case */	
	yt8521_fiber_latch_val = phy_read(phydev, MII_BMSR);
	yt8521_fiber_curr_val = phy_read(phydev, MII_BMSR);
	link = val & (BIT(YT8521_LINK_STATUS_BIT));
	if((link) && (yt8521_fiber_latch_val != yt8521_fiber_curr_val))
	{
		link = 0;
		printk (KERN_INFO "yzhang..8521 fiber link down detect,latch=%04x,curr=%04x\n", yt8521_fiber_latch_val,yt8521_fiber_curr_val);
	}
	
	if (link) {
		link_fiber = 1;
		yt8521_adjust_status(phydev, val, 0);
		link_mode_8521 = 32; //fiber mode


	} else {
		link_fiber = 0;
	}
#endif //(YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_UTP)

	if (link_utp || link_fiber) {
		phydev->link = 1;
	} else {
		phydev->link = 0;
		link_mode_8521 = 0;
	}

	if (link_utp) {
		ytphy_write_ext(phydev, 0xa000, 0);
	}

	return 0;
}

int yt8521_suspend(struct phy_device *phydev)
{
	int value;

	ytphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 2);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value | BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 0);

	return 0;
}

int yt8521_resume(struct phy_device *phydev)
{
	int value;

	ytphy_write_ext(phydev, 0xa000, 0);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 2);
	value = phy_read(phydev, MII_BMCR);
	phy_write(phydev, MII_BMCR, value & ~BMCR_PDOWN);

	ytphy_write_ext(phydev, 0xa000, 0);

	return 0;
}

static struct phy_driver ytphy_drvs[] = {
	{
		.phy_id	= PHY_ID_YT8010,
		.name		= "YT8010 Automotive Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
		.features	= PHY_BASIC_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= yt8010_config_aneg,
		.config_init	= genphy_config_init,
		.read_status	= genphy_read_status,
	}, {
		.phy_id	= PHY_ID_YT8510,
		.name		= "YT8510 100/10Mb Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
		.features	= PHY_BASIC_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= genphy_config_aneg,
		.config_init	= genphy_config_init,
		.read_status	= genphy_read_status,
	}, {
		.phy_id	= PHY_ID_YT8511,
		.name		= "YT8511 Gigabit Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
		.features	= PHY_GBIT_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= genphy_config_aneg,
		.config_init	= yt8511_config_init,
		.read_status	= genphy_read_status,
		.suspend	= genphy_suspend,
		.resume	= genphy_resume,
	}, {
		.phy_id	= PHY_ID_YT8512,
		.name		= "YT8512 Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
		.features	= PHY_BASIC_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= genphy_config_aneg,
		.config_init	= yt8512_config_init,
		.read_status	= yt8512_read_status,
		.suspend	= genphy_suspend,
		.resume	= genphy_resume,
	}, {
		.phy_id	= PHY_ID_YT8512B,
		.name		= "YT8512B Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
		.features	= PHY_BASIC_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= genphy_config_aneg,
		.config_init	= yt8512_config_init,
		.read_status	= yt8512_read_status,
		.suspend	= genphy_suspend,
		.resume	= genphy_resume,
	}, {
		.phy_id	= PHY_ID_YT8521,
		.name		= "YT8521 Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
		.features	= PHY_BASIC_FEATURES | PHY_GBIT_FEATURES,
		.flags		= PHY_POLL,
		.soft_reset	= yt8521_soft_reset,
		.config_aneg	= genphy_config_aneg,
		.aneg_done	= yt8521_aneg_done,
		.config_init	= yt8521_config_init,
		.read_status	= yt8521_read_status,
		.suspend	= yt8521_suspend,
		.resume	= yt8521_resume,
	},
};

module_phy_driver(ytphy_drvs);

MODULE_DESCRIPTION("Motorcomm PHY driver");
MODULE_AUTHOR("Leilei Zhao");
MODULE_LICENSE("GPL");

static struct mdio_device_id __maybe_unused motorcomm_tbl[] = {
	{ PHY_ID_YT8010, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8510, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8511, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8512, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8512B, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8521, MOTORCOMM_PHY_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, motorcomm_tbl);


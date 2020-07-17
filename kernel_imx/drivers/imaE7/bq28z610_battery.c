/*
 * BQ28Z610 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>

 * Copyright (C) 2018 Zafer Kaya <zaferkaya1960@hotmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#include <linux/timer.h>
#include <linux/jiffies.h>


#define DRIVER_VERSION			"1.2.0"

#define BQ28Z610_REG_TEMP		0x06
#define BQ28Z610_REG_VOLT		0x08
#define BQ28Z610_REG_AI			0x0C
#define BQ28Z610_REG_AVGC		0x14
#define BQ28Z610_REG_FLAGS		0x0A
#define BQ28Z610_REG_TTE		0x16
#define BQ28Z610_REG_TTF		0x18
#define BQ28Z610_REG_NAC		0x10 /* Nominal available capacity */
#define BQ28Z610_REG_LMD		0x12 /* Last measured discharge */
#define BQ28Z610_REG_CYCT		0x2A /* Cycle count total */
#define BQ28Z610_POWER_AVG		0x22

#define BQ28Z610_REG_RSOC		0x2C /* Relative State-of-Charge */
#define BQ28Z610_REG_DCAP		0x3C /* Design capacity */

#define BQ28Z610_FLAG_FD		BIT(4)	/* Final End-of-Discharge-Voltage flag */
#define BQ28Z610_FLAG_RCA		BIT(9)
#define BQ28Z610_FLAG_FC		BIT(5)
#define BQ28Z610_FLAG_CHGS		BIT(6)	/* Charge state flag */

struct bq28z610_data {
	struct device 		*dev;

	struct power_supply	*bat;
	struct power_supply_desc bat_desc;

	struct power_supply     *chg;
	struct power_supply_desc chg_desc;

	struct mutex		f_reg_lock;
	struct delayed_work	chg_work;

};


static enum power_supply_property bq28z610_charger_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property bq28z610_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_SHIPMENT,
};


static int voltage = 0;
static int status  = 0;
extern void (*pm_battery_off)(void);

//	u32 *X = ioremap(0x30240000, 4);
//        if ((readl(X) & (1 << 13)) == 0)

static void bq28z610_battery_off(void)
{
	struct i2c_msg msg[1];
	u32 cmd = 0x00001000;			
	msg[0].addr = 0x55;
	msg[0].flags = 0;
	msg[0].buf = &cmd;
	msg[0].len = 3;

        i2c_transfer(i2c_get_adapter(0), &msg, 1);
//        i2c_transfer(i2c_get_adapter(0), &msg, 1);
} 

static int bq28z610_read(struct bq28z610_data *data, u8 reg, bool single)
{
	struct i2c_client *client = to_i2c_client(data->dev);
	struct i2c_msg msg[2];
	unsigned char buf[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	if (!single)
		ret = get_unaligned_le16(buf);
	else
		ret = buf[0];

	return ret;

}
/*
typedef struct {
	int voltage;
	int percent;
} battery_capacity , *pbattery_capacity;

static battery_capacity rsocTable[] = {
	{4050*2000,	100},
	{4035*2000,	99},
	{4020*2000,	98},
	{4010*2000,	97},
	{4000*2000,	96},
	{3990*2000,	96},
	{3980*2000,	95},
{3976*2000,	94},
{3973*2000,	93},
	{3970*2000,	92},
	{3960*2000,	91},
	{3950*2000,	90},
{3945*2000,	89},
	{3940*2000,	88},
{3935*2000,	87},
	{3930*2000,	86},
{3925*2000,	85},
	{3920*2000,	84},
{3915*2000,	83},
	{3910*2000,	82},
{3905*2000,	81},
	{3900*2000,	80},
{3898*2000,	79},
{3897*2000,	78},
{3895*2000,	77},
{3893*2000,	76},
{3892*2000,	75},
	{3890*2000,	74},
{3884*2000,	73},
{3878*2000,	72},
{3872*2000,	71},
{3866*2000,	70},
	{3860*2000,	69},
{3854*2000,	68},
{3848*2000,	67},
{3842*2000,	66},
{3836*2000,	65},
	{3830*2000,	64},
{3820*2000,	63},
{3810*2000,	62},
{3800*2000,	61},
{3790*2000,	60},
	{3780*2000,	59},
{3776*2000,	58},
{3772*2000,	57},
{3768*2000,	56},
{3764*2000,	55},
	{3760*2000,	54},
{3756*2000,	53},
{3752*2000,	52},
{3748*2000,	51},
{3744*2000,	50},
	{3740*2000,	49},
{3736*2000,	48},
{3732*2000,	47},
{3728*2000,	46},
{3724*2000,	45},
	{3720*2000,	44},
{3716*2000,	43},
{3712*2000,	42},
{3708*2000,	41},
{3704*2000,	40},
	{3700*2000,	39},
{3696*2000,	38},
{3692*2000,	37},
{3688*2000,	36},
{3684*2000,	35},
	{3680*2000,	34},
{3676*2000,	33},
{3672*2000,	32},
{3668*2000,	31},
{3664*2000,	30},
	{3660*2000,	29},
{3656*2000,	28},
{3652*2000,	27},
{3648*2000,	26},
{3644*2000,	25},
	{3640*2000,	24},
{3636*2000,	23},
{3632*2000,	22},
{3628*2000,	21},
{3624*2000,	20},
	{3620*2000,	19},
{3616*2000,	18},
{3612*2000,	17},
{3608*2000,	16},
{3604*2000,	15},
	{3600*2000,	14},
	{3580*2000,	13},
	{3560*2000,	12},
	{3540*2000,	11},
	{3520*2000,	10},
	{3500*2000,	9},
	{3480*2000,	8},
	{3460*2000,	7},
	{3440*2000,	6},
	{3430*2000,	5},
	{3420*2000,	4},
{3320*2000,	3},
{3220*2000,	2},
{3120*2000,	1},
	{3020*2000,	0},
};

static int bq28z610_rsoc_check(int rsoc)
{
	int i;
	int tableSize = sizeof(rsocTable)/sizeof(rsocTable[0]);

	if ((rsoc < 1) || (rsoc > 99)){
		for (i = 0; i < tableSize; i++) {
			if (voltage >= rsocTable[i].voltage) {
				rsoc = rsocTable[i].percent;
				break;
			}
		}
	}
	return rsoc;
}
*/
/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq28z610_battery_read_rsoc(struct bq28z610_data *data,
	union power_supply_propval *val)
{
	int rsoc;

	rsoc = bq28z610_read(data, BQ28Z610_REG_RSOC, true);
	if (rsoc < 0) {
		dev_dbg(data->dev, "error reading relative State-of-Charge\n");
		return rsoc;
        }

//	rsoc = bq28z610_rsoc_check(rsoc);

////	if ((status == POWER_SUPPLY_STATUS_CHARGING) && (rsoc == 100)) rsoc = 99;
	if ((rsoc == 0) && (voltage >= 6000000)) rsoc = 1;
	          
	val->intval = rsoc;

	return 0;
}

/*
 * Return the battery current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq28z610_battery_current(struct bq28z610_data *data, u8 reg,
	union power_supply_propval *val)
{
	int curr;

	curr = bq28z610_read(data, reg, false);
	if (curr < 0) {
		dev_err(data->dev, "error reading current\n");
		return curr;
	}

	val->intval = (int)((s16)curr) * 1000;

	return 0;
}

/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq28z610_battery_read_uAh(struct bq28z610_data *data, u8 reg,
	union power_supply_propval *val)
{
	int charge;

	charge = bq28z610_read(data, reg, false);
	if (charge < 0) {
		dev_dbg(data->dev, "error reading charge register %02x: %d\n",reg, charge);
		return charge;
	}

	val->intval = charge * 1000;

	return 0;
}

/*
 * Read a power register. uWh
 * Return < 0 if something fails.
 */
static int bq28z610_battery_read_pwr(struct bq28z610_data *data, u8 reg,
	union power_supply_propval *val)
{
	int tval;

	tval = bq28z610_read(data, reg, false);
	if (tval < 0) {
		dev_err(data->dev, "error reading power register  %02x: %d\n",reg, tval);
		return tval;
	}

	val->intval = (int)((s16)tval) * 10000;

	return 0;
}

/*
 * Return the battery temperature in tenths of degree Celcius
 * Or < 0 if something fails.
 */
static int bq28z610_battery_read_temperature(struct bq28z610_data *data, u8 reg,
	union power_supply_propval *val)
{
	int temp;

	temp = bq28z610_read(data, reg, false);
	if (temp < 0) {
		dev_err(data->dev, "error reading temperature\n");
		return temp;
	}
	
	val->intval = (int)((s16)temp) - 2730;

	return 0;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq28z610_battery_read_time(struct bq28z610_data *data, u8 reg,
	union power_supply_propval *val)
{
	int tval;

	tval = bq28z610_read(data, reg, false);
	if (tval < 0) {
		dev_dbg(data->dev, "error reading time register %02x: %d\n",reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	val->intval = tval * 60;

	return 0;
}


/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq28z610_battery_read_cyct(struct bq28z610_data *data,
	union power_supply_propval *val)
{
	int cyct;

	cyct = bq28z610_read(data, BQ28Z610_REG_CYCT, false);
	if (cyct < 0) {
		dev_err(data->dev, "error reading cycle count total\n");
		return cyct;
	}

	val->intval = cyct;

	return 0;
}


static int bq28z610_battery_status(struct bq28z610_data *data,
	union power_supply_propval *val)
{
	int newstatus;

	int curr = bq28z610_read(data, BQ28Z610_REG_AI, false);

	if (curr < 0) {
		dev_err(data->dev, "error reading current\n");
		return curr;
	}

	if (curr & 0x7FFF) {
		if (curr & 0x8000) 
			newstatus = POWER_SUPPLY_STATUS_DISCHARGING;
		else 
			newstatus = POWER_SUPPLY_STATUS_CHARGING;
	}
	else { 
		if (bq28z610_read(data, BQ28Z610_REG_FLAGS, true) & BQ28Z610_FLAG_FC)
			newstatus = POWER_SUPPLY_STATUS_FULL;
		else 
			newstatus = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

        if (newstatus != status) {
		status = newstatus;
		power_supply_changed(data->chg);
		power_supply_changed(data->bat);
	}

	val->intval = status;


/*
	if (status == POWER_SUPPLY_STATUS_FULL)
		printk("bq28z610_battery_status FULL\n");
	else if (status == POWER_SUPPLY_STATUS_DISCHARGING)		
		printk("bq28z610_battery_status DISCHARGING\n");		
	else if (status == POWER_SUPPLY_STATUS_CHARGING)		
		printk("bq28z610_battery_status CHARGING\n");		
	else if (status == POWER_SUPPLY_STATUS_NOT_CHARGING)		
		printk("bq28z610_battery_status NOTCHARGING\n");		
*/
	return 0;
}

static int bq28z610_battery_capacity_level(struct bq28z610_data *data,
	union power_supply_propval *val)
{
	int flags;

        flags = bq28z610_read(data, BQ28Z610_REG_FLAGS, false);
	
        if (flags & BQ28Z610_FLAG_FC)
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (flags & (BQ28Z610_FLAG_FD | BQ28Z610_FLAG_RCA))
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

	return 0;
}

/*
 * Return the battery Voltage in uvolts
 * Or < 0 if something fails.
 */
static int bq28z610_battery_voltage(struct bq28z610_data *data, u8 reg,
	union power_supply_propval *val)
{
	int volt;

	volt = bq28z610_read(data, reg, false);
	if (volt < 0) {
		dev_err(data->dev, "error reading voltage\n");
		return volt;
	}

	voltage = val->intval = volt * 1000;
        
//	printk("bq28z610_battery_voltage %d\n",volt);		

	return 0;
}

static int bq28z610_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	struct bq28z610_data *data = power_supply_get_drvdata(psy);

//	mutex_lock(&data->f_reg_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
                break;
	case POWER_SUPPLY_PROP_ONLINE:
//		printk("bq28z610_CHG_ONLINE\n");
		if (status == POWER_SUPPLY_STATUS_DISCHARGING)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	default:
		ret = -EINVAL;
	}

//	printk("bq28z610 %d\n",val->intval);

//	mutex_unlock(&data->f_reg_lock);

	return ret;
}

static int bq28z610_battery_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_SHIPMENT:
//		printk("bq28z610_SHIPMENT\n");
		if (val->intval)
			pm_battery_off = bq28z610_battery_off;
		else
			pm_battery_off = NULL;
		return 0;
	default:
//		printk("bq28z610_EINVAL!\n");
		return -EINVAL;
	}
}

static int bq28z610_battery_property_is_writeable(struct power_supply *psy,
				     enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_SHIPMENT:
		return 1;
	default:
		return 0;
	}
}


static int bq28z610_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;


	struct bq28z610_data *data = power_supply_get_drvdata(psy);

	mutex_lock(&data->f_reg_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
//		printk("bq28z610_STATUS\n");
		ret = bq28z610_battery_status(data, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
//		printk("bq28z610_VOLTAGE_NOW\n");
		ret = bq28z610_battery_voltage(data, BQ28Z610_REG_VOLT, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
//		printk("bq28z610_PRESENT\n");
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
//		printk("bq28z610_CURRENT_NOW\n");
		ret = bq28z610_battery_current(data,BQ28Z610_REG_AI, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
//		printk("bq28z610_CURRENT_AVG\n");
		ret = bq28z610_battery_current(data,BQ28Z610_REG_AVGC, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
//		printk("bq28z610_CAPACITY\n");
		ret = bq28z610_battery_read_rsoc(data, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
//		printk("bq28z610_CAPACITY_LEVEL\n");
		ret = bq28z610_battery_capacity_level(data, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
//		printk("bq28z610_TEMP\n");
		ret = bq28z610_battery_read_temperature(data,BQ28Z610_REG_TEMP, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
//		printk("bq28z610_TIME_TO_EMPTY_NOW\n");
		ret = bq28z610_battery_read_time(data, BQ28Z610_REG_TTE, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
//		printk("bq28z610_TIME_TO_EMPTY_AVG\n");
		ret = bq28z610_battery_read_time(data, BQ28Z610_REG_TTE, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
//		printk("bq28z610_TIME_TO_FULL_NOW\n");
		ret = bq28z610_battery_read_time(data, BQ28Z610_REG_TTF, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
//		printk("bq28z610_TECHNOLOGY\n");
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
//		printk("bq28z610_CHARGE_NOW\n");
		ret = bq28z610_battery_read_uAh(data, BQ28Z610_REG_NAC, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
//		printk("bq28z610_CHARGE_FULL\n");
		ret = bq28z610_battery_read_uAh(data, BQ28Z610_REG_LMD, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
//		printk("bq28z610_FULL_DESIGN\n");
		ret = bq28z610_battery_read_uAh(data, BQ28Z610_REG_DCAP, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
//		printk("bq28z610_CYCLE_COUNT\n");
		ret = bq28z610_battery_read_cyct(data, val);
		break;
	case POWER_SUPPLY_PROP_POWER_AVG:
//		printk("bq28z610_POWER_AVG\n");
		ret = bq28z610_battery_read_pwr(data,BQ28Z610_POWER_AVG, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
//		printk("bq28z610_HEALTH\n");
                val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_SHIPMENT:
//		printk("bq28z610_SHIPMENT\n");
		if (pm_battery_off == bq28z610_battery_off)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
//		printk("bq28z610_EINVAL!\n");
		ret = -EINVAL;
	}

//	printk("bq28z610 %d\n",val->intval);

	mutex_unlock(&data->f_reg_lock);

	return ret;
}

static void bq28z610_check_charger_work(struct work_struct *work)
{
	struct bq28z610_data *data;
	struct delayed_work *delayed_work;

	delayed_work = to_delayed_work(work);
	data = container_of(delayed_work, struct bq28z610_data, chg_work);

	int newstatus;

//	dev_err(data->dev, "chk_chg_work enter\n");

	mutex_lock(&data->f_reg_lock);
	int curr = bq28z610_read(data, BQ28Z610_REG_AI, false);
	mutex_unlock(&data->f_reg_lock);

	if (curr < 0) {
		dev_err(data->dev, "error reading current\n");
		return curr;
	}

	if (curr & 0x7FFF) {
		if (curr & 0x8000) 
			newstatus = POWER_SUPPLY_STATUS_DISCHARGING;
		else 
			newstatus = POWER_SUPPLY_STATUS_CHARGING;
	}
	else { 
		if (bq28z610_read(data, BQ28Z610_REG_FLAGS, true) & BQ28Z610_FLAG_FC)
			newstatus = POWER_SUPPLY_STATUS_FULL;
		else 
			newstatus = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

        if (newstatus != status) {
		status = newstatus;
		power_supply_changed(data->chg);
		power_supply_changed(data->bat);
//	dev_err(data->dev, "chk_chg_work signal change of status\n");
	}

	if (newstatus == POWER_SUPPLY_STATUS_DISCHARGING)
		schedule_delayed_work(&data->chg_work, msecs_to_jiffies(10000));
	else
		schedule_delayed_work(&data->chg_work, msecs_to_jiffies(1000));
}


static int bq28z610_battery_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bq28z610_data *data;
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) return -ENOMEM;

	psy_cfg.drv_data = data;

	data->dev = dev;

	mutex_init(&data->f_reg_lock);

	data->chg_desc.name           = "bq28z610-chg";
	data->chg_desc.type           = POWER_SUPPLY_TYPE_MAINS;
	data->chg_desc.properties     = bq28z610_charger_props;
	data->chg_desc.num_properties = ARRAY_SIZE(bq28z610_charger_props);
	data->chg_desc.get_property   = bq28z610_charger_get_property;

	data->chg = power_supply_register(dev, &data->chg_desc, &psy_cfg);
	if (IS_ERR(data->chg)) {
		dev_err(dev, "failed: power supply register(chg).\n");
		goto err_chg;
	}

	data->bat_desc.name           = "bq28z610-bat";
	data->bat_desc.type           = POWER_SUPPLY_TYPE_BATTERY;
	data->bat_desc.properties     = bq28z610_battery_props;
	data->bat_desc.num_properties = ARRAY_SIZE(bq28z610_battery_props);
	data->bat_desc.get_property   = bq28z610_battery_get_property;
	data->bat_desc.set_property   = bq28z610_battery_set_property;
	data->bat_desc.property_is_writeable = bq28z610_battery_property_is_writeable;

	data->bat = power_supply_register(dev, &data->bat_desc, &psy_cfg);
	if (IS_ERR(data->bat)) {
		dev_err(dev, "failed: power supply register(bat).\n");
		goto err_bat;
	}

	i2c_set_clientdata(client, data);

	INIT_DELAYED_WORK(&data->chg_work, bq28z610_check_charger_work);

	schedule_delayed_work(&data->chg_work, msecs_to_jiffies(0));

//	dev_dbg(dev, "bq28z610 probed.\n");
	return 0;

err_bat:
	power_supply_unregister(data->bat);
err_chg:
	power_supply_unregister(data->chg);
	return -1;
}

static int bq28z610_battery_remove(struct i2c_client *client)
{
	struct bq28z610_data *data = i2c_get_clientdata(client);

	power_supply_unregister(data->chg);
	power_supply_unregister(data->bat);
		
	kfree(data);

	return 0;
}

static const struct i2c_device_id bq28z610_id[] = {
	{ "bq28z610", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq28z610_id);




#ifdef CONFIG_PM
static int bq28z610_suspend(struct device *dev)
{
	struct bq28z610_data *data = dev_get_drvdata(dev);

//	dev_err(data->dev, "bq28z610_suspend\n");

	cancel_delayed_work_sync(&data->chg_work);
	return 0;
}

static int bq28z610_resume(struct device *dev)
{
	struct bq28z610_data *data = dev_get_drvdata(dev);

//	dev_err(data->dev, "bq28z610_resume\n");

	schedule_delayed_work(&data->chg_work, msecs_to_jiffies(0));
	return 0;
}

static const struct dev_pm_ops bq28z610_pm_ops = {
	.suspend        = bq28z610_suspend,
	.resume         = bq28z610_resume,
};

#define BQ28Z610_PM_OPS       (&bq28z610_pm_ops)
#else
#define BQ28Z610_PM_OPS       (NULL)
#endif

static struct i2c_driver bq28z610_battery_driver = {
	.driver = {
		.name = "bq28z610-battery",
		.pm   = BQ28Z610_PM_OPS
	},
	.probe = bq28z610_battery_probe,
	.remove = bq28z610_battery_remove,
	.id_table = bq28z610_id,
};

static int __init bq28z610_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq28z610_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ28Z610 i2c driver\n");

	return ret;
}

static void __exit bq28z610_battery_i2c_exit(void)
{
	i2c_del_driver(&bq28z610_battery_driver);
}

module_init(bq28z610_battery_i2c_init);
module_exit(bq28z610_battery_i2c_exit);

MODULE_ALIAS("i2c:bq28z610-battery");
MODULE_AUTHOR("Zafer Kaya <ima.com.tr>");
MODULE_DESCRIPTION("BQ28Z610 battery monitor driver");
MODULE_LICENSE("GPL");

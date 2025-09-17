/*
 * (C) Copyright 2020 Rockchip Electronics Co., Ltd
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <i2c.h>
#include <asm/gpio.h>
#include "rk_ebc.h"

struct sy7673a_priv_data {
    struct udevice *dev;
    struct gpio_desc chip_en_gpio;
    struct gpio_desc vcom_ctrl_gpio;
    struct gpio_desc power_good_gpio;
    struct gpio_desc p_vdd_en_gpio;
    struct gpio_desc pmicon_gpio;
    u8 rev_id;
    u8 vadj;
    u8 vcom1;
    u8 vcom2;
};

#define msleep(a)                               udelay((a) * 1000)
#define SY7673A_MAX_REG_NUMBER                  9

//register name and address
#define SY7673A_OPERATION_MODE_CTRL             0x00
#define SY7673A_VCOM_ADJUSTMENT_CTRL01          0x01
#define SY7673A_VCOM_ADJUSTMENT_CTRL02          0x02
#define SY7673A_VLDO_VOLTAGE_ADJUSTMENT_CTRL    0x03
#define SY7673A_POWER_ON_DELAY_TIME             0x06
#define SY7673A_FAULT_FLAG                      0x07
#define SY7673A_THERMISTOR_READ_OUT             0x08
#define SY7673A_GET_STATE_INTERVAL_MS           10
#define Operation_Mode_Control                  0x0
#define OMC_OnOff                               0x80
#define OMC_VcomCtl                             0x40
#define OMC_VDDH_En                             0x10
#define OMC_VPOS_En                             0x04
#define OMC_VNEG_En                             0x02
#define OMC_VCOM_En                             0x01
#define VCOM_Adjustment_Control_1               0x1
#define VCOM_Adjustment_Control_2               0x2
#define VLDO_Voltage_Adjustment_Control         0x3
#define Power_On_Delay_Time                     0x6
#define Fault_Flag                              0x7
#define Thermistor_Readout                      0x8
#define mv_to_vcom1_reg(mv)                     (((mv) / 10) & 0xff)
#define mv_to_vcom2_reg(mv)                     (((((mv) / 10) & 0x100) >> 8)<< 7)


static struct udevice *sy7673a_dev;
static u32 pre_temp = 25;
static int sy7673a_pwr_up_flag = 0;

static int sy7673a_i2c_write(struct sy7673a_priv_data *priv_data, u8 reg, u8 val)
{
    int ret;
    u8 buf[2];
    struct i2c_msg msg;
    struct dm_i2c_chip *chip = dev_get_parent_platdata(priv_data->dev);

    buf[0] = reg;
    buf[1] = val;
    msg.addr = chip->chip_addr;
    msg.flags = 0;
    msg.len = 2;
    msg.buf = buf;

    ret = dm_i2c_xfer(priv_data->dev, &msg, 1);
    if (ret) {
        printf("davidpeng,%s,Line:%d,sy7673a i2c write failed: %d\n", __func__, __LINE__, ret);
        return ret;
    }

    return 0;
}

static int sy7673a_i2c_read(struct sy7673a_priv_data *priv_data, u8 reg, u8 *val)
{
    int ret;
    u8 data;
    struct dm_i2c_chip *chip = dev_get_parent_platdata(priv_data->dev);
    struct i2c_msg msg[] = {
        {
            .addr = chip->chip_addr,
            .flags = 0,
            .buf = (u8 *) &reg,
            .len = 1,
        }, {
            .addr = chip->chip_addr,
            .flags = I2C_M_RD,
            .buf = (u8 *) &data,
            .len = 1,
        }
    };

    ret = dm_i2c_xfer(priv_data->dev, msg, 2);
    if (ret) {
        printf("davidpeng,%s,Line:%d,sy7673a i2c read failed: %d\n", __func__, __LINE__, ret);
        return ret;
    }

    *val = data;

    return 0;
}

#if 0
static void sy7673a_dump_registers(void)
{
    u8 i, reg = 0;
    struct sy7673a_priv_data *priv_data = dev_get_priv(sy7673a_dev);

    for (i = 0; i < SY7673A_MAX_REG_NUMBER; i++) {
        sy7673a_i2c_read(priv_data, i, &reg);
        printf("Reg[%d]:0x%02X\n", i, reg);
    }
}
#endif

static int sy7673a_read_vcom_value(struct sy7673a_priv_data *priv_data, u32 *vcom_read)
{
    int ret;
    u8 vcom_reg;

    printf("davidpeng,%s,Line:%d,ENTERING\n", __func__, __LINE__);
    ret = sy7673a_i2c_read(priv_data, SY7673A_VCOM_ADJUSTMENT_CTRL01, &vcom_reg);
    if (ret) {
        printf("davidpeng,%s,Line:%d,read vcom1 failed: %d\n", __func__, __LINE__, ret);
        return -1;
    }
    *vcom_read = vcom_reg;
    ret = sy7673a_i2c_read(priv_data, SY7673A_VCOM_ADJUSTMENT_CTRL02, &vcom_reg);
    if (ret) {
        printf("davidpeng,%s,Line:%d,read vcom2 failed: %d\n", __func__, __LINE__, ret);
        return -2;
    }
    *vcom_read += ((vcom_reg & 0x80) >> 7) << 8;

    printf("davidpeng,%s,Line:%d,read vcom value: %d\n", __func__, __LINE__, *vcom_read);

    return 0;
}

static int sy7673a_set_vcom_value(struct udevice *dev, u32 set_value)
{
    int ret = 0;
    u32 vcom_readback = 0;
    u32 temp = 0;
    u8 vcom1_val, vcom2_val;
    u8 reg_val;
    struct sy7673a_priv_data *priv_data = dev_get_priv(dev);

    printf("davidpeng,%s,Line:%d,entering, set_value:%d\n", __func__, __LINE__, set_value);

    temp = set_value / 10;

    // if VCOM voltage is less than -5000mv
    if (temp > 0x1F4) {
        temp = 0x1F4;
    }

    ret = sy7673a_read_vcom_value(priv_data, &vcom_readback);
    if (ret < 0) {
        printf("davidpeng,%s,Line:%d,sy7673a read vcom value failed,ret:%d\n", __func__, __LINE__, ret);
    } else {
        if (vcom_readback == temp) {
            printf("davidpeng,%s,Line:%d,Same as pmic default value, just return.\n", __func__, __LINE__);
            return 0;
        } else {
            printf("davidpeng,%s,Line:%d,now we will set the vcom value. the set value:%d\n", __func__, __LINE__, temp);
        }
    }

    vcom1_val = (u8)(temp & 0xFF);
    vcom2_val = (u8)((temp & 0xFF00) >> 8);

    ret = sy7673a_i2c_read(priv_data, SY7673A_VCOM_ADJUSTMENT_CTRL02, &reg_val);
    if (ret) {
        printf("davidpeng,%s,Line:%d,read vcom2 reg failed,return now\n", __func__, __LINE__);
        return -1;
    } else {
        vcom2_val = (reg_val & 0x7F) | (vcom2_val << 7);
    }

    printf("davidpeng,%s,Line:%d,vcom1_val:0x%02X,vcom2_val:0x%02X\n", __func__, __LINE__, vcom1_val, vcom2_val);

    // Set vcom voltage
    ret = sy7673a_i2c_write(priv_data, SY7673A_VCOM_ADJUSTMENT_CTRL01, vcom1_val);
    if (ret) {
        printf("davidpeng,%s,Line:%d,set vcom1 reg failed,ret:%d\n", __func__, __LINE__, ret);
    }

    ret = sy7673a_i2c_write(priv_data, SY7673A_VCOM_ADJUSTMENT_CTRL02, vcom2_val);
    if (ret) {
        printf("davidpeng,%s,Line:%d,set vcom2 reg failed,ret:%d\n", __func__, __LINE__, ret);
    }

    //read vcom voltage to verify the vcom value is correct or not
    ret = sy7673a_read_vcom_value(priv_data, &vcom_readback);
    if (ret < 0) {
        printf("davidpeng,%s,Line:%d,CONFIRM,sy7673a read vcom value failed,ret:%d\n", __func__, __LINE__, ret);
    } else {
        printf("davidpeng,%s,Line:%d,CONFIRM,sy7673a read vcom value OK, the reg value:%d\n", __func__, __LINE__, vcom_readback);
        if (vcom_readback != temp) {
            printf("davidpeng,%s,Line:%d,CONFIRM,the vcom reg value is not equal to the setting value\n", __func__, __LINE__);
            return -2;
        }
    }

    return 0;
}

#if 0
static bool sy7673a_hw_power_ack(struct sy7673a_priv_data *priv_data, int up)
{
    u8 pg_status;
    int st, ret, retries_left = 10;

    do {
        ret = sy7673a_i2c_read(priv_data, SY7673A_FAULT_FLAG, &pg_status);
        if (ret) {
            printf("davidpeng,%s,Line:%d,read REG_PG_STATUS failed: %d\n", __func__, __LINE__, ret);
        }

        printf("davidpeng,%s,Line:%d,SY7673A_FAULT_FLAG: 0x%02X\n", __func__, __LINE__, pg_status);

        pg_status &= 0x01;
        if (pg_status == 0x01 && up == 1) {
            st = 1;
            printf("davidpeng,%s,Line:%d,power up OK\n", __func__, __LINE__);
        } else if (pg_status == 0x00 && up == 0) {
            st = 0;
            printf("davidpeng,%s,Line:%d,power down OK\n", __func__, __LINE__);
        } else {
            st = -1;    /* not settled yet */
            msleep(SY7673A_GET_STATE_INTERVAL_MS);
        }
        retries_left--;
    } while ((st == -1) && retries_left);

    if ((st == -1) && !retries_left) {
        printf("davidpeng,%s,Line:%d,power %s settle error (PG = %02x)\n", __func__, __LINE__, up ? "up" : "down", pg_status);
    }

    return (st == up);
}
#else
static bool sy7673a_hw_check_power_good(struct sy7673a_priv_data *priv_data, int up)
{
    int reg_val = 0;
    int st, retries_left = 12;

    dm_gpio_set_dir_flags(&priv_data->power_good_gpio, GPIOD_IS_IN);

    do {
        reg_val = dm_gpio_get_value(&priv_data->power_good_gpio);

        printf("davidpeng,%s,Line:%d,power good signal: %d\n", __func__, __LINE__, reg_val);

        if (reg_val == 0x01 && up == 1) {
            st = 1;
            printf("davidpeng,%s,Line:%d,power up OK\n", __func__, __LINE__);
        } else if (reg_val == 0x00 && up == 0) {
            st = 0;
            printf("davidpeng,%s,Line:%d,power down OK\n", __func__, __LINE__);
        } else {
            st = -1;    /* not settled yet */
            msleep(SY7673A_GET_STATE_INTERVAL_MS);
        }
        retries_left--;
    } while ((st == -1) && retries_left);

    if ((st == -1) && !retries_left) {
        printf("davidpeng,%s,Line:%d,power %s settle error\n", __func__, __LINE__, up ? "up" : "down");
    } else {
        printf("davidpeng,%s,Line:%d,power %s settle OK\n", __func__, __LINE__, up ? "up" : "down");
    }

    return (st == up);
}
#endif

static int sy7673a_power_on(struct udevice *dev)
{
    struct sy7673a_priv_data *priv_data = dev_get_priv(dev);
    int ret = 0;
    u8 reg_val = 0;

    printf("davidpeng,%s,Line:%d,entering\n", __func__, __LINE__);

    dm_gpio_set_value(&priv_data->p_vdd_en_gpio, 1);
    msleep(5);

    ret = sy7673a_i2c_read(priv_data, SY7673A_OPERATION_MODE_CTRL, &reg_val);
    if (ret) {
        printf("davidpeng,%s,Line:%d, read reg[0x00] failed\n", __func__, __LINE__);
    }

    reg_val |= 0x80;
    ret |= sy7673a_i2c_write(priv_data, SY7673A_OPERATION_MODE_CTRL, reg_val);
    if (ret) {
        printf("davidpeng,%s,Line:%d, POWER ON all power rails failed\n", __func__, __LINE__);
    }

    printf("davidpeng,%s,Line:%d,before check power good\n", __func__, __LINE__);
#if 0
    sy7673a_hw_power_ack(priv_data, 1);
#else
    sy7673a_hw_check_power_good(priv_data, 1);
#endif
    printf("davidpeng,%s,Line:%d,after check power good\n", __func__, __LINE__);

    sy7673a_pwr_up_flag = 1;

    ret = sy7673a_i2c_read(priv_data, SY7673A_THERMISTOR_READ_OUT, &reg_val);
    if (ret < 0) { // read temperature failed
        printf("davidpeng,%s,Line:%d,read temperature failed\n", __func__, __LINE__);
    } else {
        pre_temp = (u32)reg_val;
        printf("davidpeng,%s,Line:%d,read temperature successfully, temperature: %d\n", __func__, __LINE__, reg_val);
    }

    return 0;
}

static int sy7673a_power_down(struct udevice *dev)
{
    struct sy7673a_priv_data *priv_data = dev_get_priv(dev);
    int ret = 0;
    u8 reg_val = 0;

    printf("davidpeng,%s,Line:%d,entering\n", __func__, __LINE__);

    sy7673a_pwr_up_flag = 0;

    ret = sy7673a_i2c_read(priv_data, SY7673A_OPERATION_MODE_CTRL, &reg_val);
    if (ret) {
        printf("davidpeng,%s,Line:%d, read reg[0x00] failed\n", __func__, __LINE__);
    }

    reg_val &= 0x7F;
    ret |= sy7673a_i2c_write(priv_data, SY7673A_OPERATION_MODE_CTRL, reg_val);
    if (ret) {
        printf("davidpeng,%s,Line:%d, POWER ON all power rails failed\n", __func__, __LINE__);
    }

#if 0
    sy7673a_hw_power_ack(priv_data, 0);
#else
    sy7673a_hw_check_power_good(priv_data, 0);
#endif
    dm_gpio_set_value(&priv_data->p_vdd_en_gpio, 0);

    return 0;
}

static int sy7673a_temp_get(struct udevice *dev, u32 *temp)
{
    int ret;
    u8 read_val = 0;
    struct sy7673a_priv_data *priv_data = dev_get_priv(dev);

    printf("davidpeng,%s,Line:%d, entering\n", __func__, __LINE__);

    if (sy7673a_pwr_up_flag == 0) {
        *temp = pre_temp;
        printf("davidpeng,%s,Line:%d,all power rails are powered off, use last temperature\n", __func__, __LINE__);
        return 0;;
    }

    ret = sy7673a_i2c_read(priv_data, SY7673A_THERMISTOR_READ_OUT, &read_val);
    if (ret) { // read temperature failed
        *temp = pre_temp;
        printf("davidpeng,%s,Line:%d,read temperature failed, use last temperature, ret:%d\n", __func__, __LINE__, ret);
    } else {
        *temp = (u32)read_val;
        printf("davidpeng,%s,Line:%d,read temperature successfully, temperature = %d\n", __func__, __LINE__, *temp);

        pre_temp = *temp;
        printf("davidpeng,%s,Line:%d,final temperature = %d\n", __func__, __LINE__, *temp);
    }

    return 0;
}

static int SY7636A_init_client(struct udevice *dev)
{
    struct sy7673a_priv_data *priv_data = dev_get_priv(dev);
    uint8_t reg_addr;
    uint8_t reg_data;

    //sy7673a_hw_setreg(s, SY7673A_VCOM_ADJUSTMENT_CTRL01, s->vcom1);

    reg_addr = Operation_Mode_Control;
    reg_data = OMC_OnOff;
    sy7673a_i2c_write(priv_data, reg_addr, reg_data);

    reg_addr = VCOM_Adjustment_Control_1;
    reg_data = 0x7d;
    sy7673a_i2c_write(priv_data, reg_addr, reg_data);

    reg_addr = VCOM_Adjustment_Control_2;
    reg_data = 0x0;
    sy7673a_i2c_write(priv_data, reg_addr, reg_data);

    reg_addr = VLDO_Voltage_Adjustment_Control;
    reg_data = 0x60;
    sy7673a_i2c_write(priv_data, reg_addr, reg_data);

    reg_addr = Power_On_Delay_Time;
    reg_data = 0xaa;
    sy7673a_i2c_write(priv_data, reg_addr, reg_data);

    return 0;
}

static int sy7673a_hw_init(struct udevice *dev)
{
    struct sy7673a_priv_data *priv_data = dev_get_priv(dev);

    printf("davidpeng,%s,Line:%d,entering\n", __func__, __LINE__);
    //vcom voltage disable
    dm_gpio_set_value(&priv_data->vcom_ctrl_gpio, 0);
    //chip enable
    dm_gpio_set_value(&priv_data->chip_en_gpio, 1);
    msleep(20);
    dm_gpio_set_value(&priv_data->p_vdd_en_gpio, 1);
    msleep(20);
    dm_gpio_set_value(&priv_data->pmicon_gpio, 1);
    msleep(20);
    sy7673a_pwr_up_flag = 0;
    SY7636A_init_client(dev);

    return 0;
}

static void sy7673a_init_arg(struct sy7673a_priv_data *priv_data)
{
    priv_data->vcom1 = mv_to_vcom1_reg(1560);
    priv_data->vcom2 = mv_to_vcom2_reg(1560);
}

static int sy7673a_probe(struct udevice *dev)
{
    int ret;
    struct sy7673a_priv_data *sy7673a_priv = dev_get_priv(dev);

    sy7673a_priv->dev = dev;
    sy7673a_dev = dev;
    sy7673a_init_arg(sy7673a_priv);

    printk("sy7636a probe\n");

    ret = gpio_request_by_name(dev, "powerup-gpios", 0, &sy7673a_priv->chip_en_gpio, GPIOD_IS_OUT);
    if (ret) {
        printf("davidpeng,%s,Line:%d,Can not get powerup-gpios: %d\n", __func__, __LINE__, ret);
        return ret;
    }
    ret = gpio_request_by_name(dev, "vcomctl-gpios", 0, &sy7673a_priv->vcom_ctrl_gpio, GPIOD_IS_OUT);
    if (ret) {
        printf("davidpeng,%s,Line:%d,Can not get vcomctl-gpios: %d\n", __func__, __LINE__, ret);
        dm_gpio_free(dev, &sy7673a_priv->chip_en_gpio);
        return ret;
    }
    ret = gpio_request_by_name(dev, "error-gpios", 0, &sy7673a_priv->power_good_gpio, GPIOD_IS_IN);
    if (ret) {
        printf("davidpeng,%s,Line:%d,Can not get error-gpios: %d\n", __func__, __LINE__, ret);
        dm_gpio_free(dev, &sy7673a_priv->chip_en_gpio);
        dm_gpio_free(dev, &sy7673a_priv->vcom_ctrl_gpio);
        return ret;
    }
    ret = gpio_request_by_name(dev, "vdd-gpios", 0, &sy7673a_priv->p_vdd_en_gpio, GPIOD_IS_OUT);
    if (ret) {
        printf("davidpeng,%s,Line:%d,Can not get vdd-gpios: %d\n", __func__, __LINE__, ret);
        dm_gpio_free(dev, &sy7673a_priv->chip_en_gpio);
        dm_gpio_free(dev, &sy7673a_priv->vcom_ctrl_gpio);
        dm_gpio_free(dev, &sy7673a_priv->power_good_gpio);
        return ret;
    }
    ret = gpio_request_by_name(dev, "pmicon-gpios", 0, &sy7673a_priv->pmicon_gpio, GPIOD_IS_OUT);
    if (ret) {
        printf("davidpeng,%s,Line:%d,Can not get pmicon-gpios: %d\n", __func__, __LINE__, ret);
    }
    
    ret = sy7673a_hw_init(dev);
    if (ret) {
        printf("Cannot init hardware for sy7673a: %d--free all gpios!\n", ret);
        
        // 20220605: 我们需要在uboot里面同时支持两个不同的 PMIC,所以一个初始化失败的时候，需要释放
        // 对应的 GPIO，否则会影响另外一个的初始化。比如我们把 JYT的固件烧到 BOE的板子上面。
        dm_gpio_free(dev, &sy7673a_priv->chip_en_gpio);
        dm_gpio_free(dev, &sy7673a_priv->vcom_ctrl_gpio);
        dm_gpio_free(dev, &sy7673a_priv->power_good_gpio);
        dm_gpio_free(dev, &sy7673a_priv->p_vdd_en_gpio);
        dm_gpio_free(dev, &sy7673a_priv->pmicon_gpio);
        return ret;
    }

    return 0;
}

static const struct rk_ebc_pwr_ops sy7673a_funcs = {
    .power_on = sy7673a_power_on,
    .power_down = sy7673a_power_down,
    .temp_get = sy7673a_temp_get,
    .vcom_set = sy7673a_set_vcom_value,
};

static const struct udevice_id sy7673a_power_of_match[] = {
    { .compatible = "sy7636a" },
    {}
};

U_BOOT_DRIVER(sy7673a_ebc_pwr) = {
    .name = "sy7673a_ebc_pwr",
    .id = UCLASS_I2C_GENERIC,
    .of_match = sy7673a_power_of_match,
    .probe = sy7673a_probe,
    .ops = &sy7673a_funcs,
    .bind = dm_scan_fdt_dev,
    .priv_auto_alloc_size = sizeof(struct sy7673a_priv_data),
};

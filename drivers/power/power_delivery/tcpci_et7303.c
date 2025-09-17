// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2022 Rockchip Co.,Ltd.
 * Author: Wang Jie <dave.wang@rock-chips.com>
 *
 * Richtek ET7303 Type-C Chip Driver
 */

#include <dm.h>
#include <i2c.h>
#include <asm/gpio.h>
#include <power/power_delivery/tcpm.h>
#include <power/power_delivery/power_delivery.h>
#include "tcpci.h"

#define ET7303_VID         0x6DCF
#define ET7303_PID         0x1711

#define RT1711H_VID        0x29CF
#define RT1711H_PID        0x1711

#define ET7303_RTCTRL8     0x9B

/* Autoidle timeout = (tout * 2 + 1) * 6.4ms */
#define ET7303_RTCTRL8_SET(ck300, ship_off, auto_idle, tout) (((ck300) << 7) | ((ship_off) << 5) | ((auto_idle) << 3) | ((tout) & 0x07))

#define ET7303_RTCTRL11    0x9E

/* I2C timeout = (tout + 1) * 12.5ms */
#define ET7303_RTCTRL11_SET(en, tout) (((en) << 7) | ((tout) & 0x0F))

#define ET7303_RTCTRL13    0xA0
#define ET7303_RTCTRL14    0xA1
#define ET7303_RTCTRL15    0xA2
#define ET7303_RTCTRL16    0xA3
#define ET7303_I2C_RETRY_MAX_CNT 3

struct et7303_chip {
    struct udevice *udev;
    struct tcpci_data data;
    struct tcpci *tcpci;
};

static int et7303_read16(struct et7303_chip *chip, unsigned int reg)
{
    int ret = 0;
    u8 buffer[2];

    ret = dm_i2c_read(chip->udev, reg, buffer, 2);
    if (ret < 0) {
        printf("%s: cannot read %02x, ret=%d\n", __func__, reg, ret);
        return ret;
    }
    ret = ((buffer[1] << 8) & 0xFF00) + (buffer[0] & 0xFF);

    return ret;
}

static int et7303_write8(struct et7303_chip *chip, unsigned int reg, u8 val)
{
    int ret = 0;
    int i;

    for (i = 0; i < ET7303_I2C_RETRY_MAX_CNT; i++) {
        ret = dm_i2c_write(chip->udev, reg, &val, 1);
        if (!ret) {
            break;
        } else {
            udelay(200);
        }
    }

    if (ret) {
        printf("%s: cannot write 0x%02x to 0x%02x, ret=%d\n", __func__, val, reg, ret);
    }

    return ret;
}

static int et7303_write16(struct et7303_chip *chip, unsigned int reg, u16 val)
{
    int ret = 0;
    u8 buffer[2];

    buffer[0] = val & 0xFF;
    buffer[1] = (val >> 8) & 0xFF;
    ret = dm_i2c_write(chip->udev, reg, buffer, 2);
    if (ret) {
        printf("%s: cannot write 0x%02x, len=%d, ret=%d\n", __func__, reg, 2, ret);
    }

    return ret;
}

static struct et7303_chip *tdata_to_et7303(struct tcpci_data *tdata)
{
    return container_of(tdata, struct et7303_chip, data);
}

static int et7303_sw_reset(struct et7303_chip *chip)
{
    return et7303_write8(chip, ET7303_RTCTRL13, 0x01);
    //usleep_range(1000, 2000);
}

static int et7303_init(struct tcpci *tcpci, struct tcpci_data *tdata)
{
    int ret;
    struct et7303_chip *chip = tdata_to_et7303(tdata);

    /* CK 300K from 320K, shipping off, auto_idle enable, tout = 32ms */
    ret = et7303_write8(chip, ET7303_RTCTRL8, ET7303_RTCTRL8_SET(0, 1, 1, 2));
    if (ret < 0) {
        return ret;
    }

    /* I2C reset : (val + 1) * 12.5ms */
    ret = et7303_write8(chip, ET7303_RTCTRL11, ET7303_RTCTRL11_SET(1, 0x0F));
    if (ret < 0) {
        return ret;
    }

    /* tTCPCfilter : (26.7 * val) us */
    ret = et7303_write8(chip, ET7303_RTCTRL14, 0x0F);
    if (ret < 0) {
        return ret;
    }

    /*  tDRP : (51.2 + 6.4 * val) ms */
    ret = et7303_write8(chip, ET7303_RTCTRL15, 0x04);
    if (ret < 0) {
        return ret;
    }

    /* dcSRC.DRP : 33% */
    return et7303_write16(chip, ET7303_RTCTRL16, 330);
}

static int et7303_check_revision(struct et7303_chip *chip)
{
    int ret;

    ret = et7303_read16(chip, TCPC_VENDOR_ID);
    if (ret < 0) {
        printf("%s: fail to read Vendor id(%d)\n", __func__, ret);
        return ret;
    }
    if (ret != ET7303_VID && ret != RT1711H_VID) {
        printf("%s: vid is not correct, 0x%04x\n", __func__, ret);
        return -ENODEV;
    }
    ret = et7303_read16(chip, TCPC_PRODUCT_ID);
    if (ret < 0) {
        printf("%s: fail to read Product id(%d)\n", __func__, ret);
        return ret;
    }
    if (ret != ET7303_PID && ret != RT1711H_PID) {
        printf("%s: pid is not correct, 0x%04x\n", __func__, ret);
        return -ENODEV;
    }
    return 0;
}

static int et7303_set_vconn(struct tcpci *tcpci, struct tcpci_data *tdata, bool enable)
{
    struct et7303_chip *chip = tdata_to_et7303(tdata);

    return et7303_write8(chip, ET7303_RTCTRL8,
    ET7303_RTCTRL8_SET(0, 1, !enable, 2));
}

static int et7303_start_drp_toggling(struct tcpci *tcpci, struct tcpci_data *tdata, enum typec_cc_status cc)
{
    struct et7303_chip *chip = tdata_to_et7303(tdata);
    int ret;
    unsigned int reg = 0;

    switch (cc) {
        default:
        case TYPEC_CC_RP_DEF:
            reg |= (TCPC_ROLE_CTRL_RP_VAL_DEF << TCPC_ROLE_CTRL_RP_VAL_SHIFT);
            break;
        case TYPEC_CC_RP_1_5:
            reg |= (TCPC_ROLE_CTRL_RP_VAL_1_5 << TCPC_ROLE_CTRL_RP_VAL_SHIFT);
            break;
        case TYPEC_CC_RP_3_0:
            reg |= (TCPC_ROLE_CTRL_RP_VAL_3_0 << TCPC_ROLE_CTRL_RP_VAL_SHIFT);
            break;
    }

    if (cc == TYPEC_CC_RD) {
        reg |= (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT) | (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT);
    } else {
        reg |= (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) | (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT);
    }

    ret = et7303_write8(chip, TCPC_ROLE_CTRL, reg);
    if (ret < 0) {
        return ret;
    }
    //usleep_range(500, 1000);
    udelay(800);

    return 0;
}

static int et7303_probe(struct udevice *dev)
{
    int ret;
    struct et7303_chip *chip = dev_get_priv(dev);

    chip->udev = dev;

    ret = et7303_check_revision(chip);
    if (ret < 0) {
        printf("%s: check vid/pid fail(%d)\n", __func__, ret);
        return ret;
    }

    printf("uboot et7303 cc probe ok.\n");
    ret = et7303_sw_reset(chip);
    if (ret) {
        printf("%s: fail to soft reset, ret = %d\n", __func__, ret);
        return ret;
    }
    udelay(1500);

    /* Disable chip interrupts before requesting irq */
    /*
    ret = et7303_write16(chip, TCPC_ALERT_MASK, 0);
    if (ret < 0)
        return ret;
    */

    chip->data.init = et7303_init;
    chip->data.set_vconn = et7303_set_vconn;
    chip->data.start_drp_toggling = et7303_start_drp_toggling;
    chip->tcpci = tcpci_register_port(chip->udev, &chip->data);
    if (IS_ERR(chip->tcpci)) {
        return PTR_ERR(chip->tcpci);
    }

    return 0;
}

static int et7303_remove(struct udevice *dev)
{
    struct et7303_chip *chip = dev_get_priv(dev);
    int ret = 0;

    printf("PD chip et7303 remove\n");
    /* Disable chip interrupts before unregistering port */
    ret = et7303_write16(chip, TCPC_ALERT_MASK, 0);
    if (ret < 0) {
        return ret;
    }

    tcpci_unregister_port(chip->tcpci);
    return 0;
}

static int et7303_get_voltage(struct udevice *dev)
{
    struct et7303_chip *chip = dev_get_priv(dev);

    return tcpci_get_voltage_fun(chip->tcpci);
}

static int et7303_get_current(struct udevice *dev)
{
    struct et7303_chip *chip = dev_get_priv(dev);

    return tcpci_get_current_fun(chip->tcpci);
}

static int et7303_get_online(struct udevice *dev)
{
    struct et7303_chip *chip = dev_get_priv(dev);

    return tcpci_get_online_fun(chip->tcpci);
}

static struct dm_power_delivery_ops et7303_ops = {
    .get_voltage = et7303_get_voltage,
    .get_current = et7303_get_current,
    .get_online = et7303_get_online,
};

static const struct udevice_id et7303_ids[] = {
    { .compatible = "etek,et7303" },
    {},
};

U_BOOT_DRIVER(et7303) = {
    .name = "et7303",
    .id = UCLASS_PD,
    .of_match = et7303_ids,
    .ops = &et7303_ops,
    .probe = et7303_probe,
    .remove = et7303_remove,
    .priv_auto_alloc_size = sizeof(struct et7303_chip),
};

MODULE_AUTHOR("Wang Jie <dave.wang@rock-chips.com>");
MODULE_DESCRIPTION("ET7303 USB Type-C Port Controller Interface Driver");
MODULE_LICENSE("GPL");

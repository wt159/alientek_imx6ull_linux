#include "ap3216c_reg.h"
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/types.h>

#define AP3216C_NAME "ap3216c"

struct ap3216c_dev {
    struct miscdevice miscdev;
    void *private_data;
    unsigned short ir, als, ps;
};
static struct ap3216c_dev ap3216cdev;

void ap3216c_readdata(struct ap3216c_dev *dev)
{
    unsigned char buf[6];

    struct i2c_client *client = (struct i2c_client *)dev->private_data;
    i2c_smbus_read_i2c_block_data(client, AP3216C_IRDATALOW, 6, buf);

    if (buf[0] & 0X80) /* IR_OF位为1,则数据无效 */
        dev->ir = 0;
    else /* 读取IR传感器的数据   		*/
        dev->ir = ((unsigned short)buf[1] << 2) | (buf[0] & 0X03);

    dev->als = ((unsigned short)buf[3] << 8) | buf[2]; /* 读取ALS传感器的数据 			 */

    if (buf[4] & 0x40) /* IR_OF位为1,则数据无效 			*/
        dev->ps = 0;
    else /* 读取PS传感器的数据    */
        dev->ps = ((unsigned short)(buf[5] & 0X3F) << 4) | (buf[4] & 0X0F);
}

static int ap3216c_open(struct inode *inode, struct file *filp)
{
    struct i2c_client *client = NULL;
    filp->private_data = &ap3216cdev;
    client = (struct i2c_client *)ap3216cdev.private_data;
    /* 初始化AP3216C */
    i2c_smbus_write_byte_data(client, AP3216C_SYSTEMCONG, 0x04); /* 复位AP3216C */
    mdelay(50); /* AP3216C复位最少10ms */
    i2c_smbus_write_byte_data(client, AP3216C_SYSTEMCONG, 0X03); /* 开启ALS、PS+IR */
    return 0;
}

static ssize_t ap3216c_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
    short data[3];
    long err = 0;

    struct ap3216c_dev *dev = (struct ap3216c_dev *)filp->private_data;

    ap3216c_readdata(dev);

    data[0] = dev->ir;
    data[1] = dev->als;
    data[2] = dev->ps;
    err = copy_to_user(buf, data, sizeof(data));
    return 0;
}

static int ap3216c_release(struct inode *inode, struct file *filp)
{
    return 0;
}

/* AP3216C操作函数 */
static const struct file_operations ap3216c_ops = {
    .owner = THIS_MODULE,
    .open = ap3216c_open,
    .read = ap3216c_read,
    .release = ap3216c_release,
};

static int ap3216c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    printk("%s(): probe enter\n", __func__);
    ap3216cdev.miscdev.minor = MISC_DYNAMIC_MINOR;
    ap3216cdev.miscdev.name = AP3216C_NAME;
    ap3216cdev.miscdev.fops = &ap3216c_ops;
    ret = misc_register(&ap3216cdev.miscdev);
    ap3216cdev.private_data = client;
    i2c_set_clientdata(client, &ap3216cdev);
    printk("%s(): probe exit\n", __func__);
    return ret;
}

static int ap3216c_remove(struct i2c_client *client)
{
    struct ap3216c_dev *dev = i2c_get_clientdata(client);
    misc_deregister(&dev->miscdev);
    return 0;
}

// clang-format off
static const struct i2c_device_id ap3216c_id[] = {
    { "ap3216c", 0 },
    { /* Sentinel */ } 
};

static const struct of_device_id ap3216c_of_match[] = {
    { .compatible = "alientek,ap3216c" },
    { /* Sentinel */ }
};

static struct i2c_driver ap3216c_driver = {
    .probe = ap3216c_probe,
    .remove = ap3216c_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = AP3216C_NAME,
        .of_match_table = ap3216c_of_match, 
    },
    .id_table = ap3216c_id,
};
// clang-format on

static int __init ap3216c_init(void)
{
    int ret = 0;
    ret = i2c_add_driver(&ap3216c_driver);
    return ret;
}

static void __exit ap3216c_exit(void)
{
    i2c_del_driver(&ap3216c_driver);
}

module_init(ap3216c_init);
module_exit(ap3216c_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("wtp");
/**
 * @author A Wildan M
 * Direct GPIO kernel module,
 * it used to read 5 toggle
 *
 * @date March, 2023
 *
 * For more references:
 * https://datasheets.raspberrypi.com/bcm2711/bcm2711-peripherals.pdf
 *
 *
 */
#include "linux/module.h"
#include "linux/kernel.h"
#include "linux/init.h"

#include <linux/proc_fs.h>
#include <linux/slab.h>

#include <asm/io.h>

/* GPIO addr, use "dmesg | grep gpio" to find GPIO base address*/
#define GPIO_BASE 0xfe200000
#define GPIO_SET0 0x1c
#define GPIO_CLR0 0x28
#define GPIO_LEV0 0x34

#define PROCFS_FILENAME "gpio_5toggle"

/* procfs communication */
static struct proc_dir_entry *driver_proc = NULL;
static char data_buffer[8] = {0};

/* GPIO register */
static unsigned int *gpio_base_reg = NULL;

/* From user */
static unsigned char toggle_pin[5] = {0};

/**
 * Set gpio pin mode,
 * 1 for output mode,
 * 0 for input mode
 *
 */
void set_gpio_pinMode(char pin, char mode)
{
    if (mode != 0 && mode != 1)
    {
        printk("Invalid pin mode!\n");
        return;
    }
    unsigned int fsel_index = pin / 10;
    unsigned int fsel_bitpos = pin % 10;
    unsigned int *gpio_fsel_reg = gpio_base_reg + fsel_index;

    *gpio_fsel_reg &= ~(7 << (fsel_bitpos * 3));   // Reset
    *gpio_fsel_reg |= (mode << (fsel_bitpos * 3)); // Set

    return;
}

void set_gpio_pinHigh(char pin)
{
    unsigned int *gpio_set_reg = (unsigned int *)((char *)gpio_base_reg + GPIO_SET0);
    *gpio_set_reg |= (1 << pin);
    return;
}

void set_gpio_pinLow(char pin)
{
    unsigned int *gpio_clr_reg = (unsigned int *)((char *)gpio_base_reg + GPIO_CLR0);
    *gpio_clr_reg |= (1 << pin);
    return;
}

int get_gpio_pinLevel(char pin)
{
    if (pin == 0)
        return 0;
    unsigned int *gpio_lev_reg = (unsigned int *)((char *)gpio_base_reg + GPIO_LEV0);
    return ((*gpio_lev_reg & (1 << pin)) >> pin);
}

ssize_t
driver_read(struct file *file, char __user *user, size_t size, loff_t *off)
{
    unsigned int data_pin[5];

    data_pin[0] = get_gpio_pinLevel(toggle_pin[0]);
    data_pin[1] = get_gpio_pinLevel(toggle_pin[1]);
    data_pin[2] = get_gpio_pinLevel(toggle_pin[2]);
    data_pin[3] = get_gpio_pinLevel(toggle_pin[3]);
    data_pin[4] = get_gpio_pinLevel(toggle_pin[4]);

    // printk("data_pin: %d %d %d %d %d\n", data_pin[0], data_pin[1], data_pin[2], data_pin[3], data_pin[4]);

    copy_to_user(user, data_pin, 5);
    *off += 5;
    return (*off > 5 ? 0 : 5);
}
ssize_t driver_write(struct file *file, const char __user *user, size_t size, loff_t *off)
{
    memset(data_buffer, 0, sizeof(data_buffer));

    copy_from_user(data_buffer, user, size);

    unsigned char op_code;

    memcpy(&op_code, data_buffer, 1);

    /* If op_code is 0x01, it allow user to set input toggle pins */
    if (op_code == 0x01)
    {
        memcpy(toggle_pin, data_buffer + 1, 5);
        set_gpio_pinMode(toggle_pin[0], 0);
        set_gpio_pinMode(toggle_pin[1], 0);
        set_gpio_pinMode(toggle_pin[2], 0);
        set_gpio_pinMode(toggle_pin[3], 0);
        set_gpio_pinMode(toggle_pin[4], 0);
    }

    // printk("input pins: %d %d %d %d %d\n", toggle_pin[0], toggle_pin[1], toggle_pin[2], toggle_pin[3], toggle_pin[4]);

    return size;
}

static const struct proc_ops driver_proc_fops =
    {
        .proc_read = driver_read,
        .proc_write = driver_write,
};

static int __init driver_init(void)
{
    memset(toggle_pin, 0, sizeof(toggle_pin));

    printk("Mapping gpio to driver...\n");
    gpio_base_reg = (int *)ioremap(GPIO_BASE, PAGE_SIZE);
    if (gpio_base_reg == NULL)
    {
        printk("Failed map GPIO to driver\n");
        return -1;
    }

    printk("Success mapping gpio to driver at 0x%x\n", gpio_base_reg);

    printk("Creating procfs...\n");
    driver_proc = proc_create(PROCFS_FILENAME, 0666, NULL, &driver_proc_fops);
    if (driver_proc == NULL)
    {
        printk("Error while creating procfs\n");
        return -1;
    }
    printk("Success creating procfs\n");

    return 0;
}

static void __exit driver_exit(void)
{
    iounmap(gpio_base_reg);
    proc_remove(driver_proc);
    printk("Exit driver\n");
    return;
}

module_init(driver_init);
module_exit(driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Azzam Wildan Maulana");
MODULE_DESCRIPTION("GPIO read-write");
MODULE_VERSION("1.0");
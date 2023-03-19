/**
 * @author Azzam Wildan Maulana
 * @brief Just a simple linux kernel module
 *
 * @date March, 2023
 */
#include "linux/init.h"
#include "linux/module.h"
#include "linux/kernel.h"

static int __init km_init(void)
{
    printk("Welcome simple linux kernel module\n");
    return 0;
}

static void __exit km_exit(void)
{
    printk("Exiting simple linux kernel module\n");
    return;
}

module_init(km_init);
module_exit(km_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Azzam Wildan Maulana");
MODULE_DESCRIPTION("Simple Linux Kernel Module");
MODULE_VERSION("1.0");

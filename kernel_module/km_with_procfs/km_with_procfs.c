/**
 * @author Azzam Wildan Maulana
 * Using procfs to communicate with user-space
 *
 * @date March, 2023
 */
#include "linux/module.h"
#include "linux/kernel.h"
#include "linux/init.h"

#include <linux/proc_fs.h>
#include <linux/slab.h>

static struct proc_dir_entry *driver_proc = NULL;
static char data_buffer[128] = {0};

ssize_t
driver_read(struct file *file, char __user *user, size_t size, loff_t *off)
{
    copy_to_user(user, "Hello!\n", 7);
    *off += 7;
    return (*off > 7 ? 0 : 7);
}
ssize_t driver_write(struct file *file, const char __user *user, size_t size, loff_t *off)
{
    memset(data_buffer, 0, sizeof(data_buffer));

    copy_from_user(data_buffer, user, size);

    printk("from user: %s\n", data_buffer);

    return size;
}

static const struct proc_ops driver_proc_fops =
    {
        .proc_read = driver_read,
        .proc_write = driver_write,
};

static int __init driver_init(void)
{
    printk("Hello world\n");

    printk("Creating procfs\n");
    driver_proc = proc_create("my_driver", 0666, NULL, &driver_proc_fops);
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
    printk("Exit driver\n");
    proc_remove(driver_proc);
    return;
}

module_init(driver_init);
module_exit(driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Azzam Wildan Maulana");
MODULE_DESCRIPTION("Simple kernel module using proc fs to communicate with user-space");
MODULE_VERSION("1.0");
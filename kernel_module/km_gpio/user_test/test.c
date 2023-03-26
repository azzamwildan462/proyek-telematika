#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

int main()
{
    int f = open("/proc/gpio_5toggle", O_RDWR);

    unsigned char op_code = 0x01;
    unsigned char pin_[5] = {17, 27, 22, 23, 24};
    char send_buff[6];
    memcpy(send_buff, &op_code, 1);
    memcpy(send_buff + 1, pin_, 5);
    write(f, send_buff, 6);

    while (1)
    {
        usleep(500000);
        unsigned char pins[5];
        char read_buff[5];
        read(f, read_buff, 5);
        memcpy(pins, read_buff, 5);
        printf("val: %d %d %d %d %d\n", pins[0], pins[1], pins[2], pins[3], pins[4]);
    }

    return 0;
}
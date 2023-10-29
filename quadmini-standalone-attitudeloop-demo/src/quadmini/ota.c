#include <sleep.h>
#include <printf.h>
#include <sysctl.h>

#include "data_exchange.h"
#include "data_exchange_printf.h"
#include "w25qxx.h"
#include "ota.h"

void ota_task(void)
{
    if (ota_file.file_whole == 1 && ota_file.file_flashed == 0)
    {
        msleep(2);
        printk("Recieve OTA data\n\r");
        printk("Write to flash\n\r");
        w25qxx_init(3, 0, 60000000);
        printw("OTA write %.1f kb to flash\n", ota_file.total_len/1024.0);
        w25qxx_write_data(0x0, ota_file.data, ota_file.total_len);
        ota_file.file_flashed = 1;
        printk("OTA complete, Spend %u.%us\n\r"
              , (unsigned int)(sysctl_get_time_us() - ota_file.start_time) / 1000000
              , (unsigned int)((sysctl_get_time_us() - ota_file.start_time) / 1000) % 1000);
        printk("Restart SOC!\n\r");
        msleep(2);
        sysctl_reset(SYSCTL_RESET_SOC);
    }
}

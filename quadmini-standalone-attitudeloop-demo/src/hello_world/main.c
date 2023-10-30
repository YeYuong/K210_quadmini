/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <bsp.h>
#include <sysctl.h>
#include <fpioa.h>
#include <gpiohs.h>

int core1_function(void *ctx)
{
    uint64_t core = current_coreid();
    printf("Core %ld Hello world\n", core);
    while(1);
}

int main(void)
{
    sysctl_pll_set_freq(SYSCTL_PLL0, 800000000);
    sysctl_pll_set_freq(SYSCTL_PLL2, 45158400UL);
    uint64_t core = current_coreid();
    int data;
    printf("Core %ld Hello world\n", core);
    // register_core1(core1_function, NULL);
    plic_init();

    fpioa_set_function(29, FUNC_GPIOHS0 + 0);
    fpioa_set_function(24, FUNC_GPIOHS0 + 1);
    gpiohs_set_drive_mode(1, GPIO_DM_OUTPUT);
    gpiohs_set_drive_mode(0, GPIO_DM_OUTPUT);
    gpiohs_set_pin(0, GPIO_PV_HIGH);
    gpiohs_set_pin(1, GPIO_PV_HIGH);

    // // 将串口设置为GPIO转发
    // fpioa_set_function(4, FUNC_GPIOHS0 + 0); // K210 RX
    // fpioa_set_function(5, FUNC_GPIOHS0 + 1); // K210 TX
    // fpioa_set_function(6, FUNC_GPIOHS0 + 2); // ESP  TX
    // fpioa_set_function(7, FUNC_GPIOHS0 + 3); // ESP  RX

    // gpiohs_set_drive_mode(0, GPIO_DM_INPUT);
    // gpiohs_set_drive_mode(3, GPIO_DM_OUTPUT);

    // gpiohs_set_drive_mode(2, GPIO_DM_INPUT);
    // gpiohs_set_drive_mode(1, GPIO_DM_OUTPUT);

    // while(1)
    // {
    //     gpiohs_set_pin(3, gpiohs_get_pin(0));
    //     gpiohs_set_pin(1, gpiohs_get_pin(2));
    // }

    /* Clear stdin buffer before scanf */
    // sys_stdin_flush();

    // scanf("%d", &data);
    // printf("\nData is %d\n", data);
    while(1)
    {
        gpiohs_set_pin(0, GPIO_PV_HIGH);
        gpiohs_set_pin(1, GPIO_PV_HIGH);
        continue;
    }
    return 0;
}

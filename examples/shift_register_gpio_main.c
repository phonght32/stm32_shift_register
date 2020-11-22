// MIT License

// Copyright (c) 2020 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "shift_register.h"

#define TASK_SIZE   512
#define TASK_PRIOR  5

#define SPI_NUM         SPI_NUM_1
#define SPI_PINS_PACK   SPI_PINS_PACK_2
#define CAP_EDGE        SPI_CAP_FALLING_EDGE
#define FIRSTBIT        SPI_TRANS_FIRSTBIT_MSB

static const char *TAG = "APP_MAIN";
shift_register_handle_t shift_register_handle;

static void example_task(void* arg)
{
    shift_register_cfg_t shift_register_cfg = {
        .spi_num = SPI_NUM,
        .spi_pins_pack = SPI_PINS_PACK,
        .cap_edge = CAP_EDGE,
        .firstbit = FIRSTBIT
    };
    shift_register_handle = shift_register_init(&shift_register_cfg);

    while (1)
    {
        uint8_t data_send = 0x01;
        for (uint8_t i = 0; i < 8; i++) {
            shift_register_write_bytes(shift_register_handle, &data_send, 1);
            data_send = data_send << 1;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

int main(void)
{
    /* Set application log output level */
    stm_log_level_set("*", STM_LOG_NONE);
    stm_log_level_set("APP_MAIN", STM_LOG_INFO);

    /* Create task */
    xTaskCreate(example_task, "example_task", TASK_SIZE, NULL, TASK_PRIOR, NULL);

    /* Start RTOS scheduler */
    vTaskStartScheduler();
}
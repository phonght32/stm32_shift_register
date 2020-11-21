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

#ifndef _SHIFT_REGISTER_H_
#define _SHIFT_REGISTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm_err.h"
#include "driver/gpio.h"
#include "driver/spi.h"

typedef struct shift_register *shift_register_handle_t;

typedef enum {
	SHIFT_REGISTER_COMM_MODE_GPIO = 0,
	SHIFT_REGISTER_COMM_MODE_SPI,
	SHIFT_REGISTER_COMM_MODE_MAX,
} shift_register_comm_mode_t;

typedef struct {
	int 					gpio_port_data;
	int 					gpio_num_data;
	int 					gpio_port_clk;
	int 					gpio_num_clk;
	spi_num_t				spi_num;
	spi_pins_pack_t			spi_pins_pack;
} shift_register_hw_info_t;

typedef struct {
	shift_register_hw_info_t 	hw_info;
	shift_register_comm_mode_t 	comm_mode;
} shift_register_cfg_t;

shift_register_handle_t shift_register_init(shift_register_cfg_t *config);
stm_err_t shift_register_write_bytes(shift_register_handle_t handle, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif  /* _SHIFT_REGISTER_H_ */
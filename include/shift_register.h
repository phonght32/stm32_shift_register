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
#include "driver/spi.h"

typedef struct shift_register *shift_register_handle_t;

typedef struct {
	spi_num_t				spi_num;			/*!< SPI num */
	spi_pins_pack_t			spi_pins_pack;		/*!< SPI pins pack */
	spi_cap_edge_t			cap_edge;			/*!< Edge type to capture data */
	spi_trans_firstbit_t	firstbit;			/*!< MSB/LSB firstbit */
} shift_register_cfg_t;

/* 
 * @brief	Initialize shift register driver.
 * @param 	config Pointer to config structure.
 * @return
 *      - Shift register handle structure: Success.
 *      - 0: Fail.
 */
shift_register_handle_t shift_register_init(shift_register_cfg_t *config);

/* 
 * @brief	Send data.
 * @param 	handle Handle structure.
 * @param 	data Pointer to data send.
 * @param 	length Data length.
 * @return
 *      - STM_OK:   Success.
 *      - Others: 	Fail.
 */
stm_err_t shift_register_write_bytes(shift_register_handle_t handle, uint8_t *data, uint16_t length);

/* 
 * @brief	Destroy shift register instance.
 * @param 	handle Handle structure.
 * @return	None.
 */
void shift_register_destroy(shift_register_handle_t handle);


#ifdef __cplusplus
}
#endif

#endif  /* _SHIFT_REGISTER_H_ */
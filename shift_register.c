#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "stm_log.h"
#include "include/shift_register.h"

#define SHIFT_REGISTER_INIT_ERR_STR				"shift register init error"
#define SHIFT_REGISTER_WRITE_ERR_STR			"shift register write error"

#define mutex_lock(x)               while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x)             xSemaphoreGive(x)
#define mutex_create()              xSemaphoreCreateMutex()
#define mutex_destroy(x)            vQueueDelete(x)

static const char* TAG = "SHIFT_REGISTER";
#define SHIFT_REGISTER_CHECK(a, str, action)  if(!(a)) {                       		\
        STM_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);       \
        action;                                                                   	\
        }

typedef stm_err_t (*init_func)(shift_register_hw_info_t hw_info);
typedef stm_err_t (*write_func)(shift_register_hw_info_t hw_info, uint8_t data);

typedef struct shift_register {
	shift_register_hw_info_t 	hw_info;
	shift_register_comm_mode_t 	comm_mode;
	write_func 					_write;
	SemaphoreHandle_t 			lock;
} shift_register_t;

stm_err_t _init_gpio(shift_register_hw_info_t hw_info)
{
	gpio_cfg_t gpio_cfg;

	gpio_cfg.gpio_port = hw_info.gpio_port_data;
	gpio_cfg.gpio_num = hw_info.gpio_num_data;
	gpio_cfg.mode = GPIO_OUTPUT_PP;
	gpio_cfg.reg_pull_mode = GPIO_REG_PULL_NONE;
	SHIFT_REGISTER_CHECK(!gpio_config(&gpio_cfg), SHIFT_REGISTER_INIT_ERR_STR, return STM_FAIL);

	gpio_cfg.gpio_port = hw_info.gpio_port_clk;
	gpio_cfg.gpio_num = hw_info.gpio_num_clk;
	gpio_cfg.mode = GPIO_OUTPUT_PP;
	gpio_cfg.reg_pull_mode = GPIO_REG_PULL_NONE;
	SHIFT_REGISTER_CHECK(!gpio_config(&gpio_cfg), SHIFT_REGISTER_INIT_ERR_STR, return STM_FAIL);

	return STM_OK;
}

stm_err_t _init_spi(shift_register_hw_info_t hw_info)
{
	return STM_OK;
}

stm_err_t _write_gpio(shift_register_hw_info_t hw_info, uint8_t data)
{
	STM_LOGD(TAG, "data: %x", data);
	SHIFT_REGISTER_CHECK(!gpio_set_level(hw_info.gpio_port_clk, hw_info.gpio_num_clk, true), SHIFT_REGISTER_WRITE_ERR_STR, return STM_FAIL);

	for (uint8_t i = 0; i < 8; i++) {
		SHIFT_REGISTER_CHECK(!gpio_set_level(hw_info.gpio_port_data, hw_info.gpio_num_data, (data >> i) & 0x01), SHIFT_REGISTER_WRITE_ERR_STR, return STM_FAIL);
		SHIFT_REGISTER_CHECK(!gpio_set_level(hw_info.gpio_port_clk, hw_info.gpio_num_clk, false), SHIFT_REGISTER_WRITE_ERR_STR, return STM_FAIL);
		SHIFT_REGISTER_CHECK(!gpio_set_level(hw_info.gpio_port_clk, hw_info.gpio_num_clk, true), SHIFT_REGISTER_WRITE_ERR_STR, return STM_FAIL);
	}

	return STM_OK;
}

stm_err_t _write_spi(shift_register_hw_info_t hw_info, uint8_t data)
{
	return STM_OK;
}

init_func _get_init_func(shift_register_comm_mode_t comm_mode)
{
	if (comm_mode == SHIFT_REGISTER_COMM_MODE_GPIO) {
		return _init_gpio;
	} else {
		return _init_spi;
	}
}

write_func _get_write_func(shift_register_comm_mode_t comm_mode)
{
	if (comm_mode == SHIFT_REGISTER_COMM_MODE_GPIO) {
		return _write_gpio;
	} else {
		return _write_spi;
	}
}

void _shift_register_cleanup(shift_register_handle_t handle)
{
	free(handle);
}

shift_register_handle_t shift_register_init(shift_register_cfg_t *config)
{
	SHIFT_REGISTER_CHECK(config, SHIFT_REGISTER_INIT_ERR_STR, return NULL);
	SHIFT_REGISTER_CHECK(config->comm_mode < SHIFT_REGISTER_COMM_MODE_MAX, SHIFT_REGISTER_INIT_ERR_STR, return NULL);
	SHIFT_REGISTER_CHECK(config->hw_info.gpio_port_data < GPIO_PORT_MAX, SHIFT_REGISTER_INIT_ERR_STR, return NULL);
	SHIFT_REGISTER_CHECK(config->hw_info.gpio_num_data < GPIO_NUM_MAX, SHIFT_REGISTER_INIT_ERR_STR, return NULL);
	SHIFT_REGISTER_CHECK(config->hw_info.gpio_port_clk < GPIO_PORT_MAX, SHIFT_REGISTER_INIT_ERR_STR, return NULL);
	SHIFT_REGISTER_CHECK(config->hw_info.gpio_num_clk < GPIO_NUM_MAX, SHIFT_REGISTER_INIT_ERR_STR, return NULL);
	SHIFT_REGISTER_CHECK(config->hw_info.spi_num < SPI_NUM_MAX, SHIFT_REGISTER_INIT_ERR_STR, return NULL);
	SHIFT_REGISTER_CHECK(config->hw_info.spi_pins_pack < SPI_PINS_PACK_MAX, SHIFT_REGISTER_INIT_ERR_STR, return NULL);

	shift_register_handle_t handle = calloc(1, sizeof(shift_register_t));
	SHIFT_REGISTER_CHECK(handle, SHIFT_REGISTER_INIT_ERR_STR, return NULL);

	init_func _init_func = _get_init_func(config->comm_mode);
	SHIFT_REGISTER_CHECK(!_init_func(config->hw_info), SHIFT_REGISTER_INIT_ERR_STR, {_shift_register_cleanup(handle); return NULL;});

	handle->hw_info = config->hw_info;
	handle->comm_mode = config->comm_mode;
	handle->_write = _get_write_func(config->comm_mode);
	handle->lock = mutex_create();

	return handle;
}

stm_err_t shift_register_write_bytes(shift_register_handle_t handle, uint8_t *data, uint16_t length)
{
	SHIFT_REGISTER_CHECK(handle, SHIFT_REGISTER_WRITE_ERR_STR, return STM_ERR_INVALID_ARG);
	SHIFT_REGISTER_CHECK(data, SHIFT_REGISTER_WRITE_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);

	int ret;
	for (uint16_t i = 0; i < length; i++) {
		ret = handle->_write(handle->hw_info, *data);
		if (ret) {
			STM_LOGE(TAG, SHIFT_REGISTER_WRITE_ERR_STR);
			mutex_unlock(handle->lock);
			return STM_FAIL;
		}
	}

	mutex_unlock(handle->lock);

	return STM_OK;
}
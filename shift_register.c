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

typedef struct shift_register {
	spi_num_t				spi_num;			/*!< SPI num */
	spi_pins_pack_t			spi_pins_pack;		/*!< SPI pins pack */
	spi_cap_edge_t			cap_edge;			/*!< Edge type to capture data */
	spi_trans_firstbit_t	firstbit;			/*!< MSB/LSB firstbit */
	SemaphoreHandle_t 		lock;				/*!< Semaphore */
} shift_register_t;

void _shift_register_cleanup(shift_register_handle_t handle)
{
	free(handle);
}

shift_register_handle_t shift_register_init(shift_register_cfg_t *config)
{
	SHIFT_REGISTER_CHECK(config, SHIFT_REGISTER_INIT_ERR_STR, return NULL);

	shift_register_handle_t handle = calloc(1, sizeof(shift_register_t));
	SHIFT_REGISTER_CHECK(handle, SHIFT_REGISTER_INIT_ERR_STR, return NULL);

	spi_cfg_t spi_cfg = {
		.spi_num = config->spi_num,
		.spi_pins_pack = config->spi_pins_pack,
		.mode = SPI_MODE_MASTER_HALF_DUPLEX,
		.cap_edge = config->cap_edge,
		.firstbit = config->firstbit
	};
	SHIFT_REGISTER_CHECK(!spi_config(&spi_cfg), SHIFT_REGISTER_INIT_ERR_STR, return NULL);

	handle->spi_num = config->spi_num;
	handle->spi_pins_pack = config->spi_pins_pack;
	handle->cap_edge = config->cap_edge;
	handle->firstbit = config->firstbit;
	handle->lock = mutex_create();

	return handle;
}

stm_err_t shift_register_write_bytes(shift_register_handle_t handle, uint8_t *data, uint16_t length)
{
	SHIFT_REGISTER_CHECK(handle, SHIFT_REGISTER_WRITE_ERR_STR, return STM_ERR_INVALID_ARG);
	SHIFT_REGISTER_CHECK(data, SHIFT_REGISTER_WRITE_ERR_STR, return STM_ERR_INVALID_ARG);

	mutex_lock(handle->lock);
	int ret = spi_write_bytes(handle->spi_num, data, length, 100);
	if (ret) {
		STM_LOGE(TAG, SHIFT_REGISTER_WRITE_ERR_STR);
		mutex_unlock(handle->lock);
		return STM_FAIL;
	}
	mutex_unlock(handle->lock);

	return STM_OK;
}
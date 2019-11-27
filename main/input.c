#include "input.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/uart.h"

static volatile bool input_task_is_running = false;
static volatile odroid_gamepad_state gamepad_state;
static odroid_gamepad_state previous_gamepad_state;
static uint8_t debounce[ODROID_INPUT_MAX];
static volatile bool input_gamepad_initialized = false;
static SemaphoreHandle_t xSemaphore;

// Setup UART buffered IO with event queue
const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;

const int uart_num = UART_NUM_0;
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
};

void input_read(odroid_gamepad_state* out_state)
{
    if (!input_gamepad_initialized) abort();

    xSemaphoreTake(xSemaphore, portMAX_DELAY);

    *out_state = gamepad_state;

    xSemaphoreGive(xSemaphore);
}

static void input_task(void *arg)
{
    input_task_is_running = true;

    int rx_state = 0;
    uint8_t st = 1;

    while(input_task_is_running)
    {

	// Read data from UART.
	uint8_t data[128];
	int length = 0;
	ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
	length = uart_read_bytes(uart_num, data, length, 100);

	for(int i = 0; i < length; i++)
	{
		if(data[i] == '+') 
		{
			rx_state = 0;
		}
		else
		{		
			switch(rx_state)
			{
				case 0:
					if(data[i] == '$') rx_state = 5;
				break;
				case 5:
					if(data[i] == 'D') rx_state = 10;
					else if(data[i] == 'U') rx_state = 11;
				break;
				case 10:
					st = 1;
					switch(data[i])
					{
						case '9': gamepad_state.values[ODROID_INPUT_UP] = st; break;
						case 'C': gamepad_state.values[ODROID_INPUT_LEFT] = st; break;
						case 'D': gamepad_state.values[ODROID_INPUT_DOWN] = st; break;
						case 'E': gamepad_state.values[ODROID_INPUT_RIGHT] = st; break;
						case '4': gamepad_state.values[ODROID_INPUT_MENU] = st; break;
						case '5': gamepad_state.values[ODROID_INPUT_VOLUME] = st; break;
						case '7': gamepad_state.values[ODROID_INPUT_SELECT] = st; break;
						case 'K': gamepad_state.values[ODROID_INPUT_START] = st; break;
						case 'B': gamepad_state.values[ODROID_INPUT_B] = st; break;
						case 'L': gamepad_state.values[ODROID_INPUT_A] = st; break;
					}
				break;
				case 11:
					st = 0;
					switch(data[i])
					{
						case '9': gamepad_state.values[ODROID_INPUT_UP] = st; break;
						case 'C': gamepad_state.values[ODROID_INPUT_LEFT] = st; break;
						case 'D': gamepad_state.values[ODROID_INPUT_DOWN] = st; break;
						case 'E': gamepad_state.values[ODROID_INPUT_RIGHT] = st; break;
						case '4': gamepad_state.values[ODROID_INPUT_MENU] = st; break;
						case '5': gamepad_state.values[ODROID_INPUT_VOLUME] = st; break;
						case '7': gamepad_state.values[ODROID_INPUT_SELECT] = st; break;
						case 'K': gamepad_state.values[ODROID_INPUT_START] = st; break;
						case 'B': gamepad_state.values[ODROID_INPUT_B] = st; break;
						case 'L': gamepad_state.values[ODROID_INPUT_A] = st; break;
					}
				break;
			}
		}
	}

        previous_gamepad_state = gamepad_state;

        xSemaphoreGive(xSemaphore);

        // delay
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    input_gamepad_initialized = false;

    vSemaphoreDelete(xSemaphore);

    // Remove the task from scheduler
    vTaskDelete(NULL);

    // Never return
    while (1) { vTaskDelay(1);}
}


void input_init()
{
    xSemaphore = xSemaphoreCreateMutex();

    if(xSemaphore == NULL)
    {
        printf("xSemaphoreCreateMutex failed.\n");
        abort();
    }

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 18, 19));
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, \
                                        uart_buffer_size, 10, &uart_queue, 0));

    input_gamepad_initialized = true;

    // Start background polling
    xTaskCreatePinnedToCore(&input_task, "input_task", 1024 * 2, NULL, 5, NULL, 1);

    printf("%s: done.\n", __func__);
}

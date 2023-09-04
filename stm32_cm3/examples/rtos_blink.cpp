#include <FreeRTOS.h>
#include <task.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

// LED pin definitions
#define LED_PORT GPIOC
#define LED_PIN GPIO13

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) 
{
    // Handle or log the stack overflow as needed (or leave it empty).
}


// FreeRTOS task function
extern "C" void ledBlinkTask(void *pvParameters) 
{
    (void)pvParameters;

    while (1) {
        // Toggle the LED
        gpio_toggle(LED_PORT, LED_PIN);

        // Delay for 500 ms (assuming a tick rate of 1 ms)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main() {
    // Initialize system clock and GPIO
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Adjust for your clock settings
    rcc_periph_clock_enable(RCC_GPIOC);

    // Configure the LED pin as an output
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);

    // Create the LED blink task
    xTaskCreate(ledBlinkTask, "LED_Blink", configMINIMAL_STACK_SIZE, nullptr, configMAX_PRIORITIES - 1, nullptr);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // The scheduler should never return, but in case it does, handle the error
    while (1);

    return 0;
}

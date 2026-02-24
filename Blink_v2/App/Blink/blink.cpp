#include "main.h"
#include "cmsis_os.h" // or "FreeRTOS.h" depending on your setup
#include <stdio.h>
#include <string.h>
#include "../Generic/my_print.h"

// Your C++ Class (Example)
class Blinker {
public:
    void run() {
        int i = 0;
        while (true) {
            HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);

            mprintf("Hello from STM32! Count: %d\r\n", i);

            i++;
            osDelay(1000);
        }
    }
};

Blinker myBlinker; // Global instance

// THE CRITICAL PART: Extern "C" wrapper for the task function
extern "C" {

    void StartBlink(void *argument) {
        // This function is called by FreeRTOS from main.c
        
        // You can use C++ objects here!
        myBlinker.run(); 
        
        // If run() returns, the task must delete itself
        vTaskDelete( NULL );
        while(1) { } 
    }

}

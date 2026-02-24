#include "main.h"
#include "cmsis_os.h" // or "FreeRTOS.h" depending on your setup
#include <stdio.h>
#include <string.h>
#include "my_print.h"

// Your C++ Class (Example)
class Blinker {
public:
    void run() {
        int i = 0;
        while (true) {
            HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);

            mprintf("Hello from STM32! Count: %d\r\n", i);

            i++;
            osDelay(100);
        }
    }
};

Blinker myBlinker; // Global instance

// THE CRITICAL PART: Extern "C" wrapper for the task function
extern "C" {

    void StartBlinkTask(void *argument) {
        // This function is called by FreeRTOS from main.c
        
        // You can use C++ objects here!
        myBlinker.run(); 
        
        // If run() returns, the task must delete itself
        osThreadTerminate(NULL);
    }

}

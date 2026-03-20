#include "cmsis_os.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "../Generic/my_print.h"
#include "DWM3000_setup.h"
#include "DWM3000_driver.h"
#include "messages.h"
#include "../UWB_app/uwb_network.h"
#include "uwb_exchange.h"

void position_calculate(uwb_etwr_result_t result)
{
    uint32_t t_start = osKernelGetTickCount();

    switch (result) {

        case UWB_TWR_RECEIVED:
        {
            // TODO: use real function
            network_t *net = network_get_network();
            for (int i = 0; i < net->count; i++)
                net->peers[i].uncertainty = (float)(rand() % 1000) / 1000.0f;
            break;
        }

        case UWB_TWR_RECEIVED_PASSIVE:
        case UWB_TWR_EXCHANGE_COMPLETE:
        case UWB_TWR_NOTHING:
        case UWB_TWR_NOT_ENOUGH_DEVICES:
        case UWB_TWR_TX_FAILED:
        case UWB_TWR_TIMEOUT:
        case UWB_TWR_UNEXPECTED_MASTER:
        default:
            break;
    }

    /* Pad to exactly 10ms from entry regardless of which path was taken */
    uint32_t elapsed = osKernelGetTickCount() - t_start;
    if (elapsed < 10U) osDelay(10U - elapsed);
}
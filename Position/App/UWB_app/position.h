#pragma once

#include "uwb_exchange.h"   /* uwb_etwr_result_t */

void position_calculate(uwb_etwr_result_t result);

uint64_t position_calibrate_timestamp(uint64_t orig_timestamp);
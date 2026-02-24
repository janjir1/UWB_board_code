#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

// Defined in lsm6dsv_self_test.c (or similar)
bool SelfTest(void);

// Defined in lsm6dsv_read_data_polling.c (or similar)
void lsm6dsv_read_data_polling(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_INTERFACE_H
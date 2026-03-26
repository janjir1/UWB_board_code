#ifndef LSM6DSV_SELF_TEST_H
#define LSM6DSV_SELF_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/* Function Prototypes */

// Initializes the sensor and starts the self-test
void lsm6dsv_self_test(void);

// Platform-specific functions (Remove 'static' to expose them if needed externally)
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void tx_com(uint8_t *tx_buffer, uint16_t len);
void platform_delay(uint32_t ms);
void platform_init(void);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSV_SELF_TEST_H */

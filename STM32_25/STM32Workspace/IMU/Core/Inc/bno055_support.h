/*
 * bno055_support.h
 *
 *  Created on: Jul 18, 2025
 *      Author: bafna
 */

#ifndef INC_BNO055_SUPPORT_H_
#define INC_BNO055_SUPPORT_H_

#include "bno055.h"

#ifdef __cplusplus
extern "C" {
#endif

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msec(u32 msec);
s8 I2C_routine(void);
void bno055_initialize_sensor(void);
void bno055_get_readings(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_BNO055_SUPPORT_H_ */

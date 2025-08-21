#include "bno055_support.h"
#include "bno055.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

struct bno055_t bno055;
struct bno055_euler_float_t euler_data;

#define BNO055_I2C_ADDRESS 0x28
#define BNO055_DELAY_MS(ms) HAL_Delay(ms)

s8 I2C_routine(void)
{
    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msec;
    bno055.dev_addr = BNO055_I2C_ADDRESS;

    return BNO055_INIT_VALUE;
}

void bno055_initialize_sensor(void)
{
    bno055_init(&bno055);
    bno055_set_power_mode(POWER_MODE_NORMAL);
    BNO055_DELAY_MS(10);

    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    BNO055_DELAY_MS(20);
}

void bno055_get_readings(void) {
    bno055_read_euler_hrp_float(&euler_data);
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "H:%.2f R:%.2f P:%.2f\r\n",
             euler_data.h, euler_data.r, euler_data.p);

    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    FIL file;
    UINT bw;
    FRESULT res;

    res = f_open(&file, "imu_log.txt", FA_OPEN_APPEND | FA_WRITE);
    if (res == FR_OK) {
        res = f_write(&file, buffer, strlen(buffer), &bw);

        if (res != FR_OK || bw != strlen(buffer)) {
            const char *msg = "ERROR: Failed to write data to SD card\r\n";
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        }

        f_close(&file);
    } else {
        const char *msg = "ERROR: Could not open imu_log.txt on SD card\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
}



s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    if (HAL_I2C_Mem_Write(&hi2c1, (dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, HAL_MAX_DELAY) == HAL_OK)
        return 0;
    else
        return -1;
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    if (HAL_I2C_Mem_Read(&hi2c1, (dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, HAL_MAX_DELAY) == HAL_OK)
        return 0;
    else
        return -1;
}

void BNO055_delay_msec(u32 msec)
{
    HAL_Delay(msec);
}

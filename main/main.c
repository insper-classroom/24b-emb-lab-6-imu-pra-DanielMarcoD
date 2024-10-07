#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <math.h>
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "mpu6050.h"
#include <Fusion.h>

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
#define UART_TX_PIN 0
#define UART_RX_PIN 1

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);

    *temp = buffer[0] << 8 | buffer[1];
}

void imu_uart_task(void *p) {
    // Inicialização do I2C, UART e do MPU6050
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    uart_init(uart0, 115200);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    const float SAMPLE_PERIOD = 0.01f;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = {
            gyro[0] / 131.0f,
            gyro[1] / 131.0f,
            gyro[2] / 131.0f
        };

        FusionVector accelerometer = {
            acceleration[0] / 16384.0f,
            acceleration[1] / 16384.0f,
            acceleration[2] / 16384.0f
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Detectar movimento rápido e para frente
        bool mouse_click_detected = (acceleration[1] > 20000);

        // Aplicar uma escala entre -255 e 255
        int16_t roll_scaled = (int16_t)((euler.angle.roll * 255.0f) / 180.0f);
        int16_t pitch_scaled = (int16_t)((euler.angle.pitch * 255.0f) / 180.0f);

        // Forçar valores a zero se um clique foi detectado
        if (mouse_click_detected) {
            roll_scaled = 0;
            pitch_scaled = 0;
        }

        // Limitar os valores para -255 e 255
        if (roll_scaled > 255) roll_scaled = 255;
        if (roll_scaled < -255) roll_scaled = -255;
        if (pitch_scaled > 255) pitch_scaled = 255;
        if (pitch_scaled < -255) pitch_scaled = -255;

        if (roll_scaled > -5 && roll_scaled < 5) {
            roll_scaled = 0;
        }
        if (pitch_scaled > -5 && pitch_scaled < 5) {
            pitch_scaled = 0;
        }

        uint8_t msb_roll = (pitch_scaled >> 8) & 0xFF;
        uint8_t lsb_roll = pitch_scaled & 0xFF;
        uint8_t msb_pitch = (roll_scaled >> 8) & 0xFF;
        uint8_t lsb_pitch = roll_scaled & 0xFF;
        uint8_t click_status = mouse_click_detected ? 0x01 : 0x00;
        uint8_t eop = 0xFF;

        uart_putc_raw(uart0, msb_roll);
        uart_putc_raw(uart0, lsb_roll);
        uart_putc_raw(uart0, msb_pitch);
        uart_putc_raw(uart0, lsb_pitch);
        uart_putc_raw(uart0, click_status); // Envia o status do clique
        uart_putc_raw(uart0, eop);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD * 1000));
    }
}

int main() {
    stdio_init_all();
    xTaskCreate(imu_uart_task, "imu_uart_task", 8192, NULL, 1, NULL);
    vTaskStartScheduler();
    while (true);
}

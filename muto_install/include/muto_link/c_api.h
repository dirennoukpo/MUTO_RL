#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct muto_handle muto_handle;

typedef struct muto_imu_angles {
    float roll;
    float pitch;
    float yaw;
    uint8_t temperature_c;
} muto_imu_angles;

typedef struct muto_raw_imu_data {
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;
    uint16_t mag_x;
    uint16_t mag_y;
    uint16_t mag_z;
} muto_raw_imu_data;

muto_handle* muto_create_usb(const char* port, int baud);
int muto_open(muto_handle* handle);
int muto_close(muto_handle* handle);
int muto_torque_on(muto_handle* handle);
int muto_torque_off(muto_handle* handle);
int muto_servo_move(muto_handle* handle, uint8_t id, int16_t angle, uint16_t speed);
int muto_read_servo_angle_deg(muto_handle* handle, uint8_t id, int16_t* out_angle_deg);
int muto_get_imu_angles(muto_handle* handle, muto_imu_angles* out);
int muto_get_raw_imu_data(muto_handle* handle, muto_raw_imu_data* out);
const char* muto_last_error(muto_handle* handle);
void muto_destroy(muto_handle* handle);

#ifdef __cplusplus
}
#endif

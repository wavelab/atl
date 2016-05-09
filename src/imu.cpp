#include "imu.hpp"


Accelerometer::Accelerometer(void)
{
    this->x = 0.0f;
    this->y = 0.0f;
    this->z = 0.0f;
}

Gyroscope::Gyroscope(void)
{
    this->x = 0.0f;
    this->y = 0.0f;
    this->z = 0.0f;
}

Magnetometer::Magnetometer(void)
{
    this->x = 0.0f;
    this->y = 0.0f;
    this->z = 0.0f;
}

IMU::IMU(void)
{
    this->mpu9250 = new MPU9250();
	this->mpu9250->initialize();

    this->accel_data = new Accelerometer();
    this->gyro_data = new Gyroscope();
    this->mag_data = new Magnetometer();
}

void IMU::read(void)
{
    this->mpu9250->update();

    this->mpu9250->read_accelerometer(
        &this->accel_data->x,
        &this->accel_data->y,
        &this->accel_data->z
    );

    this->mpu9250->read_gyroscope(
        &this->gyro_data->x,
        &this->gyro_data->y,
        &this->gyro_data->z
    );

    this->mpu9250->read_magnetometer(
        &this->mag_data->x,
        &this->mag_data->y,
        &this->mag_data->z
    );
}

void IMU::print(void)
{
    printf("Acc: %+7.3f %+7.3f %+7.3f  ",
        this->accel_data->x,
        this->accel_data->y,
        this->accel_data->z
    );

    printf("Gyr: %+8.3f %+8.3f %+8.3f  ",
        this->gyro_data->x,
        this->gyro_data->y,
        this->gyro_data->z
    );

    printf("Mag: %+7.3f %+7.3f %+7.3f\n",
        this->mag_data->x,
        this->mag_data->y,
        this->mag_data->z
    );
}

void IMU::calculateOrientation(void)
{
    float x;
    float y;
    float z;
    int t_diff;
    clock_t now;

    /* setup */
    x = this->accel_data->x;
    y = this->accel_data->y;
    z = this->accel_data->z;


    /* calculate pitch and roll from accelerometer */
    this->accel_data->pitch = (atan(x / sqrt(pow(y, 2) + pow(z, 2)))) * 180 / M_PI;
    this->accel_data->roll = (atan(y / sqrt(pow(x, 2) + pow(z, 2)))) * 180 / M_PI;

    this->pitch = (0.98 * this->gyro->pitch) + (0.02 * this->accel->pitch);
    this->roll = (0.98 * this->gyro->roll) + (0.02 * this->accel->roll);

    /* calculate pitch and roll from gyroscope */
    this->gyro_data->roll = (this->gyro_data->x * dt) + this->roll;
    this->gyro_data->pitch = (this->gyro_data->y * dt) + this->pitch;
}

void IMU::update(void)
{


}

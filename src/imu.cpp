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

    this->last_updated = -1;
}

void IMU::calculateOrientationCF(void)
{
    float x;
    float y;
    float z;
    float dt;
    clock_t now;

    // pre-check
    if (this->last_updated == -1) {
        this->last_updated = clock();
        return;
    }

    /* setup */
    x = this->accel_data->x;
    y = this->accel_data->y;
    z = this->accel_data->z;

    // calculate dt
    now = clock();
    dt = ((double) now - this->last_updated) / CLOCKS_PER_SEC;

    // calculate pitch and roll from accelerometer
    this->accel_data->roll = (atan(x / sqrt(pow(y, 2) + pow(z, 2))));
    this->accel_data->pitch = (atan(y / sqrt(pow(x, 2) + pow(z, 2))));

    // complimentary filter
    this->roll = (0.8 * this->gyro_data->roll) + (0.2 * this->accel_data->roll);
    this->pitch = (0.8 * this->gyro_data->pitch) + (0.2 * this->accel_data->pitch);

    // calculate pitch and roll from gyroscope
    this->gyro_data->roll = (this->gyro_data->y * dt) + this->roll;
    this->gyro_data->pitch = (this->gyro_data->x * dt) + this->pitch;

    // update last_updated
    this->last_updated = clock();
}

void IMU::update(void)
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

    this->calculateOrientationCF();
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

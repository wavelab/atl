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





// IMU
IMU::IMU(void)
{
    this->mpu9250 = new MPU9250();
	this->mpu9250->initialize();

    this->accel_data = new Accelerometer();
    this->gyro_data = new Gyroscope();
    this->mag_data = new Magnetometer();

    this->last_updated = -1;
}

void IMU::calibrateGyroscope(void)
{
    float gx;
    float gy;
    float gz;
    float offset[3];

    for (int i = 0; i < 100; i++) {
        this->mpu9250->update();
        this->mpu9250->read_gyroscope(&gx, &gy, &gz);

        gx *= 180.0 / M_PI;
        gy *= 180.0 / M_PI;
        gz *= 180.0 / M_PI;

        offset[0] += (-gx * 0.0175);
        offset[1] += (-gy * 0.0175);
        offset[2] += (-gz * 0.0175);

        usleep(10000);
    }

    offset[0] /= 100.0;
    offset[1] /= 100.0;
    offset[2] /= 100.0;

    this->gyro_data->offset_x = offset[0];
    this->gyro_data->offset_y = offset[1];
    this->gyro_data->offset_z = offset[2];
}

void IMU::obtainHardIronErrors(void)
{
    int nb_samples;
    float mag_x_bound[2];
    float mag_y_bound[2];
    float mag_z_bound[2];

    // setup
    nb_samples = 1000;

    // initialize bound values
    this->update();
    mag_x_bound[0] = this->mag_data->x;
    mag_x_bound[0] = this->mag_data->x;

    mag_y_bound[0] = this->mag_data->y;
    mag_y_bound[0] = this->mag_data->y;

    mag_z_bound[0] = this->mag_data->z;
    mag_z_bound[0] = this->mag_data->z;

    // obtain min max for each magnetometer axis
    for (int i = 0; i < nb_samples; i++) {
        this->update();

        mag_x_bound[0] = std::min(this->mag_data->x, mag_x_bound[0]);
        mag_x_bound[1] = std::max(this->mag_data->x, mag_x_bound[1]);

        mag_y_bound[0] = std::min(this->mag_data->y, mag_y_bound[0]);
        mag_y_bound[1] = std::max(this->mag_data->y, mag_y_bound[1]);

        mag_z_bound[0] = std::min(this->mag_data->z, mag_z_bound[0]);
        mag_z_bound[1] = std::max(this->mag_data->z, mag_z_bound[1]);

        usleep(100);
        std::cout << "Collecting sample";
        std::cout << "(" << i + 1 << " out of " << nb_samples << ")" << std::endl;
    }

    // average min max for each magnetometer axis
    this->mag_data->offset_x = (mag_x_bound[0] + mag_x_bound[1]) / 2.0;
    this->mag_data->offset_y = (mag_y_bound[0] + mag_y_bound[1]) / 2.0;
    this->mag_data->offset_z = (mag_z_bound[0] + mag_z_bound[1]) / 2.0;
}

void IMU::obtainSoftIronErrors(void)
{


}

void IMU::calibrateMagnetometer(void)
{
    this->obtainHardIronErrors();

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

    // calculate yaw from magnetometer
    if (this->mag_data->y > 0) {
        this->yaw = 90 - atan2(this->mag_data->x, this->mag_data->y) * 180 / M_PI;
    } else if (this->mag_data->y < 0) {
        this->yaw = 270 - atan2(this->mag_data->x, this->mag_data->y) * 180 / M_PI;
    } else if (this->mag_data->y == 0 && this->mag_data->x < 0) {
        this->yaw = 180;
    } else if (this->mag_data->y == 0 && this->mag_data->x > 0) {
        this->yaw = 0.0;
    }

    // update last_updated
    this->last_updated = clock();
}

void IMU::update(void)
{
    this->mpu9250->update();

    // read sensor raw values
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

    // apply sensor offsets
    // this->accel_data->x = this->accel_data->x + this->accel_data->offset_x;
    // this->accel_data->y = this->accel_data->y + this->accel_data->offset_y;
    // this->accel_data->z = this->accel_data->z + this->accel_data->offset_z;
    //
    // this->gyro_data->x = this->gyro_data->x + this->gyro_data->offset_x;
    // this->gyro_data->y = this->gyro_data->y + this->gyro_data->offset_y;
    // this->gyro_data->z = this->gyro_data->z + this->gyro_data->offset_z;
    //
    // this->mag_data->x = this->mag_data->x + this->mag_data->offset_x;
    // this->mag_data->y = this->mag_data->y + this->mag_data->offset_y;
    // this->mag_data->z = this->mag_data->z + this->mag_data->offset_z;

    this->calculateOrientationCF();



    // if (this->last_updated != -1) {
    //     clock_t now = clock();
    //     float dt = ((double) now - this->last_updated) / CLOCKS_PER_SEC;
    //
    //     this->sensor_fusion->updateIMU(
    //         this->accel_data->x / 9.81,
    //         this->accel_data->y / 9.81,
    //         this->accel_data->z / 9.81,
    //         this->gyro_data->x,
    //         this->gyro_data->y,
    //         this->gyro_data->z,
    //         dt
    //     );
    //
    //     this->sensor_fusion->getEuler(
    //         &this->roll,
    //         &this->pitch,
    //         &this->yaw
    //     );
    // }
    //
    // this->last_updated = clock();
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

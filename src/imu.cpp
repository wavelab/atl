#include "imu.hpp"


Accelerometer::Accelerometer(void)
{
    this->x = 0.0f;
    this->y = 0.0f;
    this->z = 0.0f;
    this->offset_x = 0.0f;
    this->offset_y = 0.0f;
    this->offset_z = 0.0f;
}

Gyroscope::Gyroscope(void)
{
    this->x = 0.0f;
    this->y = 0.0f;
    this->z = 0.0f;
    this->offset_x = 0.0f;
    this->offset_y = 0.0f;
    this->offset_z = 0.0f;
}

Magnetometer::Magnetometer(void)
{
    this->x = 0.0f;
    this->y = 0.0f;
    this->z = 0.0f;
    this->offset_x = 0.0f;
    this->offset_y = 0.0f;
    this->offset_z = 0.0f;
}





// IMU
IMU::IMU(void)
{
    this->mpu9250 = new MPU9250();
	this->mpu9250->initialize();

	this->lsm9ds1 = new LSM9DS1();
    this->lsm9ds1->initialize();

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

    std::ofstream outfile;
    outfile.open("magnetometer.csv");

    // initialize bound values
    this->update();
    this->update();
    mag_x_bound[0] = this->mag_data->x;
    mag_x_bound[1] = this->mag_data->x;

    mag_y_bound[0] = this->mag_data->y;
    mag_y_bound[1] = this->mag_data->y;

    mag_z_bound[0] = this->mag_data->z;
    mag_z_bound[1] = this->mag_data->z;

    // obtain min max for each magnetometer axis
    for (int i = 0; i < nb_samples; i++) {
        this->update();

        mag_x_bound[0] = std::min(this->mag_data->x, mag_x_bound[0]);
        mag_x_bound[1] = std::max(this->mag_data->x, mag_x_bound[1]);

        mag_y_bound[0] = std::min(this->mag_data->y, mag_y_bound[0]);
        mag_y_bound[1] = std::max(this->mag_data->y, mag_y_bound[1]);

        mag_z_bound[0] = std::min(this->mag_data->z, mag_z_bound[0]);
        mag_z_bound[1] = std::max(this->mag_data->z, mag_z_bound[1]);

        usleep(100 * 1000);
        std::cout << "Collecting sample";
        std::cout << "(" << i + 1 << " out of " << nb_samples << ")" << std::endl;
        printf(
            "mag [x: %f, y: %f, z: %f]\n\n",
            this->mag_data->x,
            this->mag_data->y,
            this->mag_data->z
        );

        outfile << this->mag_data->x << ",";
        outfile << this->mag_data->y << ",";
        outfile << this->mag_data->z << std::endl;
    }

    outfile.close();

    // average min max for each magnetometer axis
    this->mag_data->offset_x = (mag_x_bound[0] + mag_x_bound[1]) / 2.0;
    this->mag_data->offset_y = (mag_y_bound[0] + mag_y_bound[1]) / 2.0;
    this->mag_data->offset_z = (mag_z_bound[0] + mag_z_bound[1]) / 2.0;

    printf("mag x [min: %f, max %f]\n", mag_x_bound[0], mag_x_bound[1]);
    printf("mag y [min: %f, max %f]\n", mag_y_bound[0], mag_y_bound[1]);
    printf("mag z [min: %f, max %f]\n", mag_z_bound[0], mag_z_bound[1]);

    printf("mag x offset: %f\n", this->mag_data->offset_x);
    printf("mag y offset: %f\n", this->mag_data->offset_y);
    printf("mag z offset: %f\n", this->mag_data->offset_z);
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
    this->yaw = atan2(this->mag_data->y, this->mag_data->x) * 180 / M_PI;
    if (this->yaw < 0) {
        this->yaw = 360 + this->yaw;
    }

    // update last_updated
    this->last_updated = clock();
}

void IMU::update(void)
{
    this->mpu9250->update();
    this->lsm9ds1->update();

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

    // this->mag_data->x = this->mag_data->x - this->mag_data->offset_x;
    // this->mag_data->y = this->mag_data->y - this->mag_data->offset_y;
    // this->mag_data->z = this->mag_data->z - this->mag_data->offset_z;

    this->mag_data->x = this->mag_data->x - 2.219;
    this->mag_data->y = this->mag_data->y - 38.869;
    this->mag_data->z = this->mag_data->z - 14.04;

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

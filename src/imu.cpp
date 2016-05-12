#include "imu.hpp"


Accelerometer::Accelerometer(void)
{
    this->x = 0.0f;
    this->y = 0.0f;
    this->z = 0.0f;

    this->offset_x = 0.0f;
    this->offset_y = 0.0f;
    this->offset_z = 0.0f;

    this->roll = 0.0f;
    this->pitch = 0.0f;
}

Gyroscope::Gyroscope(void)
{
    this->x = 0.0f;
    this->y = 0.0f;
    this->z = 0.0f;

    this->offset_x = 0.0f;
    this->offset_y = 0.0f;
    this->offset_z = 0.0f;

    this->roll = 0.0f;
    this->pitch = 0.0f;
}

Magnetometer::Magnetometer(void)
{
    this->x = 0.0f;
    this->y = 0.0f;
    this->z = 0.0f;

    this->offset_x = 0.0f;
    this->offset_y = 0.0f;
    this->offset_z = 0.0f;

    this->x_min = 0.0f;
    this->x_max = 0.0f;

    this->y_min = 0.0f;
    this->y_max = 0.0f;

    this->z_min = 0.0f;
    this->z_max = 0.0f;

    this->x_scale = 1.0f;
    this->y_scale = 1.0f;
    this->z_scale = 1.0f;

    this->bearing = 0.0f;
}





// IMU
IMU::IMU(void)
{
    this->mpu9250 = new MPU9250();
	this->mpu9250->initialize();

	this->lsm9ds1 = new LSM9DS1();
    this->lsm9ds1->initialize();

    this->accel = new Accelerometer();
    this->gyro = new Gyroscope();
    this->mag = new Magnetometer();

    this->last_updated = -1;
}

void IMU::calibrateGyroscope(void)
{
    float gx;
    float gy;
    float gz;
    float offset[3];

    // obtain average gyroscope rates
    for (int i = 0; i < 100; i++) {
        this->mpu9250->update();
        this->mpu9250->read_gyroscope(&gx, &gy, &gz);

        offset[0] += -gx;
        offset[1] += -gy;
        offset[2] += -gz;

        usleep(10000);
    }

    offset[0] /= 100.0;
    offset[1] /= 100.0;
    offset[2] /= 100.0;

    this->gyro->offset_x = offset[0];
    this->gyro->offset_y = offset[1];
    this->gyro->offset_z = offset[2];
}

void IMU::obtainHardIronErrors(void)
{
    clock_t time_start;
    float calibrate_duration;

    // setup
    time_start = clock();
    calibrate_duration = 20;

    // initialize bound values
    this->update();
    this->update();
    this->mag->x_min = this->mag->x;
    this->mag->x_max = this->mag->x;

    this->mag->y_min = this->mag->y;
    this->mag->y_max = this->mag->y;

    this->mag->z_min = this->mag->z;
    this->mag->z_max = this->mag->z;

    // obtain min max for each magnetometer axis
    // wave quadrotor in figure 8 pattern for 20 seconds
    while (((clock() - time_start) / CLOCKS_PER_SEC) < calibrate_duration) {
        this->update();

        this->mag->x_min = std::min(this->mag->x, this->mag->x_min);
        this->mag->x_max = std::max(this->mag->x, this->mag->x_max);

        this->mag->y_min = std::min(this->mag->y, this->mag->y_min);
        this->mag->y_max = std::max(this->mag->y, this->mag->y_max);

        this->mag->z_min = std::min(this->mag->z, this->mag->z_min);
        this->mag->z_max = std::max(this->mag->z, this->mag->z_max);
    }

    // average min max for each magnetometer axis
    this->mag->offset_x = (this->mag->x_min + this->mag->x_max) / 2.0;
    this->mag->offset_y = (this->mag->y_min + this->mag->y_max) / 2.0;
    this->mag->offset_z = (this->mag->z_min + this->mag->z_max) / 2.0;

    printf(
        "mag offset [x: %f, y: %f, z: %f]\n",
        this->mag->offset_x,
        this->mag->offset_y,
        this->mag->offset_z
    );
}

void IMU::obtainSoftIronErrors(void)
{
    float vmax[3];
    float vmin[3];
    float avgs[3];
    float avg;

    // calculate average distance of center
    vmax[0] = this->mag->x_max - this->mag->offset_x;
    vmax[1] = this->mag->y_max - this->mag->offset_y;
    vmax[2] = this->mag->z_max - this->mag->offset_z;

    vmin[0] = this->mag->x_min - this->mag->offset_x;
    vmin[1] = this->mag->y_min - this->mag->offset_y;
    vmin[2] = this->mag->z_min - this->mag->offset_z;

    // invert negatives
    for (int i = 0; i < 3; i++) {
        avgs[i] = (vmax[i] + (vmin[i] * -1)) / 2;
    }
    avg = (avgs[0] + avgs[1] + avgs[2]) / 3;

    // scale each axis to remove the eliptical shape
    this->mag->x_scale = avg / avgs[0];
    this->mag->y_scale = avg / avgs[1];
    this->mag->z_scale = avg / avgs[2];
}

void IMU::calibrateMagnetometer(void)
{
    this->obtainHardIronErrors();
    this->obtainSoftIronErrors();
}

void IMU::calculateOrientationCF(void)
{
    float ax;
    float ay;
    float az;
    float dt;
    clock_t now;

    // pre-check
    if (this->last_updated == -1) {
        this->last_updated = clock();
        return;
    }

    /* setup */
    ax = this->accel->x;
    ay = this->accel->y;
    az = this->accel->z;

    // calculate dt
    now = clock();
    dt = ((double) now - this->last_updated) / CLOCKS_PER_SEC;

    // calculate pitch and roll from accelerometer
    this->accel->roll = (atan(ax / sqrt(pow(ay, 2) + pow(az, 2))));
    this->accel->pitch = (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))));

    // complimentary filter
    this->roll = 0.8 * this->gyro->roll;
    this->roll += 0.2 * this->accel->roll;
    this->pitch = 0.8 * this->gyro->pitch;
    this->pitch += 0.2 * this->accel->pitch;

    // calculate pitch and roll from gyroscope
    this->gyro->roll = (this->gyro->y * dt) + this->roll;
    this->gyro->pitch = (this->gyro->x * dt) + this->pitch;

    // calculate yaw from magnetometer
    this->mag->bearing = atan2(this->mag->y, this->mag->x);
    this->mag->bearing = this->mag->bearing * 180 / M_PI;
    if (this->mag->bearing < 0) {
        this->mag->bearing = 360 + this->mag->bearing;
    }
    this->yaw = this->mag->bearing;

    // update last_updated
    this->last_updated = clock();
}

void IMU::update(void)
{
    this->mpu9250->update();
    this->lsm9ds1->update();

    // read sensor raw values
    this->mpu9250->read_accelerometer(
        &this->accel->x,
        &this->accel->y,
        &this->accel->z
    );

    this->mpu9250->read_gyroscope(
        &this->gyro->x,
        &this->gyro->y,
        &this->gyro->z
    );

    this->mpu9250->read_magnetometer(
        &this->mag->x,
        &this->mag->y,
        &this->mag->z
    );

    // apply sensor offsets
    this->accel->x = this->accel->x + this->accel->offset_x;
    this->accel->y = this->accel->y + this->accel->offset_y;
    this->accel->z = this->accel->z + this->accel->offset_z;

    this->gyro->x = this->gyro->x + this->gyro->offset_x;
    this->gyro->y = this->gyro->y + this->gyro->offset_y;
    this->gyro->z = this->gyro->z + this->gyro->offset_z;

    // this->mag->x = this->mag->x - this->mag->offset_x;
    // this->mag->y = this->mag->y - this->mag->offset_y;
    // this->mag->z = this->mag->z - this->mag->offset_z;

    this->mag->x = this->mag->x - 2.219;
    this->mag->y = this->mag->y - 38.869;
    this->mag->z = this->mag->z - 14.04;

    this->mag->x *= this->mag->x_scale;
    this->mag->y *= this->mag->y_scale;
    this->mag->z *= this->mag->z_scale;

    this->calculateOrientationCF();
}

void IMU::print(void)
{
    printf("Acc: %+7.3f %+7.3f %+7.3f  ",
        this->accel->x,
        this->accel->y,
        this->accel->z
    );

    printf("Gyr: %+8.3f %+8.3f %+8.3f  ",
        this->gyro->x,
        this->gyro->y,
        this->gyro->z
    );

    printf("Mag: %+7.3f %+7.3f %+7.3f\n",
        this->mag->x,
        this->mag->y,
        this->mag->z
    );
}

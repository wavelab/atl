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





// AHRS
void AHRS::update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        updateIMU(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt; // integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);      // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void AHRS::updateIMU(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    gx -= gyroOffset[0];
    gy -= gyroOffset[1];
    gz -= gyroOffset[2];

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt; // integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);      // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void AHRS::setGyroOffset(float offsetX, float offsetY, float offsetZ)
{
    gyroOffset[0] = offsetX;
    gyroOffset[1] = offsetY;
    gyroOffset[2] = offsetZ;
}

void AHRS::getEuler(float* roll, float* pitch, float* yaw)
{
    *roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)) * 180.0/M_PI;
    *pitch = asin(2*(q0*q2-q3*q1)) * 180.0/M_PI;
    *yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)) * 180.0/M_PI;
}

float AHRS::invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float AHRS::getW()
{
    return  q0;
}

float AHRS::getX()
{
    return  q1;
}

float AHRS::getY()
{
    return  q2;
}

float AHRS::getZ()
{
    return  q3;
}




// IMU
IMU::IMU(void)
{
    this->mpu9250 = new MPU9250();
	this->mpu9250->initialize();

    this->accel_data = new Accelerometer();
    this->gyro_data = new Gyroscope();
    this->mag_data = new Magnetometer();

    this->sensor_fusion = new AHRS();
    this->sensor_fusion->setGyroOffset(0.0, 0.0, 0.0);

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

    // this->calculateOrientationCF();



    if (this->last_updated != -1) {
        clock_t now = clock();
        float dt = ((double) now - this->last_updated) / CLOCKS_PER_SEC;

        this->sensor_fusion->updateIMU(
            this->accel_data->x / 9.81,
            this->accel_data->y / 9.81,
            this->accel_data->z / 9.81,
            this->gyro_data->x,
            this->gyro_data->y,
            this->gyro_data->z,
            dt
        );

        this->sensor_fusion->getEuler(
            &this->roll,
            &this->pitch,
            &this->yaw
        );
    }

    this->last_updated = clock();
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

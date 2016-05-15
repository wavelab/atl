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

void Accelerometer::saveConfiguration(const std::string config_path)
{
    YAML::Emitter yaml;
    std::ofstream config_file;

    // setup
    config_file.open(config_path);

    // record gyroscope
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "accelerometer";
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "offset_x";
        yaml << YAML::Value << this->offset_x;
        yaml << YAML::Key << "offset_y";
        yaml << YAML::Value << this->offset_y;
        yaml << YAML::Key << "offset_z";
        yaml << YAML::Value << this->offset_z;
        yaml << YAML::EndMap;
    yaml << YAML::EndMap;

    // write to file
    config_file << yaml.c_str() << std::endl;
    config_file.close();
    chmod(config_path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
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

void Gyroscope::saveConfiguration(const std::string config_path)
{
    YAML::Emitter yaml;
    std::ofstream config_file;

    // setup
    config_file.open(config_path);

    // record gyroscope
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "gyroscope";
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "offset_x";
        yaml << YAML::Value << this->offset_x;
        yaml << YAML::Key << "offset_y";
        yaml << YAML::Value << this->offset_y;
        yaml << YAML::Key << "offset_z";
        yaml << YAML::Value << this->offset_z;
        yaml << YAML::EndMap;
    yaml << YAML::EndMap;

    // write to file
    config_file << yaml.c_str() << std::endl;
    config_file.close();
    chmod(config_path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
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

    this->scale_x = 1.0f;
    this->scale_y = 1.0f;
    this->scale_z = 1.0f;

    this->bearing = 0.0f;
}

void Magnetometer::saveConfiguration(const std::string config_path)
{
    YAML::Emitter yaml;
    std::ofstream config_file;

    // record hard and soft iron errors
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "magnetometer";
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "offset_x";
        yaml << YAML::Value << this->offset_x;
        yaml << YAML::Key << "offset_y";
        yaml << YAML::Value << this->offset_y;
        yaml << YAML::Key << "offset_z";
        yaml << YAML::Value << this->offset_z;
        yaml << YAML::Key << "scale_x";
        yaml << YAML::Value << this->scale_x;
        yaml << YAML::Key << "scale_y";
        yaml << YAML::Value << this->scale_y;
        yaml << YAML::Key << "scale_z";
        yaml << YAML::Value << this->scale_z;
        yaml << YAML::EndMap;
    yaml << YAML::EndMap;

    // write to file
    config_file << yaml.c_str() << std::endl;
    config_file.close();
    chmod(config_path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
}





// IMU
IMU::IMU(void)
{
    this->state = IMU_IDLE;

    this->mpu9250 = new MPU9250();
	this->lsm9ds1 = new LSM9DS1();

    this->accel = new Accelerometer();
    this->gyro = new Gyroscope();
    this->mag = new Magnetometer();

    this->last_updated = -1;
}

void IMU::initialize(void)
{
	this->mpu9250->initialize();
    this->lsm9ds1->initialize();
    this->state = IMU_RUNNING;
}

int IMU::update(void)
{
    // pre-check
    if (this->state == IMU_IDLE) {
        return -1;
    }

    // poll sensors
    this->mpu9250->update();
    this->lsm9ds1->update();

    // read sensor raw values
    this->mpu9250->read_gyroscope(
        &this->gyro->x,
        &this->gyro->y,
        &this->gyro->z
    );

    this->mpu9250->read_accelerometer(
        &this->accel->x,
        &this->accel->y,
        &this->accel->z
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

    this->mag->x = (this->mag->x - this->mag->offset_x) * this->mag->scale_x;
    this->mag->y = (this->mag->y - this->mag->offset_y) * this->mag->scale_y;
    this->mag->z = (this->mag->z - this->mag->offset_z) * this->mag->scale_z;

    // fuse imu data
    this->calculateOrientationCF();

    return 0;
}

int IMU::calibrateGyroscope(const std::string config_path)
{
    float gx;
    float gy;
    float gz;
    float offset[3];

    // pre-check
    if (this->state == IMU_IDLE) {
        return -1;

    } else if (this->state == IMU_UNIT_TESTING) {
        offset[0] = 100.0;
        offset[1] = 200.0;
        offset[2] = 300.0;
        goto GYRO_OFFSET_CALC;

    } else {
        offset[0] = 0.0;
        offset[1] = 0.0;
        offset[2] = 0.0;

    }

    // obtain average gyroscope rates
    std::cout << GYROSCOPE_CALIBRATION_INSTRUCTIONS;
    for (int i = 0; i < 100; i++) {
        this->mpu9250->update();
        this->mpu9250->read_gyroscope(&gx, &gy, &gz);

        offset[0] += -gx;
        offset[1] += -gy;
        offset[2] += -gz;

        usleep(10000);
    }

GYRO_OFFSET_CALC:
    offset[0] /= 100.0;
    offset[1] /= 100.0;
    offset[2] /= 100.0;

    this->gyro->offset_x = offset[0];
    this->gyro->offset_y = offset[1];
    this->gyro->offset_z = offset[2];
    this->gyro->saveConfiguration(config_path);

    return 0;
}

void IMU::printAccelerometerCalibrationInstructions(int i)
{
    switch (i) {
    case 0:
        std::cout << "Place quadrotor flat on the ground" << std::endl;
        break;
    case 1:
        std::cout << "Place quadrotor upside down" << std::endl;
        break;
    case 2:
        std::cout << "Place quadrotor on its left side" << std::endl;
        break;
    case 3:
        std::cout << "Place quadrotor on its right side" << std::endl;
        break;
    case 4:
        std::cout << "Quadrotor nose up" << std::endl;
        break;
    case 5:
        std::cout << "Quadrotor nose down" << std::endl;
        break;
    }
    std::cout << "Press ENTER when you have done so" << std::endl;
}

void IMU::recordAccelerometerBounds(int i, float *bounds, float *data)
{
    switch (i) {
    case 0:
        bounds[5] = std::max(data[2], bounds[5]);
        break;
    case 1:
        bounds[4] = std::min(data[2], bounds[4]);
        break;
    case 2:
        bounds[3] = std::max(data[0], bounds[3]);
        break;
    case 3:
        bounds[2] = std::min(data[0], bounds[2]);
        break;
    case 4:
        bounds[1] = std::max(data[1], bounds[1]);
        break;
    case 5:
        bounds[0] = std::min(data[1], bounds[0]);
        break;
    }
}

void IMU::printAccelerometerData(char *line, float *data)
{
    for (int j = 0; j < (int) strlen(line); j++) {
        printf("\b \b");
    }
    memset(line, '\0', strlen(line));
    sprintf(line, "x: %f, y: %f, z: %f     ", data[0], data[1], data[2]);
    printf("%s", line);
    fflush(stdout);
}

int IMU::obtainAccelerometerBounds(float *bounds, char *line)
{
    char c;
    float data[3];

    // obtain accelerometer bounds
    for (int i = 0; i < 6; i++) {
        printAccelerometerCalibrationInstructions(i);

        while (1) {
            this->mpu9250->update();
            this->mpu9250->read_accelerometer(&data[0], &data[1], &data[2]);
            recordAccelerometerBounds(i, bounds, data);

            // print accelerometer data
            printAccelerometerData(line, data);

            // keyboard event
            if (kbhit()) {
                c = fgetc(stdin);
                printf("\n\n");

                // exit if 'q' was pressed
                if (c == 'q') {
                    return -2;
                } else if (c == 10) {
                    break;
                }
            }
        }
    }

    return 0;
}

int IMU::calibrateAccelerometer(const std::string config_path)
{
    char line[50];
    float bounds[6];
    YAML::Emitter yaml;
    std::ofstream config_file;

    // pre-check
    if (this->state == IMU_IDLE) {
        return -1;

    } else if (this->state == IMU_UNIT_TESTING) {
        bounds[1] = 2;
        bounds[0] = 2;
        bounds[3] = 2;
        bounds[2] = 2;
        bounds[5] = 2;
        bounds[4] = 2;

        goto ACCEL_OFFSET_CALC;
    }

    // setup
    memset(line, '\0', 50);
    config_file.open(config_path);
    nonblock(NONBLOCK_ENABLE);
    for (int i = 0; i < 6; i++) {
        bounds[0] = 0.0f;
    }

    // obtain average gyroscope rates
    std::cout << ACCELEROMETER_CALIBRATION_INSTRUCTIONS;
    if (obtainAccelerometerBounds(bounds, line) == -2) {
        return -2;
    }


ACCEL_OFFSET_CALC:
    this->accel->offset_x = (bounds[1] + bounds[0]) / 2.0;
    this->accel->offset_y = (bounds[3] + bounds[2]) / 2.0;
    this->accel->offset_z = (bounds[5] + bounds[4]) / 2.0;
    this->accel->saveConfiguration(config_path);

    return 0;
}

int IMU::obtainHardIronErrors(const std::string record_path)
{
    clock_t time_start;
    float calibrate_duration;
    std::ofstream record_file;

    // pre-check
    if (this->state == IMU_IDLE) {
        return -1;

    } else if (this->state == IMU_UNIT_TESTING) {
        this->mag->x_min = 0.0;
        this->mag->x_max = 1.0;
        this->mag->y_min = 0.0;
        this->mag->y_max = 2.0;
        this->mag->z_min = 0.0;
        this->mag->z_max = 3.0;

        goto MAG_OFFSET_CALC;
    }

    // setup
    time_start = clock();
    calibrate_duration = 20;
    record_file.open(record_path);
    record_file << "x,y,z" << std::endl;

    // initialize bound values
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

        record_file << this->mag->x << ",";
        record_file << this->mag->y << ",";
        record_file << this->mag->z << std::endl;
    }
    record_file.close();
    chmod(record_path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);

MAG_OFFSET_CALC:
    // average min max for each magnetometer axis
    this->mag->offset_x = (this->mag->x_min + this->mag->x_max) / 2.0;
    this->mag->offset_y = (this->mag->y_min + this->mag->y_max) / 2.0;
    this->mag->offset_z = (this->mag->z_min + this->mag->z_max) / 2.0;

    return 0;
}

int IMU::obtainSoftIronErrors(void)
{
    float vmax[3];
    float vmin[3];
    float avgs[3];
    float avg;

    // pre-check
    if (this->state == IMU_IDLE) {
        return -1;

    } else if (this->state == IMU_UNIT_TESTING) {
        this->mag->x_min = 0.0;
        this->mag->x_max = 1.0;
        this->mag->y_min = 0.0;
        this->mag->y_max = 2.0;
        this->mag->z_min = 0.0;
        this->mag->z_max = 3.0;

    }

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
    this->mag->scale_x = avg / avgs[0];
    this->mag->scale_y = avg / avgs[1];
    this->mag->scale_z = avg / avgs[2];

    return 0;
}

int IMU::calibrateMagnetometer(
    const std::string config_path,
    const std::string record_path
)
{
    // pre-check
    if (this->state == IMU_IDLE) {
        return -1;
    }

    // obtain hard and soft iron errors
    std::cout << MAGNETOMETER_CALIBRATION_INSTRUCTIONS;
    if (this->obtainHardIronErrors(record_path) == -1) {
        return -2;
    }

    if (this->obtainSoftIronErrors() == -1) {
        return -3;
    }

    this->mag->saveConfiguration(config_path);

    return 0;
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

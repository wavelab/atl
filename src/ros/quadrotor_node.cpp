#include "awesomo/ros/quadrotor_node.hpp"


static int fltcmp(double f1, double f2)
{
	if (fabs(f1 - f2) <= 0.0001) {
		return 0;
	} else if (f1 > f2) {
		return 1;
	} else {
		return -1;
	}
}

Quadrotor::Quadrotor(void)
{
    // wait till connected to FCU
    this->waitForConnection();

    // subscribe to topics
    this->subscribeToPose();
    this->subscribeToMocap();
    this->subscribeToIMU();

    // initialize clients to services
    this->mode_client = this->node.serviceClient<mavros_msgs::SetMode>(MODE_TOPIC);
    this->arming_client = this->node.serviceClient<mavros_msgs::CommandBool>(ARM_TOPIC);

    // initialize publishers
    this->position_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_TOPIC, 50);
    this->attitude_publisher = this->node.advertise<geometry_msgs::PoseStamped>(ATTITUDE_TOPIC, 50);
    this->throttle_publisher = this->node.advertise<std_msgs::Float64>(THROTTLE_TOPIC, 50);
    this->position_controller_x_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_X_CONTROLLER_TOPIC, 50);
    this->position_controller_y_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_Y_CONTROLLER_TOPIC, 50);
    this->position_controller_z_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_Z_CONTROLLER_TOPIC, 50);


    // state
    this->mission_state = HOVER_MODE;

    // initialize camera
    this->tag_timeout = 0;
    this->cam = new Camera(0, CAMERA_FIREFLY);
    cam->loadConfig("default", FIREFLY_640);
    cam->loadConfig("320", FIREFLY_320);
    cam->loadConfig("160", FIREFLY_160);
    cam->initCamera("320");

    // intialize carrot controller
    double look_ahead_dist = 0.2;
    double wp_threshold = 0.3;
    std::deque<Eigen::Vector3d> waypoints;
    this->carrot_controller = new CarrotController(
        waypoints,
        look_ahead_dist,
        wp_threshold
    );

    // initialize position controller
    this->position_controller = new PositionController(PID_CONFIG);
}

void Quadrotor::poseCallback(const geometry_msgs::PoseStamped &msg)
{
    //  mocapposition
    this->pose_x = msg.pose.position.x;
    this->pose_y = msg.pose.position.y;
    this->pose_z = msg.pose.position.z;

    //  mocaporientation
    quat2euler(msg.pose.orientation, &this->pose_roll, &this->pose_pitch, &this->pose_yaw);

    // print
    // ROS_INFO(
    //     "GOT POSE: [roll: %f, pitch: %f, yaw: %f, x: %f, y: %f, z: %f]",
    //     rad2deg(this->pose_roll),
    //     rad2deg(this->pose_pitch),
    //     rad2deg(this->pose_yaw),
    //     pose_x,
    //     pose_y,
    //     pose_z
    // );
}

void Quadrotor::subscribeToPose(void)
{
    ROS_INFO("subcribing to [POSE]");
    this->pose_subscriber = this->node.subscribe(
        POSE_TOPIC,
        50,
        &Quadrotor::poseCallback,
        this
    );
}

void Quadrotor::mocapCallback(const geometry_msgs::PoseStamped &msg)
{
    // mocap position
    this->mocap_x = msg.pose.position.x;
    this->mocap_y = msg.pose.position.y;
    this->mocap_z = msg.pose.position.z;

    // mocap orientation
    quat2euler(msg.pose.orientation, &this->mocap_roll, &this->mocap_pitch, &this->mocap_yaw);

    // print
    // ROS_INFO(
    //     "GOT MOCAP: [%f, %f, %f]",
    //     rad2deg(this->mocap_roll),
    //     rad2deg(this->mocap_pitch),
    //     rad2deg(this->mocap_yaw)
    // );
}

void Quadrotor::subscribeToMocap(void)
{
    ROS_INFO("subcribing to [MOCAP]");
    this->mocap_subscriber = this->node.subscribe(
        MOCAP_TOPIC,
        50,
        &Quadrotor::mocapCallback,
        this
    );
}

void Quadrotor::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    quat2euler(msg->orientation, &this->roll, &this->pitch, &this->yaw);
    // ROS_INFO(
    //     "GOT IMU: [%f, %f, %f]",
    //     rad2deg(this->roll),
    //     rad2deg(this->pitch),
    //     rad2deg(this->yaw)
    // );
}

void Quadrotor::subscribeToIMU(void)
{
    ROS_INFO("subcribing to [IMU_DATA]");
    this->imu_subscriber = this->node.subscribe(
        IMU_TOPIC,
        50,
        &Quadrotor::imuCallback,
        this
    );
}

void Quadrotor::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    state = *msg;
}

void Quadrotor::waitForConnection(void)
{
    ROS_INFO("waiting for FCU ...");

    while (ros::ok() && this->state.connected) {
        ros::spinOnce();
        sleep(1);
    }
}

int Quadrotor::arm(void)
{
    mavros_msgs::CommandBool arm_req;

    // setup
    ROS_INFO("arming awesomo ...");
    arm_req.request.value = true;

    // arm
    if (this->arming_client.call(arm_req)) {
        ROS_INFO("awesomo armed!");
    } else {
        ROS_ERROR("failed to arm awesomo!");
    }

    return 0;
}

int Quadrotor::disarm(void)
{
    mavros_msgs::CommandBool arm_req;

    // setup
    ROS_INFO("disarming awesomo ...");
    arm_req.request.value = false;

    // arm
    if (this->arming_client.call(arm_req)) {
        ROS_INFO("awesomo disarmed!");
    } else {
        ROS_ERROR("failed to disarm awesomo!");
    }

    return 0;
}

int Quadrotor::setOffboardModeOn(void)
{
    mavros_msgs::SetMode mode;

    // setup
    mode.request.custom_mode = "OFFBOARD";

    if (mode_client.call(mode) && mode.response.success) {
        ROS_INFO("Offboard enabled");
        return 0;
    } else {
        return -1;
    }
}

void Quadrotor::runMission(geometry_msgs::PoseStamped &msg)
{
    double dist;
    Eigen::Vector3d pos;
    Eigen::Vector3d wp;
    Eigen::Vector3d dest;
    Eigen::Vector3d carrot;
	std::vector<TagPose> pose_estimates;

    // msg.header.stamp = ros::Time::now();
    // msg.header.seq = seq;
    // msg.header.frame_id = "awesomo_quad_offboard";

    switch (this->mission_state) {
    case HOVER_MODE:
        // set hover coordinates
        msg.pose.position.x = -0.75;
        msg.pose.position.y = -0.75;
		msg.pose.position.z = 1.6;

        // check current position against hover coordinates
        dest << -0.75, -0.75, 1.6;
        pos << this->pose_x, this->pose_y, this->pose_z;
        dist = (dest - pos).norm();

        // check if waypoint reached
        if (dist < 1) {
            ROS_INFO("Hover state reached!");
            ROS_INFO("Transitioning to discover mode!");
            this->mission_state = DISCOVER_MODE;
        }
        break;

    case DISCOVER_MODE:
        // set hover coordinates
        msg.pose.position.x = -0.75;
        msg.pose.position.y = -0.75;
		msg.pose.position.z = 1.6;

        // find apriltag
        pose_estimates = cam->step(this->tag_timeout);
        if (pose_estimates.size()) {
            this->tag_poses.push_back(pose_estimates.at(0));
        }

        // average estimates
        if (tag_poses.size() == 20) {
            double x = 0;
            double y = 0;
            double z = 0;

            for (int i = 0; i < tag_poses.size(); i++) {
                x = x + tag_poses[i].translation(0);
                y = y + tag_poses[i].translation(1);
                z = z + tag_poses[i].translation(2);
            }

            // quad position
            pos << this->pose_x, this->pose_y, this->pose_z;
            // tag position relative to quad (ENU)
            this->tag_position << y / 20.0 , z / 20.0, x / 20.0;
            // tag positiona in world (ENU)
            this->tag_position(0) += pos(0);
            this->tag_position(1) += pos(1);
            this->tag_position(2) -= pos(2);

            ROS_INFO("Collected enough tag estimations!");
            ROS_INFO("Transitioning to planning mode!");
            this->mission_state = PLANNING_MODE;
        }

        break;

    case PLANNING_MODE:
        // set hover coordinates
        msg.pose.position.x = -0.75;
        msg.pose.position.y = -0.75;
		msg.pose.position.z = 1.6;

        // quad position
        pos << this->pose_x, this->pose_y, this->pose_z;
        this->carrot_controller->waypoints.push_back(pos);
        ROS_INFO(
            "WAYPOINT 1 [Quad Position]: (%f, %f, %f)",
            pos(0),
            pos(1),
            pos(2)
        );
        this->carrot_controller->wp_start << pos(0), pos(1), pos(2);

        // top right
        wp << -0.75, 0.75, 1.6;
        this->carrot_controller->wp_end << wp;
        this->carrot_controller->waypoints.push_back(wp);

        wp << 0.75, 0.75, 1.6;
        this->carrot_controller->waypoints.push_back(wp);

        wp << 0.75, -0.75, 1.6;
        this->carrot_controller->waypoints.push_back(wp);

        wp << 0, -0.75, 1.6;
        this->carrot_controller->waypoints.push_back(wp);

        // above tag position
        wp << this->tag_position(0), this->tag_position(1), 1.6;
        this->carrot_controller->waypoints.push_back(wp);
        ROS_INFO(
            "WAYPOINT 2 [Above Tag]: (%f, %f, %f)",
            wp(0),
            wp(1),
            wp(2)
        );

        // tag position
        this->carrot_controller->waypoints.push_back(this->tag_position);
        ROS_INFO(
            "WAYPOINT 3 [Tag Position]: (%f, %f, %f)",
            this->tag_position(0),
            this->tag_position(1),
            this->tag_position(2) + 0.3
        );

        // complete planning
        this->carrot_controller->initialized = 1;
        this->mission_state = CARROT_MODE;
        ROS_INFO("Planning complete");
        ROS_INFO("Transitioning to carrot mode!");

        break;

    case CARROT_MODE:
        pos << this->pose_x, this->pose_y, this->pose_z;

        if (this->carrot_controller->initialized == 0) {
            this->mission_state = HOVER_MODE;
            ROS_INFO("Controller not initialized");

        } else if (this->carrot_controller->update(pos, carrot)) {
            msg.pose.position.x = carrot(0);
            msg.pose.position.y = carrot(1);
            msg.pose.position.z = carrot(2);
            ROS_INFO(
                "Waypoint Start (%f, %f, %f)",
                this->carrot_controller->wp_start(0),
                this->carrot_controller->wp_start(1),
                this->carrot_controller->wp_start(2)
            );
            ROS_INFO(
                "Waypoint End (%f, %f, %f)",
                this->carrot_controller->wp_end(0),
                this->carrot_controller->wp_end(1),
                this->carrot_controller->wp_end(2)
            );
            ROS_INFO(
                "Carrot Point (%f, %f, %f)",
                carrot(0),
                carrot(1),
                carrot(2)
            );
        } else {
            // set hover coordinates
            msg.pose.position.x = 0;
            msg.pose.position.y = 0;
            msg.pose.position.z = 0;

            ROS_INFO("Final Waypoint reached");
            ROS_INFO("MISSION ACCOMPLISHED!");
            this->mission_state = MISSION_ACCOMPLISHED;
        }
        break;
    }
}

void Quadrotor::positionControllerCalculate(float x, float y, float z, ros::Time last_request)
{
    float roll;
    float pitch;
    float throttle;
    float roll_adjusted;
    float pitch_adjusted;
    float throttle_adjusted;
    ros::Duration dt;

    // configure x, y, z setpoint
    this->position_controller->x.setpoint = y;
    this->position_controller->y.setpoint = x;
    this->position_controller->T.setpoint = z;

    // position controller - calculate
    this->position_controller->dt = ros::Time::now() - last_request;
    dt = this->position_controller->dt;
    pid_calculate(&this->position_controller->x, this->pose_y, dt);
    pid_calculate(&this->position_controller->y, this->pose_x, dt);
    pid_calculate(&this->position_controller->T, this->pose_z, dt);
    roll = -this->position_controller->x.output;
    pitch = this->position_controller->y.output;
    throttle = this->position_controller->T.output;

    // adjust roll and pitch according to yaw
    if (this->pose_yaw < 0) {
        this->pose_yaw += 2 * M_PI;
    }
    roll_adjusted = cos(this->pose_yaw) * roll - sin(this->pose_yaw) * pitch;
    pitch_adjusted = sin(this->pose_yaw) * roll + cos(this->pose_yaw) * pitch;

    // throttle
    throttle_adjusted = this->position_controller->hover_throttle + throttle;
    throttle_adjusted = throttle_adjusted / (cos(roll_adjusted) * cos(pitch_adjusted));

    // ROS_INFO("setpoint (%f, %f, %f)", x, y, z);
    // ROS_INFO("actual (%f, %f, %f)", this->pose_x, this->pose_y, this->pose_z);
    // ROS_INFO("roll: %f \t pitch: %f\n", roll_adjusted, pitch_adjusted);
    // ROS_INFO("throttle: %f", throttle_adjusted);

    // update position controller
    this->position_controller->roll = roll_adjusted;
    this->position_controller->pitch = pitch_adjusted;
    this->position_controller->rpy_quat = euler2quat(roll_adjusted, pitch_adjusted, 0);
    this->position_controller->throttle = throttle_adjusted;
}

void Quadrotor::printPositionController(void)
{
    ROS_INFO("---");
    ROS_INFO("dt %f", this->position_controller->dt.toSec());
    ROS_INFO("quadrotor.pose_x %f", this->pose_x);
    ROS_INFO("quadrotor.pose_y %f", this->pose_y);
    ROS_INFO("quadrotor.pose_z %f", this->pose_z);
    ROS_INFO("quadrotor.pose_yaw %f", rad2deg(this->pose_yaw));
    ROS_INFO("quadrotor.roll %f", rad2deg(this->roll));
    ROS_INFO("quadrotor.pitch %f", rad2deg(this->pitch));
    ROS_INFO("roll.controller %f", rad2deg(this->position_controller->roll));
    ROS_INFO("pitch.controller %f", rad2deg(this->position_controller->pitch));
    ROS_INFO("throttle.controller %f", this->position_controller->throttle);
    ROS_INFO("---");
}

void Quadrotor::buildAtitudeMessage(
    geometry_msgs::PoseStamped &msg,
    int seq,
    ros::Time time
)
{
    msg.header.stamp = time;
    msg.header.seq = seq;
    msg.header.frame_id = "awesomo_quad_offboard_attitude_cmd";
    msg.pose.position.x = this->pose_x;
    msg.pose.position.y = this->pose_y;
    msg.pose.position.z = this->pose_z;
    msg.pose.orientation.x = this->position_controller->rpy_quat.x();
    msg.pose.orientation.y = this->position_controller->rpy_quat.y();
    msg.pose.orientation.z = this->position_controller->rpy_quat.z();
    msg.pose.orientation.w = this->position_controller->rpy_quat.w();
}

void Quadrotor::buildThrottleMessage(std_msgs::Float64 &msg)
{
    msg.data = this->position_controller->throttle;
}

void Quadrotor::buildPositionControllerMessage(
    geometry_msgs::PoseStamped &msg,
    int seq,
    ros::Time time
)
{
    msg.header.stamp = time;
    msg.header.seq = seq;
    msg.header.frame_id = "awesomo_quad_offboard_position_controller";
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.x = this->position_controller->rpy_quat.x();
    msg.pose.orientation.y = this->position_controller->rpy_quat.y();
    msg.pose.orientation.z = this->position_controller->rpy_quat.z();
    msg.pose.orientation.w = this->position_controller->rpy_quat.w();
}

void Quadrotor::traceSquare(
    geometry_msgs::PoseStamped &pose,
    int *index,
    ros::Time &last_request
)
{
    if ((ros::Time::now() - last_request) > ros::Duration(10.0)) {
        if (*index == 0) {
            pose.pose.position.x = 0.5;
            pose.pose.position.y = 0.5;
            *index++;
            ROS_INFO("x: 0.5\ty: 0.5");
        } else if (*index == 1) {
            pose.pose.position.x = 0.5;
            pose.pose.position.y = -0.5;
            ROS_INFO("x: 0.5\ty: -0.5");
            *index++;
        } else if (*index == 2) {
            pose.pose.position.x = -0.5;
            pose.pose.position.y = -0.5;
            ROS_INFO("x: -0.5\ty: -0.5");
            *index++;
        } else if (*index == 3) {
            pose.pose.position.x = -0.5;
            pose.pose.position.y = 0.5;
            ROS_INFO("x: -0.5\ty: 0.5");
            *index = 0;
        }

        last_request = ros::Time::now();
    }
}

int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
    ros::NodeHandle nh;
    ros::Time last_request;
    ros::Time hover_time;
    ros::Time now;

    ros::Rate rate(50.0);  // publishing rate MUST be faster than 2Hz
    Quadrotor quad;
	geometry_msgs::PoseStamped position;
	geometry_msgs::PoseStamped attitude;
    std_msgs::Float64 throttle;
	geometry_msgs::PoseStamped pid_stats;

    ROS_INFO("running ...");
    // quad.arm();
    // quad.setOffboardModeOn();
	last_request = ros::Time::now();
	int seq = 1;
	int index = 0;
	int hover_timer_start = 0;

    ros::spinOnce();
	float x = quad.pose_x;
	float y = quad.pose_y;
	float z = quad.pose_z;
	ROS_INFO("quad (%f, %f, %f)", x, y, z);

    while (ros::ok()){
        // quad.runMission(position);

        if (index == 0) {
            if ((1.0 - quad.pose_z) < 0.1 && hover_timer_start == 1 && ((ros::Time::now() - hover_time) > ros::Duration(2))) {
                ROS_INFO("HOVER COMPLETE!");
                ROS_INFO("Now moving to (0, 0, 1)!");
                hover_timer_start = 0;
                index++;
            } else if ((1.0 - quad.pose_z) < 0.1 && hover_timer_start == 0) {
                ROS_INFO("HOVER Hold!");
                hover_timer_start = 1;
                hover_time = ros::Time::now();
            }
        // } else if (index == 1) {
        //     float dx = pow(0.0 - quad.pose_x, 2);
        //     float dy = pow(0.0 - quad.pose_y, 2);
        //     float dz = pow(1.0 - quad.pose_z, 2);
        //     float dist = sqrt(dx + dy + dz);
        //
        //     if (dist < 0.1 && hover_timer_start == 1 && ((ros::Time::now() - hover_time) > ros::Duration(2))) {
        //         ROS_INFO("LANDING!");
        //         index++;
        //     } else if (dist < 0.1 && hover_timer_start == 0) {
        //         hover_timer_start = 1;
        //         hover_time = ros::Time::now();
        //     }
        }

        // position controller
        if (index == 0) {
            quad.positionControllerCalculate(x, y, 1.3, last_request);
        // } else if (index == 1) {
        } else {
            quad.positionControllerCalculate(0, 0, 1.3, last_request);
        // } else if (index == 2) {
        //     quad.positionControllerCalculate(0, 0, 0.4, last_request);
        }
        // quad.printPositionController();
        last_request = ros::Time::now();
        now = last_request;

		// publish
		// quad.position_publisher.publish(position);

		quad.buildAtitudeMessage(attitude, seq, now);
        quad.attitude_publisher.publish(attitude);

		quad.buildThrottleMessage(throttle);
        quad.throttle_publisher.publish(throttle);

        pid_stats.header.stamp = now;
        pid_stats.header.seq = seq;
        pid_stats.header.frame_id = "awesomo_position_controller_x";
        pid_stats.pose.position.x = quad.position_controller->x.p_error;
        pid_stats.pose.position.y = quad.position_controller->x.i_error;
        pid_stats.pose.position.z = quad.position_controller->x.d_error;
        quad.position_controller_x_publisher.publish(pid_stats);

        pid_stats.header.stamp = now;
        pid_stats.header.seq = seq;
        pid_stats.header.frame_id = "awesomo_position_controller_y";
        pid_stats.pose.position.x = quad.position_controller->y.p_error;
        pid_stats.pose.position.y = quad.position_controller->y.i_error;
        pid_stats.pose.position.z = quad.position_controller->y.d_error;
        quad.position_controller_y_publisher.publish(pid_stats);

        pid_stats.header.stamp = now;
        pid_stats.header.seq = seq;
        pid_stats.header.frame_id = "awesomo_position_controller_T";
        pid_stats.pose.position.x = quad.position_controller->T.p_error;
        pid_stats.pose.position.y = quad.position_controller->T.i_error;
        pid_stats.pose.position.z = quad.position_controller->T.d_error;
        quad.position_controller_z_publisher.publish(pid_stats);

		// update
		seq++;

		// end
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}

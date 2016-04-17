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
    this->position_publisher = this->node.advertise<geometry_msgs::PoseStamped>(POSITION_TOPIC, 100);
    this->attitude_publisher = this->node.advertise<geometry_msgs::PoseStamped>(ATTITUDE_TOPIC, 100);
    this->throttle_publisher = this->node.advertise<std_msgs::Float64>(THROTTLE_TOPIC, 100);

    // state
    this->mission_state = HOVER_MODE;

    // initialize camera
    this->tag_timeout = 0;
    this->cam = new Camera(0, CAMERA_FIREFLY);
    cam->loadConfig("default", FIREFLY_640);
    cam->loadConfig("320", FIREFLY_320);
    cam->loadConfig("160", FIREFLY_160);
    cam->initCamera("320");

    // intialize controller
    double look_ahead_dist = 0.2;
    double wp_threshold = 0.3;
    std::deque<Eigen::Vector3d> waypoints;

    this->controller = new CarrotController(
        waypoints,
        look_ahead_dist,
        wp_threshold
    );
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

void Quadrotor::runMission(geometry_msgs::PoseStamped &pose)
{
    double dist;
    Eigen::Vector3d pos;
    Eigen::Vector3d wp;
    Eigen::Vector3d dest;
    Eigen::Vector3d carrot;
	std::vector<TagPose> pose_estimates;

    switch (this->mission_state) {
    case HOVER_MODE:
        // set hover coordinates
        pose.pose.position.x = -0.75;
        pose.pose.position.y = -0.75;
		pose.pose.position.z = 1.6;

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
        pose.pose.position.x = -0.75;
        pose.pose.position.y = -0.75;
		pose.pose.position.z = 1.6;

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
        pose.pose.position.x = -0.75;
        pose.pose.position.y = -0.75;
		pose.pose.position.z = 1.6;

        // quad position
        pos << this->pose_x, this->pose_y, this->pose_z;
        this->controller->waypoints.push_back(pos);
        ROS_INFO(
            "WAYPOINT 1 [Quad Position]: (%f, %f, %f)",
            pos(0),
            pos(1),
            pos(2)
        );
        this->controller->wp_start << pos(0), pos(1), pos(2);

        // top right
        wp << -0.75, 0.75, 1.6;
        this->controller->wp_end << wp;
        this->controller->waypoints.push_back(wp);

        wp << 0.75, 0.75, 1.6;
        this->controller->waypoints.push_back(wp);

        wp << 0.75, -0.75, 1.6;
        this->controller->waypoints.push_back(wp);

        wp << 0, -0.75, 1.6;
        this->controller->waypoints.push_back(wp);

        // above tag position
        wp << this->tag_position(0), this->tag_position(1), 1.6;
        this->controller->waypoints.push_back(wp);
        ROS_INFO(
            "WAYPOINT 2 [Above Tag]: (%f, %f, %f)",
            wp(0),
            wp(1),
            wp(2)
        );

        // tag position
        this->controller->waypoints.push_back(this->tag_position);
        ROS_INFO(
            "WAYPOINT 3 [Tag Position]: (%f, %f, %f)",
            this->tag_position(0),
            this->tag_position(1),
            this->tag_position(2) + 0.3
        );

        // complete planning
        this->controller->initialized = 1;
        this->mission_state = CARROT_MODE;
        ROS_INFO("Planning complete");
        ROS_INFO("Transitioning to carrot mode!");

        break;

    case CARROT_MODE:
        pos << this->pose_x, this->pose_y, this->pose_z;

        if (this->controller->initialized == 0) {
            this->mission_state = HOVER_MODE;
            ROS_INFO("Controller not initialized");

        } else if (this->controller->update(pos, carrot)) {
            pose.pose.position.x = carrot(0);
            pose.pose.position.y = carrot(1);
            pose.pose.position.z = carrot(2);
            ROS_INFO(
                "Waypoint Start (%f, %f, %f)",
                this->controller->wp_start(0),
                this->controller->wp_start(1),
                this->controller->wp_start(2)
            );
            ROS_INFO(
                "Waypoint End (%f, %f, %f)",
                this->controller->wp_end(0),
                this->controller->wp_end(1),
                this->controller->wp_end(2)
            );
            ROS_INFO(
                "Carrot Point (%f, %f, %f)",
                carrot(0),
                carrot(1),
                carrot(2)
            );
        } else {
            // set hover coordinates
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 0;

            ROS_INFO("Final Waypoint reached");
            ROS_INFO("MISSION ACCOMPLISHED!");
            this->mission_state = MISSION_ACCOMPLISHED;
        }
        break;
    }
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
    ros::Duration dt;
    ros::Time last_request;

    ros::Rate rate(100.0);  // publishing rate MUST be faster than 2Hz
    Quadrotor quad;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped attitude;
    std_msgs::Float64 throttle;
    tf::Quaternion q;

    ROS_INFO("running ...");
    // quad.arm();
    // quad.setOffboardModeOn();
	last_request = ros::Time::now();
	int count = 1;
	int index = 0;


    struct pid x_controller;
    struct pid y_controller;

    // x controller
    x_controller.setpoint = 0.0;
    x_controller.dead_zone = 1.0;
    x_controller.min = deg2rad(-10);
    x_controller.max = deg2rad(10);
    x_controller.k_p = 1.0;
    x_controller.k_i = 0.0;
    x_controller.k_d = 0.0;

    // y controller
    y_controller.setpoint = 0.0;
    y_controller.dead_zone = 1.0;
    y_controller.min = deg2rad(-10);
    y_controller.max = deg2rad(10);
    y_controller.k_p = 1.0;
    y_controller.k_i = 0.0;
    y_controller.k_d = 0.0;

    float roll_input;
    float pitch_input;

    while (ros::ok()){
        // pose.header.stamp = ros::Time::now();
        // pose.header.seq = count;
        // pose.header.frame_id = "awesomo_quad_offboard";
        //
        // // hover
        // pose.pose.position.x = 0.0;
        // pose.pose.position.y = 0.0;
        // pose.pose.position.z = 1.2;

        // quad.runMission(pose);
        // quad.runMission2(pose);

		// publish
		// quad.position_publisher.publish(pose);


        x_controller.setpoint = 0.0;
        y_controller.setpoint = 0.0;

        dt = ros::Time::now() - last_request;
        pid_calculate(&x_controller, quad.pose_x, dt);
        pid_calculate(&y_controller, quad.pose_y, dt);
        roll_input = x_controller.output;
        pitch_input = y_controller.output;
        last_request = ros::Time::now();

        q = euler2quat(roll_input, pitch_input, 0);

        attitude.header.stamp = ros::Time::now();
        attitude.header.seq = count;
        attitude.header.frame_id = "awesomo_quad_offboard_attitude_cmd";
        attitude.pose.position.x = 0.0;
        attitude.pose.position.y = 0.0;
        attitude.pose.position.z = 0.0;
        attitude.pose.orientation.x = q.x();
        attitude.pose.orientation.y = q.y();
        attitude.pose.orientation.z = q.z();
        attitude.pose.orientation.w = q.w();

        throttle.data = 0.5;

        quad.attitude_publisher.publish(attitude);
        quad.throttle_publisher.publish(throttle);

        ROS_INFO("---");
        ROS_INFO("dt %f", dt.toSec());
        ROS_INFO("quadrotor.pose_x %f", quad.pose_x);
        ROS_INFO("quadrotor.pose_y %f", quad.pose_y);
        ROS_INFO("quadrotor.roll %f", rad2deg(quad.roll));
        ROS_INFO("quadrotor.pitch %f", rad2deg(quad.pitch));
        ROS_INFO("roll.controller %f", roll_input);
        ROS_INFO("pitch.controller %f", pitch_input);
        ROS_INFO("---");

		// update
		count++;

		// end
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}

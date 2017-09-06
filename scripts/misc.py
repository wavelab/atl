    quad.set_arm(True)
    quad.set_mode("DISCOVER_MODE")
    lz.set_velocity(1.0)
    quad.set_yaw(90)
    quad.set_mode("WAYPOINT_MODE")

    # quad.set_pctrl_settings({
    #     "roll": {
    #         "min": -30,
    #         "max": 30,
    #         "k_p": 0.08,
    #         "k_i": 0.0001,
    #         "k_d": 0.04
    #     },
    #     "pitch": {
    #         "min": -30,
    #         "max": 30,
    #         "k_p": 0.08,
    #         "k_i": 0.0001,
    #     },
    #     "throttle": {
    #         "hover": 0.45,
    #         "k_p": 0.1,
    #         "k_i": 0.0,
    #         "k_d": 0.05
    #     }
    # })
    #
    # up_and_down()
    # hover_side_to_side()

    # lz.set_position([1, 0, 0])
    # quad.set_hover_point([1, 1, 5.0])

    # quad.set_hover_point([0.0, 0.0, 10.0])

    # quad.set_arm(True)
    # quad.set_mode("HOVER")
    # quad.set_hover_point([0.0, 0.0, 5.0])
    # quad.set_yaw(0)

    # square(quad, 3, 1)

    # radius = 10.0
    # velocity = 1.0

    # velocity, angular_velocity = lz_circle_path(radius, velocity)
    # lz.set_velocity(1.0)
    # lz.set_angular_velocity(angular_velocity)

    # sleep(10)
    #
    # lz.set_velocity(velocity)
    # lz.set_angular_velocity(-1.0 * angular_velocity)

    # model_name = "camera"
    # pos = [0, 0, 2]
    # rpy = [0, 0, 0]
    #
    # world.set_model_pose(model_name, pos, rpy)
    # sleep(1)
    #
    # pos = [1, 1, 2]
    # world.set_model_pose(model_name, pos, rpy)
    # sleep(1)
    #
    # pos = [-1, 1, 2]
    # world.set_model_pose(model_name, pos, rpy)
    # sleep(1)
    #
    # pos = [-1, -1, 2]
    # world.set_model_pose(model_name, pos, rpy)
    # sleep(1)
    #
    # pos = [1, -1, 2]
    # world.set_model_pose(model_name, pos, rpy)
    # sleep(1)

    # sleep(1)
    # lz_straight_line(0.5)
    # sleep(1)
    # lz_straight_line(1.0)
    # sleep(1)
    # lz_straight_line(2.0)
    # sleep(1)
    # lz_straight_line(2.5)
    # sleep(1)
    # lz_straight_line(3.0)
    # sleep(1)
    # lz_straight_line(3.5)
    # sleep(1)
    # lz_straight_line(4.0)
    # sleep(1)
    # lz_straight_line(4.5)
    # sleep(1)
    # lz_straight_line(5.0)
    # sleep(1)
    # lz_straight_line(6.0)
    # sleep(1)
    # lz_straight_line(7.0)
    # sleep(1)
    # lz_straight_line(8.0)
    # sleep(1)
    # lz_straight_line(9.0)
    # sleep(1)
    # lz_straight_line(10.0)
    # lz.set_position([0, 0, 0])

    # side_to_side(quad, 4.0, 2.0)
    # square(quad, 4.0, 2.0)

    # quad.set_tctrl_settings({
    #     "roll": {
    #         "min": -30,
    #         "max": 30,
    #         "k_p": 0.8,
    #         "k_i": 0.00001,
    #         "k_d": 0.2
    #     },
    #     "pitch": {
    #         "min": -30,
    #         "max": 30,
    #         "k_p": 0.1,
    #         "k_i": 0.0,
    #         "k_d": 0.05
    #     },
    #     "throttle": {
    #         "hover": 0.5,
    #         "k_p": 0.4,
    #         "k_i": 0.0,
    #         "k_d": 0.2
    #     },
    #     "vx": {
    #         "k_p": 0.2,
    #         "k_i": 0.0,
    #         "k_d": 0.0
    #     },
    #     "vy": {
    #         "k_p": 0.5,
    #         "k_i": 0.0,
    #         "k_d": 0.0
    #     },
    #     "vz": {
    #         "k_p": 0.2,
    #         "k_i": 0.0,
    #         "k_d": 0.0
    #     }
    # })

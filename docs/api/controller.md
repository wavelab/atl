# controller.hpp

- Structures
- Classes



## Structures

    struct pid
    {
        int sample_rate;

        float setpoint;
        float output;

        float prev_error;
        float sum_error;

        float p_error;
        float i_error;
        float d_error;

        float k_p;
        float k_i;
        float k_d;

        float dead_zone;
        float min;
        float max;
    };



## Classes

- CarrotController
- PositionController


### CarrotController

#### Attributes

- `std::deque<Eigen::Vector3d> waypoints`
- `int initialized`
- `double look_ahead_dist`
- `double wp_threshold`
- `Eigen::Vector3d wp_start`
- `Eigen::Vector3d wp_end`


#### Constructors

    CarrotController();
    CarrotController(
        std::deque<Eigen::Vector3d> waypoints,
        double look_ahead_dist,
        double wp_threshold
    );
    CarrotController(std::string config_file_path);


#### Methods

    Eigen::Vector3d closestPoint(
        Eigen::Vector3d position,
        Eigen::Vector3d wp_start,
        Eigen::Vector3d wp_end
    );

Calculates the closest point on the trajectory line formed by `wp_start` and
`wp_end` relative to `position`.

---

    Eigen::Vector3d calculateCarrotPoint(
        Eigen::Vector3d position,
        double r,
        Eigen::Vector3d wp_start,
        Eigen::Vector3d wp_end
    );

Calculate new carrot point based on `position`, look ahead distance `r` and
trajectory line formed by `wp_start` and `wp_end`.

---

    int waypointReached(
        Eigen::Vector3d position,
        Eigen::Vector3d waypoint,
        double threshold
    );

Returns `0` or `1` to denote whether waypoint has been reached.

---

    int update(Eigen::Vector3d position, Eigen::Vector3d &carrot);

This method is meant to be executed within a loop that constantly updates the
robot's `position`, where `carrot` is the carrot point to be updated. Returns
`0` or `-1` for success or failure.

---


### PositionController

#### Attribute

- `struct pid x`
- `struct pid y`
- `struct pid T`
- `float roll`
- `float pitch`
- `float throttle`
- `float hover_throttle`
- `tf::Quaternion rpy_quat`
- `float dt`


#### Constructor

    PositionController(const std::string config_file);


#### Methods

    void loadConfig(const std::string config_file);

Load `config_file` where it defines the position controller configurations.

---

    void calculate(Pose p);

Calculates new `PositionController::roll`, `PositionController::pitch` and
`PositionController::throttle` values for the inner loop (atitude controller),
based on the robot's current pose `p`.

---

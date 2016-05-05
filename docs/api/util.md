# util.hpp

**Content**:
- Classes
- Functions



## Classes

- Pose
- Position



### Pose

#### Attributes

- `double x`
- `double y`
- `double z`
- `double roll`
- `double pitch`
- `double yaw`



### Position

#### Attributes

- `double x;`
- `double y;`
- `double z;`



## Functions

    double deg2rad(double d);

Convert degrees `d` to radians.

---

    double rad2deg(double r);

Convert radians `r` to degrees.

---

    int euler2Quaternion(
        const double roll,
        const double pitch,
        const double yaw,
        Eigen::Quaterniond &q
    );

Convert Euler angles in radians to quaternions `q`. Returns `0` or `-1` to
denote success or failure.

---

    int euler2RotationMatrix(
        const double roll,
        const double pitch,
        const double yaw,
        Eigen::Matrix3d &rot
    );

Convert Euler angles in radians to rotation matrix `rot`. Returns `0` or `-1` to
denote success or failure.

---

    tf::Quaternion euler2quat(double roll, double pitch, double yaw);

Convert Euler angles in radians to quaternions, the difference to
`euler2Quaternion` is this function returns a `tf::Quaternion` instead of
a `Eigen::Quaterniond`.

---

    void quat2euler(
        const geometry_msgs::Quaternion &q,
        double *roll,
        double *pitch,
        double *yaw
    );

Convert quaternion to Euler angles in radians.

---

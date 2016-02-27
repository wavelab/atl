# camera.hpp


## Camera::Camera

**Attributes**:

- `int camera_index;`
- `int image_width;`
- `int image_height;`
- `vector<AprilTags::TagDetection> apriltags;`

**Constructor**:

- `Camera(void);`

**Methods**:

- `int run(void);`


### Methods

#### Camera::run(void)

Run Camera and attempt to detect any Apriltags. The AprilTags detected will be
recorded in `Camera::apriltags`.

---

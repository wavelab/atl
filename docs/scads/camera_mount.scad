module mounting_holes()
{
    $fn = 35;
    x1 = 0.0;
    x2 = 14.0;
    y1 = 0.0;
    y2 = 30.0;

    depth = 4.0;
    diam = 2.72;

    translate([x1, y1, 0])
    cylinder(h = depth, r = diam / 2);

    translate([x1, y2, 0])
    cylinder(h = depth, r = diam / 2);

    translate([x2, y2, 0])
    cylinder(h = depth, r = diam / 2);

    translate([x2, y1, 0])
    cylinder(h = depth, r = diam / 2);
}

module central_hole_cutout(diam, thickness)
{
    $fn = 20;
    cylinder(r = diam /2, h = thickness);
}

module mounting_surface(diam, thickness)
{
    $fn = 50;
    cylinder(r = diam / 2, h = thickness);
}

module camera_mounting_plate(
    plate_diam,
    thickness,
    hole_location_radius,
    hole_diam,
    num_holes,
    central_hole_diam,
    arm_width,
    arm_length
)
{
    ximiea_cam_h = 35;
    r = plate_diam / 2;
    sagitta = r - sqrt(pow(r, 2) - pow(arm_width /2, 2));
    difference()  {
        union() {
            mounting_surface(plate_diam, thickness);
            translate([ximiea_cam_h / 2, -arm_width/2, 0])
            rotate([0, -90, 0])
            cube([44.0 + thickness, 22, thickness]);
        }
        central_hole_cutout(central_hole_diam, thickness);
    }
}

//pitch_motor mount
module pitch_motor_mount()
{
    front_plate_diam = 32;
    thickness = 2.5;
    hole_diam = 1.2;
    num_holes = 3.0;
    arm_width = 22.0;
    pitch_motor_mount_holes_radius = 3;
    pitch_motor_mount_central_hole = 2.5;
    pitch_motor_arm_length = 38.0;

    camera_mounting_plate(
        front_plate_diam,
        thickness,
        pitch_motor_mount_holes_radius,
        hole_diam,
        num_holes,
        pitch_motor_mount_central_hole,
        arm_width,
        pitch_motor_arm_length
    );
}

// run
pitch_motor_mount();

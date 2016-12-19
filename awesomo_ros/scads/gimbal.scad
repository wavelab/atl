module ximea_camera()
{
    // body
    cube([22, 27, 27], center = true);

    // lens
    translate([11, 0, 0]) {
        rotate([90, 0, 90]) {
            cylinder(17, 14.5, 14.5);
        }
    }
};

module gimbal_motor()
{
    inner_length = 10.0;
    taper_length = 2.5;
    diameter = 34.0;

    // main body
    cylinder(inner_length, diameter / 2.0, diameter / 2.0, center = true);

    // top taper
    translate([0, 0, inner_length / 2.0]) {
        cylinder(taper_length, diameter / 2.0, diameter / 2.5);
    }

    // bottom taper
    translate([0, 0, -inner_length / 2.0]) {
        rotate([180, 0, 0]) {
            cylinder(taper_length, diameter / 2.0, diameter / 2.5);
        }
    }
};

module gimbal_bar(width, depth, height, thickness)
{
    // bottom plate
    cube([depth, width, thickness], center = true);

    // right vertical plate
    translate([-depth / 2.0, -width / 2.0]) {
        cube([depth, thickness, height]);
    }

    // left vertical plate
    translate([-depth / 2.0, width / 2.0 - thickness]) {
        cube([depth, thickness, height]);
    }
};


// run
translate([0, 0, 18]) {
    color([1, 0, 0]) {
        ximea_camera();
    }
};

// pitch motor
translate([0, 57, 18]) {
    rotate([90, 0, 0]) {
        color([0, 1, 0]) {
            gimbal_motor();
        }
    }
}

// roll motor
translate([-45, 0, 20]) {
    rotate([0, 90, 0]) {
        color([0, 1, 0]) {
            gimbal_motor();
        }
    }
}

// pitch bar
translate([0, 0, 0]) {
    gimbal_bar(100, 50, 40, 5);
}

// roll bar
translate([-35, 6, 20]) {
    rotate([0, 90, 0]) {
        gimbal_bar(130, 40, 60, 5);
    }
}

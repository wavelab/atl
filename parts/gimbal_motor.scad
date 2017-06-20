// gimbal motors

module main_body()
{
    $fn = 80;
    inner_length = 14.8;
    inner_diameter = 33.25;

    outer_diameter = 34.6;
    outer_ring_length = 5.5;

    center_shaft_diam = 9.5;
    center_shaft_length = 6.0;

    hole_length = inner_length + center_shaft_length + 2;
    hole_diameter = 2.3;

    mount_hole_diam = 1.35;
    mount_hole_offset = 5.85;
    mount_holes = 3;
    mount_hole_depth = 2.5;

    top_mount_hole_offset = 13.0;
// main body

    difference()  {
        union() {
            color([0, 0, 0])
            cylinder(inner_length, inner_diameter / 2.0, inner_diameter / 2.0);
        // top ring
            color([1, 0, 0])
            translate([0, 0, inner_length - outer_ring_length])
            cylinder(outer_ring_length, outer_diameter   / 2.0, outer_diameter / 2.0);

        // center shaft
            color([0.8, 0.8, 0.8])
            translate([0, 0, inner_length ])
            cylinder(center_shaft_length, center_shaft_diam  / 2.0,center_shaft_diam / 2.0);

        // rounded thing shaft
            color([0.8, 0, 0])
            translate([0, 0, inner_length])
            cylinder(1.0, 13.0 / 2.0, 14.0 / 2.0, center = true);
        }

        // central hole
        translate([0, 0, -1.0])
        cylinder(hole_length, hole_diameter / 2.0, hole_diameter / 2.0);

        // mount holes
        for(i = [1 : mount_holes]) {
            translate([mount_hole_offset * cos(i * (360 / mount_holes)),
                mount_hole_offset * sin(i * (360 / mount_holes)),
                0])
                cylinder(r = mount_hole_diam / 2.0, h = mount_hole_depth);
        }

        // top mount holes
        for(i = [1 : mount_holes]) {
            translate([top_mount_hole_offset * cos(i * (360 / mount_holes)),
                top_mount_hole_offset * sin(i * (360 / mount_holes)),
                inner_length - mount_hole_depth + .5])
                cylinder(r = mount_hole_diam / 2.0, h = mount_hole_depth );
        }
   }
};

main_body();


module camera_body()
{
    union() {
        color([0.35, 0.35, 0.35])
        cube([22.1, 44.0, 35.0]);

        // lens mount
        color([0.8, 0.8, 0.8])
        translate([22.1, 44.0 / 2, 35.0 / 2])
        rotate([0, 90, 0])
        cylinder(r = 15.0, h = 6.2);
    }
}

module lens(width, length)
{
    $fn = 50;
    // fake lens
    color([0.5, 0.5, 0.5])
    cylinder(r = width / 2, h = length);
}

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

module pg_cameleon3()
{
    difference() {
        union() {
            camera_body();
            translate([22.1 + 6.2, 44.0 / 2, 35.0 / 2])
            rotate([0, 90, 0])
            lens(35, 20);
        }

        translate([3.1, 7.0, 0.0])
        mounting_holes();

        translate([3.1, 7.0, 35.0 - 4.0])
        mounting_holes();
    }
}

pg_cameleon3();


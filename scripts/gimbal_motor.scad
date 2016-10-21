// gimbal motors

module main_body()
{
    inner_length = 9.5;   
    inner_diameter = 33.25;
    
    outer_diameter = 34.6;
    outer_ring_length = 5.5;
    
    center_shaft_diam = 10.5;
    center_shaft_length = 6.0;
    
    // main body
    union(){
        cylinder(inner_length, inner_diameter / 2.0, inner_diameter / 2.0,              center = true);
    // top ring
        translate([0, 0, inner_length - outer_ring_length])
        cylinder(outer_ring_length, outer_diameter   / 2.0, outer_diameter / 2.0            , center = true);

    // center shaft
        translate([0, 0, (inner_length + outer_ring_length) - center_shaft_length])
        cylinder(center_shaft_length, center_shaft_diam  / 2.0,                   center_shaft_diam / 2.0, center = true);
    
    // rounded thing shaft
        translate([0, 0, (inner_length - outer_ring_length) + 1.0])
        cylinder(1.0, 14.0 / 2.0, 14.0 / 2.0, center = true);
    }
    
    
};


// run
translate([0, 57, 18]) {
    rotate([90, 0, 0]) {
        color([0, 1, 0]) {
            main_body();
        }
    }
}

module mounting_holes(num_holes, hole_spacing, hole_diam, thickness){
    for(i = [1 : num_holes]){
        translate([
                hole_spacing * cos(i * (360 / num_holes)),
                hole_spacing * sin(i * (360 / num_holes)),
                0
        ])
       cylinder(r = hole_diam / 2.0, h = thickness);
   }
};

module mounting_plate(num_holes, hole_diam, outter_diam, hole_spacing, thickness, length)
{
    $fn = 60;
    hole_dist = 2 * sin(60) * hole_spacing;
    difference(){
       hull(){
            for(i = [1 : num_holes]){
                rotate([0, 0, 60])
                translate([
                    hole_spacing * cos(i * (360 / num_holes)),
                    hole_spacing * sin(i * (360 / num_holes)),
                    0
                ])
                cylinder(r = outter_diam / 2, h = thickness);
            }
            translate([length, -(outter_diam + hole_dist) / 2, 0])
            cube([2.0, (outter_diam + hole_dist), thickness]);
        }

        rotate([0, 0, 60])
        mounting_holes(num_holes, hole_spacing, hole_diam, thickness);
    }
};


// run
union()
{
    mounting_plate(3, 1.6, 1.6 * 3, 5.5, 2.5, 30);
    translate([30, 0, 32])
    rotate([0, 90, 0])
    mounting_plate(3, 1.6, 1.6 * 4, 13.5, 2.5, 30);
}

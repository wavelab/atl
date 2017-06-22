include <common.scad>;

// plate parameters
plate_width = 130.0;
plate_depth = 3.0;


module CableRoutingHoles(plate_width, plate_depth) {
    routing_width = plate_width - 45.0;
    hole_width = 20;
    hole_length = 5;

    // inner cable fastening holes
    translate([routing_width / 2.0, 0.0, plate_depth / 2.0]) {
        cube(size=[hole_length, hole_width, plate_depth], center=true);
        translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }
}

module CableFasteningHoles(plate_width, plate_depth) {
    inner_width = plate_width - 18.0;
    outer_width = plate_width - 10.0;
    hole_width = 40;
    hole_length = 1.5;

    // inner cable fastening holes
    translate([inner_width / 2.0, 0.0, plate_depth / 2.0]) {
        cube(size=[hole_length, hole_width, plate_depth], center=true);
        translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }

    translate([-inner_width / 2.0, 0.0, plate_depth / 2.0]) {
        cube(size=[hole_length, hole_width, plate_depth], center=true);
        translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }

    translate([0.0, inner_width / 2.0, plate_depth / 2.0]) {
        cube(size=[hole_width, hole_length, plate_depth], center=true);
        translate([hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([-hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }

    translate([0.0, -inner_width / 2.0, plate_depth / 2.0]) {
        cube(size=[hole_width, hole_length, plate_depth], center=true);
        translate([hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([-hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }

    // outer cable fastening holes
    translate([outer_width / 2.0, 0.0, plate_depth / 2.0]) {
        cube(size=[hole_length, hole_width, plate_depth], center=true);
        translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }

    translate([-outer_width / 2.0, 0.0, plate_depth / 2.0]) {
        cube(size=[hole_length, hole_width, plate_depth], center=true);
        translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }

    translate([0.0, outer_width / 2.0, plate_depth / 2.0]) {
        cube(size=[hole_width, hole_length, plate_depth], center=true);
        translate([hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([-hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }

    translate([0.0, -outer_width / 2.0, plate_depth / 2.0]) {
        cube(size=[hole_width, hole_length, plate_depth], center=true);
        translate([hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([-hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }
}

module ESCMountHoles(plate_depth) {
    esc_width = 56;
    esc_length = 65;
    hole_width = 30;
    hole_length = 3;

    translate([esc_width / 2.0, 0.0, plate_depth / 2.0]) {
        cube(size=[hole_length, hole_width, plate_depth], center=true);
        translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }

    translate([-esc_width / 2.0, 0.0, plate_depth / 2.0]) {
        cube(size=[hole_length, hole_width, plate_depth], center=true);
        translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }

    translate([0.0, esc_length / 2.0, plate_depth / 2.0]) {
        cube(size=[hole_width, hole_length, plate_depth], center=true);
        translate([hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([-hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }

    translate([0.0, -esc_length / 2.0, plate_depth / 2.0]) {
        cube(size=[hole_width, hole_length, plate_depth], center=true);
        translate([hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
        translate([-hole_width / 2.0, 0.0, -plate_depth / 2.0]) {
            cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
        }
    }
}

module ArmScrewHoles(plate_width, plate_depth, plate_diagonal) {
    base_line = 23.0;
    top_line = 16.0;
    between_line = 20.0;

    E = 5.5;
    F = (plate_diagonal / 2.0) - E;
    G = F - between_line;

    // outer screw holes
    M25Hole(F, -base_line / 2.0, plate_depth / 2.0, plate_depth);
    M25Hole(F, base_line / 2.0, plate_depth / 2.0, plate_depth);

    M25Hole(-F, -base_line / 2.0, plate_depth / 2.0, plate_depth);
    M25Hole(-F, base_line / 2.0, plate_depth / 2.0, plate_depth);

    M25Hole(-base_line / 2.0, F, plate_depth / 2.0, plate_depth);
    M25Hole(base_line / 2.0, F, plate_depth / 2.0, plate_depth);

    M25Hole(-base_line / 2.0, -F, plate_depth / 2.0, plate_depth);
    M25Hole(base_line / 2.0, -F, plate_depth / 2.0, plate_depth);

    // inner screw holes
    M25Hole(G, -top_line / 2, plate_depth / 2.0, plate_depth);
    M25Hole(G, top_line / 2, plate_depth / 2.0, plate_depth);

    M25Hole(-G, -top_line / 2, plate_depth / 2.0, plate_depth);
    M25Hole(-G, top_line / 2, plate_depth / 2.0, plate_depth);

    M25Hole(-top_line / 2, -G, plate_depth / 2.0, plate_depth);
    M25Hole(top_line / 2, -G, plate_depth / 2.0, plate_depth);

    M25Hole(-top_line / 2, G, plate_depth / 2.0, plate_depth);
    M25Hole(top_line / 2, G, plate_depth / 2.0, plate_depth);
}

module TopPlate(plate_width, plate_depth) {
    difference() {
        // plate
        rotate(45) {
            translate([-plate_width / 2.0, -plate_width / 2.0, 0.0]) {
                cube([plate_width, plate_width, plate_depth]);
            }
        }

        // square cut plate corners
        A = plate_width;
        B = sqrt(2 * pow(A, 2));
        C = 30.0;
        D = 30.0;

        translate([B / 2.0, 0.0, plate_depth / 2.0]) { cube([C, D, plate_depth], true); }
        translate([-B / 2.0, 0.0, plate_depth / 2.0]) { cube([C, D, plate_depth], true); }
        translate([0.0, -B / 2.0, plate_depth / 2.0]) { cube([D, C, plate_depth], true); }
        translate([0.0, B/ 2.0, plate_depth / 2.0]) { cube([D, C, plate_depth], true); }

        // holes
        rotate(45) {
            // CableRoutingHoles(plate_width, plate_depth);
            CableFasteningHoles(plate_width, plate_depth);
            ESCMountHoles(plate_depth);
        }
        ArmScrewHoles(plate_width, plate_depth, B - C);
        StackHoles(plate_width, plate_depth, B - C);
    }
}

TopPlate(plate_width, plate_depth);

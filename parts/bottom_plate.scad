// $fn = 64;

// plate parameters
plate_width = 110.0;
plate_depth = 3.0;

module M25ScrewHole(x, y, z, depth) {
    translate([x, y, z]) {
        cylinder(h = depth, d = 2.5, center = true);
    }
}

module M6Hole(x, y, z, depth) {
    translate([x, y, z]) {
        cylinder(h = depth, d = 7.0, center = true);
    }
}

module BottomPlateScrewHoles(plate_width, plate_depth, plate_diagonal) {
    base_line = 21.0;
    between_line = 20.0;

    E = 5.5;
    F = (plate_diagonal / 2.0) - E;
    G = F - between_line;

    // outer screw holes
    M25ScrewHole(F, -base_line / 2.0, plate_depth / 2.0, plate_depth);
    M25ScrewHole(F, base_line / 2.0, plate_depth / 2.0, plate_depth);

    M25ScrewHole(-F, -base_line / 2.0, plate_depth / 2.0, plate_depth);
    M25ScrewHole(-F, base_line / 2.0, plate_depth / 2.0, plate_depth);

    M25ScrewHole(-base_line / 2.0, F, plate_depth / 2.0, plate_depth);
    M25ScrewHole(base_line / 2.0, F, plate_depth / 2.0, plate_depth);

    M25ScrewHole(-base_line / 2.0, -F, plate_depth / 2.0, plate_depth);
    M25ScrewHole(base_line / 2.0, -F, plate_depth / 2.0, plate_depth);
}

module StackHoles(plate_width, plate_depth, plate_diagonal) {
    between_line = 20.0;

    E = 5.5;
    F = (plate_diagonal / 2.0) - E;
    H = F - (between_line / 2.0);

    M6Hole(H, 0.0, plate_depth / 2.0, plate_depth);
    M6Hole(-H, 0.0, plate_depth / 2.0, plate_depth);
    M6Hole(0.0, H, plate_depth / 2.0, plate_depth);
    M6Hole(0.0, -H, plate_depth / 2.0, plate_depth);
}

module BatteryStrapHoles(plate_width, plate_depth, plate_diagonal) {
    hole_width = 5;
    hole_length = 25;
    hole_between_width = 45;

    rotate(45) {
        translate([hole_between_width / 2.0, 0.0, plate_depth / 2.0]) {
            cube([hole_width, hole_length, plate_depth], center = true);
        }
    }

    rotate(45) {
        translate([-hole_between_width / 2.0, 0.0, plate_depth / 2.0]) {
            cube([hole_width, hole_length, plate_depth], center = true);
        }
    }
}

module BatteryTray(plate_width, plate_depth, battery_width, battery_length) {
    stay_length = battery_length - plate_width;

    translate([(plate_width / 2.0), 0.0, 0.0]) {
        cube([stay_length, battery_width, plate_depth]);
    }
}

module BatteryStop(plate_width, plate_depth, battery_width, battery_length) {
    stay_length = battery_length - plate_width;
	fillet_radius = 2.0;
	stop_thickness = 2;
	stop_height = 10;

    translate([(plate_width / 2.0), 0.0, 0.0]) {
		// fillet
        rotate([90, 0, 0]) {
			translate([0.0, plate_depth, -battery_width / 2.0]) {
				difference() {
					cylinder(r=fillet_radius, h=battery_width, center=true);
					translate([-fillet_radius, fillet_radius, 0.0]) {
						cylinder(r=fillet_radius, h=battery_width, center=true);
					}
				}
			}
		}

		// stop
		cube([stop_thickness, battery_width, stop_height]);
    }
}

module BottomPlate(plate_width, plate_depth) {
    battery_length = 146;
    battery_width = 40;

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
        BottomPlateScrewHoles(plate_width, plate_depth, B - C);
        StackHoles(plate_width, plate_depth, B - C);
        BatteryStrapHoles(plate_width, plate_depth, B - C);
    }

    // battery tray
    rotate(135) {
        translate([0, -20, 0]) {
            BatteryTray(plate_width, plate_depth, battery_width, battery_length);
        }
    }

    // battery stop
    rotate(-45) {
        translate([0, -20, 0]) {
            BatteryStop(plate_width, plate_depth, battery_width, battery_length);
        }
    }

}

BottomPlate(plate_width, plate_depth);

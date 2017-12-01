use <common.scad>

// plate parameters
plate_width = 145.0;
plate_depth = 3.0;

battery_length = 146;
battery_width = 40;

module BatteryPlate(plate_width, plate_depth) {
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

    translate([B / 2.0, 0.0, plate_depth / 2.0]) {
      cube([C, D, plate_depth], true);
    }
    translate([-B / 2.0, 0.0, plate_depth / 2.0]) {
      cube([C, D, plate_depth], true);
    }
    translate([0.0, -B / 2.0, plate_depth / 2.0]) {
      cube([D, C, plate_depth], true);
    }
    translate([0.0, B/ 2.0, plate_depth / 2.0]) {
      cube([D, C, plate_depth], true);
    }

    // center holes
    translate([plate_width * 0.3, 0.0, plate_depth / 2.0]) {
      cylinder(r=6, plate_depth, center=true);
    }
    translate([0.0, plate_width * 0.3, plate_depth / 2.0]) {
      cylinder(r=6, plate_depth, center=true);
    }
    translate([-plate_width * 0.3, 0.0, plate_depth / 2.0]) {
      cylinder(r=6, plate_depth, center=true);
    }
    translate([0.0, -plate_width * 0.3, plate_depth / 2.0]) {
      cylinder(r=6, plate_depth, center=true);
    }


    // holes
    rotate(-135) {
      CableRoutingHoles(plate_width, plate_depth);
      CableFasteningHoles(plate_width, plate_depth);
    }
    StackHoles(plate_width, plate_depth, B - C);
    BatteryStrapHoles(plate_width, plate_depth);
  }
}

BatteryPlate(plate_width, plate_depth);

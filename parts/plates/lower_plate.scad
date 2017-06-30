use <common.scad>

// plate parameters
plate_width = 145.0;
plate_depth = 3.0;

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

module LowerPlate(plate_width, plate_depth) {
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

    // holes
    rotate(-135) {
      CableRoutingHoles(plate_width, plate_depth);
      CableFasteningHoles(plate_width, plate_depth);
    }
    LowerArmScrewHoles(plate_width, plate_depth, B - C);
    StackHoles(plate_width, plate_depth, B - C);

    translate([15, -15, 0])
      #OdroidXU4MountHoles(plate_depth);
      // rotate(180)
  }
}

LowerPlate(plate_width, plate_depth);

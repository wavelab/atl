module M25Hole(x, y, z, depth) {
    translate([x, y, z]) {
        cylinder(h = depth, d = 2.5, center = true);
    }
}

module M6Hole(x, y, z, depth) {
    translate([x, y, z]) {
        cylinder(h = depth, d = 7.0, center = true);
    }
}

module Plate(plate_width, plate_depth) {
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
    ArmScrewHoles(plate_width, plate_depth, B - C);
    StackHoles(plate_width, plate_depth, B - C);
  }
}

module OdroidXU4MountHoles(plate_depth) {
    depth = 76;
    width = 52;

    rotate(45) {
        M25Hole(depth / 2.0, width / 2.0, 0.0, plate_depth * 2);
        M25Hole(-depth / 2.0, width / 2.0, 0.0, plate_depth * 2);
        M25Hole(depth / 2.0, -width / 2.0, 0.0, plate_depth * 2);
        M25Hole(-depth / 2.0, -width / 2.0, 0.0, plate_depth * 2);
    }
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

module CableFasteningHoles(plate_width, plate_depth) {
  inner_width = plate_width - 10.0;
  hole_width = 60;
  hole_length = 2;

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

  rotate(90) translate([inner_width / 2.0, 0.0, plate_depth / 2.0]) {
    cube(size=[hole_length, hole_width, plate_depth], center=true);
    translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
      cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
    }
    translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
      cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
    }
  }
}

module CableRoutingHoles(plate_width, plate_depth) {
  routing_width = plate_width - 35.0;
  hole_width = 35;
  hole_length = 7;

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

  rotate(90) translate([routing_width / 2.0, 0.0, plate_depth / 2.0]) {
    cube(size=[hole_length, hole_width, plate_depth], center=true);
    translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
      cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
    }
    translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
      cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
    }
  }

  rotate(180) translate([routing_width / 2.0, 0.0, plate_depth / 2.0]) {
    cube(size=[hole_length, hole_width, plate_depth], center=true);
    translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
      cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
    }
    translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
      cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
    }
  }
}

module BatteryStrapHoles(plate_width, plate_depth) {
  hole_width = 5;
  hole_length = 25;
  hole_between_width = 45;

  rotate(45) {
    translate([hole_between_width / 2.0, 0.0, plate_depth / 2.0]) {
      cube([hole_width, hole_length, plate_depth], center = true);
      translate([0.0, hole_length / 2.0, -plate_depth / 2.0]) {
        cylinder(r=hole_width / 2.0, h=plate_depth, center=false);
      }
      translate([0.0, -hole_length / 2.0, -plate_depth / 2.0]) {
        cylinder(r=hole_width / 2.0, h=plate_depth, center=false);
      }
    }
  }

  rotate(45) {
    translate([-hole_between_width / 2.0, 0.0, plate_depth / 2.0]) {
      cube([hole_width, hole_length, plate_depth], center = true);
      translate([0.0, hole_length / 2.0, -plate_depth / 2.0]) {
        cylinder(r=hole_width / 2.0, h=plate_depth, center=false);
      }
      translate([0.0, -hole_length / 2.0, -plate_depth / 2.0]) {
        cylinder(r=hole_width / 2.0, h=plate_depth, center=false);
      }
    }
  }
}

module LowerArmScrewHoles(plate_width, plate_depth, plate_diagonal) {
  base_line = 21.0;
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
}

use <common.scad>

// plate parameters
plate_width = 130.0;
plate_depth = 3.0;


module CableRoutingHoles(plate_width, plate_depth) {
  routing_width = plate_width - 45.0;
  hole_width = 10;
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

module OuterCableFasteningHoles(plate_width, plate_depth) {
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
}

module ArmScrewHoles(plate_width, plate_depth, plate_diagonal) {
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

module CameraExtensionHoles(plate_width, plate_depth) {
  extension_width = 32;

  M6Hole(extension_width, 0.0, plate_depth / 2.0, plate_depth);
  M6Hole(-extension_width, 0.0, plate_depth / 2.0, plate_depth);
  M6Hole(0.0, extension_width, plate_depth / 2.0, plate_depth);
  M6Hole(0.0, -extension_width, plate_depth / 2.0, plate_depth);
}

module CableFasteningHoles(plate_width, plate_depth) {
  edge_padding = 65.0;
  hole_width = 5.0;
  hole_length = 1.5;

  translate([2.0, -(plate_width - edge_padding) / 2.0, plate_depth / 2.0]) {
    cube(size=[hole_length, hole_width, plate_depth], center=true);
    translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
      cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
    }
    translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
      cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
    }
  }

  translate([-2.0, -(plate_width - edge_padding) / 2.0, plate_depth / 2.0]) {
    cube(size=[hole_length, hole_width, plate_depth], center=true);
    translate([0.0, hole_width / 2.0, -plate_depth / 2.0]) {
      cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
    }
    translate([0.0, -hole_width / 2.0, -plate_depth / 2.0]) {
      cylinder(r=hole_length / 2.0, h=plate_depth, center=false);
    }
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

module CameraMountHoles(plate_depth) {
  mount_width = 27.5;

  translate([-mount_width / 2.0, -mount_width / 2.0, plate_depth / 2.0]) {
    cylinder(r=1.0, h=plate_depth, center=true);
  }

  translate([mount_width / 2.0, -mount_width / 2.0, plate_depth / 2.0]) {
    cylinder(r=1.0, h=plate_depth, center=true);
  }

  translate([-mount_width / 2.0, mount_width / 2.0, plate_depth / 2.0]) {
    cylinder(r=1.0, h=plate_depth, center=true);
  }

  translate([mount_width / 2.0, mount_width / 2.0, plate_depth / 2.0]) {
    cylinder(r=1.0, h=plate_depth, center=true);
  }
}

module BatteryStop(plate_width, plate_depth, battery_width, battery_length) {
  stay_length = battery_length - plate_width;
  fillet_radius = 2.0;
  stop_thickness = 2;
  stop_height = 42;
  stop_width = 50;

    // stop
    translate([(plate_width / 2.0) + (stop_thickness / 2.0), 0, stop_height / 2.0]) {
      cube([stop_thickness, battery_width, stop_height], center=true);
    }

    // fillet
    rotate([90, 0, 0]) {
      translate([plate_width / 2.0, plate_depth, 0.0]) {
        difference() {
          cylinder(r=fillet_radius, h=battery_width, center=true);
          translate([-fillet_radius, fillet_radius, 0.0]) {
            cylinder(r=fillet_radius, h=battery_width, center=true);
          }
        }
      }
    }
}

module Plate(plate_width, plate_depth) {
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
      OuterCableFasteningHoles(plate_width, plate_depth);
      rotate(90) CableFasteningHoles(plate_width, plate_depth);
    }
    ArmScrewHoles(plate_width, plate_depth, B - C);
    BatteryStrapHoles(plate_width, plate_depth, B - C);
    StackHoles(plate_width, plate_depth, B - C);
    CameraExtensionHoles(plate_width, plate_depth);
  }

  // battery tray
  rotate(135) {
    translate([0, -20, 0]) {
      BatteryTray(plate_width, plate_depth, battery_width, battery_length);
    }
  }


  // battery stop
  difference() {
    rotate(-45) {
      BatteryStop(plate_width, plate_depth, battery_width, battery_length);
    }

    rotate([45, 90, 0]) {
      translate([-22.5, 0, (plate_width / 2.0)]) {
        #CameraMountHoles(2);
      }
    }
  }
}

Plate(plate_width, plate_depth);

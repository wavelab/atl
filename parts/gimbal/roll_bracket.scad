module MotorMountHoles(depth) {
  mount_width = 19;
  mount_height = 16;
  screw_type = 2.5;

  translate([mount_width / 2.0, 0.0]) {
    #cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
  translate([-mount_width / 2.0, 0.0]) {
    #cylinder(r=screw_type / 2.0, h=depth, center=true);
  }

  translate([0.0, mount_height / 2.0]) {
    #cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
  translate([0.0, -mount_height / 2.0]) {
    #cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
}

module GimbalBracket(width, length, depth) {
  cube([width, length, depth], center=true);
  translate([0.0, (length / 2.0) - depth / 2.0, width / 2.0]) {
    cube([width, depth, width], center=true);
  }

  // fillet
  translate([0, (length / 2.0) - depth, depth / 2.0]) {
      rotate([0, 90]) {
        difference() {
            cylinder(r=3, h=width, center=true);
            translate([-3.0, -3.0, 0.0]) {
              cylinder(r=3, h=width, center=true);
            }
        }
      }
  }
}

module MotorVariableMountHole(depth) {
  screw_type = 1.5;
  width = 50;
  height = 12;

  translate([height / 2.0, 0.0]) {
    cube([screw_type, width, depth], center=true);
    translate([0.0, width / 2.0]) {
      cylinder(r=screw_type / 2.0, h=depth, center=true);
    }
    translate([0.0, -width / 2.0]) {
      cylinder(r=screw_type / 2.0, h=depth, center=true);
    }
  }

  translate([-height / 2.0, 0.0]) {
    cube([screw_type, width, depth], center=true);
    translate([0.0, width / 2.0]) {
      cylinder(r=screw_type / 2.0, h=depth, center=true);
    }
    translate([0.0, -width / 2.0]) {
      cylinder(r=screw_type / 2.0, h=depth, center=true);
    }
  }
}

difference() {
  bracket_width = 35;
  bracket_length = 70;
  bracket_depth = 3;

  // gimbal bracket
  GimbalBracket(bracket_width, bracket_length, bracket_depth);

  // motor mount holes
  translate([0.0, -2.0]) {
    MotorVariableMountHole(bracket_depth + 0.01);
  }

  translate([0.0, (bracket_length / 2.0) - (bracket_depth / 2.0), bracket_width / 2.0]) {
    rotate([90.0, 0.0, 0.0]) {
      MotorMountHoles(bracket_depth + 0.01);
    }
  }
}

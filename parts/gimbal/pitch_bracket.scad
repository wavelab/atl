module MotorMountHoles(depth) {
  mount_width = 12;
  mount_height = 12;
  screw_type = 1.5;

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

module PointGreyMountHoles(depth) {
  mount_width = 30;
  mount_height = 14;
  screw_type = 2.5;

  translate([mount_height / 2.0, mount_width / 2.0]) {
    #cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
  translate([mount_height / 2.0, -mount_width / 2.0]) {
    #cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
  translate([-mount_height / 2.0, mount_width / 2.0]) {
    #cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
  translate([-mount_height / 2.0, -mount_width / 2.0]) {
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

difference() {
  bracket_width = 40;
  bracket_length = 52;
  bracket_depth = 3;

  // gimbal bracket
  GimbalBracket(bracket_width, bracket_length, bracket_depth);

  // camera mount holes
  translate([0.0, -2.0, 0.0]) {
    PointGreyMountHoles(bracket_depth + 0.01);
  }

  // motor mount holes
  translate([0.0, (bracket_length / 2.0) - (bracket_depth / 2.0), 20]) {
    rotate([90.0, 0.0, 0.0]) {
      MotorMountHoles(bracket_depth + 0.01);
    }
  }
}

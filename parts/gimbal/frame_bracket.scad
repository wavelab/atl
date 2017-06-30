module MotorMountHoles(depth) {
  mount_width = 19.0;
  mount_height = 16.0;
  screw_type = 2.5;

  translate([mount_width / 2.0, 0.0]) {
    cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
  translate([-mount_width / 2.0, 0.0]) {
    cylinder(r=screw_type / 2.0, h=depth, center=true);
  }

  translate([0.0, mount_height / 2.0]) {
    cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
  translate([0.0, -mount_height / 2.0]) {
    cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
}

module GimbalBracket(width, length, depth) {
  bracket_width = 45;
  bracket_length = 70;

  // bracket
  cube([width, length, depth], center=true);
  translate([0.0, (bracket_length / 2.0) - depth / 2.0, bracket_width / 2.0]) {
    cube([bracket_width, depth, bracket_width], center=true);
  }

  // fillet
  translate([0, (length / 2.0) - depth, depth / 2.0]) {
      rotate([0, 90]) {
        difference() {
            cylinder(r=3, h=bracket_width, center=true);
            translate([-3.0, -3.0, 0.0]) {
              cylinder(r=3, h=bracket_width, center=true);
            }
        }
      }
  }
}

difference() {
  bracket_width = 35;
  bracket_length = 70;
  bracket_depth = 3;

  // gimbal bracket
  GimbalBracket(45, bracket_length, bracket_depth);

  // misumi mount holes
  translate([10.0, 0.0]) cylinder(r=5.5 / 2.0, h=bracket_depth, center=true);
  translate([-10.0, 0.0]) cylinder(r=5.5 / 2.0, h=bracket_depth, center=true);
  translate([10.0, -20.0]) cylinder(r=5.5 / 2.0, h=bracket_depth, center=true);
  translate([-10.0, -20.0]) cylinder(r=5.5 / 2.0, h=bracket_depth, center=true);

  // motor mount holes
  translate([0.0, (bracket_length / 2.0) - (bracket_depth / 2.0), (bracket_width / 2.0) + 8]) {
    rotate([90.0, 0.0, 0.0]) {
      MotorMountHoles(bracket_depth + 0.01);
    }
  }
}

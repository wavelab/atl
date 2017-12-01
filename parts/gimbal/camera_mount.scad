module CameraMount(width, length, depth) {
  // bracket
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

module USBCameraMountHoles(depth) {
  mount_width = 28.0;
  screw_type = 2.0;

  translate([mount_width / 2.0, mount_width / 2.0]) {
    #cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
  translate([mount_width / 2.0, -mount_width / 2.0]) {
    #cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
  translate([-mount_width / 2.0, mount_width / 2.0]) {
    #cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
  translate([-mount_width / 2.0, -mount_width / 2.0]) {
    #cylinder(r=screw_type / 2.0, h=depth, center=true);
  }
}

module PointGreyMountHoles(depth) {
  mount_width = 30.0;
  mount_height = 14.0;
  $fn = 32.0;
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

difference() {
  mount_width = 40.0;
  mount_length = 30.0;
  mount_depth = 3.0;

  // camera mount
  CameraMount(mount_width, mount_length, mount_depth);

  // cable holes
  translate([0.0,
             mount_length / 2.0 - mount_depth / 2.0,
             mount_width / 2.0]) {
    #cube([12.5, mount_depth, 6.0], center=true);

    translate([12.5 / 2.0, 0.0]) {
      rotate([90, 0]) {
        #cylinder(r = 6 / 2.0, h = mount_depth + 1.0, center = true);
      }
    }

    translate([-12.5 / 2.0, 0.0]) {
      rotate([90, 0]) {
        #cylinder(r = 6.0 / 2.0, h = mount_depth + 1.0, center = true);
      }
    }
  }

  // usb camera mount holes
  translate([0.0, mount_length / 2.0 - mount_depth / 2.0, mount_width / 2.0]) {
    rotate([90.0, 0.0]) {
      USBCameraMountHoles(mount_depth + 0.1);
    }
  }

  // point grey mount holes
  translate([0.0, -mount_depth, 0.0]) {
    rotate(90) {
      PointGreyMountHoles(mount_depth + 0.1);
    }
  }
}

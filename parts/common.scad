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

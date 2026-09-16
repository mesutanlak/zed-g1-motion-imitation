// Parametric ZED 2i adapter for MakerWorld universal octagon socket.
// Units: mm. Export the STL with $fn=96 for smooth screw holes.
$fn = 96;

fit = "standard"; // "standard" or "loose"
peg_r = fit == "loose" ? 17.75 : 17.90;
peg_h = 5.8;
plate_x = 80;
plate_y = 42;
plate_h = 6;
corner_r = 6;

module rounded_box_2d(x, y, r) {
    hull()
        for (sx = [-1, 1], sy = [-1, 1])
            translate([sx * (x/2-r), sy * (y/2-r)]) circle(r=r);
}

module chamfered_octagon(r, h) {
    union() {
        cylinder(h=0.7, r1=r-0.65, r2=r, $fn=8);
        translate([0,0,0.7]) cylinder(h=h-0.7, r=r, $fn=8);
    }
}

difference() {
    union() {
        chamfered_octagon(peg_r, peg_h);
        translate([0,0,peg_h])
            linear_extrude(height=plate_h) rounded_box_2d(plate_x, plate_y, corner_r);
    }
    translate([0,0,-0.1]) cylinder(h=4.9, d=12.0);
    translate([0,0,4.79]) cylinder(h=plate_h+peg_h, d=6.8);
}

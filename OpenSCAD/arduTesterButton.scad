// ardutesterButton.scad

totalHeight = 15;
stemHeight = 10;

holeDepth = 10;

topDia = 9.5;
stemDia = 12;
holeDia = 8;

roundness = 100;

difference() {
union() {
color("blue")
cylinder(h = totalHeight, d = topDia, center = true, $fn = roundness);

translate([0, 0, -totalHeight / 2 + stemHeight / 2])
color("green")
cylinder(h = stemHeight, d = stemDia, center = true, $fn = roundness);
}

translate([0, 0, -totalHeight / 2 + holeDepth / 2])
color("red")
cylinder(h = holeDepth, d = holeDia, center = true, $fn = roundness);
}
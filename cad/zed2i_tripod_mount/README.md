# ZED 2i Tripod Mount

A compact adapter plate for mounting a Stereolabs ZED 2i camera to a tripod.

## Geometry

- Plate: 80 x 35 x 8 mm, 4 mm corner radius
- Camera mounting: 2 x M3 clearance holes, 36 mm centre spacing
- M3 head recesses: 6.5 mm diameter x 3.2 mm deep, accessed from below
- Tripod opening: 7.0 mm clearance for a 1/4-20 UNC screw
- Captive nut pocket: 11.3 mm across flats x 5.8 mm deep, loaded from the top

## Hardware

- 2 x M3x10 socket-head screws for the ZED 2i
- 1 x standard 1/4-20 UNC hex nut
- Optional: 1 mm rubber/cork sheet between the camera and plate

The ZED 2i specifies a maximum screw entry of 10 mm for the M3 holes and 7 mm
for its 1/4-20 mounting hole. This adapter uses the two M3 holes so the camera
cannot rotate, while the tripod attaches independently to the central captive nut.

## Fusion 360

Open the STEP file directly in Fusion 360, or run the included
`fusion360/ZED2iTripodMount.py` script from **Utilities > Scripts and Add-Ins**
to generate a native parametric design with named features and user parameters.

For FDM printing, use PETG/ASA, 4 perimeters, 35-50% infill, and print flat with
the broad bottom face on the build plate. Test-fit the nut before final assembly.

# surfaceofrevolutions
Generates a curve using either b-spline or Bezier method, and then rotates the curve across an axis of rotation using surface of revolutions method. The program requires installation of OpenGL.

## Keyboard commands
  The following keyboard commands are used to control the
  program:

    q - Quit the program
    c - Clear the screen
    e - Erase the B-spline curve
    p - Toggle control point; default is on
    g - Toggle control polygon; default is off
    d - Toggle B-spline curve; default is off 
    s - Toggle "selection mode"; default is off
    n - Toggle surface of revolution; default is off
    h - set rho (angle of rotation) to zero again
    r - record control points into text file
    l - load control points from text file

  If "selection mode" is on, right click finds the nearest point
  and highlights it. Left click performs translation. When
  "selection mode" is off, can add control points to the display.
  
  Rotation of the points, curves, and surfaces can be performed
  by rotating about the x-axis.

## Program features
-[X] Control point input On/Off: When ON, user can add control 
      points
-[X] Control polygon On/Off: When ON, program will display 
      control polygon
-[X] B-spline curve On/Off: When ON, program will display the
      B-spline curve based on control points
-[X] Select control point: In this mode, program can allow user
      to select a control point
-[.] Delete or move selected control point: (feature of program,
      and not available through menu); program will automatically
      update the current control polygon/B-spline curve when the
      selected control point is deleted or moved
-[X] Save: Save current control points to bspline.txt
-[X] Retrieve: Retrieve control points from bspline.txt
-[X] Clear: Clear the current display window and delete all
      control points
-[X] Draw wireframe surface: display the B-spline surface as a
      wireframe mesh
-[X] Shade surface: shade the B-spline surface using pre-specified
      lighting and material parameters
-[ ] Texture surface: allow the user to texture map either an
      image or a texture pattern onto the B-spline surface; user
      can choose between the options
-[.] Have documentation available

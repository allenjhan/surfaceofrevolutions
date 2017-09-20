
/*
**
**  Allen Han
** 
**  The following keyboard commands are used to control the
**  program:
**
**    q - Quit the program
**    c - Clear the screen
**    e - Erase the B-spline curve
**    p - Toggle control point; default is on
**    g - Toggle control polygon; default is off
**    d - Toggle B-spline curve; default is off 
**    s - Toggle "selection mode"; default is off
**    n - Toggle surface of revolution; default is off
**    h - set rho (angle of rotation) to zero again
**    r - record control points into text file
**    l - load control points from text file
**
**  If "selection mode" is on, right click finds the nearest point
**  and highlights it. Left click performs translation. When
**  "selection mode" is off, can add control points to the display.
**  
**  Rotation of the points, curves, and surfaces can be performed
**  by rotating about the x-axis.
**
**  [X] Control point input On/Off: When ON, user can add control 
**      points
**  [X] Control polygon On/Off: When ON, program will display 
**      control polygon
**  [X] B-spline curve On/Off: When ON, program will display the
**      B-spline curve based on control points
**  [X] Select control point: In this mode, program can allow user
**      to select a control point
**  [.] Delete or move selected control point: (feature of program,
**      and not available through menu); program will automatically
**      update the current control polygon/B-spline curve when the
**      selected control point is deleted or moved
**  [X] Save: Save current control points to bspline.txt
**  [X] Retrieve: Retrieve control points from bspline.txt
**  [X] Clear: Clear the current display window and delete all
**      control points
**  [X] Draw wireframe surface: display the B-spline surface as a
**      wireframe mesh
**  [X] Shade surface: shade the B-spline surface using pre-specified
**      lighting and material parameters
**  [ ] Texture surface: allow the user to texture map either an
**      image or a texture pattern onto the B-spline surface; user
**      can choose between the options
**  [.] Have documentation available
**
*/

#include <GL/glut.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

typedef enum {
  BSPLINE,
} curveType;

static curveType selectCurve = BSPLINE;

typedef struct Vectors{
  GLfloat x;
  GLfloat y;
  GLfloat z;
}Vector;

static void keyboard(unsigned char key, int x, int y);
static void lightingInit();

static int ctrl_pt_on = 1;
static int ctrl_poly_on = 0;
static int bspline_on = 0;
static int selection_on = 0;
static int bsurface_on = 0;
static int calculate_bspline_curve = 0;
static int calculate_bspline_surface = 0;
static int current_selected_point = -1;

#define MAX_CPTS  75            /* Fixed maximum number of control points */
#define MAX_KNOTS MAX_CPTS+5
#define BSPLINE_PARTITION 5
#define MAX_BPTS MAX_KNOTS*BSPLINE_PARTITION

static GLfloat cpts[MAX_CPTS][3];
static int ncpts = 0;

#define GLUT_MOUSE_NULL -1
static int current_button;

static GLfloat knot[MAX_KNOTS];
static GLfloat bspline[MAX_BPTS][3];
static int num_bspline_pts = 0;

#define ANGLE_PARTITION 0.125
#define DEGREES_OF_REVOLUTION 360
static GLfloat bspline_copy[MAX_BPTS][3];
static GLfloat bspline_surface[DEGREES_OF_REVOLUTION][MAX_BPTS][6][3];

static GLfloat rho = 0;

#define TEXTURE_WIDTH 256
#define TEXTURE_HEIGHT 256

static int width = 500, height = 500;     /* Window width and height */

static Vector makeVector(GLfloat* from, GLfloat* to){
  Vector out;
  out.x = to[0] - from[0];
  out.y = to[1] - from[1];
  out.z = to[2] - from[2];

  return out;
}

static Vector normalizeVector(Vector a){
  Vector n;
  GLfloat length = sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
  n.x = a.x / length;
  n.y = a.y / length;
  n.z = a.z / length;

  return n;
}

static Vector crossProduct(Vector a, Vector b){
  Vector c;

  c.x = a.x*b.z - a.z*b.y;
  c.y = a.x*b.z - a.z*b.x;
  c.z = a.x*b.y - a.y*b.x;

  return c;
}

static GLfloat deCasteljau(int i, int p, float t){
  GLfloat return_value = 0;
  GLfloat left = 0;
  GLfloat right = 0;

  if (p==1) {
    if (knot[i]<knot[i+1] && knot[i]<=t && t<knot[i+1])
      return_value = 1.0;
    else
      return_value = 0.0;
  } else {
    if (knot[i+p-1]-knot[i]!=0.0)
      left = deCasteljau(i, p-1, t) * (t-knot[i]) / (knot[i+p-1]-knot[i]);
    else {
      #ifdef DEBUG
      printf("Singularity for left value. i is %d. p is %d.\n", i, p);
      #endif
      left = 0.0;
    }
    if (knot[i+p]-knot[i+1]!=0.0)
      right = deCasteljau(i+1, p-1, t) * (knot[i+p]-t) / (knot[i+p]-knot[i+1]);
    else {
      #ifdef DEBUG
      printf("Singularity for right value. i is %d. p is %d.\n", i, p);
      #endif
      right = 0.0;
    }
    return_value = left + right;
    #ifdef DEBUG
    printf("i is %d. return_value is %f.\n", i, return_value);
    #endif
  }

  return return_value;
}

static int setKnotArray(GLfloat* knot, int ncpts){
  int return_value = 0;
  int m = ncpts - 1;

  if (ncpts<4)
    return_value = -1;
  else {
    for(int i = 0; i<=m+4; i++){
      if (i<=3)
	knot[i] = 0;
      else if (i<=m)
	knot[i] = i-3;
      else
	knot[i] = m-2;
    }
  }

  return return_value;
}

static void calculateBsplineCurve(){
  GLfloat B0;
  GLfloat B1;
  GLfloat B2;
  GLfloat B3;
  GLfloat t;
  GLfloat interval;
  int num_knots = ncpts + 3;

  glColor3f(0.0, 1.0, 0.0);

  if (setKnotArray(knot, ncpts) < 0){
    printf("error creating knot array\n");
    return;
  } else {
    #ifdef DEBUG
    printf("knot array created successfully\n");
    for(int i=0; i<ncpts+4; i++)
      printf("%f\n", knot[i]);
    #endif
  }
  
  #ifdef DEBUG
  FILE *out;
  out = fopen("output.txt", "w");
  #endif

  num_bspline_pts = 0;
  for (int i=3; i<num_knots-3; i++){
    t = knot[i];
    interval = knot[i+1]-knot[i];
    for(int j = 0; j<BSPLINE_PARTITION; j++, num_bspline_pts++){
      B0 = deCasteljau(i, 4, t);
      B1 = deCasteljau(i-1, 4, t);
      B2 = deCasteljau(i-2, 4, t);
      B3 = deCasteljau(i-3, 4, t);

      bspline[num_bspline_pts][0] = cpts[i][0]*B0 + cpts[i-1][0]*B1 + cpts[i-2][0]*B2 + cpts[i-3][0]*B3;
      bspline[num_bspline_pts][1] = cpts[i][1]*B0 + cpts[i-1][1]*B1 + cpts[i-2][1]*B2 + cpts[i-3][1]*B3;
      bspline[num_bspline_pts][2] = cpts[i][2]*B0 + cpts[i-1][2]*B1 + cpts[i-2][2]*B2 + cpts[i-3][2]*B3;

      #ifdef DEBUG
      fprintf(out, "the current parametric variable is %f\n", t);
      fprintf(out, "blending function 0 has value %f\n", B0);
      fprintf(out, "blending function 1 has value %f\n", B1);
      fprintf(out, "blending function 2 has value %f\n", B2);
      fprintf(out, "blending function 3 has value %f\n", B3);
      fprintf(out, "index %d x-value %f\n", num_bspline_pts, bspline[num_bspline_pts][0]);
      fprintf(out, "index %d y-value %f\n", num_bspline_pts, bspline[num_bspline_pts][1]);
      fprintf(out, "index %d z-value %f\n", num_bspline_pts, bspline[num_bspline_pts][2]);
      #endif

      t += interval/BSPLINE_PARTITION;
    }
  }

  bspline[num_bspline_pts][0] = cpts[ncpts-1][0];
  bspline[num_bspline_pts][1] = cpts[ncpts-1][1];
  bspline[num_bspline_pts][2] = cpts[ncpts-1][2];
  num_bspline_pts++;

  #ifdef DEBUG
  fprintf(out, "index %d x-value %f\n", num_bspline_pts, bspline[num_bspline_pts][0]);
  fprintf(out, "index %d y-value %f\n", num_bspline_pts, bspline[num_bspline_pts][1]);
  fprintf(out, "index %d z-value %f\n", num_bspline_pts, bspline[num_bspline_pts][2]);
  
  fclose(out);
  #endif

}
  
static void drawBsplineCurve(){
  // draw the bspline curve
  glColor3f(0.0, 1.0, 0.0);
  for (int k=0; k<num_bspline_pts-1; k++){
    glBegin(GL_LINES);
    glVertex3fv(bspline[k]);
    glVertex3fv(bspline[k+1]);
    glEnd();
  }
}

static void calculateBsplineSurface(){

  GLfloat theta_incr = 1/(float) ANGLE_PARTITION;
  GLfloat theta_incr_rad = theta_incr * M_PI / 180;
  GLfloat theta = 0;
  GLfloat i_vertex3x = 0;
  GLfloat i_vertex3y = 0;
  GLfloat i_vertex3z = 0;
  GLfloat ip1_vertex3x = 0;
  GLfloat ip1_vertex3y = 0;
  GLfloat ip1_vertex3z = 0;

  #ifdef DEBUG
  printf("calculateBsplineSurface() entered\n");

  FILE *out;
  out = fopen("calculateBsplineSurface_debug.txt","w");

  printf("theta_incr has value %f\n", theta_incr);
  #endif

  for(int i=0; i<num_bspline_pts; i++){
    bspline_copy[i][0] = bspline[i][0];
    bspline_copy[i][1] = bspline[i][1];
    bspline_copy[i][2] = bspline[i][2];
  }
  
  int j = 0;
  while(theta<DEGREES_OF_REVOLUTION){
    #ifdef DEBUG
    fprintf(out, "theta: %f\n", theta);
    #endif
    int i = 0;
    for(; i<num_bspline_pts-1; i++){
      ip1_vertex3x = bspline_copy[i+1][0]*cos(theta_incr_rad) + bspline_copy[i+1][2]*sin(theta_incr_rad);
      ip1_vertex3y = bspline_copy[i+1][1];
      ip1_vertex3z = -1*bspline_copy[i+1][0]*sin(theta_incr_rad) + bspline_copy[i+1][2]*cos(theta_incr_rad);
      i_vertex3x = bspline_copy[i][0]*cos(theta_incr_rad) + bspline_copy[i][2]*sin(theta_incr_rad);
      i_vertex3y = bspline_copy[i][1];
      i_vertex3z = -1*bspline_copy[i][0]*sin(theta_incr_rad) + bspline_copy[i][2]*cos(theta_incr_rad);

      bspline_surface[j][i][0][0] = bspline_copy[i][0];
      bspline_surface[j][i][0][1] = bspline_copy[i][1];
      bspline_surface[j][i][0][2] = bspline_copy[i][2];
      bspline_surface[j][i][1][0] = bspline_copy[i+1][0];
      bspline_surface[j][i][1][1] = bspline_copy[i+1][1];
      bspline_surface[j][i][1][2] = bspline_copy[i+1][2];
      bspline_surface[j][i][2][0] = ip1_vertex3x;
      bspline_surface[j][i][2][1] = ip1_vertex3y;
      bspline_surface[j][i][2][2] = ip1_vertex3z;

      bspline_surface[j][i][3][0] = ip1_vertex3x;
      bspline_surface[j][i][3][1] = ip1_vertex3y;
      bspline_surface[j][i][3][2] = ip1_vertex3z;
      bspline_surface[j][i][4][0] = i_vertex3x;
      bspline_surface[j][i][4][1] = i_vertex3y;
      bspline_surface[j][i][4][2] = i_vertex3z;
      bspline_surface[j][i][5][0] = bspline_copy[i][0];
      bspline_surface[j][i][5][1] = bspline_copy[i][1];
      bspline_surface[j][i][5][2] = bspline_copy[i][2];
      
      #ifdef DEBUG
      fprintf(out, "triangle 1, vertex 1: %f %f %f\n", bspline_copy[i][0], bspline_copy[i][1], bspline_copy[i][2]);
      fprintf(out, "triangle 1, vertex 2: %f %f %f\n", bspline_copy[i+1][0], bspline_copy[i+1][1], bspline_copy[i+1][2]);
      fprintf(out, "triangle 1, vertex 3: %f %f %f\n", ip1_vertex3x, ip1_vertex3y, ip1_vertex3z);
      fprintf(out, "triangle 2, vertex 1: %f %f %f\n", ip1_vertex3x, ip1_vertex3y, ip1_vertex3z);
      fprintf(out, "triangle 2, vertex 2: %f %f %f\n", i_vertex3x, i_vertex3y, i_vertex3z);
      fprintf(out, "triangle 2, vertex 3: %f %f %f\n", bspline_copy[i][0], bspline_copy[i][1], bspline_copy[i][2]);
      #endif

      bspline_copy[i][0] = i_vertex3x;
      bspline_copy[i][1] = i_vertex3y;
      bspline_copy[i][2] = i_vertex3z;
    }
    bspline_copy[i][0] = ip1_vertex3x;
    bspline_copy[i][1] = ip1_vertex3y;
    bspline_copy[i][2] = ip1_vertex3z;
    
    theta = theta + theta_incr;
    j++;
  }
  #ifdef DEBUG
  fclose(out);
  #endif

}

static void drawBsplineWireframeSurface(){
  glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
  glColor4f(0.0, 0.0, 1, 1);

  int total_angles = DEGREES_OF_REVOLUTION*ANGLE_PARTITION;
  
  for(int j=0; j<total_angles; j++){
    for(int i=0; i<num_bspline_pts; i++){
      glBegin(GL_TRIANGLES);
      glVertex3fv(bspline_surface[j][i][0]);
      glVertex3fv(bspline_surface[j][i][1]);
      glVertex3fv(bspline_surface[j][i][2]);
      glEnd();

      glBegin(GL_TRIANGLES);
      glVertex3fv(bspline_surface[j][i][3]);
      glVertex3fv(bspline_surface[j][i][4]);
      glVertex3fv(bspline_surface[j][i][5]);
      glEnd();

    }
  }
}

static void drawBsplineLightedSurface(){
  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  lightingInit();

  #ifdef DEBUG
  FILE *out;
  out = fopen("bspline_lighted_surface.txt", "w");
  #endif

  int total_angles = DEGREES_OF_REVOLUTION*ANGLE_PARTITION;
  
  for(int j=0; j<total_angles; j++){
    for(int i=0; i<num_bspline_pts; i++){
      glBegin(GL_TRIANGLES);
      glVertex3fv(bspline_surface[j][i][0]);
      glVertex3fv(bspline_surface[j][i][1]);
      glVertex3fv(bspline_surface[j][i][2]);
      glEnd();

      if(i!=num_bspline_pts-1){
	Vector triangle1_side1 = makeVector(bspline_surface[j][i][1], bspline_surface[j][i][0]);
	Vector triangle1_side2 = makeVector(bspline_surface[j][i][1], bspline_surface[j][i][2]);
	Vector triangle1_normal = crossProduct(triangle1_side1, triangle1_side2);
	Vector normalized_triangle1_normal = normalizeVector(triangle1_normal);
	glNormal3f(normalized_triangle1_normal.x, normalized_triangle1_normal.y, normalized_triangle1_normal.z);

        #ifdef DEBUG
	fprintf(out, "1: index j is %d, index i is %d, normal is %f %f %f\n", j, i,
		normalized_triangle1_normal.x, normalized_triangle1_normal.y, normalized_triangle1_normal.z);
        #endif
      }
      
      glBegin(GL_TRIANGLES);
      glVertex3fv(bspline_surface[j][i][3]);
      glVertex3fv(bspline_surface[j][i][4]);
      glVertex3fv(bspline_surface[j][i][5]);
      glEnd();

      if(i!=num_bspline_pts-1){
	Vector triangle2_side1 = makeVector(bspline_surface[j][i][4], bspline_surface[j][i][3]);
	Vector triangle2_side2 = makeVector(bspline_surface[j][i][4], bspline_surface[j][i][5]);
	Vector triangle2_normal = crossProduct(triangle2_side1, triangle2_side2);
	Vector normalized_triangle2_normal = normalizeVector(triangle2_normal);
	glNormal3f(normalized_triangle2_normal.x, normalized_triangle2_normal.y, normalized_triangle2_normal.z);

        #ifdef DEBUG
        fprintf(out, "2: index j is %d, index i is %d, normal is %f %f %f\n", j, i,
		normalized_triangle2_normal.x, normalized_triangle2_normal.y, normalized_triangle2_normal.z);
        #endif
      }
    }
  }

  #ifdef DEBUG
  fclose(out);
  #endif
}

static void drawBsplineTexturedSurface(){

  GLubyte m_tex[TEXTURE_WIDTH][TEXTURE_HEIGHT][3];
  
  FILE *in = fopen("ref/marble256.bin", "r");
  #ifdef DEBUG
  FILE *m_tex_out;
  m_tex_out = fopen("marble_texture_read_in.txt", "w");
  #endif

  GLubyte* marble_color_ptr = m_tex[0][0];
  for(int i=0; i<3; i++) {
    marble_color_ptr = m_tex[0][0]+i;
    for(int j=0; j<TEXTURE_WIDTH; j++) {
      for(int k=0; k<TEXTURE_HEIGHT; k++) {
	fscanf(in, "%c", marble_color_ptr);

        #ifdef DEBUG
	fprintf(m_tex_out, "RGB %d width-index %d height-index %d marble_color %d\n", i, j, k, (int) *marble_color_ptr);
        #endif

	marble_color_ptr+=3;
      }
    }
    #ifdef DEBUG
    printf("i is currently %d, and will be %d\n", i, i+1);
    #endif
  }

  fclose(in);
  #ifdef DEBUG
  fclose(m_tex_out);
  #endif
  
  GLuint tex_name_array[1];
  
  glEnable(GL_TEXTURE_2D);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, m_tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

  int total_angles = DEGREES_OF_REVOLUTION*ANGLE_PARTITION;
  for(int j=0; j<total_angles; j++){
    for(int i=0; i<num_bspline_pts; i++){
      glBegin(GL_TRIANGLES);
      glTexCoord2f(j/total_angles, i/num_bspline_pts);
      glVertex3fv(bspline_surface[j][i][0]);
      glTexCoord2f(j/total_angles, (i+1)/num_bspline_pts);
      glVertex3fv(bspline_surface[j][i][1]);
      glTexCoord2f((j+1)/total_angles, i/num_bspline_pts);
      glVertex3fv(bspline_surface[j][i][2]);
      glEnd();

      glBegin(GL_TRIANGLES);
      glTexCoord2f((j+1)/total_angles, (i+1)/num_bspline_pts);
      glVertex3fv(bspline_surface[j][i][3]);
      glTexCoord2f((j+1)/total_angles, i/num_bspline_pts);
      glVertex3fv(bspline_surface[j][i][4]);
      glTexCoord2f(j/total_angles, i/num_bspline_pts);
      glVertex3fv(bspline_surface[j][i][5]);
      glEnd();
    }
  }

  glDisable(GL_TEXTURE_2D);
}

static void bsplineMain(){
  if (calculate_bspline_curve == 1) 
    calculateBsplineCurve();
  if (bspline_on == 1)
    drawBsplineCurve();
  if (calculate_bspline_surface == 1)
    calculateBsplineSurface();
  if (bsurface_on == 1)
    drawBsplineWireframeSurface();
  else if (bsurface_on == 2)
    drawBsplineLightedSurface();
  else if (bsurface_on == 3)
    drawBsplineTexturedSurface();
}

static void display(void){
  int i;

  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  glDisable(GL_LIGHTING);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glRotatef(rho, 1.0, 0.0, 0.0);

  if (ctrl_pt_on==1){
    glPointSize(5.0);
    glBegin(GL_POINTS);
    for (i = 0; i < ncpts; i++){
      if (i == current_selected_point && selection_on == 1)
	glColor3f(1.0, 0.0, 1.0);
      else
	glColor3f(0.0, 0.0, 0.0);
      glVertex3fv(cpts[i]);
    }
    glEnd();
  }

  if (ctrl_poly_on==1 && ncpts>1){
    glColor3f(1.0, 1.0, 0.0);
    for (i = 0; i < ncpts-1; i++){
      glBegin(GL_LINES);
      glVertex3fv(cpts[i]);
      glVertex3fv(cpts[i+1]);
      glEnd();
    }
    glBegin(GL_LINES);
    glVertex3fv(cpts[ncpts-1]);
    glVertex3fv(cpts[0]);
    glEnd();
  }

  if ( bspline_on==1 || bsurface_on!=0){
    bsplineMain();
  }

  calculate_bspline_curve = 0;
  calculate_bspline_surface = 0;

  glFlush();
}


static void mouse(int button, int state, int x, int y){
  float wx, wy;

  if (selection_on==0){
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN){
      current_button = GLUT_LEFT_BUTTON;
    } else if (button == 3 && state == GLUT_DOWN){
      current_button = GLUT_MIDDLE_BUTTON;
      rho += 5.0;
    } else if (button == 4 && state == GLUT_DOWN) {
      current_button = GLUT_MIDDLE_BUTTON;
      rho -= 5.0;
    } else {
      current_button = GLUT_MOUSE_NULL;
      return;
    }
  } else {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN){
      current_button = GLUT_LEFT_BUTTON;
    } else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN){
      current_button = GLUT_RIGHT_BUTTON;
    } else if (button == 3 && state == GLUT_DOWN){
      current_button = GLUT_MIDDLE_BUTTON;
      rho += 5.0;
    } else if (button == 4 && state == GLUT_DOWN) {
      current_button = GLUT_MIDDLE_BUTTON;
      rho -= 5.0;
    } else {
      current_button = GLUT_MOUSE_NULL;
      return;
    }
  }

  /* Translate back to our coordinate system */
  wx = (2.0 * x) / (float)(width - 1) - 1.0;
  wy = (2.0 * (height - 1 - y)) / (float)(height - 1) - 1.0;

  if (selection_on==0 && button==GLUT_LEFT_BUTTON && state==GLUT_DOWN){
    if (ncpts>=MAX_CPTS){
      printf("Maximum number of control points is 75.\n");
      return;
    }

    /* Save the point */
    cpts[ncpts][0] = wx;
    cpts[ncpts][1] = wy;
    cpts[ncpts][2] = 0.0;
    ncpts++;

    calculate_bspline_curve = 1;
    calculate_bspline_surface = 1;
  } else if (selection_on==1 && button==GLUT_RIGHT_BUTTON && state==GLUT_DOWN){
    int i = 0;
    GLfloat current_distance_squared = 0;

    int closest_point_to_cursor = 0;
    GLfloat shortest_distance_squared = FLT_MAX;

    while (i<ncpts){
      current_distance_squared = (cpts[i][0]-wx)*(cpts[i][0]-wx)+
	                         (cpts[i][1]-wy)*(cpts[i][1]-wy);
      if (current_distance_squared < shortest_distance_squared){
	closest_point_to_cursor = i;
	shortest_distance_squared = current_distance_squared;
      }
      i++;
    }

    current_selected_point = closest_point_to_cursor;
  } else if (selection_on==1 && button==GLUT_LEFT_BUTTON && state==GLUT_DOWN){

    if ( wx>cpts[current_selected_point][0] )
      cpts[current_selected_point][0] += 0.01;
    else 
      cpts[current_selected_point][0] -= 0.01;

    if ( wy>cpts[current_selected_point][1] )
      cpts[current_selected_point][1] += 0.01;
    else 
      cpts[current_selected_point][1] -= 0.01;

    calculate_bspline_curve = 1;
    calculate_bspline_surface = 1;
  }
  
  display();
}

static void moveObject(int x, int y){
  float wx;
  float wy;
  wx = (2.0 * x) / (float)(width - 1) - 1.0;
  wy = (2.0 * (height - 1 - y)) / (float)(height - 1) - 1.0;
  if ( selection_on==1 && current_button == GLUT_LEFT_BUTTON ){
    if ( wx>cpts[current_selected_point][0] )
      cpts[current_selected_point][0] += 0.01;
    else 
      cpts[current_selected_point][0] -= 0.01;

    if ( wy>cpts[current_selected_point][1] )
      cpts[current_selected_point][1] += 0.01;
    else 
      cpts[current_selected_point][1] -= 0.01;

    calculate_bspline_curve = 1;
    display();
  } 
}

/* This routine handles keystroke commands */
static void keyboard(unsigned char key, int x, int y){
  FILE *record;
  int fileSuccessfullyRead1;
  int fileSuccessfullyRead2;
  int fileSuccessfullyRead3;
  int fileSuccessfullyRead4;
  
  switch (key) {
  case 'q': case 'Q':
    exit(0);
    break;
  case 'c': case 'C':
    ncpts = 0;
    num_bspline_pts = 0;
  case 'h': case 'H':
    rho = 0;
    break;
  case 'e': case 'E':
    num_bspline_pts = 0;
    break;
  case 'p': case 'P':
    if (ctrl_pt_on == 0)
      ctrl_pt_on = 1;
    else
      ctrl_pt_on = 0;
    break;
  case 'g': case 'G':
    if (ctrl_poly_on == 0)
      ctrl_poly_on = 1;
    else
      ctrl_poly_on = 0;
    break;
  case 'd': case 'D':
    if (bspline_on == 0){
      calculate_bspline_curve = 1;
      bspline_on = 1;
    } else
      bspline_on = 0;
    break;
  case 's': case 'S':
    if (selection_on == 0)
      selection_on = 1;
    else {
      selection_on = 0;
      current_selected_point = -1;
    }
    break;
  case 'n': case 'N':
    if (bsurface_on == 0){
      calculate_bspline_curve = 1;
      calculate_bspline_surface = 1;
      bsurface_on = 1;
    } else if (bsurface_on == 1){
      calculate_bspline_curve = 1;
      calculate_bspline_surface = 1;
      bsurface_on = 2;
    } else if (bsurface_on == 2){
      calculate_bspline_curve = 1;
      calculate_bspline_surface = 1;
      bsurface_on = 3;
    } else
      bsurface_on = 0;

    #ifdef DEBUG
    printf("bsurface_on has the value %d\n", bsurface_on);
    #endif
    break;
  case 'r': case 'R':
    record = fopen("bspline.txt", "w");
    if (record == NULL) {
      printf("Warning: Could not open file for recording.\n");
    } else {
      for(int j=0; j<ncpts; j++){
	fprintf(record, "%d ", j);
	fprintf(record, "%f ", cpts[j][0]);
	fprintf(record, "%f ", cpts[j][1]);
	fprintf(record, "%f ", cpts[j][2]);
      }
      fclose(record);
      printf("Control points recorded in file.\n");
    }
    break;
  case 'l':case 'L':
    record = fopen("bspline.txt", "r");
    if (record == NULL){
      printf("Warning: Could not open file to read.\n");
    } else {
      ncpts = 0;
      int i=0;
      int tempVariable=0;
      do {
	fileSuccessfullyRead1 = fscanf(record, "%d", &tempVariable);
	fileSuccessfullyRead2 = fscanf(record, "%f", &cpts[i][0]);
	fileSuccessfullyRead3 = fscanf(record, "%f", &cpts[i][1]);
	fileSuccessfullyRead4 = fscanf(record, "%f", &cpts[i][2]);
	i++;
      } while(i<MAX_CPTS &&
	      tempVariable==i &&
	      fileSuccessfullyRead1 != EOF &&
	      fileSuccessfullyRead2 != EOF &&
	      fileSuccessfullyRead3 != EOF &&
	      fileSuccessfullyRead4 != EOF );

      if (tempVariable!=i){
	printf("Error. Coordinate index invalid. Loop condition broken.\n");
	i--;
      }
      ncpts = i;
      fclose(record);
    }
    break;
  }

  display();
}

/* This routine handles window resizes */
void reshape(int w, int h){
  width = w;
  height = h;
  
  /* Set the transformations */
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glViewport(0, 0, w, h);
}

void lightingInit(){
  GLfloat mat_specular[]={1.0, 1.0, 0.0, 1.0};
  GLfloat mat_diffuse[]={0.7, 0.7, 0.0, 1.0};
  GLfloat mat_ambient[]={0.0, 0.2, 0.0, 1.0};
  GLfloat mat_shininess={100.0};

  GLfloat light_pos[] = {0.0, 0.0, -7.0, 1.0};

  glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_AUTO_NORMAL);
	
  glShadeModel(GL_SMOOTH);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
}

void main(int argc, char **argv){
  #ifdef DEBUG
  printf("%d %d %d \n", GLUT_LEFT_BUTTON, GLUT_RIGHT_BUTTON, GLUT_MIDDLE_BUTTON);
  #endif
  
  /* Intialize the program */
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB|GLUT_DEPTH|GLUT_SINGLE);
  glutInitWindowSize(width, height);
  glutCreateWindow("curves");
  
  /* Register the callbacks */
  glutDisplayFunc(display);
  glutMouseFunc(mouse);
  glutKeyboardFunc(keyboard);
  glutReshapeFunc(reshape);
  glutMotionFunc(moveObject);

  glClearColor(1.0, 1.0, 1.0, 1.0);

  glutMainLoop();
}


make:
	gcc surfaceofrevolutions.c -lglut -lGL -lGLU -lX11 -lm -L/usr/lib/X11 -o surfaceofrevolutions
debug:
	gcc surfaceofrevolutions.c -g -lglut -lGL -lGLU -lX11 -lm -L/usr/lib/X11 -DDEBUG -o surfaceofrevolutions

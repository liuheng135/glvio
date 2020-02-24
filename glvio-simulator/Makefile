PROJECT_PATH = $(shell pwd)

INCLUDE := -I$(PROJECT_PATH)
INCLUDE += -I/usr/local/Cellar 

LIB := -lopencv_video 
LIB += -lopencv_features2d 
LIB += -lopencv_core 
LIB += -lopencv_highgui 
LIB += -lopencv_imgproc 
LIB += -lopencv_imgcodecs 
LIB += -lopencv_videoio 
LIB += -lopencv_calib3d
LIB += -lopencv_viz

all:
	g++ main.cpp quaternion.c ahrs.c geometry.c image_lib.c $(INCLUDE) $(LIB) -o glvio.o

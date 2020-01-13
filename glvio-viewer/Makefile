PROJECT_PATH = $(shell pwd)

INCLUDE := -I$(PROJECT_PATH)
INCLUDE += -I/usr/local/Cellar 
#INCLUDE += -I/usr/local/Cellar/eigen/3.3.7/include/eigen3
#INCLUDE += -I/usr/local/Cellar/vtk/8.2.0_3/include/vtk-8.2

LIB := -lopencv_video 
LIB += -lopencv_features2d 
LIB += -lopencv_core 
LIB += -lopencv_highgui 
LIB += -lopencv_imgproc 
LIB += -lopencv_imgcodecs 
LIB += -lopencv_videoio 
LIB += -lopencv_calib3d
LIB += -lopencv_viz

#LIB += -lpcl_common
#LIB += -lpcl_visualization
#LIB += -lvtkCommonCore-8.2
#LIB += -lpcl_io
#LIB += -lpcl_io_ply
#LIB += -lpcl_features
#LIB += -lpcl_keypoints

all:
	g++ glvio-receiver.cpp lwlink.c udp.c quaternion.c  $(INCLUDE) $(LIB) -o glvio-viewer.o

clean:
	rm *.o

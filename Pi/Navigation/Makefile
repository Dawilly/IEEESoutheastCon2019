CPPFLAGS = -Wall -Wextra
INCFLAGS = -I/usr/local/include/opencv4/ -I/usr/local/include/opencv4/opencv2 -I/usr/local/include/raspicam
GPIOFLAGS = -pthread -lpigpio
RASPICAMLIBFLAGS = -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util
OPENCVLIBFLAGS = -lopencv_imgcodecs -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc
OBJ = Dijikstra.o Graph.o Edge.o Vertex.o Serial8N1.o camera.o
EXE = navigation_cam_test

all: $(EXE)

$(EXE): $(OBJ)
	g++ $(CPPFLAGS) main.cpp $(OBJ) -o $(EXE) $(INCFLAGS) $(GPIOFLAGS) $(RASPICAMLIBFLAGS) $(OPENCVLIBFLAGS)

test: $(EXE)
	sudo ./$(EXE) /dev/ttyS0 graph.txt waypoints.txt

%.o: %.cpp
	g++ -c $< -o $@  $(CPPFLAGS) $(INCFLAGS)

clean:
	rm -f $(EXE) *.o

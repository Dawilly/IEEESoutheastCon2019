INCFLAGS = -I/usr/local/include/opencv4/ -I/usr/local/include/opencv4/opencv2 -I/usr/local/include/raspicam
RASPICAMLIBFLAGS = -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util
OPENCVLIBFLAGS = -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc
EXE = camera_test

all: $(EXE)

test: all
	./$(EXE)

$(EXE):
	g++ camera_test.cpp -o $(EXE) $(INCFLAGS) $(RASPICAMLIBFLAGS) $(OPENCVLIBFLAGS)

clean:
	rm -f $(EXE)

########LIBRAIRIES

LIBS_ffmpeg = -lm -lz -lpthread -lavformat -lavcodec -lavutil

LIBS_opencv = -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lippicv -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core 

LIBS_autres = -lpthread -ldl -lm

#LIBS = $(LIBS_autres) $(LIBS_ffmpeg) $(LIBS_opencv)
LIBS=$(LIBS_opencv)
########CONSTANTES

CXXFLAGS = -O2 -g -Wall -fmessage-length=0 -D__STDC_CONSTANT_MACROS 
OBJS = calibration_camera.o
TARGET = calibration_camera
########REGLES

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS) 
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

%.o: %.cpp 
	$(CXX) -o $@ -c $< $(CXXFLAGS) $(LIBS)


clean: 
	rm -f $(OBJS) $(TARGET)


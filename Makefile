CXX ?= g++
CC ?= gcc
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv5)
QT_CFLAGS := $(shell pkg-config --cflags Qt6Widgets)
# opencv5.pc pulls viz, whose VTK libraries are not installed here.
OPENCV_LIBS := -lopencv_highgui -lopencv_rgbd -lopencv_stereo -lopencv_calib -lopencv_imgcodecs -lopencv_imgproc -lopencv_geometry -lopencv_core
QT_LIBS := $(shell pkg-config --libs Qt6Widgets)

CFLAGS := -O2 -Wall -Wextra -std=c11 -fPIE
CXXFLAGS := -O2 -Wall -Wextra -std=c++17 -fPIE -mno-direct-extern-access $(OPENCV_CFLAGS) $(QT_CFLAGS)
LDFLAGS := -fPIE -pie

SRC_C := src/fox_v4l2.c src/fox_calib.c
SRC_CXX := src/fox_scan.cpp src/fox_live.cpp src/fox_app.cpp src/main.cpp
OBJ := $(SRC_C:.c=.o) $(SRC_CXX:.cpp=.o)

.PHONY: all clean

all: fox3d

fox3d: $(OBJ)
	$(CXX) $(LDFLAGS) -o $@ $(OBJ) $(OPENCV_LIBS) $(QT_LIBS)

src/%.o: src/%.c src/fox_v4l2.h src/fox_calib.h
	$(CC) $(CFLAGS) -c -o $@ $<

src/%.o: src/%.cpp src/fox_scan.h src/fox_calib.h src/fox_v4l2.h src/fox_live.h src/mc_tables.inc
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ) fox3d

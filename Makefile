# THE TRANSMUTER - Modern Build System
CXX = g++
CXXFLAGS = -std=c++23 -I/usr/include/eigen3 -O3 -Wall

# Added -lGLU here to fix the 'undefined reference'
LIBS = -lGL -lGLU -lGLEW -lglfw

TARGET = transmuter

all: $(TARGET)

$(TARGET): src/main.cc
	$(CXX) $(CXXFLAGS) src/main.cc -o $(TARGET) $(LIBS)

clean:
	rm -f $(TARGET)
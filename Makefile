# THE TRANSMUTER - Project Makefile
CXX = g++
CXXFLAGS = -std=c++23 -O3 -Wall \
           -I/usr/include/eigen3 \
           -I./sw/core/inc \
           -I./sw/geometry/inc \
           -I./sw/pathgen/inc \
           -I./sw/visualization/inc

LIBS = -lGL -lGLU -lGLEW -lglfw

# Source files
SRCS = sw/main.cc \
       $(wildcard sw/core/src/*.cc) \
       $(wildcard sw/geometry/src/*.cc) \
       $(wildcard sw/pathgen/src/*.cc) \
       $(wildcard sw/visualization/src/*.cc)

OBJS = $(SRCS:.cc=.o)
TARGET = Transmuter

# Default target
all: docs $(TARGET)

# Link the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LIBS)

# Compile source files
%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Documentation target
docs:
	@echo "Updating project documentation..."
	@chmod +x docs/generate_doc_tree.sh
	@./docs/generate_doc_tree.sh

# Cleanup
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean docs
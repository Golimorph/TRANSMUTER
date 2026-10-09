# THE TRANSMUTER - Project Makefile
CXX = g++

# Include paths for all modules
INC = -I/usr/include/eigen3 \
      -I./sw/controller/if \
      -I./sw/controller/inc \
	  -I./sw/controller/programs/if \
      -I./sw/controller/programs/inc \
      -I./sw/geometry/if \
      -I./sw/geometry/pointCloud/inc \
      -I./sw/geometry/shapes/analyticShapes/inc \
      -I./sw/geometry/shapes/pointCloudShapes/inc \
      -I./sw/pathGeneration/if \
      -I./sw/pathGeneration/inc \
      -I./sw/pathGeneration/dynamics/if \
      -I./sw/pathGeneration/dynamics/inc \
      -I./sw/pathGeneration/dynamics/toolForce/if \
      -I./sw/pathGeneration/dynamics/toolForce/inc \
	  -I./sw/pathGeneration/pathAlgorithms/if \
	  -I./sw/pathGeneration/pathAlgorithms/threeAxisRasterAlgorithm/inc \
	  -I./sw/pathGeneration/pathAlgorithms/threeAxisSpiralAlgorithm/inc \
      -I./sw/pathGeneration/pathAlgorithms/fiveAxisNormalAlgorithm/inc \
      -I./sw/pathGeneration/pathAlgorithms/fiveAxisNormalAlgorithm/geometry/inc \
      -I./sw/visualization/if \
      -I./sw/visualization/inc \
      -I./sw/visualization/playback/inc \
      -I./sw/visualization/machineVisualization/if \
      -I./sw/visualization/machineVisualization/inc \
	  -I./sw/utils/inc

# Source files
SRCS = sw/main.cc \
       $(wildcard sw/controller/src/*.cc) \
       $(wildcard sw/controller/programs/src/*.cc) \
       $(wildcard sw/geometry/src/*.cc) \
	   $(wildcard sw/geometry/pointCloud/src/*.cc) \
	   $(wildcard sw/geometry/shapes/analyticShapes/src/*.cc) \
	   $(wildcard sw/geometry/shapes/pointCloudShapes/src/*.cc) \
       $(wildcard sw/pathGeneration/src/*.cc) \
       $(wildcard sw/pathGeneration/dynamics/src/*.cc) \
       $(wildcard sw/pathGeneration/dynamics/toolForce/src/*.cc) \
       $(wildcard sw/pathGeneration/pathAlgorithms/src/*.cc) \
       $(wildcard sw/visualization/machineVisualization/src/*.cc) \
	   $(wildcard sw/pathGeneration/pathAlgorithms/*/src/*.cc) \
       $(wildcard sw/pathGeneration/pathAlgorithms/*/*/src/*.cc) \
       $(wildcard sw/visualization/playback/src/*.cc) \
       $(wildcard sw/visualization/src/*.cc)

OBJS = $(SRCS:.cc=.o)
DEPS = $(OBJS:.o=.d)

#Libraries
LIBS = -lGL -lGLU -lGLEW -lglfw

#Compilation flags
CXXFLAGS = -std=c++23 -O3 -Wall -fno-exceptions $(INC)
DEPFLAGS = -MMD -MP

# Executable name
TARGET = transmuter

# By default, build the executable, format the code, and update documentation, see format and docs targets below
all: format docs $(TARGET)

# Link the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LIBS)

# Compile source files
$(OBJS): Makefile

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) -c $< -o $@

-include $(DEPS)

# Formatting target: Enforces Allman style via .clang-format
format:
	@echo "Formatting source files to Allman style."
	@find sw -iname "*.h" -o -iname "*.cc" | xargs clang-format -i

# Documentation target
docs:
	@echo "Updating project documentation."
	@chmod +x docs/generate_doc_tree.sh
	@./docs/generate_doc_tree.sh

run: $(TARGET)
	./$(TARGET) quickTorusMilling

# Cleanup
clean:
	rm -f $(OBJS) $(DEPS) $(TARGET)

.PHONY: all clean docs format

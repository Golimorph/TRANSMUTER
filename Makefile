# THE TRANSMUTER - Project Makefile
CXX = g++

# Include paths for all modules
INC = -I/usr/include/eigen3 \
      -I./sw/core/inc \
      -I./sw/geometry/inc \
      -I./sw/pathgen/inc \
      -I./sw/rendering/inc \
      -I./sw/visualization/inc

# Source files
SRCS = sw/main.cc \
       $(wildcard sw/core/src/*.cc) \
       $(wildcard sw/geometry/src/*.cc) \
       $(wildcard sw/pathgen/src/*.cc) \
       $(wildcard sw/visualization/src/*.cc)

OBJS = $(SRCS:.cc=.o)

#Libraries
LIBS = -lGL -lGLU -lGLEW -lglfw

#Compilation flags
CXXFLAGS = -std=c++23 -O3 -Wall $(INC)

# Executable name
TARGET = transmuter

# By default, build the executable, format the code, and update documentation, see format and docs targets below
all: format docs $(TARGET)

# Link the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LIBS)

# Compile source files
%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Formatting target: Enforces Allman style via .clang-format
format:
	@echo "Formatting source files to Allman style."
	@find sw -iname "*.h" -o -iname "*.cc" | xargs clang-format -i

# Documentation target
docs:
	@echo "Updating project documentation."
	@chmod +x docs/generate_doc_tree.sh
	@./docs/generate_doc_tree.sh

# Cleanup
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean docs format
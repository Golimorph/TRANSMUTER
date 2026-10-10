# THE TRANSMUTER - Project Makefile
CXX = g++
SHELL := /bin/bash

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

# By default, format the code, update documentation, and build the executable.
all: format docs
	@$(MAKE) --no-print-directory build-with-progress

# Count the work Make actually schedules, then run it with one progress display.
# Compiler and linker output is captured by their recipes and printed on failure.
build-with-progress:
	@echo "Building..."
	@set -u; \
	total=$$($(MAKE) --no-print-directory -n $(TARGET) | awk '/ -c / || / -o $(TARGET)( |$$)/ { count++ } END { print count + 0 }'); \
	if [ "$$total" -eq 0 ]; then \
		output=$$($(MAKE) --no-print-directory $(TARGET) 2>&1); status=$$?; \
		if [ "$$status" -ne 0 ]; then printf '%s\n' "$$output"; fi; \
		exit "$$status"; \
	fi; \
	progress_dir=$$(mktemp -d "$${TMPDIR:-/tmp}/transmuter-build-progress.XXXXXX"); \
	progress_pipe="$$progress_dir/completed"; \
	mkfifo "$$progress_pipe"; \
	trap 'rm -rf "$$progress_dir"' EXIT; \
	exec 9<>"$$progress_pipe"; \
	draw_progress() { \
		local percent=$$((completed * 100 / total)); \
		local filled=$$((percent / 5)); \
		local bar=$$(printf '%*s' "$$filled" '' | tr ' ' '#'); \
		bar="$$bar$$(printf '%*s' "$$((20 - filled))" '' | tr ' ' '.')"; \
		local spin='|/-\\'; \
		local marker=$${spin:$$((spinner_index % 4)):1}; \
		spinner_index=$$((spinner_index + 1)); \
		if [ -t 1 ]; then \
			printf '\r\033[K[%s] %3d%% [%s] %d/%d build steps' "$$marker" "$$percent" "$$bar" "$$completed" "$$total"; \
		elif [ "$$completed" -ne "$$last_printed" ]; then \
			printf 'Build progress: %3d%% [%s] %d/%d build steps\n' "$$percent" "$$bar" "$$completed" "$$total"; \
			last_printed=$$completed; \
		fi; \
	}; \
	progress_loop() { \
		local completed=0 spinner_index=0 last_printed=-1 event; \
		while true; do \
			if IFS= read -r -t 0.1 -n 1 event; then \
				if [ "$$event" = e ] || [ "$$event" = f ]; then break; fi; \
				completed=$$((completed + 1)); \
			fi; \
			draw_progress; \
		done; \
		if [ "$$event" = e ]; then completed=$$total; draw_progress; fi; \
		printf '\n'; \
	}; \
	progress_loop <&9 & progress_pid=$$!; \
	output=$$(BUILD_PROGRESS_PIPE="$$progress_pipe" $(MAKE) --no-print-directory $(TARGET) 2>&1); status=$$?; \
	if [ "$$status" -eq 0 ]; then printf e > "$$progress_pipe"; else printf f > "$$progress_pipe"; fi; \
	wait "$$progress_pid"; \
	if [ "$$status" -ne 0 ]; then printf '%s\n' "$$output"; exit "$$status"; fi; \
	:

.PHONY: build-with-progress

# Link the executable
$(TARGET): $(OBJS)
	@$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LIBS) || exit $$?; \
	if [ -n "$${BUILD_PROGRESS_PIPE:-}" ]; then printf x > "$$BUILD_PROGRESS_PIPE"; fi

# Compile source files
$(OBJS): Makefile

%.o: %.cc
	@$(CXX) $(CXXFLAGS) $(DEPFLAGS) -c $< -o $@ || exit $$?; \
	if [ -n "$${BUILD_PROGRESS_PIPE:-}" ]; then printf x > "$$BUILD_PROGRESS_PIPE"; fi

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

run: format docs
	@$(MAKE) --no-print-directory build-with-progress
	@./$(TARGET) interactiveTorusMilling

# Cleanup
clean:
	rm -f $(OBJS) $(DEPS) $(TARGET)

.PHONY: all build-with-progress clean docs format run

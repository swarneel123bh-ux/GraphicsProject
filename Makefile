# Build the live window app (live).
# Usage (from this folder):
#   make            build build/live
#   make live       same as make
#   make run-live   build and run live
#   make clean      delete build/

CC     := cc
CFLAGS := -std=c11 -O2 -Wall -Wno-deprecated-declarations -pthread
BUILD  := build

LIVE_SRC := scratch_raytracer/live.c

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
GLFW_PREFIX ?= $(shell brew --prefix glfw 2>/dev/null || echo /opt/homebrew)
LIVE_INC := -I$(GLFW_PREFIX)/include
LIVE_LIB := -L$(GLFW_PREFIX)/lib -lglfw -framework OpenGL
else
LIVE_INC :=
LIVE_LIB := -lglfw -lGL
endif

.PHONY: all live run-live clean

all: live

live: $(BUILD)/live

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/live: $(LIVE_SRC) | $(BUILD)
	$(CC) $(CFLAGS) $(LIVE_INC) -o $@ $< $(LIVE_LIB) -lm

run-live: live
	./$(BUILD)/live

clean:
	rm -rf $(BUILD)

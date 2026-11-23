# Build the live window app with GLFW or FreeGLUT, whichever is installed.
# Usage (from this folder):
#   make                    build build/live-<backend> (GLFW is preferred if both are found)
#   make BACKEND=glfw       force GLFW
#   make BACKEND=freeglut   force FreeGLUT
#   make run-live           build and run
#   make clean              delete build/
#
# Install one of them:
#   macOS:    brew install glfw            or   brew install freeglut
#   Linux:    sudo apt install build-essential libglfw3-dev libgl-dev
#             sudo apt install build-essential freeglut3-dev libgl-dev
#   Windows:  in the MSYS2 MINGW64 shell, run make from there (not cmd or PowerShell):
#             pacman -S make mingw-w64-x86_64-gcc mingw-w64-x86_64-glfw
#             pacman -S make mingw-w64-x86_64-gcc mingw-w64-x86_64-freeglut

CFLAGS   := -std=c11 -O2 -Wall -Wno-deprecated-declarations -pthread
BUILD    := build
LIVE_SRC := scratch_raytracer/live.c
UNAME_S  := $(shell uname -s)

ifeq ($(OS),Windows_NT)
CC       := gcc
EXE      := .exe
PREFIX   := $(or $(MSYSTEM_PREFIX),/mingw64)
GLFW_HDR := $(PREFIX)/include/GLFW/glfw3.h
GLFW_INC := -I$(PREFIX)/include
GLFW_NAME := $(if $(wildcard $(PREFIX)/lib/libglfw.dll.a $(PREFIX)/lib/libglfw.a),glfw,glfw3)
GLFW_LIB := -L$(PREFIX)/lib -l$(GLFW_NAME) -lopengl32 -lgdi32
FG_HDR   := $(PREFIX)/include/GL/freeglut.h
FG_INC   := -I$(PREFIX)/include
FG_NAME  := $(if $(wildcard $(PREFIX)/lib/libglut.dll.a $(PREFIX)/lib/libglut.a),glut,freeglut)
FG_LIB   := -L$(PREFIX)/lib -l$(FG_NAME) -lopengl32 -lglu32 -lwinmm -lgdi32
else ifeq ($(UNAME_S),Darwin)
EXE      :=
CC       := cc
BREW     := $(or $(shell brew --prefix 2>/dev/null),/opt/homebrew)
GLFW_HDR := $(BREW)/opt/glfw/include/GLFW/glfw3.h
GLFW_INC := -I$(BREW)/opt/glfw/include
GLFW_LIB := -L$(BREW)/opt/glfw/lib -lglfw -framework OpenGL
FG_HDR   := $(BREW)/opt/freeglut/include/GL/freeglut.h
FG_INC   := -I$(BREW)/opt/freeglut/include
FG_LIB   := -L$(BREW)/opt/freeglut/lib -lglut -framework OpenGL
else
EXE      :=
CC       := cc
GLFW_HDR := $(firstword $(wildcard /usr/include/GLFW/glfw3.h /usr/local/include/GLFW/glfw3.h))
GLFW_INC :=
GLFW_LIB := -lglfw -lGL
FG_HDR   := $(firstword $(wildcard /usr/include/GL/freeglut.h /usr/local/include/GL/freeglut.h))
FG_INC   :=
FG_LIB   := -lglut -lGL
endif

HAVE_GLFW := $(wildcard $(GLFW_HDR))
HAVE_FG   := $(wildcard $(FG_HDR))

ifeq ($(BACKEND),)
  ifneq ($(HAVE_GLFW),)
    BACKEND := glfw
  else ifneq ($(HAVE_FG),)
    BACKEND := freeglut
  else
    $(error Neither GLFW nor FreeGLUT was found. Install one (see the top of this Makefile))
  endif
endif

ifeq ($(BACKEND),glfw)
ifeq ($(HAVE_GLFW),)
  $(error BACKEND=glfw, but GLFW was not found: $(GLFW_HDR))
endif
BK_DEF := -DUSE_GLFW
BK_INC := $(GLFW_INC)
BK_LIB := $(GLFW_LIB)
else ifeq ($(BACKEND),freeglut)
ifeq ($(HAVE_FG),)
  $(error BACKEND=freeglut, but FreeGLUT was not found: $(FG_HDR))
endif
BK_DEF := -DUSE_FREEGLUT
BK_INC := $(FG_INC)
BK_LIB := $(FG_LIB)
else
  $(error BACKEND must be glfw or freeglut, not '$(BACKEND)')
endif

$(info Window backend: $(BACKEND))
LIVE := $(BUILD)/live-$(BACKEND)$(EXE)

.PHONY: all live run-live clean

all: live

live: $(LIVE)

$(BUILD):
	mkdir -p $(BUILD)

$(LIVE): $(LIVE_SRC) | $(BUILD)
	$(CC) $(CFLAGS) $(BK_DEF) $(BK_INC) -o $@ $< $(BK_LIB) -lm

run-live: live
	./$(LIVE)

clean:
	rm -rf $(BUILD)

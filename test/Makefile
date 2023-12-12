# # Makefile for ThreadedSolarSystem

# # *****************************************************
# # Variables to control Makefile operation

# CXX = g++
# CXX = nvc++
# RM = rm -f -r

# # Directory this example is contained in
# MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
# DIR := $(notdir $(patsubst %/,%,$(dir $(MKFILE_PATH))))
# UNAME    := $(shell uname)

# # Dependencies
# _DEPS = \

# # Main source file
# TARGET = testThreadedSolarSystem
# TARGET = accTestThreadedSolarSystem

# # Object files
# ODIR = obj
# _OBJ = $(TARGET).o
# OBJ = $(patsubst %, $(ODIR)/%, $(_OBJ))

# # To create obj directory
# dummy_build_folder := $(shell mkdir -p $(ODIR))

# # Flags
# NOWARN = -Wno-unused-parameter -Wno-unused-function -Wno-narrowing \
# 			-Wno-sizeof-array-argument -Wno-sign-compare -Wno-unused-variable

# ifeq ($(UNAME), Linux)
# GL_FLAGS := -lGLU  -lGL
# BREW :=
# endif
# ifeq ($(UNAME), CYGWIN_NT-10.0)
# GL_FLAGS := -lGLU  -lGL
# BREW :=
# endif
# ifeq ($(UNAME), Darwin)
# #GL_FLAGS := -framework OpenGL 
# #BREW := -lomp -I"$(brew --prefix libomp)/include" 
# endif

# CXXFLAGS =  \
#         -I$(TSGL_HOME)/include/TSGL \
#         -I$(TSGL_HOME)/include/freetype2 \
# 		-fopenmp -fast -acc=multicore \

# LFLAGS = -ltsgl -lfreetype -lGLEW -lglfw $(GL_FLAGS) -fopenmp  \
# 			-L$(TSGL_HOME)/lib -fast -acc=multicore \

# NVCFLAGS = 	-fast -acc \
# 			-I$(TSGL_HOME)/include/TSGL \
# 			-I$(TSGL_HOME)/include/freetype2

# HPC Compiler Reference Manual
# https://docs.nvidia.com/hpc-sdk/compilers/hpc-compilers-ref-guide/index.html

INCLUDE_FLAGS = -I$(TSGL_HOME)/include/TSGL \
				-I$(TSGL_HOME)/include/freetype2
LIBRARY_FLAGS = -ltsgl -lfreetype -lGLEW -lglfw -lGL -lGLU

ACC_MULTICORE = -mp -acc=multicore -Minfo=opt

ACC_GPU = -acc=gpu -gpu=managed -Minfo=accel

GCC_OPT = -Og
OMP = -fopenmp
NVC_OPT = -fast

# Accept only ISO conforming C++ code
ISO_CONFORMING = -a 

# Prints warnings from included system header files
PEDANTIC_WARNING = --pedantic

# instructs the compiler to display all error messages (inform, warn, severe and fatal).
INFORM_WARNINGS = -Minform=inform

WARNINGS = $(ISO_CONFORMING) $(PEDANTIC_WARNING) $(INFORM_WARNINGS)

TARGET = test
	
$(TARGET)MP: $(TARGET).cpp
	nvc++ $(NVC_OPT) $(LIBRARY_FLAGS) $(INCLUDE_FLAGS) $(OMP) $(TARGET).cpp -o $(TARGET)MP $(ACC_MULTICORE) $(WARNINGS)

$(TARGET)GPU: $(TARGET).cpp
	nvc++ $(NVC_OPT) $(LIBRARY_FLAGS) $(INCLUDE_FLAGS) $(TARGET).cpp -o $(TARGET)GPU $(ACC_GPU) -DGPU

all:
	$(TARGET)MP $(TARGET)GPU

clean:
	rm -rf $(TARGET)MP $(TARGET)GPU



# # ****************************************************
# # Targets needed to bring the executable up to date

# all: $(TARGET)

# $(ODIR)/%.o: %.cpp $(_DEPS)
# 	$(CXX) -c -o $@ $< $(CXXFLAGS) $(LFLAGS)

# $(TARGET): $(OBJ)
# 	$(CXX) -o $@ $^ $(LFLAGS)

# .PHONY: clean

# clean:
# 	$(RM) $(ODIR)/*.o $(ODIR) $(TARGET)
# 	@echo ""
# 	@tput setaf 5;
# 	@echo "*************** All output files removed from $(DIR)! ***************"
# 	@tput sgr0;
# 	@echo ""

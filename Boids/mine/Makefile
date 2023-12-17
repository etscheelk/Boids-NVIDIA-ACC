#######################################################################

# Copyright (c) 1998 Gary William Flake -- Permission granted for any
# use provied that the author's comments are neither modified nor
# removed. No warranty is given or implied.

# Makefile for example code -- Requires GNU make.

# Undefine the line below only if you want the Linux SVGA graphics
# driver builtin.  Note that you will have to make each program
# set-UID root for this to work.

#VGA      = 1
X11      = 1
XINCLUDE = /usr/X11/include
XLIBS    = /usr/X11/lib

COPTS    = -O3  -Wall  -D__dest_os=unix -fopenmp    # LS added openmp here
CC       = gcc

PGCC     = nvc++
# initial accelerator options
AOPTS= -fast -ta=tesla:cc75 -Minfo=accel

# accelerator options for managed memory
AOPTSM= -fast -ta=tesla:cc75,managed -Minfo=all

OMP = -fopenmp 
#######################################################################

# Do not edit this section

#vpath %.c ../src

# ifdef VGA
# ifdef X11
# # VGA and X11
# PLOTOBJS  = vgaplot.o x11plot.o x11cplot.o pgmplot.o psplot.o rawplot.o
# PLOTFLAGS = -DPLOTX11 -DPLOTVGA
# LIBS      = -lmisc -lm -lX11 -lvga
# else
# # VGA and !X11
# PLOTOBJS  = vgaplot.o pgmplot.o psplot.o rawplot.o
# PLOTFLAGS = -DPLOTVGA
# LIBS      = -lmisc -lm -lvga
# endif
# else
# ifdef X11
# !VGA and X11

ifdef X11
PLOTOBJS  = x11plot.o x11cplot.o pgmplot.o psplot.o rawplot.o
PLOTFLAGS = -DPLOTX11
LIBS      = -lmisc -lm -lX11
else
# !VGA and !X11
PLOTOBJS  = pgmplot.o psplot.o rawplot.o
PLOTFLAGS =
LIBS      = -lmisc -lm
endif
# endif

CFLAGS   = $(COPTS) -I$(XINCLUDE) $(PLOTFLAGS)
LDFLAGS  = -L. -L$(XLIBS)

# include ../Makefile.inc

# PROGS    = $(DEMOS)
PROGS = boids
ACC_VERSION = boids_acc
ACC_VERSION_GPU = boids_acc_gpu

#######################################################################

default: $(PROGS) 

$(PROGS): % : %.o libmisc.a
	$(CC) -o $@ $< $(LDFLAGS) $(LIBS) $(OMP)

$(ACC_VERSION): boids_acc.c libmisc.a
	$(PGCC) $(AOPTSM) -o $(ACC_VERSION) boids_acc.c $(LDFLAGS) $(LIBS) $(OMP)

$(ACC_VERSION): boids_acc.c libmisc.a
	$(PGCC) $(AOPTSM) -o $(ACC_VERSION) boids_acc.c $(LDFLAGS) $(LIBS) $(OMP)

$(ACC_VERSION_GPU): boids_acc_gpu.c libmisc.a
	nvc -acc=gpu -gpu=managed -Minfo=accel -fast -o $(ACC_VERSION_GPU) boids_acc_gpu.c $(LDFLAGS) $(LIBS) $(OMP)




misc: misc.c
	nvc -c -fast misc.c -o misc.o

boidsMP: boids.cpp misc
	nvc++ -c -fast -fopenmp -mp -acc=multicore -Minfo=opt boids.cpp misc.o -o boidsMP.o

boidsGPU: boids.cpp misc
	nvc++ -c -fast -acc=gpu -gpu=managed -Minfo=accel boids.cpp misc.o -o boidsGPU.o -DGPU


tsglBoidsMP: tsglBoids.cpp boidsMP misc
	nvc++ -fast tsglBoids.cpp boidsMP.o misc.o -I$(TSGL_HOME)/include/TSGL -I$(TSGL_HOME)/include/freetype2 -ltsgl -lfreetype -lGLEW -lglfw -lGL -lGLU -o tsglBoidsMP -fopenmp -mp -acc=multicore -Minfo=opt -g

tsglBoidsGPU: tsglBoids.cpp boidsGPU misc
	nvc++ -fast tsglBoids.cpp boidsGPU.o misc.o -I$(TSGL_HOME)/include/TSGL -I$(TSGL_HOME)/include/freetype2 -ltsgl -lfreetype -lGLEW -lglfw -lGL -lGLU -o tsglBoidsGPU -acc=gpu -Minfo=accel -DGPU -g


test: boids.cpp
	nvc++ -fast tsglBoids.cpp boids.o misc.o -I$(TSGL_HOME)/include/TSGL -I$(TSGL_HOME)/include/freetype2 -ltsgl -lfreetype -lGLEW -lglfw -lGL -lGLU 


libmisc.a: misc.o plot.o $(PLOTOBJS)
	ar cr $@ $^
	ranlib $@

# bifur1d.o phase1d.o: maps1d.c

clean:
	rm -f $(PROGS) *.a *.o

#######################################################################

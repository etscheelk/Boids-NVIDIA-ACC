
misc: misc.c
	nvc -c -fast misc.c -o misc.o

boidsMP: boids.cpp misc
	nvc++ -c -fast -fopenmp -mp -acc=multicore -Minfo=opt boids.cpp misc.o -o boidsMP.o

boidsGPU: boids.cpp misc
	nvc++ -c -fast -acc=gpu -gpu=managed -Minfo=accel boids.cpp misc.o -o boidsGPU.o -DGPU


fboidsMP: noTSGLBoids.cpp boidsMP misc
	nvc++ -fast noTSGLBoids.cpp boidsMP.o misc.o -o fboidsMP -fopenmp -mp -acc=multicore -Minfo=opt

fboidsGPU: noTSGLBoids.cpp boidsGPU misc
	nvc++ -fast noTSGLBoids.cpp boidsGPU.o misc.o -o fboidsGPU -acc=gpu -Minfo=accel -DGPU



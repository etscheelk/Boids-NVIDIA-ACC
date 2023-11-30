# Boids-NVIDIA-ACC

Boids visualization using Thread Safe Graphics Library [(TSGL)](https://github.com/Calvin-CS/TSGL) from Calvin College. This is an API for OpenGL and is required for building.

Written for OpenACC with the intent to test multicore CPU as well as GPU threading for higher boid counts.

## Boids

Boids are a form of simulated bird, coined by Craig Reynolds in 1986. [Click for Craig's paper.](http://www.cs.toronto.edu/~dt/siggraph97-course/cwr87/) [Click for the Boids wikipedia article.](https://en.wikipedia.org/wiki/Boids)

## Dev Environment

I worked using WSL2 Ubuntu environment for the entirety of this project. 

## Dependencies

- [TSGL](https://github.com/Calvin-CS/TSGL)
- NVIDIA HPC ACC
- OpenMP
- X Window
CXXFLAGS=-std=c++11

all: raytracer

raytracer: main.o renderer.o
	c++ -o raytracer main.o renderer.o $(CXXFLAGS)

clean:
	rm -f *.o raytracer

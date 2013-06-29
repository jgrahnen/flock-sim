CC=g++
CFLAGS=-c -g -std=c++0x -Wall -Wextra -Werror
OBJDIR=obj/
VPATH=src/:src/geometry.:src/sdl/:$(OBJDIR)
LIBS=SDL geometry
LIBDIR=src/geometry/

all-objects = flock.o Boid.o sdl-wrapper.o

all: libgeometry.a $(all-objects)
	$(CC) -o flocking $(addprefix $(OBJDIR), $(all-objects)) -L${LIBDIR} $(addprefix -l, $(LIBS))

libgeometry.a:
	cd src/geometry && make

flock.o: flock.cpp Boid.h
	$(CC) $(CFLAGS) $< -o $(OBJDIR)$@

Boid.o: Boid.cpp Boid.h
	$(CC) $(CFLAGS) $< -o $(OBJDIR)$@

sdl-wrapper.o: sdl-wrapper.cpp sdl-wrapper.h
	$(CC) $(CFLAGS) $< -o $(OBJDIR)$@

doc:
	doxygen Doxyfile

clean:
	rm -rf $(OBJDIR)*.o
	rm -f src/geometry/*.a
	rm -f src/geometry/*.o

.PHONY: doc clean

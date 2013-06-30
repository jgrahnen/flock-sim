Flocking simulator
==================

Simulates "Boids" that move in a flock-like manner toward your mouse pointer. 
This is a rough implementation of the simulation algorithm for reproducing the
behavior of a flock first outlined in Reynolds (1987, Computer Graphics 21:4).

This implementation is actually more akin to simulating the swimming behavior 
of a school of fish than a flock of birds. The most noticeable changes are the
absence of any 3D elements, the consequent absence of flight mechanics, and
the representation of the medium surrounding the Boids as more water-like
than air-like.

Author: Johan Grahnen (johan.grahnen@gmail.com)
Copyright: MIT license (see LICENSE)
Tested on: Linux Mint 14 (nadia), 64-bit, kernel v 3.5.0-17.

Dependencies
------------

Requires Simple DirectMedia Layer (SDL) v 1.2 or higher (developer version)
to be installed, with SDL.h available in your include path.

For ease of compilation, use gcc v 4.7 or higher. Generally speaking, any
version of gcc that supports the C++0X standard should work.

Installation
------------

Download and uncompress the archive, and type

    make

in the project root directory to compile and link the executable.

To get rid of intermediate object files:

    make clean

after compilation and linking.

To generate HTML documentation, make sure you have Doxygen installed and in
your PATH. Then

    make doc

and take a look at doc/html/index.html.

Only tested on Linux Mint 14. Could compile on other OSes -- who knows? Best of
luck!

Usage
-----

The simulation consists of a flock of Boids migrating around the playing
window, following your mouse pointer. The behavior of the Boids can be altered
by adjusting the simulation parameters: increasing the coefficient of
cohesion makes them stick tighter together, decreasing the coefficient of
attraction causes them to care less about the location of the mouse pointer,
and so on. See below for an extended explanation of the parameters.

When you're tired of staring at the Boids progress across the screen, simply
close the window to quit.

Launch the simulation with default parameter values by typing

    cat default.params | xargs ./flocking

and watch them flit about. This combination of parameters should provide a
reasonable compromise between the various components of Boid behavior. Users
with more recent systems may wish to scale all the parameters down, since
there is no limit on frame rate and the simulation may move too fast. On the 
other hand, a more powerful machine will work great for increasing the number 
of Boids.

There are 5 parameters that control the simulation:

1. **Number of Boids**. Default setting is 100. The simulation algorithm
   scales _extremely_ poorly (O(N^3) in the number of Boids), so large settings
   may not work that well.

2. **Cohesion coefficient.** Default setting is 0.005. Controls the degree to
   which the Boids are attracted to each other. Higher values force the Boids
   to behave as one large flock, lower values result in many smaller flocks.

3. **Separation coefficient.** Default setting is 0.2. Controls the degree to
   which the Boids are repulsed by one another. High values produce sparser
   flock patterns, very low values can cause Boids to overlap. Also functions
   as an ersatz collision detection system.

4. **Alignment coefficient.** Default setting is 0.05. Controls the degree to
   which the Boids tend to move in the same direction. High values result in
   flocks that are highly cohesive and have a lot inertia (turn slowly).

5. **Attraction coefficient.** Default setting is 1.0. Controls how interested
   the Boids are in following the mouse pointer. High values cause very close
   tracking of the cursor, although the Boids cannot perceive the position of
   the cursor very well at a distance and the effect is mostly noticeable
   at close range.

Open issues
-----------

Consult the TODO file for suggestions for new features and known problems.

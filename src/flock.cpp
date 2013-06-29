/**
 * \file flock.cpp
 *
 * Driver program to run a flocking simulation. Generates starting conditions,
 * manages the simulations, draws results to screen and takes user input.
 *
 * @author	Johan Grahnen
 * @since	2013-06-29
 */

/**
 * Definitions.
 */
#define PI 3.14159265
#define WRAPPED false // Should Boids wrap around the edge of the playing field?

/**
 * Includes.
 */
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <algorithm>
#include <stdexcept>
#include "Boid.h"
#include "sdl/sdl-wrapper.h"

using namespace std;

/**
 * Generates (crappy) random whole integers on some interval.
 *
 * Doesn't actually generate numbers that are uniformly distributed on the 
 * interval, but reasonably close.
 * 
 * @param start	Start of the interval.
 * @param end	End of the interval.
 * @return	Floating-point representation of the integer.
 */
float badRandom(int start, int end){
	return (float) (start + rand() % (end-start));
}

/**
 * Find the closest animation sprite for a rotating moving object.
 *
 * Determines the closest animation frame to a particular direction of travel. 
 * Assumes that the frames are ordered from 0 to N-1, rotated  
 * counter-clockwise from the X-axis.
 *
 * @param velocity	Direction in which the object is moving.
 * @param numFrames	The number of frames in the animated sprite.
 */
unsigned int closestFrame(Vector velocity, unsigned int numFrames){
	/* Calculate the angle to the X-axis.
	 *
	 * The coordinate system on screen is left-handed, not the standard
	 * right-handedness, so a 90 degree rotation needs to be applied.
	 */
	float angle = -90.0 + (180.0/PI)*atan2(velocity.x, velocity.y);

	/* Convert to the frame with closest rotations, assuming they're
	 * ordred counter-clockwise.
	 *
	 * Note that atan2() delivers signed angles on [-pi,pi], not
	 * unsigned ones on [0,2*pi], so it needs some transformation.
	 */
	float degreesPerFrame = 360.0 / (float) numFrames;
	unsigned int frame = angle >= 0.0 ? (unsigned int) floor(angle/degreesPerFrame) : (unsigned int) floor((angle+360.0)/degreesPerFrame);

	return frame;
}

/**
 * Entry point.
 *
 * Handles user input, simulation time stepping, draws results to screen.
 *
 * @see		Boid.cpp
 * @see		SDL.h
 */
int main(int argc, char* argv[]){
	string usage = " [# of boids] [cohesion param] [separation param] [alignment param] [attraction param]";

	/* Check for arguments.
	 */
	if(argc < 6){
		cerr << "Usage: " << argv[0] << usage << endl;
		exit(1);
	}

	/* Read arguments.
	 */
	unsigned int numBoids = atoi(argv[1]);
	float cohesionCoeff = atof(argv[2]);
	float separationCoeff = atof(argv[3]);
	float alignmentCoeff = atof(argv[4]);
	float attractionCoeff = atof(argv[5]);

	/* Setup the drawing area and load graphics.
	 */
	const unsigned int screenWidth = 1200;
	const unsigned int screenHeight = 700;
	const pair<int, int> screenCenter(screenWidth/2, screenHeight/2);
	const unsigned int boidHeight = 20;
	const unsigned int boidWidth = 20;
	const unsigned int numAnimFrames = 12;
	pair<int,int> screenLimits(screenWidth, screenHeight);
	const char* birdIconFile = "gfx/red-arrow-rot-12x.bmp";

	SDL_Surface* screen = initializeDisplay(screenWidth, screenHeight);
	if(!screen) cleanUpAndQuit();

	SDL_Surface* birdIcons = loadBMPImage(birdIconFile);
	if(!birdIcons) cleanUpAndQuit();
	if(!transparentize(birdIcons, 255, 0, 255)) cleanUpAndQuit();

	/* Instantiate a population of boids with random coordinates, 
	 * initially moving outwards from the center of the screen.
	 */
	srand(time(NULL));
	vector<Boid> pop;
	for(unsigned int i = 0; i < numBoids; i++){
		/* Start a little ways away from the middle of the
		 * box, and head outwards.
		 */
		float x = badRandom(screenCenter.first - 100, screenCenter.first + 100);
		float y = badRandom(screenCenter.second - 100, screenCenter.second + 100);
		Point coordinates(x, y);
		Vector velocity(copysign(3.0, x-screenCenter.first), copysign(3.0, y-screenCenter.second));
		pop.push_back(Boid(coordinates, velocity, cohesionCoeff, separationCoeff, alignmentCoeff, attractionCoeff, Point(screenLimits.first, screenLimits.second)));
	}

	/* Run simulation and display results until the user gets sick of it.
	 */
	vector<Boid> newPop;
	SDL_Event event;
	Point mousePos(screenCenter.first, screenCenter.second);
	bool running = true;
	while(running){
		/* Advance the simulation one step.
		 */
		newPop.clear();
		for(unsigned int i = 0; i < pop.size(); i++){
			/* Only consider the coordinates of the rest of the
			 * flock, not yourself.
			 */
			vector<Boid> allOthers(pop);
			allOthers.erase(allOthers.begin() + i);

			 /*
			 * Wrap-around the screen as necessary, or deal with
			 * errors that can occur during collision with the
			 * edge of the simulation world.
			 */
			if(WRAPPED){
				newPop.push_back(pop[i].wrappedStep(allOthers, mousePos, screenLimits.first, screenLimits.second));
			}
			else{
				try{
					newPop.push_back(pop[i].step(allOthers, mousePos));
				}
				catch(domain_error& e){
					/* Just cheat and replace the boid
					 * at some random valid position near
					 * the center of the screen.
					 */
					float x = badRandom(screenCenter.first - 100, screenCenter.first + 100);
					float y = badRandom(screenCenter.second - 100, screenCenter.second + 100);
					newPop.push_back(Boid(Point(x,y), Vector(0.0, 0.0), cohesionCoeff, separationCoeff, alignmentCoeff, attractionCoeff, Point(screenLimits.first, screenLimits.second)));
				}	
			}
		}
		pop = newPop;
		
		/* Setup drawing for the next frame.
		 */
		SDL_FillRect(screen, NULL, SDL_MapRGB(screen->format, 0, 0, 0));

		/* Draw the new population.
		 */
		for(unsigned int i = 0; i < pop.size(); i++){
			Point coordinates = pop[i].getCoordinates();
			Vector velocity = pop[i].getVelocity();

			/* Draw part of the animation sprite.
			 */
			unsigned int frameNum = closestFrame(velocity, numAnimFrames);
			unsigned int x = coordinates.x - (int)boidHeight*0.5;
			unsigned int y = coordinates.y - (int)boidWidth*0.5; // Image should be _centered_ on the coordinates
			drawPartOfImage(screen, birdIcons, x, y, frameNum*20, 0, boidHeight, boidWidth); 
		}

		/* Preform the actual rendering.
		 */
		SDL_Flip(screen);

		/* Check for the user quitting the application or moving
		 * the mouse.
		 */
		while(SDL_PollEvent(&event)){
			switch(event.type){
				case SDL_QUIT:
					running = false;
					break;
				case SDL_MOUSEMOTION:
					mousePos.x = event.motion.x;
					mousePos.y = event.motion.y;
					break;
			}
		}
	}

	/* Clean-up SDL resources.
	 */
	SDL_Quit();

	exit(0);
}

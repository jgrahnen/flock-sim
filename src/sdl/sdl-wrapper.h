/**
 * \file sdl-wrapper.h
 *
 * Functions that simplify handling the Simple DirectMedia Layer. See
 * implementation for more details.
 *
 * @author	Johan Grahnen
 * @since	2013-06-29
 * @see		sdl-wrapper.cpp
 */

/* Include guards.
 */
#ifndef SDL_WRAPPER_H
#define SDL_WRAPPER_H

/**
 * Includes.
 */
#include <SDL/SDL.h>

bool drawPixel32(SDL_Surface* screen, unsigned int x, unsigned int y, Uint8 R, Uint8 G, Uint8 B);

bool drawWholeImage(SDL_Surface* targetDisplay, SDL_Surface* sourceDisplay, unsigned int x, unsigned int y);

bool drawPartOfImage(SDL_Surface* targetDisplay, SDL_Surface* sourceDisplay, unsigned int targetX, unsigned int targetY, unsigned int sourceX, unsigned int sourceY, unsigned int sourceWidth, unsigned int sourceHeight);

bool transparentize(SDL_Surface* targetDisplay, unsigned int R, unsigned int G, unsigned int B);

SDL_Surface* initializeDisplay(unsigned int width, unsigned int height);

SDL_Surface* loadBMPImage(const char* file);

void cleanUpAndQuit();

#endif

/**
 * \file sdl-wrapper.cpp
 *
 * Functions that simplify using the Simple DirectMedia Layer (SDL) to draw 
 * stuff to screen.
 *
 * Mainly facilities for drawing individual pixes/images to the screen.
 *
 * @author	Johan Grahnen
 * @since	2013-06-29
 * @see		sdl-wrapper.h
 * @see		SDL.h
 */

/**
 * Includes.
 */
#include "sdl-wrapper.h"
#include <iostream>

using namespace std;

/**
 * Draws a single pixel on a 32-bit color-depth screen.
 *
 * Only sets the pixel, doesn't update the screen.
 *
 * @param screen	The surface to draw on.
 * @param x		X coordinate to draw at.
 * @param y		Y coordinate to draw at (note: more positive, more
 * 			downwards!).
 * @param R		Amount of red (in RGB).
 * @param G		Amount of green.
 * @param B		Amount of blue.
 * @return		false on inability to lock the screen to draw, true 
 * 			otherwise.
 */
bool drawPixel32(SDL_Surface* screen, unsigned int x, unsigned int y, Uint8 R, Uint8 G, Uint8 B){
	Uint32 color = SDL_MapRGB(screen->format, R, G, B);

	/* Lock surface if necessary.
	 */
	if(SDL_MUSTLOCK(screen)){
		if( SDL_LockSurface(screen) < 0){
			return false;
		}
	}

	/* Set the pixel.
	 */
	Uint32* pBuffer = (Uint32 *)screen->pixels + y*screen->pitch/4 + x;
	*pBuffer = color;

	/* Unlock.
	 */
	if(SDL_MUSTLOCK(screen)){
		SDL_UnlockSurface(screen);
	}

	return true;
}

/** 
 * Draws an image to the target screen by blitting. 
 *
 * Doesn't update the screen automatically. No bounds checking of target
 * coordinates.
 *
 * @param targetDisplay	Surface to draw on.
 * @param sourceDisplay	Image to draw on the target.
 * @param x		X coordinate to start drawing.
 * @param y		Y coordinate to start drawing (larger is further down).
 * @return		false on inability to lock the target screen, true 
 * 			otherwise.
 */
bool drawWholeImage(SDL_Surface* targetDisplay, SDL_Surface* sourceDisplay, unsigned int x, unsigned int y){
	/* Lock surface if necessary.
	 */
	if(SDL_MUSTLOCK(targetDisplay)){
		if( SDL_LockSurface(targetDisplay) < 0){
			return false;
		}
	}
	
	/* Blit the source onto the target.
	 */
	SDL_Rect destRect;
	destRect.x = x;
	destRect.y = y;
	SDL_BlitSurface(sourceDisplay, NULL, targetDisplay, &destRect);

	/* Unlock.
	 */
	if(SDL_MUSTLOCK(targetDisplay)){
		SDL_UnlockSurface(targetDisplay);
	}

	return true;
}

/**
 * Draws part of an image onto the target surface by blitting.
 *
 * Doesn't update the screen. No bounds checking on either source or target
 * coordinates.
 *
 * @param targetDisplay	Surface to draw on.
 * @param sourceDisplay	Image to copy from.
 * @param targetX	X coordinate to start drawing.
 * @param targetY	Y coordinate to start drawing (larger is further down)
 * @param sourceX	X coordinate of upper left of the portion of the image
 * 			to be copied.
 * @param sourceY	Y coordinate of upper left of the partion of the image
 * 			to be copied.
 * @param sourceWidth	Width of the portion of the image to be copied.
 * @param sourceHeight	Height of the portion of the image to be copied.
 * @return		false on inability to lock the screen before drawing,
 * 			true otherwise.
 */
bool drawPartOfImage(SDL_Surface* targetDisplay, SDL_Surface* sourceDisplay, unsigned int targetX, unsigned int targetY, unsigned int sourceX, unsigned int sourceY, unsigned int sourceWidth, unsigned int sourceHeight){
	/* Lock surface if necessary.
	 */
	if(SDL_MUSTLOCK(targetDisplay)){
		if( SDL_LockSurface(targetDisplay) < 0){
			return false;
		}
	}
	
	/* Blit the source onto the target.
	 *
	 * Note: this will not go well if attempting to color outside the 
	 * lines.
	 */
	SDL_Rect destRect;
	SDL_Rect sourceRect;
	destRect.x = targetX;
	destRect.y = targetY;
	sourceRect.x = sourceX;
	sourceRect.y = sourceY;
	sourceRect.w = sourceWidth;
	sourceRect.h = sourceHeight;

	SDL_BlitSurface(sourceDisplay, &sourceRect, targetDisplay, &destRect);

	/* Unlock.
	 */
	if(SDL_MUSTLOCK(targetDisplay)){
		SDL_UnlockSurface(targetDisplay);
	}

	return true;
}

/**
 * Makes a particular RGB color transparent on the surface at hand.
 *
 * Note: results of feeding in RGB values >255 are rather uncertain, so avoid
 * that.
 *
 * @param targetDisplay	Surface to be modified.
 * @param R		Amount of red (in RGB) for transparent color.
 * @param G		Amount of green.
 * @param B		Amount of blue.
 * @return		false if the surface doesn't exist, true otherwise.
 */
bool transparentize(SDL_Surface* targetDisplay, unsigned int R, unsigned int G, unsigned int B){
	if(targetDisplay == NULL){
		return false;
	}

	SDL_SetColorKey(targetDisplay, SDL_SRCCOLORKEY | SDL_RLEACCEL, SDL_MapRGB(targetDisplay->format, R, G, B));

	return true;
}

/**
 * Turns on SDL drawing and creates a 32-bit, hardware accelerated display.
 *
 * @param width		Width of the display.
 * @param height	Height of the display.
 * @return		The drawable surface on success, NULL on failure.
 */
SDL_Surface* initializeDisplay(unsigned int width, unsigned int height){
	SDL_Surface* display = NULL;

	/* Initialize the library itself.
	 */
	if(SDL_Init(SDL_INIT_EVERYTHING) < 0){
		return display;
	}

	/* Setup the display.
	 */
	if((display = SDL_SetVideoMode(width, height, 32, SDL_HWSURFACE | SDL_DOUBLEBUF)) == NULL){
		return NULL;
	}

	return display;
}

/**
 * Loads a BMP image from file.
 *
 * @param file	The file name.
 * @return	The properly formatted image as a Surface or NULL on failure.
 */
SDL_Surface* loadBMPImage(const char* file){
	SDL_Surface* loaded = 0;
	SDL_Surface* image = 0;

	if((loaded = SDL_LoadBMP(file)) == NULL){
		return loaded;
	}
	else{
		image = SDL_DisplayFormat(loaded);
		SDL_FreeSurface(loaded);
	}

	return image;
}

/**
 * Cleans up memory after SDL in the event of an error. 
 *
 * Exits with exit code 1 and prints the most recent error to STDERR.
 *
 * @see		SDL_Quit()
 */
void cleanUpAndQuit(){
	SDL_Quit();
	cerr << SDL_GetError() << endl;
	exit(1);
}

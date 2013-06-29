/**
 * \file Boid.h
 *
 * Class to represent a Boid for flocking. See implementation for more details.
 *
 * @see		Boid.cpp
 */

/* Idempotency.
 */
#ifndef BOID_H
#define BOID_H

/**
 * Includes.
 */
#include <vector>
#include "geometry/point.h"
#include "geometry/vector.h"

/**
 * Definitions.
 */
using namespace std;

class Boid {
	public:
		Boid(Point currentCoords, Vector velocity, float cohesionStrength, float separationStrength, float alignmentStrength, float attractionStrength, Point edgeOfWorld);
		~Boid();

		Boid step(const vector<Boid>& otherBoids, const Point& destination) const;
		Boid wrappedStep(const vector<Boid>& otherBoids, const Point& destination, const int maxX, const int maxY) const;
		Point getCoordinates() const;
		Vector getVelocity() const;

	protected:
		Vector compositeAcceleration(const vector<Boid>& otherBoids, const Point& destination) const;

		Vector accelCohesion(const vector<Boid>& otherBoids) const;
		Vector accelSeparation(const vector<Boid>& otherBoids) const;
		Vector accelAlignment(const vector<Boid>& otherBoids) const;
		Vector accelToward(const Point& coordinates) const;

		Vector stokesDrag(Vector currentVelocity) const;
		Vector dampenMotion(Vector currentChangeVector, float maxChangeRate) const;

		/* Properties.
		 */
		Point coords;
		Vector velocity;
		float cohesion;
		float separation;
		float alignment;
		float attraction;
		Point edges;
};

/* End idempotency.
 */
#endif

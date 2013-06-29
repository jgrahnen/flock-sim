/**
 * \file	Boid.cpp
 *
 * Implementation of Boid class for simulated flocking.
 *
 * Assumes a 2D coordinate system where positive Y values point "down" and
 * positive X values point "right" (for compatibility with screen drawing 
 * algorithms).
 *
 * See Reynolds (1987, Computer Graphics 21:4) for the original idea. This
 * implementation is better thought of as a "schooling" simulation for fish
 * than a flocking simulation of birds.
 *
 * @author	Johan Grahnen
 * @since	2013-06-29
 */

/**
 * Includes.
 */
#include "Boid.h"
#include <cmath>
#include <numeric>
#include <iostream>
#include <stdexcept>
#include <utility>

/** 
 * Definitions.
 */
#define PI 3.14159265

/**
 * Constructor from values.
 *
 * @param currentCoords		Location of the boid.
 * @param velocityComponents	X and Y components of velocity vector.
 * @param cohesionStrength	Multiplier for the flock cohension acceleration
 * 				component.
 * @param separationStrength	Multiplier for the boid-boid separation
 * 				acceleration component.
 * @param alignmentStrength	Multiplier for the flock velocity alignment
 * 				acceleration component.
 * @param attractionStrength	Multiplier for the destination attraction
 * 				acceleration component.
 * @param edgeOfWorld		X and Y coordinates of the maximum extent of
 * 				the simulated space.
 * @return			A fully specified object.
 */
Boid::Boid(Point currentCoords, Vector velocityComponents, float cohesionStrength, float separationStrength, float alignmentStrength, float attractionStrength, Point edgeOfWorld){
	coords = currentCoords;
	velocity = velocityComponents;
	cohesion = cohesionStrength;
	separation = separationStrength;
	alignment = alignmentStrength;
	attraction = attractionStrength;
	edges = edgeOfWorld;
}

/**
 * Default destructor, necessary for inheritance.
 */
Boid::~Boid(){
}

/**
 * Step to the next tic of the simulation, assuming that the edges of the
 * simulation world are solid walls.
 *
 * Calculates where the boid will be in the next tic of the simluation, and the
 * velocity it will have due to the various accelerations acting on it.
 *
 * Attempts to resolve collisions with the edges of the simulation world
 * as fully elastic, but throws a std::domain_error if it fails to do so
 * or if the novel position is outside the simulation world for any reason.
 *
 * @param otherBoids	The other boids with which it can interact.
 * @param destination	Coordinates toward which the boid should head.
 * @return		A new boid with the next position and velocity.
 * @throws		std::domain_error
 * @see			Boid::compositeAcceleration()
 * @see			Boid::stokesDrag()
 */
Boid Boid::step(const vector<Boid>& otherBoids, const Point& destination) const{
	Vector novelVelocity(velocity);
	Point novelCoords(coords);

	/* Get the total acceleration due to external factors.
	 */
	Vector acceleration = compositeAcceleration(otherBoids, destination);

	/* Update the velocity.
	 *
	 * Subject to viscous damping.
	 */
	Vector drag = stokesDrag(this->velocity);
	novelVelocity += acceleration + drag;

 	/* New position due to velocity. For convenience, we'll use a time step 
	 * of 1 here.
	 */
	novelCoords += novelVelocity;

	/* Make sure Boids bounce off the walls.
	 *
	 * Simple a posteriori elastic collision: if the Boid attempts to fly 
	 * outside the drawing area, reflect its coordinates and motion about 
	 * that edge.
	 *
	 * At high velocities, or very close to the edge of the drawing area, 
	 * this tends to work poorly.
	 */
	float distToX = edges.x - novelCoords.x;
	float distToY = edges.y - novelCoords.y;
	if(distToX > edges.x || distToX < 0){
		novelVelocity.x *= -1.0;
		novelCoords.x = distToX < 0 ? novelCoords.x + 2*distToX : abs(novelCoords.x);
	}
	if(distToY > edges.y || distToY < 0){
		novelVelocity.y *= -1.0;
		novelCoords.y = distToY < 0 ? novelCoords.y + 2*distToY : abs(novelCoords.y);
	}

	/* Never tolerate flying off the end of the world.
	 *
	 * Seems rather vulnerable to rounding errors between FP and int
	 * (see above).
	 */
	if(novelCoords.x > edges.x
			|| novelCoords.x < 0 
			|| novelCoords.y > edges.y 
			|| novelCoords.y < 0
	){
		throw domain_error("Attempting to put Boid outside of the world!");
	}

	return Boid(novelCoords, novelVelocity, this->cohesion, this->separation, this->alignment, this->attraction, this->edges);
}

/**
 * Step to the next tic of the simulation, assuming that the edges of the
 * simulated world wrap around.
 *
 * Produces a new boid by calculating the acceleration due to various factors, 
 * thereby obtaining new velocity and position.
 *
 * If the Boid flies outside of the maximum allowed coordinates, "wrap around" 
 * the screen (Asteroids-style).
 *
 * @param otherBoids	The other boids with which it can interact.
 * @param destination	Coordinates toward which the boid should head.
 * @param maxX		Edge of the world in the X direction.
 * @param maxY		Edge of the world in the Y direction.
 * @return		A new boid with the next position and velocity.
 * @see			Boid::compositeAcceleration()
 * @see			Boid::stokesDrag()
 */
Boid Boid::wrappedStep(const vector<Boid>& otherBoids, const Point& destination, const int maxX, const int maxY) const{
	Vector novelVelocity(velocity);
	Point novelCoords(coords);

	/* Get the total acceleration due to external factors.
	 */
	Vector acceleration = compositeAcceleration(otherBoids, destination);

	/* Update the velocity.
	 *
	 * Subject to viscous damping.
	 */
	Vector drag = stokesDrag(this->velocity);
	novelVelocity += acceleration + drag;

	/* New position due to velocity. For convenience, we'll
	 * use a time step of 1 here.
	 */
	novelCoords += novelVelocity;

	/* Wrap coordinates around if they're outside the screen.
	 *
	 * Topologically speaking, the Boid is moving on a 3D torus, the
	 * surface of which is projected onto the 2D screen.
	 */
	if(novelCoords.x > maxX){
		novelCoords.x -= maxX;
	}
	else if(novelCoords.x < 0){
		novelCoords.x += maxX;
	}
	if(novelCoords.y > maxY){
		novelCoords.y -= maxY;
	}
	else if(novelCoords.y < 0){
		novelCoords.y += maxY;
	}

	return Boid(novelCoords, novelVelocity, this->cohesion, this->separation, this->alignment, this->attraction, this->edges);
}

/**
 * Getter for current coordinates.
 *
 * @return	The current boid coordinates.
 */
Point Boid::getCoordinates() const{
	return coords;
}

/**
 * Getter for current velocity.
 *
 * @return	The current boid velocity vector.
 */
Vector Boid::getVelocity() const{
	return velocity;
}

/**
 * Calculates the overall acceleration vector acting on the boid.
 *
 * Sums up all the various accelerations the Boid is subject to due to
 * interaction with the world and other Boids.
 *
 * @param otherBoids	The other boids with which it can interact.
 * @param destination	Coordinates toward which the boid should head.
 * @return		Acceleration vector acting on the boid.
 * @see			Boid::accelCohesion()
 * @see			Boid::accelSeparation()
 * @see			Boid::accelAlignment()
 * @see			Boid::accelToward()
 */
Vector Boid::compositeAcceleration(const vector<Boid>& otherBoids, const Point& destination) const{
	Vector totAcc(0.0, 0.0);

	/* Accelerate toward the other Boids.
	 */
	Vector dueToCohesion = accelCohesion(otherBoids);
	totAcc += cohesion*dueToCohesion;

	/* And away from other Boids, too.
	 */
	Vector dueToSep = accelSeparation(otherBoids);
	totAcc += separation*dueToSep;

	/* While trying to match speeds with them.
	 */
	Vector dueToAlign = accelAlignment(otherBoids);
	totAcc += alignment*dueToAlign;

	/* And heading toward some position.
	 */
	Vector dueToAttraction = accelToward(destination);
	totAcc += attraction*dueToAttraction;

	return totAcc;
}
	
/**
 * Acceleration due to tendency to stick to other boids in the flock.
 *
 * Accelerates the Boid toward the geometric centroid of the others of its
 * kind. Rate of acceleration determined by a 'perception' parameter, and
 * direction by which Boids are close enough to "see".
 *
 * @param otherBoids	The other boids with which it can interact.
 * @return		Acceleration vector due to flock cohesion.
 */
Vector Boid::accelCohesion(const vector<Boid>& otherBoids) const{
	Vector acc(0.0, 0.0);
	Vector xBasisVector(1.0, 0.0);
	Vector yBasisVector(0.0, 1.0);
	Point centroid(0.0, 0.0);

	/* Check where all the other Boids are, and find
	 * the centroid of their positions. However, weight the
	 * calculation of the centroid by a rapidly decreasing
	 * factor based on distance (can't perceive very far-away
	 * flockmates that well).
	 */
	float dist, percepFactor;
	float percepTotal = 0.0;
	for(auto it = otherBoids.begin(); it != otherBoids.end(); it++){
		dist = d(coords, it->getCoordinates());
		double fallOff = 2.75; // Compromise between light/sound propagation for water (fish, r^3) and air (birds, r^2) 
		percepFactor = min(1.0, 1.0/pow(dist,fallOff));

		Point scaledOther = percepFactor * it->getCoordinates();
		centroid = centroid + scaledOther;
		percepTotal += percepFactor;
	}
	centroid = centroid / percepTotal;

	/* Now head toward that centroid (biased toward nearby
	 * flockmates).
	 *
	 * This math should probably be abstracted quite a bit...
	 * a ProjectOnto() function or something?
	 */
	Point position(coords);
        Point diff = position - centroid;
	Vector sepVector(diff.x,diff.y);
	float norm = sepVector.len();
	sepVector.normalize();
	xBasisVector.normalize();
	yBasisVector.normalize();
	float theta_x = acos(xBasisVector * sepVector);
	float theta_y = acos(yBasisVector * sepVector);

	acc.x -= norm*cos(theta_x);
	acc.y -= norm*cos(theta_y);

	return acc;
}

/**
 * Acceleration due to tendency to not collide with other boids in the flock.
 *
 * In practice, accelerate away from every other Boid in the flock that
 * can be perceived. Degree of perception scales inversely with distance.
 *
 * @param otherBoids	The other boids with which it can interact.
 * @return		Acceleration vector due to boid-boid separation.
 */
Vector Boid::accelSeparation(const vector<Boid>& otherBoids) const{
	Vector acc(0.0, 0.0);
	Vector xBasisVector(1.0, 0.0);
	Vector yBasisVector(0.0, 1.0);

	/* Check where the other Boids are, and avoid them
	 * proportional to the inverse square of their distance:
	 * can't see distant Boids very well, but want to stay
	 * the hell away from really close ones.
	 *
	 * Math here should be simplified with a projection
	 * method, or something.
	 */
	float dist;
	for(auto it = otherBoids.begin(); it != otherBoids.end(); it++){
		Point position(coords);
		Point diff = position - it->getCoordinates();
		Vector sepVector(diff.x,diff.y);
		dist = sepVector.len();
		sepVector.normalize();
		xBasisVector.normalize();
		yBasisVector.normalize();
		float theta_x = acos(xBasisVector * sepVector);
		float theta_y = acos(yBasisVector * sepVector);
		
		double collision_dist = 100.0; // 10 pixels per boid, so r = 10.0 gives you r^2 = 100 and unit acceleration
		acc.x += (collision_dist/pow(dist,2))*cos(theta_x);
		acc.y += (collision_dist/pow(dist,2))*cos(theta_y);
	}

	return acc;
}

/**
 * Acceleration due to tendency to move in the same direction as other boids
 * in the flock.
 *
 * In practice, try to match the velocity vector of nearby "perceivable"
 * boids. Ability to perceive other boids scales inversely with distance
 * to them.
 *
 * @param otherBoids	The other boids with which it can interact.
 * @return		Acceleration vector due to flock velocity alignment.
 */
Vector Boid::accelAlignment(const vector<Boid>& otherBoids) const{
	Vector acc(0.0, 0.0);
	Vector xBasisVector(1.0, 0.0);
	Vector yBasisVector(0.0, 1.0);
	Vector commonVeloc(0.0, 0.0);

	/* Check where all the other Boids are, and find
	 * their average velocity. However, weight the
	 * calculation of the velocity by a rapidly decreasing
	 * factor based on distance (can't perceive very far-away
	 * flockmates that well).
	 */
	float dist, percepFactor;
	float percepTotal = 0.0;
	for(auto it = otherBoids.begin(); it != otherBoids.end(); it++){
		dist = d(coords, it->getCoordinates());
		double fallOff = 2.75; // Compromise between light/sound propagation for water (fish, r^3) and air (birds, r^2) 
		percepFactor = min(1.0, 1.0/pow(dist,fallOff));

		commonVeloc += percepFactor * it->getVelocity();
		percepTotal += percepFactor;
	}
	commonVeloc /= percepTotal;

	/* Now head in the direction of that velocity vector (biased toward 
	 * nearby flockmates), attempting to match speeds as well.
	 */
	Vector temp(velocity);
	Vector velocDiff = temp - commonVeloc;
	acc -= velocDiff;

	return acc;
}

/**
 * Acceleration due to tendency to move toward a particular goal.
 *
 * In practice, accelerate toward these particular coordinates, but with
 * a magnitude inversely proportional to the distance from them.
 *
 * @param coordinates	The point toward which boids should move.
 * @return		Acceleration vector due to goal seeking.
 */
Vector Boid::accelToward(const Point& coordinates) const{
	Vector acc(0.0, 0.0);
	Vector xBasisVector(1.0, 0.0);
	Vector yBasisVector(0.0, 1.0);
	Point destination(coordinates);

	/* Now head toward that point, acceleration inversely
	 * proportional to distance.
	 *
	 * This math should probably be abstracted quite a bit...
	 * a ProjectOnto() function or something?
	 */
	Point position(coords);
        Point diff = position - destination;
	Vector sepVector(diff.x,diff.y);
	double percepDecay = 0.1; // Smaller -> Boids see the destination when it's further away
	float norm = 1.0 / (1.0 + percepDecay*sepVector.len());
	sepVector.normalize();
	xBasisVector.normalize();
	yBasisVector.normalize();
	float theta_x = acos(xBasisVector * sepVector);
	float theta_y = acos(yBasisVector * sepVector);

	acc.x -= norm*cos(theta_x);
	acc.y -= norm*cos(theta_y);

	return acc;
}

/**
 * Function to calculate the force vector of the Stokes drag due to
 * the surrounding medium.
 *
 * Drag force is
 *
 * F_d = -C_d*v
 *
 * where C_d is a coefficient of drag incorporating medium density and
 * object characterstic length scale (usually cross-sectional area
 * in the direction of travel), and v is the velocity vector.
 *
 * Note that the force is opposite to the direction of travel, and
 * that we assume laminar flow (turbulent flow would result in
 * Newton drag) with fairly low coefficient of drag.
 *
 * @param currentVelocity	The current velocity of a boid.
 * @return			The force vector of drag.
 */
Vector Boid::stokesDrag(Vector currentVelocity) const{
	Vector force(0.0, 0.0);
	Vector zero(0.0,0.0);

	/* If there's no movement, there should be no drag.
	 */
	if(currentVelocity == zero){
		return force;
	}

	/* Drag is directly proportional to the norm of the
	 * vector.
	 */
	float dragCoefficient = 0.005;
	float norm = currentVelocity.len();
	float drag = -1.0*dragCoefficient*norm;

	/* Project the total drag onto the X and Y axis.
	 */
	Vector xBasisVector(1.0, 0.0);
	Vector yBasisVector(0.0, 1.0);
	Vector velocVector = currentVelocity;
	velocVector.normalize();
	xBasisVector.normalize();
	yBasisVector.normalize();

	float theta_x = acos(xBasisVector * velocVector);
	float theta_y = acos(yBasisVector * velocVector);

	force.x = drag*cos(theta_x);
	force.y = drag*cos(theta_y);

	return force;
}

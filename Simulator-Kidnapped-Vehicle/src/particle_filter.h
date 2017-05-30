/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"
#include <math.h>
#include <float.h>
#include <stdio.h>

using namespace std;

struct Particle {

	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
};



class ParticleFilter {
	// Number of particles to draw
	int num_particles;

	// Flag, if filter is initialized
	bool is_initialized;

	// Vector of weights of all particles
	vector<double> weights;

	public:
	// Set of current particles
	vector<Particle> particles;

	// Constructor
	// @param M Number of particles, whether the particle is initialized
	ParticleFilter() : num_particles(0), is_initialized(false) {}

	// Destructor
	~ParticleFilter() {}

	/*
	 * Initializes particle filter by initializing particles to Gaussian
	 * distribution around first position and all the weights set to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init(double x, double y, double theta, double std[]);

	/*
	 * Predicts the state for the next time step using the process model.
	 * @param delta_t: Time between time step t and t+1 in measurements [s]
	 * @param std_pos[]: Array of dimension 3 [standard deviation of x [m],
	 *																				 standard deviation of y [m],
	 *   																       standard deviation of yaw [rad]]
	 * @param velocity: Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate: Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(double delta_t, double std_pos[],
									double velocity, double yaw_rate);

	/*
	 * Updates the weights for each particle based on the likelihood of the
	 * observed measurements.
	 * @param sensor_range: Range [m] of sensor
	 * @param std_landmark[]: Array of dimension 2 [standard deviation of range [m],
	 *   																						standard deviation of bearing [rad]]
	 * @param observations: Vector of landmark observations
	 * @param map: Map class containing map landmarks
	 */
	void updateWeights(double sensor_range, double std_landmark[],
										 vector<LandmarkObs> observations, Map map_landmarks);

	/*
	 * Resample particles with replacement with probability proportional to weight
	 */
	void resample();

	/*
	 * Writes particle positions to a file.
	 * @param filename: File to write particle positions to.
	 */
	void write(string filename);

	/*
	 * Returns whether particle filter is initialized yet or not.
	 */
	const bool initialized() const
	{
		return is_initialized;
	}

	/*
	 * Set a particles list of associations, along with the associations calculated world x,y coordinates
	 * This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
	 */
	Particle SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y);

	std::string getAssociations(Particle best);
	std::string getSenseX(Particle best);
	std::string getSenseY(Particle best);
private:
	/*
	 * Convert the passed in vehicle co-ordinates into map co-ordinates from
	 * the perspective of the particle in question
	 */
	 LandmarkObs convertVehicleToMapCoords(LandmarkObs observationToConvert,
 																				 Particle particle);
	 /*
 	 * Finds which observations correspond to which landmark
 	 * (likely by using a nearest-neighbors data association).
 	 * @param landmarks: List of landmarks
 	 * @param observation: Current list of converted observation
 	 */
	vector<LandmarkObs> dataAssociation(vector<Map::single_landmark_s> landmarks,
		 																	vector<LandmarkObs> observations);
};



#endif /* PARTICLE_FILTER_H_ */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

// Initializes particle filter by initializing particles to
// Gaussian distribution around first position and all the weights set to 1.
void ParticleFilter::init(double x, double y, double theta, double std[])
{
	// NOTE: The number of particles needs to be tuned
	num_particles = 200;

	// Object of random number engine class that generate pseudo-random numbers
	default_random_engine gen;

  // Standard deviations for x, y, and theta
	double std_x, std_y, std_theta;

	// Set standard deviations for x, y, and theta.
  std_x = std[0];
	std_y = std[1];
  std_theta = std[2];

  // Create a normal (Gaussian) distribution for position along x.
	normal_distribution<double> dist_x(x, std_x);
  // Create a normal (Gaussian) distribution for position along y.
	normal_distribution<double> dist_y(y, std_y);
  // Create a normal (Gaussian) distribution for heading of the car.
	normal_distribution<double> dist_theta(theta, std_theta);

	// Add random Gaussian noise to each particle.
	for (int par_index = 0; par_index < num_particles; ++par_index)
	{
		double sample_x, sample_y, sample_theta;

		// Sample from these normal distrubtions for x, y and theta
    // NOTE 1: "gen" is the random engine initialized earlier
		// NOTE 2: The generator object (g) supplies uniformly-distributed random
		//         integers through its operator() member function.
		//         The normal_distribution object transforms the values obtained
		//         this way so that successive calls to this member function with
		//         the same arguments produce floating-point values that follow a
		//         Normal distribution with the appropriate parameters.
		// NOTE 3: dist_x's constructor is overloaded with a member function of the
		//         same name.
		sample_x = dist_x(gen);
    sample_y = dist_y(gen);
    sample_theta = dist_theta(gen);

		// Variable of type Particle to store values before being appended to
		// the vector of particles
		Particle new_particle;

		// Set the id of the particle to be the same as the current index
		new_particle.id = par_index;
		// Set the particle position in x, y and angle theta from the individual
		// samples from the distribution of the respective means and sigmas
		new_particle.x = sample_x;
		new_particle.y = sample_y;
		new_particle.theta = sample_theta;

		// The weight needs to be set to 1.0 initially
		new_particle.weight = 1.0;

		// Append the new particle to the vector
		particles.push_back(new_particle);
	}

	// Since this function is called only once(first measurement), set to True
	is_initialized = true;
}

// Predicts the state(set of particles) for the next time step
// using the process model.
void ParticleFilter::prediction(double delta_t, double std_pos[],
																double velocity, double yaw_rate)
{
	// Add measurements to each particle and add random Gaussian noise.
	// Object of random number engine class that generate pseudo-random numbers
	default_random_engine gen;

  // Standard deviations for x, y, and theta
	double std_x, std_y, std_theta;

	// Set standard deviations for x, y, and theta.
  std_x = std_pos[0];
	std_y = std_pos[1];
  std_theta = std_pos[2];

	// Create a normal (Gaussian) distribution for noise along position x.
	normal_distribution<double> noise_dist_x(0, std_x);
	// Create a normal (Gaussian) distribution for noise along position y.
	normal_distribution<double> noise_dist_y(0, std_y);
	// Create a normal (Gaussian) distribution for noise of direction theta.
	normal_distribution<double> noise_dist_theta(0, std_theta);

	// NOTE: Adding noise using the following resources
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Prediction for position x,y and angle theta for each of the particles
	for(size_t par_index = 0; par_index < particles.size(); par_index++)
	{
		// Temporary variable to store the particle's previous state's theta
		double prev_theta = particles[par_index].theta;

		// Avoid divide by zero error and update prediction for the particle
		if(abs(yaw_rate) > 0.0001)
		{
			// Update the position x, y and angle theta of the particle
			particles[par_index].x += (velocity/yaw_rate) * \
																 (sin(prev_theta + (yaw_rate * delta_t)) - \
																	sin(prev_theta));
			particles[par_index].y += (velocity/yaw_rate) * \
																 (cos(prev_theta) - \
																	cos(prev_theta + (yaw_rate * delta_t)));
		}
		else
		{
			// Update the position x, y and angle theta of the particle
			particles[par_index].x += velocity * delta_t * cos(prev_theta);
			particles[par_index].y += velocity * delta_t * sin(prev_theta);
		}
		// Update theta
		particles[par_index].theta = prev_theta + yaw_rate * delta_t;

		// Add random gaussian noise for each of the above updated measurements
		particles[par_index].x += noise_dist_x(gen);
		particles[par_index].y += noise_dist_y(gen);
		particles[par_index].theta += noise_dist_theta(gen);
	}
}

// Find the closest landmark to the current observation
size_t ParticleFilter::dataAssociation(vector<Map::single_landmark_s> landmarks,
																			 LandmarkObs observation)
{
	// Start of with the maximum possible value
	double minDistance = DBL_MAX;
	size_t indexOfLandmark;

	// Find the landmark closest to the observation
	for(size_t land_index = 0; land_index < landmarks.size(); land_index++)
	{
		double currentDistance = dist(landmarks[land_index].x_f,
																	landmarks[land_index].y_f,
																	observation.x,
																	observation.y);

		// Update the minimum distance found and the index if
		// another landmark is closer to this observation
		if(currentDistance <= minDistance)
		{
			minDistance = currentDistance;
			indexOfLandmark = land_index;
		}
	}

	return indexOfLandmark;
}

// Update all the weights of the particles in the particle filter
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
																	 vector<LandmarkObs> observations,
																	 Map map_landmarks)
{
	// Set standard deviations for x, y
	double std_x, std_y;
	double var_x, var_y;

	// Set the standard deviation and calculate variance
	std_x = std_landmark[0];
	std_y = std_landmark[1];
	var_x = std_x * std_x;
	var_y = std_y * std_y;

	// Go through the list of particles
	for(size_t par_index = 0; par_index < particles.size(); par_index++)
	{
		// For the given list of landmarks find the predicted landmarks within
		// the range of the car sensor
		vector<Map::single_landmark_s> predicted_landmarks;
		for(size_t land_index = 0; land_index < map_landmarks.landmark_list.size(); land_index++)
		{
			// Calculate the difference between the particle prediction & landmark
			double distanceDiff = dist(particles[par_index].x,
																 particles[par_index].y,
																 map_landmarks.landmark_list[land_index].x_f,
															 	 map_landmarks.landmark_list[land_index].y_f);

			// Create a new list of landmarks within sensor range for data association
			if(distanceDiff <= sensor_range)
			{
				predicted_landmarks.push_back(map_landmarks.landmark_list[land_index]);
			}
		}

		// For the list of observations, convert to map-coordinates, find the
		// closest landmark and finally update the weight of the particle
		for(size_t obs_index = 0; obs_index < observations.size(); obs_index++)
		{
			// Convert from car to map-coordinates
			LandmarkObs convertedObs = convertVehicleToMapCoords(observations[obs_index],
																													 particles[par_index]);

			// Find the closest predicted landmark to this observation
			size_t closestLandIndex = dataAssociation(predicted_landmarks, convertedObs);

			// Update the weights of each particle using a
		  // a multi-variate Gaussian distribution.
		  // Info: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
			// NOTE: Since the rank of covariance is 2, bi-variate guassian equation
			//       is used for updating the weights

			// Variables to store the square of the difference between measured and
			// predicted x and y values
			double sq_diff_x = predicted_landmarks[closestLandIndex].x_f - convertedObs.x;
			double sq_diff_y = predicted_landmarks[closestLandIndex].y_f - convertedObs.y;
			sq_diff_x = sq_diff_x * sq_diff_x;
			sq_diff_y = sq_diff_y * sq_diff_y;

			// Variable to store the result of the multivariate-gaussian
			double multi_gaussian;
			multi_gaussian = (1 / (2 * M_PI * std_x * std_y)) * \
			                                exp(-((sq_diff_x) / (2 * var_x) + (sq_diff_y) / (2 * var_y)));

			particles[par_index].weight *= multi_gaussian;
		}
	}
}

// Resample particles with replacement with probability proportional to weight.
void ParticleFilter::resample()
{
	// Copy of the particles vector list
	vector<Particle> particlesCopy = particles;

	// Empty the existing particle list
	particles.erase(particles.begin(), particles.end());

	// Vector of weights of the particles
	vector<double> weights;
	for(size_t par_index = 0; par_index < particlesCopy.size(); par_index++)
	{
		weights.push_back(particlesCopy[par_index].weight);
	}

	// Object of random number engine class that generate pseudo-random numbers
	// NOTE: http://en.cppreference.com/w/cpp/numeric/random/mersenne_twister_engine
	mt19937 gen;

	// Object for generating discrete distribution based on the weights vector
	discrete_distribution<double> weights_dist(weights.begin(), weights.end());

	// With the discrete distribution pick out particles according to their
	// weights. The higher the weight of the particle, the higher are the chances
	// of the particle being included multiple times.
	// Discrete_distribution is used here to pick particles with the appropriate
	// weights(i.e. which meet a threshold)
	// http://www.cplusplus.com/reference/random/discrete_distribution/
	// NOTE: Here is an example which helps with the understanding
	//       http://coliru.stacked-crooked.com/a/3c9005a4cc0ed9d6
	for(size_t par_index = 0; par_index < particlesCopy.size(); par_index++)
	{
		// Append the particle to the new list
		// NOTE: Calling weights_dist with the generator returns the index of one
		//       of weights in the vector which was used to generate the distribution.
		particles.push_back(particlesCopy[weights_dist(gen)]);
	}

	/*for(size_t par_index = 0; par_index < particles.size(); par_index++)
	{
		cout << particles[par_index].id << ", ";
		cout << particles[par_index].x << ", ";
		cout << particles[par_index].y << ", ";
		cout << particles[par_index].theta << ", ";
		cout << particles[par_index].weight << "\n";
	}*/
}

void ParticleFilter::write(string filename)
{
	// You don't need to modify this file.
	ofstream dataFile;
	dataFile.open(filename, ios::app);
	for (int i = 0; i < num_particles; ++i)
	{
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}

// Convert the passed in vehicle co-ordinates into map co-ordinates from
// the perspective of the particle in question
LandmarkObs ParticleFilter::convertVehicleToMapCoords(LandmarkObs observationToConvert,
																					 		 				Particle particle)
{
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
	//   for the fact that the map's y-axis actually points downwards.)
	//   1. http://planning.cs.uiuc.edu/node99.html
	//   2. http://www.sunshine2k.de/articles/RotationDerivation.pdf
	LandmarkObs convertedObservation;
	convertedObservation.id = observationToConvert.id;
	convertedObservation.x = particle.x + \
													 observationToConvert.x * cos(particle.theta) - \
													 observationToConvert.y * sin(particle.theta);

	convertedObservation.y = particle.y + \
													 observationToConvert.x * sin(particle.theta) + \
													 observationToConvert.y * cos(particle.theta);

	return convertedObservation;
}

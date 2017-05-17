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
	num_particles = 100;

	// Since this function is called only once(first measurement), set to True
	is_initialized = true;

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
}

// Predicts the state(set of particles) for the next time step
// using the process model.
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
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

	// NOTE: Adding noise using the following resources
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  // Create a normal (Gaussian) distribution for noise along position x.
	normal_distribution<double> noise_dist_x(0.0, std_x);
  // Create a normal (Gaussian) distribution for noise along position y.
	normal_distribution<double> noise_dist_y(0.0, std_y);
  // Create a normal (Gaussian) distribution for noise of direction theta.
	normal_distribution<double> noise_dist_theta(0.0, std_theta);

	// Prediction for position x,y and angle theta for each of the particles
	for(size_t par_index = 0; par_index < particles.size(); par_index++)
	{
		// Temporary variable to store the particle's previous state's theta
		double prev_theta = particles[par_index].theta;

		// Update the position x, y and angle theta of the particle
		particles[par_index].x += (velocity/yaw_rate) * \
															 (sin((prev_theta) + (yaw_rate * delta_t)) - \
															  sin(prev_theta));
		particles[par_index].y += (velocity/yaw_rate) * \
															 (cos(prev_theta)- \
															  cos((prev_theta) + (yaw_rate * delta_t)));
		particles[par_index].theta += delta_t;

		// Add random gaussian noise for each of the above updated measurements
		particles[par_index].x += noise_dist_x(gen);
		particles[par_index].y += noise_dist_y(gen);
		particles[par_index].theta += noise_dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations)
{
	// TODO: Find the predicted measurement that is closest to each
	// observed measurement and assign the observed measurement to this
	// particular landmark.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
																	 vector<LandmarkObs> observations,
																	 Map map_landmarks)
{
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution

	// Go through the list of particles
	//for(size_t par_index = 0; par_index < particles.size(); par_index++)
	for(size_t par_index = 0; par_index <= 0; par_index++)
	{
		// Vector of converted observations
		vector<LandmarkObs> converted_obs;

		// Transform the vehicle observation into the map co-ordinates from
		// the perspective of the particle
		for(size_t obs_index = 0; obs_index < observations.size(); obs_index++)
		{
			// Get the current observation
			LandmarkObs current_obs = observations[obs_index];

			// Convert it from vehicle to map co-ordinates
			convertVehicleToMapCoords(current_obs, particles[par_index]);

			// Add it to the list of converted observations
			converted_obs.push_back(current_obs);
		}

		// Associate the closest observation(Euclidean Distance) to the landmark

	}
}

void ParticleFilter::resample()
{
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
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
void ParticleFilter::convertVehicleToMapCoords(LandmarkObs &observation,
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
	observation.x = particle.x + \
									observation.x * cos(particle.theta) - \
									observation.y * sin(particle.theta);

	observation.y = particle.y + \
									observation.x * sin(particle.theta) + \
									observation.y * cos(particle.theta);
}

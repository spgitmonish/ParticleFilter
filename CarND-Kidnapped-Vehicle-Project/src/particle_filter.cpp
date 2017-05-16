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

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations)
{
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
																	 std::vector<LandmarkObs> observations,
																	 Map map_landmarks)
{
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample()
{
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
}

void ParticleFilter::write(std::string filename)
{
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i)
	{
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}

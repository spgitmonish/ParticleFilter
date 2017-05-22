#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

int main()
{
	// NOTE: These parameters are related to grading.
	// Number of time steps before accuracy is checked by grader.
	int time_steps_before_lock_required = 100;
	// Max allowable runtime to pass [sec]
	double max_runtime = 45;
	// Max allowable translation error to pass [m]
	double max_translation_error = 1;
	// Max allowable yaw error [rad]
	double max_yaw_error = 0.05;

	// Start timer.
	int start = clock();

	// Time elapsed between measurements [sec]
	double delta_t = 0.1;
	// Sensor range [m]
	double sensor_range = 50;

	/*
	 * Sigmas - just an estimate, usually comes from uncertainty of sensor, but
	 * if you used fused data from multiple sensors, it's difficult to find
	 * these uncertainties directly.
	 */
	// GPS measurement uncertainty [x [m], y [m], theta [rad]]
	double sigma_pos [3] = {0.3, 0.3, 0.01};
	// Landmark measurement uncertainty [x [m], y [m]]
	double sigma_landmark [2] = {0.3, 0.3};

	// Noise generation(normal distribution)
	default_random_engine gen;
	normal_distribution<double> N_x_init(0, sigma_pos[0]);
	normal_distribution<double> N_y_init(0, sigma_pos[1]);
	normal_distribution<double> N_theta_init(0, sigma_pos[2]);
	normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
	normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
	double n_x, n_y, n_theta, n_range, n_heading;

	// Read map data
	Map map;
	if (!read_map_data("data/map_data.txt", map))
	{
		cout << "Error: Could not open map file" << endl;
		return -1;
	}

	// Read position data
	vector<control_s> position_meas;
	if (!read_control_data("data/control_data.txt", position_meas))
	{
		cout << "Error: Could not open position/control measurement file" << endl;
		return -1;
	}

	// Read ground truth data
	vector<ground_truth> gt;
	if (!read_gt_data("data/gt_data.txt", gt))
	{
		cout << "Error: Could not open ground truth data file" << endl;
		return -1;
	}

	// Run particle filter!
	int num_time_steps = position_meas.size();
	// Object of particle filter class
	ParticleFilter pf;
	// Variables to keep track of the error
	double total_error[3] = {0, 0, 0};
	double cum_mean_error[3] = {0, 0, 0};

	for (int i = 0; i < num_time_steps; ++i)
	{
		cout << "\nTime step: " << i << endl;

		// Read in landmark observations for current time step.
		ostringstream file;
		file << "data/observation/observations_" << setfill('0') << setw(6) << i+1 << ".txt";
		vector<LandmarkObs> observations;
		if (!read_landmark_data(file.str(), observations))
		{
			cout << "Error: Could not open observation file " << i+1 << endl;
			return -1;
		}

		// Initialize particle filter if this is the first time step.
		if (!pf.initialized())
		{
			n_x = N_x_init(gen);
			n_y = N_y_init(gen);
			n_theta = N_theta_init(gen);
			// Add noise to the ground truth for the initialization step
			pf.init(gt[i].x + n_x, gt[i].y + n_y, gt[i].theta + n_theta, sigma_pos);

		#if DEBUG
			for(size_t par_index = 0; par_index < pf.particles.size(); par_index++)
			{
				cout << pf.particles[par_index].id << ", ";
				cout << pf.particles[par_index].x << ", ";
				cout << pf.particles[par_index].y << ", ";
				cout << pf.particles[par_index].theta << ", ";
				cout << pf.particles[par_index].weight << "\n";
			}
		#endif
		}
		else
		{
			// Predict the vehicle's next state (noiseless).
			pf.prediction(delta_t, sigma_pos, position_meas[i-1].velocity, position_meas[i-1].yawrate);
		}

		// Simulate the addition of noise to noiseless observation data.
		vector<LandmarkObs> noisy_observations;
		LandmarkObs obs;
		for (int j = 0; j < observations.size(); ++j)
		{
			n_x = N_obs_x(gen);
			n_y = N_obs_y(gen);
			obs = observations[j];
			obs.x = obs.x + n_x;
			obs.y = obs.y + n_y;
			noisy_observations.push_back(obs);
		}

		// Update the weights of the particles and resample
		pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
		pf.resample();

		// Particles information after each iteration
		string par_output = "par_filter_output" + to_string(i) + ".txt";
		pf.write(par_output);
	#if DEBUG
		cout << "Post " << endl;
		cout << pf.particles[0].x << ", " << pf.particles[0].y << ", " << pf.particles[0].theta << "\n";
	#endif

		// Calculate and output the average weighted error of the particle filter
		// over all time steps so far.
		vector<Particle> particles = pf.particles;
		int num_particles = particles.size();
		double highest_weight = 0.0;
		Particle best_particle;

		for (int i = 0; i < num_particles; ++i)
		{
			if (particles[i].weight > highest_weight)
			{
				highest_weight = particles[i].weight;
				best_particle = particles[i];
			}
		}
		double *avg_error = getError(gt[i].x, gt[i].y, gt[i].theta, best_particle.x, best_particle.y, best_particle.theta);

		for (int j = 0; j < 3; ++j)
		{
			total_error[j] += avg_error[j];
			cum_mean_error[j] = total_error[j] / (double)(i + 1);
		}

		// Print the cumulative weighted error
		cout << "Cumulative mean weighted error: x " << cum_mean_error[0] << " y " << cum_mean_error[1] << " yaw " << cum_mean_error[2] << endl;

		// If the error is too high, say so and then exit.
		if (i >= time_steps_before_lock_required)
		{
			if (cum_mean_error[0] > max_translation_error || cum_mean_error[1] > max_translation_error || cum_mean_error[2] > max_yaw_error)
			{
				if (cum_mean_error[0] > max_translation_error)
				{
					cout << "Your x error, " << cum_mean_error[0] << " is larger than the maximum allowable error, " << max_translation_error << endl;
				}
				else if (cum_mean_error[1] > max_translation_error)
				{
					cout << "Your y error, " << cum_mean_error[1] << " is larger than the maximum allowable error, " << max_translation_error << endl;
				}
				else
				{
					cout << "Your yaw error, " << cum_mean_error[2] << " is larger than the maximum allowable error, " << max_yaw_error << endl;
				}
				return -1;
			}
		}
	}

	// Output the runtime for the filter.
	int stop = clock();
	double runtime = (stop - start) / double(CLOCKS_PER_SEC);
	cout << "Runtime (sec): " << runtime << endl;

	// Print success if accuracy and runtime are sufficient
	// NOTE: This isn't just for the starter code
	if (runtime < max_runtime && pf.initialized())
	{
		cout << "Success! Your particle filter passed!" << endl;
	}
	else if (!pf.initialized())
	{
		cout << "This is the starter code. You haven't initialized your filter." << endl;
	}
	else
	{
		cout << "Your runtime " << runtime << " is larger than the maximum allowable runtime, " << max_runtime << endl;
		return -1;
	}

	return 0;
}

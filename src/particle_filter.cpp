/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 200;
	for (int i = 0; i < num_particles; ++i) {
 
        particles[i].id = i;
		particles[i].x = sense_x;
		particles[i].y = sense_y;
		particles[i].theta = sense_theta;
		particles[i].weight = 1;

		// noise ???!?!?!?!!?
	}

	is_initialized = true;

	return

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	for (int i = 0; i < num_particles; ++i) {

        // if no yaw (driving straight):
		if fabs(yaw_rate) == 0 { 

            // use formulas from lessons
			particles[i].x += velocity * delta_t * cos(particles[i].theta); // cos > adjacent > x
            particles[i].y += velocity * delta_t * sin(particles[i].theta); // sin > opposite > y
			particles[i].theta = 0 // going straight, no yaw/theta??

		}
		// if yaw (steering/turning front wheels):
		else {

			particles[i].x  = velocity/yaw_rate * (sin(particles.theta + (yaw_rate * delta_t)) - sin(particle.theta))
	        particles[i].y += velocity/yaw_rate * (cos(particles.theta) - cos(particles.theta + (yaw_rate * delta_t)));
		    particles[i].theta = yaw_rate * delta_t;
		}
	}

	return;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	std:vector<LandmarkObs> closest_landmarks;
	LandmarkObs closest;

	for (int i = 0; i < observations.size()){

		double shortest = 9007199254740991 // big number!

		for (int j = 0; j < predicted.size()) {
			
			double distance = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
			if (distance < shortest) {
				shortest = distance;
				closest = prediction[j];
			}
		}

		closest_landmarks.push_back(closest);
	}

	return closest_landmarks;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
    
	double sigma_x = std_landmark[0];
	double sigma_y = std_landmark[1];


	for (int i = 0; i < particles.size(); ++i) {

        // simpler to call p instead of this index
		Particle p = particles[i];
        
		// transform (translate / rotate) observations from particle POV to Map POV
		std::vector<LandmarkObs> transformed_observations;

	    // for( auto it = x.begin(); it != x.end(); i++)
		for (auto observations: observations) {

			LandmarkObs transformed_observations // to hold transformed observation
			transformed_observations.x  = p.x + (observation.x * cos(p.theta)) - (observation.y * sin(p.theta));
			transformed_observations.y  = p.y + (observation.x * sin(p.theta)) + (observation.y * cos(p.theta));
			transformed_observations.id = observation.id;

			transformed_observations.push_back(transformed_observation);
		}

		// gather all landmarks that can be seen by the particle
		std::vector<LandmarkObs> predicted;
		for (auto landmark: map_landmarks.landmark_list) {

			double distance = dist(p.x, p.y, landmark.x_f, landmark.y_f);

			if (distance < sensor_range) {
				LandmarkObs current_landmark;
				current_landmark.id = landmark.id_i;
				current_landmark.x  = landmark.x_f;
				current_landmark.y  = landmark.y_f;
				predicted.push_back(current_landmark);
			}
		}

		// link closest landmarks to observations of particle
		std::vector<LandmarkObs>  linked_landmarks;
		linked_landmarks = dataAssociation(predicted, transformed_observations);

		double weight = 1;
		for (ing j=0; < linked_landmarks.size(); ++j) {

			double dx = transformed_observations[j].x - linked_landmarks[j].x;
			double dy = transformed_observations[j].y - linked_landmarks[j].y;

			// multivariate-gaussian probability - normalization term
			gauss_norm = 1.0 / (2 * M_PI * sigma_x * sigma_y);
			double exponent = (-dx*dx / (2 * sigma_x ** 2)) + (-dy*dy / (2 * sigma_y ** 2));
			weight *= gauss_norm * exp(-exponent);
		}

		p.weight = probability;
		weights[i] = probability;
	}

	return ;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	discrete_distribution<int> d(weights.begin(), weights.end());
	vector<Particle> weighted_sample(num_particles);

	for (int i = 0; i < num_particles; i++) {

		int j = d(gen);
		weighted_sample[i] = particles[j];

	}

	particles = weighted_sample;

	return;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    // particle: the particle to assign each listed association, and association's (x,y) world coordinates 
	// mapping to associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

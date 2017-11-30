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

    cout << "init" << endl;

    default_random_engine gen;
	normal_distribution<double> x_gauss(x, std[0]);
	normal_distribution<double> y_gauss(y, std[1]);
	normal_distribution<double> theta_gauss(theta, std[2]);

	for (int i = 0; i < num_particles; ++i) {

        cout << "init - num_particles loop" << endl;
		cout << "init - i:" << i << endl;
        cout << "init - x:" << x_gauss << endl;
        cout << "init - y:" << y_gauss << endl;
        cout << "init - theta:" << theta_gauss << endl;

        Particle particle;
        particle.id = i;
		particle.x = x_gauss(gen);
		particle.y = y_gauss(gen);
		particle.theta = theta_gauss(gen);
		particle.weight = 1.0;

        particles.push_back(particle);
		cout << "GOT ITTTT" << endl;
	}

	is_initialized = true;

	return;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    cout << "prediction" << endl;

	for (int i = 0; i < num_particles; ++i) {

        cout << "prediction - num_particles looppppp" << endl;
        cout << "prediction - yaw rate is: " << num_particles << endl;


        // if no yaw (driving straight):
		if (fabs(yaw_rate) == 0) { 
            
			cout << "prediction - yaw rate numparticles" << endl;

            // use formulas from lessons
			particles[i].x += velocity * delta_t * cos(particles[i].theta); // cos > adjacent > x
            particles[i].y += velocity * delta_t * sin(particles[i].theta); // sin > opposite > y
			particles[i].theta = 0; // going straight, no yaw/theta??

		}
		// if yaw (steering/turning front wheels):
		else {

            cout << "prediction - yaw rate else 82" << endl;

			particles[i].x  = velocity/yaw_rate * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
	        particles[i].y += velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
		    particles[i].theta = yaw_rate * delta_t;
		}
	}

	//return;
    cout << "prediction finished" << endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	std:vector<LandmarkObs> closest_landmarks;
	LandmarkObs closest;
	double big_start = 9007199254740991; // big number!


    cout << "dataAssociation" << endl;


	for (int i = 0; i < observations.size(); i++){

        int current_j;
        double current_smallest_error = big_start;
        //cout << "first ass loop" << endl;
		
		for (int j = 0; j < predicted.size(); j++) {
			//cout << "second j ass loop" << endl;
			double error = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
			if (error < current_smallest_error) {
				//cout << "distance is shorter! ass loop j" << endl;
				current_j = j;
				current_smallest_error = error;
				//closest = predicted[j];
			}
		}

        //cout << "before observations[i]" << endl;
		observations[i].id = current_j;
        //cout << "after observations[i]" << endl;

	}

    // change to not return a value
	// return closest_landmarks;

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

    cout << "updateWeights" << endl;


	for (int i = 0; i < particles.size(); ++i) {

        // simpler to call p instead of this index
		Particle p = particles[i];
        
		// transform (translate / rotate) observations from particle POV to Map POV
		std::vector<LandmarkObs> transformed_observations;

	    // for( auto it = x.begin(); it != x.end(); i++)
		for (auto observation: observations) {

			LandmarkObs transformed_observation; // to hold transformed observation
			transformed_observation.x  = p.x + (observation.x * cos(p.theta)) - (observation.y * sin(p.theta));
			transformed_observation.y  = p.y + (observation.x * sin(p.theta)) + (observation.y * cos(p.theta));
			transformed_observation.id = observation.id;

			transformed_observations.push_back(transformed_observation);
		}

		// gather all landmarks that can be seen by the particle
		std::vector<LandmarkObs> landmarks_seen;
		for (auto landmark: map_landmarks.landmark_list) {

			double distance = dist(p.x, p.y, landmark.x_f, landmark.y_f);

			if (distance < sensor_range) {
				LandmarkObs current_landmark;
				current_landmark.id = landmark.id_i;
				current_landmark.x  = landmark.x_f;
				current_landmark.y  = landmark.y_f;
				landmarks_seen.push_back(current_landmark);
			}
		}

		// compare distances from vehicle observation to particle observation
		// update particle weights for use in next round of sampling and observations

		this -> dataAssociation(landmarks_seen, transformed_observations);

		//std::vector<LandmarkObs>  linked_landmarks;
		//inked_landmarks = closest_landmarks;

		double weight = 1;
		//double gauss_norm;


		for (int j=0; j < transformed_observations.size(); ++j) {
            cout << "transformed observations loop" << endl;
            int transformed_id = transformed_observations[j].id;
			double dx = transformed_observations[j].x - landmarks_seen[transformed_id].x;
			double dy = transformed_observations[j].y - landmarks_seen[transformed_id].y;

			// multivariate-gaussian probability - normalization term
			double gauss_norm = 1.0 / (2 * M_PI * sigma_x * sigma_y);
			double exponent = (-dx*dx / (2 * sigma_x * sigma_x)) + (-dy*dy / (2 * sigma_y * sigma_y));
			weight *= gauss_norm * exp(-exponent);
			cout << "weight: " << weight << endl;
			cout << "trans obs loop end" << endl;
		}

        // TODO: create a push_back() instead
		this -> particles[i].weight = weight;
		cout << "thisthisthisthisthisthisthisthisthisthisthis " << this << endl;
		cout << "weight begin" << endl;
		cout << "weight isisisisisis: " << weight << endl;
		this -> weights[i] = weight;
		cout << "this: " << this << endl;
		cout << "weights end" << endl;
	}

	cout << "end updateWeights" << endl;

	//return ;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	discrete_distribution<int> d(weights.begin(), weights.end());
	vector<Particle> weighted_samples(num_particles);
    default_random_engine gen;

    cout << "resample" << endl;


	for (int i = 0; i < num_particles; i++) {
        cout << "begin for loop";
		int j = d(gen);
		weighted_samples[i] = particles[j];

	}

	particles = weighted_samples;

	//return;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    // particle: the particle to assign each listed association, and association's (x,y) world coordinates 
	// mapping to associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    cout << "setAssociations" << endl;
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

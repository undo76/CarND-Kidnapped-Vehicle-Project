/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *
 *  Modified by:
 *      Author: Manuel Santos
 */

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <sstream>
#include <string>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  default_random_engine generator;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  particles = vector<Particle>(num_particles);
  generate(begin(particles), end(particles), [&]() -> Particle {
    Particle p;
    p.x = dist_x(generator);
    p.y = dist_y(generator);
    p.theta = dist_theta(generator);
    return p;
  });

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  default_random_engine generator;
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  for (Particle &p : particles) {
    const double noise_x = dist_x(generator);
    const double noise_y = dist_y(generator);
    const double noise_theta = dist_theta(generator);

    if (yaw_rate < 1.e-5) {
      p.x += velocity * delta_t * cos(p.theta);
      p.y += velocity * delta_t * sin(p.theta);
    } else {
      p.x += (velocity / yaw_rate) *
             (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      p.y += (velocity / yaw_rate) *
             (-cos(p.theta + yaw_rate * delta_t) + cos(p.theta));
    }

    // I am assuming that the magnitude of the noise is proportional
    // to delta_t. Empirically, I get better results, I think that it
    // would be more correct to give the noise of the velocity and the
    // yaw_rate instead.
    p.x += noise_x * delta_t;
    p.y += noise_y * delta_t;
    p.theta += yaw_rate * delta_t + noise_theta * delta_t;
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations) {
  for (auto &o : observations) {
    double best = numeric_limits<double>::max();

    for (auto &p : predicted) {
      double distance = dist(o.x, o.y, p.x, p.y);
      if (distance < best) {
        best = distance;
        o.id = p.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  for (auto &p : particles) {
    // predicted <- landmarks in the sensor_range of the current particle
    vector<LandmarkObs> predicted;
    for (auto &landmark : map_landmarks.landmark_list) {
      if (dist(p.x, p.y, landmark.x_f, landmark.y_f) < sensor_range) {
        LandmarkObs landmark_as_obs;
        landmark_as_obs.id = landmark.id_i;
        landmark_as_obs.x = landmark.x_f;
        landmark_as_obs.y = landmark.y_f;
        predicted.push_back(landmark_as_obs);
      }
    }

    // Convert observations to map coordinates, based in current particle
    // coordinates.
    vector<LandmarkObs> observations_m(observations.size());
    transform(begin(observations), end(observations), begin(observations_m),
              [&](const LandmarkObs &obs) {
                LandmarkObs obs_m;
                obs_m.x = p.x + obs.x * cos(p.theta) - obs.y * sin(p.theta);
                obs_m.y = p.y + obs.x * sin(p.theta) + obs.y * cos(p.theta);
                return obs_m;
              });

    // Set associations and weights of current particle
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    if (predicted.empty()) {
      p.weight = 0.;
    } else {
      // Assign nearest predicted landmark id to observations_m
      dataAssociation(predicted, observations_m);
      p.weight = 1.;
      for (LandmarkObs &obs : observations_m) {
        associations.push_back(obs.id);
        sense_x.push_back(obs.x);
        sense_y.push_back(obs.y);

        auto landmark = find_if(
            begin(predicted), end(predicted),
            [&](LandmarkObs &landmark) { return landmark.id == obs.id; });

        p.weight *=
            multivariate_gaussian_pdf(landmark->x, landmark->y, std_landmark[0],
                                      std_landmark[1], obs.x, obs.y);
      }
    }
    SetAssociations(p, associations, sense_x, sense_y);
  }

  // Store weights
  transform(begin(particles), end(particles), begin(weights),
            [](Particle &particle) -> double { return particle.weight; });
}

void ParticleFilter::resample() {
  default_random_engine generator;
  discrete_distribution<int> index_dist(begin(weights), end(weights));

  vector<Particle> selectedParticles(particles.size());
  generate(begin(selectedParticles), end(selectedParticles),
           [&]() -> Particle { return particles[index_dist(generator)]; });
  particles = selectedParticles;
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     std::vector<int> associations,
                                     std::vector<double> sense_x,
                                     std::vector<double> sense_y) {
  // Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(begin(v), end(v), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(begin(v), end(v), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(begin(v), end(v), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

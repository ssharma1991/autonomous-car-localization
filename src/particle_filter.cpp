/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang, Shashank Sharma
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 20;  // TODO: Set the number of particles
  
  std::default_random_engine gen;
  std::normal_distribution<double> x_dist(0,std[0]);
  std::normal_distribution<double> y_dist(0,std[1]);
  std::normal_distribution<double> t_dist(0,std[2]);
  
  for (int i=0;i<num_particles;i++){
    Particle P;
    P.x=x+x_dist(gen);
    P.y=y+y_dist(gen);
    P.theta=theta+t_dist(gen);
    P.weight=1;
    particles.push_back(P);
  }
  is_initialized=true;
  
  //Print output
  //std::cout<<"init particles"<<std::endl;
  //for (int i=0;i<num_particles;i++){
  //  std::cout<<"x:"<<particles[i].x<<" y:"<<particles[i].y<<" theta:"<<particles[i].theta<<std::endl;
  //}
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  std::normal_distribution<double> x_dist(0,std_pos[0]);
  std::normal_distribution<double> y_dist(0,std_pos[1]);
  std::normal_distribution<double> t_dist(0,std_pos[2]);
  
  for (int i=0;i<particles.size();i++){
    Particle P=particles[i];
    double xf,yf,tf;
    if (fabs(yaw_rate)<pow(10,-20)){
      std::cout<<"Linear Motion Model Used"<<std::endl;
      tf=P.theta;
      xf=P.x+velocity*delta_t*cos(P.theta);
      yf=P.y+velocity*delta_t*sin(P.theta);
    }else{
      tf=P.theta+yaw_rate*delta_t;
      xf=P.x+velocity/yaw_rate*(sin(tf)-sin(P.theta));
      yf=P.y+velocity/yaw_rate*(-cos(tf)+cos(P.theta));
    }
    particles[i].x=xf+x_dist(gen);
    particles[i].y=yf+y_dist(gen);
    particles[i].theta=tf+t_dist(gen);
  }
  
  //Print output
  //std::cout<<"predicted particles"<<std::endl;
  //std::cout<<"yaw rate:"<<yaw_rate<<" vel:"<<velocity<<" dt:"<<delta_t<<std::endl;
  //for (int i=0;i<num_particles;i++){
  //  std::cout<<"x:"<<particles[i].x<<" y:"<<particles[i].y<<" theta:"<<particles[i].theta<<std::endl;
  //}

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
for (int i=0; i<observations.size();i++){
  double d_min=pow(10, 100);
  int index_min=-1;
  for (int j=0; j<predicted.size();j++){
    double d=dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
    //std::cout<<"dist:"<<d<<std::endl;
    if (d<d_min){
      d_min=d;
      index_min=j;
    }
  }
  observations[i].id=index_min;
}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
  //Print data inputs
  //std::cout<<"std_landmark:"<<std_landmark[0]<<","<<std_landmark[1]<<std::endl;
  //for (int i=0;i<observations.size();i++){
  //  std::cout<<"obs_x:"<<observations[i].x<<" obs_y:"<<observations[i].y<<" id:"<<observations[i].id<<std::endl;
  //}
  //for (int i=0;i<map_landmarks.landmark_list.size();i++){
  //  std::cout<<"land_x:"<<map_landmarks.landmark_list[i].x_f<<" land_y:"<<map_landmarks.landmark_list[i].y_f<<" id:"<<map_landmarks.landmark_list[i].id_i<<std::endl;
  //}
  
  //convert Map data into <LandmarkObs> vector
  vector<LandmarkObs> map;
  for (int i=0;i<map_landmarks.landmark_list.size();i++){
    LandmarkObs L;
    L.x=map_landmarks.landmark_list[i].x_f;
    L.y=map_landmarks.landmark_list[i].y_f;
    map.push_back(L);
  }
  
  std::vector<double> new_weights; 
  for (int p=0;p<particles.size();p++){
    vector<LandmarkObs> observations_m;
    for (int i=0;i<observations.size();i++){
      double x_m=particles[p].x+observations[i].x*cos(particles[p].theta)-observations[i].y*sin(particles[p].theta);
      double y_m=particles[p].y+observations[i].x*sin(particles[p].theta)+observations[i].y*cos(particles[p].theta);
      LandmarkObs L;
      L.x=x_m;
      L.y=y_m;
      observations_m.push_back(L);
    }
    
    ParticleFilter::dataAssociation(map,observations_m);
    //for (int j=0;j<observations_m.size();j++){
    //  std::cout<<"obs_x:"<<observations_m[j].x<<" obs_y:"<<observations_m[j].y<<" id:"<<observations_m[j].id<<std::endl;
    //  std::cout<<"land_x:"<<map[observations_m[j].id].x<<" land_y:"<<map[observations_m[j].id].y<<std::endl;
    //}
    
    float wt=1;
    for (int i=0;i<observations_m.size();i++){
      int land_id=observations_m[i].id;
      double sig_x=std_landmark[0];
      double sig_y=std_landmark[1];
      double x_obs=observations_m[i].x;
      double y_obs=observations_m[i].y;
      double mu_x=map[land_id].x;
      double mu_y=map[land_id].y;
      double gauss_norm = 1/(2*M_PI*sig_x*sig_y);
      double exponent= (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))+ (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));
      double w_i= gauss_norm * exp(-exponent);
      //std::cout<<"weight ith observation:"<<w_i<<std::endl;
      wt*=w_i;
    }
    particles[p].weight=wt;
    new_weights.push_back(wt);
    
    //std::cout<<"Updated weight:"<<wt<<std::endl;
  }
weights=new_weights;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  vector <Particle> new_particles;
  std::default_random_engine generator;
  std::discrete_distribution <int> distribution (weights.begin(),weights.end());
  
  //Print probability distribution
  std::cout << "displaying probabilities:";
  for (double x:distribution.probabilities()) std::cout << x << " ";
  std::cout << std::endl;
  
  for (int i=0;i<particles.size();i++){
    int index=distribution(generator);
    new_particles.push_back(particles[index]);
  }
particles=new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
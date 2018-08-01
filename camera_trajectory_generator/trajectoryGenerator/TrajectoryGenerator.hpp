/*
 * This file is part of SceneNet RGB-D.
 *
 * Copyright (C) 2017 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is SemanticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/semantic-fusion/scenenet-rgbd-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <memory>
#include <random>
#include <utility>

class CollisionInterface {
public:
  // True implies a collision, false no collision
  virtual bool collided(const TooN::Vector<3,float> position, const TooN::Vector<3,float> next_position) = 0;
};


class TrajectoryGenerator {
public:
  typedef TooN::Vector<3> Vector;
  typedef std::pair<Vector,Vector> Pose;
  TrajectoryGenerator(std::shared_ptr<CollisionInterface> collision_checker,
                      const int number_attempts = 100);

  bool Initialise(const Vector initial_pose, const Vector lookat_pose);

  Pose Step(const float timestep = 0.025);

  void set_max_speed(const float max_speed) {
    max_speed_ = max_speed;
  }

  void set_max_acceleration(const float max_acceleration) {
    max_acceleration_ = max_acceleration;
  }

  void set_drag(const float drag) {
    drag_ = drag;
  }

  Pose CalculatePose();
private:
  void UpdateObject(const float timestep, Vector& position, Vector& velocity);

  Vector random_unit_vector();

  std::shared_ptr<CollisionInterface> collision_checker_;
  std::default_random_engine random_eng_;

  const int num_attempts_;
  const int num_attempts_until_bounce_;
  float max_speed_;
  float max_acceleration_;
  float drag_;
  bool initialised_;
  int steps_;

  Vector velocity_;
  Vector position_;
  // To perform random rotations, a second randomly walking point is introduced
  // as a target, which the first one looks at.
  // NOTE This rotation update allows the camera to pan, and tilt, but not roll
  // Human cameramen rarely roll much, the footage will look more natural and 
  // in practive an accelerometer would enable us to find the gravity vector 
  // and correct the image without rolling as well.
  // NOTE A roll tolerance could be introduced to slightly change the roll angle
  Vector target_velocity_;
  Vector target_position_;
};


#endif

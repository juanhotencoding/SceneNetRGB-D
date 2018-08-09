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

#include "SceneNetCamera.h"

const float deg2rad = 0.017453292519943295;

SceneNetCamera::SceneNetCamera( Vector3 eye,  Vector3 lookat,  Vector3 up, float hFoV, float vFoV)
    : eye(eye)
    , lookat(lookat)
    , up(up)
{
    ulen_ = tanf(hFoV*0.5f*deg2rad);
    vlen_ = tanf(vFoV*0.5f*deg2rad);
    init();
}

void SceneNetCamera::init()
{
    up = normalize(up);
    lookdir = lookat-eye;
    lookdir = normalize(lookdir);
    camera_u = normalize(cross(lookdir, up));
    camera_v = normalize(cross(camera_u, lookdir));
    // Scale by length in normalised units
    camera_u = camera_u * ulen_;
    camera_v = camera_v * vlen_;
}

void SceneNetCamera::transform(const std::pair<TooN::Vector<3> ,TooN::Vector<3> >& start, const std::pair<TooN::Vector<3> ,TooN::Vector<3> >& end, const float interp)
{
  const TooN::Vector<3> start_trans = start.first;
  const TooN::Vector<3> end_trans = end.first;
  const TooN::Vector<3> new_eye = ((1.0 - interp) * start_trans) + (interp * end_trans);
  eye.x = new_eye[0];
  eye.y = new_eye[1];
  eye.z = new_eye[2];
  const TooN::Vector<3> lookat_start_trans = start.second;
  const TooN::Vector<3> lookat_end_trans = end.second;
  const TooN::Vector<3> new_lookat = ((1.0 - interp) * lookat_start_trans) + (interp * lookat_end_trans);
  lookat.x = new_lookat[0];
  lookat.y = new_lookat[1];
  lookat.z = new_lookat[2];
  // leave up as always positive y
  init();
}

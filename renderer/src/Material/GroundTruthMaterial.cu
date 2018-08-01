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

#include <Util/helpers.h>
#include <optix.h>
#include <optixu/optixu_math_namespace.h>

#include "Renderer/Helpers/random.h"
#include "Renderer/Hitpoint.h"
#include "Renderer/RadiancePRD.h"
#include "Renderer/RayType.h"
#include "rendererConfig.h"

using namespace optix;

rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );
rtDeclareVariable(RadiancePRD, radiancePrd, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float, tHit, rtIntersectionDistance, );

// Globally set for the whole program
rtDeclareVariable(float, MaxDepth, , );
rtDeclareVariable(unsigned int, MaxDepthInUnits, , );
rtDeclareVariable(unsigned int, InstanceID, , );

RT_PROGRAM void closestHitProgram()
{
    if (tHit < MaxDepth) {
        radiancePrd.gt_depth = static_cast<unsigned int>(MaxDepthInUnits * (tHit / MaxDepth));
    } else {
        radiancePrd.gt_depth = 0;
    }
    radiancePrd.instance_id = InstanceID;
}

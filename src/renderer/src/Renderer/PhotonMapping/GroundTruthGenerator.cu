/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include "rendererConfig.h"
#include "Renderer/RadiancePRD.h"
#include "Renderer/Hitpoint.h"
#include "Renderer/RayType.h"
#include "Renderer/Helpers/random.h"
#include "Scene/SceneNetCamera.h"
#include "Util/helpers.h"

using namespace optix;

rtBuffer<unsigned int, 2> rawOutputDepthBuffer;
rtBuffer<uchar4, 2> outputDepthBuffer;
rtBuffer<unsigned int, 2> rawOutputVoxelBuffer;
rtBuffer<uchar4, 2> outputVoxelBuffer;
rtBuffer<unsigned int, 2> rawOutputClassBuffer;
rtBuffer<uchar4, 2> outputClassBuffer;
rtBuffer<unsigned int, 2> rawOutputInstanceBuffer;
rtBuffer<uchar4, 2> outputInstanceBuffer;
rtBuffer<float3, 2> hitpointBuffer;

rtDeclareVariable(unsigned int, MaxDepthInUnits, , );


rtDeclareVariable(rtObject, sceneRootObject, , );
rtDeclareVariable(SceneNetCamera, camera, , );
rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );
rtDeclareVariable(uint2, launchDim, rtLaunchDim, );
rtDeclareVariable(RadiancePRD, radiancePrd, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );

__device__ __inline uchar4 int_to_random_colour(const unsigned int value)
{
    return make_color(make_float3(static_cast<float>((value*2654435761)%213)/213.0,
                                  static_cast<float>((value*2654435761)%217)/217.0,
                                  static_cast<float>((value*2654435761)%231)/231.0));
}

__device__ __inline uchar4 depth_to_colour(const unsigned int depth)
{
    return make_color(make_float3(static_cast<float>(depth) / MaxDepthInUnits));
}


RT_PROGRAM void generateRay()
{
    RadiancePRD radiancePrd;
    float2 screen = make_float2(rawOutputDepthBuffer.size());
    float2 sample = make_float2(0.5,0.5);
    float2 d = ( make_float2(launchIndex) + sample ) / screen * 2.0f - 1.0f;
    float3 rayOrigin = camera.eye;
    float3 rayDirection = normalize(d.x*camera.camera_u + d.y*camera.camera_v + camera.lookdir);
    Ray ray(rayOrigin, rayDirection, RayType::GROUND_TRUTH, 0.001f);
    rtTrace(sceneRootObject, ray, radiancePrd);

    rawOutputDepthBuffer[launchIndex] = radiancePrd.gt_depth;
    rawOutputInstanceBuffer[launchIndex] = radiancePrd.instance_id;
    hitpointBuffer[launchIndex] = radiancePrd.position;
    
    outputDepthBuffer[launchIndex] = depth_to_colour(radiancePrd.gt_depth);
    outputInstanceBuffer[launchIndex] = int_to_random_colour(radiancePrd.instance_id);
}

//
// Miss program
//

RT_PROGRAM void miss()
{
    radiancePrd.gt_depth = 0;
    radiancePrd.instance_id = 0;
}

//
// Exception handler program
//

RT_PROGRAM void exception()
{
    printf("Exception GT!\n");
}

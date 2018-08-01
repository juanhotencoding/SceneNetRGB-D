/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include "Renderer/RadiancePRD.h"
#include "Renderer/ShadowPRD.h"
#include "Renderer/PhotonMapping/PhotonPRD.h"
#include "Renderer/RayType.h"

using namespace optix;

rtDeclareVariable(rtObject, sceneRootObject, , );
rtDeclareVariable(float3, shadingNormal, attribute shadingNormal, ); 
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float, tHit, rtIntersectionDistance, );
rtDeclareVariable(RadiancePRD, radiancePrd, rtPayload, );
rtDeclareVariable(float3, powerPerArea, , );
rtDeclareVariable(float3, Kd, , );
rtDeclareVariable(ShadowPRD, shadowPrd, rtPayload, );
rtDeclareVariable(PhotonPRD, photonPrd, rtPayload, );

/*
// Radiance Program
*/

RT_PROGRAM void closestHitRadiance()
{
    float3 worldShadingNormal = normalize( rtTransformNormal( RT_OBJECT_TO_WORLD, shadingNormal ) );
    if (dot(worldShadingNormal,ray.direction) <= 0) {
    float3 Le = powerPerArea/M_PIf;
    radiancePrd.radiance += radiancePrd.attenuation*Le;
    radiancePrd.flags |= PRD_HIT_EMITTER;
    radiancePrd.lastTHit = tHit;
    } else {
    float3 Le = powerPerArea/M_PIf;
    radiancePrd.radiance += 0.01 * radiancePrd.attenuation*Le;
    radiancePrd.flags |= PRD_HIT_EMITTER;
    radiancePrd.lastTHit = tHit;
    }
}

/*
// Photon Program
*/

RT_PROGRAM void closestHitPhoton()
{
   photonPrd.depth++;
    /*
    float3 hitPoint = ray.origin + tHit*ray.direction;
    Ray newPhoton (hitPoint, ray.direction, RayType::PHOTON, 0.01 );
    rtTrace(sceneRootObject, newPhoton, photonPrd);
    */
}

RT_PROGRAM void gatherAnyHitOnEmitter()
{
    shadowPrd.attenuation = 1.0f;
    rtTerminateRay();
}

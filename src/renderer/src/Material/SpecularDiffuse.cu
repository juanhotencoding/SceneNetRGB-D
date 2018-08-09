/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include "rendererConfig.h"
#include "Renderer/Hitpoint.h"
#include "Renderer/RayType.h"
#include "Renderer/RadiancePRD.h"
#include "Renderer/PhotonMapping/PhotonPRD.h"
#include "Renderer/PhotonMapping/Photon.h"
#include "Renderer/Helpers/random.h"
#include "Renderer/Helpers/helpers.h"
#include "Renderer/Helpers/samplers.h"
#include "Renderer/Helpers/store_photon.h"

using namespace optix;

rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );
rtDeclareVariable(RadiancePRD, radiancePrd, rtPayload, );
rtDeclareVariable(PhotonPRD, photonPrd, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float, tHit, rtIntersectionDistance, );

rtDeclareVariable(float3, geometricNormal, attribute geometricNormal, ); 
rtDeclareVariable(float3, shadingNormal, attribute shadingNormal, ); 

rtBuffer<Photon, 1> photons;
rtBuffer<Hitpoint, 2> raytracePassOutputBuffer;
rtDeclareVariable(rtObject, sceneRootObject, , );
rtDeclareVariable(uint, maxPhotonDepositsPerEmitted, , );
// Possibly needed if we swap to phong or something simpler
//rtBuffer<Light, 1> lights;
rtDeclareVariable(float3, Kd, , );
rtDeclareVariable(float3, Ks, , );
rtDeclareVariable(float, alpha, , );

__device__ __inline float sumVector(const float3 & vect) {
    return vect.x + vect.y + vect.z;
}


/*
// Radiance Program
*/
RT_PROGRAM void closestHitRadiance()
{
    float3 worldShadingNormal = normalize( rtTransformNormal( RT_OBJECT_TO_WORLD, shadingNormal ) );
    float3 hitPoint = ray.origin + tHit*ray.direction;
    radiancePrd.flags |= PRD_HIT_NON_SPECULAR;
    radiancePrd.flags |= PRD_HIT_SPECULAR;
    radiancePrd.normal = worldShadingNormal;
    radiancePrd.position = hitPoint;
    radiancePrd.lastTHit = tHit;
    radiancePrd.depth++;
    // Add diffuse component
    float3 value3 = Kd;
    //value3.x = powf(value3.x,2.4);
    //value3.y = powf(value3.y,2.4);
    //value3.z = powf(value3.z,2.4);
    radiancePrd.attenuation *= value3;
    //radiancePrd.attenuation *= Kd;
    // Add specular component
    float3 reflectedRayDirection = reflect(ray.direction, worldShadingNormal);
    if(radiancePrd.depth <= MAX_SPECULAR_TRACE_DEPTH) {
        float3 cum_radiance = make_float3(0.f);
        for (int i = 0; i < NUM_SPECULAR_SAMPLES; i++) {
            float3 newRayDirection = sampleUnitHemisphereCosExp(reflectedRayDirection, 
                    getRandomUniformFloat2(&radiancePrd.randomState),alpha);
            RadiancePRD subradiance;
            subradiance.attenuation = make_float3( 1.0f );
            subradiance.radiance = make_float3(0.f);
            subradiance.depth = radiancePrd.depth;
            subradiance.flags = 0;
            subradiance.randomState = radiancePrd.randomState;
            Ray newRay = Ray(hitPoint, newRayDirection, RayType::RADIANCE, 0.001, RT_DEFAULT_MAX );
            rtTrace( sceneRootObject, newRay, subradiance);
            radiancePrd.randomState = subradiance.randomState;
            cum_radiance += subradiance.radiance;
        }
        radiancePrd.radiance = (cum_radiance * Ks) / NUM_SPECULAR_SAMPLES;
    }
}


/*
// Photon Program
*/

RT_PROGRAM void closestHitPhoton()
{
    float3 worldShadingNormal = normalize( rtTransformNormal( RT_OBJECT_TO_WORLD, shadingNormal ) );
    float3 hitPoint = ray.origin + tHit*ray.direction;
    float3 newPhotonDirection;

    // Russian roulette sampling
    float specularDiffuseOrAbsorb = getRandomUniformFloat(&photonPrd.randomState);
    float3 both = Kd + Ks;
    float diffuseThreshold = dot(photonPrd.power,Kd) / dot(photonPrd.power,both);
    float specularThreshold = dot(photonPrd.power,Ks) / dot(photonPrd.power,both);
    if (specularDiffuseOrAbsorb < diffuseThreshold) {
        if(photonPrd.depth >= 1 && photonPrd.numStoredPhotons < maxPhotonDepositsPerEmitted) {
            Photon photon (photonPrd.power, hitPoint, ray.direction, worldShadingNormal);
            STORE_PHOTON(photon);
        }
        photonPrd.power *= (Kd/diffuseThreshold);
        newPhotonDirection = sampleUnitHemisphereCos(worldShadingNormal, getRandomUniformFloat2(&photonPrd.randomState));
    } else if (specularDiffuseOrAbsorb < (diffuseThreshold + specularThreshold)) {
        photonPrd.power *= (Ks/specularThreshold);
        newPhotonDirection = reflect(ray.direction, worldShadingNormal);
    } else /* ABSORBED */ {
        return;
    }

    photonPrd.depth++;
    if(photonPrd.depth >= MAX_PHOTON_TRACE_DEPTH || photonPrd.weight < 0.001) {
        return;
    }

    if(photonPrd.numStoredPhotons >= maxPhotonDepositsPerEmitted)
        return;

    optix::Ray newRay( hitPoint, newPhotonDirection, RayType::PHOTON, 0.01 );
    rtTrace(sceneRootObject, newRay, photonPrd);
}

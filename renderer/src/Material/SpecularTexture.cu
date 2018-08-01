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
#include "Renderer/ShadowPRD.h"
#include "Renderer/PhotonMapping/PhotonPRD.h"
#include "Renderer/PhotonMapping/Photon.h"
#include "Renderer/Helpers/random.h"
#include "Renderer/Helpers/helpers.h"
#include "Renderer/Helpers/samplers.h"
#include "Renderer/Helpers/store_photon.h"
#include "Renderer/Light.h"

//#define OPTIX_MATERIAL_DUMP

using namespace optix;

rtDeclareVariable(rtObject, sceneRootObject, , );
rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );
rtDeclareVariable(RadiancePRD, radiancePrd, rtPayload, );
rtDeclareVariable(PhotonPRD, photonPrd, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float, tHit, rtIntersectionDistance, );
rtBuffer<Light, 1> lights;

rtDeclareVariable(float3, geometricNormal, attribute geometricNormal, ); 
rtDeclareVariable(float3, shadingNormal, attribute shadingNormal, ); 
rtDeclareVariable(float3, tangent, attribute tangent, ); 
rtDeclareVariable(float3, bitangent, attribute bitangent, ); 
rtDeclareVariable(float2, textureCoordinate, attribute textureCoordinate, );

rtBuffer<Photon, 1> photons;
rtTextureSampler<uchar4, 2, cudaReadModeNormalizedFloat> diffuseSampler;
rtTextureSampler<uchar4, 2, cudaReadModeNormalizedFloat> normalMapSampler;
rtDeclareVariable(uint, hasNormals, , );
rtDeclareVariable(uint, maxPhotonDepositsPerEmitted, , );
rtDeclareVariable(float3, Ks, , );
rtDeclareVariable(float, alpha, , );

// Radiance Program

__inline__ __device__ float3 getNormalMappedNormal(const float3 & normal,
                                                   const float3 & tangent,
                                                   const float3 & bitangent,
                                                   const float4 & normalMap)
{
    float4 nMap = 2*normalMap - 1;
    float3 N;
    N.x = nMap.x*tangent.x + nMap.y*bitangent.x + nMap.z*normal.x;
    N.y = nMap.x*tangent.y + nMap.y*bitangent.y + nMap.z*normal.y;
    N.z = nMap.x*tangent.z + nMap.y*bitangent.z + nMap.z*normal.z;
    return normalize(N);
}


/* THIS IS CHEAP PHONG SPECULAR - assumes lights are only source of specular
 */
RT_PROGRAM void closestHitRadiance()
{
    float3 worldShadingNormal = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, shadingNormal));
    float3 worldGeometryNormal = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, geometricNormal));
    worldShadingNormal = worldGeometryNormal;
    float3 hitPoint = ray.origin + tHit*ray.direction;

    float3 normal = worldShadingNormal;

    if(hasNormals)
    {
        float3 worldTangent = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, tangent));
        float3 worldBitangent = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, bitangent));
        normal = getNormalMappedNormal(worldShadingNormal, worldTangent, worldBitangent,
                                       tex2D(normalMapSampler, textureCoordinate.x, textureCoordinate.y));
    }


    int numLights = lights.size();
    for(int light_idx = 0; light_idx < numLights; light_idx++) {
        Light & light = lights[light_idx];
        float3 light_pos = light.position;
        float3 light_power = light.power;
        float3 ffnormal = faceforward(worldShadingNormal,-ray.direction,worldGeometryNormal);
        if(light.lightType == Light::AREA)
        {
            const int num_samples = 16;
            float3 stored_radiance = optix::make_float3(0.0);
            for (int sample = 0; sample < num_samples; ++sample) {
                float2 sample = getRandomUniformFloat2(&radiancePrd.randomState);
                float3 light_pos_sample = light_pos + sample.x*light.v1 + sample.y*light.v2;
                float3 l = normalize(light_pos_sample - hitPoint);
                float lightDistance = optix::length(light_pos_sample - hitPoint) + optix::length(tHit*ray.direction);
                float3 n = normalize(normal);
                float3 r = normalize(reflect(l,n));
                float3 v = normalize((ray.origin-hitPoint));
                float dot_val = dot(r,v);
                if (dot_val < 0.0) {
                    ShadowPRD shadowPrd;
                    shadowPrd.attenuation = 1.0f;
                    optix::Ray shadow_ray (hitPoint, l, RayType::SHADOW, 0.05,0.01);
                    rtTrace(sceneRootObject, shadow_ray, shadowPrd);
                    stored_radiance += (shadowPrd.attenuation * Ks * light_power * pow(-dot_val, alpha)) * (1.0 / 500.0 * M_PIf*lightDistance*lightDistance);
                }
            }
            stored_radiance /= num_samples;
            radiancePrd.radiance += stored_radiance;
        }
        else if(light.lightType == Light::POINT)
        {
            float3 l = normalize(light_pos - hitPoint);
            float lightDistance = optix::length(light_pos - hitPoint) + optix::length(tHit*ray.direction);
            float3 n = normalize(normal);
            float3 r = normalize(reflect(l,n));
            float3 v = normalize((ray.origin-hitPoint));
            float dot_val = dot(r,v);
            if (dot_val < 0.0) {
                ShadowPRD shadowPrd;
                shadowPrd.attenuation = 1.0f;
                optix::Ray shadow_ray (hitPoint, l, RayType::SHADOW, 0.01,0.01);
                rtTrace(sceneRootObject, shadow_ray, shadowPrd);
                radiancePrd.radiance += (shadowPrd.attenuation * Ks * light_power * pow(-dot_val, alpha)) * (1.0 / 500.0 * M_PIf*lightDistance*lightDistance);
            }
        }
    }

    radiancePrd.flags |= PRD_HIT_NON_SPECULAR;
    radiancePrd.normal = normal;
    radiancePrd.position = hitPoint;
    radiancePrd.lastTHit = tHit;

    float4 value = tex2D( diffuseSampler, textureCoordinate.x, textureCoordinate.y );
    float3 value3 = make_float3(value.x, value.y, value.z);
    //float3 value3 = make_float3(value.x+0.5, value.y+0.5, value.z+0.5);
    //value3 /= 1.5;
    // Cancel out the standard Camera Response Function which makes colours
    // bleed out
    //value3.x = powf(value3.x,2.4);
    //value3.y = powf(value3.y,2.4);
    //value3.z = powf(value3.z,2.4);
    radiancePrd.attenuation *= value3;
}


/* THIS IS MORE CORRECT BUT MUCH MORE EXPENSIVE
 *
RT_PROGRAM void closestHitRadiance()
{
    float3 worldShadingNormal = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, shadingNormal));
    float3 hitPoint = ray.origin + tHit*ray.direction;

    float3 normal = worldShadingNormal;

    if(hasNormals)
    {
        float3 worldTangent = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, tangent));
        float3 worldBitangent = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, bitangent));
        normal = getNormalMappedNormal(worldShadingNormal, worldTangent, worldBitangent,
                                       tex2D(normalMapSampler, textureCoordinate.x, textureCoordinate.y));
    }

    radiancePrd.flags |= PRD_HIT_NON_SPECULAR;
    radiancePrd.flags |= PRD_HIT_SPECULAR;
    radiancePrd.normal = normal;
    radiancePrd.position = hitPoint;
    radiancePrd.lastTHit = tHit;
    radiancePrd.depth++;

//    if(radiancePrd.flags & PRD_PATH_TRACING)
//    {
//        radiancePrd.randomNewDirection = sampleUnitHemisphereCos(worldShadingNormal, getRandomUniformFloat2(&radiancePrd.randomState));
//    }

    float4 value = tex2D( diffuseSampler, textureCoordinate.x, textureCoordinate.y );
    float3 value3 = make_float3(value.x, value.y, value.z);
    value3.x = powf(value3.x,2.4);
    value3.y = powf(value3.y,2.4);
    value3.z = powf(value3.z,2.4);
    radiancePrd.attenuation *= value3;
    // Add specular component
    float3 reflectedRayDirection = reflect(ray.direction, normal);
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
*/

// Photon Program
RT_PROGRAM void closestHitPhoton()
{
    float3 worldShadingNormal = normalize(rtTransformNormal( RT_OBJECT_TO_WORLD, shadingNormal));
    float3 normal = worldShadingNormal;

    if(hasNormals)
    {
        float3 worldTangent = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, tangent));
        float3 worldBitangent = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, bitangent));
        normal = getNormalMappedNormal(worldShadingNormal, worldTangent, worldBitangent,
                        tex2D(normalMapSampler, textureCoordinate.x, textureCoordinate.y));
    }

    float3 hitPoint = ray.origin + tHit*ray.direction;
    float3 newPhotonDirection;

    /// Record hit if it has bounced at least once
    if(photonPrd.depth >= 1)
    {
        Photon photon (photonPrd.power, hitPoint, ray.direction, worldShadingNormal);
        STORE_PHOTON(photon);
    }

    float4 value = tex2D(diffuseSampler, textureCoordinate.x, textureCoordinate.y);
    float3 value3 = make_float3(value.x, value.y, value.z);
    photonPrd.power *= value3;

#ifdef OPTIX_MATERIAL_DUMP
    for(int i = 0; i<photonPrd.depth;i++) printf("\t");
        printf("Hit diffuse at P(%.2f %.2f %.2f) t=%.3f\n", hitPoint.x, hitPoint.y, hitPoint.z, tHit);
#endif

    photonPrd.weight *= fmaxf(value3);

    // Use russian roulette sampling from depth X to limit the length of the path

    if( photonPrd.depth >= PHOTON_TRACING_RR_START_DEPTH)
    {
        float probContinue = favgf(value3);
        float probSample = getRandomUniformFloat(&photonPrd.randomState);
        if(probSample >= probContinue )
        {
            return;
        }
        photonPrd.power /= probContinue;
    }

    photonPrd.depth++;
    if(photonPrd.depth >= MAX_PHOTON_TRACE_DEPTH || photonPrd.weight < 0.01)
    {
        return;
    }

#if ACCELERATION_STRUCTURE == ACCELERATION_STRUCTURE_UNIFORM_GRID || ACCELERATION_STRUCTURE == ACCELERATION_STRUCTURE_KD_TREE_CPU
    if(photonPrd.numStoredPhotons >= maxPhotonDepositsPerEmitted)
        return;
#endif

    newPhotonDirection = sampleUnitHemisphereCos(worldShadingNormal, getRandomUniformFloat2(&photonPrd.randomState));
    optix::Ray newRay( hitPoint, newPhotonDirection, RayType::PHOTON, 0.01 );
    rtTrace(sceneRootObject, newRay, photonPrd);
    /*
    float3 worldShadingNormal = normalize(rtTransformNormal( RT_OBJECT_TO_WORLD, shadingNormal));
    float3 normal = worldShadingNormal;

    if(hasNormals)
    {
        float3 worldTangent = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, tangent));
        float3 worldBitangent = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, bitangent));
        normal = getNormalMappedNormal(worldShadingNormal, worldTangent, worldBitangent,
                        tex2D(normalMapSampler, textureCoordinate.x, textureCoordinate.y));
    }

    float3 hitPoint = ray.origin + tHit*ray.direction;
    float3 newPhotonDirection;

    float4 value = tex2D(diffuseSampler, textureCoordinate.x, textureCoordinate.y);
    float3 Kd = make_float3(value.x, value.y, value.z);

    // Russian roulette sampling
    float specularDiffuseOrAbsorb = getRandomUniformFloat(&photonPrd.randomState);
    float3 both = Kd + Ks;
    float diffuseThreshold = dot(photonPrd.power,Kd) / dot(photonPrd.power,both);
    float specularThreshold = dot(photonPrd.power,Ks) / dot(photonPrd.power,both);
    if (specularDiffuseOrAbsorb < diffuseThreshold) {
        if(photonPrd.depth >= 1 && photonPrd.numStoredPhotons < maxPhotonDepositsPerEmitted) {
            Photon photon (photonPrd.power, hitPoint, ray.direction, normal);
            STORE_PHOTON(photon);
        }
        photonPrd.power *= Kd;
        newPhotonDirection = sampleUnitHemisphereCos(normal, getRandomUniformFloat2(&photonPrd.randomState));
    } else if (specularDiffuseOrAbsorb < (diffuseThreshold + specularThreshold)) {
        photonPrd.power *= (Ks/specularThreshold);
        newPhotonDirection = reflect(ray.direction, normal);
    } else {
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
    */
}


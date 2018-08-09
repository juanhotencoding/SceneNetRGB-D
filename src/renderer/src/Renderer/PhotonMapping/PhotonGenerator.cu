/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include <cuda_runtime.h>
#include "rendererConfig.h"
#include "Renderer/Light.h"
#include "Renderer/ShadowPRD.h"
#include "Renderer/RayType.h"
#include "Renderer/Helpers/helpers.h"
#include "Renderer/Helpers/samplers.h"
#include "Renderer/Helpers/random.h"
#include "Renderer/PhotonMapping/Photon.h"
#include "Renderer/PhotonMapping/PhotonPRD.h"
#include "Math/Sphere.h"

using namespace optix;

rtDeclareVariable(rtObject, sceneRootObject, , );
rtBuffer<Photon, 1> photons;
rtBuffer<RandomState, 2> randomStates;
rtDeclareVariable(uint, maxPhotonDepositsPerEmitted, , );
rtDeclareVariable(uint, photonLaunchWidth, , );
rtDeclareVariable(uint, photonLaunchHeight, , );
rtBuffer<Light, 1> lights;
rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );
rtDeclareVariable(uint2, launchDim, rtLaunchDim, );
rtDeclareVariable(Sphere, sceneBoundingSphere, , );

rtDeclareVariable(uint, photonIterationNumber, , );

#if ENABLE_RENDER_DEBUG_OUTPUT
rtBuffer<unsigned int, 2> debugPhotonPathLengthBuffer;
#endif

static __device__ void generatePhotonOriginAndDirection(const Light& light, RandomState& state, const Sphere & boundingSphere, 
    float3& origin, float3& direction, float& photonPowerFactor)
{
    origin = light.position;
    float2 sample1 = getRandomUniformFloat2(&state);

    if(light.lightType == Light::POLYAREA)
    {
        float2 sample2 = getRandomUniformFloat2(&state);
        float total = sample2.x+sample2.y;
        sample2.x /= total;
        sample2.y /= total;
        origin += sample1.x*(optix::float3)light.v1 + sample1.y*(optix::float3)light.v2;
        direction = sampleUnitHemisphere(light.normal, sample2);
    }
    else if(light.lightType == Light::AREA)
    {
        float2 sample2 = getRandomUniformFloat2(&state);
        origin += sample1.x*(optix::float3)light.v1 + sample1.y*(optix::float3)light.v2;
        direction = sampleUnitHemisphere(light.normal, sample2);
        //direction = sampleUnitSphere(sample1);
    }
    else if(light.lightType == Light::POINT)
    {
        // If the point light is well outside the bounding sphere, we make sure to emit 
        // only at the scene (to emulate a directional light)
        float3 sceneCenterToLight = light.position-boundingSphere.center;
        float lightDistance = length(sceneCenterToLight);
        sceneCenterToLight /= lightDistance;
        bool lightWellOutsideSphere = (lightDistance > 1.5*boundingSphere.radius);
        // If light is far away, send photons at the scene and reduce the power based on the solid angle of the scene bounding sphere
        if(lightWellOutsideSphere)
        {
            float3 pointOnDisc = sampleDisc(sample1, boundingSphere.center, boundingSphere.radius, sceneCenterToLight);
            direction = normalize(pointOnDisc-origin);
            // Solid angle of sample disc calculated with http://planetmath.org/calculatingthesolidangleofdisc
            photonPowerFactor = (1  - lightDistance * rsqrtf(boundingSphere.radius*boundingSphere.radius+lightDistance*lightDistance)) / 2.f;
        }
        else
        {
            direction = sampleUnitSphere(sample1);
            origin += direction * light.radius;
        }
    }
    else if(light.lightType == Light::SPOT)
    {
        float3 pointOnDisc = sampleDisc(sample1, origin+light.direction, sinf(light.angle/2), light.direction);
        direction = normalize(pointOnDisc-origin);
    }
}

RT_PROGRAM void generator()
{
    const int num_attempts = 8;
    for (int i = 0; i < num_attempts; ++i) {
        PhotonPRD photonPrd;
        photonPrd.pm_index = ((photonIterationNumber * photonLaunchHeight + launchIndex.y) * photonLaunchWidth + launchIndex.x)*maxPhotonDepositsPerEmitted;
        photonPrd.numStoredPhotons = 0;
        photonPrd.depth = 0;
        photonPrd.weight = 1.0f;
        photonPrd.randomState = randomStates[launchIndex];
        if (photonIterationNumber == 0) {
            for(unsigned int j = 0; j < maxPhotonDepositsPerEmitted; ++j) {
                photons[photonPrd.pm_index+j].power = make_float3(0.0f);
                photons[photonPrd.pm_index+j].position = make_float3(0.0f);
            }
        }

        int lightIndex = 0;
        if(lights.size() > 1)
        {
            float sample = getRandomUniformFloat(&photonPrd.randomState);
            lightIndex = intmin((int)(sample*lights.size()), lights.size()-1);
        }

        Light light = lights[lightIndex];
        float powerScale = lights.size();

        photonPrd.power = light.power*powerScale;

        float3 rayOrigin, rayDirection;

        float photonPowerFactor = 1.f;
        generatePhotonOriginAndDirection(light, photonPrd.randomState, sceneBoundingSphere, rayOrigin, rayDirection, photonPowerFactor);
        photonPrd.power *= photonPowerFactor;

        Ray photon = Ray(rayOrigin, rayDirection, RayType::PHOTON, 0.01, RT_DEFAULT_MAX );

        rtTrace( sceneRootObject, photon, photonPrd );

        // Clear photons owned by this thread
        bool waste_photon = false;
        for(unsigned int j = 0; j < maxPhotonDepositsPerEmitted; ++j)
        {
            if(!(fmaxf(photons[photonPrd.pm_index+j].power) > 0.0f)) {
                waste_photon = true;
            }
        }
        randomStates[launchIndex] = photonPrd.randomState;
        if (!waste_photon) {
            break;
        }
    }


#if ENABLE_RENDER_DEBUG_OUTPUT
    debugPhotonPathLengthBuffer[launchIndex] = photonPrd.depth;
#endif

}

rtDeclareVariable(PhotonPRD, photonPrd, rtPayload, );
RT_PROGRAM void miss()
{
    OPTIX_DEBUG_PRINT(photonPrd.depth, "Photon missed geometry.\n");
}

//
// Exception handler program
//

rtDeclareVariable(float3, exceptionErrorColor, , );
RT_PROGRAM void exception()
{
    const unsigned int code = rtGetExceptionCode();

//    RT_EXCEPTION_PROGRAM_ID_INVALID
//    RT_EXCEPTION_TEXTURE_ID_INVALID
//    RT_EXCEPTION_BUFFER_ID_INVALID
//    RT_EXCEPTION_INDEX_OUT_OF_BOUNDS
//    RT_EXCEPTION_STACK_OVERFLOW
//    RT_EXCEPTION_BUFFER_INDEX_OUT_OF_BOUNDS
//    RT_EXCEPTION_INVALID_RAY
//    RT_EXCEPTION_INTERNAL_ERROR
//    RT_EXCEPTION_USER

//    RT_EXCEPTION_ALL
    if( code == RT_EXCEPTION_PROGRAM_ID_INVALID )
    {
        printf("Program Id is invalid\n");
    }
    if( code == RT_EXCEPTION_TEXTURE_ID_INVALID )
    {
        printf("Texture Id is invalid\n");
    }
    if( code == RT_EXCEPTION_BUFFER_ID_INVALID )
    {
        printf("Buffer Id is invalid\n");
    }
    if( code == RT_EXCEPTION_INDEX_OUT_OF_BOUNDS )
    {
        printf("RT_EXCEPTION_INDEX_OUT_OF_BOUNDS\n");
    }
    if( code == RT_EXCEPTION_STACK_OVERFLOW )
    {
        printf("RT_EXCEPTION_STACK_OVERFLOW\n");
    }
    if( code == RT_EXCEPTION_BUFFER_INDEX_OUT_OF_BOUNDS )
    {
        printf("RT_EXCEPTION_BUFFER_INDEX_OUT_OF_BOUNDS\n");
    }
    if( code == RT_EXCEPTION_INVALID_RAY )
    {
        printf("RT_EXCEPTION_INVALID_RAY\n");
    }
    if( code == RT_EXCEPTION_INTERNAL_ERROR )
    {
        printf("RT_EXCEPTION_INTERNAL_ERROR\n");
    }
//    else
//    {
//        printf("Exception User\n");
//    }

//    output_buffer[launch_index] = error;
//    else

    rtPrintExceptionDetails();
    printf("Exception Photon!\n");
    //photonPrd.power = make_float3(0,0,0);
}

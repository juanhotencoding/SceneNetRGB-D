/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/
#include <cuda.h>
#include <curand_kernel.h>
#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include "rendererConfig.h"
#include "Renderer/Helpers/random.h"
#include "Renderer/Light.h"
#include "Renderer/RayType.h"
#include "Renderer/Hitpoint.h"
#include "Renderer/ShadowPRD.h"
#include "Renderer/Helpers/light.h"

using namespace optix;

rtDeclareVariable(rtObject, sceneRootObject, , );
rtBuffer<Hitpoint, 2> raytracePassOutputBuffer;
rtBuffer<float3, 2> directRadianceBuffer;
rtBuffer<RandomState, 2> randomStates;
rtBuffer<Light, 1> lights;
rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );
rtDeclareVariable(ShadowPRD, shadowPrd, rtPayload, );
rtDeclareVariable(uint, localIterationNumber, , );

RT_PROGRAM void kernel()
{
    Hitpoint rec = raytracePassOutputBuffer[launchIndex];
    if (localIterationNumber == 0) {
        directRadianceBuffer[launchIndex] = optix::make_float3(0);
    }
    
    /*
    // Compute direct radiance
    */
    if (rec.flags != PRD_MISS) {
        int numLights = lights.size();
        const int numShadowSamples = NUM_SHADOW_SAMPLES; 
        float3 directRadiance = make_float3(0);
        if(numShadowSamples > 0)
        {
            float3 avgLightRadiance = make_float3(0.f);

            for(int shadowSample = 0; shadowSample < numShadowSamples; shadowSample++)
            {
                float sample = getRandomUniformFloat(&randomStates[launchIndex]);
                int randomLightIndex = intmin(int(sample*numLights), lights.size()-1);
                Light & light = lights[randomLightIndex];
                float scale = numLights;
                float3 lightContrib = getLightContribution(light, rec.position, rec.normal, sceneRootObject, randomStates[launchIndex]);
                avgLightRadiance += scale * lightContrib;
            }

            directRadiance = rec.attenuation*avgLightRadiance/numShadowSamples;
        }
        directRadianceBuffer[launchIndex] += directRadiance;
        directRadianceBuffer[launchIndex] += rec.radiance;
        //Ambient approximation for super fast non-GI rendering
        //directRadianceBuffer[launchIndex] += 0.04 * rec.attenuation;
    } else {
        directRadianceBuffer[launchIndex] += rec.radiance;
        //Ambient approximation for super fast non-GI rendering
        //directRadianceBuffer[launchIndex] += 0.04 * rec.attenuation;
    }
}

RT_PROGRAM void gatherAnyHitOnNonEmitter()
{
    shadowPrd.attenuation = 0.0f;
    rtTerminateRay();
}

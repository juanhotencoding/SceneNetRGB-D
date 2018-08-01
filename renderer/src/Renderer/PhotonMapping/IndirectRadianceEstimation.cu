/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

//#define MAX_DEPTH 20

#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include "rendererConfig.h"
#include "Renderer/Hitpoint.h"
#include "Renderer/Light.h"
#include "Renderer/PhotonMapping/Photon.h"
#include "Renderer/PhotonMapping/PhotonGrid.h"
#include "Renderer/RadiancePRD.h"
#include "Renderer/RayType.h"

using namespace optix;

rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );

rtBuffer<Hitpoint, 2> raytracePassOutputBuffer;
rtBuffer<float3, 2> indirectRadianceBuffer;

rtDeclareVariable(float, alpha, , );
rtDeclareVariable(float, emittedPhotonsPerIterationFloat, , );
rtDeclareVariable(float, maxPhotonIterations, , );
rtDeclareVariable(float, ppmRadius, ,);
rtDeclareVariable(float, ppmRadiusSquared, ,);
rtDeclareVariable(float, ppmRadiusSquaredNew, ,);
rtDeclareVariable(float, numPhotonMaps, , );
rtDeclareVariable(uint, localIterationNumber, , );

rtBuffer<Photon, 1> photonKdTree;

#if ENABLE_RENDER_DEBUG_OUTPUT
rtBuffer<uint, 2> debugIndirectRadianceCellsVisisted;
rtBuffer<uint, 2> debugIndirectRadiancePhotonsVisisted;
#endif

__device__ __inline float validPhoton(const Photon & photon, const float distance2, const float radius2, const float3 & hitNormal)
{
    /*
    float dot = hitNormal.x * -photon.rayDirection[0];
    dot += hitNormal.y * -photon.rayDirection[1];
    dot += hitNormal.z * -photon.rayDirection[2];
    return distance2 <= radius2;// && dot >= 0; 
    */
    return distance2 <= radius2 && dot(-photon.rayDirection, hitNormal) >= 0; 
}

__device__ __inline float3 photonPower(const Photon & photon, const float distance2, const float radius2)
{
    // Use the gaussian filter from Realistic Image Synthesis Using Photon Mapping, Wann Jensen
    const float alpha = 1.818;
    const float beta = 1.953;
    const float expNegativeBeta = 0.141847;
    float weight = alpha*(1 - (1-exp(-beta*distance2/(2*radius2)))/(1-expNegativeBeta));
    /*
    float3 photonPower;
    photonPower.x = static_cast<float>(photon.power[0]) / 255000.0;
    photonPower.y = static_cast<float>(photon.power[1]) / 255000.0;
    photonPower.z = static_cast<float>(photon.power[2]) / 255000.0;
    */
    return photon.power*weight;
}

RT_PROGRAM void kernel()
{
    clock_t start = clock();
    Hitpoint rec = raytracePassOutputBuffer[launchIndex];
    
    float3 indirectAccumulatedPower = make_float3( 0.0f, 0.0f, 0.0f );

    int _dPhotonsVisited = 0;

    //if (false)
    if(rec.flags & PRD_HIT_NON_SPECULAR)
    {
        float radius2 = ppmRadiusSquared;

        // This code is based on the PPM sample in Optix 3.0.0 SDK by NVIDIA

        const size_t MAX_DEPTH = 21;
        unsigned int stack[MAX_DEPTH];
        unsigned int stack_current = 0;
        unsigned int node = 0;
        #define push_node(N) stack[stack_current++] = (N)
        #define pop_node() stack[--stack_current]

        push_node(0);
        do 
        {
            Photon& photon = photonKdTree[ node ];
            _dPhotonsVisited++;
            uint axis = photon.axis;
            if( !( axis & PPM_NULL ) )
            {
                float3 diff = rec.position - photon.position;
                float distance2 = dot(diff, diff);
                if(validPhoton(photon, distance2, radius2, rec.normal))
                {
                    indirectAccumulatedPower += photonPower(photon, distance2, radius2);
                }

                // Recurse
                if( !( axis & PPM_LEAF ) ) {
                    float d;
                    if      ( axis & PPM_X ) d = diff.x;
                    else if ( axis & PPM_Y ) d = diff.y;
                    else                     d = diff.z;
                    // Calculate the next child selector. 0 is left, 1 is right.
                    int selector = d < 0.0f ? 0 : 1;
                    if( d*d < radius2 ) {
                        push_node( (node<<1) + 2 - selector );
                    }
                    node = (node<<1) + 1 + selector;
                } else {
                    node = pop_node();
                }
            } else {
                node = pop_node();
            }
        }
        while ( node && stack_current < MAX_DEPTH );
    }

    float3 indirectRadiance = indirectAccumulatedPower * rec.attenuation * (1.0f/(M_PIf*ppmRadiusSquared)) *  (1.0f/(maxPhotonIterations*emittedPhotonsPerIterationFloat));

    // Add contribution from volumetric radiance
#if ENABLE_PARTICIPATING_MEDIA
    indirectRadiance += rec.volumetricRadiance / emittedPhotonsPerIterationFloat;
#endif

    if (localIterationNumber == 0) {
        indirectRadianceBuffer[launchIndex] = indirectRadiance;
    } else {
        indirectRadianceBuffer[launchIndex] += indirectRadiance;
    }

#if ENABLE_RENDER_DEBUG_OUTPUT
    debugIndirectRadianceCellsVisisted[launchIndex] = _dCellsVisited;
    debugIndirectRadiancePhotonsVisisted[launchIndex] = _dPhotonsVisited;
#endif

}

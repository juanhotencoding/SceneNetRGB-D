/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include <Util/helpers.h>

using namespace optix;

rtBuffer<float3, 2> rawOutputImageBuffer;
rtBuffer<uchar4, 2> outputImageBuffer;
rtBuffer<float3, 2> indirectRadianceBuffer;
rtBuffer<float3, 2> directRadianceBuffer;
rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );
rtDeclareVariable(uint2, launchDim, rtLaunchDim, );
rtDeclareVariable(uint, resDownsample, , );
rtDeclareVariable(uint, numIterationsSqrt, , );

RT_PROGRAM void kernel()
{
    float3 finalRadiance = optix::make_float3(0,0,0);
    for (int i = 0; i < resDownsample; ++i) {
        for (int j = 0; j < resDownsample; ++j) {
            finalRadiance += directRadianceBuffer[launchIndex * resDownsample + optix::make_uint2(i,j)];
            //finalRadiance += directRadianceBuffer[launchIndex * resDownsample + optix::make_uint2(i,j)];
            finalRadiance += indirectRadianceBuffer[launchIndex * resDownsample + optix::make_uint2(i,j)];
        }
    }
    finalRadiance /= (resDownsample * resDownsample * numIterationsSqrt * numIterationsSqrt);
    rawOutputImageBuffer[launchIndex] = finalRadiance;
}

/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#pragma once 

#include "Renderer/RandomState.h"
#include <stdint.h>

static void __device__ initializeRandomState(RandomState* state, unsigned int seed, unsigned int index)
{
    curand_init(seed, index, index, state);
}

// Return a float from 0,1
static __device__ __inline__ float getRandomUniformFloat( RandomState* state )
{
    return curand_uniform(state);
}

// Return a float form a normal distribution N(0,1)
static __device__ __inline__ float getRandomNormalFloat( RandomState* state )
{
    return curand_normal(state);
}

static __device__ __inline__ optix::float2 getRandomUniformFloat2( RandomState* state )
{
    optix::float2 sample;
    sample.x = getRandomUniformFloat(state);
    sample.y = getRandomUniformFloat(state);
    return sample;
}

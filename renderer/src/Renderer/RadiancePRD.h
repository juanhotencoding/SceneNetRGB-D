/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#pragma once
#include "RandomState.h"

struct RadiancePRD
{
    optix::float3 attenuation;
    optix::float3 radiance;
    optix::uint depth;
    optix::float3 position;
    optix::float3 normal;
    optix::uint flags;
    union
    {
        optix::float3 randomNewDirection;
        optix::float3 Le;
    };
    RandomState randomState;
#if ENABLE_PARTICIPATING_MEDIA
    optix::float3 volumetricRadiance;
#endif
    float lastTHit;
    unsigned int gt_depth;
    unsigned int instance_id;
};

static const unsigned int PRD_HIT_EMITTER      = (1u << 31);
static const unsigned int PRD_ERROR            = (1u << 30);
static const unsigned int PRD_MISS             = (1u << 29);
static const unsigned int PRD_HIT_SPECULAR     = (1u << 28);
static const unsigned int PRD_HIT_NON_SPECULAR = (1u << 27);

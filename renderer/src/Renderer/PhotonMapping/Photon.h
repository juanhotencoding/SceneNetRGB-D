/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#pragma once
#include "rendererConfig.h"
#include <stdint.h>

struct Photon
{
#ifdef __CUDACC__
    __device__ __inline Photon(const optix::float3 & power, const optix::float3 & position, const optix::float3 & rayDirection, const optix::float3 & normal)
        : power(power), position(position), rayDirection(rayDirection)
    { }

    __device__ __inline Photon(const Photon& other)
        : power(other.power), position(other.position), rayDirection(other.rayDirection)
    { }

    __device__ __inline Photon(void)
    {

    }
#endif
    optix::float3 position;
    optix::float3 power;
    optix::float3 rayDirection;
    uint8_t   axis;
#if ENABLE_PARTICIPATING_MEDIA
    optix::uint numDeposits;
#endif
};

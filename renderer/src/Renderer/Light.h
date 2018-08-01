/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#pragma once

#include "Math/Vector3.h"

class Light
{
public:
    enum LightType {AREA, POINT, SPOT, POLYAREA};

#ifndef __CUDACC__
    Light(){};
    Light(Vector3 power, Vector3 v1, Vector3 v2, Vector3 v3, Vector3 normal);
    Light(Vector3 power, Vector3 position, Vector3 v1, Vector3 v2);
    Light(Vector3 power, Vector3 position, float radius);
    Light(Vector3 power, Vector3 position, Vector3 direction, float angle);

#endif

    optix::float3 power;
    optix::float3 position;
    optix::float3 v1;
    optix::float3 v2;
    
    float inverseArea;
    union
    {
        float area; // area
        float angle; // spot
        float radius; // point
    };

    union
    {
        optix::float3 normal; // area
        optix::float3 direction; // spot
    };

    LightType lightType;
};

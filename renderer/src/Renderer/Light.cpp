/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#include "Light.h"
#include <iostream>
#include <optixu/optixu_math_namespace.h>

Light::Light(Vector3 power, Vector3 v1_, Vector3 v2_, Vector3 v3_, Vector3 normal_)
: power(power)
,    position(v1_)
,    v1(v2_-v1_)
,    v2(v3_-v1_)
,    lightType(LightType::POLYAREA)
{
    //Using herons formula
    float a_length = (v1_ - v2_).length();
    float b_length = (v1_ - v3_).length();
    float c_length = (v2_ - v3_).length();
    float s = a_length + b_length + c_length;
    normal = optix::normalize(normal_);
    area = sqrtf(s*(s-a_length)*(s-b_length)*(s-c_length));
    inverseArea = 1.0f/area;
}

Light::Light( Vector3 power, Vector3 position, Vector3 v1, Vector3 v2 )
    : power(power),
    position(position),
    v1(v1),
    v2(v2),
    lightType(LightType::AREA)
{
    optix::float3 crossProduct = optix::cross(v1, v2);
    normal = Vector3(optix::normalize(crossProduct));
    std::cout<<"Normal:"<<normal.x<<" "<<normal.y<<" "<<normal.z<<std::endl;
    area = length(crossProduct);
    inverseArea = 1.0f/area;
}

Light::Light(Vector3 power, Vector3 position, float radius)
    : power(power),
    position(position),
    radius(radius),
    lightType(LightType::POINT)
{

}

Light::Light( Vector3 power, Vector3 position, Vector3 direction, float angle )
    : power(power), position(position), direction(direction), angle(angle), lightType(LightType::SPOT)
{
    direction = optix::normalize(direction);
}

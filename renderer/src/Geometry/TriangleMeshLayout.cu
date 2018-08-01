/*
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#include <optix.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h>
#include <optixu/optixu_aabb_namespace.h>

using namespace optix;

rtBuffer<float3> vertexBuffer;
rtBuffer<float3> normalBuffer;
rtBuffer<float3> tangentBuffer;
rtBuffer<float3> bitangentBuffer;
rtBuffer<float2> texCoordBuffer;
rtBuffer<int3> indexBuffer;
rtDeclareVariable(unsigned int, hasTangentsAndBitangents, ,);

rtDeclareVariable(float2, textureCoordinate, attribute textureCoordinate, );
rtDeclareVariable(float3, geometricNormal, attribute geometricNormal, );
rtDeclareVariable(float3, shadingNormal, attribute shadingNormal, );
rtDeclareVariable(float3, tangent, attribute tangent, );
rtDeclareVariable(float3, bitangent, attribute bitangent, );

rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );

RT_PROGRAM void mesh_intersect(int primIdx)
{
    int3 index = indexBuffer[primIdx];

    float3 p0 = vertexBuffer[index.x];
    float3 p1 = vertexBuffer[index.y];
    float3 p2 = vertexBuffer[index.z];

    // Intersect ray with triangle
    float3 n, myshadingNormal;
    float  t, beta, gamma;
    if(intersect_triangle(ray, p0, p1, p2, n, t, beta, gamma))
    {
        if (normalBuffer.size() > 0) {
            float3 n0 = normalBuffer[index.x];
            float3 n1 = normalBuffer[index.y];
            float3 n2 = normalBuffer[index.z];
            myshadingNormal = normalize( n1*beta + n2*gamma + n0*(1.0f-beta-gamma) );
            if (dot(myshadingNormal,ray.direction) >= 0.0) {
                // We add a fallback, but push the intersection distance
                // slightly further back to give priority to any faces which may
                // have normal information encoded the right way
                if (dot(n,ray.direction) >= 0.0) {
                    myshadingNormal = -n;
                    t += 0.001;
                } else {
                    myshadingNormal = n;
                    t += 0.001;
                }
            }
            if (texCoordBuffer.size() == 0) {
                t += 0.001;
            }
        }
        if(rtPotentialIntersection( t ))
        {
            if ( normalBuffer.size() == 0 ) {
                shadingNormal = normalize( n );
            } else {
                shadingNormal = myshadingNormal;

                if(hasTangentsAndBitangents) {
                    float3 t0 = tangentBuffer[index.x];
                    float3 t1 = tangentBuffer[index.y];
                    float3 t2 = tangentBuffer[index.z];
                    tangent = normalize( t1*beta + t2*gamma + t0*(1.0f-beta-gamma) );

                    float3 b0 = bitangentBuffer[index.x];
                    float3 b1 = bitangentBuffer[index.y];
                    float3 b2 = bitangentBuffer[index.z];
                    bitangent = normalize( b1*beta + b2*gamma + b0*(1.0f-beta-gamma) );
                }
            }

            //geometricNormal = shadingNormal;
            geometricNormal = normalize( n );

            // Texture UV coordinates
            if (texCoordBuffer.size() == 0)
            {
                textureCoordinate = make_float2( 0.0f );
            }
            else
            {
                float2 t0 = texCoordBuffer[index.x];
                float2 t1 = texCoordBuffer[index.y];
                float2 t2 = texCoordBuffer[index.z];
                textureCoordinate = t1*beta + t2*gamma + t0*(1.0f-beta-gamma);

                textureCoordinate.y = 1 - textureCoordinate.y;
            }

            rtReportIntersection(0);
        }
    }
}

RT_PROGRAM void mesh_bounds (int primIdx, float result[6])
{
    const int3 index = indexBuffer[primIdx];

    const float3 v0   = vertexBuffer[ index.x ];
    const float3 v1   = vertexBuffer[ index.y ];
    const float3 v2   = vertexBuffer[ index.z ];
    const float  area = length(cross(v1-v0, v2-v0));

    optix::Aabb* aabb = (optix::Aabb*)result;

    if(area > 0.0f && !isinf(area))
    {
        aabb->m_min = fminf( fminf( v0, v1), v2 );
        aabb->m_max = fmaxf( fmaxf( v0, v1), v2 );
    }
    else
    {
        aabb->invalidate();
    }
}

/*
 * Copyright (c) 2008 - 2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and proprietary
 * rights in and to this software, related documentation and any modifications thereto.
 * Any use, reproduction, disclosure or distribution of this software and related
 * documentation without an express license agreement from NVIDIA Corporation is strictly
 * prohibited.
 *
 * TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, THIS SOFTWARE IS PROVIDED *AS IS*
 * AND NVIDIA AND ITS SUPPLIERS DISCLAIM ALL WARRANTIES, EITHER EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE.  IN NO EVENT SHALL NVIDIA OR ITS SUPPLIERS BE LIABLE FOR ANY
 * SPECIAL, INCIDENTAL, INDIRECT, OR CONSEQUENTIAL DAMAGES WHATSOEVER (INCLUDING, WITHOUT
 * LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION, LOSS OF
 * BUSINESS INFORMATION, OR ANY OTHER PECUNIARY LOSS) ARISING OUT OF THE USE OF OR
 * INABILITY TO USE THIS SOFTWARE, EVEN IF NVIDIA HAS BEEN ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGES
 */

//#include <optix.h>
//#include <optixu/optixu_math_namespace.h>
//#include <optixu/optixu_matrix_namespace.h>
//#include <optixu/optixu_aabb_namespace.h>
//#include "intersection_refinement.h"

//using namespace optix;

//// This is to be plugged into an RTgeometry object to represent
//// a triangle mesh with a vertex buffer of triangle soup (triangle list)
//// with an interleaved position, normal, texturecoordinate layout.

//rtBuffer<float3> vertex_buffer;
//rtBuffer<float3> normal_buffer;
//rtBuffer<float2> texcoord_buffer;
//rtBuffer<int3>   vindex_buffer;    // position indices
//rtBuffer<int3>   nindex_buffer;    // normal indices
//rtBuffer<int3>   tindex_buffer;    // texcoord indices

//rtBuffer<uint>   material_buffer; // per-face material index
//rtDeclareVariable(float3, back_hit_point, attribute back_hit_point, );
//rtDeclareVariable(float3, front_hit_point, attribute front_hit_point, );
//rtDeclareVariable(float3, texcoord, attribute texcoord, );
//rtDeclareVariable(float3, geometric_normal, attribute geometric_normal, );
//rtDeclareVariable(float3, shading_normal, attribute shading_normal, );

//rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );


//RT_PROGRAM void mesh_intersect( int primIdx )
//{
//  int3 v_idx = vindex_buffer[primIdx];

//  float3 p0 = vertex_buffer[ v_idx.x ];
//  float3 p1 = vertex_buffer[ v_idx.y ];
//  float3 p2 = vertex_buffer[ v_idx.z ];

//  // Intersect ray with triangle
//  float3 n;
//  float  t, beta, gamma;
//  if( intersect_triangle( ray, p0, p1, p2, n, t, beta, gamma ) ) {

//    if(  rtPotentialIntersection( t ) ) {

//      // Calculate normals and tex coords
//      float3 geo_n = normalize( n );
//      int3 n_idx = nindex_buffer[ primIdx ];
//      if ( normal_buffer.size() == 0 || n_idx.x < 0 || n_idx.y < 0 || n_idx.z < 0 ) {
//        shading_normal = geo_n;
//      } else {
//        float3 n0 = normal_buffer[ n_idx.x ];
//        float3 n1 = normal_buffer[ n_idx.y ];
//        float3 n2 = normal_buffer[ n_idx.z ];
//        shading_normal = normalize( n1*beta + n2*gamma + n0*(1.0f-beta-gamma) );
//      }
//      geometric_normal = geo_n;

//      int3 t_idx = tindex_buffer[ primIdx ];
//      if ( texcoord_buffer.size() == 0 || t_idx.x < 0 || t_idx.y < 0 || t_idx.z < 0 ) {
//        texcoord = make_float3( 0.0f, 0.0f, 0.0f );
//      } else {

//        float2 t0 = texcoord_buffer[ t_idx.x ];
//        float2 t1 = texcoord_buffer[ t_idx.y ];
//        float2 t2 = texcoord_buffer[ t_idx.z ];
//        texcoord = make_float3( t1*beta + t2*gamma + t0*(1.0f-beta-gamma) );
//      }

//      refine_and_offset_hitpoint( ray.origin + t*ray.direction, ray.direction,
//                                  geo_n, p0,
//                                  back_hit_point, front_hit_point );

//      rtReportIntersection(material_buffer[primIdx]);
//    }
//  }
//}


//RT_PROGRAM void mesh_bounds (int primIdx, float result[6])
//{
//  const int3 v_idx = vindex_buffer[primIdx];

//  const float3 v0 = vertex_buffer[ v_idx.x ];
//  const float3 v1 = vertex_buffer[ v_idx.y ];
//  const float3 v2 = vertex_buffer[ v_idx.z ];
//  const float  area = length(cross(v1-v0, v2-v0));

//  optix::Aabb* aabb = (optix::Aabb*)result;

//  if(area > 0.0f && !isinf(area)) {
//    aabb->m_min = fminf( fminf( v0, v1), v2 );
//    aabb->m_max = fmaxf( fmaxf( v0, v1), v2 );
//  } else {
//    aabb->invalidate();
//  }
//}


//#include <optix.h>
//#include <optixu/optixu_math_namespace.h>
//#include <optixu/optixu_matrix_namespace.h>
//#include <optixu/optixu_aabb_namespace.h>

//using namespace optix;

//// This is to be plugged into an RTgeometry object to represent
//// a triangle mesh with a vertex buffer of triangle soup (triangle list)
//// with an interleaved position, normal, texturecoordinate layout.

//rtBuffer<float3> vertex_buffer;
//rtBuffer<float3> normal_buffer;
//rtBuffer<float2> texcoord_buffer;
//rtBuffer<int3>   vindex_buffer;    // position indices
//rtBuffer<int3>   nindex_buffer;    // normal indices
//rtBuffer<int3>   tindex_buffer;    // texcoord indices

//rtBuffer<uint>   material_buffer; // per-face material index
//rtDeclareVariable(float3, texcoord, attribute texcoord, );
//rtDeclareVariable(float3, geometric_normal, attribute geometric_normal, );
//rtDeclareVariable(float3, shading_normal, attribute shading_normal, );
//rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );

//RT_PROGRAM void mesh_intersect( int primIdx )
//{
//  int3 v_idx = vindex_buffer[primIdx];

//  float3 p0 = vertex_buffer[ v_idx.x ];
//  float3 p1 = vertex_buffer[ v_idx.y ];
//  float3 p2 = vertex_buffer[ v_idx.z ];

//  // Intersect ray with triangle
//  float3 n;
//  float  t, beta, gamma;
//  if( intersect_triangle( ray, p0, p1, p2, n, t, beta, gamma ) ) {

//    if(  rtPotentialIntersection( t ) ) {

//      int3 n_idx = nindex_buffer[ primIdx ];

//      if ( normal_buffer.size() == 0 || n_idx.x < 0 || n_idx.y < 0 || n_idx.z < 0 ) {
//        shading_normal = normalize( n );
//      } else {
//        float3 n0 = normal_buffer[ n_idx.x ];
//        float3 n1 = normal_buffer[ n_idx.y ];
//        float3 n2 = normal_buffer[ n_idx.z ];
//        shading_normal = normalize( n1*beta + n2*gamma + n0*(1.0f-beta-gamma) );
//      }
//      geometric_normal = normalize( n );

//      int3 t_idx = tindex_buffer[ primIdx ];
//      if ( texcoord_buffer.size() == 0 || t_idx.x < 0 || t_idx.y < 0 || t_idx.z < 0 ) {
//        texcoord = make_float3( 0.0f, 0.0f, 0.0f );
//      } else {
//        float2 t0 = texcoord_buffer[ t_idx.x ];
//        float2 t1 = texcoord_buffer[ t_idx.y ];
//        float2 t2 = texcoord_buffer[ t_idx.z ];
//        texcoord = make_float3( t1*beta + t2*gamma + t0*(1.0f-beta-gamma) );
//      }

//      rtReportIntersection(material_buffer[primIdx]);
//    }
//  }
//}

//RT_PROGRAM void mesh_bounds (int primIdx, float result[6])
//{
//  const int3 v_idx = vindex_buffer[primIdx];

//  const float3 v0   = vertex_buffer[ v_idx.x ];
//  const float3 v1   = vertex_buffer[ v_idx.y ];
//  const float3 v2   = vertex_buffer[ v_idx.z ];
//  const float  area = length(cross(v1-v0, v2-v0));

//  optix::Aabb* aabb = (optix::Aabb*)result;

//  if(area > 0.0f && !isinf(area)) {
//    aabb->m_min = fminf( fminf( v0, v1), v2 );
//    aabb->m_max = fmaxf( fmaxf( v0, v1), v2 );
//  } else {
//    aabb->invalidate();
//  }
//}

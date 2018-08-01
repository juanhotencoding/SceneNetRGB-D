/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#pragma once

#define PPM_X         ( 1 << 0 )
#define PPM_Y         ( 1 << 1 )
#define PPM_Z         ( 1 << 2 )
#define PPM_LEAF      ( 1 << 3 )
#define PPM_NULL      ( 1 << 4 )

#define MAX_PHOTONS_DEPOSITS_PER_EMITTED 2

#define ENABLE_RENDER_DEBUG_OUTPUT 0
#define ENABLE_PARTICIPATING_MEDIA 0

#define MAX_PHOTON_TRACE_DEPTH (ENABLE_PARTICIPATING_MEDIA?15:7)
#define MAX_RADIANCE_TRACE_DEPTH 7
#define NUM_VOLUMETRIC_PHOTONS 200000
#define PHOTON_TRACING_RR_START_DEPTH 3
#define PATH_TRACING_RR_START_DEPTH 3

#define NUM_SPECULAR_SAMPLES 8
#define MAX_SPECULAR_TRACE_DEPTH 2
#define NUM_SHADOW_SAMPLES 16

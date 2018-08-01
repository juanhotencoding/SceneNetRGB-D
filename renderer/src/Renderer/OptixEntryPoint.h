/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#pragma once

namespace OptixEntryPoint
{
    enum  {
        PPM_RAYTRACE_PASS,
        PPM_PHOTON_PASS,
        PPM_INDIRECT_RADIANCE_ESTIMATION_PASS,
        PPM_DIRECT_RADIANCE_ESTIMATION_PASS,
        PPM_OUTPUT_PASS,
        PPM_POST_PROCESS_PASS,
        PPM_GROUND_TRUTH_PASS,
#if ENABLE_PARTICIPATING_MEDIA
        PPM_CLEAR_VOLUMETRIC_PHOTONS_PASS,
#endif
        NUM_PASSES
    };
}

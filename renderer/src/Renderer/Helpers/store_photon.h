/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#pragma once
#include "Renderer/PhotonMapping/PhotonGrid.h"

// Unfortunately, we need a macro for photon storing code

#define STORE_PHOTON(photon) \
    photons[photonPrd.pm_index + photonPrd.numStoredPhotons] = photon; \
    photonPrd.numStoredPhotons++;

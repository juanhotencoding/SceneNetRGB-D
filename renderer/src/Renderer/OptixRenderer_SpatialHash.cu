/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#include <cstdio>
#include <cmath>

#include <cuda.h>

#include "rendererConfig.h"

#include <optix_world.h>
#include "Renderer/PhotonMapping/Photon.h"
#include "Renderer/PhotonMapping/PhotonGrid.h"
#include "Renderer/Hitpoint.h"
#include "Renderer/OptixRenderer.h"
//#include "Util/sutil/sutil.h"
#include "Renderer/OptixEntryPoint.h"
#include "Renderer/Helpers/optix.h"
#include "Renderer/Helpers/random.h"
#include "Math/Vector3.h"

using namespace optix;

/*
// Initialize random state buffer
*/

static void __global__ initRandomStateBuffer(RandomState* states, unsigned int seed, unsigned int num)
{
    unsigned int index = blockIdx.x*blockDim.x + threadIdx.x;
    if(index < num)
    {
        initializeRandomState(&states[index], seed, index);
    }
}

static void initializeRandomStateBuffer(optix::Buffer & buffer, int numStates)
{
    unsigned int seed = 574133*(unsigned int)clock() + 47844152748*(unsigned int)time(NULL);
    printf("Seeding on %d clock: %d time: %d \n", seed, (unsigned int)clock(), (unsigned int)time(NULL));

    RandomState* states = getDevicePtr<RandomState>(buffer, 0);
    const int blockSize = 256;
    int numBlocks = numStates/blockSize + (numStates % blockSize == 0 ? 0 : 1);
    initRandomStateBuffer<<<numBlocks, blockSize>>>(states, seed, numStates);
    cudaDeviceSynchronize();
    printf("Seeding complete\n");
}

void OptixRenderer::initializeRandomStates()
{
    RTsize size[2];
    m_randomStatesBuffer->getSize(size[0], size[1]);
    int num = size[0]*size[1];
    initializeRandomStateBuffer(m_randomStatesBuffer, num);
}

/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#include "OptixRenderer.h"
#include "Renderer/PhotonMapping/Photon.h"
#include "rendererConfig.h"
#include <limits>

// This can be sped up with pivot sorting only... to decrease caching time
void sort_p(std::vector<Photon> &p, int left, int right, int k, int axis) {
    if (axis == 0) {
      std::sort(p.begin()+left, p.begin()+right,[](Photon const &l, Photon const &r) { return l.position.x < r.position.x; });
    } else if (axis == 1) {
      std::sort(p.begin()+left, p.begin()+right,[](Photon const &l, Photon const &r) { return l.position.y < r.position.y; });
    } else if (axis == 2) {
      std::sort(p.begin()+left, p.begin()+right,[](Photon const &l, Photon const &r) { return l.position.z < r.position.z; });
    }
}

inline RT_HOSTDEVICE int max_component(optix::float3 a)
{
    if(a.x > a.y && a.x  > a.z) {
        return 0;
    } else if(a.y > a.z) {
        return 1;
    }
    return 2;
}

static void buildKDTree(std::vector<Photon> &photons, int start, int end, int depth, Photon* kd_tree, int current_root,
    optix::float3 bbmin, optix::float3 bbmax)
{
    // If we have zero photons, this is a NULL node
    if( end - start == 0 ) {
        kd_tree[current_root].axis = PPM_NULL;
        kd_tree[current_root].power = optix::make_float3( 0.0f );
        return;
    }

    // If we have a single photon
    if( end - start == 1 ) {
        photons[start].axis = PPM_LEAF;
        kd_tree[current_root] = (photons[start]);
        return;
    }

    // Choose axis to split on
    int axis;

    optix::float3 diag = bbmax-bbmin;
    axis = max_component(diag);

    int median = (start+end) / 2;
    switch( axis ) {
    case 0:
        sort_p(photons, start, end-1, median,0);
        photons[median].axis = PPM_X;
        break;
    case 1:
        sort_p(photons, start, end-1, median,1);
        photons[median].axis = PPM_Y;
        break;
    case 2:
        sort_p(photons, start, end-1, median,2);
        photons[median].axis = PPM_Z;
        break;
    }
    optix::float3 rightMin = bbmin;
    optix::float3 leftMax  = bbmax;
    optix::float3 midPoint = (photons[median]).position;
    switch( axis ) {
    case 0:
        rightMin.x = midPoint.x;
        leftMax.x  = midPoint.x;
        break;
    case 1:
        rightMin.y = midPoint.y;
        leftMax.y  = midPoint.y;
        break;
    case 2:
        rightMin.z = midPoint.z;
        leftMax.z  = midPoint.z;
        break;
    }

    kd_tree[current_root] = (photons[median]);
    buildKDTree( photons, start, median, depth+1, kd_tree, 2*current_root+1, bbmin,  leftMax );
    buildKDTree( photons, median+1, end, depth+1, kd_tree, 2*current_root+2, rightMin, bbmax );
}


void OptixRenderer::selectPhotonMap(int map_id) {
  if (map_id != m_photon_map_used) {
    auto selectedHostMap = stored_photon_kdtree_maps_.find(map_id);
    if (selectedHostMap != stored_photon_kdtree_maps_.end()) {
      printf("Selecting photon map %i\n",map_id);
      Photon* kd_data = reinterpret_cast<Photon*>(m_photonKdTree->map());
      std::vector<Photon>& photon_map = selectedHostMap->second;
      const int mapsize = photon_map.size();
      for (int i = 0; i < mapsize; ++i) {
        kd_data[i] = photon_map[i];
      }
      m_photonKdTree->unmap();
      m_photon_map_used = map_id;
    } else {
      printf("Invalid photon map requested: %i\n",map_id);
    }
  }
}

void OptixRenderer::saveHostPhotonMap(int map_id, Photon* kd_data, int size) {
  if (stored_photon_kdtree_maps_.find(map_id) == stored_photon_kdtree_maps_.end()) {
    stored_photon_kdtree_maps_[map_id];
  } else {
    stored_photon_kdtree_maps_[map_id].clear();
  }
  for (int i = 0; i < size; ++i) {
    stored_photon_kdtree_maps_[map_id].push_back(Photon(kd_data[i]));
  }
}

void OptixRenderer::createPhotonKdTreeOnCPU(int map_id)
{
    Photon* photons_host = reinterpret_cast<Photon*>( m_photons->map() );
    Photon* photonKdTree_host = reinterpret_cast<Photon*>( m_photonKdTree->map() );

    std::vector<Photon> photons_vect;
    int numValidPhotons = NUM_PHOTONS;
    int valid_index = 0;
    for (unsigned int i = 0; i < numValidPhotons - 1; ++i) {
        if ((fmaxf(photons_host[i].power) > 0.0f)) {
            photons_host[valid_index] = photons_host[i];
            photons_vect.push_back(photons_host[i]);
            valid_index++;
        }
    }
    std::cout<<"Valid photons"<<photons_vect.size()<<std::endl;

    optix::float3 bbmin = optix::make_float3(0.0f);
    optix::float3 bbmax = optix::make_float3(0.0f);

    bbmin = optix::make_float3(  std::numeric_limits<float>::max() );
    bbmax = optix::make_float3( -std::numeric_limits<float>::max() );

    // Compute the bounds of the photons
    for(unsigned int i = 0; i < photons_vect.size(); ++i)
    {
        optix::float3 position = (photons_vect[i]).position;
        bbmin = fminf(bbmin, position);
        bbmax = fmaxf(bbmax, position);
    }

    // Now build KD tree
    buildKDTree(photons_vect, 0, photons_vect.size(), 0, photonKdTree_host, 0, bbmin, bbmax);
    saveHostPhotonMap(map_id,photonKdTree_host,NUM_PHOTONS);
    m_photon_map_used = map_id;

    m_numberOfPhotonsLastFrame = valid_index;

    m_photonKdTree->unmap();
    m_photons->unmap();
}


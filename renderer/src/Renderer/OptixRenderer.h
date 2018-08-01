/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <optixu/optixpp_namespace.h>
#include <optixu/optixu_aabb_namespace.h>
#pragma GCC diagnostic pop

#include "Math/AAB.h"
#include "Scene/IScene.h"
#include "Scene/SceneNet.h"
#include "Util/ptxpath.h"
#include "Renderer/PhotonMapping/Photon.h"
#include <map>



struct cmpuint3 {
    bool operator()(const optix::uint3& a, const optix::uint3& b) const {
        if (a.x != b.x)
            return (a.x < b.x);
        if (a.y != b.y)
            return (a.y < b.y);
        return (a.z < b.z);
    }
};


enum RenderType {IMAGE = 0,DEPTH,CLASS,INSTANCE,NUM_RENDER_TYPES};

class OptixRenderer {
public:

    OptixRenderer(const int width, const int height, const bool enableGL);
    ~OptixRenderer();

    // This initializes a new context and restarts the scene graph
    void initialize(const int device,int num_maps=-1,int num_iterations=-1);
    void initScene(IScene& scene);
    void calculatePhotonMap();
    void setNumPhotonMaps(int maps);
    void setNumIterations(int itr);

    // The main render calls
    void render(const SceneNetCamera& camera, const std::pair<TooN::Vector<3>,TooN::Vector<3> >& pose_start, const std::pair<TooN::Vector<3>,TooN::Vector<3> >& pose_end, RenderType type = IMAGE, const float shutter_speed = 0.025);

    // This returns the output buffer in a renderable form (i.e. uchar4)
    optix::Buffer getOutputBuffer(RenderType type);
    unsigned int getOutputBufferSizeBytes(RenderType type) const ;
    // This returns the output buffer in its original form (i.e. ints floats)
    optix::Buffer getRawOutputBuffer(RenderType type);
    unsigned int getRawOutputBufferSizeBytes(RenderType type) const ;

    optix::Context getContext();

    void selectPhotonMap(int map_id);
    void saveHostPhotonMap(int map_id, Photon* kd_data, int size);

    const static unsigned int NUM_PHOTON_MAPS;
    const static unsigned int NUM_PHOTONS;
    const static float PPM_INITIAL_RADIUS;
    const static unsigned int PHOTON_GRID_MAX_SIZE;
    const static unsigned int EMITTED_PHOTONS_PER_ITERATION;
    const static unsigned int NUM_PHOTON_ITERATIONS;

private:
    void compile();
    void loadObjGeometry( const std::string& filename, optix::Aabb& bbox );
    void initializeRandomStates();
    void createPhotonKdTreeOnCPU(int map_id = 0);
    optix::Buffer createGLOutputBuffer(RTformat format, unsigned int width, unsigned int height);


    optix::Buffer m_outputBuffer[NUM_RENDER_TYPES];
    optix::Buffer m_rawOutputBuffer[NUM_RENDER_TYPES];
    optix::Buffer m_hitpointBuffer;

    optix::Buffer m_photons;
    optix::Buffer m_photonKdTree;
    optix::Buffer m_hashmapOffsetTable;
    optix::Buffer m_photonsHashCells;
    optix::Buffer m_raytracePassOutputBuffer;
    optix::Buffer m_directRadianceBuffer;
    optix::Buffer m_indirectRadianceBuffer;
    optix::Group m_sceneRootGroup;
    optix::Buffer m_lightBuffer;
    optix::Buffer m_randomStatesBuffer;

    int m_iteration;
    unsigned long long m_numberOfPhotonsLastFrame;
    float m_spatialHashMapCellSize;
    AAB m_sceneAABB;
    optix::uint3 m_gridSize;
    unsigned int m_spatialHashMapNumCells;
    std::map<int, std::vector<Photon> > stored_photon_kdtree_maps_;
    int m_photon_map_used;

    unsigned int m_photonKdTreeSize;
    const unsigned int m_width;
    const unsigned int m_height;
    const bool m_enableGL;

    bool m_initialized;
    int m_device;

    const static unsigned int MAX_BOUNCES;
    const static unsigned int MAX_PHOTON_COUNT;
    const static unsigned int PHOTON_LAUNCH_WIDTH;
    const static unsigned int PHOTON_LAUNCH_HEIGHT;
    const static unsigned int RES_DOWNSAMPLE;
    const static unsigned int NUM_ITERATIONS;
   
    optix::Context m_context;
    int m_optixDeviceOrdinal;

    optix::float3 m_scene_min, m_scene_max;

    int m_num_photon_maps;
    int m_num_iterations;
    bool m_calulate_photon_map_everytime;
};

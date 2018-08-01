/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <limits>
#include <random>

#include <cuda.h>
#include <curand_kernel.h>

#include <GL/glew.h>
#include <GL/gl.h>

#include <optixu/optixu.h>

#include "OptixRenderer.h"
#include "RandomState.h"
#include "Renderer/Hitpoint.h"
#include "Renderer/Light.h"
#include "Renderer/OptixEntryPoint.h"
#include "Renderer/RayType.h"
#include "Renderer/PhotonMapping/Photon.h"
#include "Util/sutil/sutil.h"
#include "rendererConfig.h"

const unsigned int OptixRenderer::PHOTON_GRID_MAX_SIZE = 0;
const unsigned int OptixRenderer::MAX_PHOTON_COUNT = MAX_PHOTONS_DEPOSITS_PER_EMITTED;
const unsigned int OptixRenderer::PHOTON_LAUNCH_WIDTH = 512;
const unsigned int OptixRenderer::PHOTON_LAUNCH_HEIGHT = 1024;
// Ensure that NUM PHOTONS are a power of 2 for stochastic hash
const unsigned int OptixRenderer::EMITTED_PHOTONS_PER_ITERATION = OptixRenderer::PHOTON_LAUNCH_WIDTH*OptixRenderer::PHOTON_LAUNCH_HEIGHT;

const unsigned int OptixRenderer::NUM_PHOTON_ITERATIONS = 32;
const unsigned int OptixRenderer::NUM_PHOTONS = OptixRenderer::EMITTED_PHOTONS_PER_ITERATION*OptixRenderer::NUM_PHOTON_ITERATIONS*OptixRenderer::MAX_PHOTON_COUNT;

const unsigned int OptixRenderer::NUM_PHOTON_MAPS = 4;
const unsigned int OptixRenderer::RES_DOWNSAMPLE = 1;
const unsigned int OptixRenderer::NUM_ITERATIONS = 4;

using namespace optix;

inline unsigned int pow2roundup(unsigned int x)
{
  --x;
  x |= x >> 1;
  x |= x >> 2;
  x |= x >> 4;
  x |= x >> 8;
  x |= x >> 16;
  return x+1;
}

inline float max(float a, float b)
{
  return a > b ? a : b;
}

OptixRenderer::OptixRenderer(const int width, const int height, const bool enableGL) : 
  m_initialized(false),
  m_enableGL(enableGL),
  m_photon_map_used(0),
  m_width(width),
  m_height(height),
  m_calulate_photon_map_everytime(false)
{
    m_context = optix::Context::create();
    if(!m_context)
    {
      assert(false);
    }
}

OptixRenderer::~OptixRenderer()
{
  printf("Context Destroy\n");
  m_context->destroy();
}

void OptixRenderer::initialize(const int device,int num_photon_maps,int num_iterations)
{
  if(m_initialized)
  {
    m_context->destroy();
    m_context = optix::Context::create();
    if(!m_context)
    {
      assert(false);
    }
    m_initialized = false;
  }

  if (num_photon_maps < 0) {
    m_num_photon_maps = NUM_PHOTON_MAPS;
  } else if (num_photon_maps >= 16) {
    m_num_photon_maps = 1;
    m_calulate_photon_map_everytime = true;
  } else {
    m_num_photon_maps = num_photon_maps;
  }
  if (num_iterations < 0) {
    m_num_iterations = NUM_ITERATIONS;
  } else {
    m_num_iterations = num_iterations;
  }

  int max_num_devices    = m_context->getDeviceCount();
  printf("There are %i devices\n",max_num_devices);
  m_device = 0;
  m_context->setDevices(&m_device,&m_device+1);

  m_context->setRayTypeCount(RayType::NUM_RAY_TYPES);
  m_context->setEntryPointCount(OptixEntryPoint::NUM_PASSES);
  m_context->setStackSize(3000);//ENABLE_PARTICIPATING_MEDIA ? 3000 : 1596);

  m_context["maxPhotonDepositsPerEmitted"]->setUint(MAX_PHOTON_COUNT);
  m_context["ppmAlpha"]->setFloat(0);
  m_context["totalEmitted"]->setFloat(0.0f);
  m_context["iterationNumber"]->setFloat(0.0f);
  m_context["localIterationNumber"]->setUint(0);
  m_context["ppmRadius"]->setFloat(0.f);
  m_context["ppmRadiusSquared"]->setFloat(0.f);
  m_context["ppmRadiusSquaredNew"]->setFloat(0.f);
  m_context["ppmDefaultRadius2"]->setFloat(0.f);
  m_context["emittedPhotonsPerIteration"]->setUint(EMITTED_PHOTONS_PER_ITERATION);
  m_context["emittedPhotonsPerIterationFloat"]->setFloat(float(EMITTED_PHOTONS_PER_ITERATION));
  m_context["maxPhotonIterations"]->setFloat(float(NUM_PHOTON_ITERATIONS));
  m_context["numPhotonMaps"]->setFloat(float(m_num_photon_maps));
  m_context["photonLaunchWidth"]->setUint(PHOTON_LAUNCH_WIDTH);
  m_context["photonLaunchHeight"]->setUint(PHOTON_LAUNCH_HEIGHT);
  m_context["participatingMedium"]->setUint(0);

  // An empty scene root node
  optix::Group group = m_context->createGroup();
  m_context["sceneRootObject"]->set(group);

  // Ray Trace OptixEntryPoint Output Buffer
  m_raytracePassOutputBuffer = m_context->createBuffer( RT_BUFFER_INPUT_OUTPUT );
  m_raytracePassOutputBuffer->setFormat( RT_FORMAT_USER );
  m_raytracePassOutputBuffer->setElementSize( sizeof( Hitpoint ) );
  m_raytracePassOutputBuffer->setSize( m_width * RES_DOWNSAMPLE, m_height * RES_DOWNSAMPLE);
  m_context["raytracePassOutputBuffer"]->set( m_raytracePassOutputBuffer );


  // Ray OptixEntryPoint Generation Program
  {
    Program generatorProgram = m_context->createProgramFromPTXFile(ptxpath() +  "RayGeneratorPPM.cu.ptx", "generateRay" );
    Program exceptionProgram = m_context->createProgramFromPTXFile(ptxpath() +  "RayGeneratorPPM.cu.ptx", "exception" );
    Program missProgram = m_context->createProgramFromPTXFile(ptxpath() +  "RayGeneratorPPM.cu.ptx", "miss" );

    m_context->setRayGenerationProgram( OptixEntryPoint::PPM_RAYTRACE_PASS, generatorProgram );
    m_context->setExceptionProgram( OptixEntryPoint::PPM_RAYTRACE_PASS, exceptionProgram );
    m_context->setMissProgram(RayType::RADIANCE, missProgram);
    m_context->setMissProgram(RayType::RADIANCE_IN_PARTICIPATING_MEDIUM, missProgram);
  }

  // Ray GroundTruth Generation Program
  {
    Program generatorProgram = m_context->createProgramFromPTXFile(ptxpath() +  "GroundTruthGenerator.cu.ptx", "generateRay" );
    Program exceptionProgram = m_context->createProgramFromPTXFile(ptxpath() +  "GroundTruthGenerator.cu.ptx", "exception" );
    Program missProgram = m_context->createProgramFromPTXFile(ptxpath() +  "GroundTruthGenerator.cu.ptx", "miss" );

    m_context->setRayGenerationProgram( OptixEntryPoint::PPM_GROUND_TRUTH_PASS, generatorProgram );
    m_context->setExceptionProgram( OptixEntryPoint::PPM_GROUND_TRUTH_PASS, exceptionProgram );
    m_context->setMissProgram(RayType::GROUND_TRUTH, missProgram);
  }

  // Photon Tracing OptixEntryPoint
  {
    Program generatorProgram = m_context->createProgramFromPTXFile(ptxpath() +  "PhotonGenerator.cu.ptx", "generator" );
    Program exceptionProgram = m_context->createProgramFromPTXFile(ptxpath() +  "PhotonGenerator.cu.ptx", "exception" );
    Program missProgram = m_context->createProgramFromPTXFile(ptxpath() +  "PhotonGenerator.cu.ptx", "miss");
    m_context->setRayGenerationProgram(OptixEntryPoint::PPM_PHOTON_PASS, generatorProgram);
    m_context->setMissProgram(OptixEntryPoint::PPM_PHOTON_PASS, missProgram);
    m_context->setExceptionProgram(OptixEntryPoint::PPM_PHOTON_PASS, exceptionProgram);
  }

  m_photons = m_context->createBuffer(RT_BUFFER_OUTPUT);
  m_photons->setFormat( RT_FORMAT_USER );
  m_photons->setElementSize( sizeof( Photon ) );
  m_photons->setSize( NUM_PHOTONS );
  m_context["photons"]->set( m_photons );
  m_context["photonsSize"]->setUint( NUM_PHOTONS );


  m_photonKdTreeSize = NUM_PHOTONS;//pow2roundup( NUM_PHOTONS + 1 ) - 1;
  m_photonKdTree = m_context->createBuffer( RT_BUFFER_INPUT );
  m_photonKdTree->setFormat( RT_FORMAT_USER );
  m_photonKdTree->setElementSize( sizeof( Photon ) );
  m_photonKdTree->setSize( m_photonKdTreeSize );
  m_context["photonKdTree"]->set( m_photonKdTree );

  // Indirect Radiance Estimation Buffer
  m_indirectRadianceBuffer = m_context->createBuffer( RT_BUFFER_INPUT_OUTPUT, RT_FORMAT_FLOAT3, m_width*RES_DOWNSAMPLE, m_height*RES_DOWNSAMPLE );
  m_context["indirectRadianceBuffer"]->set( m_indirectRadianceBuffer );

  // Indirect Radiance Estimation Program
  {
    Program program = m_context->createProgramFromPTXFile(ptxpath() +  "IndirectRadianceEstimation.cu.ptx", "kernel" );
    m_context->setRayGenerationProgram(OptixEntryPoint::PPM_INDIRECT_RADIANCE_ESTIMATION_PASS, program );
  }

  // Direct Radiance Estimation Buffer
  m_directRadianceBuffer = m_context->createBuffer( RT_BUFFER_OUTPUT, RT_FORMAT_FLOAT3, m_width*RES_DOWNSAMPLE, m_height*RES_DOWNSAMPLE );
  m_context["directRadianceBuffer"]->set( m_directRadianceBuffer );

  // Direct Radiance Estimation Program
  {
    Program program = m_context->createProgramFromPTXFile(ptxpath() +  "DirectRadianceEstimation.cu.ptx", "kernel" );
    m_context->setRayGenerationProgram(OptixEntryPoint::PPM_DIRECT_RADIANCE_ESTIMATION_PASS, program );
  }

  // Raw Output Buffers
  {
    m_rawOutputBuffer[IMAGE] = m_context->createBuffer( RT_BUFFER_OUTPUT, RT_FORMAT_FLOAT3, m_width, m_height );
    m_context["rawOutputImageBuffer"]->set(m_rawOutputBuffer[IMAGE]);
    m_rawOutputBuffer[DEPTH] = m_context->createBuffer( RT_BUFFER_OUTPUT, RT_FORMAT_UNSIGNED_INT, m_width, m_height );
    m_context["rawOutputDepthBuffer"]->set(m_rawOutputBuffer[DEPTH]);
    m_hitpointBuffer = m_context->createBuffer( RT_BUFFER_OUTPUT, RT_FORMAT_FLOAT3, m_width, m_height );
    m_context["hitpointBuffer"]->set(m_hitpointBuffer);
    m_rawOutputBuffer[INSTANCE] = m_context->createBuffer( RT_BUFFER_OUTPUT, RT_FORMAT_UNSIGNED_INT, m_width, m_height );
    m_context["rawOutputInstanceBuffer"]->set(m_rawOutputBuffer[INSTANCE]);
  }
  // Renderable output buffers
  {
    if (m_enableGL) {
      m_outputBuffer[IMAGE] = createGLOutputBuffer(RT_FORMAT_UNSIGNED_BYTE4, m_width, m_height);
      m_outputBuffer[DEPTH] = createGLOutputBuffer(RT_FORMAT_UNSIGNED_BYTE4, m_width, m_height);
      m_outputBuffer[INSTANCE] = createGLOutputBuffer(RT_FORMAT_UNSIGNED_BYTE4, m_width, m_height);
    } else {
      m_outputBuffer[IMAGE] = m_context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_UNSIGNED_BYTE4, m_width, m_height);
      m_outputBuffer[DEPTH] = m_context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_UNSIGNED_BYTE4, m_width, m_height); 
      m_outputBuffer[INSTANCE] = m_context->createBuffer(RT_BUFFER_OUTPUT, RT_FORMAT_UNSIGNED_BYTE4, m_width, m_height); 
    }
    m_context["outputImageBuffer"]->set(m_outputBuffer[IMAGE]);
    m_context["outputDepthBuffer"]->set(m_outputBuffer[DEPTH]);
    m_context["outputInstanceBuffer"]->set(m_outputBuffer[INSTANCE]);
  }

  // Output Program
  {
    Program program = m_context->createProgramFromPTXFile(ptxpath() +  "Output.cu.ptx", "kernel" );
    m_context->setRayGenerationProgram(OptixEntryPoint::PPM_OUTPUT_PASS, program );
  }

  // Output Program
  {
    Program program = m_context->createProgramFromPTXFile(ptxpath() +  "PostProcess.cu.ptx", "kernel" );
    m_context->setRayGenerationProgram(OptixEntryPoint::PPM_POST_PROCESS_PASS, program );
  }

  // Random state buffer (must be large enough to give states to both photons and image pixels)
  m_randomStatesBuffer = m_context->createBuffer(RT_BUFFER_INPUT_OUTPUT|RT_BUFFER_GPU_LOCAL);
  m_randomStatesBuffer->setFormat( RT_FORMAT_USER );
  m_randomStatesBuffer->setElementSize( sizeof( RandomState ) );
  int random_size_width = m_width;
  int random_size_height = m_height;
  if (m_width < PHOTON_LAUNCH_WIDTH) {
    random_size_width = PHOTON_LAUNCH_WIDTH;
  }
  if (m_height < PHOTON_LAUNCH_HEIGHT) {
    random_size_height = PHOTON_LAUNCH_HEIGHT;
  }
  m_randomStatesBuffer->setSize(random_size_width,random_size_height);
  m_context["randomStates"]->set(m_randomStatesBuffer);

  // Light sources buffer
  m_lightBuffer = m_context->createBuffer(RT_BUFFER_INPUT);
  m_lightBuffer->setFormat(RT_FORMAT_USER);
  m_lightBuffer->setElementSize(sizeof(Light));
  m_lightBuffer->setSize(1);
  m_context["lights"]->set( m_lightBuffer );

  initializeRandomStates();
  m_initialized = true;
}

void OptixRenderer::compile() {
  std::cout<<"About to validate"<<std::endl;
  m_context->validate();
  std::cout<<"About to compile"<<std::endl;
  m_context->compile();
}

void OptixRenderer::initScene(IScene & scene ) {
  if(!m_initialized) {
    assert(false);
  }
  m_sceneRootGroup = scene.getSceneRootGroup(m_context);
  m_context["sceneRootObject"]->set(m_sceneRootGroup);
  m_sceneAABB = scene.getSceneAABB();
  Sphere sceneBoundingSphere = m_sceneAABB.getBoundingSphere();
  m_context["sceneBoundingSphere"]->setUserData(sizeof(Sphere), &sceneBoundingSphere);

  // Add the lights from the scene to the light buffer
  const std::vector<Light> & lights = scene.getSceneLights();
  if(lights.size() == 0) {
    //Must have some lighting in the scene
    assert(false);
  }

  m_lightBuffer->setSize(lights.size());
  Light* lights_host = (Light*)m_lightBuffer->map();
  memcpy(lights_host, scene.getSceneLights().data(), sizeof(Light)*lights.size());
  m_lightBuffer->unmap();
  compile();
  m_iteration = 0;
}

void OptixRenderer::setNumPhotonMaps(int maps) {
  if (maps >= 8) {
    m_num_photon_maps = 1;
    m_calulate_photon_map_everytime = true;
  } else {
    m_num_photon_maps = maps;
  }
  m_context["numPhotonMaps"]->setFloat(float(m_num_photon_maps));
}

void OptixRenderer::setNumIterations(int itr) {
  m_num_iterations = itr;
}

void OptixRenderer::calculatePhotonMap() {
  for (int photon_map = 0; photon_map < m_num_photon_maps; ++photon_map) {
    printf("Calculating photon map[%i]\n",photon_map);
    for (int iterationNumber = 0; iterationNumber < NUM_PHOTON_ITERATIONS; ++iterationNumber) {
      std::cout<<"Photon Iter Num:"<<iterationNumber<<std::endl;
      m_context["photonIterationNumber"]->setUint(iterationNumber);
      m_context->launch( OptixEntryPoint::PPM_PHOTON_PASS,
                        static_cast<unsigned int>(PHOTON_LAUNCH_WIDTH),
                        static_cast<unsigned int>(PHOTON_LAUNCH_HEIGHT) );
    }
    createPhotonKdTreeOnCPU(photon_map);
  }
}

void OptixRenderer::render(const SceneNetCamera& camera, const std::pair<TooN::Vector<3>,TooN::Vector<3> >& pose_start, const std::pair<TooN::Vector<3>,TooN::Vector<3> >& pose_end, RenderType type, const float shutter_speed)
{
  if(!m_initialized) {
    printf("Returning render\n");
    return;
  }

  double traceStartTime, traceEndTime;
  traceStartTime = sutil::currentTime();
  double t0, t1;

  // This performs the ground truth render instead if desired
  if (type != RenderType::IMAGE) {
    // The ground truth takes the perfect midpoint of the shutter
    SceneNetCamera transformed_camera(camera);
    transformed_camera.transform(pose_start,pose_end,0.5);
    m_context["camera"]->setUserData(sizeof(SceneNetCamera), &transformed_camera);
    m_context->launch(OptixEntryPoint::PPM_GROUND_TRUTH_PASS,
                      static_cast<unsigned int>(m_width),
                      static_cast<unsigned int>(m_height) );
    return;
  }

  const int num_iterations = m_num_iterations;
  const float ppmDecay      = 1.0;
  const float ppmMinRadius  = 0.001;
  // For slower renderings with reduced photon artifacts increase this to 0.05
  const float initppmRadius = 0.01;
  float ppmRadius           = initppmRadius;
  m_context["numIterationsSqrt"]->setUint(num_iterations);
  m_context["resDownsample"]->setUint((unsigned int)RES_DOWNSAMPLE);
  const int photon_map_swap = ((num_iterations * num_iterations + 1) / m_num_photon_maps);

  std::random_device m_rd;
  std::mt19937 gen(m_rd());
  std::uniform_real_distribution<float> dist(0.0,1.0);
  for(int iterationNumber = 0; iterationNumber < num_iterations * num_iterations; ++iterationNumber) {
    // Choose a random camera location, this is for motion blur integration
    float pose_sample = dist(gen);
    SceneNetCamera transformed_camera(camera);
    transformed_camera.transform(pose_start,pose_end,pose_sample);
    m_context["camera"]->setUserData( sizeof(SceneNetCamera), &transformed_camera);

    int localIterationNumber = iterationNumber;
    m_context["iterationNumber"]->setFloat( static_cast<float>(localIterationNumber));
    m_context["localIterationNumber"]->setUint((unsigned int)localIterationNumber);

    // Allow for multiple photon maps
    if (m_calulate_photon_map_everytime) {
      calculatePhotonMap();
    }
    if (iterationNumber % photon_map_swap == 0) {
      int photon_map_used = (m_photon_map_used + 1) % m_num_photon_maps;
      selectPhotonMap(photon_map_used);
      ppmRadius =          initppmRadius;
    }

    // Update PPM Radius for next photon tracing pass
    float ppmRadiusSquared = ppmRadius * ppmRadius;
    m_context["ppmRadius"]->setFloat(ppmRadius);
    m_context["ppmRadiusSquared"]->setFloat(ppmRadiusSquared);
    ppmRadius *= ppmDecay;
    ppmRadius = max(ppmMinRadius,ppmRadius);

    // Trace viewing rays
    {
      t0 = sutil::currentTime();
      m_context->launch( OptixEntryPoint::PPM_RAYTRACE_PASS,
                        static_cast<unsigned int>(RES_DOWNSAMPLE*m_width),
                        static_cast<unsigned int>(RES_DOWNSAMPLE*m_height) );
      t1 = sutil::currentTime();
    }
    // PPM Indirect Estimation (using the photon map)
    {
      t0 = sutil::currentTime();
      m_context->launch(OptixEntryPoint::PPM_INDIRECT_RADIANCE_ESTIMATION_PASS,
                        RES_DOWNSAMPLE*m_width, RES_DOWNSAMPLE*m_height);
      t1 = sutil::currentTime();
    }
    // Direct Radiance Estimation
    {
      t0 = sutil::currentTime();
      m_context->launch(OptixEntryPoint::PPM_DIRECT_RADIANCE_ESTIMATION_PASS,
                        RES_DOWNSAMPLE*m_width, RES_DOWNSAMPLE*m_height);
      t1 = sutil::currentTime();
    }
    m_iteration++;
  }
  // Combine indirect and direct buffers in the output buffer
  m_context->launch(OptixEntryPoint::PPM_OUTPUT_PASS, m_width, m_height);
  m_context->launch(OptixEntryPoint::PPM_POST_PROCESS_PASS, m_width, m_height);
  traceEndTime = sutil::currentTime();
  printf("render complete in %.4f.\n", traceEndTime-traceStartTime);
}

static inline unsigned int max(unsigned int a, unsigned int b) {
  return a > b ? a : b;
}

optix::Context OptixRenderer::getContext() { 
  return m_context; 
}

optix::Buffer OptixRenderer::getOutputBuffer(RenderType type) {
  return m_outputBuffer[type];
}

unsigned int OptixRenderer::getOutputBufferSizeBytes(RenderType type) const {
  // All output buffers are uchar 4 type
  return m_width * m_height * sizeof(optix::uchar4);
}

optix::Buffer OptixRenderer::getRawOutputBuffer(RenderType type) {
  return m_rawOutputBuffer[type];
}

unsigned int OptixRenderer::getRawOutputBufferSizeBytes(RenderType type) const {
  // The raw image is in float3 format, the rest are ints
  if (type == IMAGE) {
    return m_width * m_height * sizeof(optix::float3);
  }
  return m_width * m_height * sizeof(optix::uint);
}

optix::Buffer OptixRenderer::createGLOutputBuffer(RTformat format, unsigned int width, unsigned int height)  {
  optix::Buffer buffer;
  // First allocate the memory for the GL buffer, then attach it to OptiX.
  GLuint vbo = 0;
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  size_t element_size;
  m_context->checkError(rtuGetSizeForRTformat(format, &element_size));
  glBufferData(GL_ARRAY_BUFFER, element_size * width * height, 0, GL_STREAM_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  buffer = m_context->createBufferFromGLBO(RT_BUFFER_OUTPUT, vbo);
  buffer->setFormat(format);
  buffer->setSize( width, height );
  return buffer;
}

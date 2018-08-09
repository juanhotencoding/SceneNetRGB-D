/*
 * This file is part of SceneNet RGB-D.
 *
 * Copyright (C) 2017 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is SemanticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/semantic-fusion/scenenet-rgbd-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include <algorithm>
#include <dirent.h>
#include <iomanip>
#include <iostream>
#include <utility>

#include <optixu/optixpp_namespace.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>

#include "SceneNet.h"
#include "Renderer/RayType.h"
#include "Geometry/SphereInstance.h"
#include "Geometry/Transform.h"
#include "Geometry/AABInstance.h"
#include "Math/Sphere.h"
#include "Math/Vector3.h"
#include "Material/Glass.h"
#include "Material/Texture.h"
#include "Material/SpecularTexture.h"
#include "Material/ParticipatingMedium.h"
#include "Material/Mirror.h"
#include "Material/DiffuseEmitter.h"


SceneNet::SceneNet(std::string save_base,std::string layout_file, std::string obj_base_folder,std::string layout_base_folder,int seed) {
    save_folder_ = save_base;
    obj_base_folder_ = obj_base_folder;
    layout_base_folder_ = layout_base_folder;
    init_traj_gen = false;
    instance_id_ = 1;
    ParseLayoutFile(layout_file);
    std::random_device rd;
    //This is used if there is no seed
    int random_number = rd();
    if (seed != 0) {
      random_number = seed;
    }
    std::cout<<"RANDOM TEXTURE AND LIGHT NUMBER:"<<random_number<<std::endl;
    rand_gen_.reset(new std::mt19937(random_number));
    //Parse wnid file
    time_ = 0.0;
    std::ifstream file(obj_base_folder+ "filtered_model_info_and_texture.txt");
    int line_number = 1;
    std::string str;
    while (std::getline(file, str)) {
      std::istringstream buffer(str);
      std::string model_id,wnlemmas,wordtags;
      if (line_number > 1) {
        buffer >> model_id >> wnlemmas >> wordtags;
        shapenet_id_to_class_[model_id] = std::make_pair(wnlemmas,wordtags);
        shapenet_index_to_modelid_.push_back(model_id);
      }
      line_number++;
    }
    // Parse scenenet mat->wnid file
    std::ifstream mat_file(layout_base_folder+ "scenenet_mat_to_wnid.txt");
    while (std::getline(mat_file, str)) {
      std::istringstream buffer(str);
      std::string mat_prefix, wnid;
      buffer >> mat_prefix >> wnid;
      scenenetlayout_mat_to_wnid[mat_prefix] = wnid;
    }
    // Open up log file
    pose_number_ = 0;
    std::cout<<save_folder_<<"/render_info.log"<<std::endl;
    log_file.open (save_folder_+"/render_info.log");
    log_file<<"text_layout_file:"<<layout_file<<std::endl;
}

SceneNet::~SceneNet() {
  log_file.close();
}

optix::Group SceneNet::getSceneRootGroup(optix::Context & context) {
    // Set ground truth rendering parameters
    // For depth this means millimeters
    context["MaxDepth"]->setFloat(65.535);
    context["MaxDepthInUnits"]->setUint(65535);
    // Setup background lighting
    {
        std::uniform_real_distribution<float> brightness(0.0,1.5);
        std::uniform_real_distribution<float> light_colour_scalar(0.5,1.0);
        float bright = brightness(*rand_gen_);
        float r_scalar = 2.0;//bright * light_colour_scalar(*rand_gen_);
        float g_scalar = 2.0;//bright * light_colour_scalar(*rand_gen_);
        float b_scalar = 2.0;//bright * light_colour_scalar(*rand_gen_);
        context["defaultOutdoorLight"]->setFloat(optix::make_float3(r_scalar,g_scalar,b_scalar));
    }

    m_pgram_bounding_box = context->createProgramFromPTXFile(ptxpath() + "parallelogram.cu.ptx", "bounds" );
    m_pgram_intersection = context->createProgramFromPTXFile(ptxpath() + "parallelogram.cu.ptx", "intersect" );
    m_pgram_sphere_bounding_box = context->createProgramFromPTXFile(ptxpath() + "Sphere.cu.ptx", "bounds" );
    m_pgram_sphere_intersection = context->createProgramFromPTXFile(ptxpath() + "Sphere.cu.ptx", "intersect" );
    m_pgram_mesh_bounding_box = context->createProgramFromPTXFile(ptxpath() + "TriangleMesh.cu.ptx", "mesh_bounds" );
    m_pgram_mesh_intersection = context->createProgramFromPTXFile(ptxpath() + "TriangleMesh.cu.ptx", "mesh_intersect" );

    // create geometry instances
    std::vector<optix::GeometryInstance> gis;
    // Scene boundaries with buffer
    m_sceneAABB.min = Vector3(-10,-2,-10) - 1.0f;
    m_sceneAABB.max = Vector3(10,8,10) + 1.0f;

    std::vector<optix::GeometryInstance> obj_ginstances;
    if (createOBJLayout(layout_obj_, context, obj_ginstances)) {
      for(int i =0; i < obj_ginstances.size(); i++) {
        gis.push_back(obj_ginstances.at(i));
      }
    }

    // Random Lights
    m_sceneLights.clear();
    std::uniform_int_distribution<> num_lights_dist(1,5);
    std::uniform_real_distribution<float> light_y_pos_dist(0.5,0.95);
    std::uniform_real_distribution<float> light_xz_pos_dist(0.2,0.8);
    std::uniform_real_distribution<float> light_power_dist(0.01e2f,2.0e2);
    std::uniform_real_distribution<float> light_colour_scalar(0.5,1.0);
    std::uniform_real_distribution<float> light_radius(0.05,0.1);
    std::uniform_real_distribution<float> light_size_dist(0.2,0.7);
    std::uniform_int_distribution<> light_type_dist(0,1);
    int num_lights = num_lights_dist(*rand_gen_);
    float total_power = light_power_dist(*rand_gen_);

    std::cout<<"Num lights"<<num_lights<<std::endl;
    for (int light_id = 0; light_id < num_lights; ++light_id) {
      int light_type = light_type_dist(*rand_gen_);
      float light_y_pos = m_layout_min.y + light_y_pos_dist(*rand_gen_) * m_layout_size.y;
      float light_x_pos = m_layout_min.x + light_xz_pos_dist(*rand_gen_) * m_layout_size.x;
      float light_z_pos = m_layout_min.z + light_xz_pos_dist(*rand_gen_) * m_layout_size.z;
      float r_scalar = light_colour_scalar(*rand_gen_);
      float g_scalar = light_colour_scalar(*rand_gen_);
      float b_scalar = light_colour_scalar(*rand_gen_);
      optix::float3 power  = optix::make_float3(total_power * r_scalar / num_lights, total_power * g_scalar / num_lights,total_power * b_scalar / num_lights);
      optix::float3 anchor = optix::make_float3( light_x_pos , light_y_pos, light_z_pos);
      if (light_type==0) {
        const float radius = light_radius(*rand_gen_);
        Light light(power, anchor, radius);
        m_sceneLights.push_back(light);
      } else {
        optix::float3 v1     = optix::make_float3( 0.0f, 0.0f, -light_size_dist(*rand_gen_));
        optix::float3 v2     = optix::make_float3(light_size_dist(*rand_gen_), 0.0f, 0.0f);
        Light light(power, anchor, v1, v2);
        m_sceneLights.push_back(light);
      }
    }

    for(int i = 0; i < m_sceneLights.size(); i++) {
      if (m_sceneLights[i].lightType == Light::POINT) {
        log_file<<"instance:"<< instance_id_ << ";03665924;lightbulb;"<<
          "position["<<m_sceneLights[i].position.x<<","<<m_sceneLights[i].position.y<<","<<m_sceneLights[i].position.z<<"];radius["<<m_sceneLights[i].radius<<"];power["<<m_sceneLights[i].power.x<<","<<m_sceneLights[i].power.y<<","<<m_sceneLights[i].power.z<<"]" << std::endl;
        DiffuseEmitter emitter = DiffuseEmitter(m_sceneLights[i].power, Vector3(1),0,instance_id_);
        emitter.setInverseArea(5.0f);//10000.0/(4.0 * 3.14159 * m_sceneLights[i].radius * m_sceneLights[i].radius));

        Vector3 ball_pos = m_sceneLights[i].position;// + Vector3(0.2,0.0,0.0);
        gis.push_back(createSphere(context, ball_pos, m_sceneLights[i].radius, emitter));
      instance_id_++;
      } else if (m_sceneLights[i].lightType == Light::AREA) {
        log_file<<"instance:"<< instance_id_ << ";03665924;lightbulb;"<<
          "position["<<m_sceneLights[i].position.x<<","<<m_sceneLights[i].position.y<<","<<m_sceneLights[i].position.z<<"];v1["<<m_sceneLights[i].v1.x<<","<<m_sceneLights[i].v1.y<<","<<m_sceneLights[i].v1.z<<"];v2["<<
m_sceneLights[i].v2.x<<","<<m_sceneLights[i].v2.y<<","<<m_sceneLights[i].v2.z<<"];power["<<m_sceneLights[i].power.x<<","<<m_sceneLights[i].power.y<<","<<m_sceneLights[i].power.z<<"]" << std::endl;
        DiffuseEmitter emitter = DiffuseEmitter(m_sceneLights[i].power, Vector3(1),0,instance_id_);
        emitter.setInverseArea(5.0f);
        gis.push_back(createParallelogram(context, m_sceneLights[i].position,
                                                   m_sceneLights[i].v1,
                                                   m_sceneLights[i].v2,
                                          emitter));
      instance_id_++;
      } 
    }

    // We now ignore this in favour of a log which links instances to classes
    std::string mesh_path = obj_base_folder_;

    for(int obj = 0; obj < map_object_info_.size(); obj++) {
      ObjectInfo object = map_object_info_[obj];
      obj_ginstances.clear();
      m_materials.clear();
      const std::string model_string = object.object_id;
      // To view a single object set it here
      std::cout<<"Model string:"<<model_string<<" instance:"<<instance_id_<<std::endl;
      if (createOBJMesh(object, context, obj_ginstances)) {
        std::cout<<"instance:"<< instance_id_ << ";" << shapenet_id_to_class_[model_string].first << ";" << shapenet_id_to_class_[model_string].second <<";"<<model_string<<std::endl;
        log_file<<"instance:"<< instance_id_ << ";" << shapenet_id_to_class_[model_string].first << ";" << shapenet_id_to_class_[model_string].second <<";"<<model_string<<std::endl;
        for(int i =0; i < obj_ginstances.size(); i++) {
          gis.push_back(obj_ginstances.at(i));
        }
        instance_id_++;
      }
    }

    // Create geometry group
    optix::GeometryGroup geometry_group = context->createGeometryGroup();
    geometry_group->setChildCount( static_cast<unsigned int>( gis.size()) );
    for (int i = 0; i < gis.size(); ++i)
        geometry_group->setChild(i, gis[i]);

    geometry_group->setAcceleration(context->createAcceleration("Sbvh", "Bvh"));

    optix::Group gro = context->createGroup();
    gro->setChildCount(1);
    gro->setChild(0, geometry_group);

    optix::Acceleration acceleration = context->createAcceleration("Sbvh", "Bvh");
    gro->setAcceleration(acceleration);

    return gro;
}

void SceneNet::ParseLayoutFile(std::string layout_file) {
  // Parse layout file
  std::ifstream layout_stream(layout_file);
  std::string str; 
  int line_number = 0;
  int object_id_count = 0;
  std::string obj_id;
  int object_scale_count = 0;
  float obj_scale;
  int object_transform_count = 0;
  float pose[6];
  float timestamp;
  float obj_transform[4*4];
  obj_transform[12] = 0.0;
  obj_transform[13] = 0.0;
  obj_transform[14] = 0.0;
  obj_transform[15] = 1.0;
  int object_wnid_count = 0;
  bool object_mode = false;
  bool scale_mode = false;
  bool transform_mode = false;
  bool wnid_mode = false;
  bool pose_mode = false;
  const int header_point = 1;
  while (std::getline(layout_stream, str)) {
    //  std::cout<<"get line:"<<str<<std::endl;
    std::istringstream buffer(str);
    if (str.empty())
      continue;
    //std::cout<<str<<std::endl;
    if (line_number < header_point) {
      //std::cout<<"get line number:"<<str<<std::endl;
      std::string tmp;
      buffer >> tmp;
      buffer >> tmp;
      layout_obj_ = layout_base_folder_ + tmp;
    } else {
      if (str == "object" || str == "end") {
        std::cout<<"object"<<std::endl;
        if (object_id_count != 1 || object_scale_count != 1 || object_transform_count != 3) {
            std::cout<<"problem?"<<std::endl;
          if (line_number > header_point) {
            std::cout<<"Error - misaligned text info - line:"<<line_number<<std::endl;
          }
        } else {
            ObjectInfo this_object;
            this_object.object_id = obj_id;
            this_object.scale = obj_scale;
            this_object.transform = optix::Matrix4x4(obj_transform);
            std::cout<<"Adding object:"<<this_object.object_id<<std::endl;
            std::cout<<"Adding object scale:"<<this_object.scale<<std::endl;
            std::cout<<"Adding object transform:"<<std::endl;
            map_object_info_.push_back(this_object);
	
            object_id_count = 0;
            object_scale_count = 0;
            object_transform_count = 0;
            object_wnid_count = 0;
        }
        object_mode = true;
        wnid_mode = scale_mode = transform_mode = false;
      } else if (str == "scale") {
        scale_mode = true;
        wnid_mode = object_mode = transform_mode = false;
      } else if (str == "transformation") {
        transform_mode = true;
        wnid_mode = object_mode = scale_mode = false;
      } else if (str == "wnid") {
        wnid_mode = true;
        transform_mode = object_mode = scale_mode = false;
      } else if (str == "#first three elements, eye and last three, lookAt") {
          if (object_id_count > 0) {
            ObjectInfo this_object;
            this_object.object_id = obj_id;
            this_object.scale = obj_scale;
            this_object.transform = optix::Matrix4x4(obj_transform);
            std::cout<<"Adding object:"<<this_object.object_id<<std::endl;
            std::cout<<"Adding object scale:"<<this_object.scale<<std::endl;
            std::cout<<"Adding object transform:"<<std::endl;
            map_object_info_.push_back(this_object);
            object_id_count = 0;
            object_scale_count = 0;
            object_transform_count = 0;
            object_wnid_count = 0;
         }
        std::getline(layout_stream, str);
        std::getline(layout_stream, str);
        pose_mode = true;
        wnid_mode = transform_mode = object_mode = scale_mode = false;
      } else if (object_mode) {
        object_id_count++;
        obj_id = str;
      } else if (scale_mode) {
        object_scale_count++;
        buffer>>obj_scale;
      } else if (transform_mode) {
        buffer>>obj_transform[object_transform_count*4+0]>>
                obj_transform[object_transform_count*4+1]>>
                obj_transform[object_transform_count*4+2]>>
                obj_transform[object_transform_count*4+3];
        object_transform_count++;
      } else if (wnid_mode) {
        object_wnid_count++;
      } else if (pose_mode) {
        buffer>>timestamp>>pose[0]>>
                pose[1]>>
                pose[2]>>
                pose[3]>>
                pose[4]>>
                pose[5];
        TooN::Vector<3> eye = TooN::makeVector(pose[0],pose[1],pose[2]);
        TooN::Vector<3> lookat = TooN::makeVector(pose[3],pose[4],pose[5]);
        std::pair<TooN::Vector<3>,TooN::Vector<3>> pose = std::make_pair(eye,lookat);
        saved_poses_.push_back(pose);
        saved_timestamps_.push_back(timestamp);
      } else {
        std::cout<<"Error - unexpected line:"<<line_number<<std::endl;
      }
    }
    line_number++;
  }
}


bool SceneNet::createOptixGeometry(aiMesh* mesh,optix::Context& context,std::vector<optix::GeometryInstance>& obj_ginstances) {
  const unsigned int numFaces    = mesh->mNumFaces;
  const unsigned int numVertices = mesh->mNumVertices;
  const int num_normals = mesh->mNumFaces*3;
  optix::Geometry geometry = context->createGeometry();
  geometry->setPrimitiveCount(numFaces);
  geometry->setIntersectionProgram(m_pgram_mesh_intersection);
  geometry->setBoundingBoxProgram(m_pgram_mesh_bounding_box);

  /// Create vertex, normal and texture buffer
  optix::Buffer vertexBuffer = context->createBuffer( RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, numVertices);
  optix::float3* vertexBuffer_Host = static_cast<optix::float3*>( vertexBuffer->map() );

  optix::Buffer normalBuffer = context->createBuffer( RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, numVertices);

  optix::float3* normalBuffer_Host = static_cast<optix::float3*>( normalBuffer->map() );

  geometry["vertexBuffer"]->setBuffer(vertexBuffer);
  geometry["normalBuffer"]->setBuffer(normalBuffer);

  /// Copy vertex and normal buffers
  memcpy( static_cast<void*>( vertexBuffer_Host ),
         static_cast<void*>( mesh->mVertices ),
         sizeof( optix::float3 )*numVertices);
  vertexBuffer->unmap();

  if (num_normals > 0) {
    memcpy( static_cast<void*>( normalBuffer_Host ),
           static_cast<void*>( mesh->mNormals),
           sizeof( optix::float3 )*numVertices);
  }
  normalBuffer->unmap();

  //Material* matl = m_materials.at(0);
  Material* matl = m_materials.at(mesh->mMaterialIndex);
  std::cout<<"Adding material index:"<<mesh->mMaterialIndex<<std::endl;
  // Transfer texture coordinates to buffer
  optix::Buffer texCoordBuffer;
  if (mesh->HasTextureCoords(0) && matl->materialType() == "Texture") { 
    std::cout<<"Mesh has texture coords - making texture"<<std::endl;
    texCoordBuffer = context->createBuffer( RT_BUFFER_INPUT, RT_FORMAT_FLOAT2, numVertices);
    optix::float2* texCoordBuffer_Host = static_cast<optix::float2*>( texCoordBuffer->map());
    for(unsigned int i = 0; i < mesh->mNumVertices; i++) {
      aiVector3D texCoord = (mesh->mTextureCoords[0])[i];
      texCoordBuffer_Host[i].x = texCoord.x;
      texCoordBuffer_Host[i].y = texCoord.y;
    }
    texCoordBuffer->unmap();
  } else {
    texCoordBuffer = context->createBuffer( RT_BUFFER_INPUT, RT_FORMAT_FLOAT2, 0);
  }

  geometry["texCoordBuffer"]->setBuffer(texCoordBuffer);

  /// Tangents and bi-tangents buffers
  geometry["hasTangentsAndBitangents"]->setUint(mesh->HasTangentsAndBitangents() ? 1 : 0);
  if (mesh->HasTangentsAndBitangents()) {
    optix::Buffer tangentBuffer = context->createBuffer( RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, numVertices);
    optix::float3* tangentBuffer_Host = static_cast<optix::float3*>( tangentBuffer->map() );
    memcpy( static_cast<void*>( tangentBuffer_Host ),
           static_cast<void*>( mesh->mTangents),
           sizeof( optix::float3 )*numVertices);
    tangentBuffer->unmap();

    optix::Buffer bitangentBuffer = context->createBuffer( RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, numVertices);
    optix::float3* bitangentBuffer_Host = static_cast<optix::float3*>( bitangentBuffer->map() );
    memcpy( static_cast<void*>( bitangentBuffer_Host ),
           static_cast<void*>( mesh->mBitangents),
           sizeof( optix::float3 )*numVertices);
    bitangentBuffer->unmap();

    geometry["tangentBuffer"]->setBuffer(tangentBuffer);
    geometry["bitangentBuffer"]->setBuffer(bitangentBuffer);
  } else {
    optix::Buffer emptyBuffer = context->createBuffer(RT_BUFFER_INPUT_OUTPUT, RT_FORMAT_FLOAT3, 0);
    geometry["tangentBuffer"]->setBuffer(emptyBuffer);
    geometry["bitangentBuffer"]->setBuffer(emptyBuffer);
  }

  /// Create index buffer
  optix::Buffer indexBuffer = context->createBuffer( RT_BUFFER_INPUT, RT_FORMAT_INT3, numFaces );
  optix::int3* indexBuffer_Host = static_cast<optix::int3*>( indexBuffer->map() );
  geometry["indexBuffer"]->setBuffer(indexBuffer);

  //// Copy index buffer from host to device
  for(unsigned int i = 0; i < mesh->mNumFaces; i++) {
    aiFace face = mesh->mFaces[i];
    indexBuffer_Host[i].x = face.mIndices[0];
    indexBuffer_Host[i].y = face.mIndices[1];
    indexBuffer_Host[i].z = face.mIndices[2];
  }

  indexBuffer->unmap();

  optix::Material optix_material = matl->getOptixMaterial(context);
  std::cout<<"Pushback instance with material"<<matl->materialType()<<std::endl;
  optix::GeometryInstance gi = context->createGeometryInstance( geometry, &optix_material, &optix_material+1 );
  matl->registerBaseGeometryInstanceValues(gi);

  obj_ginstances.push_back(gi);
  return true;
}

bool SceneNet::createOBJLayout(std::string& layout_file,
                             optix::Context& context,
                             std::vector<optix::GeometryInstance>& obj_ginstances)
{
  std::cout<<"Loading layout file:"<<layout_file<<std::endl;
  Assimp::Importer importer;
  const aiScene *m_scene =importer.ReadFile(layout_file,aiProcess_Triangulate | aiProcess_SortByPType);
  if(!m_scene) {
      std::cout<<"An error occurred reading:"<<layout_file<<std::endl;
      exit(1);
  }
  //Get base name from which the textures are looked up
  std::size_t found = layout_file.find_last_of("/");
  std::string file_base_name = layout_file.substr(0,found);
  loadSceneMaterials(file_base_name,m_scene,true);

  // Build rotation matrix from angle
  const float scale = 1.0;

  aiVector3D m_obj_min, m_obj_max, m_obj_centre, m_obj_size;
  m_obj_min.x = m_obj_min.y = m_obj_min.z = 1e15;
  m_obj_max.x = m_obj_max.y = m_obj_max.z = -1e15;
  for(int i =0; i < m_scene->mNumMeshes; i++) {
      aiMesh* mesh = m_scene->mMeshes[i];
      std::cout<<"mesh material index:"<<mesh->mMaterialIndex<<std::endl;
      aiVector3D* vertices_f3 = reinterpret_cast<aiVector3D*>( mesh->mVertices );
      for(int v = 0; v < mesh->mNumVertices; v++) {
          if ( vertices_f3[v].x < m_obj_min.x ) {
              m_obj_min.x = vertices_f3[v].x ;
          }
          if ( vertices_f3[v].y < m_obj_min.y ) {
              m_obj_min.y = vertices_f3[v].y ;
          }
          if ( vertices_f3[v].z < m_obj_min.z ) {
              m_obj_min.z = vertices_f3[v].z ;
          }
          if ( vertices_f3[v].x > m_obj_max.x ) {
              m_obj_max.x = vertices_f3[v].x ;
          }
          if ( vertices_f3[v].y > m_obj_max.y ) {
              m_obj_max.y = vertices_f3[v].y ;
          }
          if ( vertices_f3[v].z > m_obj_max.z ) {
              m_obj_max.z = vertices_f3[v].z;
          }
      }
  }
  m_obj_size = m_obj_max - m_obj_min;
  m_layout_max = m_obj_max;
  m_layout_min = m_obj_min;
  m_layout_size = m_obj_size;
  std::cout<<"The LAYOUT size is:"<<m_obj_size.x<<" "<<m_obj_size.y<<" "<<m_obj_size.z<<std::endl;
  std::cout<<"The LAYOUT min is:"<<m_obj_min.x<<" "<<m_obj_min.y<<" "<<m_obj_min.z<<std::endl;
  m_obj_centre = m_obj_min + (0.5f * m_obj_size);

  for(int mesh_id =0; mesh_id < m_scene->mNumMeshes; mesh_id++) {
    aiMesh* mesh = m_scene->mMeshes[mesh_id];
    unsigned int numVertices = mesh->mNumVertices;

    aiVector3D* vertices_f3 = reinterpret_cast<aiVector3D*>( mesh->mVertices );
    aiVector3D* normals_f3  = reinterpret_cast<aiVector3D*>( mesh->mNormals );

    room_center = m_obj_centre;
  }
  for(int mesh_id =0; mesh_id < m_scene->mNumMeshes; mesh_id++) {
    aiMesh* mesh = m_scene->mMeshes[mesh_id];
    if (!createOptixGeometry(mesh,context,obj_ginstances)) {
        exit(1);
    }
  }
  return true;
}


bool SceneNet::createOBJMesh(ObjectInfo object, optix::Context& context,
                             std::vector<optix::GeometryInstance>& obj_ginstances) {
  std::string file_base_name = obj_base_folder_ + object.object_id + "/models";
  DIR *dir;
  struct dirent *ent;
  std::string a_filename;

  if ((dir = opendir(file_base_name.c_str())) != NULL) {
    while ((ent = readdir (dir)) != NULL) {
      a_filename = std::string(ent->d_name);
      if (a_filename.length() <= 4) {
        continue;
      }
      if (a_filename.substr(a_filename.length() - 3,3) == "obj") {
        break;
      }
    }
    closedir (dir);
  } else {
    std::cout<<"Error: Couldnt open dir:"<<file_base_name<<std::endl;
    return false;
  }

  std::string filename = std::string(file_base_name) + std::string("/") + a_filename;

  float min_obj_x, min_obj_y, min_obj_z; 	
  float max_obj_x, max_obj_y, max_obj_z; 	
	
  min_obj_x = min_obj_y = min_obj_z = 1e15;	
  max_obj_x = max_obj_y = max_obj_z = -1e15;	

  /// Read the OBJ model in assimp
  Assimp::Importer importer;
  const aiScene *m_scene = importer.ReadFile(filename,aiProcess_Triangulate | aiProcess_SortByPType | 
                                                      aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph);
  /// Throw error if the model can't be read and exit
  if(!m_scene) {
      std::string error = std::string("An error occurred in Assimp during reading of this file");
      std::cout<<error<<std::endl;
      exit(1);
  }

  /// Build rotation matrix from angle
  aiVector3D m_obj_min, m_obj_max, m_obj_centre, m_obj_size;
  m_obj_min.x = m_obj_min.y = m_obj_min.z = 1e15;
  m_obj_max.x = m_obj_max.y = m_obj_max.z = -1e15;
  for(int i =0; i < m_scene->mNumMeshes; i++) {
      aiMesh* mesh = m_scene->mMeshes[i];
      if (!mesh->HasNormals()) {
        continue;
      }
      aiVector3D* vertices_f3 = reinterpret_cast<aiVector3D*>( mesh->mVertices );
      for(int v = 0; v < mesh->mNumVertices; v++) {
          if ( vertices_f3[v].x < m_obj_min.x ) {
              m_obj_min.x = vertices_f3[v].x ;
          }
          if ( vertices_f3[v].y < m_obj_min.y ) {
              m_obj_min.y = vertices_f3[v].y ;
          }
          if ( vertices_f3[v].z < m_obj_min.z ) {
              m_obj_min.z = vertices_f3[v].z ;
          }
          if ( vertices_f3[v].x > m_obj_max.x ) {
              m_obj_max.x = vertices_f3[v].x ;
          }
          if ( vertices_f3[v].y > m_obj_max.y ) {
              m_obj_max.y = vertices_f3[v].y ;
          }
          if ( vertices_f3[v].z > m_obj_max.z ) {
              m_obj_max.z = vertices_f3[v].z;
          }
      }
  }
  m_obj_size = m_obj_max - m_obj_min;
  m_obj_centre = m_obj_min + (0.5f * m_obj_size);

  //This is to sort out the 60% margin we give it in the physics engine
  m_obj_centre.y -= (m_obj_size.y * 0.6);

  const float scale = object.scale/m_obj_size.y;
  /// Now load the materials...
  loadSceneMaterials(file_base_name,m_scene);
  std::cout<<"Working on mesh"<<std::endl;

  for(int mesh_id = 0; mesh_id < m_scene->mNumMeshes; mesh_id++) {
    if (mesh_id > 200) {
        break;
    }
    std::cout<<"Mesh id"<<mesh_id<<std::endl;
    aiMesh* mesh = m_scene->mMeshes[mesh_id];
    if (!mesh->HasNormals()) {
      continue;
    }
    unsigned int numFaces    = mesh->mNumFaces;
    unsigned int numVertices = mesh->mNumVertices;

    aiVector3D* vertices_f3 = reinterpret_cast<aiVector3D*>( mesh->mVertices );
    aiVector3D* normals_f3  = reinterpret_cast<aiVector3D*>( mesh->mNormals );
    for (int i = 0; i < numVertices; i++) {
      vertices_f3[i] -= m_obj_centre;
      vertices_f3[i].x *= scale;
      vertices_f3[i].y *= scale;
      vertices_f3[i].z *= scale;
      /// We dont want to do translation here..
      optix::float4 v4 = optix::make_float4(vertices_f3[i].x,vertices_f3[i].y,vertices_f3[i].z, 1.0f);
      optix::float3 vt = optix::make_float3(object.transform * v4);
      vertices_f3[i].x = vt.x;
      vertices_f3[i].y = vt.y;
      vertices_f3[i].z = vt.z;
    }

    for(int i = 0; i < numVertices; i++) {
	    if ( vertices_f3[i].x < min_obj_x ) { min_obj_x = vertices_f3[i].x ; }
	    if ( vertices_f3[i].y < min_obj_y ) { min_obj_y = vertices_f3[i].y ; }
	    if ( vertices_f3[i].z < min_obj_z ) { min_obj_z = vertices_f3[i].z ; }
	    if ( vertices_f3[i].x > max_obj_x ) { max_obj_x = vertices_f3[i].x ; }
	    if ( vertices_f3[i].y > max_obj_y ) { max_obj_y = vertices_f3[i].y ; }
	    if ( vertices_f3[i].z > max_obj_z ) { max_obj_z = vertices_f3[i].z; }
    }
    /// Transform normals -> each vertex has its normal
    int num_normals = mesh->mNumFaces*3;
    for (int i = 0; i < num_normals; ++i) { 
      optix::float4 v4 = optix::make_float4(normals_f3[i].x,normals_f3[i].y,normals_f3[i].z, 0.0f);
      optix::float3 v3 = optix::normalize(optix::make_float3(object.transform * v4));
      normals_f3[i].x = v3.x;
      normals_f3[i].y = v3.y;
      normals_f3[i].z = v3.z;
    }

    if (!createOptixGeometry(mesh,context,obj_ginstances)) {
      std::cout<<"Failed to create optix geometry"<<std::endl;
    }
  }
  std::cout<<"Return true on Mesh"<<std::endl;
  return true;
}

void SceneNet::loadSceneMaterials(const std::string& model_path, const aiScene* m_scene,bool separate_materials) {
    std::uniform_real_distribution<float> color_dist(0.5,1.0);
    std::uniform_real_distribution<float> power_dist(0.0,1.0);
    assert(m_materials.size()==0);
    //Class id is not used anymore
    int class_id = 0;
    // This allows us to easily change to only diffuse
    Material* mat_diffuse_default = new Diffuse(Vector3(0.8,0.8,0.8),class_id,instance_id_);
    std::cout<<"Loading scene materials for model:"<<model_path<<" instance id "<<instance_id_<<std::endl;
    m_materials.push_back(mat_diffuse_default);

    for(unsigned int i = 0; i < m_scene->mNumMaterials; i++) {
        float r = color_dist(*rand_gen_);
        float g = color_dist(*rand_gen_);
        float b = color_dist(*rand_gen_);
        aiMaterial* material = m_scene->mMaterials[i];
        aiString name;
        material->Get(AI_MATKEY_NAME, name);
        std::string strMat(name.C_Str());
        std::cout<<"Str Mat"<<strMat<<std::endl;
        if ( strMat.find("DefaultMaterial")!= std::string::npos ) {
            std::cout<<"continuing"<<std::endl;
            continue;
        }
        /// Check if this is an Emitter
        aiColor3D emissivePower;
        if(material->Get(AI_MATKEY_COLOR_EMISSIVE, emissivePower) == AI_SUCCESS
                && colorHasAnyComponent(emissivePower)) {
            aiColor3D diffuseColor;
            if(material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor) != AI_SUCCESS) {
                diffuseColor.r = 1;
                diffuseColor.g = 1;
                diffuseColor.b = 1;
            }
            float power = power_dist(*rand_gen_);
            DiffuseEmitter* mat_diff_emit = new DiffuseEmitter(power * toFloat3(emissivePower), toFloat3(diffuseColor), class_id,instance_id_);
            std::cout<<"instance_id_"<<instance_id_<<std::endl;
            if (separate_materials) {
              std::cout<<"instance_id_"<<instance_id_<<std::endl;
              log_file<<"instance:"<< instance_id_ << ";03665924;lightbulb,"<<strMat<<";"<<std::endl;
              instance_id_++;
            }
            mat_diff_emit->setInverseArea(5.0f);
            m_materials.push_back(mat_diff_emit);
            continue;
        }

        /// Textured material
        std::string lookup;
        if (separate_materials) {
          std::size_t found = strMat.find_first_of(".");
          if (found!=std::string::npos) {
            lookup = strMat.substr(0,found);
          } else {
            lookup = strMat;
          }
          std::transform(lookup.begin(), lookup.end(), lookup.begin(), ::tolower);
          //std::cout<<"Lookup:"<<lookup<<std::endl;
        }
        aiString textureName;
        if(material->Get(AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE, 0), textureName) == AI_SUCCESS) {
            std::string textureAbsoluteFilePath = model_path + "/" + std::string(textureName.C_Str());
            std::string Kdpath(textureName.C_Str());
            std::cout<<"Kdpath:"<<Kdpath<<std::endl;
            bool is_glass = false;
            if (separate_materials) {
              std::string texprefix = "../texture_library/";
              if (Kdpath.substr(texprefix.length(),Kdpath.length() - texprefix.length()) == std::string("glass")) {
                is_glass = true;
                std::cout<<"pure glass:"<<Kdpath<<std::endl;
              }
              if (Kdpath.substr(texprefix.length(),Kdpath.length() - texprefix.length()) == std::string("glass_plastic")) {
                std::uniform_int_distribution<> glass_dist(0,1);
                is_glass = !static_cast<bool>(glass_dist(*rand_gen_));
                if (is_glass) {
                  std::cout<<"Glass plastic is now glass"<<std::endl;
                } else {
                  std::cout<<"Glass plastic is now plastic"<<std::endl;
                }
              }
              if (Kdpath.substr(texprefix.length(),Kdpath.length() - texprefix.length()) == std::string("metal_plastic_glass")) {
                std::uniform_int_distribution<> glass_dist(0,2);
                is_glass = !static_cast<bool>(glass_dist(*rand_gen_));
                if (is_glass) {
                  std::cout<<"Metal glass plastic is now glass"<<std::endl;
                } else {
                  std::cout<<"Metal glass plastic is now metal or plastic"<<std::endl;
                }
              }
            }

            if (is_glass) {
              // Glass Material
              aiColor3D diffuseColor;
              if(material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor) != AI_SUCCESS) {
                diffuseColor.r = 1;
                diffuseColor.g = 1;
                diffuseColor.b = 1;
              }
              Material* mat_glass = new Glass(1.5, optix::make_float3(diffuseColor.r,diffuseColor.g,diffuseColor.b),class_id,instance_id_);
              if (separate_materials) {
                std::string wnid = scenenetlayout_mat_to_wnid[lookup];
                log_file<<"instance:"<< instance_id_ << ";"<<wnid<<";"<<strMat<<";"<<std::endl;
                instance_id_++;
              }
              m_materials.push_back(mat_glass);
              continue;
            }
            if (separate_materials && Kdpath.substr(std::string("../texture_library/").length(),6) == std::string("mirror")) {
              // Reflective/mirror material
              std::cout<<"is now mirror"<<std::endl;
              Material*  mat_mirror = new Mirror(optix::make_float3(0.8,0.9,0.8),class_id,instance_id_);
              if (separate_materials) {
                std::string wnid = scenenetlayout_mat_to_wnid[lookup];
                log_file<<"instance:"<< instance_id_ << ";"<<wnid<<";"<<strMat<<";"<<std::endl;
                instance_id_++;
              }
              m_materials.push_back(mat_mirror);
              continue;
            }
            std::cout<<"Past specials"<<std::endl;


            //Pick random layout
            DIR *dir;
            struct dirent *ent;
            std::vector<std::string> available_textures;
            std::cout<<"textureAbs path:"<<textureAbsoluteFilePath.c_str()<<std::endl;
            if ((dir = opendir(textureAbsoluteFilePath.c_str())) != NULL) {
              while ((ent = readdir (dir)) != NULL) {
                std::string a_filename = std::string(ent->d_name);
                if (a_filename.length() <= 4) {
                  std::cout<<"Continuing in loop"<<std::endl;
                  continue;
                }
                available_textures.push_back(a_filename);
              }
              closedir (dir);
              std::cout<<"closedDir"<<std::endl;
              //Now pick a random one
              std::uniform_int_distribution<> texture_dist(0,available_textures.size()-1);
              int texture_choice = texture_dist(*rand_gen_);
              std::string texture_file = textureAbsoluteFilePath + "/" + available_textures[texture_choice];
              std::cout<<"Selected texture absolute path::"<<texture_file<<std::endl;
              if (!extra_scene_textures_.empty()) {
                texture_file = extra_scene_textures_[0];
                std::cout<<"Overwriting selected texture absolute path::"<<texture_file<<std::endl;
                extra_scene_textures_.erase(extra_scene_textures_.begin());
              }
              std::uniform_int_distribution<> specular(0,2);
              bool diffuse_texture = static_cast<bool>(specular(*rand_gen_));
              Material* matl = NULL;
              if (diffuse_texture) {
                  std::cout<<"Matl is texture"<<std::endl;
                matl = new Texture(texture_file,class_id,instance_id_);
              } else {
                std::uniform_real_distribution<float> Ks(0.0,0.1);
                std::uniform_real_distribution<float> exp(80.0,300.0);
                matl = new SpecularTexture(texture_file,optix::make_float3(Ks(*rand_gen_)),exp(*rand_gen_),class_id,instance_id_);
              }
              if (separate_materials) {
                std::string wnid = scenenetlayout_mat_to_wnid[lookup];
                log_file<<"instance:"<< instance_id_ << ";"<<wnid<<";"<<strMat<<";"<<std::endl;
                instance_id_++;
              }
              std::cout<<"Push back"<<std::endl;
              m_materials.push_back(matl);
              continue;
            } else {
              Material* matl = new Texture(textureAbsoluteFilePath,class_id,instance_id_);
              if (separate_materials) {
                std::string wnid = scenenetlayout_mat_to_wnid[lookup];
                log_file<<"instance:"<< instance_id_ << ";"<<wnid<<";"<<strMat<<";"<<std::endl;
                instance_id_++;
              }
              m_materials.push_back(matl);
            }
          std::cout<<"Continue to next"<<std::endl;
          continue;
        }

        // Glass Material
        float opacity;
        if(material->Get(AI_MATKEY_OPACITY, opacity) == AI_SUCCESS && opacity < 0.5f) {
            aiColor3D diffuseColor;
            if(material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor) != AI_SUCCESS) {
                diffuseColor.r = 1;
                diffuseColor.g = 1;
                diffuseColor.b = 1;
            }
            Material* mat_glass = new Glass(1.5, optix::make_float3(diffuseColor.r,diffuseColor.g,diffuseColor.b),class_id,instance_id_);
            if (separate_materials) {
                std::string wnid = scenenetlayout_mat_to_wnid[lookup];
                log_file<<"instance:"<< instance_id_ << ";"<<wnid<<";"<<strMat<<";"<<std::endl;
              instance_id_++;
            }
            m_materials.push_back(mat_glass);
            continue;
        }
        // Reflective/mirror material
        aiColor3D reflectiveColor;
        if(material->Get(AI_MATKEY_COLOR_REFLECTIVE, reflectiveColor) == AI_SUCCESS && colorHasAnyComponent(reflectiveColor)) {
            Material*  mat_mirror = new Mirror(toFloat3(reflectiveColor),class_id,instance_id_);
            if (separate_materials) {
                std::string wnid = scenenetlayout_mat_to_wnid[lookup];
                log_file<<"instance:"<< instance_id_ << ";"<<wnid<<";"<<strMat<<";"<<std::endl;
              instance_id_++;
            }
            m_materials.push_back(mat_mirror);
            continue;
        }

        // Diffuse
        aiColor3D diffuseColor;
        if(material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor) == AI_SUCCESS) {                      
            Material* mat_diffuse = new Diffuse(toFloat3(diffuseColor),class_id,instance_id_);
            if (separate_materials) {
                std::string wnid = scenenetlayout_mat_to_wnid[lookup];
                log_file<<"instance:"<< instance_id_ << ";"<<wnid<<";"<<strMat<<";"<<std::endl;
              instance_id_++;
            }
            m_materials.push_back(mat_diffuse);
            continue;
        }

        // Fall back to a red diffuse material
        Material* mat_diffuse = new Diffuse(optix::make_float3(1,0,0),class_id,instance_id_);
        if (separate_materials) {
          std::string wnid = scenenetlayout_mat_to_wnid[lookup];
          log_file<<"instance:"<< instance_id_ << ";"<<wnid<<";"<<strMat<<";"<<std::endl;
          instance_id_++;
        }
        m_materials.push_back(mat_diffuse);
    }
    std::cout<<"Loaded materials"<<std::endl;
}

const std::vector<Light> & SceneNet::getSceneLights(void) const {
    return m_sceneLights;
}

SceneNetCamera SceneNet::getCamera() const
{
    SceneNetCamera default_Camera =  SceneNetCamera(optix::make_float3(0.0,0.0,0.0),   // Eye
                                     optix::make_float3(0.0,0.0,0.0),                  // Target
                                     optix::make_float3(0.0f, 1.0f,  0.0f),            // Up Vector
                                     60.0f,                                            // hFoV
                                     45.0f);                                           // vFoV
    return default_Camera;
}

std::pair<TooN::Vector<3>,TooN::Vector<3>> SceneNet::getPose(bool restart, bool second) {
  if (restart) {
    pose_number_ = 0; 
  }
  if (second) {
    pose_number_ = 1; 
  }
  if (pose_number_ < saved_poses_.size()) {
    pose_ = saved_poses_[pose_number_];
    time_ = saved_timestamps_[pose_number_];
    if(!restart && !second) {
      log_file<<std::setprecision(6)<<"time:"<<time_<<" pose:"<<pose_.first[0]<<","<<pose_.first[1]<<","<<pose_.first[2]<<" lookat:"<<pose_.second[0]<<","<<pose_.second[1]<<","<<pose_.second[2]<<std::endl;
    }    
    pose_number_++;
    return pose_;
  }
  TooN::Vector<3> eye = TooN::makeVector(0.0,0.0,0.0);
  TooN::Vector<3> lookat = TooN::makeVector(0.0,0.0,0.0);
  std::pair<TooN::Vector<3>,TooN::Vector<3>> pose = std::make_pair(eye,lookat);
  return pose;
}

const char* SceneNet::getSceneName() const {
    return SceneNet::getSceneNetSceneName();
}

AAB SceneNet::getSceneAABB() const {
    return m_sceneAABB;
}

const char* SceneNet::getSceneNetSceneName() {
     return "SceneNet";
}

unsigned int SceneNet::getNumTriangles() const {
    return 0;
}

optix::Matrix3x3 SceneNet::Rotation(optix::float3 rotation) {
  float alpha = deg_to_rad( rotation.x );
  float beta  = deg_to_rad( rotation.y );
  float gamma = deg_to_rad( rotation.z );

  float s_a = sinf(alpha);
  float c_a = cosf(alpha);

  float s_b = sinf(beta);
  float c_b = cosf(beta);

  float s_g = sinf(gamma);
  float c_g = cosf(gamma);

  float rotate_x[3*3] = {   1,    0,    0,
    0,   c_a, -s_a,
    0,   s_a,  c_a };

  float rotate_y[3*3] = {  c_b,   0,   -s_b,
    0,    1,    0,
    s_b,   0,   c_b };

  float rotate_z[3*3] = {  c_g, -s_g,   0,
    s_g,  c_g,   0,
    0,    0,    1 };

  optix::Matrix3x3 mat_x(rotate_x);
  optix::Matrix3x3 mat_y(rotate_y);
  optix::Matrix3x3 mat_z(rotate_z);

  optix::Matrix3x3 mat = mat_z * mat_y * mat_x;

  return mat;
}

optix::float3 SceneNet::toFloat3(aiVector3D vector) const {
    return optix::make_float3(vector.x, vector.y, vector.z);
}

optix::float3 SceneNet::toFloat3( aiColor3D vector) const {
    return optix::make_float3(vector.r, vector.g, vector.b);
}

optix::GeometryInstance SceneNet::createParallelogram(optix::Context & context, const optix::float3& anchor,
    const optix::float3& offset1, const optix::float3& offset2, Material & material) {
    optix::Geometry parallelogram = context->createGeometry();
    parallelogram->setPrimitiveCount( 1u );
    parallelogram->setIntersectionProgram( m_pgram_intersection );
    parallelogram->setBoundingBoxProgram( m_pgram_bounding_box );

    optix::float3 normal = optix::normalize( optix::cross( offset1, offset2 ) );
    float d = optix::dot( normal, anchor );
    optix::float4 plane = optix::make_float4( normal, d );

    optix::float3 v1 = offset1 / optix::dot( offset1, offset1 );
    optix::float3 v2 = offset2 / optix::dot( offset2, offset2 );

    parallelogram["plane"]->setFloat( plane );
    parallelogram["anchor"]->setFloat( anchor );
    parallelogram["v1"]->setFloat( v1 );
    parallelogram["v2"]->setFloat( v2 );

    optix::Material matl = material.getOptixMaterial(context);

    optix::GeometryInstance gi = context->createGeometryInstance( parallelogram, &matl, &matl+1 );
    material.registerBaseGeometryInstanceValues(gi);
    return gi;
}

optix::GeometryInstance SceneNet::createSphere(optix::Context & context, const optix::float3& center,
    const float& radius, Material & material) {
    optix::Geometry sphere = context->createGeometry();
    sphere->setPrimitiveCount( 1u );
    sphere->setIntersectionProgram( m_pgram_sphere_intersection );
    sphere->setBoundingBoxProgram( m_pgram_sphere_bounding_box );

    sphere["center"]->setFloat( center);
    sphere["radius"]->setFloat( radius );

    optix::Material matl = material.getOptixMaterial(context);

    optix::GeometryInstance gi = context->createGeometryInstance( sphere, &matl, &matl+1 );
    material.registerBaseGeometryInstanceValues(gi);
    return gi;
}


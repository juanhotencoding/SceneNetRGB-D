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

#ifndef SCENENET_H
#define SCENENET_H

#include "IScene.h"
#include "Material/Diffuse.h"
#include "Math/Vector3.h"
#include "Renderer/Light.h"
#include "Util/ptxpath.h"
#include "Util/sutil/OptiXMesh.h"
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <assimp/cimport.h>
#include <assimp/types.h>
#include <assimp/vector3.h>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optixu/optixu_math_namespace.h>
#include <random>
#include "SceneNetCamera.h"

struct aiScene;
struct aiMesh;
struct aiNode;
struct aiColor3D;

class Material;

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)

struct ObjectInfo {
  std::string object_id;
  float scale;
  optix::Matrix4x4 transform;
};


class SceneNet : public IScene
{
public:
    SceneNet(std::string save_base,std::string layout_file,std::string obj_base_folder,std::string layout_base_folder,int seed=0);
    virtual ~SceneNet();
    virtual optix::Group getSceneRootGroup(optix::Context & context);
    virtual const std::vector<Light> & getSceneLights() const;
    virtual SceneNetCamera getCamera() const;
    virtual std::pair<TooN::Vector<3>,TooN::Vector<3>> getPose(bool restart=false,bool second=false);
    virtual const char* getSceneName() const;
    static const char* getSceneNetSceneName();
    virtual unsigned int getNumTriangles() const;
    virtual AAB getSceneAABB() const;

    void loadSceneMaterials(const std::string& file_base_name,
                            const aiScene* m_scene,bool separate_materials=false);

    bool colorHasAnyComponent(const aiColor3D & color) {
        return color.r > 0 && color.g > 0 && color.b > 0;
    }

    TooN::Vector<3> basePosition() const {
        return TooN::makeVector(0.0f, 1.50f, 1.0f);
    }

    TooN::Vector<3> targetPosition() const {
        return TooN::makeVector(0.0f, 1.50f, -1.00f);
    }

    float deg_to_rad( float degrees ) {
        return degrees * M_PIf / 180.0f;
    }

    optix::Matrix3x3 Rotation(optix::float3 rotation);
    optix::float3 toFloat3( aiVector3D vector) const;
    optix::float3 toFloat3( aiColor3D vector ) const;

    optix::float3 scene_min() const { return toFloat3(m_layout_min); }
    optix::float3 scene_max() const { return toFloat3(m_layout_max); }

private:
    void ParseLayoutFile(std::string layout_file);
    optix::GeometryInstance createParallelogram(optix::Context & context, const optix::float3& anchor, const optix::float3& offset1, const optix::float3& offset2, Material & material);
    optix::GeometryInstance createSphere(optix::Context & context, const optix::float3& center,
                                         const float& radius, Material & material);

    bool createOptixGeometry(aiMesh* mesh,
                             optix::Context& context, 
                             std::vector<optix::GeometryInstance>& obj_ginstances);


    bool createOBJLayout(std::string& layout_file,
                        optix::Context& context,
                        std::vector<optix::GeometryInstance>& obj_ginstances);

    bool createOBJMesh(ObjectInfo object,
                       optix::Context& context,
                       std::vector<optix::GeometryInstance> &obj_ginstances);

    std::pair<TooN::Vector<3>,TooN::Vector<3>> pose_;

    int instance_id_;

    std::vector<std::string>dirNames;

    optix::Material m_material;
    optix::Material m_glassMaterial;

    bool init_traj_gen;

    optix::Program m_pgram_bounding_box;
    optix::Program m_pgram_intersection;
    optix::Program m_pgram_sphere_bounding_box;
    optix::Program m_pgram_sphere_intersection;
    optix::Program m_pgram_mesh_bounding_box;
    optix::Program m_pgram_mesh_intersection;

    std::vector<Material*> m_materials;
    std::vector<optix::Material> m_optix_materials;

    std::vector<optix::Geometry> m_geometries;

    std::string save_folder_;
    std::string obj_base_folder_;
    std::string layout_base_folder_;
    std::string layout_obj_;

    std::string layout_model_file_;

    int object_pose_;

    std::vector<Light> m_sceneLights;
    AAB m_sceneAABB;

    float x_origin;
    float y_origin;
    float z_origin;
    float x_size;
    float y_size;
    float z_size;
    float time_;

    aiVector3D room_center;

    std::shared_ptr<std::mt19937> rand_gen_;

    std::vector<ObjectInfo> map_object_info_;

    std::map<std::string,std::pair<std::string,std::string> > shapenet_id_to_class_;
    std::map<std::string,std::string> scenenetlayout_mat_to_wnid;
    std::vector<std::string> shapenet_index_to_modelid_;
    std::string save_base_;
    std::ofstream log_file;
    int pose_number_;
    std::vector<std::pair<TooN::Vector<3>,TooN::Vector<3>>> saved_poses_;
    std::vector<float> saved_timestamps_;
    aiVector3D m_layout_min, m_layout_max, m_layout_size;
    optix::Group gro;
    bool has_group;

    std::vector<std::string> extra_scene_textures_;
    std::vector<Light> m_sceneLights_load_;
};
#endif

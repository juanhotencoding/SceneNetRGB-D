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

#include <dirent.h>
#include <iostream>
#include <fstream>
#include <random>
#include <stdlib.h>
#include <string>
#include <tuple>

#include <assimp/Importer.hpp>
#include <assimp/cimport.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/types.h>
#include <assimp/vector3.h>
#include <boost/filesystem.hpp>
#include <cvd/image_io.h>
#include <pangolin/display/display.h>
#include <pangolin/pangolin.h>
#include <TooN/TooN.h>

#include "trajectoryGenerator/TrajectoryGenerator.hpp"
#include "assimpobjloader/assimp_obj_loader.h"

using namespace std;

#define RADPERDEG 0.0174533

void Arrow(GLdouble x1,GLdouble y1,GLdouble z1,GLdouble x2,GLdouble y2,GLdouble z2,GLdouble D)
{
  double x=x2-x1;
  double y=y2-y1;
  double z=z2-z1;
  double L=sqrt(x*x+y*y+z*z);

    GLUquadricObj *quadObj;

    glPushMatrix ();

      glTranslated(x1,y1,z1);

      if((x!=0.)||(y!=0.)) {
        glRotated(atan2(y,x)/RADPERDEG,0.,0.,1.);
        glRotated(atan2(sqrt(x*x+y*y),z)/RADPERDEG,0.,1.,0.);
      } else if (z<0){
        glRotated(180,1.,0.,0.);
      }

      glTranslatef(0,0,L-4*D);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluCylinder(quadObj, 2*D, 0.0, 4*D, 32, 1);
      gluDeleteQuadric(quadObj);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluDisk(quadObj, 0.0, 2*D, 32, 1);
      gluDeleteQuadric(quadObj);

      glTranslatef(0,0,-L+4*D);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluCylinder(quadObj, D, D, L-4*D, 32, 1);
      gluDeleteQuadric(quadObj);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluDisk(quadObj, 0.0, D, 32, 1);
      gluDeleteQuadric(quadObj);

    glPopMatrix ();
}

void drawAxes(GLdouble length)
{
    glPushMatrix();
    glColor3f(1.0,0,0);
    glTranslatef(-length,0,0);
    Arrow(0,0,0, 2*length,0,0, 0.1);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0.0,1.0,0);
    glTranslatef(0,-length,0);
    Arrow(0,0,0, 0,2*length,0, 0.1);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0.0,0.0,1.0);
    glTranslatef(0,0,-length);
    Arrow(0,0,0, 0,0,2*length, 0.1);
    glPopMatrix();
}

class Scene {

public:

  Scene(std::string layout_file, std::string scenenet_layouts, pangolin::View& camera, float near_plane, float far_plane)
  : camera_(camera)
  , near_plane_(near_plane)
  , far_plane_(far_plane)
  , built_scene(false) {
      check_rgb = false;
      depth_arrayf = new float[640*480];
      rgb_array = new unsigned char[640*480*3];

      ifstream mylayoutfile(layout_file);
      int n_chars = 500;
      char readlinedata[n_chars];
      mylayoutfile.getline(readlinedata,n_chars);
      std::string line(readlinedata);
      std::replace(line.begin(), line.end(), ':', ' ');
      istringstream iss(line);
      std::string layout_string, layout_fileName, model_pathname;

      iss >> layout_string;
      iss >> layout_fileName;
      models_path.push_back(layout_fileName);

      layout_fileName  = scenenet_layouts+layout_fileName;

      AssimpObjLoader room_layout(layout_fileName);
      number_of_vertices_x3_shape = room_layout.get_numVertes_submeshes();
      shape_vertices = room_layout.get_shape_vertices();
      min_max_bb = room_layout.get_min_max_3d_bounding_box();

      meshNames = room_layout.getMeshNames();

      models_num_verts_x3_shape.push_back(number_of_vertices_x3_shape);
      model_shape_vertices.push_back(shape_vertices);
      min_max_bbs.push_back(min_max_bb);

      TooN::SE3<>T_wc;
      float scale;
      scales_of_objects.push_back(1.0f);
      transformations_of_objects.push_back(T_wc);

      std::string wnid = "";
      wnids_model.push_back(wnid);
      while(true) {
          mylayoutfile >> model_pathname;
          if(mylayoutfile.eof())
            break;
          mylayoutfile >> wnid;
          wnids_model.push_back(wnid);
          std::string model_file(model_pathname);
          models_path.push_back(model_file);
          mylayoutfile >> scale;
          scales_of_objects.push_back(scale);

          mylayoutfile >> T_wc;
          transformations_of_objects.push_back(T_wc);
          std::cout<<"T_wc = " << T_wc << std::endl;
      }

      mylayoutfile.close();

      for(int i = 1; i < models_path.size(); i++) {
          std::string model_file = models_path.at(i);
          std::cout<<model_file << std::endl;
          std::string wnid = wnids_model.at(i);
          AssimpObjLoader obj_model(model_file, wnid);
          models_num_verts_x3_shape.push_back(obj_model.get_numVertes_submeshes());
          model_shape_vertices.push_back(obj_model.get_shape_vertices());
          min_max_bbs.push_back(obj_model.get_min_max_3d_bounding_box());
      }

      for(int i = 0; i < 100000; i++) {
          float rand_r = (float)rand()/RAND_MAX;
          float rand_g = (float)rand()/RAND_MAX;
          float rand_b = (float)rand()/RAND_MAX;

          red_colours.push_back(rand_r);
          green_colours.push_back(rand_g);
          blue_colours.push_back(rand_b);
      }
  }

  ~Scene() {
      delete[] depth_arrayf;
      delete[] rgb_array;
  }

  bool collided (const TooN::Vector<3,float> position, const TooN::Vector<3,float> next_position) {
      pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrixRDF_BottomLeft(640,480,420.0,420.0,320,240,near_plane_,far_plane_),
        pangolin::ModelViewLookAt(3,3,3, 0,0,0, pangolin::AxisNegZ)
      );

      TooN::Vector<3>eye      = position;
      TooN::Vector<3>lookAt   = next_position;
      TooN::Vector<3>look_dir = lookAt - eye;
      TooN::normalize(look_dir);

      TooN::Vector<3>up_vec   = TooN::makeVector(0.0f,-1.0f,0.0f);
      TooN::Vector<3>right    = look_dir ^ up_vec;
      TooN::Vector<3>newup    = look_dir ^ right;

      /// Draw Camera
      TooN::Matrix<3>RotMat = TooN::Zeros(3);
      RotMat.T()[0] = right;
      RotMat.T()[1] = newup;
      RotMat.T()[2] = look_dir;

      TooN::SE3<>se3_pose = TooN::SE3<>(TooN::SO3<>(RotMat),eye);
      TooN::SE3<>T_wc = se3_pose;
      TooN::SE3<>T_cw = T_wc.inverse();
      TooN::SO3<>Rot        = T_cw.get_rotation();

      TooN::Matrix<3>SO3Mat = Rot.get_matrix();
      TooN::Vector<3>trans  = T_cw.get_translation();
      TooN::Matrix<4>SE3Mat = TooN::Identity(4);

      SE3Mat.slice(0,0,3,3) = SO3Mat;
      SE3Mat(0,3) = trans[0];
      SE3Mat(1,3) = trans[1];
      SE3Mat(2,3) = trans[2];

      /// Ref: http://www.felixgers.de/teaching/jogl/generalTransfo.html
      /// It should be a transpose - stored in column major
      pangolin::OpenGlMatrix openglSE3Matrix;
      for(int col = 0; col < 4; col++ ) {
        for(int row = 0; row < 4; row++) {
          openglSE3Matrix.m[col*4+row] = SE3Mat(row,col);
        }
      }

      s_cam.SetModelViewMatrix(openglSE3Matrix);
      s_cam.Apply();
      camera_.ActivateScissorAndClear(s_cam);
      float min_depth = render_with_current_camera(true,640,480);

      if (min_depth < 0.3 || min_depth > 999.0) {
        return true;
      }
      return false;
  }

  float render_with_current_camera(bool show3DRoom, int width, int height,
                                   int objects2plot=1000,
                                   bool show_layout=true,
                                   bool max_reasonable=false) {
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT);
        glColor3f(1.0f,1.0f,1.0f);

        /// Draw object
        if (show3DRoom && !built_scene) {
            std::vector<float> num_vertices_copy;
            TooN::Vector<3>room_center;
            std::cout<<"::::::::::::::::::::::::::::::::objects2plot = " << objects2plot << std::endl;
            std::cout<<"objs size:"<<transformations_of_objects.size()<<std::endl;
            for(int objs = 0; objs < transformations_of_objects.size(); objs++) {
                min_max_bb = min_max_bbs.at(objs);
                number_of_vertices_x3_shape = models_num_verts_x3_shape.at(objs);
                shape_vertices = model_shape_vertices.at(objs);
                TooN::Vector<3>size   = TooN::makeVector(min_max_bb.at(1)-min_max_bb.at(0),
                                                       (min_max_bb.at(3)-min_max_bb.at(2)),
                                                       min_max_bb.at(5)-min_max_bb.at(4));
                TooN::Vector<3>center = 0.5f * TooN::makeVector(min_max_bb.at(1)+min_max_bb.at(0),
                                                                (min_max_bb.at(3)+min_max_bb.at(2)),
                                                                min_max_bb.at(5)+min_max_bb.at(4));
                center[1] = center[1] - size[1]*0.6f;
                if (objs == 0) {
                    center = center*0.0f;
                }
                int num_meshes = number_of_vertices_x3_shape.size();
                TooN::SE3<>T_wc = transformations_of_objects.at(objs);
                TooN::SO3<>RMat = T_wc.get_rotation();
                float scale = scales_of_objects.at(objs)/size[1];
                if (objs == 0)
                    scale = 1.0f;
                std::cout<<"objs: " << objs << ", scale = " << scale << std::endl;
                TooN::Vector<3>translation = T_wc.get_translation();
                std::cout<<"Num meshes"<<num_meshes<<std::endl;
                for(int i = 0 ; i < num_meshes ;i++) {
                    float * shape_vertices_copy = new float[number_of_vertices_x3_shape.at(i)];
                    for(int k = 0; k <number_of_vertices_x3_shape.at(i); k+=9) {
                        float v1_x = shape_vertices[i][k+0];
                        float v1_y = shape_vertices[i][k+1];
                        float v1_z = shape_vertices[i][k+2];
                        TooN::Vector<3>r_v1 = RMat * TooN::makeVector(v1_x-center[0],
                                                                      v1_y-center[1],
                                                                      v1_z-center[2]);
                        shape_vertices_copy[k+0] = (r_v1[0])*scale + translation[0];
                        shape_vertices_copy[k+1] = (r_v1[1])*scale + translation[1];
                        shape_vertices_copy[k+2] = (r_v1[2])*scale + translation[2];
                        float v2_x = shape_vertices[i][k+3];
                        float v2_y = shape_vertices[i][k+4];
                        float v2_z = shape_vertices[i][k+5];
                        TooN::Vector<3>r_v2 = RMat * TooN::makeVector(v2_x-center[0],
                                v2_y-center[1],
                                v2_z-center[2]);
                        shape_vertices_copy[k+3] = (r_v2[0])*scale + translation[0];
                        shape_vertices_copy[k+4] = (r_v2[1])*scale + translation[1];
                        shape_vertices_copy[k+5] = (r_v2[2])*scale + translation[2];
                        float v3_x = shape_vertices[i][k+6];
                        float v3_y = shape_vertices[i][k+7];
                        float v3_z = shape_vertices[i][k+8];
                        TooN::Vector<3>r_v3 = RMat * TooN::makeVector(v3_x-center[0],
                                                                      v3_y-center[1],
                                                                      v3_z-center[2]);
                        shape_vertices_copy[k+6] = (r_v3[0])*scale + translation[0];
                        shape_vertices_copy[k+7] = (r_v3[1])*scale + translation[1];
                        shape_vertices_copy[k+8] = (r_v3[2])*scale + translation[2];
                    }
                    for (int j = 0; j < number_of_vertices_x3_shape.at(i); j++) {
                        num_vertices_copy.push_back(shape_vertices_copy[j]);
                    }
                    sub_mesh_indices.push_back(num_vertices_copy.size()/3);
                    delete[] shape_vertices_copy;
                }
                obj_indices.push_back(num_vertices_copy.size()/3);
                shape_vertices.clear();
                built_scene = true;
            }
            // generate a new VBO and get the associated ID
            glGenBuffersARB(1, &vboId);
            // bind VBO in order to use
            glBindBufferARB(GL_ARRAY_BUFFER_ARB, vboId);
            // upload data to VBO
            vertex_size = (num_vertices_copy.size() / 3);
            glBufferDataARB(GL_ARRAY_BUFFER_ARB, sizeof(float) * num_vertices_copy.size(),
                            num_vertices_copy.data(), GL_STATIC_DRAW_ARB);

        }

        glEnableClientState(GL_VERTEX_ARRAY);
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, vboId);
        int last_index = 0;
        for(size_t i = 0; i < std::min(objects2plot,(int)obj_indices.size()); ++i) {
          int this_index = obj_indices.at(i);
          glColor3f(red_colours.at(i),green_colours.at(i),blue_colours.at(i));
          glVertexPointer(3,GL_FLOAT, 0, 0);
          glDrawArrays(GL_TRIANGLES,last_index,(this_index - last_index));
          last_index = this_index;
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

        //Read rgb
        if (check_rgb) {
          glReadPixels(150, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, rgb_array);
          for(int i = 0; i < 3*width*height; i+=3) {
            int r = static_cast<int>(rgb_array[i+0]);
            int g = static_cast<int>(rgb_array[i+1]);
            int b = static_cast<int>(rgb_array[i+2]);
            int color = (((r*1000)+g)*1000)+b;
            num_colors_seen.insert(color);
          }
        }
        /// Render depth
        glReadPixels(150, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, depth_arrayf);
        int scale = 5000;
        if(!max_reasonable) {
            float min_depth = 1E5;
            for(int i = 0; i < width*height; i++) {
                float z_b = depth_arrayf[i];
                float z_n = 2.0f * z_b - 1.0f;
                depth_arrayf[i] = 2.0 * near_plane_ * far_plane_ / (far_plane_ + near_plane_ - z_n * (far_plane_ - near_plane_));
                if (min_depth > depth_arrayf[i])
                    min_depth = depth_arrayf[i];
            }
            return min_depth;
        } else {
            float max_depth = 0.01;
            for(int i = 0; i < width*height; i++) {
                float z_b = depth_arrayf[i];
                float z_n = 2.0f * z_b - 1.0f;
                depth_arrayf[i] = 2.0 * near_plane_ * far_plane_ / (far_plane_ + near_plane_ - z_n * (far_plane_ - near_plane_));
                if (depth_arrayf[i] < 30.0 && max_depth < depth_arrayf[i]) {
                    max_depth = depth_arrayf[i];
                }
            }
            return max_depth;
       }
  }

  std::vector< float > scales_of_objects;
  std::vector< std::string > models_path;
  std::vector< TooN::SE3<> > transformations_of_objects;
  std::vector< std::string > wnids_model;
  std::vector< std::vector<float> > min_max_bbs;
  std::set<int> num_colors_seen;
  bool check_rgb;

private:
  std::vector<float>red_colours;
  std::vector<float>green_colours;
  std::vector<float>blue_colours;

  std::vector<int>number_of_vertices_x3_shape;
  std::vector<float*>shape_vertices;
  std::vector<float>min_max_bb;

  std::vector< std::vector<int> > models_num_verts_x3_shape;
  std::vector< std::vector<float*> > model_shape_vertices;

  std::vector<std::string>meshNames;
  pangolin::View& camera_;

  float near_plane_;
  float far_plane_;

  GLuint vboId;
  bool built_scene;
  int vertex_size;

  std::vector<int>sub_mesh_indices;
  std::vector<int>obj_indices;

  float* depth_arrayf;
  unsigned char* rgb_array;
};

class MyCollisionInterface : public CollisionInterface {
public:
    bool collided (const TooN::Vector<3,float> position, const TooN::Vector<3,float> next_position) {
      return thescene_->collided(position,next_position);
    }

    MyCollisionInterface(Scene* thescene) {
        thescene_ = thescene;
    }
private:
    Scene* thescene_;
};

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout<<"Too few arguments"<<std::endl;
        std::cout<<"./camera_trajectory_generator /path/to/ShapeNet/ /path/to/SceneNetLayouts/ /path/to/scene_description.txt"<<std::endl;
        std::cout<<"Note that the folders should be followed by trailing / in the command"<<std::endl;
        exit(1);
    }
    std::string shapenets_dir = std::string(argv[1]);
    std::cout<<"ShapeNet directory:"<<shapenets_dir<<std::endl;
    std::string layouts_dir = std::string(argv[2]);
    std::cout<<"Layouts directory:"<<layouts_dir<<std::endl;
    std::string scene_description_file = std::string(argv[3]);
    std::cout<<"Input Scene Description:"<<scene_description_file<<std::endl;

    //GL::glutInit(&argc, argv);
    
    int w_width  = 640;
    int w_height = 480;
    const int UI_WIDTH = 150;

    pangolin::CreateWindowAndBind("GUISandbox",w_width+150,w_height);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glewInit();

    /// Create a Panel
    pangolin::View& d_panel = pangolin::CreatePanel("ui")
            .SetBounds(1.0, 0.0, 0, pangolin::Attach::Pix(150));

    float near_plane = 0.01;
    float far_plane  = 1000;

    pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrixRDF_BottomLeft(640,480,420.0,420.0,320,240,near_plane,far_plane),
      pangolin::ModelViewLookAt(3,3,3, 0,0,0, pangolin::AxisNegZ)
    );

    pangolin::OpenGlRenderState saved_s_cam(
      pangolin::ProjectionMatrixRDF_BottomLeft(640,480,420.0,420.0,320,240,near_plane,far_plane),
      pangolin::ModelViewLookAt(3,3,3, 0,0,0, pangolin::AxisNegZ)
    );

    /// Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::Display("cam")
      .SetBounds(0.0, 1, pangolin::Attach::Pix(UI_WIDTH), 1, -640.0f/480.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));


    std::cout<<"Loading my scene"<<std::endl;
    Scene myscene(scene_description_file,layouts_dir,d_cam,near_plane,far_plane);
    std::cout<<"Loaded my scene"<<std::endl;

    srand(time(NULL));

    bool all_intersecting = false;

    std::shared_ptr<MyCollisionInterface> collision_interface(new MyCollisionInterface(&myscene));

    TrajectoryGenerator trajGen(collision_interface,5);
    trajGen.set_max_speed(3.0);
    trajGen.set_max_acceleration(0.5);

    std::vector<float>min_max_bb = myscene.min_max_bbs.at(0);

    TooN::Vector<3>room_center = 0.5f * TooN::makeVector(min_max_bb.at(1)+min_max_bb.at(0),
                                                    min_max_bb.at(3)+min_max_bb.at(2),
                                                    min_max_bb.at(5)+min_max_bb.at(4));

    TooN::Vector<3>room_size   = TooN::makeVector(min_max_bb.at(1)-min_max_bb.at(0),
                                             (min_max_bb.at(3)-min_max_bb.at(2)),
                                             min_max_bb.at(5)-min_max_bb.at(4));
    //Setup random number generator
    std::random_device rd;
    std::mt19937 mt(rd());

    std::uniform_real_distribution<> real_random_number(-0.4f,0.4f);

    float rand_x = real_random_number(mt)*room_size[0] + room_center[0];
    float rand_y = real_random_number(mt)*room_size[1] + room_center[1];
    float rand_z = real_random_number(mt)*room_size[2] + room_center[2];

    TooN::Vector<3>basePosition   = TooN::makeVector(rand_x, rand_y, rand_z);
    TooN::Vector<3>targetPosition;
    float distance = 1000.0;
    do {
      rand_x = real_random_number(mt)*room_size[0] + room_center[0];
      rand_y = real_random_number(mt)*room_size[1] + room_center[1];
      rand_z = real_random_number(mt)*room_size[2] + room_center[2];
      targetPosition = TooN::makeVector(rand_x, rand_y, rand_z);
      TooN::Vector<3> diff = (targetPosition - basePosition);
      distance = TooN::norm_2(diff);
    } while (distance > 0.5);

    trajGen.Initialise(basePosition, targetPosition);
    TooN::SE3<>se3_pose;
    pangolin::OpenGlMatrix openglSE3Matrix;

    int width = 640;
    int height = 480;
    bool is_s_cam_changed = false;
    int count = 0;

    CVD::Image<CVD::Rgb<CVD::byte> > img_flipped(CVD::ImageRef(640,480));
    CVD::Image<u_int16_t>depth_image(CVD::ImageRef(width,height));

    bool loaded_shapes = false;
    GLuint vboId;

    std::vector<TrajectoryGenerator::Pose> eyeLookAt_start_Poses;
    std::vector<TrajectoryGenerator::Pose> eyeLookAt_end_Poses;

    const float shutter_speed = 1.0f/60.0f;
    const float time_step = 1.0f/25.0f;
    const int burn_in = 500;
    int pose_t = 0;
    TrajectoryGenerator::Pose previous_pose;
    float max_distance_pose_from_start = 0.0;
    float max_distance_target_from_start = 0.0;
    // Also check that it is and stays within the room boundary
    for (int burn = 0; burn < burn_in; burn++){
      if (burn % 10 == 0) {
        myscene.check_rgb = true;
      }
      TrajectoryGenerator::Pose pose     = trajGen.Step(time_step - shutter_speed);
      TrajectoryGenerator::Pose end_pose = trajGen.Step(shutter_speed);
      if (pose_t == 0) {
          previous_pose = pose;
      } else {
          float pose_distance_away = TooN::norm_2(pose.first - previous_pose.first);
          if (pose_distance_away > max_distance_pose_from_start)
            max_distance_pose_from_start = pose_distance_away;
          float target_distance_away = TooN::norm_2(pose.second - previous_pose.second);
          if (target_distance_away > max_distance_target_from_start)
            max_distance_target_from_start = target_distance_away;
      }
      if (pose.first[0] < min_max_bb[0] || pose.first[0] > min_max_bb[1]
          || pose.first[1] < min_max_bb[2] || pose.first[1] > min_max_bb[3]
          || pose.first[2] < min_max_bb[4] || pose.first[2] > min_max_bb[5]) {
        std::cout<<"Min max:"<<min_max_bb[0]<<" "<<
        min_max_bb[1]<<" "<<
        min_max_bb[2]<<" "<<
        min_max_bb[3]<<" "<<
        min_max_bb[4]<<" "<<
        min_max_bb[5]<<" "<<std::endl;
        std::cout<<"Pose out of bounds"<<std::endl;
        exit(1);
      }
      if (pose.second[0] < min_max_bb[0] || pose.second[0] > min_max_bb[1]
          || pose.second[1] < min_max_bb[2] || pose.second[1] > min_max_bb[3]
          || pose.second[2] < min_max_bb[4] || pose.second[2] > min_max_bb[5]) {
        std::cout<<"Target Pose out of bounds"<<std::endl;
        exit(1);
      }
      pose_t++;
      if (burn % 10 == 0) {
        myscene.check_rgb = false;
      }
    }

    if (max_distance_pose_from_start < 1.0) {
      std::cout<<"Pose didn't move"<<max_distance_pose_from_start<<std::endl;
      exit(1);
    }
    if (max_distance_target_from_start < 1.0) {
      std::cout<<"Target didn't move"<<max_distance_target_from_start<<std::endl;
      exit(1);
    }
    if (myscene.num_colors_seen.size() < 10) {
      std::cout<<"Did not see enough objects ("<<myscene.num_colors_seen.size()<<") - could be dud-trajectory outside walls etc, skipping"<<std::endl;
      exit(1);
    } else {
      std::cout<<"Saw enough objects ("<<myscene.num_colors_seen.size()<<")"<<std::endl;
    }
    myscene.check_rgb = false;

    const int total_num_frames = 7500;
    int frame_count = 0;
    float current_time_sec = 0.0;
    std::vector<float> time_of_end_pose;
    std::vector<float> time_of_start_pose;
    while(!pangolin::ShouldQuit() && frame_count < total_num_frames) {
        std::cout<<"."<<std::flush;
        frame_count++;
        static pangolin::Var<float> end_pt("ui.end_pt",0.1,0,10);
        static pangolin::Var<float>line_width("ui.line_width",2,1,10);
        static pangolin::Var<int>objects2plot("ui.objects2plot",150,0,150);

        static pangolin::Var<bool> show3DRoom("ui.show3DRoom",true);
        static pangolin::Var<bool> show_layout("ui.show_layout",true);

        static pangolin::Var<bool> render_camera_view("ui.render_camera_view",true);

        float pose_start_time = current_time_sec;
        current_time_sec += shutter_speed;
        float pose_end_time = current_time_sec;
        current_time_sec += time_step - shutter_speed;
        TrajectoryGenerator::Pose pose     = trajGen.Step(time_step - shutter_speed);
        TrajectoryGenerator::Pose end_pose = trajGen.Step(shutter_speed);
        if (pose.first[0] < min_max_bb[0] || pose.first[0] > min_max_bb[1]
        || pose.first[1] < min_max_bb[2] || pose.first[1] > min_max_bb[3]
        || pose.first[2] < min_max_bb[4] || pose.first[2] > min_max_bb[5]) {
          std::cout<<std::endl<<"Min max:"<<min_max_bb[0]<<" "<<
          min_max_bb[1]<<" "<<
          min_max_bb[2]<<" "<<
          min_max_bb[3]<<" "<<
          min_max_bb[4]<<" "<<
          min_max_bb[5]<<" "<<std::endl;
          std::cout<<std::endl<<"Pose out of bounds"<<std::endl;
          exit(1);
        }
        if (pose.second[0] < min_max_bb[0] || pose.second[0] > min_max_bb[1]
        || pose.second[1] < min_max_bb[2] || pose.second[1] > min_max_bb[3]
        || pose.second[2] < min_max_bb[4] || pose.second[2] > min_max_bb[5]) {
          std::cout<<std::endl<<"Target Pose out of bounds"<<std::endl;
          exit(1);
        }
        TooN::Vector<3>eye    = pose.first;
        TooN::Vector<3>lookAt = pose.second;

        eyeLookAt_start_Poses.push_back(pose);
        time_of_start_pose.push_back(pose_start_time);
        eyeLookAt_end_Poses.push_back(end_pose);
        time_of_end_pose.push_back(pose_end_time);

        TooN::Vector<3>look_dir = lookAt - eye;
        TooN::normalize(look_dir);
        TooN::Vector<3>up_vec   = TooN::makeVector(0.0f,-1.0f,0.0f);
        TooN::Vector<3>right = look_dir ^ up_vec;
        TooN::Vector<3>newup = look_dir ^ right;

        /// Draw Camera
        TooN::Matrix<3>RotMat = TooN::Zeros(3);
        RotMat.T()[0] = right;
        RotMat.T()[1] = newup;
        RotMat.T()[2] = look_dir;

        se3_pose = TooN::SE3<>(TooN::SO3<>(RotMat),eye);

        if (render_camera_view) {
            if (!is_s_cam_changed)
                saved_s_cam = s_cam;

            TooN::SE3<>T_wc = se3_pose;
            TooN::SE3<>T_cw = T_wc.inverse();

            TooN::SO3<>Rot        = T_cw.get_rotation();
            TooN::Matrix<3>SO3Mat = Rot.get_matrix();
            TooN::Vector<3>trans  = T_cw.get_translation();

            TooN::Matrix<4>SE3Mat = TooN::Identity(4);

            SE3Mat.slice(0,0,3,3) = SO3Mat;

            SE3Mat(0,3) = trans[0];
            SE3Mat(1,3) = trans[1];
            SE3Mat(2,3) = trans[2];

            /// Ref: http://www.felixgers.de/teaching/jogl/generalTransfo.html
            /// It should be a transpose - stored in column major
            for(int col = 0; col < 4; col++ ) {
                for(int row = 0; row < 4; row++) {
                    openglSE3Matrix.m[col*4+row] = SE3Mat(row,col);
                }
            }
            s_cam.SetModelViewMatrix(openglSE3Matrix);
            s_cam.Apply();
            is_s_cam_changed = true;
        } else if (is_s_cam_changed) {
              s_cam = saved_s_cam;
              std::cout<<std::endl<<"Reverting camera"<<std::endl;
              is_s_cam_changed = false;
        }
        d_cam.ActivateScissorAndClear(s_cam);
        //double check that we have a sensible viewpoint on the first frame
        if (frame_count == 1) {
          float max_depth = myscene.render_with_current_camera(show3DRoom,width,
                                                              height,
                                                              objects2plot,
                                                              show_layout,true);
          if (max_depth < 0.5) {
            std::cout<<std::endl<<"Bad initial frame"<<std::endl;
              exit(1);
          }
        }
        float min_depth = myscene.render_with_current_camera(show3DRoom,width,
                                                             height,
                                                             objects2plot,
                                                             show_layout);
        count++;
        d_panel.Render();
        pangolin::FinishFrame();

    }
    std::cout<<std::endl;

    std::string output_filename = "./scene_and_trajectory_description.txt";
    std::cout<<"Outputting text file"<<output_filename<<std::endl;
    ofstream ofile(output_filename);
    std::cout<<"total_scales " << myscene.scales_of_objects.size() << std::endl;
    std::cout<<"total_transformations " << myscene.transformations_of_objects.size() << std::endl;
    std::cout<<"total_model_paths " << myscene.models_path.size() << std::endl;

    for(int obj = 0; obj < myscene.scales_of_objects.size(); obj++) {
        if (obj == 0) {
            ofile<<"layout_file: " << myscene.models_path.at(obj) << std::endl;
            continue;
        }
        float scale = myscene.scales_of_objects.at(obj);
        std::cout<<"model_path = " << myscene.models_path.at(obj) << std::endl;
        std::string model_path = myscene.models_path.at(obj);
        TooN::SE3<>T_wo = myscene.transformations_of_objects.at(obj);
        std::string model_path_hash;
        std::cout<<model_path<<std::endl;
        model_path_hash = model_path.substr(shapenets_dir.length(),
                               model_path.length()
                               -shapenets_dir.length()
                               -std::string("/models/model_normalized.obj").length());
        ofile<<"object" << std::endl;
        ofile<<model_path_hash<<std::endl;
        ofile<<"wnid" << std::endl;
        ofile<<myscene.wnids_model.at(obj)<<std::endl;
        ofile<<"scale"<<std::endl;
        ofile<< scale << std::endl;
        ofile<<"transformation"<<std::endl;
        ofile<< T_wo << std::endl;
    }

    ofile <<"#first three elements, eye and last three, lookAt" << std::endl;
    ofile <<"#frame rate per second: " << static_cast<int>(1.0/time_step) << std::endl;
    ofile <<"#shutter_speed (s): " << shutter_speed << std::endl;
    ofile << std::endl;

    for(int i = 0; i < eyeLookAt_start_Poses.size(); i++) {
        ofile << time_of_start_pose.at(i) << " " <<eyeLookAt_start_Poses.at(i).first << " ";
        ofile << eyeLookAt_start_Poses.at(i).second << std::endl;
        ofile << time_of_end_pose.at(i) << " " << eyeLookAt_end_Poses.at(i).first << " ";
        ofile << eyeLookAt_end_Poses.at(i).second << std::endl;
        ofile << std::endl;
    }
    ofile.close();
}

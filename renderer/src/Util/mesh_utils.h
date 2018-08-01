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

#include <glm/glm.hpp>
#include <GL/gl.h>
#include <iostream>
#include "../tinyobjloader/tiny_obj_loader.h"
#include <TooN/TooN.h>

#include <unistd.h>
#include <iostream>
#include <fstream>

#include <vector_types.h>
#include <vector_functions.h>

#include <boost/chrono.hpp>
#include <boost/timer.hpp>

#include <dirent.h>
#include <fstream>

std::vector<std::string> getSubdirs(std::string dir_path)
{
    std::vector<std::string>dirNames;

    const char* PATH = dir_path.c_str();//"/home/ankur/workspace/code/ShapeNet/ShapeNetCore.v1/03001627/";

    DIR *dir = opendir(PATH);

    struct dirent *entry = readdir(dir);

    while (entry != NULL) {
        if (entry->d_type == DT_DIR) {
            std::string cur_dir = std::string(entry->d_name);
            if ( cur_dir.size() > 3 ) {
                cur_dir = dir_path + cur_dir;
                dirNames.push_back(cur_dir);
            }
        }
        entry = readdir(dir);
    }

    closedir(dir);

    return dirNames;
}


void getScalesForModels(std::map<std::string, float>& objects_scales,
                        std::map<std::string, float>& objects_scales_var)
{
#ifdef USE_ARCHIVE_3D_NET_DATA
    objects_scales["beds"] = 1.9;
    objects_scales["chairs"] = 1.0;
    objects_scales["tables"] = 0.9;
    objects_scales["lamps"] = 1.2;
    objects_scales["rack"] = 1.4;
    objects_scales["bin"] = 0.4;
    objects_scales["shoes"] = 0.3;
    objects_scales["cupboard"] = 2.3;
#else
    objects_scales["cupboard"] = 2.0;
    /// Generated from NYU bedrooms with MATLAB
    objects_scales["cabinet"] = 9.515046e-01;
    objects_scales["bed"] = 7.914470e-01;
    objects_scales["chair"] = 7.549843e-01;
    objects_scales["sofa"] = 6.884906e-01;
    objects_scales["table"] = 5.116806e-01;
    objects_scales["book_shelf_stanford"] = 1.5;
    objects_scales["painting"] = 6.509395e-01;
    objects_scales["desk"] = 8.589204e-01;
    objects_scales["curtain"] = 2.0;//1.317684e+00;
    objects_scales["tv"] = 2*5.918130e-01;
    objects_scales["night_stand"] = 5.017400e-01;
    objects_scales["lamp"] = 8.006681e-01;
    objects_scales["plant"] = 5.0e-01;
    objects_scales["clutter"] = 0.28;
    objects_scales["plate"] = 0.01;
    objects_scales["office_desk"] = 1.0;
    objects_scales["gym"] = 0.7;
    objects_scales_var["bed"] = 9.412461e-02;
    objects_scales_var["chair"] = 3.234992e-02;
    objects_scales_var["sofa"] = 2.587976e-02;
    objects_scales_var["table"] = 4.789561e-02;
    objects_scales_var["painting"] = 1.295109e-01;
    objects_scales_var["desk"] = 1.677821e-01;
    objects_scales_var["curtain"] = 2.628007e-01;
    objects_scales_var["tv"] = 2.250542e-02;
    objects_scales_var["night_stand"] = 4.285187e-02;
    objects_scales_var["lamp"] = 0.96080e-01;
    objects_scales_var["plant"] = 3.0e-02;
    objects_scales_var["clutter"] = 0.05;
    objects_scales_var["office_desk"] = 3e-02;
    objects_scales_var["plate"] = 0.01;
    objects_scales_var["gym"] = 0.01;
    objects_scales_var["book_shelf_stanford"] = 1e-3;
    objects_scales_var["cupboard"] = 9e-02;
#endif
    return;
}

void draw_bb_xyz_minmax(float x_min, float x_max,
                        float y_min, float y_max,
                        float z_min, float z_max)
{
    glBegin(GL_QUADS);        // Draw The Cube Using quads
        glColor4f(0.0f,1.0f,0.0f,0.6f);    // Color Blue

//                      glBegin(GL_LINE_LOOP);
        glVertex3f( x_max*1.0f, y_max*1.0f,z_min*1.0f);    // Top Right Of The Quad (Top)
        glVertex3f( x_min*1.0f, y_max*1.0f,z_min*1.0f);    // Top Left Of The Quad (Top)
        glVertex3f(x_min*1.0f, y_max*1.0f, z_max*1.0f);    // Bottom Left Of The Quad (Top)
        glVertex3f( x_max*1.0f, y_max*1.0f, z_max*1.0f);    // Bottom Right Of The Quad (Top)
//                      glEnd();

        glColor4f(1.0f,0.5f,0.0f,0.6f);    // Color Orange

//                      glBegin(GL_LINE_LOOP);
        glVertex3f( x_max*1.0f,y_min*1.0f, z_max*1.0f);    // Top Right Of The Quad (Bottom)
        glVertex3f(x_min*1.0f,y_min*1.0f, z_max*1.0f);    // Top Left Of The Quad (Bottom)
        glVertex3f(x_min*1.0f,y_min*1.0f,z_min*1.0f);    // Bottom Left Of The Quad (Bottom)
        glVertex3f( x_max*1.0f,y_min*1.0f,z_min*1.0f);    // Bottom Right Of The Quad (Bottom)
//                      glEnd();

        glColor4f(1.0f,0.0f,0.0f,0.6f);    // Color Red

//                      glBegin(GL_LINE_LOOP);
        glVertex3f( x_max*1.0f, y_max*1.0f, z_max*1.0f);    // Top Right Of The Quad (Front)
        glVertex3f(x_min*1.0f, y_max*1.0f, z_max*1.0f);    // Top Left Of The Quad (Front)
        glVertex3f(x_min*1.0f,y_min*1.0f, z_max*1.0f);    // Bottom Left Of The Quad (Front)
        glVertex3f( x_max*1.0f,y_min*1.0f, z_max*1.0f);    // Bottom Right Of The Quad (Front)
//                      glEnd();

        glColor4f(1.0f,1.0f,0.0f,0.6f);    // Color Yellow

//                      glBegin(GL_LINE_LOOP);
        glVertex3f( x_max*1.0f,y_min*1.0f,z_min*1.0f);    // Top Right Of The Quad (Back)
        glVertex3f(x_min*1.0f,y_min*1.0f,z_min*1.0f);    // Top Left Of The Quad (Back)
        glVertex3f(x_min*1.0f, y_max*1.0f,z_min*1.0f);    // Bottom Left Of The Quad (Back)
        glVertex3f( x_max*1.0f, y_max*1.0f,z_min*1.0f);    // Bottom Right Of The Quad (Back)
//                      glEnd();

        glColor4f(0.0f,0.0f,1.0f,0.6f);    // Color Blue

//                      glBegin(GL_LINE_LOOP);
        glVertex3f(x_min*1.0f, y_max*1.0f, z_max*1.0f);    // Top Right Of The Quad (Left)
        glVertex3f(x_min*1.0f, y_max*1.0f,z_min*1.0f);    // Top Left Of The Quad (Left)
        glVertex3f(x_min*1.0f,y_min*1.0f,z_min*1.0f);    // Bottom Left Of The Quad (Left)
        glVertex3f(x_min*1.0f,y_min*1.0f, z_max*1.0f);    // Bottom Right Of The Quad (Left)
//                      glEnd();

        glColor4f(1.0f,0.0f,1.0f,0.6f);    // Color Violet

//                      glBegin(GL_LINE_LOOP);
        glVertex3f( x_max*1.0f, y_max*1.0f,z_min*1.0f);    // Top Right Of The Quad (Right)
        glVertex3f( x_max*1.0f, y_max*1.0f, z_max*1.0f);    // Top Left Of The Quad (Right)
        glVertex3f( x_max*1.0f,y_min*1.0f, z_max*1.0f);    // Bottom Left Of The Quad (Right)
        glVertex3f( x_max*1.0f,y_min*1.0f,z_min*1.0f);    // Bottom Right Of The Quad (Right)
        glEnd();            // End Drawing The Cube - See more at: http://www.codemiles.com/c-opengl-examples/draw-3d-cube-using-opengl-t9018.html#sthash.vKmu1Epd.dpuf

}

void draw_bbox_only(TooN::Vector<3>& size_,
                    TooN::Vector<3>& center_)
{

    glPushMatrix();

    glColor3f(1,0,1);

    glTranslatef(center_[0],center_[1],center_[2]);
    glScalef(size_[0],size_[1],size_[2]);


    glBegin(GL_LINES);

    //front
    glVertex3f(-0.5f, 0.5f, 0.5f);
    glVertex3f(0.5f, 0.5f, 0.5f);

    glVertex3f(0.5f, 0.5f, 0.5f);
    glVertex3f(0.5f, -0.5f, 0.5f);

    glVertex3f(0.5f, -0.5f, 0.5f);
    glVertex3f(-0.5f, -0.5f, 0.5f);

    glVertex3f(-0.5f, -0.5f, 0.5f);
    glVertex3f(-0.5f, 0.5f, 0.5f);

    //right
    glVertex3f(0.5f, 0.5f, 0.5f);
    glVertex3f(0.5f, 0.5f, -0.5f);

    glVertex3f(0.5f, 0.5f, -0.5f);
    glVertex3f(0.5f, -0.5f, -0.5f);

    glVertex3f(0.5f, -0.5f, -0.5f);
    glVertex3f(0.5f, -0.5f, 0.5f);

    //back
    glVertex3f(0.5f, 0.5f, -0.5f);
    glVertex3f(-0.5f, 0.5f, -0.5f);

    glVertex3f(-0.5f, -0.5f, -0.5f);
    glVertex3f(0.5f, -0.5f, -0.5f);

    glVertex3f(-0.5f, -0.5f, -0.5f);
    glVertex3f(-0.5f, 0.5f, -0.5f);

    //left
    glVertex3f(-0.5f, 0.5f, -0.5f);
    glVertex3f(-0.5f, 0.5f, 0.5f);

    glVertex3f(-0.5f, -0.5f, 0.5f);
    glVertex3f(-0.5f, -0.5f, -0.5f);
    glEnd();

    glPopMatrix();

    glClearColor(0,0,0,0);

}

void get_bbox(tinyobj::mesh_t* mesh,
              TooN::Vector<3>& size_,
              TooN::Vector<3>& center_)
{
    GLfloat
      min_x, max_x,
      min_y, max_y,
      min_z, max_z;
    min_x = max_x = mesh->positions[0];
    min_y = max_y = mesh->positions[1];
    min_z = max_z = mesh->positions[2];
    for (int i = 0; i < mesh->positions.size()/3; i++) {
      if (mesh->positions[3*i+0] < min_x) min_x = mesh->positions[3*i+0];
      if (mesh->positions[3*i+0] > max_x) max_x = mesh->positions[3*i+0];
      if (mesh->positions[3*i+1] < min_y) min_y = mesh->positions[3*i+1];
      if (mesh->positions[3*i+1] > max_y) max_y = mesh->positions[3*i+1];
      if (mesh->positions[3*i+2] < min_z) min_z = mesh->positions[3*i+2];
      if (mesh->positions[3*i+2] > max_z) max_z = mesh->positions[3*i+2];
    }
    glm::vec3 size = glm::vec3(max_x-min_x, max_y-min_y, max_z-min_z);
    glm::vec3 center = glm::vec3((min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2);

    size_   = TooN::makeVector(size.x,size.y,size.z);
    center_ = TooN::makeVector(center.x,center.y,center.z);
}

void get_bbox_vec(std::vector<float> mesh,
              TooN::Vector<3>& size_,
              TooN::Vector<3>& center_)
{
    GLfloat
      min_x, max_x,
      min_y, max_y,
      min_z, max_z;
    min_x = max_x = mesh.at(0);
    min_y = max_y = mesh.at(1);
    min_z = max_z = mesh.at(2);

    for (int i = 0; i < mesh.size()/3; i++)
    {

      if (mesh.at(3*i+0) < min_x) min_x = mesh.at(3*i+0);
      if (mesh.at(3*i+0) > max_x) max_x = mesh.at(3*i+0);

      if (mesh.at(3*i+1) < min_y) min_y = mesh.at(3*i+1);
      if (mesh.at(3*i+1) > max_y) max_y = mesh.at(3*i+1);

      if (mesh.at(3*i+2) < min_z) min_z = mesh.at(3*i+2);
      if (mesh.at(3*i+2) > max_z) max_z = mesh.at(3*i+2);
    }

    glm::vec3 size = glm::vec3(max_x-min_x, max_y-min_y, max_z-min_z);
    glm::vec3 center = glm::vec3((min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2);

    size_   = TooN::makeVector(size.x,size.y,size.z);
    center_ = TooN::makeVector(center.x,center.y,center.z);
}

void draw_bbox(tinyobj::mesh_t* mesh) {
  if (mesh->positions.size() == 0)
    return;

  GLfloat
    min_x, max_x,
    min_y, max_y,
    min_z, max_z;
  min_x = max_x = mesh->positions[0];
  min_y = max_y = mesh->positions[1];
  min_z = max_z = mesh->positions[2];
  for (int i = 0; i < mesh->positions.size()/3; i++) {
    if (mesh->positions[3*i+0] < min_x) min_x = mesh->positions[3*i+0];
    if (mesh->positions[3*i+0] > max_x) max_x = mesh->positions[3*i+0];
    if (mesh->positions[3*i+1] < min_y) min_y = mesh->positions[3*i+1];
    if (mesh->positions[3*i+1] > max_y) max_y = mesh->positions[3*i+1];
    if (mesh->positions[3*i+2] < min_z) min_z = mesh->positions[3*i+2];
    if (mesh->positions[3*i+2] > max_z) max_z = mesh->positions[3*i+2];
  }
  glm::vec3 size = glm::vec3(max_x-min_x, max_y-min_y, max_z-min_z);
  glm::vec3 center = glm::vec3((min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2);

  glPushMatrix();

  glColor3f(1,0,1);

  glTranslatef(center.x,center.y,center.z);
  glScalef(size.x,size.y,size.z);
  // White side - BACK

  glBegin(GL_LINES);

  //front
  glVertex3f(-0.5f, 0.5f, 0.5f);
  glVertex3f(0.5f, 0.5f, 0.5f);

  glVertex3f(0.5f, 0.5f, 0.5f);
  glVertex3f(0.5f, -0.5f, 0.5f);

  glVertex3f(0.5f, -0.5f, 0.5f);
  glVertex3f(-0.5f, -0.5f, 0.5f);

  glVertex3f(-0.5f, -0.5f, 0.5f);
  glVertex3f(-0.5f, 0.5f, 0.5f);

  //right
  glVertex3f(0.5f, 0.5f, 0.5f);
  glVertex3f(0.5f, 0.5f, -0.5f);

  glVertex3f(0.5f, 0.5f, -0.5f);
  glVertex3f(0.5f, -0.5f, -0.5f);

  glVertex3f(0.5f, -0.5f, -0.5f);
  glVertex3f(0.5f, -0.5f, 0.5f);

  //back
  glVertex3f(0.5f, 0.5f, -0.5f);
  glVertex3f(-0.5f, 0.5f, -0.5f);

  glVertex3f(-0.5f, -0.5f, -0.5f);
  glVertex3f(0.5f, -0.5f, -0.5f);

  glVertex3f(-0.5f, -0.5f, -0.5f);
  glVertex3f(-0.5f, 0.5f, -0.5f);

  //left
  glVertex3f(-0.5f, 0.5f, -0.5f);
  glVertex3f(-0.5f, 0.5f, 0.5f);

  glVertex3f(-0.5f, -0.5f, 0.5f);
  glVertex3f(-0.5f, -0.5f, -0.5f);
  glEnd();

  glPopMatrix();

  glClearColor(0,0,0,0);
}

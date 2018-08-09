#ifndef _ASSIMP_OBJ_LOADER_H
#define _ASSIMP_OBJ_LOADER_H

#include <iostream>

#include <TooN/TooN.h>


#include <assimp/vector3.h>
#include <assimp/types.h>
#include <assimp/cimport.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>

#include <set>
#include <map>
#include <vector>

#include <vector_types.h>
#include <vector_functions.h>

#include <fstream>
#include <boost/filesystem.hpp>

#include <TooN/se3.h>

using namespace std;

class AssimpObjLoader{

public:
    AssimpObjLoader()
    {

    }
    AssimpObjLoader(std::string& objfileName,
                    std::string _wnid = "")
    {
        wnid = _wnid;

        objectFileName = objfileName;

        Assimp::Importer importer;

        const aiScene *m_scene =importer.ReadFile( objfileName,
                                                   aiProcess_Triangulate     |
                                                   aiProcess_JoinIdenticalVertices  |
                                                   aiProcess_FindInvalidData |
                                                   aiProcess_CalcTangentSpace
                                                   );

        /// Get the contents of the obj file into the arrays
        /// 1. Find the number of total vertices.
        /// 2. Find the number of submeshes.
        /// Use this to initialise the arrays.

        min_x =  1E10;
        max_x = -1E10;

        min_y =  1E10;
        max_y = -1E10;

        min_z =  1E10;
        max_z = -1E10;

        total_vertices_floor = 0;
        total_faces_floor    = 0;

        total_faces    = 0;
        total_meshes   = 0;

        for(int i =0; i < m_scene->mNumMeshes; i++)
        {
            aiMesh* mesh = m_scene->mMeshes[i];

              std::cout<<"Mesh:"<<mesh->mName.C_Str()<<std::endl;
            if (wnid.length() > 1 && !mesh->HasNormals())
            {
              continue;
            }

            std::string current_mesh_name(mesh->mName.C_Str());

            for(int s = 0; s < current_mesh_name.size(); s++)
            {
                current_mesh_name.at(s) = std::tolower(current_mesh_name.at(s));
            }

            //if ( current_mesh_name.find("floor") != std::string::npos)
            {
                total_vertices_floor += mesh->mNumVertices;
                total_faces_floor    += mesh->mNumFaces;
            }

            total_faces    += mesh->mNumFaces;

            total_meshes++;
            std::cout<<"total meshes:"<<total_meshes<<std::endl;
        }

        std::cout<<"Number of Meshes loaded = " << total_meshes << std::endl;


        vertex2face_floor  = std::vector< std::vector<int> >(total_vertices_floor);

        index2vertex_floor = std::vector< std::vector<float> > (total_vertices_floor);
        face2vertex_floor  = std::vector< std::vector<int> > (total_faces_floor);

        shape_vertices = std::vector<float*>(total_meshes,NULL);
        shape_normals  = std::vector<float*>(total_meshes,NULL);
        number_of_vertices_x3_shape = std::vector<int>(total_meshes);

        all_vertices = new float[total_faces*3*3];
        all_normals  = new float[total_faces*3*3];

        all_vertices_array_size = total_faces*3*3;
        all_normals_array_size  = total_faces*3*3;

        int vface = 0;
        int face_no =0;
        int mesh_no = 0;

        /// offset for vertices
        int offset = 0;

        for(int i =0; i < m_scene->mNumMeshes; i++)
        {
            aiMesh* mesh = m_scene->mMeshes[i];

            aiVector3D* vertices_f3 = reinterpret_cast<aiVector3D*>( mesh->mVertices );
            //aiVector3D* normals_f3  = reinterpret_cast<aiVector3D*>( mesh->mNormals );

            int count = 0;

            if (wnid.length() > 1 && !mesh->HasNormals())
            {
              continue;
            }

            std::cout<<"mesh name = "<< mesh->mName.C_Str() << std::endl;
            std::string current_mesh_name(mesh->mName.C_Str());

            for(int s = 0; s < current_mesh_name.size(); s++)
            {
                current_mesh_name.at(s) = std::tolower(current_mesh_name.at(s));
            }

            meshNames.push_back(current_mesh_name);


            shape_vertices[mesh_no] = new float[mesh->mNumFaces*3*3];
             shape_normals[mesh_no] = new float[mesh->mNumFaces*3*3];

            for(unsigned int f = 0; f < mesh->mNumFaces; f++)
            {
                aiFace face = mesh->mFaces[f];

                int v1 = face.mIndices[0];
                int v2 = face.mIndices[1];
                int v3 = face.mIndices[2];

                TooN::Vector<3>vertex_v1 = TooN::makeVector(vertices_f3[v1].x,
                                                            vertices_f3[v1].y,
                                                            vertices_f3[v1].z);

                TooN::Vector<3>vertex_v2 =  TooN::makeVector(vertices_f3[v2].x,
                                                             vertices_f3[v2].y,
                                                             vertices_f3[v2].z);


                TooN::Vector<3>vertex_v3 = TooN::makeVector(vertices_f3[v3].x,
                                                            vertices_f3[v3].y,
                                                            vertices_f3[v3].z);

                /// Compute the geometric normal

                TooN::Vector<3>v1v2 = vertex_v1 - vertex_v2;
                TooN::Vector<3>v3v2 = vertex_v3 - vertex_v2;

                TooN::Vector<3>normal_cross_product = v1v2 ^ v3v2;

                float norm = std::sqrt(normal_cross_product[0]*normal_cross_product[0] +
                            normal_cross_product[1]*normal_cross_product[1] +
                            normal_cross_product[2]*normal_cross_product[2]);

                normal_cross_product = normal_cross_product / norm;

                /// Populate the first vertex

                shape_vertices[mesh_no][count+0] = vertex_v1[0];
                shape_vertices[mesh_no][count+1] = vertex_v1[1];
                shape_vertices[mesh_no][count+2] = vertex_v1[2];

                all_vertices[vface+0] = vertices_f3[v1].x;
                all_vertices[vface+1] = vertices_f3[v1].y;
                all_vertices[vface+2] = vertices_f3[v1].z;

                /*
                norm = sqrt(normals_f3[v1].x*normals_f3[v1].x +
                            normals_f3[v1].y*normals_f3[v1].y +
                            normals_f3[v1].z*normals_f3[v1].z);
                            */

                all_normals[vface+0] =  normal_cross_product[0];//(normals_f3[v1].x)/norm;
                all_normals[vface+1] =  normal_cross_product[1];//(normals_f3[v1].y)/norm;
                all_normals[vface+2] =  normal_cross_product[1];//(normals_f3[v1].z)/norm;
                vface+=3;

                count+=3;

                /// Populate the second vertex

                shape_vertices[mesh_no][count+0] = vertex_v2[0];
                shape_vertices[mesh_no][count+1] = vertex_v2[1];
                shape_vertices[mesh_no][count+2] = vertex_v2[2];

                all_vertices[vface+0] = vertices_f3[v2].x;
                all_vertices[vface+1] = vertices_f3[v2].y;
                all_vertices[vface+2] = vertices_f3[v2].z;

                /*
                norm = sqrt(normals_f3[v2].x*normals_f3[v2].x +
                            normals_f3[v2].y*normals_f3[v2].y +
                            normals_f3[v2].z*normals_f3[v2].z);
                            */

                all_normals[vface+0]  =  normal_cross_product[0];//normals_f3[v2].x/norm;
                all_normals[vface+1]  =  normal_cross_product[1];//normals_f3[v2].y/norm;
                all_normals[vface+2]  =  normal_cross_product[2];//normals_f3[v2].z/norm;
                vface+=3;


                count+=3;


                /// Populate the third vertex

                shape_vertices[mesh_no][count+0] = vertex_v3[0];
                shape_vertices[mesh_no][count+1] = vertex_v3[1];
                shape_vertices[mesh_no][count+2] = vertex_v3[2];

                all_vertices[vface+0] = vertices_f3[v3].x;
                all_vertices[vface+1] = vertices_f3[v3].y;
                all_vertices[vface+2] = vertices_f3[v3].z;

                /*
                norm = sqrt(normals_f3[v3].x*normals_f3[v3].x +
                            normals_f3[v3].y*normals_f3[v3].y +
                            normals_f3[v3].z*normals_f3[v3].z);
                            */

                all_normals[vface+0] =  normal_cross_product[0];//normals_f3[v3].x/norm;
                all_normals[vface+1] =  normal_cross_product[1];//normals_f3[v3].y/norm;
                all_normals[vface+2] =  normal_cross_product[2];//normals_f3[v3].z/norm;
                vface+=3;

                count+=3;


                std::vector<float>vertexX;
                vertexX.push_back(vertex_v1[0]);
                vertexX.push_back(vertex_v2[0]);
                vertexX.push_back(vertex_v3[0]);


                std::vector<float>vertexY;
                vertexY.push_back(vertex_v1[1]);
                vertexY.push_back(vertex_v2[1]);
                vertexY.push_back(vertex_v3[1]);


                std::vector<float>vertexZ;
                vertexZ.push_back(vertex_v1[2]);
                vertexZ.push_back(vertex_v2[2]);
                vertexZ.push_back(vertex_v3[2]);

                std::sort(vertexX.begin(),vertexX.end());
                std::sort(vertexY.begin(),vertexY.end());
                std::sort(vertexZ.begin(),vertexZ.end());

                if ( min_x > vertexX.at(0) )
                    min_x = vertexX.at(0);

                if ( max_x < vertexX.at(2) )
                    max_x = vertexX.at(2);

                if ( min_y > vertexY.at(0) )
                    min_y = vertexY.at(0);

                if ( max_y < vertexY.at(2) )
                    max_y = vertexY.at(2);

                if ( min_z > vertexZ.at(0) )
                    min_z = vertexZ.at(0);

                if ( max_z < vertexZ.at(2) )
                    max_z = vertexZ.at(2);


//                mean_point[0] += vertex_v1[0];
//                mean_point[1] += vertex_v1[1];
//                mean_point[2] += vertex_v1[2];

//                mean_point[0] += vertex_v2[0];
//                mean_point[1] += vertex_v2[1];
//                mean_point[2] += vertex_v2[2];

//                mean_point[0] += vertex_v3[0];
//                mean_point[1] += vertex_v3[1];
//                mean_point[2] += vertex_v3[2];

//                if ( current_mesh_name.find("floor") != std::string::npos)
                /*{

                    vertex2face_floor.at(v1+offset).push_back(face_no);
                    vertex2face_floor.at(v2+offset).push_back(face_no);
                    vertex2face_floor.at(v3+offset).push_back(face_no);

                    face2vertex_floor.at(face_no).push_back(v1+offset);
                    face2vertex_floor.at(face_no).push_back(v2+offset);
                    face2vertex_floor.at(face_no).push_back(v3+offset);

                    std::vector<float>XYZ;

                    XYZ.push_back(vertices_f3[v1].x);
                    XYZ.push_back(vertices_f3[v1].y);
                    XYZ.push_back(vertices_f3[v1].z);

                    index2vertex_floor.at(v1+offset) = XYZ;

                    XYZ = std::vector<float>();

                    XYZ.push_back(vertices_f3[v2].x);
                    XYZ.push_back(vertices_f3[v2].y);
                    XYZ.push_back(vertices_f3[v2].z);

                    index2vertex_floor.at(v2+offset) = XYZ;


                    XYZ = std::vector<float>();

                    XYZ.push_back(vertices_f3[v3].x);
                    XYZ.push_back(vertices_f3[v3].y);
                    XYZ.push_back(vertices_f3[v3].z);

                    index2vertex_floor.at(v3+offset) = XYZ;


//                    floor_vertices[floor_count_vert+0] = vertices_f3[v1].x;
//                    floor_vertices[floor_count_vert+1] = vertices_f3[v1].y;
//                    floor_vertices[floor_count_vert+2] = vertices_f3[v1].z;

//                    floor_vertices[floor_count_vert+3] = vertices_f3[v2].x;
//                    floor_vertices[floor_count_vert+4] = vertices_f3[v2].y;
//                    floor_vertices[floor_count_vert+5] = vertices_f3[v2].z;

//                    floor_vertices[floor_count_vert+6] = vertices_f3[v3].x;
//                    floor_vertices[floor_count_vert+7] = vertices_f3[v3].y;
//                    floor_vertices[floor_count_vert+8] = vertices_f3[v3].z;

//                    floor_count_vert+= 9;

                    face_no++;
                }*/

            }

//            if ( current_mesh_name.find("floor") != std::string::npos )
                    offset = offset + mesh->mNumVertices;

            number_of_vertices_x3_shape[mesh_no] = mesh->mNumFaces*3*3;
            mesh_no++;
        }


//        int number_floor_vertices = 0;


        /// Get the Floor Plan

        /*std::map<std::pair<int,int>,int>pair_map;
        std::map<std::pair<int,int>,int>::iterator iter;

        for(int f = 0; f < face_no; f++)
        {
            int v1 = face2vertex_floor.at(f)[0];
            int v2 = face2vertex_floor.at(f)[1];
            int v3 = face2vertex_floor.at(f)[2];


            std::pair<int, int>edge_pair1(v1,v2);
            std::pair<int, int>edge_pair2(v2,v1);

            if ( pair_map.find(edge_pair1) != pair_map.end() ||
                 pair_map.find(edge_pair2) != pair_map.end() )
            {
                pair_map[edge_pair1] = 1;
                pair_map[edge_pair2] = 1;

//                std::cout<<"edge already exists" << std::endl;
            }
            else
            {
                pair_map[edge_pair1] = 0;
//                pair_map[edge_pair2] = 0;
            }

            edge_pair1 = std::make_pair(v2,v3);
            edge_pair2 = std::make_pair(v3,v2);

            if ( pair_map.find(edge_pair1) != pair_map.end() ||
                 pair_map.find(edge_pair2) != pair_map.end() )
            {
                pair_map[edge_pair1] = 1;
                pair_map[edge_pair2] = 1;

//                std::cout<<"edge already exists" << std::endl;
            }
            else
            {
                pair_map[edge_pair1] = 0;
//                pair_map[edge_pair2] = 0;
            }

            edge_pair1 = std::make_pair(v1,v3);
            edge_pair2 = std::make_pair(v3,v1);

            if ( pair_map.find(edge_pair1) != pair_map.end() ||
                 pair_map.find(edge_pair2) != pair_map.end() )
            {
                pair_map[edge_pair1] = 1;
                pair_map[edge_pair2] = 1;

//                std::cout<<"edge already exists" << std::endl;
            }
            else
            {
                pair_map[edge_pair1] = 0;
//                pair_map[edge_pair2] = 0;
            }

//            for(iter = pair_map.begin(); iter != pair_map.end(); iter++ )
            {
//                std::pair<int,int>edge_pair = iter->first;
//                std::cout<<"pair: " << edge_pair.first<<", "<< edge_pair.second<<", "<< iter->second << std::endl;
            }

//            getchar();

        }

        std::map<std::pair<int,int>,int>::iterator iterv;


//        std::cout<<"edges.. " << std::endl;
//        for(iter = pair_map.begin(); iter != pair_map.end(); iter++ )
//        {
//            std::pair<int,int>edge_pair = iter->first;
//            std::cout<<"pair: " << edge_pair.first<<", "<< edge_pair.second<<", "<< iter->second << std::endl;
//        }

        std::map<std::pair<int,int>, int >peripheral_edges;
        peripheral_edges = pair_map;

        std::cout<<"peripheral_edges: before = " << peripheral_edges.size() << std::endl;

        for(iterv = peripheral_edges.begin(), iter = pair_map.begin();
            iterv != peripheral_edges.end() && iter != pair_map.end();
            iterv++, iter++)
        {
            ///edges that already exist should be removed!
            if ( iter->second == 1 )
            {
                peripheral_edges.erase(iterv);
            }
        }

        std::cout<<"peripheral_edges: after = " << peripheral_edges.size() << std::endl;



        for(iterv = peripheral_edges.begin();
            iterv != peripheral_edges.end();
            iterv++)
        {
            std::pair<int,int>edge_pair = iterv->first;

            std::vector<float>v1XYZ = index2vertex_floor.at(edge_pair.first);
            std::vector<float>v2XYZ = index2vertex_floor.at(edge_pair.second);

            peripheral_vertices.push_back(v1XYZ);
            peripheral_vertices.push_back(v2XYZ);

        }*/

    }

    std::string get_obj_file_path()
    {
        return objectFileName;
    }

    std::string get_wnid()
    {
        return wnid;
    }

    int get_all_vertices_size()
    {
        return all_vertices_array_size;
    }

    int get_all_normals_size()
    {
        return all_normals_array_size;
    }

    float* get_all_vertices()
    {
        return all_vertices;
    }

    float* get_all_normals()
    {
        return all_normals;
    }

    std::vector<int> get_numVertes_submeshes()
    {
        return number_of_vertices_x3_shape;
    }

    std::vector<float*> get_shape_vertices()
    {
        return shape_vertices;
    }

    int get_numMeshes()
    {
        return total_meshes;
    }

    std::vector<std::string>getMeshNames()
    {
        return meshNames;
    }

    std::vector<std::vector<float> > getPeripheralVertices()
    {
        return peripheral_vertices;
    }

    std::vector<std::vector<int> > get_face2vertex_floor()
    {
        return face2vertex_floor;
    }
    std::vector<std::vector<float> > get_index2vertex_floor()
    {
        return index2vertex_floor;
    }

    std::vector<float> get_min_max_3d_bounding_box()
    {
        std::vector<float>min_max_bb;

        min_max_bb.push_back(min_x);
        min_max_bb.push_back(max_x);

        min_max_bb.push_back(min_y);
        min_max_bb.push_back(max_y);

        min_max_bb.push_back(min_z);
        min_max_bb.push_back(max_z);

        return min_max_bb;
    }



public:

    float* all_normals;
    float* all_vertices;

    int total_meshes;
    int total_faces;

    int total_vertices_floor;
    int total_faces_floor;

    int all_vertices_array_size;
    int all_normals_array_size;

    std::vector<float*>shape_vertices;//(total_meshes,NULL);
    std::vector<float*>shape_normals;//(total_meshes,NULL);

    std::vector<int>number_of_vertices_x3_shape;//(m_scene->mNumMeshes,0);

    std::vector<std::string>meshNames;

    std::vector< std::vector<int> >vertex2face_floor;//(total_vertices_floor);
    std::vector< std::vector<int> > face2vertex_floor;//(total_faces_floor);
    std::vector< std::vector<float> > index2vertex_floor;

//    std::vector< std::vector<flot> > vertices_floor;
    std::vector< std::vector<float> > peripheral_vertices;

    std::string wnid, objectFileName;

    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;


};

class ShapeNetModel{

public:

    ShapeNetModel(std::string& model_info_file,
                  std::string& shapenets_sunrgbd_mapping_file)
    {
        ifstream models_file(model_info_file);

        //"/home/dysondemo/workspace/code/ShapeNetNew/obj/model_info_and_texture.txt";

        char readlinedata[1300];

        double up_x, up_y, up_z;
        double front_x, front_y, front_z;

        int count_dir = 0;

        while(1)
        {
            models_file.getline(readlinedata,1300);

            if ( models_file.eof())
                break;


            istringstream iss(readlinedata);

            iss >> directory_string;

            directories.push_back(directory_string);

            iss >> wnids;

            {
                std::replace(wnids.begin(),wnids.end(),',',' ');
                istringstream wnid_split(wnids);

                wnid_split >> wnids;
            }

            wnids_shapenet.push_back(wnids);

            iss >> tags;

            iss >> up_vector;

            iss >> front_vector;

            std::replace(up_vector.begin(),up_vector.end(),',',' ');
            istringstream s_iss(up_vector);

            s_iss >> up_x;
            s_iss >> up_y;
            s_iss >> up_z;

            model_up_vector.push_back(make_float3(up_x, up_y, up_z));

            std::replace(front_vector.begin(),front_vector.end(),',',' ');
            istringstream f_iss(front_vector);

            f_iss >> front_x;
            f_iss >> front_y;
            f_iss >> front_z;

            model_front_vector.push_back(make_float3(front_x, front_y, front_z));

            if (up_z && !up_x && !up_y)
            {
    //            std::cout<<"count_dir = " << count_dir;
    //            std::cout<<", directory    = " << directory_string;

    //            std::cout<<", up_vector = " << up_x <<", " << up_y <<", "<<up_z;
    //            std::cout<<", front_vector = " << front_x <<", " << front_y <<", "<<front_z << std::endl;

            }

            count_dir = count_dir+1;


        }


        std::cout<<"count = " << count_dir << std::endl;

        models_file.close();


        ifstream ShapeNets2SUNRGBDMapping(shapenets_sunrgbd_mapping_file);

        while(1)
        {
            ShapeNets2SUNRGBDMapping.getline(readlinedata,1300);

            if ( ShapeNets2SUNRGBDMapping.eof())
                break;

            istringstream iss(readlinedata);

            std::string wnid, shapenet_object_name, isFound, sunrgbd_mapping;

            iss >> wnid;
            iss >> shapenet_object_name;
            iss >> isFound;

            if ( isFound == "Found")
            {
                iss >> sunrgbd_mapping;

                wnid_to_sunrgbd_mapping[wnid] = sunrgbd_mapping;
            }

        }

        ShapeNets2SUNRGBDMapping.close();

        std::cout<<"wnid_to_sunrgbd_mapping = " << wnid_to_sunrgbd_mapping.size() << std::endl;



    }

    std::string get_sunrgbd_object_mapping(std::string& wnid)
    {
        if ( wnid_to_sunrgbd_mapping.find(wnid) == wnid_to_sunrgbd_mapping.end())
            return "NOTFOUND";
        else
            return wnid_to_sunrgbd_mapping[wnid];
    }

    void read_random_model(AssimpObjLoader*& thisobjmodel)
    {
        int rand_directory = ((float)rand()/RAND_MAX)*(directories.size()-1);

        std::string obj_basename = string("/home/dysondemo/workspace/code/ShapeNetNew/obj/")
                                    + std::string(directories.at(rand_directory)) + std::string("/models/model.obj");

        std::cout<<"Trying to read this file..." << obj_basename << std::endl;

        if ( !boost::filesystem::exists(obj_basename) )
        {
            int model_length = std::string("model.obj").length();
            obj_basename = obj_basename.replace(obj_basename.length()-model_length,
                                                std::string("warehouse_model.obj").length(),
                                                "warehouse_model.obj");

            std::cout<<"objname = " << obj_basename << std::endl;
        }

        if ( !boost::filesystem::exists(obj_basename) )
        {
            obj_basename = string("/home/dysondemo/workspace/code/ShapeNetNew/obj/")
                           + std::string(directories.at(rand_directory)) + std::string("/models/untitled.obj");
        }



        std::string this_wnid = wnids_shapenet.at(rand_directory);

        thisobjmodel = new AssimpObjLoader(obj_basename, this_wnid);

        std::cout<<"This random model has been read..." << std::endl;

        return;
    }

public:

    std::vector<std::string>directories;
    std::string directory_string, wnids, tags, up_vector, front_vector;

    std::map<std::string, std::string>wnid_to_sunrgbd_mapping;

    std::vector<std::string>wnids_shapenet;

    std::vector<float3>model_up_vector;
    std::vector<float3>model_front_vector;

};


class SceneNetParser{

public:

    SceneNetParser(std::string fileName)
    {

        ifstream ifile(fileName);

        std::string parsedstring, previousstring, wnid;

        int count = 0;

        while(1)
        {
            ifile >> parsedstring;

            /// The first string - the path to the floor plan
            if ( count == 0 )
            {
                parsedstring = std::string("/home/dysondemo/workspace/code/ScenenetLayouts/")
                                + parsedstring;

                model_paths.push_back(parsedstring);

                scales.push_back(1);

                std::string wnid="000000";

                wnids.push_back(wnid);

                TooN::SE3<>T_wc;

                transformations.push_back(T_wc);

                count++;
            }

            if ( ifile.eof() )
                break;

            std::cout<<parsedstring<<std::endl;

            if ( previousstring == "object" )
            {
                model_paths.push_back(parsedstring);
            }

            if ( previousstring == "scale" )
            {
                float scale = atof(parsedstring.c_str());
                scales.push_back(scale);
            }

            if ( previousstring == "wnid")
            {
                wnids.push_back(parsedstring);
            }

            if ( parsedstring == "transformation" )
            {
                TooN::SE3<>T_wc;

                ifile >> T_wc;

                std::cout<<"T_wc = " << T_wc << std::endl;

                transformations.push_back(T_wc);
            }

            previousstring = parsedstring;

        }

        ifile.close();

        for(int i = 0; i < transformations.size(); i++)
        {
            std::cout<<"object = " << model_paths.at(i) << std::endl;
            std::cout<<"wnid   = " << wnids.at(i) << std::endl;
            std::cout<<"scale  = " << scales.at(i) << std::endl;
            std::cout<<"T_wc   = " << transformations.at(i) << std::endl;
        }

    }



public:

    std::vector<std::string>model_paths;
    std::vector<float>scales;
    std::vector<std::string>wnids;
    std::vector<TooN::SE3<> > transformations;

};


#endif

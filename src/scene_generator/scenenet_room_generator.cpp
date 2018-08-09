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

#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactContainerDEM.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/solver/ChSolverDEM.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include <gsl/gsl_histogram.h>
#include <gsl/gsl_rng.h>

#include <algorithm>
#include <dirent.h>
#include <memory>
#include <random>
#include <set>
#include <stdlib.h>
#include <vector>


using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;

using namespace std;

const float kSMALL_OBJECT_SIZE = 0.4;
const std::string dataset_split("train"); // This is the set from which to build the scene, e.g. train,val, or test

class ObjScaling {
public:
  ObjScaling(std::string fileName) : percentile(0.0f) {
    std::ifstream object_scale_file(fileName);
    int n_chars = 3000;
    char readlinedata[n_chars];
    std::string room_type, objectName;
    object_scale_file.getline(readlinedata,n_chars);
    int n_lines = 0;
    while(true) {
        object_scale_file.getline(readlinedata,n_chars);
        std::string myreadline(readlinedata);
        if (object_scale_file.eof())
            break;
        n_lines++;
        istringstream iss(readlinedata);
        int i = 0, k = 0;
        std::vector<float>dims;
        while(iss.good()) {
            std::string substr;
            getline( iss, substr, ',' );
            if ( i == 0 ) {
                room_type = substr;
            } else {
                if( std::any_of(substr.begin(), substr.end(), ::isdigit) && k < 3) {
                    size_t found = substr.find('(');
                    if (found != std::string::npos)
                        substr = substr.substr(found+1,substr.size());
                    found = substr.find(')');
                    if (found != std::string::npos)
                        substr = substr.substr(0,substr.size()-1);
                    dims.push_back(atof(substr.c_str()));
                    k++;
                    if (k == 3) {
                        std::tuple<float, float, float> wdh;
                        wdh = std::make_tuple(dims.at(0), dims.at(1), dims.at(2));
                        object_minMax_mapping[objectName].push_back(wdh);
                        k = 0;
                        dims.clear();
                    }
                } else {
                    objectName = substr;
                }
            }
            i++;
        }
    }
    std::cout<<"Files have been read... "<< std::endl;
    object_scale_file.close();
    prepare_histograms();
    std::cout<<"Complete... "<< std::endl;
  }


/// Ref: http://stackoverflow.com/questions/18456761/using-gsl-histogram-pdf-sample-to-sample-from-an-ad-hoc-distribution
    void prepare_histograms() {
        std::map<std::string, std::vector<std::tuple<float, float, float> > >::iterator object_itr;
        for(object_itr = object_minMax_mapping.begin();
            object_itr != object_minMax_mapping.end();
            object_itr++) {
            float min_val =  1000.0f;
            float max_val = -1000.0f;
            ///Get the min and max
            if ( object_itr->second.size() < 2) {
              continue;
            }
            std::vector<float>x_div_y;
            std::vector<float>z_div_y;
            for(int i = 0 ; i < object_itr->second.size(); i++) {
                if (min_val > std::get<2>(object_itr->second.at(i)))
                    min_val = std::get<2>(object_itr->second.at(i));
                if (max_val < std::get<2>(object_itr->second.at(i)))
                    max_val = std::get<2>(object_itr->second.at(i));
                x_div_y.push_back(std::get<0>(object_itr->second.at(i))/
                                  std::get<2>(object_itr->second.at(i)));
                z_div_y.push_back(std::get<1>(object_itr->second.at(i))/
                                  std::get<2>(object_itr->second.at(i)));
            }
            if (max_val == min_val)
              max_val += 0.000001;
            std::sort(x_div_y.begin(), x_div_y.end());
            std::sort(z_div_y.begin(), z_div_y.end());
            size_t  Bins = 50;
            if (object_itr->second.size() < 50) {
              Bins = object_itr->second.size();
            }
            gsl_histogram* hist = gsl_histogram_alloc (Bins);
            assert( hist != NULL );
            gsl_histogram_set_ranges_uniform(hist, min_val, max_val);
            for(int i = 0 ; i < object_itr->second.size(); i++) {
                std::tuple<float, float, float> u = object_itr->second.at(i);
                gsl_histogram_increment (hist, std::get<2>(u));
            }
            obj_scale_histograms[object_itr->first] = hist;

            float x_div_y_min = x_div_y[0];
            float x_div_y_max = x_div_y[x_div_y.size()-1];
            obj_x_div_y_bounds[object_itr->first] = std::tuple<float,float>(x_div_y_min, x_div_y_max);
            float z_div_y_min = z_div_y[0];
            float z_div_y_max = z_div_y[z_div_y.size() - 1];
            obj_z_div_y_bounds[object_itr->first] = std::tuple<float,float>(z_div_y_min, z_div_y_max);
        }
    }

    void get_histogram(std::string object_name, std::vector<float>& histogram) {
        std::map<std::string,gsl_histogram* >::iterator object_hist_itr;
        for(object_hist_itr  = obj_scale_histograms.begin();
            object_hist_itr != obj_scale_histograms.end();
            object_hist_itr++) {
            if (object_hist_itr->first.find(object_name) != std::string::npos &&
                 (object_hist_itr->first.size() -  object_name.size()) <= 1 ) {
                std::cout<<object_hist_itr->first<<std::endl;
                int size_hist = object_hist_itr->second->n;
                for(int i = 0; i < size_hist; i++) {
                    histogram.push_back(object_hist_itr->second->bin[i]);
                }
                break;
            }
        }
    }

    double get_scale_via_probability(std::string& object_name) {
        std::map<std::string,gsl_histogram* >::iterator object_hist_itr;
        for(object_hist_itr  = obj_scale_histograms.begin();
            object_hist_itr != obj_scale_histograms.end();
            object_hist_itr++) {
            if ( object_hist_itr->first.find(object_name) != std::string::npos &&
                 (object_hist_itr->first.size() -  object_name.size()) <= 1 ) {
                /// Create the histogram pdf
                gsl_histogram_pdf*  MyHistPdf = gsl_histogram_pdf_alloc (object_hist_itr->second->n);
                assert( MyHistPdf != NULL );
                /// check if the status is OK
                int status = gsl_histogram_pdf_init (MyHistPdf, object_hist_itr->second);
                assert( status != GSL_EDOM );
                double r = (double)rand()/RAND_MAX;
                return gsl_histogram_pdf_sample(MyHistPdf,r);

                ///Usage: https://www.gnu.org/software/gsl/manual/html_node/The-histogram-probability-distribution-struct.html
                float scale = gsl_histogram_pdf_sample(MyHistPdf,r);
                return scale;
            }
        }
        // just rescaling everything else to about 30 cms
        return -1;
    }

public:
    std::map<std::string, std::vector<std::tuple<float, float, float> > >object_minMax_mapping;
    std::map<std::string, gsl_histogram* > obj_scale_histograms;
    std::map<std::string, std::tuple<float, float> > obj_x_div_y_bounds;
    std::map<std::string, std::tuple<float, float> > obj_z_div_y_bounds;
    float percentile;
};

struct ObjectSampleInfo {
  float probability;
  std::set<std::string> wnids;
};

class ShapeNetModel {
public:
    ShapeNetModel(std::string shapenet_folder,std::string model_info_file,std::string shapenets_sunrgbd_mapping_file, std::string sampling_probability_file, std::mt19937& rand_gen )
  : m_randgen(rand_gen)
    {
        ifstream models_file(model_info_file);
        std::cout<<"Model info:"<<model_info_file<<std::endl;
        char readlinedata[1300];
        double up_x, up_y, up_z;
        double front_x, front_y, front_z;
        int count_dir = 0;
        //This is the title
        models_file.getline(readlinedata,1300);
        while(true) {
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
            std::vector<float> inner_points;
            inner_points.push_back(up_x);
            inner_points.push_back(up_y);
            inner_points.push_back(up_z);
            model_up_vector.push_back(inner_points);
            std::replace(front_vector.begin(),front_vector.end(),',',' ');
            istringstream f_iss(front_vector);
            f_iss >> front_x;
            f_iss >> front_y;
            f_iss >> front_z;
            inner_points.clear();
            inner_points.push_back(front_x);
            inner_points.push_back(front_y);
            inner_points.push_back(front_z);
            model_front_vector.push_back(inner_points);
            count_dir = count_dir+1;
        }
        std::cout<<"count = " << count_dir << std::endl;
        models_file.close();
        ifstream ShapeNets2SUNRGBDMapping(shapenets_sunrgbd_mapping_file);
        std::cout<<"mapping file:"<<shapenets_sunrgbd_mapping_file<<std::endl;
        while(true) {
            ShapeNets2SUNRGBDMapping.getline(readlinedata,1300);
            if (ShapeNets2SUNRGBDMapping.eof())
                break;
            istringstream iss(readlinedata);
            std::string wnid, shapenet_object_name, isFound, sunrgbd_mapping;
            iss >> wnid;
            iss >> shapenet_object_name;
            iss >> isFound;
            if (isFound == "Found") {
                iss >> sunrgbd_mapping;
                wnid_to_sunrgbd_mapping[wnid] = sunrgbd_mapping;
                std::cout<<"wnid_to_sunrgbd_mapping = "<<wnid<<"->" << wnid_to_sunrgbd_mapping[wnid] << std::endl;
            }
        }
        ShapeNets2SUNRGBDMapping.close();

        ifstream model_sampling_file(sampling_probability_file);
        std::cout<<"sampling prob filepath"<<sampling_probability_file<<std::endl;
        std::string scene_name;
        std::string current_line;
        std::string token;
        while(std::getline(model_sampling_file,current_line)) {
          if (current_line == "scene_type") {
            std::getline(model_sampling_file,current_line);
            scene_name = current_line;
            std::getline(model_sampling_file,current_line);
            if (current_line != "class_probabilities") {
              std::cout<<"ERROR Scene must be followed by probabilities";
            exit(1);
            }
          }
          ObjectSampleInfo info;
          istringstream iss(current_line);
          iss >> info.probability;
          while(iss >> token) {
            info.wnids.insert(token);
          }
          scene_to_object_samples[scene_name].push_back(info);
        }
        std::string txt_file = dataset_split + std::string("_split_filtered_model_info_and_texture.txt ");
        base_dir = shapenet_folder;
        std::cout<<"base_dir = " << base_dir << std::endl;
    }

    std::string get_sunrgbd_object_mapping(std::string& wnid) {
        if (wnid_to_sunrgbd_mapping.find(wnid) == wnid_to_sunrgbd_mapping.end())
            return "NOTFOUND";
        else
            return wnid_to_sunrgbd_mapping[wnid];
    }

    int get_random_modelid(std::string scene_type) {
      std::uniform_real_distribution<> sample_dist(0.0,1.0);
      std::uniform_int_distribution<> dist(0,directories.size()-1);
      float chance_of_totally_random = sample_dist(m_randgen);
      if (chance_of_totally_random < 0.05) {
        std::cout<<"Totally random object"<<std::endl;
        int random_choice = dist(m_randgen);
        return random_choice;
      }
      std::vector<ObjectSampleInfo>& scene_info = scene_to_object_samples[scene_type];
      float max_total = 0.0;
      for(auto const& object_info : scene_info) {
        max_total += object_info.probability;
      }
      std::uniform_real_distribution<> object_dist(0.0,max_total);
      float selected_sample = object_dist(m_randgen);
      float cumulative_amount = 0.0;
      std::set<std::string> selected_allowed_wnids;
      for(auto const& object_info : scene_info) {
        cumulative_amount += object_info.probability;
        if (selected_sample <= cumulative_amount) {
          selected_allowed_wnids = object_info.wnids;
          break;
        }
      }
      if (selected_allowed_wnids.empty()) {
        std::cout<<"Empty!!!"<<std::endl;
        return -1;
      }
      //This ensures that we are not equal weighting by the set of wnids, but
      //instead by the number in the shapenets dataset, and simply choosing for
      //the relative proportions of each class.
      int counter = 0;
      int random_choice = dist(m_randgen);
      while(true) {
        if (selected_allowed_wnids.find(wnid_from_model_id(random_choice)) != selected_allowed_wnids.end()) {
          return random_choice;
        }
        random_choice++;
        if (random_choice >= directories.size()) {
          random_choice = 0;
        }
        if (counter > 50000) {
          std::cout<<"Couldnt stuck looking for random object within set"<<*(selected_allowed_wnids.begin())<<std::endl;
          exit(1);
        }
        counter++;
      }
    }

    std::string wnid_from_model_id(int modelid) {
      return wnids_shapenet.at(modelid);
    }

    std::string model_filepath_from_model_id(int modelid) {
        DIR *dir;
        struct dirent *ent;
        std::string a_filename;

        std::string dir_name = base_dir + std::string(directories.at(modelid))+ std::string("/models/");
        if ((dir = opendir(dir_name.c_str())) != NULL) {
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
            std::cout<<"Error: Couldnt open dir:"<<dir_name<<std::endl;
            exit(1);
        }
        return dir_name + a_filename;
    }
public:
    std::vector<std::string>directories;
    std::string directory_string, wnids, tags, up_vector, front_vector;
    std::map<std::string, std::string>wnid_to_sunrgbd_mapping;
    std::map<std::string,std::vector<ObjectSampleInfo> > scene_to_object_samples;
    std::vector<std::string>wnids_shapenet;
    std::vector<std::vector<float> >model_up_vector;
    std::vector<std::vector<float> >model_front_vector;

    std::string base_dir;
    std::mt19937 m_randgen;
};


struct RandomObject {
  float size; //m height
  std::string model;
  std::string wnid;
  std::shared_ptr<ChBody> physics_body;
  ChVector<> pos;
  ChMatrix33<> rot;
};

RandomObject get_random_object(std::mt19937& mt, ObjScaling& myobjectscaling, ShapeNetModel& shapenetmodel, std::string scene_type, bool largeobject) {
  RandomObject random_object;
  //Pick a model
  bool found_appropriate_size = false;
  int counter = 0;
  while (!found_appropriate_size) {
    int model_id = shapenetmodel.get_random_modelid(scene_type);
    std::string wnid = shapenetmodel.wnid_from_model_id(model_id);
    std::string objectName = shapenetmodel.get_sunrgbd_object_mapping(wnid);
    std::string model_file_path = shapenetmodel.model_filepath_from_model_id(model_id);
    random_object.model = model_file_path;
    random_object.wnid = wnid;
    //Pick a size from the random distribution of this class

    std::string thisObjectName = objectName;

    size_t found = thisObjectName.find_first_of(',');

    thisObjectName = objectName.substr(0,found);

    objectName = thisObjectName;

    float scale = myobjectscaling.get_scale_via_probability(objectName);
    if (scale < 0) {
      continue;
    }
    if (largeobject && scale <= kSMALL_OBJECT_SIZE) {
      continue;
    }
    if (!largeobject && scale >= kSMALL_OBJECT_SIZE) {
      continue;
    }
    found_appropriate_size = true;
    if (largeobject) {
      std::cout<<"Adding large object";
    } else {
      std::cout<<"Adding small object";
    }
    std::cout<<": "<<objectName<<" Y scale:"<<scale;
    random_object.size = scale;//dist(mt);
    counter++;
    if (counter > 100) {
      std::cout<<"Got stuck finding appropriate size for object:"<<model_file_path<<std::endl;
      exit(1);
      break;
    }
  }
  return random_object;
}


int number_of_objects(std::shared_ptr<ChBody> scene, float obj_per_square_metre) {
    ChVector<> bbmin, bbmax;
    scene->GetTotalAABB(bbmin,bbmax);
    ChVector<> size = (bbmax - bbmin);
    double area_m2 = size.x * size.z;
    //truncate to get num objects (i.e. round down)
    return static_cast<int>(area_m2 * obj_per_square_metre);
}

std::shared_ptr<ChBody> add_object(ChSystem& system, RandomObject& object, ChVector<> pos, double mass = 10.0, 
                                   ChMatrix33<> rot = ChMatrix33<>(1), bool cog_below = true, bool fixed = false, bool triangle_collision = false) {
    double height = object.size;
    std::string model_name = object.model;
    ChVector<> bbmin, bbmax;
    ChTriangleMeshConnected mmesh;

    mmesh.LoadWavefrontMesh(model_name,false,false);
    ChVector<> center;

    double scale_factor;
    {
      std::vector<ChVector<double> > points = mmesh.getCoordsVertices();
      double xmin,ymin,zmin;
      double xmax,ymax,zmax;

      xmin = ymin = zmin = 10e12;
      xmax = ymax = zmax = -10e12;

      for(auto point : points) {
          if (xmin > point.x) xmin = point.x;
          if (ymin > point.y) ymin = point.y;
          if (zmin > point.z) zmin = point.z;
          if (xmax < point.x) xmax = point.x;
          if (ymax < point.y) ymax = point.y;
          if (zmax < point.z) zmax = point.z;
      }
      ChVector<> bbmin(xmin,ymin,zmin);
      ChVector<> bbmax(xmax,ymax,zmax);
      center = (bbmax + bbmin) * 0.5;
      const double desired_y_size = height;
      double y_size = bbmax.y - bbmin.y;
      if (cog_below) {
        center.y -= (y_size * 0.6);
      }
      scale_factor = (desired_y_size / y_size);
      double z_size = scale_factor * (bbmax.z - bbmin.z);
      double x_size = scale_factor * (bbmax.x - bbmin.x);
      std::cout<<" Z size:"<<z_size<<" X size:"<<x_size<<std::endl;
      if (y_size * z_size * x_size > 5.0) {
        std::cout<<" Too Big! Skipping"<<std::endl;
        return std::shared_ptr<ChBody>();
      }
    }

    mmesh.Transform(-center*scale_factor , ChMatrix33<>(scale_factor)); // scale to a smaller cube
    mmesh.RepairDuplicateVertexes(1e-6); // if meshes are not watertight

    auto mfalling = std::make_shared<ChBody>();

    mfalling->SetPos(pos);
    mfalling->SetRot(rot);
    mfalling->SetMass(mass);
    mfalling->SetBodyFixed(fixed);
    mfalling->SetInertia(mass);
    mfalling->GetCollisionModel()->ClearModel();
    if (triangle_collision) {
      mfalling->GetCollisionModel()->AddTriangleMesh(mmesh,false, false, VNULL, ChMatrix33<>(1), 0.005);
    } else {
      mfalling->GetCollisionModel()->AddConvexHull(mmesh.getCoordsVertices(),ChVector<>(0),ChMatrix33<>(1));
    }
    mfalling->GetCollisionModel()->BuildModel();
    mfalling->SetCollide(true);
    mfalling->GetTotalAABB(bbmin,bbmax);
    system.Add(mfalling);

    auto masset_mesh = std::make_shared<ChTriangleMeshShape>();
    masset_mesh->SetMesh(mmesh);
    masset_mesh->SetBackfaceCull(true);
    mfalling->AddAsset(masset_mesh);
    return mfalling;
}

class CheckCollisions : public ChReportContactCallback {
public:
  virtual bool ReportContactCallback(const ChVector<>& pA, 
                                     const ChVector<>& pB, 
                                     const ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const ChVector<>& react_forces,
                                     const ChVector<>& react_torques,
                                     ChContactable* modA,
                                     ChContactable* modB) override {
    ChMatrix33<>& mplanecoord = const_cast<ChMatrix33<>&>(plane_coord);
    ChVector<> v1 = pA; 
    ChVector<> v2; 
    ChVector<> vn = mplanecoord.Get_A_Xaxis();
    if (distance < -0.03) {
      contacts[modA] = true;
      contacts[modB] = true;
    }
    return true;
  }   
  std::map<ChContactable*,bool> contacts;
};

int main(int argc, char* argv[])
{ 
    if (argc < 2) {
        std::cout<<"Too few arguments"<<std::endl;
        std::cout<<"./scenenet_room_generator /path/to/ShapeNet/ /path/to/SceneNetLayouts/"<<std::endl;
        std::cout<<"Note that the folders should be followed by trailing / in the command"<<std::endl;
        exit(1);
    }
    std::string shapenets_dir = std::string(argv[1]);
    std::cout<<"ShapeNet directory:"<<shapenets_dir<<std::endl;
    std::string layouts_dir = std::string(argv[2]);
    std::cout<<"Layouts directory:"<<layouts_dir<<std::endl;

    //Setup random number generator
    std::random_device rd;
    std::mt19937 mt(rd());

    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem(false,1000,20);
    // No bouncing
    mphysicalSystem.SetMinBounceSpeed(100.0);

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    /*
    ChIrrApp application(&mphysicalSystem, L"SceneNet Room Generator", core::dimension2d<u32>(640, 480), false, true);
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 1, -1));
    application.AddLightWithShadow(core::vector3df(0.0f, 3.5f, 0.0f), core::vector3df(0, 0, -1.0f), 3, 2.2, 7.2, 40, 512,
                                   video::SColorf(1.5f, 1.5f, 1.5f),true);
    application.AddLightWithShadow(core::vector3df(3.0f, 3.5f, 0.0f), core::vector3df(0, 0, -1.0f), 3, 2.2, 7.2, 40, 512,
                                   video::SColorf(0.5f, 0.5f, 0.5f),true);
    application.AddLightWithShadow(core::vector3df(3.0f, 3.5f, 3.0f), core::vector3df(0, 0, -1.0f), 3, 2.2, 7.2, 40, 512,
                                   video::SColorf(0.5f, 0.5f, 0.5f),true);
    application.AddLightWithShadow(core::vector3df(0.0f, 3.5f, 3.0f), core::vector3df(0, 0, -1.0f), 3, 2.2, 7.2, 40, 512,
                                   video::SColorf(0.5f, 0.5f, 0.5f),true);
    application.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_DISTANCES);
    */
    // End visualisation code

    // Create all the rigid bodies.
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // - Create a floor
    std::cout<<"Creating floor"<<std::endl;
    auto scene = std::make_shared<ChBody>();
    scene->SetPos(ChVector<>(0, 0, 0));
    scene->SetBodyFixed(true);
    mphysicalSystem.Add(scene);

    ChTriangleMeshConnected mmeshbox;

    // Load random scenes file
    std::vector<std::string> layout_paths;
    std::ifstream layouts_file("../textfiles/"+dataset_split+"_layouts.txt");
    std::copy(std::istream_iterator<std::string>(layouts_file), std::istream_iterator<std::string>(), std::back_inserter(layout_paths));
    std::cout<<"About to load layouts paths"<<std::endl;
    for(auto str : layout_paths) {
        std::cout<<str<<std::endl;
    }
    layouts_file.close();
    std::cout<<"Loaded random scene file"<<std::endl;

    // Pick random layout
    std::uniform_int_distribution<> layout_dist(0,layout_paths.size()-1);
    std::cout<<"Picking random"<<layout_paths.size()-1<<std::endl;
    int layout_int = layout_dist(mt);
    std::cout<<"Selecting from:"<<layout_int<<std::endl;
    std::string layout_filepath = layouts_dir+layout_paths[layout_int];
    std::cout<<"Loading layout:"<<layout_filepath<<std::endl;
    std::string scene_type = layout_paths[layout_int].substr(2);
    int split_pos = scene_type.find('/');
    scene_type = scene_type.substr(0,split_pos);
    mmeshbox.LoadWavefrontMesh(layout_filepath,false,false);

    mmeshbox.Transform(ChVector<>(0), ChMatrix33<>(1)); // scale to a smaller cube

    scene->GetCollisionModel()->ClearModel();
    scene->GetCollisionModel()->AddTriangleMesh(mmeshbox,false, false, VNULL, ChMatrix33<>(1), 0.005);
    scene->GetCollisionModel()->BuildModel();
    scene->SetCollide(true);

    auto masset_meshbox = std::make_shared<ChTriangleMeshShape>();
    masset_meshbox->SetMesh(mmeshbox);
    scene->AddAsset(masset_meshbox);

    auto masset_texture = std::make_shared<ChTexture>();
    masset_texture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    scene->AddAsset(masset_texture);
    ChVector<> bbmin, bbmax;
    scene->GetTotalAABB(bbmin,bbmax);
    std::cout<<"min of scene"<<bbmin.x<<" "<<bbmin.y<<" "<<bbmin.z<<" "<<std::endl;
    std::cout<<"max of scene"<<bbmax.x<<" "<<bbmax.y<<" "<<bbmax.z<<" "<<std::endl;

    std::vector<RandomObject>objects;
    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);

    ObjScaling myobjscaling("../textfiles/objects_in_scene.txt");

    ShapeNetModel myshapenetmodel(shapenets_dir,
                                  "../textfiles/"+dataset_split+"_split_filtered_model_info_and_texture.txt",
                                  "../textfiles/ShapeNetsSUNRGBDMapping.txt",
                                  "../textfiles/wnid_sample_probabilities.txt",mt);

    std::uniform_real_distribution<double> large_dist(0.1, 0.5);
    const int num_large_objects = std::max(3,std::min(30,number_of_objects(scene,large_dist(mt))));

    std::cout<<"layout type:"<<scene_type<<std::endl;
    for (int object_id = 0; object_id < num_large_objects; object_id++) {
      //Choose uniform random position
      RandomObject this_object = get_random_object(mt,myobjscaling,myshapenetmodel,scene_type,true);

      double x = bbmin.x + uniform_dist(mt) * (bbmax.x - bbmin.x);
      double y = bbmin.y + uniform_dist(mt) * (bbmax.y - bbmin.y) * 0.5;
      double z = bbmin.z + uniform_dist(mt) * (bbmax.z - bbmin.z);
      std::cout<<"object position:"<<x<<" "<<y<<" "<<z<<" "<<std::endl;

      auto object_physics_body = add_object(mphysicalSystem,this_object,ChVector<>(x,y,z));
      if (object_physics_body) {
        this_object.physics_body = object_physics_body;
        objects.push_back(this_object);
      } else {
        std::cout<<"Not pushing back object"<<std::endl;
      }
    }

    std::uniform_real_distribution<double> small_dist(0.5, 3.0);
    const int num_small_objects = std::max(10,std::min(70,number_of_objects(scene,small_dist(mt))));
    for (int object_id = 0; object_id < num_small_objects; object_id++) {
      //Choose uniform random position
      RandomObject this_object = get_random_object(mt,myobjscaling,myshapenetmodel,scene_type,false);
      double x = bbmin.x + uniform_dist(mt) * (bbmax.x - bbmin.x);
      double y = bbmin.y + uniform_dist(mt) * (bbmax.y - bbmin.y);
      double z = bbmin.z + uniform_dist(mt) * (bbmax.z - bbmin.z);
      auto object_physics_body = add_object(mphysicalSystem,this_object,ChVector<>(x,y,z),0.1);
      this_object.physics_body = object_physics_body;
      objects.push_back(this_object);
    }

    std::cout<<"Simulating system"<<std::endl;
    mphysicalSystem.Setup();

    const float total_time = 60.0;
    /*
    */
    // This is for non-rendered physical simulation
    mphysicalSystem.SetEndTime(total_time);
    GetLog() << "Entire in one";
    mphysicalSystem.DoEntireDynamics();

    /* 
    // This is for rendered physical simulation
    const float timestep = 0.05;
    application.AssetBindAll();
    application.AssetUpdateAll();
    application.AddShadowAll();
    application.SetTimestep(timestep);
    application.SetPaused(false);

    application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));
    GetLog() << "Done!";
    float time = 0.0;
    while (application.GetDevice()->run()) {
      std::cout<<"step"<<std::endl;
        application.DoStep();
        application.DrawAll();
        application.GetVideoDriver()->endScene();
        time += timestep;
        if (time > total_time) 
          break;
    }
    while (application.GetDevice()->run()) {
        application.DrawAll();
        application.GetVideoDriver()->endScene();
    }
    */

    CheckCollisions collision_checker;
    mphysicalSystem.GetContactContainer()->ReportAllContacts(&collision_checker);

    std::string filename_path = "./scene_description.txt";
    std::ofstream ofile(filename_path);
    ofile << "layout_file:"<<layout_paths[layout_int]<<std::endl;
    for(auto object : objects) {
      RandomObject& this_object = object;
      std::shared_ptr<ChBody>body = this_object.physics_body;
      ChContactable* a_contact = body->GetCollisionModel()->GetContactable();
      if (collision_checker.contacts[a_contact]) {
        std::cout<<"Body in bad contact"<<this_object.model<<std::endl;
        continue;
      }
      ChMatrix33<> myRot = body->Amatrix;
      ChVector<>   mypos = body->GetPos();
      if (mypos.y < -7) {
        std::cout<<"Body below threshold"<<this_object.model<<std::endl;
        continue;
      }
      ChVector<> speed = body->GetPos_dt();
      if (speed.Length() > 0.05) {
        std::cout<<"Body still moving"<<this_object.model<<std::endl;
        std::cout<<"Speed"<<speed.Length()<<std::endl;
        continue;
      }
      ChVector<> rot = body->GetWvel_loc();
      if (rot.Length() > 0.25) {
        std::cout<<"Body still spinning"<<this_object.model<<std::endl;
        std::cout<<"Rot"<<rot.Length()<<std::endl;
        continue;
      }
      ofile << this_object.model <<std::endl;
      ofile << this_object.wnid <<std::endl;
      ofile << this_object.size <<std::endl;
      ofile << myRot(0)<<" "<<myRot(1)<<" "<<myRot(2) <<" "<<mypos.x<<std::endl;
      ofile << myRot(3)<<" "<<myRot(4)<<" "<<myRot(5) <<" "<<mypos.y<<std::endl;
      ofile << myRot(6)<<" "<<myRot(7)<<" "<<myRot(8) <<" "<<mypos.z<<std::endl;
      std::cout<<std::endl;
      std::cout<<"x:"<<mypos.x<<" y:"<<mypos.y<<" z:"<<mypos.z<<std::endl;
    }
    return 0;
}

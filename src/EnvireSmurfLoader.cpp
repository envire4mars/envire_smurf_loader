/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the MARS simulation framework.
 *
 *  MARS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation, either version 3
 *  of the License, or (at your option) any later version.
 *
 *  MARS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with MARS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * \file EnvireSmurfLoader.cpp
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Tests
 *
 * Version 0.1
 */

#include "EnvireSmurfLoader.hpp"

#include <mars/interfaces/Logging.hpp>

#include <lib_manager/LibManager.hpp>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/LoadCenter.h>
#include <mars/sim/PhysicsMapper.h>
#include <mars/sim/SimNode.h>
#include <mars/utils/misc.h>

#include <lib_config/YAMLConfiguration.hpp>
// To populate the Graph from the smurf
#include <envire_smurf/GraphLoader.hpp>

// For the floor
//#include <mars/sim/ConfigMapItem.h>

// To log the graph
#include <base/Time.hpp>
#include <envire_core/graph/EnvireGraph.hpp>

#include <mars/plugins/envire_managers/EnvireStorageManager.hpp>

#include "SimNodeCreator.h"
#include "SimJointCreator.h"
#include "SimMotorCreator.h"
#include "SimSensorCreator.h"

using vertex_descriptor = envire::core::GraphTraits::vertex_descriptor;

namespace mars {
    namespace plugins {
        namespace EnvireSmurfLoader {
            
            using namespace mars::plugins::envire_managers;
            
            EnvireSmurfLoader::EnvireSmurfLoader(lib_manager::LibManager *theManager)
                : LoadSceneInterface(theManager), control(NULL), nextGroupId(1)
            {
                mars::interfaces::SimulatorInterface *marsSim;
                marsSim = libManager->getLibraryAs<mars::interfaces::SimulatorInterface>("mars_sim");
                if(marsSim) {
                    control = marsSim->getControlCenter();
                    //control->loadCenter->loadScene[".zsmurf"] = this; // zipped smurf model
                    //control->loadCenter->loadScene[".zsmurfs"] = this; // zipped smurf scene
                    control->loadCenter->loadScene[".smurf"] = this; // smurf model
                    //control->loadCenter->loadScene[".smurfs"] = this; // smurf scene
                    //control->loadCenter->loadScene[".svg"] = this; // smurfed vector graphic
                    control->loadCenter->loadScene[".urdf"] = this; // urdf model
                    LOG_INFO("envire_smurf_loader: SMURF loader to loadCenter");                    
                }

                center = SIM_CENTER_FRAME_NAME;
            }

            EnvireSmurfLoader::~EnvireSmurfLoader() {
              if(control) {
                control->loadCenter->loadScene.erase(".zsmurf");
                control->loadCenter->loadScene.erase(".zsmurfs");
                control->loadCenter->loadScene.erase(".smurf");
                control->loadCenter->loadScene.erase(".smurfs");
                control->loadCenter->loadScene.erase(".svg");
                control->loadCenter->loadScene.erase(".urdf");
                libManager->releaseLibrary("mars_sim");
              }                
            }       

            bool EnvireSmurfLoader::loadFile(std::string filename, std::string tmpPath,
                                    std::string robotname)
            {
                LOG_DEBUG("[EnvireSmurfLoader::loadFile] load smurf to zero position");
                std::cout << "EnvireSmurfLoader::loadFile: " << filename << " " << tmpPath << std::endl;

                this->smurf_filename = filename;

                vertex_descriptor center = EnvireStorageManager::instance()->getGraph()->getVertex(SIM_CENTER_FRAME_NAME);
                envire::core::Transform iniPose;
                iniPose.transform.orientation = base::Quaterniond::Identity();
                iniPose.transform.translation << 0.0, 0.0, 0.3;
                addRobot(filename, center, iniPose);
                createSimObjects();
                return true;
            }

            bool EnvireSmurfLoader::loadFile(std::string filename, std::string tmpPath,
                                std::string robotname, mars::utils::Vector pos, mars::utils::Vector rot)
            {
                LOG_DEBUG("[EnvireSmurfLoader::loadFile] Smurf loader given position");

                this->smurf_filename = filename;

                std::string suffix = utils::getFilenameSuffix(filename);
                vertex_descriptor center = EnvireStorageManager::instance()->getGraph()->getVertex(SIM_CENTER_FRAME_NAME);
                envire::core::Transform iniPose;
                // FIXME TODO use rot input. Is it Euler angles or scaled axis?
                iniPose.transform.orientation = base::Quaterniond::Identity();
                iniPose.transform.translation = pos;
                addRobot(filename, center, iniPose);
                createSimObjects();
                return true;
            }    

            int EnvireSmurfLoader::saveFile(std::string filename, std::string tmpPath)
            {
                return 0;
            }


            void EnvireSmurfLoader::addRobot(std::string filename, vertex_descriptor center, envire::core::Transform iniPose)
            {
                std::string path = libConfig::YAMLConfigParser::applyStringVariableInsertions(filename); 
                LOG_DEBUG("Robot Path: %s",  path.c_str() );
                smurf::Robot* robot = new( smurf::Robot);
                robot->loadFromSmurf(path);
                envire::smurf::GraphLoader graphLoader(EnvireStorageManager::instance()->getGraph());
                graphLoader.loadRobot(nextGroupId, center, iniPose, *robot);
            }
            
            vertex_descriptor EnvireSmurfLoader::addCenter()
            {
                center = SIM_CENTER_FRAME_NAME;
                if (! EnvireStorageManager::instance()->getGraph()->containsFrame(center))
                    EnvireStorageManager::instance()->getGraph()->addFrame(center);
                return EnvireStorageManager::instance()->getGraph()->getVertex(center);
            }

            /*void EnvireSmurfLoader::addFloor(const vertex_descriptor &center)
            {
                mars::interfaces::NodeData data;
                data.init("floorData", mars::utils::Vector(0,0,0));
                data.initPrimitive(interfaces::NODE_TYPE_BOX, mars::utils::Vector(25, 25, 0.1), 0.0001);
                data.movable = false;
                mars::sim::PhysicsConfigMapItem::Ptr item(new mars::sim::PhysicsConfigMapItem);
                data.material.transparency = 0.5;
                //data.material.ambientFront = mars::utils::Color(0.0, 1.0, 0.0, 1.0);
                // TODO Fix the material data is lost in the conversion from/to configmap
                data.material.emissionFront = mars::utils::Color(1.0, 1.0, 1.0, 1.0);
                LOG_DEBUG("Color of the Item in the addFloor: %f , %f, %f, %f", data.material.emissionFront.a , data.material.emissionFront.b, data.material.emissionFront.g, data.material.emissionFront.r );
                data.toConfigMap(&(item.get()->getData()));
                EnvireStorageManager::instance()->getGraph()->addItemToFrame(EnvireStorageManager::instance()->getGraph()->getFrameId(center), item);
            }  */         

            void EnvireSmurfLoader::createSimObjects()
            {
                //materials
                //nodes
                loadNodes();
                loadJoints();
                loadMotors();
                //TODO: control->motors->connectMimics();
                loadSensors();
                //controller
                //light
                //graphic
            }             

            void EnvireSmurfLoader::loadNodes()
            {
#ifdef DEBUG
                LOG_DEBUG("[EnvireSmurfLoader::loadNodes] ------------------- Parse the graph and create SimNodes -------------------");
#endif                

                std::string filename_path = mars::utils::getPathOfFile(this->smurf_filename);
                std::cout << "----------filename_path: " << filename_path << std::endl;

                SimNodeCreatorFrame         sn_frame(control, center);
                sn_frame.setFilenamePath(filename_path);
                SimNodeCreatorCollidable    sn_collidable(control, center);
                sn_collidable.setFilenamePath(filename_path);
                SimNodeCreatorInertial      sn_inertial(control, center);
                sn_inertial.setFilenamePath(filename_path);
                SimNodeCreatorVisual        sn_visual(control, center);
                sn_visual.setFilenamePath(filename_path);

                // search the graph
                envire::core::EnvireGraph::vertex_iterator v_itr, v_end;
                boost::tie(v_itr, v_end) = EnvireStorageManager::instance()->getGraph()->getVertices();
                for(; v_itr != v_end; v_itr++)
                {
#ifdef DEBUG                    
                    envire::core::FrameId frame_id = EnvireStorageManager::instance()->getGraph()->getFrameId(*v_itr);
                    LOG_DEBUG(("[EnvireSmurfLoader::loadNodes] --- IN ***" + frame_id + "*** ---" ).c_str());

#endif 
                    // search for smurf::Frame, Collidable and Inertial
                    sn_frame.create(v_itr);
                    sn_collidable.create(v_itr);
                    sn_inertial.create(v_itr);
                    sn_visual.create(v_itr);
                }
            }                   

            void EnvireSmurfLoader::loadJoints()
            {
#ifdef DEBUG
                LOG_DEBUG("[EnvireSmurfLoader::loadJoints] ------------------- Parse the graph and create SimJoints -------------------");
#endif                

                SimJointCreatorJoint            sj_joint(control, center);
                SimJointCreatorStaticTranf      sj_static_tranf(control, center);

                // search the graph
                envire::core::EnvireGraph::vertex_iterator v_itr, v_end;
                boost::tie(v_itr, v_end) = EnvireStorageManager::instance()->getGraph()->getVertices();
                for(; v_itr != v_end; v_itr++)
                {              
#ifdef DEBUG          
                    envire::core::FrameId frame_id = EnvireStorageManager::instance()->getGraph()->getFrameId(*v_itr);                
                    LOG_DEBUG(("[EnvireSmurfLoader::loadJoints] --- IN ***" + frame_id + "*** ---" ).c_str());
#endif 
                    //
                    sj_joint.create(v_itr);
                    sj_static_tranf.create(v_itr);
                }
            }       

            void EnvireSmurfLoader::loadMotors()
            {
#ifdef DEBUG
                LOG_DEBUG("[EnvireSmurfLoader::loadMotors] ------------------- Parse the graph and create SimMotors -------------------");
#endif                

                SimMotorCreator sm(control, center);

                // search the graph
                envire::core::EnvireGraph::vertex_iterator v_itr, v_end;
                boost::tie(v_itr, v_end) = EnvireStorageManager::instance()->getGraph()->getVertices();
                for(; v_itr != v_end; v_itr++)
                {              
#ifdef DEBUG          
                    envire::core::FrameId frame_id = EnvireStorageManager::instance()->getGraph()->getFrameId(*v_itr);                
                    LOG_DEBUG(("[EnvireSmurfLoader::loadMotors] --- IN ***" + frame_id + "*** ---" ).c_str());
#endif 
                    //
                    sm.create(v_itr);
                }                
            }

            void EnvireSmurfLoader::loadSensors()
            {
#ifdef DEBUG
                LOG_DEBUG("[EnvireSmurfLoader::loadSensors] ------------------- Parse the graph and create SimSensors -------------------");
#endif                

                SimSensorCreator sm(control, center);

                // search the graph
                envire::core::EnvireGraph::vertex_iterator v_itr, v_end;
                boost::tie(v_itr, v_end) = EnvireStorageManager::instance()->getGraph()->getVertices();
                for(; v_itr != v_end; v_itr++)
                {              
#ifdef DEBUG          
                    envire::core::FrameId frame_id = EnvireStorageManager::instance()->getGraph()->getFrameId(*v_itr);                
                    LOG_DEBUG(("[EnvireSmurfLoader::loadSensors] --- IN ***" + frame_id + "*** ---" ).c_str());
#endif 
                    //
                    sm.create(v_itr);
                }                
            }            

        } // end of namespace EnvireSmurfLoader
    } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::EnvireSmurfLoader::EnvireSmurfLoader);
CREATE_LIB(mars::plugins::EnvireSmurfLoader::EnvireSmurfLoader);

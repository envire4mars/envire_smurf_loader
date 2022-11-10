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

#include <mars/entity_generation/entity_factory/EntityFactoryManager.h>

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
                : mars::interfaces::MarsPluginTemplate(theManager, "EnvireSmurfLoader"),
                  mars::entity_generation::EntityFactoryInterface("smurf, urdf"), 
                  nextGroupId(1)
            {
                LOG_INFO("envire_smurf_loader: SMURF loader to loadCenter");                    

                mars::entity_generation::EntityFactoryManager* factoryManager =
                    theManager->acquireLibraryAs<mars::entity_generation::EntityFactoryManager>(
                        "mars_entity_factory");
                factoryManager->registerFactory("smurf", this);
                factoryManager->registerFactory("urdf", this);
                theManager->releaseLibrary("mars_entity_factory");
            }

            std::shared_ptr<mars::sim::SimEntity> EnvireSmurfLoader::createEntity(const configmaps::ConfigMap& config) { 
                configmaps::ConfigMap entityconfig = config;
                std::string path = (std::string)entityconfig["path"];
                std::string file = (std::string)entityconfig["file"];
                
                this->smurf_filename = path + file;            

                std::cout << "Loading Entity from file: " << this->smurf_filename << " ..." << std::endl;

                // load robot model from smurf
                smurf::Robot* robot = new(smurf::Robot);
                robot->loadFromSmurf(libConfig::YAMLConfigParser::applyStringVariableInsertions(this->smurf_filename));    

                // recreate robot model in the graph
                vertex_descriptor center = EnvireStorageManager::instance()->getGraph()->getVertex(SIM_CENTER_FRAME_NAME);
                envire::core::Transform iniPose;
                iniPose.transform.orientation = base::Quaterniond::Identity();
                iniPose.transform.translation << 0.0, 0.0, 0.3;

                envire::smurf::GraphLoader graphLoader(EnvireStorageManager::instance()->getGraph());
                graphLoader.loadRobot(nextGroupId, center, iniPose, *robot);

                // create all corresponding sim objects for robot model
                createSimObjects();

                // create SimEntity for robot model
                entityconfig["abs_path"] = mars::utils::pathJoin(mars::utils::getCurrentWorkingDir(), path);
                entityconfig["name"] = robot->getModelName();
                entityconfig["frame_id"] = robot->getRootFrame()->getName(); 

                return std::make_shared<mars::sim::SimEntity>(control, entityconfig);
            }               

            void EnvireSmurfLoader::addRobot(std::string filename, vertex_descriptor center, envire::core::Transform iniPose)
            {

            }  

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

                SimNodeCreatorFrame         sn_frame(control, SIM_CENTER_FRAME_NAME);
                sn_frame.setFilenamePath(filename_path);
                SimNodeCreatorCollidable    sn_collidable(control, SIM_CENTER_FRAME_NAME);
                sn_collidable.setFilenamePath(filename_path);
                SimNodeCreatorInertial      sn_inertial(control, SIM_CENTER_FRAME_NAME);
                sn_inertial.setFilenamePath(filename_path);
                SimNodeCreatorVisual        sn_visual(control, SIM_CENTER_FRAME_NAME);
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

                SimJointCreatorJoint            sj_joint(control, SIM_CENTER_FRAME_NAME);
                SimJointCreatorStaticTranf      sj_static_tranf(control, SIM_CENTER_FRAME_NAME);

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

                SimMotorCreator sm(control, SIM_CENTER_FRAME_NAME);

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

                SimSensorCreator sm(control, SIM_CENTER_FRAME_NAME);

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

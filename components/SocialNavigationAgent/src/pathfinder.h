/*
 * Copyright 2017 pbustos <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <AGMCommonBehavior.h>
#include <CommonBehavior.h>
#include <thread>
#include <mutex>
#include <memory>
#include <functional>
#include "currenttarget.h"
#include "road.h"
#include "sampler.h"
#include "pathplanner.h"
#include "drawroad.h"
#include "projector.h"
#include "controller.h"
#include "navigationstate.h"
//#include <innermodel/innermodelmgr.h>
#include <qlog/qlog.h>

#define USE_QTGUI

#ifdef USE_QTGUI
	#include "innerviewer.h"
#endif


using namespace std;

namespace robocomp
{
	namespace pathfinder
	{
		typedef map<string, string> ParameterMap;		
		
		class PathFinder
		{
			public:
				using InnerPtr = std::shared_ptr<InnerModel>;
				PathFinder() = default;
				void initialize(const InnerPtr &innerModel_,
								const shared_ptr< RoboCompCommonBehavior::ParameterList > &configparams_,
								LaserPrx laser_prx,
								OmniRobotPrx omnirobot_proxy);
				void releaseRoad();
				Road& getRoad();
				RoboCompTrajectoryRobot2D::NavState getState(){ return state->toIce(); };
				bool structuralchange = false;
				
			/////////////////////////////
			/// Interface
			////////////////////////////
			void go(float x, float z, const ParameterMap &parameters = ParameterMap());
			//void setInnerModel(InnerModel* innerModel_){ innerModel = innerModel_; };
			void innerModelChanged(InnerPtr &innerModel_, bool structural = false, vector <bool> pn = {false,false,false,false,false,false} ); //Le estoy metiendo el vector de personas para actualizar su posición
			
			#ifdef USE_QTGUI
				InnerViewer *viewer = nullptr;
			#endif
			
			void run();
			///////////////////////////
			
			private:
				std::shared_ptr<NavigationState> state;
				mutable std::mutex mymutex;
				Road road;
				PathPlanner pathplanner;
				std::shared_ptr<CurrentTarget> currenttarget;
				std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;
				
				std::string robotname = "robot";  ///OJO
				
				DrawRoad drawroad;
				Projector projector;
				Controller controller;
			
				InnerPtr innerModel;
				
				std::thread thread_planner;
				std::thread thread_projector;
				std::thread thread_controller;
				
		};	
	} //path
} //robocomp

#endif // PATHFINDER_H
